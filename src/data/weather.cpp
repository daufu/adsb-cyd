#include "weather.h"
#include "http_mutex.h"
#include "../config.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PNGdec.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// ---------- 狀態 ----------
static bool     _enabled = true;
static bool     _ready = false;
static uint32_t _last_update = 0;
static uint32_t _last_try = 0;
////static int      _last_range_idx = -1; //range 變要重抓或至少重投影；v1 簡化：重抓
static float    _stored_for_range = 0; //這張 intensity 對應的 range
static float    _want_range = 50.0f;

// 8-bit 強度圖（中心 = home）。static 不吃 heap fragment
static uint8_t  wx_map[WX_N * WX_N]; //完整draw用
static uint8_t  wx_map_tmp[WX_N * WX_N]; //decode時暫存，成功再memcpy

/* 
png_buf是下載buffer (用完就free??): static(免fragment), 固定占48KB RAM.
free heap長期<80KB->改 malloc(48*1024) + 原本largest-block檢查, 用完立刻free.
*/
static uint8_t  png_static_buf[48 * 1024];
static uint8_t *png_buf = nullptr;
static int      png_len = 0;
static int      png_pos = 0;
static bool     png_use_static = true;

static PNG png;
static int png_w = 0, png_h = 0;

// Buffer mutex: 只保護"swap瞬間"與"draw讀取"
static SemaphoreHandle_t _wx_buf_mtx = nullptr;
static inline bool wx_buf_take(TickType_t t) { //似http_mutex_acquire
    return _wx_buf_mtx && xSemaphoreTake(_wx_buf_mtx, t) == pdTRUE;
}
static inline void wx_buf_give() { //似http_mutex_release
    if (_wx_buf_mtx) xSemaphoreGive(_wx_buf_mtx);
}

// Weather JSON小，static省heap
static char wx_json_buf[4096];
#if ARDUINOJSON_VERSION_MAJOR >= 7
// V7：可設 capacity；若支援 filter 更好。小 JSON 用固定 document
static JsonDocument wx_doc;  // 或用 StaticJsonDocument 若你鎖 V6 習慣
#else
static StaticJsonDocument<3072> wx_doc;
#endif


// ---------- 顏色：你提供的綠雷達風 + 夜間版 ----------
static uint16_t wx_color_green(uint8_t v) {
    if (v < 40)  return ((0 & 0xF8) << 8) | ((40 & 0xFC) << 3) | (80 >> 3);   // 深藍
    if (v < 80)  return ((0 & 0xF8) << 8) | ((120 & 0xFC) << 3) | (200 >> 3);  // 藍
    if (v < 120) return ((0 & 0xF8) << 8) | ((200 & 0xFC) << 3) | (160 >> 3);  // 青
    if (v < 170) return ((200 & 0xF8) << 8) | ((200 & 0xFC) << 3) | (0 >> 3);  // 黃
    if (v < 210) return ((255 & 0xF8) << 8) | ((120 & 0xFC) << 3) | (0 >> 3);  // 橙
    return ((255 & 0xF8) << 8) | ((40 & 0xFC) << 3) | (40 >> 3);               // 紅
}

// 夜間：偏琥珀/紅，避免太亮青藍刺眼
static uint16_t wx_color_night(uint8_t v) {
    if (v < 40)  return ((20 & 0xF8) << 8) | ((10 & 0xFC) << 3) | (40 >> 3);
    if (v < 80)  return ((40 & 0xF8) << 8) | ((20 & 0xFC) << 3) | (80 >> 3);
    if (v < 120) return ((120 & 0xF8) << 8) | ((50 & 0xFC) << 3) | (20 >> 3);
    if (v < 170) return ((180 & 0xF8) << 8) | ((90 & 0xFC) << 3) | (0 >> 3);
    if (v < 210) return ((220 & 0xF8) << 8) | ((70 & 0xFC) << 3) | (0 >> 3);
    return ((255 & 0xF8) << 8) | ((40 & 0xFC) << 3) | (20 >> 3);
}

static inline uint16_t wx_color(uint8_t v, bool night) {
    return night ? wx_color_night(v) : wx_color_green(v);
}

// ---------- range → zoom ----------
static int zoom_for_range(float range_nm) {
    if (range_nm >= 120.0f) return 6;
    if (range_nm >= 80.0f)  return 7;
    if (range_nm >= 35.0f)  return 8;
    if (range_nm >= 12.0f)  return 9;
    return 10; // free 上限
}

// ---------- PNGdec 從 memory 讀 ----------
static void *pngOpen(const char *filename, int32_t *size) {
    (void)filename;
    *size = png_len;
    png_pos = 0;
    return (void *)1;
}
static void pngClose(void *handle) { (void)handle; }
static int32_t pngRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
    (void)handle;
    if (png_pos >= png_len) return 0;
    int32_t n = length;
    if (png_pos + n > png_len) n = png_len - png_pos;
    memcpy(buffer, png_buf + png_pos, n);
    png_pos += n;
    return n;
}
static int32_t pngSeek(PNGFILE *handle, int32_t position) {
    (void)handle;
    if (position < 0) position = 0;
    if (position > png_len) position = png_len;
    png_pos = position;
    return png_pos;
}

/*
pngDrawCB(): PNGdec每解碼出PNG其中一行像素(256個點)，會自動跑一次這func <-即是fetch_and_decode()'s png.decode()


*/

// 解碼時：把 PNG 一行 → intensity，並 downsample 進 wx_map_tmp
// 假設 PNG 是 256x256，中心=home，整張對應"約 2*range 的方框"
// 把方框映射到 WX_N×WX_N（覆蓋 ±range 的正方形，之後畫時再裁圓）


static int pngDrawCB(PNGDRAW *pDraw) {
    // pDraw->y, pDraw->iWidth, pDraw->pPixels
    // RainViewer多為RGBA/RGB; 用getLineAsRGB565最省事. 但我們要intensity: 從 RGB 估"有多濕"

    // 行緩衝(stack,256夠): 陣列放這一行256個像素color
    uint16_t line[256]; 
	
    if (pDraw->iWidth > 256) return 0;

	// 把這行color轉ST7789常用RGB565格式:
    // 透明底變黑/混合-Universal Blue無雨多半alpha=0; bkgd=0 表示黑底
    png.getLineAsRGB565(pDraw, line, PNG_RGB565_LITTLE_ENDIAN, 0x00000000);

    const int y = pDraw->y; //當前解碼到第幾行(0-255)
    if (y < 0 || y >= png_h) return 1;

	// 接下核心迴圈把256欄的PNG縮小寫入到64欄的 wx_map_tmp 矩陣:
    // 256 → WX_N downsample
    // src 座標 (sx,sy) 對應 dst (dx,dy)
	/* 
	原理1: 水平抽樣 (256px 縮 64px)
	int sx = (dx * png_w) / WX_N;
	算式解析: png_w=256, WX_N=64
	-當求小圖第 0 個點(dx=0)時，抓大圖第 0 * 4 = 0 個像素
	-當求小圖第 1 個點(dx=1)時，抓大圖第 1 * 4 = 4 個像素
	-當求小圖第 2 個點(dx=2)時，抓大圖第 2 * 4 = 8 個像素
	why這樣做? 這叫最近鄰抽樣(Nearest-neighbor sampling), 每隔4個點抓1個像素, 直接把256欄壓成64欄.
	
	原理2: 位元解包 (RGB565 拆成 R, G, B)
	uint16_t c = line[sx]; 抓出該像素的16-bit顏色(RGB565 格式)
	RGB565中, 16個binary bit分配:
	前 5 位: 紅色 R (0 ~ 31)
	中間 6 位: 綠色 G (0 ~ 63)
	最後 5 位: 藍色 B (0 ~ 31)
	uint8_t r = (c >> 11) & 0x1F; //右移 11 位，留下最左邊 5 位 (R)
	uint8_t g = (c >> 5)  & 0x3F; //右移 5 位，留下中間 6 位 (G)
	uint8_t b = c & 0x1F;         //直接與 0x1F 做 AND 操作，留下最右邊 5 位 (B)
	
	原理 3: 數值標準化 (將 5/6 bit 轉為 8 bit 即 0~255)
	uint16_t R = (r * 255) / 31;
	uint16_t G = (g * 255) / 63;
	uint16_t B = (b * 255) / 31;
	解析: r 最大值是31. (r * 255) / 31; 把 0~31 的數字等比例放大成0~255. G 同理(最大值 63)。
	
	原理 4: 輝度/亮度公式(Luminance) - 人視覺亮度公式(Rec.601 標準)
	uint16_t lum = (R * 30 + G * 59 + B * 11) / 100;
	輝度 = 0.30 * R + 0.59 * G + 0.11 * B
	解析: 人眼對綠最敏感(佔 59%), 對紅次之(30%), 對藍最不敏感(僅11%). 公式算這顏色在人眼"看起來多亮".
	
	原理 5: 雨量強度算式->雷達雨雲淺藍/綠(小雨); 黃(中雨), 紅/紫 (大/暴雨). 透明背景/無雨區黑色.
	uint8_t inten = 0;
	if (lum > 8) { //亮度>8, 代表不是黑背景, 是有顏色雨雲.
		uint16_t mx = R;
		if (G > mx) mx = G;
		if (B > mx) mx = B; //找出 R, G, B 三個顏色中最亮的那一個值
		inten = (uint8_t)constrain((int)((mx * 2 + lum) / 3), 0, 255);
	}
	
	解析: 雨量強度 inten= ( (最亮顏色成分  mx * 2) + 整體輝度lum ) / 3
	例: 
	狀況 A: 亮紅暴雨區(R=255, G=0, B=0)
	- lum = (255 * 30 + 0 + 0) / 100 = 76.5
	- 最大顏色值 mx = 255
	- inten = ((255 * 2) + 76.5) / 3 = 195 (強度非常高)
	狀況 B: 淺綠的小雨區(R=0, G=150, B=0)
	- lum = (0 + 150 * 59 + 0) / 100 = 88.5
	- 最大顏色值 mx = 150
	- inten = ((150 * 2) + 88.5) / 3 = 129（中等強）
	
	why這樣做? 雷達圖雨點顏色鮮豔，用最大顏色"mx * 2 + 亮度lum"的加權平均，完美把彩雷達圖，轉0(無雨)到 255(特大暴雨)的單一數值，存進1個 Byte裡
	
	原理 6: 垂直抽樣與二維陣列寫入
	int dy = (y * WX_N) / png_h; // 垂直方向：把 256 行 (y) 縮小映射到 64 行 (dy)
	if (dy >= 0 && dy < WX_N) {
		wx_map_tmp[dy * WX_N + dx] = inten; // 寫入一維陣列 (等同二維座標 [dy][dx])
	}
	
	解析：
	- dy = (y * 64) / 256 = y / 4
	- 原圖第 0, 1, 2, 3 行，通通對應到小圖的第 0 行（dy = 0）。
	- 原圖第 4, 5, 6, 7 行，通通對應到小圖的第 1 行（dy = 1）。
	- 1維陣列轉2維座標公式: C語言ram是一直線的. 把表格(dx, dy)存進1維陣列，公式:
	陣列位置 = 行號 dy * 每行長度 WX_N + 列號 dx
	*/
    for (int dx = 0; dx < WX_N; dx++) {
        int sx = (dx * png_w) / WX_N;
        if (sx >= pDraw->iWidth) sx = pDraw->iWidth - 1;

        uint16_t c = line[sx];
        // RGB565 → 粗 intensity
        uint8_t r = (c >> 11) & 0x1F;
        uint8_t g = (c >> 5) & 0x3F;
        uint8_t b = c & 0x1F;
        // 放大到 0-255 近似
        uint16_t R = (r * 255) / 31;
        uint16_t G = (g * 255) / 63;
        uint16_t B = (b * 255) / 31;

        // 無雨：接近黑
        uint16_t lum = (R * 30 + G * 59 + B * 11) / 100;
        uint8_t inten = 0;
        if (lum > 8) {
            // Universal Blue：雨愈大愈「飽和/偏紫紅」；用 max + lum 混合
            uint16_t mx = R;
            if (G > mx) mx = G;
            if (B > mx) mx = B;
            inten = (uint8_t)constrain((int)((mx * 2 + lum) / 3), 0, 255);
        }

        int dy = (y * WX_N) / png_h;
        if (dy >= 0 && dy < WX_N)
            wx_map_tmp[dy * WX_N + dx] = inten;
    }
    return 1;
}

/* 
// http_get_to_buf(): 改做2 fucntion處理json同圖.

// ---------- HTTP 拉 JSON + PNG ----------
static bool http_get_to_buf(const char *url, uint8_t **out, int *out_len, int max_len) {
    *out = nullptr;
    *out_len = 0;

    WiFiClientSecure client;
    client.setInsecure();
    client.setHandshakeTimeout(12);

    HTTPClient http;
    http.begin(client, url);
    http.setUserAgent("adsb-cyd-wx/1.0");
    http.setTimeout(12000);
    http.setConnectTimeout(10000);

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[WX] GET fail %d %s\n", code, url);
        http.end();
        return false;
    }

    int len = http.getSize(); // 可能 -1
    WiFiClient *stream = http.getStreamPtr();

    // 預留上限：JSON 4KB；PNG 建議 48KB
    int cap = max_len;
    uint8_t *buf = (uint8_t *)malloc(cap);
    if (!buf) {
        Serial.println("[WX] malloc fail");
        http.end();
        return false;
    }

    int pos = 0;
    uint32_t t0 = millis();
    while (http.connected() && (millis() - t0 < 15000)) {
        size_t avail = stream->available();
        if (!avail) {
            if (!http.connected()) break;
            delay(1);
            continue;
        }
        if (pos + (int)avail > cap) {
            // 必要時擴一點（最多到 max_len）
            Serial.println("[WX] buffer overflow");
            free(buf);
            http.end();
            return false;
        }
        int rd = stream->readBytes(buf + pos, avail);
        pos += rd;
        if (len > 0 && pos >= len) break;
    }
    http.end();

    *out = buf;
    *out_len = pos;
    return pos > 0;
}  */
// JSON → static wx_json_buf
static bool http_get_json(const char *url, size_t &out_len) {
    out_len = 0;
    WiFiClientSecure client;
    client.setInsecure();
    client.setHandshakeTimeout(12);
    HTTPClient http;
    http.begin(client, url);
    http.setUserAgent("adsb-cyd-wx/1.0");
    http.setTimeout(12000);
    http.setConnectTimeout(10000);

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[WX] GET fail %d\n", code);
        http.end();
        return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    int pos = 0;
    uint32_t t0 = millis();
    const int cap = (int)sizeof(wx_json_buf) - 1;
    while (http.connected() && (millis() - t0 < 12000) && pos < cap) {
        size_t avail = stream->available();
        if (!avail) {
            if (!http.connected()) break;
            vTaskDelay(1);
            continue;
        }
        int rd = stream->readBytes(wx_json_buf + pos, min((int)avail, cap - pos));
        pos += rd;
    }
    http.end();
    wx_json_buf[pos] = 0;
    out_len = (size_t)pos;
    return pos > 0;
}

// PNG → static 或 malloc
static bool http_get_png(const char *url) {
    if (png_buf && !png_use_static) {
        free(png_buf);
        png_buf = nullptr;
    }
    png_len = 0;
    png_pos = 0;

    size_t free_h = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t max_b  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    if (free_h < 70000 || max_b < 50000) {
        Serial.printf("[WX] low mem free=%u maxblk=%u\n", (unsigned)free_h, (unsigned)max_b);
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();
    client.setHandshakeTimeout(12);
    HTTPClient http;
    http.begin(client, url);
    http.setUserAgent("adsb-cyd-wx/1.0");
    http.setTimeout(15000);

    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("[WX] PNG fail %d\n", code);
        http.end();
        return false;
    }

    const int cap = 48 * 1024;
    if (png_use_static) {
        png_buf = png_static_buf;
    } else {
        png_buf = (uint8_t *)malloc(cap);
        if (!png_buf) { http.end(); return false; }
    }

    WiFiClient *stream = http.getStreamPtr();
    int pos = 0;
    uint32_t t0 = millis();
    while (http.connected() && (millis() - t0 < 15000) && pos < cap) {
        size_t avail = stream->available();
        if (!avail) {
            if (!http.connected()) break;
            vTaskDelay(1);
            continue;
        }
        int rd = stream->readBytes(png_buf + pos, min((int)avail, cap - pos));
        pos += rd;
    }
    http.end();
    png_len = pos;
    return pos > 100; // 至少要有 PNG header
}


static bool fetch_and_decode(float range_nm) {
    if (WiFi.status() != WL_CONNECTED) return false;

	// ==下載前Heap檢查 ==
    // free heap (Bytes)
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    // 最大連續heap (Bytes)
    size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    // 90KB = 92160 Bytes, 60KB = 61440 Bytes
    if (free_heap < 92160 || max_block < 61440) {
        Serial.printf("[WX] Abort download. Insufficent HEAP! Free: %lu, Largest Blk: %lu\n",(unsigned long)free_heap, (unsigned long)max_block);
        return false; // 直接退出，把資源留給航班雷達
    }
    // ================
	
	
	/*
    // 1) JSON
    uint8_t *jbuf = nullptr;
    int jlen = 0;
    if (!http_get_to_buf("https://api.rainviewer.com/public/weather-maps.json",
                         &jbuf, &jlen, 4096)) {
        return false;
    }

    // ArduinoJson 靜態文件較省 heap
    JsonDocument doc; // 若舊版用 StaticJsonDocument<2048>
    DeserializationError err = deserializeJson(doc, jbuf, jlen);
    free(jbuf);
    jbuf = nullptr;
    if (err) {
        Serial.printf("[WX] json err %s\n", err.c_str());
        return false;
    }  */
	// ---- A: JSON（http_mutex）----
    if (!http_mutex_acquire(pdMS_TO_TICKS(3000))) {
        Serial.println("[WX] http mutex busy (json)");
        return false;
    }
    size_t jlen = 0;
    bool j_ok = http_get_json("https://api.rainviewer.com/public/weather-maps.json", jlen);
    http_mutex_release();
    if (!j_ok) return false;

	/*
    const char *host = doc["host"] | "https://tilecache.rainviewer.com";
    JsonArray past = doc["radar"]["past"].as<JsonArray>();
    if (past.isNull() || past.size() == 0) {
        Serial.println("[WX] no past frames");
        return false;
    }
    // 最新 = 最後一筆
    JsonObject frame = past[past.size() - 1];
    const char *path = frame["path"] | "";
    if (!path[0]) return false;

    int z = zoom_for_range(range_nm);
    char url[256];
	//	pngDrawCB只寫wx_map_tmp，不用lock. 下面line在callback stack上 OK.
    snprintf(url, sizeof(url),
             "%s%s/256/%d/%.4f/%.4f/2/1_0.png",
             host, path, z, (double)HOME_LAT, (double)HOME_LON); // size=256, color=2, smooth=1 snow=0
    Serial.printf("[WX] tile %s\n", url);
	
	
    //Serial.printf("[WX] free heap before png: %lu\n",(unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
	*/
	// ---- B: parse（無網路 lock）----
    wx_doc.clear();
    DeserializationError err = deserializeJson(wx_doc, wx_json_buf, jlen);
    if (err) {
        Serial.printf("[WX] json err %s\n", err.c_str());
        return false;
    }
    const char *host = wx_doc["host"] | "https://tilecache.rainviewer.com";
    JsonArray past = wx_doc["radar"]["past"].as<JsonArray>();
    if (past.isNull() || past.size() == 0) return false;
    JsonObject frame = past[past.size() - 1]; //抓陣列最後一個(最新時間點):past[past.size()-1]拿過去紀錄的"最後一筆"->最近幾分鐘最新雷達圖
    const char *path = frame["path"] | "";
    if (!path[0]) return false;

    int z = zoom_for_range(range_nm);
    // free上限注意:文件寫max zoom可能7/10，zoom_for_range已cap到10
    char url[256];
    snprintf(url, sizeof(url), "%s%s/256/%d/%.4f/%.4f/2/1_0.png",
             host, path, z, (double)HOME_LAT, (double)HOME_LON);  // size=256, color=2, smooth=1 snow=0
    Serial.printf("[WX] tile %s\n", url);
    wx_doc.clear(); // 立刻放掉 JSON 佔用
	
	
	// == 第 2 道防線：下載 48KB PNG 前的終極檢查 ==
    size_t max_block_before_png = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    Serial.printf("[WX] Prepare to dl PNG, largest free block: %lu\n", (unsigned long)max_block_before_png);
    // 因為底下的 http_get_to_buf 設定了最大 48 * 1024 (49152 Bytes) 的空間
    // 如果最大連續區塊小於 50000 Bytes，一定會當機，所以必須阻擋
    if (max_block_before_png < 50000) {
        Serial.println("[WX] Insufficent largest block, Abort dl png.");
        return false;
    }
    // ===================================

	/*
    // 2) PNG
    if (png_buf) { free(png_buf); png_buf = nullptr; png_len = 0; }
    if (!http_get_to_buf(url, &png_buf, &png_len, 48 * 1024)) {
        return false;
    }
    Serial.printf("[WX] png %d bytes, heap=%lu\n", png_len,
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
	*/
    // ---- C: PNG（http_mutex）----
    if (!http_mutex_acquire(pdMS_TO_TICKS(5000))) {
        Serial.println("[WX] http mutex busy (png)");
        return false;
    }
    bool p_ok = http_get_png(url);
    http_mutex_release();
    if (!p_ok) {
        if (png_buf && !png_use_static) { free(png_buf); png_buf = nullptr; }
        return false;
    }
    Serial.printf("[WX] png %d bytes heap=%lu\n", png_len,
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
				  
	
	/*
    memset(wx_map_tmp, 0, sizeof(wx_map_tmp));

    int rc = png.open((const char *)"", pngOpen, pngClose, pngRead, pngSeek, pngDrawCB);
    if (rc != PNG_SUCCESS) {
        Serial.printf("[WX] png open fail %d\n", rc);
        free(png_buf); png_buf = nullptr; png_len = 0;
        return false;
    }

    png_w = png.getWidth();
    png_h = png.getHeight();
    Serial.printf("[WX] png %dx%d bpp=%d\n", png_w, png_h, png.getBpp());

    // 解碼（callback 填 wx_map_tmp）
    rc = png.decode(nullptr, 0);
    png.close();

    free(png_buf); png_buf = nullptr; png_len = 0;

    if (rc != PNG_SUCCESS) {
        Serial.printf("[WX] decode fail %d\n", rc);
        return false;
    }  */
	// ---- D: decode → tmp（不 lock buffer）----
    memset(wx_map_tmp, 0, sizeof(wx_map_tmp)); //把64x64臨時陣列清空為0
    int rc = png.open((const char *)"", pngOpen, pngClose, pngRead, pngSeek, pngDrawCB);
    if (rc != PNG_SUCCESS) {
        Serial.printf("[WX] png open fail %d\n", rc);
        if (png_buf && !png_use_static) { free(png_buf); png_buf = nullptr; }
        png_len = 0;
        return false;
    }
    png_w = png.getWidth();
    png_h = png.getHeight();
	
	//png.decode()極小RAM處理大圖: 不一次整圖decode放RAM: 解碼0th行→call pngDrawCB → 解碼1st行→ call pngDrawCB...
    rc = png.decode(nullptr, 0); //開始解碼 自動多次pngDrawCB
	
    png.close();
    if (png_buf && !png_use_static) { free(png_buf); png_buf = nullptr; }
    png_len = 0;
    if (rc != PNG_SUCCESS) {
        Serial.printf("[WX] decode fail %d\n", rc);
        return false;
    }
	

	/*
    memcpy(wx_map, wx_map_tmp, sizeof(wx_map));
    _stored_for_range = range_nm;
    _ready = true;
    _last_update = millis();
    Serial.printf("[WX] ok heap=%lu\n",
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return true;   */
	//
	// 雙重緩衝安全交接 (memcpy)-解碼成功後，搶佔wx_buf互斥鎖，超快memcpy把剛算好的wx_map_tmp蓋到顯示用的wx_map.
	// why? draw_radar隨時讀wx_map, 若直接解碼到wx_map->畫面會見天氣圖"一格一格慢慢變好"的過程. 
	// 先wx_map_tmp處理好, 花0.001秒copy去wx_map -> 看到無瑕疵畫面!
	// ---- 只在 swap 時 lock（極短）----
    if (wx_buf_take(pdMS_TO_TICKS(50))) { 
        memcpy(wx_map, wx_map_tmp, sizeof(wx_map)); //瞬間複製 4KB 數據
        _stored_for_range = range_nm;
        _ready = true;
        _last_update = millis();
        wx_buf_give();
    } else {
        // 極少數：畫圖正持有；可再試一次或下輪再 swap
        if (wx_buf_take(pdMS_TO_TICKS(200))) {
            memcpy(wx_map, wx_map_tmp, sizeof(wx_map));
            _stored_for_range = range_nm;
            _ready = true;
            _last_update = millis();
            wx_buf_give();
        }
    }
    Serial.printf("[WX] ok heap=%lu\n",
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return true;
}

// ---------- 公開 API ----------
/*
void weather_init() {
    memset(wx_map, 0, sizeof(wx_map));
    _ready = false;
    _last_update = 0;
    _last_try = 0;
    Serial.printf("[WX] map %d bytes static\n", (int)sizeof(wx_map));
}  */
void weather_init() {
    memset(wx_map, 0, sizeof(wx_map));
    memset(wx_map_tmp, 0, sizeof(wx_map_tmp));
    _ready = false;
    _last_update = 0;
    _last_try = 0;
    if (!_wx_buf_mtx) _wx_buf_mtx = xSemaphoreCreateMutex();
    Serial.printf("[WX] map %d+%d bytes static, png_static=%d\n",
                  (int)sizeof(wx_map), (int)sizeof(wx_map_tmp),
                  (int)sizeof(png_static_buf));
}

/*
void weather_poll() {
    if (!_enabled) return;
    if (WiFi.status() != WL_CONNECTED) return;

    uint32_t now = millis();
    // 啟動後 20s 再抓，避開 boot 搶 heap；之後每 5 分鐘
    if (_last_update == 0 && now < 20000) return;
    if (_last_update > 0 && (now - _last_update) < WX_UPDATE_MS) return;
    if ((now - _last_try) < 30000) return; // 失敗冷卻 30s
    _last_try = now;

	
    // 用「目前主畫面常用 range」——由 main 設也可以；這裡先用 50nm 預設
    // 更好：main 呼叫 weather_request_range(RANGES[range_idx])
	//
    //extern float weather_request_range_nm; // 見下 main 小改
	//float rng = weather_request_range_nm;    
	float rng = _want_range;

    if (rng < 1.0f) rng = 50.0f;

    if (!http_mutex_acquire(pdMS_TO_TICKS(100))) {
        Serial.println("[WX] mutex busy");
        return;
    }
    bool ok = fetch_and_decode(rng);
    http_mutex_release();
    if (!ok) Serial.println("[WX] fetch failed");
}*/
static void weather_task(void *param) {
    (void)param;
    // 等 WiFi
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    // ★ 啟動後至少 20 秒
    vTaskDelay(pdMS_TO_TICKS(WX_FIRST_DELAY_MS));

    for (;;) {
        if (!_enabled) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        uint32_t now = millis();
        bool due = (_last_update == 0) ||
                   (now - _last_update >= WX_UPDATE_MS);
        // 失敗冷卻 30s
        if (due && (now - _last_try >= 30000)) {
            _last_try = now;
            float rng = _want_range;
            if (rng < 1.0f) rng = 50.0f;

            // 可選：避開 ADS-B fetch 準點（用 fetcher_last_update）
            // 若剛 fetch 完 <2s 或距離下次 fetch <3s，可再 delay
            bool ok = fetch_and_decode(rng);
            if (!ok) Serial.println("[WX] fetch failed");
        }
        // 任務很閒：睡 5s 檢查一次即可，省 CPU
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void weather_start_task() {
    // Core 0、低優先；stack 先 10240，穩後可 8192
    xTaskCreatePinnedToCore(
        weather_task, "weather",
        10240, nullptr,
        0,          // priority 低於 fetch(1)
        nullptr,
        0           // core 0
    );
}


/*
void weather_draw_overlay(
    TFT_eSPI &tft,
    float range_nm,
    int cx, int cy, int radar_r,
    bool night_mode
) {
    if (!_enabled || !_ready) return;

    // intensity 圖假設覆蓋「以 home 為中心、邊長 = 2*stored_range 的正方形」
    // 若目前 range 與 stored 差很多，比例會歪；v1 可接受，或 range 變就重抓
    float base = _stored_for_range > 1.0f ? _stored_for_range : range_nm;
    // 地圖座標：wx 像素 (i,j) → nm_east/north
    // i=0 → -base nm east?  中心 WX_N/2
    const float half = (WX_N - 1) * 0.5f;
    const float nm_per_wx = (2.0f * base) / (float)WX_N; // 整張寬 2*base nm
    const float scale = (float)radar_r / range_nm;       // px per nm

    // 只掃可能落在圓內的點；步進 1
    for (int j = 0; j < WX_N; j++) {
        float nm_n = (half - (float)j) * nm_per_wx; // 上北下南
        for (int i = 0; i < WX_N; i++) {
            uint8_t v = wx_map[j * WX_N + i];
            if (v < WX_MIN_INTENSITY) continue;

            float nm_e = ((float)i - half) * nm_per_wx;
            // 若這張是依 base 存的，而顯示 range 不同：仍用同一 nm，再 scale 到螢幕
            float dist2 = nm_e * nm_e + nm_n * nm_n;
            if (dist2 > range_nm * range_nm) continue;

            int px = cx + (int)(nm_e * scale);
            int py = cy - (int)(nm_n * scale);

            // 圓外再擋一次
            int dx = px - cx, dy = py - cy;
            if (dx * dx + dy * dy > radar_r * radar_r) continue;

            // 不 pushImage：單點。雨區密時可用 drawPixel
            tft.drawPixel(px, py, wx_color(v, night_mode));
        }
        // 防 WDT
        if ((j & 15) == 0) yield();
    }
}   */
// 畫echo圖,用盡STATUS BAR下面整個畫面
void weather_draw_overlay(
    TFT_eSPI &tft,
    float range_nm,
    int cx, int cy, int radar_r,
    bool night_mode
) {
    if (!_enabled || !_ready) return;

    // try-lock：拿不到就這幀仍畫weather（因swap 極短通常拿得到）
    if (!wx_buf_take(0)) {
        // 仍畫weather(資料是舊,即上一完整幀).也可uncomment下一行直接return -> 免風險
        //return;
        // 建議短等一下
        if (!wx_buf_take(pdMS_TO_TICKS(2))) return;
    }
	
	// base: 目前天氣圖基於多少海浬下載
    float base = _stored_for_range > 1.0f ? _stored_for_range : range_nm;
	// half: 64x64陣列中心點 (63 * 0.5 = 31.5)
    const float half = (WX_N - 1) * 0.5f;
	// nm_per_wx: 陣列裡每一個格子，代表現實中多少海浬
    const float nm_per_wx = (2.0f * base) / (float)WX_N;
	// scale: 螢幕像素與海浬的縮放比例 (保留原版的完美設計)
    const float scale = (float)radar_r / range_nm;
    const float r2 = range_nm * range_nm;
    const int rr2 = radar_r * radar_r;

	// 開始掃64x64的二維陣列 (天氣強度圖)
    for (int j = 0; j < WX_N; j++) {
		// 算這格在現實中距離中心點的"北方(Y)"多少海浬
        float nm_n = (half - (float)j) * nm_per_wx;
        for (int i = 0; i < WX_N; i++) {
			// 讀天氣強度(0~255)
            uint8_t v = wx_map[j * WX_N + i];
			// 如強度太低(沒下雨)，跳過不畫 節省效能
            if (v < WX_MIN_INTENSITY) continue;
			// 算這格在現實中距離中心點的"東方(X)"多少海浬
            float nm_e = ((float)i - half) * nm_per_wx;
			
			//
			// uncomment下面line: 若想限制畫在圓radar裡面
			// 檢查這片雲實際海浬距離，有沒有超出設定的 range_nm 圓圈
            //if (nm_e * nm_e + nm_n * nm_n > r2) continue;
			
			// 計在螢幕上的x,y像素座標
            int px = cx + (int)(nm_e * scale);
            int py = cy - (int)(nm_n * scale);
						
			//
			// uncomment下面2 line: 若想限制畫在圓radar裡面
			// 檢查這片雲在螢幕上的像素位置，有沒有超出雷達的 radar_r 圓圈
			// 圓外再擋一次
            //int dx = px - cx, dy = py - cy;
            //if (dx * dx + dy * dy > rr2) continue;
			
			//
            // 螢幕邊界與Status Bar限制: 
            // 1. px < 0 或是 px >= 320 -> 超出螢幕左右，不畫
            // 2. py >= 240 -> 超出螢幕下方，不畫
            // 3. py < 20 (STATUS_H) -> 這是頂部狀態欄，絕對不能畫！
			// comment this line: 若想限制畫在圓radar裡面
			////////// if (px < 0 || px >= LCD_H_RES || py >= LCD_V_RES || py < STATUS_H) continue;
            if (px < 0 || px >= LCD_H_RES || py >= LCD_V_RES || py < 20) continue;
			
            tft.drawPixel(px, py, wx_color(v, night_mode));
        }
		//每畫完一行(16的倍數),讓CPU給其他FreeRTOS任務,免Watchdog報錯重啟
        if ((j & 15) == 0) yield();
    }

    wx_buf_give();
}


// main 更新"想要的 range"
void weather_set_range(float range_nm) { _want_range = range_nm; }
void weather_set_enabled(bool on) { _enabled = on; }
bool weather_get_enabled() { return _enabled; }
bool weather_ready() { return _ready && _enabled; }
uint32_t weather_last_update() { return _last_update; }
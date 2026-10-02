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

// ---------- 狀態 ----------
static bool     _enabled = true;
static bool     _ready = false;
static uint32_t _last_update = 0;
static uint32_t _last_try = 0;
static int      _last_range_idx = -1;   // range 變要重抓或至少重投影；v1 簡化：重抓
static float    _stored_for_range = 0;  // 這張 intensity 對應的 range

// 8-bit 強度圖（中心 = home）。static 不吃 heap fragment
static uint8_t  wx_map[WX_N * WX_N];
static uint8_t  wx_map_tmp[WX_N * WX_N]; // decode 時暫存，成功再 memcpy

// PNG 下載 buffer（只用完就 free）
static uint8_t *png_buf = nullptr;
static int      png_len = 0;
static int      png_pos = 0;

static PNG png;

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

// 解碼時：把 PNG 一行 → intensity，並 downsample 進 wx_map_tmp
// 假設 PNG 是 256x256，中心=home，整張對應「約 2*range 的方框」
// 我們把方框映射到 WX_N×WX_N（覆蓋 ±range 的正方形，之後畫時再裁圓）
static int png_w = 0, png_h = 0;

static int pngDrawCB(PNGDRAW *pDraw) {
    // pDraw->y, pDraw->iWidth, pDraw->pPixels
    // RainViewer 多為 RGBA 或 RGB；用 getLineAsRGB565 最省事
    // 但我們要 intensity：從 RGB 估「有多濕」

    // 行緩衝（stack，256 夠）
    uint16_t line[256];
    if (pDraw->iWidth > 256) return 0;

    // 轉 RGB565；透明底會變黑或混合——Universal Blue 無雨多半 alpha=0
    // bkgd=0 表示黑底
    png.getLineAsRGB565(pDraw, line, PNG_RGB565_LITTLE_ENDIAN, 0x00000000);

    const int y = pDraw->y;
    if (y < 0 || y >= png_h) return 1;

    // 256 → WX_N downsample
    // src 座標 (sx,sy) 對應 dst (dx,dy)
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
}

static bool fetch_and_decode(float range_nm) {
    if (WiFi.status() != WL_CONNECTED) return false;

	// ======== 【新增】第 1 道防線：下載任何資料前的 Heap 檢查 ========
    // 取得剩餘的總記憶體 (單位：Bytes)
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    // 取得最大一塊完整的連續記憶體 (單位：Bytes)
    size_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    // 90KB = 92160 Bytes, 60KB = 61440 Bytes
    if (free_heap < 92160 || max_block < 61440) {
        Serial.printf("[WX] 放棄下載！記憶體過低。總剩餘: %lu, 最大區塊: %lu\n", 
                      (unsigned long)free_heap, (unsigned long)max_block);
        return false; // 直接退出，把資源留給航班雷達
    }
    // ================================================================
	
	
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
    }

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
    // size=256, color=2, smooth=1 snow=0
    snprintf(url, sizeof(url),
             "%s%s/256/%d/%.4f/%.4f/2/1_0.png",
             host, path, z, (double)HOME_LAT, (double)HOME_LON);
    Serial.printf("[WX] tile %s\n", url);
	
	/*
    Serial.printf("[WX] free heap before png: %lu\n",
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
	*/
	// ======== 【新增】第 2 道防線：下載 48KB PNG 前的終極檢查 ========
    size_t max_block_before_png = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    Serial.printf("[WX] Prepare to dl PNG, largest free block: %lu\n", (unsigned long)max_block_before_png);
    // 因為底下的 http_get_to_buf 設定了最大 48 * 1024 (49152 Bytes) 的空間
    // 如果最大連續區塊小於 50000 Bytes，一定會當機，所以必須阻擋
    if (max_block_before_png < 50000) {
        Serial.println("[WX] Insufficent largest free block, Abort dl png.");
        return false;
    }
    // ================================================================

	
    // 2) PNG
    if (png_buf) { free(png_buf); png_buf = nullptr; png_len = 0; }
    if (!http_get_to_buf(url, &png_buf, &png_len, 48 * 1024)) {
        return false;
    }
    Serial.printf("[WX] png %d bytes, heap=%lu\n", png_len,
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

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
    }

    memcpy(wx_map, wx_map_tmp, sizeof(wx_map));
    _stored_for_range = range_nm;
    _ready = true;
    _last_update = millis();
    Serial.printf("[WX] ok heap=%lu\n",
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return true;
}

// ---------- 公開 API ----------
void weather_init() {
    memset(wx_map, 0, sizeof(wx_map));
    _ready = false;
    _last_update = 0;
    _last_try = 0;
    Serial.printf("[WX] map %d bytes static\n", (int)sizeof(wx_map));
}

void weather_set_enabled(bool on) { _enabled = on; }
bool weather_get_enabled() { return _enabled; }
bool weather_ready() { return _ready && _enabled; }
uint32_t weather_last_update() { return _last_update; }

// 由 main 更新「想要的 range」
//float weather_request_range_nm = 50.0f;
static float _want_range = 50.0f;
void weather_set_range(float range_nm) { _want_range = range_nm; }
// poll 裡用 _want_range，刪掉 extern

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
	/*
    extern float weather_request_range_nm; // 見下 main 小改
	float rng = weather_request_range_nm;    */
	float rng = _want_range;

    if (rng < 1.0f) rng = 50.0f;

    if (!http_mutex_acquire(pdMS_TO_TICKS(100))) {
        Serial.println("[WX] mutex busy");
        return;
    }
    bool ok = fetch_and_decode(rng);
    http_mutex_release();
    if (!ok) Serial.println("[WX] fetch failed");
}




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
}
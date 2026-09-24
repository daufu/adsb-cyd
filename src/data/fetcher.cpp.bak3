#include "fetcher.h"
#include "error_log.h"
#include "http_mutex.h"
#include "../config.h"
#include "../data/storage.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>

static volatile NetType _active_net = NET_NONE;

static AircraftList *_aircraft_list = nullptr;
static uint32_t _last_update = 0;
static TaskHandle_t _fetch_task_handle = nullptr;
static TaskHandle_t _route_task_handle = nullptr;
static FetcherStats _fstats = {};

// Military alert dedup
#define ALERTED_MAX 64
static char _alerted_hexes[ALERTED_MAX][7];
static int _alerted_count = 0;
static int _alerted_write = 0;

static bool already_alerted(const char *hex) {
    for (int i = 0; i < _alerted_count; i++) {
        if (strcmp(_alerted_hexes[i], hex) == 0) return true;
    }
    return false;
}

static void mark_alerted(const char *hex) {
    strlcpy(_alerted_hexes[_alerted_write], hex, 7);
    _alerted_write = (_alerted_write + 1) % ALERTED_MAX;
    if (_alerted_count < ALERTED_MAX) _alerted_count++;
}

static bool check_military(const char *hex) {
    uint32_t h = strtoul(hex, nullptr, 16);
    if (h >= 0xADF7C8 && h <= 0xAFFFFF) return true;
    if (h >= 0xA00001 && h <= 0xA00FFF) return true;
    if (h >= 0x43C000 && h <= 0x43CFFF) return true;
    if (h >= 0x3B0000 && h <= 0x3BFFFF) return true;
    if (h >= 0x3F4000 && h <= 0x3F7FFF) return true;
    if (h >= 0xC0CDF9 && h <= 0xC0FFFF) return true;
    if (h >= 0x7C8000 && h <= 0x7CBFFF) return true;
    if (h >= 0x0A4000 && h <= 0x0A4FFF) return true;
    return false;
}

static bool check_emergency(uint16_t squawk) {
    return squawk == 7500 || squawk == 7600 || squawk == 7700;
}

static int find_aircraft(const char *hex) {
    for (int i = 0; i < _aircraft_list->count; i++) {
        if (strcmp(_aircraft_list->aircraft[i].icao_hex, hex) == 0)
            return i;
    }
    return -1;
}

struct ParsedEntry {
    char hex[7];
    char callsign[9];
    char registration[9];
    char type_code[5];
    char category[3];
    char desc[40];
    char owner_op[32];
    float lat, lon;
    int32_t altitude;
    int16_t speed, heading, vert_rate;
    uint16_t squawk;
    bool on_ground;
    float mach;
    int16_t ias, tas;
    int32_t nav_altitude;
    float roll;
    float nav_qnh;
	float dist_nm;   // ← 新增：相對 HOME 的距離
};

static void apply_parsed(Aircraft &a, const ParsedEntry &p, bool is_new) {
    strlcpy(a.icao_hex, p.hex, sizeof(a.icao_hex));
    strlcpy(a.callsign, p.callsign, sizeof(a.callsign));
    strlcpy(a.registration, p.registration, sizeof(a.registration));
    strlcpy(a.type_code, p.type_code, sizeof(a.type_code));
    strlcpy(a.category, p.category, sizeof(a.category));
    strlcpy(a.desc, p.desc, sizeof(a.desc));
    strlcpy(a.owner_op, p.owner_op, sizeof(a.owner_op));
    a.lat = p.lat;
    a.lon = p.lon;
    a.altitude = p.altitude;
    a.speed = p.speed;
    a.heading = p.heading;
    a.vert_rate = p.vert_rate;
    a.squawk = p.squawk;
    a.on_ground = p.on_ground;
    a.mach = p.mach;
    a.ias = p.ias;
    a.tas = p.tas;
    a.nav_altitude = p.nav_altitude;
    a.roll = p.roll;
    a.nav_qnh = p.nav_qnh;
    a.is_military = check_military(a.icao_hex);
    a.is_emergency = check_emergency(a.squawk);
    a.is_watched = false;
    a.last_seen = millis();
    a.stale_since = 0;

    if (is_new) a.trail_count = 0;

    if (a.lat != 0.0f || a.lon != 0.0f) {
        if (a.trail_count < TRAIL_LENGTH) {
            a.trail[a.trail_count] = {a.lat, a.lon, a.altitude, a.last_seen};
            a.trail_count++;
        } else {
            memmove(&a.trail[0], &a.trail[1], (TRAIL_LENGTH - 1) * sizeof(TrailPoint));
            a.trail[TRAIL_LENGTH - 1] = {a.lat, a.lon, a.altitude, a.last_seen};
        }
    }
}

static void parse_aircraft_json(JsonDocument &doc) {
    JsonArray ac = doc["ac"].as<JsonArray>();
    Serial.printf("parse: ac array size=%d\n", ac.size());

    //static ParsedEntry parsed[MAX_AIRCRAFT];
	static ParsedEntry parsed[MAX_AIRCRAFT * 2];
	
    int parsed_count = 0;

    for (JsonObject obj : ac) {
        if (parsed_count >= MAX_AIRCRAFT) break;
		/*
        float lat = obj["lat"] | 0.0f;
        float lon = obj["lon"] | 0.0f;
        if (lat == 0.0f && lon == 0.0f) continue;

        ParsedEntry &p = parsed[parsed_count];
        strlcpy(p.hex, obj["hex"] | "", sizeof(p.hex));
        strlcpy(p.callsign, obj["flight"] | "", sizeof(p.callsign));
        for (int i = strlen(p.callsign) - 1; i >= 0 && p.callsign[i] == ' '; i--)
            p.callsign[i] = '\0';
        strlcpy(p.registration, obj["r"] | "", sizeof(p.registration));
        strlcpy(p.type_code, obj["t"] | "", sizeof(p.type_code));
        strlcpy(p.category, obj["category"] | "", sizeof(p.category));
        strlcpy(p.desc, obj["desc"] | "", sizeof(p.desc));
        strlcpy(p.owner_op, obj["ownOp"] | "", sizeof(p.owner_op));
        p.lat = lat;
        p.lon = lon;
        p.altitude = obj["alt_baro"].is<int>() ? obj["alt_baro"].as<int>() : 0;
        p.speed = (int16_t)(obj["gs"] | 0.0f);
        p.heading = (int16_t)(obj["track"] | 0.0f);
        p.vert_rate = (int16_t)(obj["baro_rate"] | 0.0f);
        p.squawk = strtoul(obj["squawk"] | "0", nullptr, 10);
        p.on_ground = obj["alt_baro"] == "ground";
        p.mach = obj["mach"] | 0.0f;
        p.ias = (int16_t)(obj["ias"] | 0.0f);
        p.tas = (int16_t)(obj["tas"] | 0.0f);
        p.nav_altitude = obj["nav_altitude_mcp"] | 0;
        p.roll = obj["roll"] | 0.0f;
        p.nav_qnh = obj["nav_qnh"] | 0.0f;
        parsed_count++;
		*/
		float lat = obj["lat"] | 0.0f;
        float lon = obj["lon"] | 0.0f;
        if (lat == 0.0f && lon == 0.0f) continue;

        // 略過地面機（香港機場周邊地面機很多，雷達用處不大）
        bool on_gnd = (obj["alt_baro"] == "ground");
        if (on_gnd) continue;

        // 簡易距離（nm），與 main.cpp 相同近似公式
        float dlat = lat - HOME_LAT;
        float dlon = lon - HOME_LON;
        float cos_lat = cosf(HOME_LAT * 3.14159265f / 180.0f);
        float nm_n = dlat * 60.0f;
        float nm_e = dlon * 60.0f * cos_lat;
        float dist = sqrtf(nm_n * nm_n + nm_e * nm_e);

        // 比 API radius 再緊一點也可以（例如只要 radius 內）
        if (dist > (float)ADSB_RADIUS_NM) continue;

        // 先盡量收進暫存；若超過 MAX*2 就先停，避免爆
        if (parsed_count >= MAX_AIRCRAFT * 2) break;

        ParsedEntry &p = parsed[parsed_count];
        strlcpy(p.hex, obj["hex"] | "", sizeof(p.hex));
        strlcpy(p.callsign, obj["flight"] | "", sizeof(p.callsign));
        for (int i = (int)strlen(p.callsign) - 1; i >= 0 && p.callsign[i] == ' '; i--)
            p.callsign[i] = '\0';
        strlcpy(p.registration, obj["r"] | "", sizeof(p.registration));
        strlcpy(p.type_code, obj["t"] | "", sizeof(p.type_code));
        strlcpy(p.category, obj["category"] | "", sizeof(p.category));
        strlcpy(p.desc, obj["desc"] | "", sizeof(p.desc));
        strlcpy(p.owner_op, obj["ownOp"] | "", sizeof(p.owner_op));
        p.lat = lat;
        p.lon = lon;
        p.altitude = obj["alt_baro"].is<int>() ? obj["alt_baro"].as<int>() : 0;
        p.speed = (int16_t)(obj["gs"] | 0.0f);
        p.heading = (int16_t)(obj["track"] | 0.0f);
        p.vert_rate = (int16_t)(obj["baro_rate"] | 0.0f);
        p.squawk = (uint16_t)strtoul(obj["squawk"] | "0", nullptr, 10);
        p.on_ground = false;
        p.mach = obj["mach"] | 0.0f;
        p.ias = (int16_t)(obj["ias"] | 0.0f);
        p.tas = (int16_t)(obj["tas"] | 0.0f);
        p.nav_altitude = obj["nav_altitude_mcp"] | 0;
        p.roll = obj["roll"] | 0.0f;
        p.nav_qnh = obj["nav_qnh"] | 0.0f;
        p.dist_nm = dist;
        parsed_count++;
    }

	/*
	Every time fresh flight data is downloaded, organize the planes, 
	update existing ones, add new ones, clean up planes that've left the radar range.
	*/
	//
	// 自己ADD. 
	// Step 1: Sorting the Planes by Distance:
	// sorts all parsed aircraft by how close they're to u, putting the closest ones first.
	// 依距離由近到遠排序（簡單 insertion sort，飛機數少夠用）
    for (int i = 1; i < parsed_count; i++) {
        ParsedEntry tmp = parsed[i];
        int j = i - 1;
        while (j >= 0 && parsed[j].dist_nm > tmp.dist_nm) {
            parsed[j + 1] = parsed[j];
            j--;
        }
        parsed[j + 1] = tmp;
    }
	
    if (parsed_count > MAX_AIRCRAFT) parsed_count = MAX_AIRCRAFT;
	
    if (!_aircraft_list->aircraft || !_aircraft_list->lock()) return;

    uint32_t now = millis();
    bool seen[MAX_AIRCRAFT] = {};

	//
	// Step 2: Updating or Adding Planes to Your List:
	// checks every incoming plane against active list using its unique ID (hex code).
	// - If the plane is already there: updates its position, speed, alt (apply_parsed), marks it as seen.
	// - If it's a new plane: If <= maximum limit (MAX_AIRCRAFT), it adds the plane to active list.
    for (int p = 0; p < parsed_count; p++) {
        int idx = find_aircraft(parsed[p].hex);
        if (idx >= 0) {
			// 已在追蹤清單中 -> 更新資料
            apply_parsed(_aircraft_list->aircraft[idx], parsed[p], false);
            seen[idx] = true;
        } else if (_aircraft_list->count < MAX_AIRCRAFT) {
			// 新飛機 -> 加進清單
            int new_idx = _aircraft_list->count;
            _aircraft_list->aircraft[new_idx].clear();
            apply_parsed(_aircraft_list->aircraft[new_idx], parsed[p], true);
            _aircraft_list->count++;
            seen[new_idx] = true;
        }
    }
	
	//
	// Step 3: Clean Up "Ghost" Planes (Disappearing Aircraft):
	// 有時 update might miss a plane (weak wifi/data drops). Instead of instantly deleting the plane from screen, 
	// the code gives missing planes a grace period (GHOST_TIMEOUT_MS).
	// If a plane isn't seen in latest update, it notes the time it disappeared. 
	// If it stays missing for too long, it finally removes it from the list (_aircraft_list->count = write).
    int write = 0;
    for (int i = 0; i < _aircraft_list->count; i++) {
        Aircraft &a = _aircraft_list->aircraft[i];
        if (!seen[i]) {
            if (a.stale_since == 0) a.stale_since = now;
            if (now - a.stale_since > GHOST_TIMEOUT_MS) continue;
        }
        if (write != i) _aircraft_list->aircraft[write] = _aircraft_list->aircraft[i];
        write++;
    }
    _aircraft_list->count = write;

    _aircraft_list->unlock();
    _last_update = millis();
}

static bool network_connected() {
    if (WiFi.status() == WL_CONNECTED) {
        _active_net = NET_WIFI;
        return true;
    }
    _active_net = NET_NONE;
    return false;
}

static void update_ip_addr() {
    if (_active_net == NET_WIFI)
        strlcpy(_fstats.ip_addr, WiFi.localIP().toString().c_str(), sizeof(_fstats.ip_addr));
    else
        strlcpy(_fstats.ip_addr, "N/A", sizeof(_fstats.ip_addr));
}

static void fetch_task(void *param) {
    // Wait for WiFi with retry and radio recycle
    Serial.print("Fetcher: waiting for WiFi");
    int wait_cycles = 0;
    while (!network_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        wait_cycles++;
        // After 20s (40 cycles), recycle radio and retry
        if (wait_cycles % 40 == 0) {
            Serial.printf("\nWiFi retry — recycling radio (attempt %d)\n", wait_cycles / 40 + 1);
            WiFi.disconnect(false);
            WiFi.mode(WIFI_OFF);
            vTaskDelay(pdMS_TO_TICKS(500));
            WiFi.mode(WIFI_STA);
            WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass);
        }
    }
    update_ip_addr();
    Serial.printf("\nWiFi connected, IP: %s\n", _fstats.ip_addr);

    char url[128];
    snprintf(url, sizeof(url), "https://api.adsb.lol/v2/point/%.4f/%.4f/%d",
             HOME_LAT, HOME_LON, ADSB_RADIUS_NM);
    Serial.printf("ADS-B API URL: %s\n", url);

    while (true) {
        if (network_connected()) {
            if (http_mutex_acquire(pdMS_TO_TICKS(15000))) {
                WiFiClientSecure client;
                client.setInsecure();
                client.setHandshakeTimeout(10);

                HTTPClient http;
                http.begin(client, url);
                http.setUserAgent("adsb-cyd-proj/1.0"); // ←加這行
                http.setTimeout(10000);
                uint32_t t0 = millis();
                int httpCode = http.GET();
                Serial.printf("Fetch: HTTP %d, %lums, heap=%lu\n",
                    httpCode, (unsigned long)(millis() - t0),
                    (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

                if (httpCode == HTTP_CODE_OK) {
                    _fstats.last_fetch_ms = millis() - t0;

                    // Read full response with deadline loop (fixes truncated JSON)
					/*
                    int content_len = http.getSize();
                    size_t buf_size = (content_len > 0) ? (size_t)content_len + 1 : 64 * 1024;
                    char *buf = (char *)malloc(buf_size);*/
					int content_len = http.getSize();
                    // 限制最大 buffer，避免忙區 JSON 把 DRAM 打爆
                    const size_t BUF_CAP = 48 * 1024;
                    size_t buf_size;
                    if (content_len > 0) {
                        buf_size = (size_t)content_len + 1;
                        if (buf_size > BUF_CAP) buf_size = BUF_CAP;
                    } else {
                        buf_size = BUF_CAP;
                    }
                    char *buf = (char *)malloc(buf_size);
                    size_t total = 0;

                    if (buf) {
                        size_t target = (content_len > 0) ? (size_t)content_len : buf_size - 1;
						if (target > buf_size - 1) target = buf_size - 1;
                        WiFiClient *stream = http.getStreamPtr();
                        uint32_t deadline = millis() + 15000;
                        while (total < target && millis() < deadline) {
                            int avail = stream->available();
                            if (avail > 0) {
                                int to_read = min((size_t)avail, target - total);
                                total += stream->readBytes(buf + total, to_read);
                            } else if (!stream->connected()) {
                                break;
                            } else {
                                vTaskDelay(1);
                            }
                        }
                        buf[total] = '\0';
                    }
                    _fstats.bytes_received += total;

                    http.end();           // <-- moved up: free TLS session buffers before we parse
                    http_mutex_release(); // <-- release the mutex too, since we're done with the network

                    // Parse with filter — only extract fields we need
                    JsonDocument filter;
                    JsonObject af = filter["ac"][0].to<JsonObject>();
                    af["hex"] = true;
                    af["flight"] = true;
                    af["r"] = true;
                    af["t"] = true;
                    af["category"] = true;
                    af["desc"] = true;
                    af["ownOp"] = true;
                    af["lat"] = true;
                    af["lon"] = true;
                    af["alt_baro"] = true;
                    af["gs"] = true;
                    af["track"] = true;
                    af["baro_rate"] = true;
                    af["squawk"] = true;
                    af["mach"] = true;
                    af["ias"] = true;
                    af["tas"] = true;
                    af["nav_altitude_mcp"] = true;
                    af["roll"] = true;
                    af["nav_qnh"] = true;

                    JsonDocument doc;
                    DeserializationError err = (buf && total > 0)
                        ? deserializeJson(doc, buf, total, DeserializationOption::Filter(filter))
                        : DeserializationError::EmptyInput;
                    if (buf) free(buf);

                    if (!err) {
                        _fstats.fetch_ok++;
                        Serial.printf("JSON OK, ac array ptr=%p, heap=%lu\n",
                            (void*)_aircraft_list->aircraft,
                            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
                        parse_aircraft_json(doc);
                        Serial.printf("Fetched %d ac, heap=%lu\n",
                            _aircraft_list->count,
                            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
                    } else {
                        _fstats.fetch_fail++;
                        error_log_add("JSON: %s", err.c_str());
                        Serial.printf("JSON error: %s\n", err.c_str());
                    }
                } else {
                    _fstats.fetch_fail++;
                    error_log_add("HTTP %d", httpCode);
					//
					//上面IF_BLOCK (成功 HTTP 200/HTTP_CODE_OK): 有呼叫 http.end() + http_mutex_release()。
					//失敗時（超時、429 限流、斷線等）: 這兩行也要加, 否則鎖永遠被占住
					//之後每次抓都拿不到鎖 → 再也不更新, 畫面永遠顯示舊資料
					http.end();				//關掉HTTP/TLS
					http_mutex_release();	//把鎖還回去
                }
				
				//http.end();
                //http_mutex_release();
            }
        } else {
            error_log_add("Network down");
            WiFi.reconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(ADSB_POLL_INTERVAL_MS));
    }
}

static void route_enrich_task(void *param) {
    while (!network_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (true) {
        char callsign[9] = {};
        char icao_hex[7] = {};
        bool found = false;

        if (_aircraft_list->lock(pdMS_TO_TICKS(100))) {
            for (int i = 0; i < _aircraft_list->count; i++) {
                Aircraft &a = _aircraft_list->aircraft[i];
                if (a.callsign[0] && !a.origin[0] && a.stale_since == 0) {
                    strlcpy(callsign, a.callsign, sizeof(callsign));
                    strlcpy(icao_hex, a.icao_hex, sizeof(icao_hex));
                    found = true;
                    break;
                }
            }
            _aircraft_list->unlock();
        }

        if (!found || !network_connected()) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        char origin[5] = {};
        char dest[5] = {};

        if (http_mutex_acquire(pdMS_TO_TICKS(12000))) {
            char url[128];
            snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", callsign);
            WiFiClientSecure client;
            client.setInsecure();
            client.setHandshakeTimeout(8);
            HTTPClient http;
            http.begin(client, url);
            http.setUserAgent("adsb-cyd-proj/1.0"); // ←加這行
            http.setTimeout(8000);
            int code = http.GET();

            if (code == HTTP_CODE_OK) {
                WiFiClient *stream = http.getStreamPtr();
                JsonDocument doc;
                if (!deserializeJson(doc, *stream)) {
                    JsonObject route = doc["response"]["flightroute"];
                    strlcpy(origin, route["origin"]["iata_code"] | "", sizeof(origin));
                    strlcpy(dest, route["destination"]["iata_code"] | "", sizeof(dest));
                    _fstats.enrich_ok++;
                } else {
                    _fstats.enrich_fail++;
                }
            } else {
                _fstats.enrich_fail++;
            }
            http.end();
            http_mutex_release();
        }

        if (_aircraft_list->lock(pdMS_TO_TICKS(100))) {
            int idx = find_aircraft(icao_hex);
            if (idx >= 0) {
                Aircraft &a = _aircraft_list->aircraft[idx];
                if (origin[0]) strlcpy(a.origin, origin, sizeof(a.origin));
                else strlcpy(a.origin, "-", sizeof(a.origin));
                if (dest[0]) strlcpy(a.dest, dest, sizeof(a.dest));
                else strlcpy(a.dest, "-", sizeof(a.dest));
                Serial.printf("Route: %s %s->%s\n", callsign, a.origin, a.dest);
            }
            _aircraft_list->unlock();
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void fetcher_init(AircraftList *list) {
    _aircraft_list = list;
    http_mutex_init();

    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);

    // Start WiFi non-blocking — fetch_task handles retry/wait
    Serial.printf("WiFi SSID: [%s]\n", g_config.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass);

	/*Arduino loop()（UI/touch）預設Core 1跑; 把HTTP/JSON搬走去Core 0,UI就唔被"同 core 阻塞"
	*/
    //xTaskCreatePinnedToCore(fetch_task, "adsb_fetch", 32768, nullptr, 1, &_fetch_task_handle, 1);
    //xTaskCreatePinnedToCore(route_enrich_task, "route_enrich", 8192, nullptr, 0, &_route_task_handle, 1);
	xTaskCreatePinnedToCore(fetch_task, "adsb_fetch", 32768, nullptr, 1, &_fetch_task_handle, 0);
    xTaskCreatePinnedToCore(route_enrich_task, "route_enrich", 8192, nullptr, 0, &_route_task_handle, 0);
}

bool fetcher_wifi_connected() {
    return network_connected();
}

NetType fetcher_connection_type() {
    return _active_net;
}

uint32_t fetcher_last_update() {
    return _last_update;
}

const FetcherStats* fetcher_get_stats() {
    return &_fstats;
}

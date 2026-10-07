#include "enrichment.h"
#include "http_mutex.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <cstring>

#define MAX_CACHE 10

static AircraftEnrichment _cache[MAX_CACHE];
static char _cache_keys[MAX_CACHE][7];
static int _cache_count = 0;

static void (*_pending_callback)(AircraftEnrichment *) = nullptr;
static volatile bool _task_running = false;

static volatile AircraftEnrichment *_deferred_entry = nullptr;
static volatile bool _deferred_ready = false;

struct EnrichParams {
    char icao_hex[7];
    char callsign[9];
};


static char enrich_resp_buf[3072];  // callsign/aircraft 通常 <2KB
static JsonDocument enrich_doc;
static EnrichParams s_enrich_params;  // 不要 malloc
//static helper function讀body
static bool http_read_to_static(HTTPClient &http, char *buf, size_t cap, size_t &total) {
    total = 0;
    int content_len = http.getSize();
    size_t target = (content_len > 0) ? (size_t)content_len : (cap - 1);
    if (target > cap - 1) target = cap - 1;

    WiFiClient *stream = http.getStreamPtr();
    uint32_t deadline = millis() + 8000;
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
    return total > 0;
}

AircraftEnrichment *enrichment_get_cached(const char *icao_hex) {
    for (int i = 0; i < _cache_count; i++) {
        if (strcmp(_cache_keys[i], icao_hex) == 0 && _cache[i].loaded) {
            return &_cache[i];
        }
    }
    return nullptr;
}

static AircraftEnrichment *get_or_create_cache_entry(const char *icao_hex) {
    for (int i = 0; i < _cache_count; i++) {
        if (strcmp(_cache_keys[i], icao_hex) == 0) return &_cache[i];
    }
    int idx = _cache_count < MAX_CACHE ? _cache_count++ : 0;
    memset(&_cache[idx], 0, sizeof(AircraftEnrichment));
    strlcpy(_cache_keys[idx], icao_hex, 7);
    return &_cache[idx];
}

static void notify_callback(AircraftEnrichment *entry) {
    _deferred_entry = entry;
    _deferred_ready = true;
}

static void fetch_task(void *param) {
    _task_running = true;
    EnrichParams *params = (EnrichParams *)param;
    AircraftEnrichment *entry = get_or_create_cache_entry(params->icao_hex);
    entry->loading = true;

    // Stage 1: Airport names
    if (params->callsign[0] && http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", params->callsign);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setUserAgent("adsb-cyd-proj/1.0"); // ←加這行
        http.setTimeout(5000);
		/* 
		//default: non-static JsonObject
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonObject route = doc["response"]["flightroute"];
                strlcpy(entry->origin_airport, route["origin"]["name"] | "", sizeof(entry->origin_airport));
                strlcpy(entry->destination_airport, route["destination"]["name"] | "", sizeof(entry->destination_airport));
                const char *airline = route["airline"]["name"] | "";
                if (airline[0]) strlcpy(entry->airline, airline, sizeof(entry->airline));
            }
        }  */
		// 改用static JsonObject: route指向enrich_doc固定heap地址
		if (http.GET() == HTTP_CODE_OK) {
			size_t total = 0;
			if (http_read_to_static(http, enrich_resp_buf, sizeof(enrich_resp_buf), total)) {
				enrich_doc.clear();
				if (!deserializeJson(enrich_doc, enrich_resp_buf, total)) {
					JsonObject route = enrich_doc["response"]["flightroute"];
					strlcpy(entry->origin_airport, route["origin"]["name"] | "", sizeof(entry->origin_airport));
					strlcpy(entry->destination_airport, route["destination"]["name"] | "", sizeof(entry->destination_airport));
					const char *airline = route["airline"]["name"] | "";
					if (airline[0]) strlcpy(entry->airline, airline, sizeof(entry->airline));
				}
			}
		}
		
		http.end();
        http_mutex_release();
        notify_callback(entry);
    }

    // Stage 2: Aircraft details
    if (http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s", params->icao_hex);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setUserAgent("adsb-cyd-proj/1.0"); // ←加這行
        http.setTimeout(5000);
		/* 
		//default: non-static JsonObject
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonObject ac = doc["response"]["aircraft"];
                strlcpy(entry->manufacturer, ac["manufacturer"] | "", sizeof(entry->manufacturer));
                strlcpy(entry->model, ac["type"] | "", sizeof(entry->model));
                strlcpy(entry->owner, ac["registered_owner"] | "", sizeof(entry->owner));
                strlcpy(entry->registered_country, ac["registered_owner_country_name"] | "",
                        sizeof(entry->registered_country));
                entry->engine_count = ac["engine_count"] | 0;
                strlcpy(entry->engine_type, ac["engine_type"] | "", sizeof(entry->engine_type));
                entry->year_built = ac["year_built"] | 0;
            }
        }  */		
		// 改用static JsonObject: ac指向enrich_doc固定heap地址
		if (http.GET() == HTTP_CODE_OK) {
			size_t total = 0;
			if (http_read_to_static(http, enrich_resp_buf, sizeof(enrich_resp_buf), total)) {
				enrich_doc.clear();
				if (!deserializeJson(enrich_doc, enrich_resp_buf, total)) {
					JsonObject ac = enrich_doc["response"]["aircraft"];
					strlcpy(entry->manufacturer, ac["manufacturer"] | "", sizeof(entry->manufacturer));
					strlcpy(entry->model, ac["type"] | "", sizeof(entry->model));
					strlcpy(entry->owner, ac["registered_owner"] | "", sizeof(entry->owner));
					strlcpy(entry->registered_country, ac["registered_owner_country_name"] | "",
							sizeof(entry->registered_country));
					entry->engine_count = ac["engine_count"] | 0;
					strlcpy(entry->engine_type, ac["engine_type"] | "", sizeof(entry->engine_type));
					entry->year_built = ac["year_built"] | 0;
				}
			}
		}
        http.end();
        http_mutex_release();
        notify_callback(entry);
    }

    // Stage 3: Photo
    vTaskDelay(pdMS_TO_TICKS(200));
    if (http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url),
                 "https://api.planespotters.net/pub/photos/hex/%s", params->icao_hex);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setUserAgent("adsb-cyd-proj/1.0"); // ←加這行
        http.setTimeout(5000);		
		/* 
		//default: non-static JsonObject
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonArray photos = doc["photos"].as<JsonArray>();
                if (photos.size() > 0) {
                    strlcpy(entry->photo_url, photos[0]["thumbnail_large"]["src"] | "", sizeof(entry->photo_url));
                    strlcpy(entry->photo_photographer, photos[0]["photographer"] | "", sizeof(entry->photo_photographer));
                }
            }
        }  */		
		// 改用static JsonObject: photos指向enrich_doc固定heap地址
		if (http.GET() == HTTP_CODE_OK) {
			size_t total = 0;
			if (http_read_to_static(http, enrich_resp_buf, sizeof(enrich_resp_buf), total)) {
				enrich_doc.clear();
				if (!deserializeJson(enrich_doc, enrich_resp_buf, total)) {
					JsonArray photos = enrich_doc["photos"].as<JsonArray>();
					if (photos.size() > 0) {
						strlcpy(entry->photo_url, photos[0]["thumbnail_large"]["src"] | "", sizeof(entry->photo_url));
						strlcpy(entry->photo_photographer, photos[0]["photographer"] | "", sizeof(entry->photo_photographer));
					}
				}
			}
		}
        http.end();
        http_mutex_release();
    }

    entry->loaded = true;
    entry->loading = false;
    notify_callback(entry);

	/*
	// 因改用static EnrichParams s_enrich_params, 此行唔需要
    free(params); 	*/
    _task_running = false;
	Serial.printf("Enrichment-High Water Mark: %uB\n",(unsigned int)uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(nullptr);
}

void enrichment_fetch(const char *icao_hex, const char *registration,
                      const char *callsign,
                      void (*callback)(AircraftEnrichment *data)) {
    AircraftEnrichment *cached = enrichment_get_cached(icao_hex);
    if (cached) {
        callback(cached);
        return;
    }

    if (_task_running) return;

    _pending_callback = callback;
    _deferred_ready = false;


	/*
	// - params不malloc:
	// 原本: 非static EnrichParams s_enrich_params
	EnrichParams *params = (EnrichParams *)malloc(sizeof(EnrichParams));
    strlcpy(params->icao_hex, icao_hex, sizeof(params->icao_hex));
    strlcpy(params->callsign, callsign ? callsign : "", sizeof(params->callsign));
	
    //loop()（UI/touch）預設Core 1跑; 把HTTP/JSON搬走去Core 0,UI就唔被"同 core 阻塞"
	//
	//xTaskCreatePinnedToCore(fetch_task, "enrich", 8192, params, 0, nullptr, 1);
	xTaskCreatePinnedToCore(fetch_task, "enrich", 8192, params, 0, nullptr, 0);  	*/
	// - 改用 static EnrichParams s_enrich_params
	strlcpy(s_enrich_params.icao_hex, icao_hex, sizeof(s_enrich_params.icao_hex));
	strlcpy(s_enrich_params.callsign, callsign ? callsign : "", sizeof(s_enrich_params.callsign));
	xTaskCreatePinnedToCore(fetch_task, "enrich", 8192, &s_enrich_params, 0, nullptr, 0);
}

// Poll from main loop instead of LVGL timer
void enrichment_poll() {
    if (_deferred_ready && _pending_callback && _deferred_entry) {
        _deferred_ready = false;
        _pending_callback((AircraftEnrichment *)_deferred_entry);
    }
}

void enrichment_init() {
    // No-op on CYD (no LVGL timer — use enrichment_poll() from main loop)
}

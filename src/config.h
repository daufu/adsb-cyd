#pragma once

// WiFi credentials
#define WIFI_SSID "hello"  
#define WIFI_PASS "123456789" 

// Home location
// 香港 22.3193,114.1694 ; 22.3780,114.1130 ; tai mo shan  , 114.1120   
#define HOME_LAT 22.4085 // tai mo shan緯度
#define HOME_LON 114.1120 // tai mo shan經度

// ADS-B settings — reduced for CYD (no PSRAM, 320KB DRAM)
/*
長途/預警模式（250 nm）：只用來監控台海中線及遠方飛來的航班。此模式下關閉航班編號文字，只畫小點，避免畫面擠爆。
標準珠三角模式（50 nm 或 60 nm）：你的日常主畫面。顯示香港全地圖，並開啟航班編號。這時你可以清楚看到深港澳三地機場的互動。
近景模式（15-20 nm）：看香港跑道最後進場。
- started working when reducing #define ADSB_RADIUS_NM from 75 to 20.default 75 
default 75 
tried 20
*/
#define ADSB_RADIUS_NM 40   
#define ADSB_POLL_INTERVAL_MS 15000   //15000毫秒=10秒; default=5000
#define MAX_AIRCRAFT 25	//50
#define TRAIL_LENGTH 6	//15

// CYD display
#define LCD_H_RES 320
#define LCD_V_RES 240

// CYD touch calibration (landscape rotation 1)
#define TOUCH_X_MIN 2600
#define TOUCH_X_MAX 1100
#define TOUCH_Y_MIN 3950
#define TOUCH_Y_MAX 280

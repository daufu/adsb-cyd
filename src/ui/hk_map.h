#pragma once
#include <TFT_eSPI.h>

// 畫香港/珠三角簡化線條地圖（背景）
// - home_lat/home_lon：雷達中心（對應 config.h HOME_LAT/HOME_LON）
// - range_nm：目前雷達 range（150/100/50/20/5）
// - radar_cx/cy/r：雷達圓心同半徑（像素）
// - color：線條顏色（建議用 pal->grid，夠暗唔搶飛機）
void hk_map_draw(
    TFT_eSPI &tft,
    float home_lat, float home_lon,
    float range_nm,
    int radar_cx, int radar_cy, int radar_r,
    uint16_t color
);
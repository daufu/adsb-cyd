#pragma once
#include <TFT_eSPI.h>
#include <stdint.h>
#include <stdbool.h>

// 強度圖解析度（奇數較好，中心對齊）。128 → 16KB
#define WX_N           64 //改64. 原128
#define WX_UPDATE_MS   (5UL * 60UL * 1000UL)  // 5 分鐘
#define WX_MIN_INTENSITY  8   // 低於此當無雨，不畫

void weather_init();
void weather_poll();          // 在 loop 或 enrichment 旁呼叫
bool weather_ready();
void weather_set_range(float range_nm);

uint32_t weather_last_update();

// 畫在雷達圓內；night=true 用夜間色
void weather_draw_overlay(
    TFT_eSPI &tft,
    float range_nm,
    int cx, int cy, int radar_r,
    bool night_mode
);

// 可選：設定開關
void weather_set_enabled(bool on);
bool weather_get_enabled();

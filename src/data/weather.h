#pragma once
#include <TFT_eSPI.h>
#include <stdint.h>
#include <stdbool.h>

//WX_N? 強度圖解析度(奇數較好，中心對齊). 128→16KB; 64→4KB；
//RAM緊可48,畫質略降. 原128
#define WX_N				64  
#define WX_UPDATE_MS		(10UL * 60UL * 1000UL) //10min
#define WX_MIN_INTENSITY	8 //低於此當無雨，不畫
#define WX_FIRST_DELAY_MS	(20UL * 1000UL) //啟動後20秒才第一次

void weather_init();

/*
//不可以在loop長期HTTP → 改用weather_start_task()
void weather_poll(); //在loop或enrichment旁呼叫  */
void weather_start_task();


bool weather_ready();
void weather_set_range(float range_nm);
bool weather_ready();
uint32_t weather_last_update();


// 畫在雷達圓內；night=true 用夜間色
// 只從 main/draw 呼叫; 內部 try-lock buffer
void weather_draw_overlay(
    TFT_eSPI &tft,
    float range_nm,
    int cx, int cy, int radar_r,
    bool night_mode
);

// 可選:設定開關
void weather_set_enabled(bool on);
bool weather_get_enabled();

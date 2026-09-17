#include "hk_map.h"
#include <math.h>

// === 幾何/投影說明（同 main.cpp 同一種近似）===
// nm_north = (lat - home_lat) * 60
// nm_east  = (lon - home_lon) * 60 * cos(home_lat)
// px = cx + nm_east  * (radar_r / range_nm)
// py = cy - nm_north * (radar_r / range_nm)

struct GeoPt {
    float lat;
    float lon;
};

static inline bool project_if_in_range(
    const GeoPt &p,
    float home_lat, float home_lon,
    float cos_lat,
    float range_nm,
    float scale,
    int cx, int cy,
    int &x, int &y
) {
    float dlat = p.lat - home_lat;
    float dlon = p.lon - home_lon;

    float nm_n = dlat * 60.0f;
    float nm_e = dlon * 60.0f * cos_lat;

    float dist2 = nm_n * nm_n + nm_e * nm_e;
    float r2 = range_nm * range_nm;
    if (dist2 > r2) return false;

    x = cx + (int)(nm_e * scale);
    y = cy - (int)(nm_n * scale);
    return true;
}

static void draw_path(
    TFT_eSPI &tft,
    const GeoPt *pts, int n,
    float home_lat, float home_lon,
    float cos_lat,
    float range_nm,
    float scale,
    int cx, int cy,
    uint16_t color
) {
    if (!pts || n < 2) return;

    for (int i = 1; i < n; i++) {
        int x0, y0, x1, y1;
        bool in0 = project_if_in_range(pts[i - 1], home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x0, y0);
        bool in1 = project_if_in_range(pts[i],     home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x1, y1);

        // Beginner-friendly：只畫「兩端都入 range」嘅線段（避免畫到雷達圈外）
		/*
        if (in0 && in1) {
            tft.drawLine(x0, y0, x1, y1, color);
        }*/
		tft.drawLine(x0, y0, x1, y1, color);
    }
}

// =====================================================
// 下面係「超簡化」香港/珠三角輪廓點（你可再加密/調整）
// 目標：
// - 香港：新界(閉合)、九龍(閉合)、港島(閉合)、大嶼山(閉合)
// - 加入后海灣(Deep Bay)凹位
// - 加入大埔/吐露港(Tolo Harbour)凹位
// - 加入西貢半島向東突出
// - 深圳 coastline（蛇口 → 寶安機場）獨立
// - 珠海 coastline 獨立
// - 澳門（機場附近）coastline 獨立
// =====================================================

// --- 新界九龍（閉合）---
static const GeoPt P_NT_KLN[] = {
    {22.4104f, 113.8986f},
    {22.4853f, 113.9961f},
    {22.4771f, 114.0189f},
    {22.5075f, 114.0329f},
    {22.5029f, 114.0506f},
    {22.5415f, 114.1316f},
    {22.5613f, 114.1646f},
    {22.5473f, 114.2228f},
    {22.5274f, 114.2177f},
    {22.5461f, 114.2494f},
    {22.5040f, 114.3368f},
    {22.4654f, 114.2659f},
    {22.4900f, 114.2798f},
    {22.4701f, 114.2127f},
    {22.4479f, 114.1835f},
    {22.4256f, 114.2139f},
    {22.4432f, 114.2848f},
    {22.4748f, 114.3152f},
    {22.4315f, 114.4001f},
    {22.4116f, 114.4077f},
    {22.4209f, 114.3786f},
    {22.3975f, 114.3672f},
    {22.3870f, 114.3937f},
    {22.3284f, 114.3596f},
    {22.3882f, 114.3190f},
    {22.3917f, 114.2810f},
    {22.3776f, 114.2937f},
    {22.3601f, 114.2608f},
    {22.3027f, 114.3241f},
    {22.2827f, 114.2899f},
    {22.2652f, 114.3051f},
    {22.2687f, 114.2684f},
    {22.3003f, 114.2646f},
    {22.2980f, 114.2481f},
    {22.2851f, 114.2367f},
    {22.3144f, 114.1975f},
    {22.2945f, 114.1709f},
    {22.3027f, 114.1519f},
    {22.3694f, 114.1050f},
    {22.3542f, 114.0240f},
    {22.3776f, 113.9771f},
    {22.3589f, 113.9442f},
    {22.3753f, 113.9126f},
    {22.4010f, 113.9151f},
    {22.4104f, 113.8986f}, // close
};

// --- 大嶼山 & VHHH（閉合）---
static const GeoPt P_LANTAU_VHHH[] = {
    {22.2305f, 113.8363f},
    {22.2891f, 113.8946f},
    {22.2897f, 113.9171f},
    {22.2813f, 113.9333f},
    {22.2917f, 113.9368f},
    {22.2969f, 113.8932f},
    {22.3211f, 113.8791f},
    {22.3345f, 113.9183f},
    {22.3177f, 113.9421f},
    {22.2969f, 113.9370f},
    {22.2935f, 113.9426f},
    {22.3051f, 113.9608f},
    {22.3043f, 113.9776f},
    {22.3453f, 114.0467f},
    {22.3384f, 114.0570f},
    {22.3086f, 114.0547f},
    {22.3064f, 114.0369f},
    {22.3090f, 114.0173f},
    {22.2926f, 114.0224f},
    {22.2645f, 114.0201f},
    {22.2736f, 114.0028f},
    {22.2693f, 113.9981f},
    {22.2541f, 114.0131f},
    {22.2429f, 113.9958f},
    {22.2286f, 114.0178f},
    {22.2165f, 114.0024f},
    {22.2157f, 113.9827f},
    {22.2243f, 113.9832f},
    {22.2394f, 113.9720f},
    {22.2209f, 113.9183f},
    {22.2170f, 113.9304f},
    {22.2105f, 113.9304f},
    {22.2083f, 113.9118f},
    {22.2213f, 113.8912f},
    {22.1984f, 113.8496f},
    {22.2305f, 113.8363f}, // close
};

// --- 香港島（閉合）---
static const GeoPt P_HK_ISLAND[] = {
    {22.2804f, 114.1161f},
    {22.2906f, 114.1453f},
    {22.2828f, 114.1805f},
    {22.2937f, 114.2040f},
    {22.2832f, 114.2336f},
    {22.2618f, 114.2575f},
    {22.2439f, 114.2484f},
    {22.2092f, 114.2575f},
    {22.2075f, 114.2444f},
    {22.2408f, 114.2347f},
    {22.2430f, 114.2250f},
    {22.2314f, 114.2296f},
    {22.2310f, 114.2231f},
    {22.2224f, 114.2148f},
    {22.2147f, 114.2231f},
    {22.1980f, 114.2230f},
    {22.1960f, 114.2118f},
    {22.2169f, 114.2104f},
    {22.2103f, 114.1984f},
    {22.2370f, 114.1942f},
    {22.2442f, 114.1836f},
    {22.2292f, 114.1724f},
    {22.2461f, 114.1604f},
    {22.2487f, 114.1330f},
    {22.2804f, 114.1161f}, // close
};

// --- LAMMA 島（閉合）---
static const GeoPt P_LAMMA[] = {
    {22.2409f, 114.1183f},
    {22.2155f, 114.1337f},
    {22.2058f, 114.1274f},
    {22.2097f, 114.1548f},
    {22.1960f, 114.1443f},
    {22.1836f, 114.1471f},
    {22.1915f, 114.1358f},
    {22.1817f, 114.1295f},
    {22.1823f, 114.1154f},
    {22.2175f, 114.1211f},
    {22.2201f, 114.1119f},
    {22.2110f, 114.1049f},
    {22.2344f, 114.1084f},
    {22.2409f, 114.1183f}, // close
};

// --- 深圳 海岸線：蛇口, 寶安機場向北---
static const GeoPt P_SHENZHEN_COAST[] = {
    {22.7349f, 113.7374f},
    {22.5685f, 113.8388f},
    {22.4508f, 113.8845f},
    {22.4906f, 113.9446f},
    {22.5221f, 113.9500f},
    {22.5238f, 114.0074f},
    {22.5075f, 114.0380f}, // not closed, coastline north-to-south
};

// --- 珠海澳門 向北海岸線---
static const GeoPt P_ZHUHAI_MACAU[] = {
    {22.8113f, 113.5910f},
    {22.5773f, 113.7039f},
    {22.3197f, 113.6078f},
    {22.2286f, 113.5709f},
    {22.1981f, 113.5838f},
    {22.1757f, 113.5407f},
    {22.1745f, 113.5788f},
    {22.1322f, 113.5915f},
    {22.0581f, 113.5038f},
    {21.9875f, 113.3758f},
    {22.0310f, 113.2604f},
    {21.9168f, 113.2917f},
    {21.7786f, 113.0528f}, // not closed, coastline north-to-south
};


void hk_map_draw(
    TFT_eSPI &tft,
    float home_lat, float home_lon,
    float range_nm,
    int radar_cx, int radar_cy, int radar_r,
    uint16_t color
) {
    if (range_nm <= 0.1f) return;

    float cos_lat = cosf(home_lat * (float)M_PI / 180.0f);
    float scale = (float)radar_r / range_nm;

    // 先畫地圖，再由 main.cpp 畫 rings/crosshair/sweep/飛機（層次會好啲）
    draw_path(tft, P_NT_KLN, (int)(sizeof(P_NT_KLN) / sizeof(P_NT_KLN[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);

    draw_path(tft, P_LANTAU_VHHH, (int)(sizeof(P_LANTAU_VHHH) / sizeof(P_LANTAU_VHHH[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);

	draw_path(tft, P_HK_ISLAND, (int)(sizeof(P_HK_ISLAND) / sizeof(P_HK_ISLAND[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);

    draw_path(tft, P_LAMMA, (int)(sizeof(P_LAMMA) / sizeof(P_LAMMA[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);
			  
    draw_path(tft, P_SHENZHEN_COAST, (int)(sizeof(P_SHENZHEN_COAST) / sizeof(P_SHENZHEN_COAST[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);
			  
    draw_path(tft, P_ZHUHAI_MACAU, (int)(sizeof(P_ZHUHAI_MACAU) / sizeof(P_ZHUHAI_MACAU[0])),
			home_lat, home_lon, cos_lat, range_nm, scale, radar_cx, radar_cy, color);
}
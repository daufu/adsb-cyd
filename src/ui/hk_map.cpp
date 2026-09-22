#include "hk_map.h"
#include <math.h>

/*
map有2種顯示style: 
Style 1.  [原本] map inside radar circle. 
Style 2.  [自己add] map畫出radar circle外、用滿status bar以下整個螢幕區域(320x220).

流程:
main.cpp: draw_radar() -> hk_map.cpp: hk_map_draw() -> 
1. draw_path() 用 project_if_in_range() 確認是否超出radar圈, 再判斷: 不讓畫到雷達圈外
2. draw_path() 用 project_to_pixel() 確認是否超出範圍, 再判斷: 不讓線碰到status bar 

*/
// === 幾何/投影說明（同 main.cpp 同一種近似）===
// nm_north = (lat - home_lat) * 60
// nm_east  = (lon - home_lon) * 60 * cos(home_lat)
// px = cx + nm_east  * (radar_r / range_nm)
// py = cy - nm_north * (radar_r / range_nm)

struct GeoPt {
    float lat;
    float lon;
};


/* 
Style 1.  [原本] map inside radar circle. 
-計算 nm 距離後，if (dist2 > r2) return false; → 超出 range_nm 的點不投影。
-Uncomment本function: 若用Style 1. 
*/
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


/* 
Style 2.  [自己add] map畫出radar circle外、用滿status bar以下整個螢幕區域(320x220).
-計算 nm 距離後，if (dist2 > r2) return false; → 超出 range_nm 的點不投影.
-map會填滿radar圈外的corners(左右上下), 但不畫進y=0~19 的status bar.
-Uncomment本function: 若用Style 2.
*/
static inline bool project_to_pixel( 
    const GeoPt &p,
    float home_lat, float home_lon,
    float cos_lat,
    float range_nm,  //保留給 soft clip 用      
    float scale,
    int cx, int cy,
    int &x, int &y
) {
	/*
	算目標點在螢幕哪個像素點(x, y)
	北 (North)
			 ^
			 |             * 目標點 (Target)
			 |            / |
			 |           /  |
	  nm_n = |          /   | 
		3浬  |         /    | 還有這段「斜直線」才是真正的距離！
			 |        /     |
			 |       /      |
			 +------+-------+---> 東 (East)
			Home  nm_e = 4浬
	*/
	// 用目標緯度/經度,減Home緯度/經度: 你緯度25度, 目標緯度26度，那dlat就是1 (代表差1度)
	// lat 緯度線(橫線): 測南北位 ; lon 經度線(直線): 測東西位.
    float dlat = p.lat - home_lat;
    float dlon = p.lon - home_lon;

	// 把度數 變成 海浬 (Nautical Miles)
	// why乘60? 地球上緯度差1度，距離約60nm.所以dlat*60.0f就成了"往北/往南走了多少海浬nm_n).
	// cos_lat? 地球圓, 經度線(直線)赤道最寬, 越靠近南北極就縮一起. cos_lat是修正係數, 調整東西方向的距離, 才不會算錯.
    float nm_n = dlat * 60.0f;  
    float nm_e = dlon * 60.0f * cos_lat;

	/*
	- Trangle畢氏定理: 斜邊^2 = 底邊^2 + 高^2
	北邊^2 = nm_n*nm_n = 3*3 = 9 ; 東邊^2 = nm_e * nm_e = 4*4 = 16 ; 
	兩邊平方相加=9+16=25 (即斜邊^2 = 25) -> 就是dist2. Target真正直線距離: 25開方根 = 5
	- CPU計開根號(sqrt)慢!
	幹脆把dist2 = 25 (斜邊^2) 和 max_r^2(max_r * max_r), 直接比較大小:
	25(斜邊^2) 跟 max_r^2比大小 & 5(25開方根) 跟 max_r 比大小, 效果完全一樣!

	功能: 
	允許畫出radar circle外, 用盡status bar下整screen
	Soft 保護: 太遠(>2.5x range)就不畫, 避免極端座標 + 省CPU(這些點本來就很少). 
	- 若目標離你太遠(超過雷達圈2.5倍緩衝區), 就不要浪費CPU算它. 直接回傳false->不畫.
	*/
    float dist2 = nm_n * nm_n + nm_e * nm_e;
    float max_r = range_nm * 2.5f;   //可調2.0-3.0, 越大外圍越多(更多外圍海岸線出現). 
    if (dist2 > max_r * max_r) return false;

	/* 把海浬 變 螢幕像素Pixels:
	1. cx & cy: 螢幕的正中心點(Center X, Center Y).
	2. scale(縮放比例): 用來放大/縮小map. 假設比例是1浬 對應 10個像素，scale就是10.
	3. why x用加，y用減(cy -)?  
	- X軸(東西方向): 螢幕越右數字越大, 所以向東是加(+).
	- Y軸(南北方向): 易混淆! 真實世界 緯度往北是"增加"(赤道0度, 北極6x度); 但螢幕上，座標的0點在最左上方，往下走數字反而變大.
	e.g.
	- 螢幕中心cx,cy=160,120 (mon 320x240); 放大scale=10 (1海浬 = 10個像素點)
	x = 160 + (nm_e * 10) （向東加，向西減）
	y = 120 - (nm_n * 10) （向北減，向南加）
		電腦螢幕 (0,0 在左上角)
		(0,0)  --------> X軸 (向右變大)
		  |
		  |  螢幕中心(160, 120)
		  v 
		 Y軸(向下變大. 注意: 跟真實世界相反)
	
	A. 真實世界: 在正東方 2 浬
	nm_n=0, nm_e=2 (往東 2 浬)
	x = 160 + (2 * 10) = 180
	y = 120 - (0 * 10) = 120
	結果: 180, 120 -> 螢幕中心往右移 20 像素	
	B. 真實世界:在家西方 3 浬 (向西是負)
	nm_n = 0, nm_e = -3 (往西 3 浬)
	x = 160 + (-3 * 10) = 130
	y = 120 - (0 * 10) = 120
	結果: 130, 120 -> 螢幕中心往左移 30 像素
	C. 真實世界: 在家北方 4 浬
	nm_n = 4（往北 4 浬）, nm_e = 0
	x = 160 + (0 * 10) = 160
	y = 120 - (4 * 10) = 120 - 40 = 80 (因往上所以用減)
	結果: 160, 80 -> 螢幕中心往上移 40 像素
	D. 真實世界: 在家南方 2.5 浬 (向南是負)）
	nm_n = -2.5, nm_e = 0
	x = 160 + (0 * 10) = 160
	y = 120 - (-2.5 * 10) = 120 - (-25) = 120 + 25 = 145 (負負得正, 變加)
	結果像素: 160, 145 -> 螢幕中心往下移 25 像素
	E. 東北 / 真實世界: 往東 3 浬, 北 2 浬
	nm_n = 2, nm_e = 3
	x = 160 + (3 * 10) = 190 (往右)
	y = 120 - (2 * 10) = 100 (往上)
	結果像素: 190, 100 -> 落在螢幕的右上角
	F. 西北 / 真實世界: 往西 2 浬，北 3 浬
	nm_n = 3（北）, nm_e = -2 (西是負)
	x = 160 + (-2 * 10) = 140 (往左)
	y = 120 - (3 * 10) = 90 (往上)
	結果像素: 140, 90 -> 落在螢幕的左上角
	G. 西南 / 真實世界: 往西 4 浬, 南 1 浬
	nm_n = -1 (南是負), nm_e = -4 (西是負)
	x = 160 + (-4 * 10) = 120 (往左)
	y = 120 - (-1 * 10) = 120 + 10 = 130 (往下)
	結果像素：(120, 130) -> 落在螢幕的左下角方向
	*/
    x = cx + (int)(nm_e * scale);
    y = cy - (int)(nm_n * scale);
    return true;
}



/*
if (in0 && in1) 才drawLine:
Style 1. map inside radar circle
Style 2. map畫出radar circle外
*/
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

	// 
	// Style 1: [原本] map inside radar circle. 
    for (int i = 1; i < n; i++) {
        int x0, y0, x1, y1;
        bool in0 = project_if_in_range(pts[i - 1], home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x0, y0);
        bool in1 = project_if_in_range(pts[i],     home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x1, y1);

		// Beginner-friendly: 只畫「兩端都入 range」嘅線段(避免畫到雷達圈外)
        if (in0 && in1) {
            tft.drawLine(x0, y0, x1, y1, color);
        }
		
		// (可選)防 watchdog / 讓系統喘氣: 每 32 條線讓出一下 CPU
		if ((i & 31) == 0) yield();
		 
    }
	
	/* 	*/
	
	//
	// Style 2: map畫出radar circle外
    for (int i = 1; i < n; i++) {
        int x0, y0, x1, y1;
        int x0, y0, x1, y1;
        bool in0 = project_to_pixel(pts[i - 1], home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x0, y0);  // 若沒改名就繼續用 project_if_in_range
        bool in1 = project_to_pixel(pts[i],     home_lat, home_lon, cos_lat, range_nm, scale, cx, cy, x1, y1);

		//
        // 允許畫出 circle 外. 但不讓線碰到 status bar (y < 20). 同時rough screen檢查, 免極端座標.
		// if (in0 && in1): 
		// - 線段的2點都 within 2.5 倍的幾何緩衝區, 條件才成立;
		// - 線段其中1點不在 2.5 倍的幾何緩衝區, 條件不成立.
		，而另一個端點落在之外（in1 = false），這整條線段（line a）就會因為條件不成立而完全不被繪製。
        if (in0 && in1) {

			/*			
			// 不超越頂status bar (STATUS_H=20), 也不要畫太離譜
			Even within 2.5倍的幾何緩衝區, 仍有機畫到status bar裡面, so加下面condition:
			1. y0 >= 20 && y1 >= 20(最重要): 當線段的2端點 Y axis都= or <20, 才畫線.
			完全切斷任何畫入頂部 Status Bar (Y < 20)
			2. y0 < 250 && y1 < 250 (螢幕bottom防護): 螢幕高只有240 px (Y : 0~239). 此處放寬至250px緩衝.
			免draw出螢幕下方過遠的線段.
			3. x0 > -50 && x0 < 370 && x1 > -50 && x1 < 370 (左右邊界與防溢位)
			螢幕寬320 px (X: 0 ~ 319). 此處:左放寬到 -50, 右放寬到 370.
			目的: TFT_eSPI具裁剪Clipping功能, 如丟的axis val太離譜(如X axis 達數千, 極大負數), 
			可能繪圖庫整數溢位Integer Overflow, 令螢幕出現奇怪穿過全螢幕斜線(飛線). 
			這行是安全網, 將過度極端的座標filter.
			*/
            if (y0 >= 20 && y1 >= 20 && 
                y0 < 250 && y1 < 250 && 
                x0 > -50 && x0 < 370 && x1 > -50 && x1 < 370) {
                tft.drawLine(x0, y0, x1, y1, color);
            }
        }
		
		// (可選) 防 watchdog / 讓系統喘氣：每 32 條線讓出一下 CPU
		if ((i & 31) == 0) yield();
		 
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

	/* 
	北極 (Latitude 90°) -> 經線全部聚合在一起，距離變成 0！
        \      |      /
         \     |     /   <-- 越往北走，東西方向的 1 度實際距離越短！
          \    |    /
           \   |   /
    赤道 (Latitude 0°) -> 經線最寬 (1度 = 60浬)
	
	EXPLAIN cos_lat
	- 香港北緯22.4, 頭頂上的經度1度，實際走起距離會比赤道短. 須算一"折扣率"修正東西方向距離 -> 是數學上cos
	- CODE包含2步:
	步驟一: 把"人類的度數" 變 "電腦看的弳度"
	home_lat: 你家緯度 HK: 22.4085
	WHY 乘 M_PI / 180.0f? 人類用Degree (圓圈360度). C++三角函數(cosf)必須用"弳度(Radians)" (一圓圈是2 pi弳度)
	數學換算公式: 弳度 = 度數 * pi / 180
	步驟二: 用cosf() 算經度縮放折扣率
	cosf() 是餘弦函數(Cosine). 數學上把緯度丟進Cosine, 會吐出一0 到 1 之間的數字, 代表東西方向縮放比例.
	- 以香港為例
	假設: HOME_LAT = 22.4085 度 ; M_PI大約=3.14159轉成弳度：$22.4085 \times \frac{3.14159}{180} \approx 0.391$ 弳度算 Cosine：$\cos(0.391) \approx \mathbf{0.924}$這個 0.924 是什麼意思？這代表在香港這個緯度，經度差 1 度的實際東西距離，只有赤道的 $92.4\%$ 這麼寬！
	*/
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
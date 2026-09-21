#pragma once

// ═══════════════════════════════════════════════════════════════════
// 測試設定 —— 所有「為了驗證而偏離正式行為」的開關集中在這一個檔
// ═══════════════════════════════════════════════════════════════════
//
// 為什麼要獨立成一個檔：這些開關散在 main.cpp 各處時，很容易帶著某個
// 還沒改回來的設定就正式上線 —— 而且不會報錯，只會安靜地做錯事。
// 集中在這裡之後，出機前只要看這一個檔：全部 false / 正式值就對了。
//
// ── 出機檢查表 ──────────────────────────────────────────────
//   TEST_SMALL_SCAN        = false
//   TEST_FAKE_CAPTURE      = false      （Python 接上後）
//   TEST_SIMULATE_SAMPLE   = false
//   TEST_SIMULATE_FEED     = false
//   TEST_HOMING_EVERY_N    = 10         （確認誤差小之後）
// ─────────────────────────────────────────────────────────────

// ── 掃描範圍 ────────────────────────────────────────────────
// true  = 3x2 共 6 格、間距 400 步，相機走一小圈就結束，適合反覆驗流程
// false = 正式的 10x7 共 70 格、間距 1360/1040（實測值，不要改）
#define TEST_SMALL_SCAN        0

// ── 拍照 ────────────────────────────────────────────────────
// true  = FakeCapture：不等 PC，延遲 TEST_CAPTURE_DELAY_MS 就當作拍好了
// false = SerialCapture：送 CAPTURE:<row>,<col> 等 PC 回 OK
// ★ Python 還沒接上時必須維持 1，否則掃描會一路逾時。
#define TEST_FAKE_CAPTURE      1
#define TEST_CAPTURE_DELAY_MS  300

// ── 感測器模擬 ──────────────────────────────────────────────
// A1／A2 都已接上實體開關，正常情況維持 0。
// 拆掉感測器單獨測運動時才需要開。
#define TEST_SIMULATE_SAMPLE   0   // 1 = 當作頂部永遠有樣本，不看 A1
#define TEST_SIMULATE_FEED     0   // 1 = 不等 A2，改走 TEST_FEED_STEPS
#define TEST_FEED_STEPS        32000UL   // 800 PPR 下的行程中點（全行程 64000）

// ── 排程歸位 ────────────────────────────────────────────────
// 每幾輪重新歸位一次，把累積誤差清掉。測試期設小一點好觀察；
// 確認誤差夠小之後放寬到 10，省時間。
#define TEST_HOMING_EVERY_N    2

// ═══════════════════════════════════════════════════════════════════
// 以下由上面的開關展開，不用改
// ═══════════════════════════════════════════════════════════════════
#if TEST_SMALL_SCAN
  #define CFG_STEPS_PER_COL  400UL
  #define CFG_STEPS_PER_ROW  400UL
  #define CFG_COLS           3
  #define CFG_ROWS           2
#else
  #define CFG_STEPS_PER_COL  1360UL   // 實測值
  #define CFG_STEPS_PER_ROW  1040UL   // 實測值
  #define CFG_COLS           10
  #define CFG_ROWS           7        // 樣本盤實際列數
#endif

/*
 * 樣本推送台 + 相機拍照 整合控制（單顆 Arduino UNO + CNC Shield）
 *
 * 架構：Arduino 負責所有運動與安全邏輯；PC 端 Python 只負責「收到 CAPTURE 就拍一張、回 OK」。
 * 這樣就算 PC 當掉，Arduino 該停的還是會停。
 *
 * 一次只有一顆馬達在動（輸送台動時相機停，相機動時輸送台停），
 * 所以阻塞式發脈波是安全的，不需要改成定時器中斷。
 */

// ═══════════════════════════════════════════
// 腳位定義
// ═══════════════════════════════════════════

// 相機 XY（CNC Shield 固定腳位，不能改）
#define X_STP    2
#define X_DIR    5
#define Y_STP    3
#define Y_DIR    6
#define EN       8    // A4988 致能（LOW = 啟用）。注意：只管相機，管不到 DM542
#define X_LIMIT  9    // END STOPS 的 X+/X- 端子（內部共線）→ C1、C2
#define Y_LIMIT 10    // END STOPS 的 Y+/Y- 端子（內部共線）→ A1//A2、B1//B2

// 輸送台（走 Shield 左上角 Z.STEP / Z.DIR 排針，接到 DM542 的 PUL- / DIR-）
#define CONV_PUL 4
#define CONV_DIR 7

// 各種開關與感測
#define CONV_BOTTOM    11   // 輸送台底部限位 → END STOPS 的 Z+/Z- 端子
#define CONV_TOP       A0   // 輸送台頂部限位 → Abort 排針
#define SAMPLE_PRESENT A1   // 頂部還有沒有樣本 → Hold 排針
#define IR_SENSOR      A2   // 紅外線（樣本到相機下方）→ Resume 排針
#define START_SW       A3   // 自鎖開關（啟動/停止）→ CoolEn 排針

// ═══════════════════════════════════════════
// 訊號極性 —— 這一段一定要照你的實際硬體核對
// true  = 觸發時腳位被拉到 LOW（常開 NO 開關接地、或感測器低電位輸出）
// false = 觸發時腳位是 HIGH（常閉 NC 開關斷開後被 pullup 拉高）
// ═══════════════════════════════════════════
#define X_LIMIT_ACTIVE_LOW       true
#define Y_LIMIT_ACTIVE_LOW       true
#define CONV_BOTTOM_ACTIVE_LOW   false  // 你原本就是 NC 接法（C→GND），斷開才是觸發
#define CONV_TOP_ACTIVE_LOW      true   // ← 新裝的，如果也用 NC 要改成 false
#define SAMPLE_PRESENT_ACTIVE_LOW true   // 觸發 = 有樣本
#define IR_ACTIVE_LOW            true   // 觸發 = 偵測到樣本
#define START_SW_ACTIVE_LOW      true   // 觸發 = 啟動

// 馬達方向（先猜，實機第一次測試務必用手扶著、隨時準備斷電）
#define CONV_DIR_DOWN  HIGH   // 輸送台往下（往相機、往底部）
#define CONV_DIR_UP    LOW

// ═══════════════════════════════════════════
// 運動參數
// ═══════════════════════════════════════════
#define CONV_DELAY_US    800   // 輸送台脈波半週期
#define CAM_DELAY_US     800   // 相機脈波半週期
#define HOME_FAST_US     800   // homing 快速接近
#define HOME_SLOW_US    2500   // homing 慢速二次接近（提高重現性）
#define HOME_BACKOFF     200   // 碰到開關後退開的步數

#define STEPS_PER_COL   1360   // 相機 X 一格
#define STEPS_PER_ROW   1040   // 相機 Y 一列
#define COLS              10
#define ROWS               1   // ★ 你原本由 Python 決定，這裡要填實際列數

// 逾時保護：超過這個步數還沒等到該等的訊號就判定卡料/故障
#define MAX_STEPS_TO_IR      100000L
#define MAX_STEPS_TO_BOTTOM  100000L
#define MAX_STEPS_TO_TOP     200000L
#define MAX_HOMING_STEPS     200000L

#define HOMING_EVERY_N_ROUNDS 10   // 相機每幾輪做一次完整 homing

// ═══════════════════════════════════════════
// 狀態
// ═══════════════════════════════════════════
enum State {
  ST_IDLE,        // 等待自鎖開關
  ST_INIT_HOME,   // 開機/啟動後的初始歸home
  ST_FEED,        // 推樣本到相機下方（等紅外線）
  ST_IMAGE,       // 相機掃描拍照
  ST_TO_BOTTOM,   // 推到底部
  ST_TO_TOP,      // 回到頂部
  ST_NO_SAMPLE,   // 沒樣本了，正常結束
  ST_FAULT        // 故障停機
};

State state = ST_IDLE;
long roundCount = 0;
bool homedThisSession = false;

// ═══════════════════════════════════════════
// 基本工具
// ═══════════════════════════════════════════

bool triggered(int pin, bool activeLow) {
  return activeLow ? (digitalRead(pin) == LOW) : (digitalRead(pin) == HIGH);
}

// 簡單消抖：連續讀到 3 次相同才算數，避免馬達雜訊誤觸發
bool triggeredStable(int pin, bool activeLow) {
  for (int i = 0; i < 3; i++) {
    if (!triggered(pin, activeLow)) return false;
    delayMicroseconds(200);
  }
  return true;
}

void pulse(int stpPin, int delayUs) {
  digitalWrite(stpPin, HIGH);
  delayMicroseconds(delayUs);
  digitalWrite(stpPin, LOW);
  delayMicroseconds(delayUs);
}

void fault(const char* reason) {
  digitalWrite(EN, HIGH);      // 關掉 A4988 輸出
  state = ST_FAULT;
  Serial.print("FAULT:");
  Serial.println(reason);
}

// ═══════════════════════════════════════════
// 輸送台
// ═══════════════════════════════════════════

// 往指定方向走，直到某個訊號觸發為止
// 回傳 true = 正常觸發停止；false = 超過 maxSteps（卡料/故障）
bool convMoveUntil(int dirLevel, int pin, bool activeLow, long maxSteps) {
  digitalWrite(CONV_DIR, dirLevel);
  delayMicroseconds(5);   // DIR 建立時間
  for (long i = 0; i < maxSteps; i++) {
    if (triggeredStable(pin, activeLow)) return true;
    pulse(CONV_PUL, CONV_DELAY_US);
  }
  return false;
}

// 走固定步數（用在 homing 的退開動作）
void convMoveSteps(int dirLevel, long steps, int delayUs) {
  digitalWrite(CONV_DIR, dirLevel);
  delayMicroseconds(5);
  for (long i = 0; i < steps; i++) pulse(CONV_PUL, delayUs);
}

// 輸送台歸home：回到頂部
bool convHome() {
  Serial.println("HOMING:CONV");
  // 已經在頂部的話先退開一點，確保是「碰上去」而不是「本來就壓著」
  if (triggeredStable(CONV_TOP, CONV_TOP_ACTIVE_LOW)) {
    convMoveSteps(CONV_DIR_DOWN, HOME_BACKOFF, HOME_FAST_US);
  }
  if (!convMoveUntil(CONV_DIR_UP, CONV_TOP, CONV_TOP_ACTIVE_LOW, MAX_HOMING_STEPS)) {
    fault("CONV_HOME_TIMEOUT");
    return false;
  }
  // 退開後慢速再碰一次，位置比較準
  convMoveSteps(CONV_DIR_DOWN, HOME_BACKOFF, HOME_FAST_US);
  digitalWrite(CONV_DIR, CONV_DIR_UP);
  delayMicroseconds(5);
  for (long i = 0; i < HOME_BACKOFF * 4L; i++) {
    if (triggeredStable(CONV_TOP, CONV_TOP_ACTIVE_LOW)) return true;
    pulse(CONV_PUL, HOME_SLOW_US);
  }
  fault("CONV_HOME_RETRY_FAILED");
  return false;
}

// ═══════════════════════════════════════════
// 相機 XY
// ═══════════════════════════════════════════

// 走固定步數，每步檢查該軸限位；碰到就急停
// 回傳 true = 正常走完；false = 撞到限位
bool camMove(int stpPin, int dirPin, int dirLevel, long steps,
             int limitPin, bool limitActiveLow, const char* name) {
  digitalWrite(dirPin, dirLevel);
  delayMicroseconds(5);
  for (long i = 0; i < steps; i++) {
    if (triggeredStable(limitPin, limitActiveLow)) {
      Serial.print("LIMIT_HIT:");
      Serial.println(name);
      return false;
    }
    pulse(stpPin, CAM_DELAY_US);
  }
  return true;
}

// 單軸歸home
bool camHomeAxis(int stpPin, int dirPin, int towardLimit, int awayFromLimit,
                 int limitPin, bool limitActiveLow, const char* name) {
  // 本來就壓在開關上 → 先退開
  if (triggeredStable(limitPin, limitActiveLow)) {
    digitalWrite(dirPin, awayFromLimit);
    delayMicroseconds(5);
    for (long i = 0; i < HOME_BACKOFF; i++) pulse(stpPin, HOME_FAST_US);
  }
  // 快速接近
  digitalWrite(dirPin, towardLimit);
  delayMicroseconds(5);
  bool found = false;
  for (long i = 0; i < MAX_HOMING_STEPS; i++) {
    if (triggeredStable(limitPin, limitActiveLow)) { found = true; break; }
    pulse(stpPin, HOME_FAST_US);
  }
  if (!found) { fault(name); return false; }

  // 退開 → 慢速再碰一次
  digitalWrite(dirPin, awayFromLimit);
  delayMicroseconds(5);
  for (long i = 0; i < HOME_BACKOFF; i++) pulse(stpPin, HOME_FAST_US);

  digitalWrite(dirPin, towardLimit);
  delayMicroseconds(5);
  for (long i = 0; i < HOME_BACKOFF * 4L; i++) {
    if (triggeredStable(limitPin, limitActiveLow)) return true;
    pulse(stpPin, HOME_SLOW_US);
  }
  fault(name);
  return false;
}

bool camHome() {
  Serial.println("HOMING:CAM");
  // ★ 方向假設：X 原點在 LOW 方向、Y 原點在 HIGH 方向（照你原本程式的慣例）
  if (!camHomeAxis(X_STP, X_DIR, LOW, HIGH, X_LIMIT, X_LIMIT_ACTIVE_LOW, "CAM_HOME_X")) return false;
  if (!camHomeAxis(Y_STP, Y_DIR, HIGH, LOW, Y_LIMIT, Y_LIMIT_ACTIVE_LOW, "CAM_HOME_Y")) return false;
  Serial.println("HOMED:CAM");
  return true;
}

// 免費的丟步檢查：照步數回到原點後，原點開關應該要是觸發狀態
// 回傳 true = 位置正確
bool camVerifyAtOrigin() {
  bool xOk = triggeredStable(X_LIMIT, X_LIMIT_ACTIVE_LOW);
  bool yOk = triggeredStable(Y_LIMIT, Y_LIMIT_ACTIVE_LOW);
  return xOk && yOk;
}

// 請 PC 拍一張，等 Python 回 OK
// 回傳 false = 逾時（Python 沒回應）
bool capture(int col, int row) {
  Serial.print("CAPTURE:");
  Serial.print(row);
  Serial.print(",");
  Serial.println(col);

  unsigned long t0 = millis();
  String reply = "";
  while (millis() - t0 < 10000UL) {       // 最多等 10 秒
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        reply.trim();
        if (reply == "OK") return true;
        reply = "";
      } else {
        reply += c;
      }
    }
  }
  return false;
}

// 一輪完整掃描：蛇形走位，每個點請 PC 拍一張，最後回原點
bool runImagingCycle() {
  Serial.println("IMAGING:START");

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      if (!capture(col, row)) { fault("CAPTURE_TIMEOUT"); return false; }

      if (col < COLS - 1) {
        // 蛇形：偶數列往 +X，奇數列往 -X
        int dir = (row % 2 == 0) ? HIGH : LOW;
        if (!camMove(X_STP, X_DIR, dir, STEPS_PER_COL,
                     X_LIMIT, X_LIMIT_ACTIVE_LOW, "X")) {
          fault("CAM_X_LIMIT"); return false;
        }
      }
    }
    if (row < ROWS - 1) {
      if (!camMove(Y_STP, Y_DIR, LOW, STEPS_PER_ROW,
                   Y_LIMIT, Y_LIMIT_ACTIVE_LOW, "Y")) {
        fault("CAM_Y_LIMIT"); return false;
      }
    }
  }

  // 依步數回原點
  int lastRow = ROWS - 1;
  if (lastRow % 2 == 0) {   // 停在 +X 端，要往回走
    if (!camMove(X_STP, X_DIR, LOW, STEPS_PER_COL * (long)(COLS - 1),
                 X_LIMIT, X_LIMIT_ACTIVE_LOW, "X")) { fault("CAM_X_LIMIT"); return false; }
  }
  if (ROWS > 1) {
    if (!camMove(Y_STP, Y_DIR, HIGH, STEPS_PER_ROW * (long)(ROWS - 1),
                 Y_LIMIT, Y_LIMIT_ACTIVE_LOW, "Y")) { fault("CAM_Y_LIMIT"); return false; }
  }

  Serial.println("IMAGING:DONE");
  return true;
}

// ═══════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════

void setup() {
  pinMode(X_STP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(CONV_PUL, OUTPUT); pinMode(CONV_DIR, OUTPUT);
  pinMode(EN, OUTPUT);

  pinMode(X_LIMIT,        INPUT_PULLUP);
  pinMode(Y_LIMIT,        INPUT_PULLUP);
  pinMode(CONV_BOTTOM,    INPUT_PULLUP);
  pinMode(CONV_TOP,       INPUT_PULLUP);
  pinMode(SAMPLE_PRESENT, INPUT_PULLUP);
  pinMode(IR_SENSOR,      INPUT_PULLUP);
  pinMode(START_SW,       INPUT_PULLUP);

  digitalWrite(EN, LOW);
  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  // 自鎖開關關掉 → 任何狀態都回到 IDLE
  if (!triggered(START_SW, START_SW_ACTIVE_LOW)) {
    if (state != ST_IDLE) {
      Serial.println("STOPPED");
      state = ST_IDLE;
      homedThisSession = false;
    }
    return;
  }

  switch (state) {

    case ST_IDLE:
      state = ST_INIT_HOME;
      break;

    case ST_INIT_HOME:
      // 開機位置是未知的，一定要先建立基準
      digitalWrite(EN, LOW);
      if (!camHome()) break;
      if (!convHome()) break;
      homedThisSession = true;
      roundCount = 0;
      state = ST_TO_TOP;   // 已在頂部，直接進入「檢查有無樣本」
      break;

    case ST_TO_TOP:
      if (!triggeredStable(SAMPLE_PRESENT, SAMPLE_PRESENT_ACTIVE_LOW)) {
        Serial.println("NO_SAMPLE");
        state = ST_NO_SAMPLE;
        break;
      }
      state = ST_FEED;
      break;

    case ST_FEED:
      Serial.println("FEEDING");
      // 往下推，等紅外線。這階段只看紅外線，不看底部開關
      if (!convMoveUntil(CONV_DIR_DOWN, IR_SENSOR, IR_ACTIVE_LOW, MAX_STEPS_TO_IR)) {
        fault("NO_IR_JAM");   // 推很久都沒偵測到樣本 → 卡料或掉料
        break;
      }
      Serial.println("IR_DETECTED");
      state = ST_IMAGE;
      break;

    case ST_IMAGE:
      if (!runImagingCycle()) break;   // 失敗時 runImagingCycle 內部已經 fault()

      roundCount++;

      // 每 N 輪做完整 homing；其餘每輪做一次免費的原點檢查
      if (roundCount % HOMING_EVERY_N_ROUNDS == 0) {
        Serial.println("SCHEDULED_HOMING");
        if (!camHome()) break;
      } else if (!camVerifyAtOrigin()) {
        // 照步數回到原點了，開關卻沒觸發 → 這輪丟步，當場修正
        Serial.println("WARN:LOST_STEPS");
        if (!camHome()) break;
      }

      state = ST_TO_BOTTOM;
      break;

    case ST_TO_BOTTOM:
      Serial.println("TO_BOTTOM");
      // 這階段只看底部開關，完全不看紅外線 → 樣本還壓在感測器上也不會卡死
      if (!convMoveUntil(CONV_DIR_DOWN, CONV_BOTTOM, CONV_BOTTOM_ACTIVE_LOW, MAX_STEPS_TO_BOTTOM)) {
        fault("NO_BOTTOM_LIMIT");
        break;
      }
      Serial.println("AT_BOTTOM");

      // 回頂部
      Serial.println("RETURN_TOP");
      if (!convMoveUntil(CONV_DIR_UP, CONV_TOP, CONV_TOP_ACTIVE_LOW, MAX_STEPS_TO_TOP)) {
        fault("NO_TOP_LIMIT");
        break;
      }
      Serial.print("ROUND_DONE:");
      Serial.println(roundCount);
      state = ST_TO_TOP;   // 檢查還有沒有樣本，有就跑下一輪
      break;

    case ST_NO_SAMPLE:
      delay(500);   // 待命，等人補料或關掉自鎖開關
      break;

    case ST_FAULT:
      delay(500);   // 停在這裡，要把自鎖開關關掉再開才會重來
      break;
  }
}

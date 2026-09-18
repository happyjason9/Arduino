#define X_DIR 5
#define X_STP 2
#define Y_DIR 6
#define Y_STP 3
#define EN    8

#define STEPS_PER_COL 1360
#define STEPS_PER_ROW 1040
#define COLS 10

void stepMotor(int stpPin, int steps, int delayUs) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(stpPin, LOW);
    delayMicroseconds(delayUs);
  }
}

void stepBoth(int xSteps, int ySteps, int delayUs) {
  int maxSteps = max(xSteps, ySteps);
  for (int i = 0; i < maxSteps; i++) {
    if (i < xSteps) digitalWrite(X_STP, HIGH);
    if (i < ySteps) digitalWrite(Y_STP, HIGH);
    delayMicroseconds(delayUs);
    if (i < xSteps) digitalWrite(X_STP, LOW);
    if (i < ySteps) digitalWrite(Y_STP, LOW);
    delayMicroseconds(delayUs);
  }
}

void setup() {
  pinMode(X_DIR, OUTPUT);
  pinMode(X_STP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_STP, OUTPUT);
  pinMode(EN,    OUTPUT);
  digitalWrite(EN, LOW);
  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("X+:")) {
    int steps = cmd.substring(3).toInt();
    digitalWrite(X_DIR, HIGH);
    stepMotor(X_STP, steps, 800);
  } else if (cmd.startsWith("X-:")) {
    int steps = cmd.substring(3).toInt();
    digitalWrite(X_DIR, LOW);
    stepMotor(X_STP, steps, 800);
  } else if (cmd == "X+") {
    digitalWrite(X_DIR, HIGH);
    stepMotor(X_STP, STEPS_PER_COL, 800);
  } else if (cmd == "X-") {
    digitalWrite(X_DIR, LOW);
    stepMotor(X_STP, STEPS_PER_COL, 800);
  } else if (cmd.startsWith("Y+:")) {
    int steps = cmd.substring(3).toInt();
    digitalWrite(Y_DIR, LOW);
    stepMotor(Y_STP, steps, 800);
  } else if (cmd == "Y+") {
    digitalWrite(Y_DIR, LOW);
    stepMotor(Y_STP, STEPS_PER_ROW, 800);
  } else if (cmd == "HOME") {
    // X 和 Y 同時回原點，方向和步數由 Python 計算後發來
    // 這裡不處理，由 Python 分開發 X-/X+ 和 Y_HOME
  } else if (cmd.startsWith("XHOME:")) {
    int xSteps = cmd.substring(6).toInt();
    if (xSteps > 0) digitalWrite(X_DIR, HIGH);
    else { digitalWrite(X_DIR, LOW); xSteps = -xSteps; }
    stepMotor(X_STP, xSteps, 800);
  } else if (cmd.startsWith("YHOME:")) {
    int ySteps = cmd.substring(6).toInt();
    digitalWrite(Y_DIR, HIGH);
    stepMotor(Y_STP, ySteps, 800);
  }

  Serial.println("OK");
}


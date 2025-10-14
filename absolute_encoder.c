float deg = 45;     // Desired rotation (user input)
float s = 0;        // Built-in encoder counts
int sm1 = 0;        // Built-in encoder channel A
int sm2 = 0;        // Built-in encoder channel B
int r = 0;          // toggle flag for incremental encoder counting

float er = 0;       // PI proportional error
float eri = 0;      // PI integral sum

int t = 0;          // time ms
int t0 = 0;         // memory for start time

int finish = 0;
int rep = 1;

int grayRef = 0;    // reference (binary 0–31)
float absPrev = 0;  // previous absolute angle
int dir = 0;        // +1=CW, -1=CCW, 0=still

const int grayPins[5] = {9, 10, 11, 12, 13}; // MSB..LSB

// ---- Helper: read 5-bit Gray code ----
int readGrayCode() {
  int g = 0;
  for (int i = 0; i < 5; ++i)
    g = (g << 1) | (digitalRead(grayPins[i]) & 0x1);
  return g;
}

// ---- Gray to binary ----
int grayToBinary(int gray) {
  int bin = gray;
  while (gray >>= 1) bin ^= gray;
  return bin;
}

// ---- Convert binary 0–31 to degrees ----
float indexToDeg(int idx) {
  idx = (idx % 32 + 32) % 32;
  return idx * 11.25;
}

// ---- Read absolute encoder (deg) relative to calibration ----
float readAbsoluteEncoderDeg() {
  int raw = readGrayCode();
  int bin = grayToBinary(raw);

  int diffSteps = bin - grayRef;
  if (diffSteps > 16) diffSteps -= 32;
  if (diffSteps < -16) diffSteps += 32;

  return diffSteps * 11.25; // signed displacement
}

// ---- Calibrate current Gray encoder as home ----
void calibrateGrayEncoder() {
  int raw = readGrayCode();
  int bin = grayToBinary(raw);
  grayRef = bin;
  absPrev = 0;
 
}

// ---- Direction update ----
void updateDirection() {
  float absNow = readAbsoluteEncoderDeg();
  float delta = absNow - absPrev;

  if (delta > 180.0) delta -= 360.0;
  if (delta < -180.0) delta += 360.0;

  const float threshold = 2.0; // deg
  if (delta > threshold) dir = +1;
  else if (delta < -threshold) dir = -1;
  else dir = 0;

  absPrev = absNow;
}

// ---- Setup ----
void setup() {
  Serial.begin(250000);
  Serial.println("Enter desired rotation in degrees:");
  while (Serial.available() == 0);
  deg = Serial.readString().toFloat();
  if (deg < 0) analogWrite(3, 255);
  deg = abs(deg);

  for (int i = 0; i < 5; ++i) pinMode(grayPins[i], INPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);

  calibrateGrayEncoder();
}

float kp = 0.6 * 90.0 / 45.0;
float ki = 0.02;

// ---- Main Loop ----
void loop() {
  kp = 0.6 * 90.0 / deg;

  t = millis();
  t0 = t;

  calibrateGrayEncoder(); // recalibrate each repetition

  while (t < t0 + 4000 && rep <= 10) {
    // PI controller every ~10 ms
    if (t % 10 == 0) {
      if (s < deg * 114.0 * 2.0 / 360.0) {
        er = deg - s * 360.0 / 228.0;
        eri += er;
        int pwm = (int)(kp * er + ki * eri);
        pwm = constrain(pwm, 0, 255);
        analogWrite(6, pwm);
      } else {
        analogWrite(6, 0);
        eri = 0;
      }
      delay(1);
    }

    // Built-in encoder counting
    sm1 = digitalRead(7);
    sm2 = digitalRead(8);
    if (sm1 != sm2 && r == 0) { s++; r = 1; }
    if (sm1 == sm2 && r == 1) { s++; r = 0; }

    // Update direction every 100 ms
    static unsigned long lastDirCheck = 0;
    if ((millis() - lastDirCheck > 100)&dir==0) {
      updateDirection();
      lastDirCheck = millis();
    }

    t = millis();
    finish = 1;
  }

  if (finish == 1) {
    delay(500);
    rep++;

    float grayDispDeg = readAbsoluteEncoderDeg();
    float builtinDeg = s * 360.0 / 228.0;
    float errorDeg = abs(grayDispDeg) - builtinDeg;

    Serial.println("===== Rotation Summary =====");
    Serial.print("Gray encoder displacement (deg): ");
    Serial.println(abs(grayDispDeg), 2);
    Serial.print("Built-in encoder displacement (deg): ");
    Serial.println(builtinDeg, 2);
    Serial.print("Error (Gray - Built-in): ");
    Serial.println(errorDeg, 2);
    if (dir == 1) Serial.println("Direction: CW");
    else if (dir == -1) Serial.println("Direction: CCW");
    else Serial.println("Direction: Still");
    Serial.println("============================\n");

    s = 0;
    finish = 0;
    analogWrite(6, 0);
    dir = 0;
  }
}
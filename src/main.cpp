// ----------------------------------------------------
// ライブラリ
// ----------------------------------------------------
#include <Arduino.h>
#include <CAN.h>
#include <HardwareSerial.h> // IM920用

// --- IM920sL ---
// ESP32のUART2 (Serial2) を使用
// RXピン = 25, TXピン = 26
#define IM920_RX_PIN 25
#define IM920_TX_PIN 26
HardwareSerial IM920_SERIAL(2); // UART2を使用

// ----------------------------------------------------
// Robomas (C620) 関連
// ----------------------------------------------------
const int NUM_MOTORS = 8;
const int MOTOR_GROUP_SIZE = 4;
const int PID_INTERVAL_MS = 50;
const int16_t OUTPUT_LIMIT = 5000;

float Kp = 1;
float Ki = 0.3;
float Kd = 0.0;

float integral[NUM_MOTORS] = {0};
float prev_error[NUM_MOTORS] = {0};
float prev_error2[NUM_MOTORS] = {0};
float target_vel[NUM_MOTORS] = {0}; // rpm

volatile int16_t pos[NUM_MOTORS] = {0};
volatile int16_t current_vel[NUM_MOTORS] = {0};
int16_t motorspeedArray[NUM_MOTORS] = {0};
volatile bool data_size_error_flag_ = false;

// ----------------------------------------------------
// IM920で受信するデータ構造体
// ----------------------------------------------------
struct DS3Data {
  uint32_t buttons; // 4バイト (PSボタン対応)
  uint8_t leftStickX; // 1バイト
  uint8_t leftStickY; // 1バイト
  uint8_t rightStickX; // 1バイト
  uint8_t rightStickY; // 1バイト
};

DS3Data ds3Data; // 受信データを格納するグローバル変数

// ----------------------------------------------------
// IM920で受信するボタン定義 (Arduino送信側と統一)
// ----------------------------------------------------
#define TRIANGLE 0x0001
#define CIRCLE   0x0002
#define CROSS    0x0004
#define SQUARE   0x0008
#define UP       0x0010
#define RIGHT    0x0020
#define DOWN     0x0040
#define LEFT     0x0080
#define L2       0x0100
#define R2       0x0200
#define L1       0x0400
#define R1       0x0800
#define L3       0x1000
#define R3       0x2000
#define SELECT   0x4000
#define START    0x8000
#define PS       0x10000 // 17ビット目

// ----------------------------------------------------
// モーター制御ロジックが使用するグローバル変数
// ----------------------------------------------------
bool button_circle = false;
bool button_cross = false;
bool button_square = false;
bool button_triangle = false;
bool button_L1 = false;
bool button_R1 = false;
bool button_L2 = false;
bool button_R2 = false;
bool button_share = false; // PS3のSELECT
bool button_options = false; // PS3のSTART
bool button_L3 = false;
bool button_R3 = false;
bool button_PS = false;
bool button_touchpad = false; // PS3/IM920では使用しない
bool button_up = false;
bool button_down = false;
bool button_left = false;
bool button_right = false;

int8_t left_stick_x = 0;
int8_t left_stick_y = 0;
int8_t right_stick_x = 0;
int8_t right_stick_y = 0;
uint8_t L2_Value = 0;
uint8_t R2_Value = 0;
// ----------------------------------------------------

// --- プロトタイプ宣言 ---
void Robomasinit();

void RobomasRotate(int16_t *speeds);

void OmniControl(uint8_t deadzone, float max_rpm, float rot_rpm);

int16_t pid_control(uint8_t index, float target, int16_t current);

int16_t pid_control_vel(uint8_t index, float target, int16_t current);

void can_callback(int packetSize);

void debug_output();

void setMotor(int pwm_ch, int dir_pin, int speed);

// IM920受信用のヘルパー関数
void updateControllerState();

void printDS3Data(); // ボタンデバッグ表示用の関数


// === 電磁弁接続ピン ===
const int valvePin = 2;
// === モータードライバ接続ピン ===
const int PWM1_PIN = 14; // 吸盤前後
const int DIR1_PIN = 18;
const int PWM2_PIN = 15; // 吸盤前後
const int DIR2_PIN = 19;
const int PWM3_PIN = 13; // 吸盤吸引
const int DIR3_PIN = 17;
const int PWM4_PIN = 12; // 吸盤吸引 (リミットスイッチと重複注意)
const int DIR4_PIN = 16;
const int PWM5_PIN = 4; // 吸盤昇降
const int DIR5_PIN = 5;

// === PWM設定 ===
const int PWM_FREQ = 20000; // 20kHz
const int PWM_RES = 10; // 10bit (0-1023)
const int PWM1_CH = 0;
const int PWM2_CH = 1;
const int PWM3_CH = 2;
const int PWM4_CH = 3;
const int PWM5_CH = 4;
// === 出力設定 ===
const int MOTOR_POWER = 200; // 0～1023

// --- その他グローバル変数 ---
bool prev_circle = false;
bool prev_square = false;
bool circlePressed = false;
bool squarePressed = false;
bool highSpeed = false;
bool prev_options = false;

// === リミットスイッチピン設定 ===
const int LIMIT_TOP_PIN = 27; // 上限スイッチ
const int LIMIT_BOTTOM_PIN = 12; // 下限スイッチ

// ----------------------------------------------------
// setup()
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 - IM920 受信機 (ECIO高速化 / CAN有効) 起動");

  // --- IM920 (UART2) 初期化 ---
  // ▼▼▼ ECIOモード (19200bps) で通信 ▼▼▼
  IM920_SERIAL.begin(19200, SERIAL_8N1, IM920_RX_PIN, IM920_TX_PIN);
  Serial.println("IM920sL (Serial2) 19200bps 準備完了");

  // --- CAN初期化 ---
  CAN.setPins(22, 23);
  Robomasinit();
  CAN.onReceive(can_callback);

  // --- ピン初期化 ---
  pinMode(valvePin, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT); // (PWMではなくGPIOとして使用)

  // リミットスイッチ初期化 (INPUT_PULLUP)
  pinMode(LIMIT_TOP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);

  // --- PWM初期化 ---
  ledcSetup(PWM1_CH, PWM_FREQ, PWM_RES);
  ledcSetup(PWM2_CH, PWM_FREQ, PWM_RES);
  // ledcSetup(PWM3_CH, PWM_FREQ, PWM_RES); // GPIOとして使用
  // ledcSetup(PWM4_CH, PWM_FREQ, PWM_RES); // コメントアウト
  ledcSetup(PWM5_CH, PWM_FREQ, PWM_RES);

  ledcAttachPin(PWM1_PIN, PWM1_CH);
  ledcAttachPin(PWM2_PIN, PWM2_CH);
  // ledcAttachPin(PWM3_PIN, PWM3_CH); // GPIOとして使用
  // ledcAttachPin(PWM4_PIN, PWM4_CH); // コメントアウト
  ledcAttachPin(PWM5_PIN, PWM5_CH);
}

// ----------------------------------------------------
// loop()
// ----------------------------------------------------
void loop() {
  // ▼▼▼ IM920 受信処理 (ECIO / バイナリ受信) ▼▼▼
  if (IM920_SERIAL.available()) {
    // 1. ヘッダ部 (コロンまで) を読み飛ばす (例: "00,1234,AB:")
    String header = IM920_SERIAL.readStringUntil(':');
    if (header.length() == 0) {
      // タイムアウトやデータ破損の場合、ループの最初に戻る
    } else {
      // 2. 8バイトの生データ (ds3Dataのサイズ) を待つ
      if (IM920_SERIAL.available() >= sizeof(DS3Data)) {
        // 3. 構造体に直接バイナリデータを読み込む
        IM920_SERIAL.readBytes((uint8_t *) &ds3Data, sizeof(DS3Data));

        // 4. 終端の \r\n を読み飛ばす
        while (IM920_SERIAL.available() && IM920_SERIAL.peek() != '\n') {
          IM920_SERIAL.read();
        }
        if (IM920_SERIAL.available()) {
          IM920_SERIAL.read(); // '\n' を消費
        }

        // 5. データをグローバル変数にマッピング
        updateControllerState();

        // 6. デバッグ表示 (ボタン・スティック)
        printDS3Data();
      } else {
        // データが8バイト未満だった (エラー)
        Serial.println("Payload size error (expected 8 bytes)");
        // バッファをクリア
        while (IM920_SERIAL.available()) IM920_SERIAL.read();
      }
    }
  }
  // --- IM920 受信処理 ここまで ---


  // --- 以下、既存のモーター制御・PID制御ロジック (変更なし) ---
  static unsigned long last_pid_time = millis();
  unsigned long now = millis();

  // === スピードモード制御 ===
  if (button_options && !prev_options) {
    highSpeed = !highSpeed;
  }
  prev_options = button_options;

  float max_rpm = highSpeed ? 5000 : 2000;
  float rot_rpm = highSpeed ? 4000 : 2000;

  OmniControl(10, max_rpm, rot_rpm);

  // ---電磁弁制御---
  if (button_R1) {
    digitalWrite(valvePin, HIGH);
  }
  if (button_R2) {
    digitalWrite(valvePin, LOW);
  }

  // --- 昇降モーター制御 (PWM5_CH) ---
  bool limitTop = (digitalRead(LIMIT_TOP_PIN) == LOW);
  bool limitBottom = (digitalRead(LIMIT_BOTTOM_PIN) == HIGH);

  if (button_triangle && limitTop) {
    // 上限未到達時のみ上昇
    setMotor(PWM5_CH, DIR5_PIN, -800);
  } else if (button_cross && limitBottom) {
    // 下限未到達時のみ下降
    setMotor(PWM5_CH, DIR5_PIN, 800);
  } else {
    setMotor(PWM5_CH, DIR5_PIN, 0);
  }

  // === モーター1, 2 (L1=正転, L2=逆転) ===
  if (button_L2) {
    setMotor(PWM1_CH, DIR1_PIN, MOTOR_POWER * 1.5);
    setMotor(PWM2_CH, DIR2_PIN, MOTOR_POWER * 1.5);
  } else if (button_L1) {
    setMotor(PWM2_CH, DIR2_PIN, -MOTOR_POWER * 2);
    setMotor(PWM1_CH, DIR1_PIN, -MOTOR_POWER * 2);
  } else {
    setMotor(PWM1_CH, DIR1_PIN, 0);
    setMotor(PWM2_CH, DIR2_PIN, 0);
  }

  // --- PID制御 ---
  if (now - last_pid_time >= PID_INTERVAL_MS) {
    last_pid_time = now;
    for (int i = 0; i < NUM_MOTORS; i++) {
      motorspeedArray[i] = pid_control(i, target_vel[i], current_vel[i]);
    }
    RobomasRotate(motorspeedArray);
  }

  // --- ボタン○ (トグル) ---
  if (button_circle && !prev_circle) {
    circlePressed = !circlePressed;
    if (circlePressed) {
      digitalWrite(PWM3_PIN, HIGH); // PWMピンをGPIO HIGHとして使用
    } else {
      digitalWrite(PWM3_PIN, LOW);
    }
  }
  prev_circle = button_circle;

  // --- ボタン□ (トグル) ---
  if (button_square && !prev_square) {
    squarePressed = !squarePressed;
    if (squarePressed) {
      digitalWrite(DIR3_PIN, HIGH);
    } else {
      digitalWrite(DIR3_PIN, LOW);
    }
  }
  prev_square = button_square;
}


// ----------------------------------------------------
// IM920受信 -> グローバル変数へのマッピング関数
// (Y軸スティックのオーバーフロー対策済み)
// ----------------------------------------------------
void updateControllerState() {
  // 1. ボタンをマッピング
  uint32_t btn = ds3Data.buttons; // IM920から受信したボタンデータ

  button_circle = (btn & CIRCLE);
  button_cross = (btn & CROSS);
  button_square = (btn & SQUARE);
  button_triangle = (btn & TRIANGLE);
  button_L1 = (btn & L1);
  button_R1 = (btn & R1);
  button_L2 = (btn & L2);
  button_R2 = (btn & R2);
  button_share = (btn & SELECT); // PS3 Select -> 'share'
  button_options = (btn & START); // PS3 Start -> 'options'
  button_L3 = (btn & L3);
  button_R3 = (btn & R3);
  button_PS = (btn & PS);
  button_touchpad = false; // PS3/IM920にはない
  button_up = (btn & UP);
  button_down = (btn & DOWN);
  button_left = (btn & LEFT);
  button_right = (btn & RIGHT);

  // 2. スティックをマッピング (0-255 -> -127 to 127)

  // X軸 (反転なし)
  left_stick_x = (int8_t) (ds3Data.leftStickX - 128);
  right_stick_x = (int8_t) (ds3Data.rightStickX - 128);

  // Y軸 (反転あり、オーバーフロー対策)
  int16_t temp_ly = -((int16_t) ds3Data.leftStickY - 128);
  if (temp_ly > 127) {
    temp_ly = 127; // +128 を +127 に丸める
  }
  left_stick_y = (int8_t) temp_ly;

  int16_t temp_ry = -((int16_t) ds3Data.rightStickY - 128);
  if (temp_ry > 127) {
    temp_ry = 127; // +128 を +127 に丸める
  }
  right_stick_y = (int8_t) temp_ry;

  // 3. L2/R2アナログ値
  L2_Value = (button_L2) ? 255 : 0;
  R2_Value = (button_R2) ? 255 : 0;
}


// ----------------------------------------------------
// (HEX解析ヘルパー関数はECIOモードでは不要)
// ----------------------------------------------------


// ----------------------------------------------------
// ★ ボタンデバッグ表示用の関数 ★
// ----------------------------------------------------
void printDS3Data() {
  // アナログスティックの値 (0-255 の生データ)
  Serial.print("LX:");
  Serial.print(ds3Data.leftStickX);
  Serial.print(" LY:");
  Serial.print(ds3Data.leftStickY);
  Serial.print(" RX:");
  Serial.print(ds3Data.rightStickX);
  Serial.print(" RY:");
  Serial.print(ds3Data.rightStickY);
  Serial.print(" | ");

  // ボタン（押されているものだけ表示）
  uint32_t btn = ds3Data.buttons;

  if (btn & TRIANGLE) Serial.print("TRI ");
  if (btn & CIRCLE) Serial.print("CIR ");
  if (btn & CROSS) Serial.print("CRO ");
  if (btn & SQUARE) Serial.print("SQU ");
  if (btn & UP) Serial.print("UP ");
  if (btn & DOWN) Serial.print("DN ");
  if (btn & LEFT) Serial.print("LT ");
  if (btn & RIGHT) Serial.print("RT ");
  if (btn & L1) Serial.print("L1 ");
  if (btn & R1) Serial.print("R1 ");
  if (btn & L2) Serial.print("L2 ");
  if (btn & R2) Serial.print("R2 ");
  if (btn & L3) Serial.print("L3 ");
  if (btn & R3) Serial.print("R3 ");
  if (btn & SELECT) Serial.print("SEL ");
  if (btn & START) Serial.print("STA ");
  if (btn & PS) Serial.print("PS_BTN ");

  Serial.println();
}


// ----------------------------------------------------
// 既存のモーター制御/CAN/PID関数 (変更なし)
// ----------------------------------------------------

void setMotor(int pwm_ch, int dir_pin, int speed) {
  if (speed > 0) {
    digitalWrite(dir_pin, HIGH); // 正転
    ledcWrite(pwm_ch, speed);
  } else if (speed < 0) {
    digitalWrite(dir_pin, LOW); // 逆転
    ledcWrite(pwm_ch, -speed);
  } else {
    ledcWrite(pwm_ch, 0); // 停止
  }
}

void debug_output() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(motorspeedArray[i]);
    Serial.print(i == NUM_MOTORS - 1 ? '\n' : ',');
  }
  Serial.print("current_vel:");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(current_vel[i]);
    Serial.print(i == NUM_MOTORS - 1 ? '\n' : ',');
  }
  Serial.print("target_vel:");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print(target_vel[i]);
    Serial.print(i == NUM_MOTORS - 1 ? '\n' : ',');
  }
}

int16_t pid_control(uint8_t index, float target, int16_t current) {
  float error = target - current;
  integral[index] += error;
  float derivative = error - prev_error[index];
  prev_error[index] = error;
  float output = Kp * error + Ki * integral[index] + Kd * derivative;
  output = constrain(output, -1 * OUTPUT_LIMIT, OUTPUT_LIMIT);
  return static_cast<int16_t>(output);
}

int16_t pid_control_vel(uint8_t index, float target, int16_t current) {
  float error = target - current;
  float deltaU = Kp * (error - prev_error[index])
                 + Ki * error
                 + Kd * (error - 2 * prev_error[index] + prev_error2[index]);
  motorspeedArray[index] += deltaU;
  motorspeedArray[index] = constrain(motorspeedArray[index], -OUTPUT_LIMIT, OUTPUT_LIMIT);
  prev_error2[index] = prev_error[index];
  prev_error[index] = error;
  return motorspeedArray[index];
}

void can_callback(int packetSize) {
  if (packetSize != 8) {
    data_size_error_flag_ = true;
    return;
  }
  uint16_t packetId = CAN.packetId();
  int motor_index = -1;
  motor_index = packetId - 0x200;
  if (motor_index < 0 || motor_index >= NUM_MOTORS) return;
  uint8_t data[8];
  for (int i = 0; i < 8 && CAN.available(); i++) {
    data[i] = CAN.read();
  }
  pos[motor_index - 1] = static_cast<int16_t>((data[0] << 8) | data[1]);
  current_vel[motor_index - 1] = static_cast<int16_t>((data[2] << 8) | data[3]);
}

void Robomasinit() {
  if (!CAN.begin(1000000)) {
    while (1);
  }
  volatile uint32_t *pREG_IER = (volatile uint32_t *) 0x3ff6b010;
  *pREG_IER &= ~(uint8_t) 0x10;
}

void RobomasRotate(int16_t *speeds) {
  CAN.beginPacket(0x200, 8);
  for (int i = 0; i < MOTOR_GROUP_SIZE; i++) {
    int16_t spd = constrain(speeds[i], -OUTPUT_LIMIT, OUTPUT_LIMIT);
    CAN.write(highByte(spd));
    CAN.write(lowByte(spd));
  }
  CAN.endPacket();

  CAN.beginPacket(0x1FF, 8);
  for (int i = MOTOR_GROUP_SIZE; i < NUM_MOTORS; i++) {
    int16_t spd = constrain(speeds[i], -OUTPUT_LIMIT, OUTPUT_LIMIT);
    CAN.write(highByte(spd));
    CAN.write(lowByte(spd));
  }
  CAN.endPacket();
}

void OmniControl(uint8_t deadzone, float max_rpm, float rot_rpm) {
  const float STICK_MAX = 127.0f;
  int lx = left_stick_x;
  int ly = left_stick_y;
  if (abs(lx) < deadzone) lx = 0;
  if (abs(ly) < deadzone) ly = 0;

  if (button_up) {
    ly = 127;
  } else if (button_down) {
    ly = -127;
  } else if (button_left) {
    lx = -127;
  } else if (button_right) {
    lx = 127;
  }

  float Vx = ((float) lx / STICK_MAX) * max_rpm;
  float Vy = ((float) ly / STICK_MAX) * max_rpm;

  int rx = right_stick_x;
  if (abs(rx) < deadzone) rx = 0;
  float omega = ((float) rx / STICK_MAX) * rot_rpm;

  float vFL = Vy + Vx + omega; // 前左
  float vFR = Vy - Vx - omega; // 前右
  float vRL = Vy - Vx + omega; // 後左
  float vRR = Vy + Vx - omega; // 後右

  target_vel[2] = -vFL;
  target_vel[1] = vFR;
  target_vel[3] = -vRL;
  target_vel[0] = vRR;
}

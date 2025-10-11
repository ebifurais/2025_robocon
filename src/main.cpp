#include <Arduino.h>
#include <CAN.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#define USE_PS3
#ifdef USE_PS4
#include <PS4Controller.h>
#endif

#ifdef USE_PS3
#include <Ps3Controller.h>
#endif

//08:d1:f9:37:08:3e

//for c620>
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
//forc620<
//for ps4

bool button_circle = false;
bool button_cross = false;
bool button_square = false;
bool button_triangle = false;
bool button_L1 = false;
bool button_R1 = false;
bool button_L2 = false;
bool button_R2 = false;
bool button_share = false;
bool button_options = false;
bool button_L3 = false;
bool button_R3 = false;
bool button_PS = false;
bool button_touchpad = false;
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
//for ps4>

//prtotipe_defines
void Robomasinit();

void RobomasRotate(int16_t *speeds);

void OmniControl(uint8_t deadzone, float max_rpm, float rot_rpm);

int16_t pid_control(uint8_t index, float target, int16_t current);

int16_t pid_control_vel(uint8_t index, float target, int16_t current);

void can_callback(int packetSize);

void debug_output();

void recive_PS4();

void recive_PS3();

// === 電磁弁接続ピン ===
const int valvePin = 2;
// === モータードライバ接続ピン ===
// 吸盤前後
const int PWM1_PIN = 14;
const int DIR1_PIN = 18;

// 吸盤前後
const int PWM2_PIN = 15;
const int DIR2_PIN = 19;

//吸盤吸引
const int PWM3_PIN = 13;
const int DIR3_PIN = 17;
//吸盤吸引
const int PWM4_PIN = 12;
const int DIR4_PIN = 16;
//吸盤昇降
const int PWM5_PIN = 4;
const int DIR5_PIN = 5;

// === PWM設定 ===
const int PWM_FREQ = 20000; // 20kHz (Cytron推奨)
const int PWM_RES = 10; // 10bit分解能 (0-1023)

// チャンネル割当
const int PWM1_CH = 0;
const int PWM2_CH = 1;
const int PWM3_CH = 2;
const int PWM4_CH = 3;
const int PWM5_CH = 4;
// === 出力設定 ===
const int MOTOR_POWER = 200; // 出力強さ (0～1023)

// --- モータ制御関数 ---
void setMotor(int pwm_ch, int dir_pin, int speed) {
  // Serial.printf("%d, %d, %d\n", pwm_ch, dir_pin, speed);
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

// 追加で必要なグローバル変数
bool prev_circle = false;
bool prev_square = false;
bool circlePressed = false;
bool squarePressed = false;
bool highSpeed = false;
bool prev_options = false;
// === リミットスイッチピン設定 ===
const int LIMIT_TOP_PIN = 27; // 上限スイッチ
const int LIMIT_BOTTOM_PIN = 12; // 下限スイッチ

void setup() {
  Serial.begin(115200);
  delay(100);
  CAN.setPins(22, 23);
  Robomasinit();
  CAN.onReceive(can_callback);
  //電磁弁初期化
  pinMode(valvePin, OUTPUT);
  // DIRピン初期化
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  // リミットスイッチ初期化
  pinMode(LIMIT_TOP_PIN, INPUT_PULLUP);
  pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);

  // PWM初期化
  ledcSetup(PWM1_CH, PWM_FREQ, PWM_RES);
  ledcSetup(PWM2_CH, PWM_FREQ, PWM_RES);
  //ledcSetup(PWM3_CH, PWM_FREQ, PWM_RES);
  //ledcSetup(PWM4_CH, PWM_FREQ, PWM_RES);
  ledcSetup(PWM5_CH, PWM_FREQ, PWM_RES);

  ledcAttachPin(PWM1_PIN, PWM1_CH);
  ledcAttachPin(PWM2_PIN, PWM2_CH);
  //ledcAttachPin(PWM3_PIN, PWM3_CH);
  //ledcAttachPin(PWM4_PIN, PWM4_CH);
  ledcAttachPin(PWM5_PIN, PWM5_CH);

#ifdef USE_PS4
  PS4.begin();
#endif

#ifdef USE_PS3
  Ps3.begin();
#endif
}

void loop() {
#ifdef USE_PS4
  recive_PS4();
#endif

#ifdef USE_PS3
  recive_PS3();
#endif

  static unsigned long last_pid_time = millis();
  unsigned long now = millis();

  // === スピードモード制御 ===

  // STARTボタンが押された瞬間にトグル
  if (button_options && !prev_options) {
    highSpeed = !highSpeed;
  }
  prev_options = button_options;

  // モードに応じて速度設定
  float max_rpm = highSpeed ? 5000 : 1000; // 並進速度
  float rot_rpm = highSpeed ? 1000 : 500; // 旋回速度

  OmniControl(10, max_rpm, rot_rpm);
  // ---電磁弁制御---
  if (button_R1) {
    digitalWrite(valvePin, HIGH);
  }
  if (button_R2) {
    digitalWrite(valvePin, LOW);
  }
  // --- 5番目モーターを△×ボタンで制御 ---
  // --- 昇降モーター制御 (PWM5_CH) ---
  bool limitTop = (digitalRead(LIMIT_TOP_PIN) == LOW); // 上限押された？
  bool limitBottom = (digitalRead(LIMIT_BOTTOM_PIN) == HIGH); // 下限押された？

  if (button_triangle && limitTop) {
    // △押下で上昇（上限で停止）
    setMotor(PWM5_CH, DIR5_PIN, 800);
  } else if (button_cross && limitBottom) {
    // ×押下で下降（下限で停止）
    setMotor(PWM5_CH, DIR5_PIN, -800);
  } else {
    // どちらも押されていない or リミット到達 → 停止
    setMotor(PWM5_CH, DIR5_PIN, 0);
  }
  // === モーター1 (L1=正転, L2=逆転) ===
  if (button_L2) {
    setMotor(PWM1_CH, DIR1_PIN, MOTOR_POWER * 1.5);
    setMotor(PWM2_CH, DIR2_PIN, -MOTOR_POWER * 1.5);
  } else if (button_L1) {
    setMotor(PWM2_CH, DIR2_PIN, MOTOR_POWER * 2);
    setMotor(PWM1_CH, DIR1_PIN, -MOTOR_POWER * 2);
  } else {
    setMotor(PWM1_CH, DIR1_PIN, 0);
    setMotor(PWM2_CH, DIR2_PIN, 0);
  }

  if (now - last_pid_time >= PID_INTERVAL_MS) {
    last_pid_time = now;
    for (int i = 0; i < NUM_MOTORS; i++) {
      motorspeedArray[i] = pid_control(i, target_vel[i], current_vel[i]);
    }

    RobomasRotate(motorspeedArray);
  }

  // --- ボタン○ ---
  if (button_circle && !prev_circle) {
    // ボタンが押された瞬間
    circlePressed = !circlePressed; // 状態をトグル
    if (circlePressed) {
      digitalWrite(PWM3_PIN, HIGH);
      //setMotor(PWM3_CH, DIR3_PIN, 1023);
    } else {
      digitalWrite(PWM3_PIN, LOW);
      //setMotor(PWM3_CH, DIR3_PIN, 0);
    }
  }
  prev_circle = button_circle; // 状態を更新

  // --- ボタン△ ---
  if (button_square && !prev_square) {
    // ボタンが押された瞬間
    squarePressed = !squarePressed; // 状態をトグル
    if (squarePressed) {
      digitalWrite(DIR3_PIN, HIGH);
      //setMotor(PWM4_CH, DIR4_PIN, 1023);
    } else {
      digitalWrite(DIR3_PIN, LOW);
      //setMotor(PWM4_CH, DIR4_PIN, 0);
    }
  }
  prev_square = button_square; // 状態を更新
}


void debug_output() {
  // デバッグ出力
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
  Serial.print("target_vel:");
  Serial.println(target_vel[0]);
  Serial.print("current_vel:");
  Serial.println(current_vel[0]);
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

  // Δu を計算
  float deltaU = Kp * (error - prev_error[index])
                 + Ki * error
                 + Kd * (error - 2 * prev_error[index] + prev_error2[index]);

  // 出力を更新
  motorspeedArray[index] += deltaU;

  // 制約をかける
  motorspeedArray[index] = constrain(motorspeedArray[index], -OUTPUT_LIMIT, OUTPUT_LIMIT);

  // 誤差を更新
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
    while (1); // 通信失敗時は無限ループ
  }

  // 不要な割り込み無効化（ESP32の場合）
  volatile uint32_t *pREG_IER = (volatile uint32_t *) 0x3ff6b010;
  *pREG_IER &= ~(uint8_t) 0x10;
}

void RobomasRotate(int16_t *speeds) {
  // 上位モータ群（0～3）
  CAN.beginPacket(0x200, 8);
  for (int i = 0; i < MOTOR_GROUP_SIZE; i++) {
    int16_t spd = constrain(speeds[i], -OUTPUT_LIMIT, OUTPUT_LIMIT);
    CAN.write(highByte(spd));
    CAN.write(lowByte(spd));
  }
  CAN.endPacket();

  // 下位モータ群（4～7）
  CAN.beginPacket(0x1FF, 8);
  for (int i = MOTOR_GROUP_SIZE; i < NUM_MOTORS; i++) {
    int16_t spd = constrain(speeds[i], -OUTPUT_LIMIT, OUTPUT_LIMIT);
    CAN.write(highByte(spd));
    CAN.write(lowByte(spd));
  }
  CAN.endPacket();
}


// stick 値からオムニホイール速度を計算して target_vel[] に反映する
void OmniControl(uint8_t deadzone = 10, float max_rpm = 3000, float rot_rpm = 1000) {
  // スティックの最大値をfloat型で定義
  const float STICK_MAX = 127.0f;

  // 左スティック入力（並進）
  int lx = left_stick_x * 0.3;
  int ly = left_stick_y * 0.3;

  // デッドゾーン処理
  if (abs(lx) < deadzone) lx = 0;
  if (abs(ly) < deadzone) ly = 0;

  // --- 十字キーによる並進制御追加 ---
  // 十字キーが押された場合はスティック入力より優先
  if (button_up) {
    ly = 63; // 前進
  } else if (button_down) {
    ly = -63; // 後退
  } else if (button_left) {
    lx = -63; // 左移動
  } else if (button_right) {
    lx = 63; // 右移動
  }

  // map()関数は整数演算のため、範囲外の値(-128)が入力されると予期せぬ動作をします。
  // float型で直接割合を計算することで、この問題を回避し、より正確な制御が可能になります。
  float Vx = ((float) lx / STICK_MAX) * max_rpm;
  float Vy = ((float) ly / STICK_MAX) * max_rpm;

  // 右スティック入力（旋回）
  int rx = right_stick_x;
  if (abs(rx) < deadzone) rx = 0;

  float omega = ((float) rx / STICK_MAX) * rot_rpm;
  // Serial.print("Vx:");
  // Serial.print(Vx);
  // Serial.print(" Vy:");
  // Serial.print(Vy);
  // Serial.print(" omega:");
  // Serial.println(omega);
  // --- オムニ4輪の速度分解 ---
  float vFL = Vy + Vx + omega; // 前左
  float vFR = Vy - Vx - omega; // 前右
  float vRL = Vy - Vx + omega; // 後左
  float vRR = Vy + Vx - omega; // 後右
  //Serial.println(vFL);

  // target_vel に反映（モータ0～3をオムニ用とする）
  target_vel[2] = -vFL;
  target_vel[1] = vFR;
  target_vel[3] = -vRL;
  target_vel[0] = vRR;
}

#ifdef USE_PS4
void recive_PS4() {
  // PS4コントローラの接続状態を確認
  if (PS4.isConnected()) {
    // 接続されている場合、必要に応じてデータを取得・処理
    // 例: ボタンの状態やスティックの位置を取得
    button_circle = PS4.Circle();
    button_cross = PS4.Cross();
    button_square = PS4.Square();
    button_triangle = PS4.Triangle();
    button_L1 = PS4.L1();
    button_R1 = PS4.R1();
    button_L2 = PS4.L2();
    button_R2 = PS4.R2();
    button_share = PS4.Share();
    button_options = PS4.Options();
    button_L3 = PS4.L3();
    button_R3 = PS4.R3();
    button_PS = PS4.PSButton();
    button_touchpad = PS4.Touchpad();
    button_up = PS4.Up();
    button_down = PS4.Down();
    button_left = PS4.Left();
    button_right = PS4.Right();
    left_stick_x = PS4.LStickX();
    left_stick_y = PS4.LStickY();
    right_stick_x = PS4.RStickX();
    right_stick_y = PS4.RStickY();
    L2_Value = PS4.L2Value();
    R2_Value = PS4.R2Value();
  } else {
    // コントローラが接続されていない場合の処理（必要に応じて）
  }
  return;
}
#endif


#ifdef USE_PS3
// --- PS3コントローラ受信関数 ---
void recive_PS3() {
  // PS3コントローラの接続状態を確認
  if (Ps3.isConnected()) {
    // 接続されている場合、必要に応じてデータを取得・処理
    // 例: ボタンの状態やスティックの位置を取得
    button_circle = Ps3.data.button.circle;
    button_cross = Ps3.data.button.cross;
    button_square = Ps3.data.button.square;
    button_triangle = Ps3.data.button.triangle;
    button_L1 = Ps3.data.button.l1;
    button_R1 = Ps3.data.button.r1;
    button_L2 = Ps3.data.button.l2;
    button_R2 = Ps3.data.button.r2;
    button_share = Ps3.data.button.select;
    button_options = Ps3.data.button.start;
    button_L3 = Ps3.data.button.l3;
    button_R3 = Ps3.data.button.r3;
    button_PS = Ps3.data.button.ps;
    // PS3コントローラにはタッチパッドボタンがないため、常にfalseに設定
    button_touchpad = false;
    button_up = Ps3.data.button.up;
    button_down = Ps3.data.button.down;
    button_left = Ps3.data.button.left;
    button_right = Ps3.data.button.right;
    left_stick_x = Ps3.data.analog.stick.lx;
    left_stick_y = -(Ps3.data.analog.stick.ly + (Ps3.data.analog.stick.ly < 0 ? 1 : 0));
    // 0未満の時1を足すことで-128～127を-127～127にして正負反転を安全に
    right_stick_x = Ps3.data.analog.stick.rx;
    right_stick_y = -(Ps3.data.analog.stick.ry + (Ps3.data.analog.stick.ry < 0 ? 1 : 0));
    // 0未満の時1を足すことで-128～127を-127～127にして正負反転を安全に
    L2_Value = Ps3.data.analog.button.l2;
    R2_Value = Ps3.data.analog.button.r2;
  } else {
    // コントローラが接続されていない場合の処理（必要に応じて）
  }
  return;
}
#endif

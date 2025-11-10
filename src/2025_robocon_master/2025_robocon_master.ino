// ----------------------------------------------------
// 必要なライブラリの読み込み
// ----------------------------------------------------
#include <SPI.h>
#include <PS3USB.h> // USB Host Shieldライブラリ
#include <Usb.h>
#include <SoftwareSerial.h> // ソフトウェアシリアル

// ----------------------------------------------------
// グローバル変数の定義
// ----------------------------------------------------

// --- USB Host Shield ---
USB Usb;
PS3USB PS3(&Usb);

// --- IM920sL ---
SoftwareSerial IM920_SERIAL(A4, A5); // ユーザー指定のピン

// ▼▼▼ 修正点 1: BUSYピンをA3に定義 ▼▼▼
const int IM920_BUSY_PIN = A3;

// --- PCデバッグ用 ---
#define DEBUG_SERIAL Serial

// --- 送信するDualShock 3のデータ構造体 (8バイト) ---
struct DS3Data {
  uint32_t buttons; // 4バイト (PSボタン対応)
  uint8_t leftStickX; // 1バイト (0-255)
  uint8_t leftStickY; // 1バイト (0-255)
  uint8_t rightStickX; // 1バイト (0-255)
  uint8_t rightStickY; // 1バイト (0-255)
};

// --- ESP32側（受信機）で定義されているボタンの値 ---
#define ESP32_TRIANGLE 0x0001
#define ESP32_CIRCLE   0x0002
#define ESP32_CROSS    0x0004
#define ESP32_SQUARE   0x0008
#define ESP32_UP       0x0010
#define ESP32_RIGHT    0x0020
#define ESP32_DOWN     0x0040
#define ESP32_LEFT     0x0080
#define ESP32_L2       0x0100
#define ESP32_R2       0x0200
#define ESP32_L1       0x0400
#define ESP32_R1       0x0800
#define ESP32_L3       0x1000
#define ESP32_R3       0x2000
#define ESP32_SELECT   0x4000
#define ESP32_START    0x8000
#define ESP32_PS       0x10000


// ----------------------------------------------------
// setup() 関数
// ----------------------------------------------------
void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Arduino Uno - DS3 送信機 (BUSYピン監視 / HEX) 起動");

  IM920_SERIAL.begin(19200);

  // ▼▼▼ 修正点 2: BUSYピンを INPUT に設定 ▼▼▼
  pinMode(IM920_BUSY_PIN, INPUT);

  if (Usb.Init() == -1) {
    DEBUG_SERIAL.println("USB Host Shield 初期化失敗");
    while (1); // 停止
  }

  delay(1000);
  DEBUG_SERIAL.println("準備完了");
}


// ----------------------------------------------------
// loop() 関数
// ----------------------------------------------------
void loop() {
  Usb.Task();

  // ▼▼▼ 修正点 3: BUSYピンがLOW (準備OK) になるまで待つ ▼▼▼
  // IM920が処理中 (BUSY = HIGH) の場合は、ここで待機する
  // (キャリアセンスや送信処理が終わるのを待つ)
  while (digitalRead(IM920_BUSY_PIN) == HIGH) {
    // BUSYの間も Usb.Task() は呼び続けて接続を維持する
    Usb.Task();
  }
  // ▲▲▲ IM920が「受信可能」な状態になった ▲▲▲


  if (PS3.PS3Connected) {
    // 1. コントローラーの「現在の状態」を構造体に格納
    DS3Data ds3Data;
    uint32_t currentButtons = 0;

    // (ボタンマッピング ... 変更なし)
    if (PS3.getButtonPress(TRIANGLE)) currentButtons |= ESP32_TRIANGLE;
    if (PS3.getButtonPress(CIRCLE)) currentButtons |= ESP32_CIRCLE;
    if (PS3.getButtonPress(CROSS)) currentButtons |= ESP32_CROSS;
    if (PS3.getButtonPress(SQUARE)) currentButtons |= ESP32_SQUARE;
    if (PS3.getButtonPress(UP)) currentButtons |= ESP32_UP;
    if (PS3.getButtonPress(RIGHT)) currentButtons |= ESP32_RIGHT;
    if (PS3.getButtonPress(DOWN)) currentButtons |= ESP32_DOWN;
    if (PS3.getButtonPress(LEFT)) currentButtons |= ESP32_LEFT;
    if (PS3.getButtonPress(L1)) currentButtons |= ESP32_L1;
    if (PS3.getButtonPress(R1)) currentButtons |= ESP32_R1;
    if (PS3.getButtonPress(L2)) currentButtons |= ESP32_L2;
    if (PS3.getButtonPress(R2)) currentButtons |= ESP32_R2;
    if (PS3.getButtonPress(L3)) currentButtons |= ESP32_L3;
    if (PS3.getButtonPress(R3)) currentButtons |= ESP32_R3;
    if (PS3.getButtonPress(SELECT)) currentButtons |= ESP32_SELECT;
    if (PS3.getButtonPress(START)) currentButtons |= ESP32_START;
    if (PS3.getButtonPress(PS)) currentButtons |= ESP32_PS;

    ds3Data.buttons = currentButtons;
    ds3Data.leftStickX = PS3.getAnalogHat(LeftHatX);
    ds3Data.leftStickY = PS3.getAnalogHat(LeftHatY);
    ds3Data.rightStickX = PS3.getAnalogHat(RightHatX);
    ds3Data.rightStickY = PS3.getAnalogHat(RightHatY);

    // 2. 8バイトの構造体(ds3Data)を16文字のHEX文字列に変換
    char hexPayload[17];
    uint8_t *dataPtr = (uint8_t *) &ds3Data;

    for (int i = 0; i < sizeof(DS3Data); i++) {
      sprintf(&hexPayload[i * 2], "%02X", dataPtr[i]);
    }
    hexPayload[16] = '\0'; // 終端


    // 3. TXDAコマンドで送信 (HEXモード)
    IM920_SERIAL.print("TXDA ");
    IM920_SERIAL.print(hexPayload);
    IM920_SERIAL.print("\r\n"); // 終端子 <CR><LF>
    // 4. デバッグ用にPCにも送信内容を表示
    DEBUG_SERIAL.println(hexPayload);

    // 応答（OK/NG）を読み飛ばす
    while (IM920_SERIAL.available()) {
      IM920_SERIAL.read();
    }

    // ▼▼▼ 修正点 4: delay(25) を削除 ▼▼▼
    // (BUSY監視ループが待機の代わりになるため不要)
    // delay(25);
  } else {
    // 未接続時
    DEBUG_SERIAL.println("PS3 コントローラを接続してください...");
    delay(1000);
  }
}

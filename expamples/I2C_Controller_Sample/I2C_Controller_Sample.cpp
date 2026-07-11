#include <Arduino.h>

#include <M5Unified.h>
#include <Wire.h>

// Targetアドレスを 0x20にする
#define ATOM_1_ADDR 0x20

// 本体LED（GPIO27）を制御するシンプルなマクロ
#define LED_GREEN() neopixelWrite(27, 0, 50, 0)  // 緑（明るさ50）
#define LED_BLUE()  neopixelWrite(27, 0, 0, 50)  // 青（明るさ50）
#define LED_OFF()   neopixelWrite(27, 0, 0, 0)   // 消灯

int sendFileNumber = 32; 
uint32_t lastSendMillis = 0;
const uint32_t sendIntervalMs = 800; // 送信間隔（ms）表示側でスライドイン表示が終わるまで待ってから送信    
uint32_t lastLedMillis = 0;
bool ledState = false;

bool sendCommand(uint8_t slaveAddr, uint8_t fileNum) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(0x30);        // コマンドヘッダ
    Wire.write((uint8_t)fileNum); // ファイル番号
    uint8_t error = Wire.endTransmission();
    return (error == 0);
}

void setup(void) {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  // 送信側Atom Liteの背面ソケットからI2C出力 (SDA:25, SCL:21)
  // 送信側Atom LiteのGROVE端子からI2C出力 (SDA:26, SCL:32)
  Wire.begin(26, 32, 400000);
  //pinMode(26, INPUT_PULLUP);      //外部でプルアップしない場合、外部プルアップしたほうが安定
  //pinMode(32, INPUT_PULLUP);

  LED_GREEN(); 
  delay(200);
  LED_OFF(); 

  Serial.println("I2C Ready. Press BtnA to send command.");
}

void loop() {
    M5.update();
    uint32_t now = millis();

    //LEDの1秒周期（500msごと反転）定期点滅処理
    if (now - lastLedMillis >= 500) {
        lastLedMillis = now;
        ledState = !ledState; // 状態を反転
        
        if (ledState) {
            LED_GREEN();
        } else {
            LED_OFF();
        }
    }

    // ボタンが押されている間、600msごとに処理を実行
    if (M5.BtnA.isPressed()) {
        if (now - lastSendMillis >= sendIntervalMs) {
            
            Serial.printf("\n--- Start Transmitting (File: %d) ---\n", sendFileNumber);
            
            // コマンド送信中はLEDを青色に点灯
            LED_BLUE();
            sendCommand(ATOM_1_ADDR, sendFileNumber);
            LED_OFF();

            // 32〜0へのカウントダウンループ
            sendFileNumber--;
            if (sendFileNumber < 0) {
                sendFileNumber = 32;
            }
            lastSendMillis = now;
        }
    }
    delay(10);
}





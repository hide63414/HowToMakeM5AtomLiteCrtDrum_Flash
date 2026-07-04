#include <Arduino.h>

// エディタの自動整形(アルファベット順並び替え)によるエラーを防ぐため
// 以下の順番を固定します
// clang-format off
#include <M5UnitRCA.h>
#include <M5Unified.h>
// clang-format on
#include <LittleFS.h>
#include <Preferences.h>

//33個のbinファイル名（LittleFS用）を定義
const char* vfrFileList[] = {
    "/arpeggioAhi_1minBIN565_165.bin",
    "/arpeggioAlow_1minBIN565_165.bin",
    "/arpeggioBhi_1minBIN565_165.bin",
    "/arpeggioBlow_1minBIN565_165.bin",
    "/dr_hihat_1minBIN565_165.bin",
    "/dr_snare_1minBIN565_165.bin",
    "/dr_kick_1minBIN565_30.bin",
    "/lowpulse1_1minBIN565_30.bin",
    "/lowpulse2_1minBIN565_30.bin",
    "/lowpulse3_1minBIN565_30.bin",
    "/lowpulse4_1minBIN565_30.bin",
    "/lowpulse5_1minBIN565_30.bin",
    "/org1_30secBIN565_165.bin",
    "/org2_30secBIN565_165.bin",
    "/org3_30secBIN565_165.bin",
    "/org4_30secBIN565_165.bin",
    "/org5_30secBIN565_165.bin",
    "/org6_30secBIN565_165.bin",
    "/org7_30secBIN565_165.bin",
    "/org8_30secBIN565_165.bin",
    "/org9_30secBIN565_165.bin",
    "/org10_30secBIN565_165.bin",
    "/tri1_1minBIN565_30.bin",
    "/tri2_1minBIN565_30.bin",
    "/tri3_1minBIN565_30.bin",
    "/tri4_1minBIN565_30.bin",
    "/tri5_1minBIN565_30.bin",
    "/tri6_1minBIN565_30.bin",
    "/tri7_1minBIN565_30.bin",
    "/tri8_1minBIN565_30.bin",
    "/tri9_1minBIN565_30.bin",
    "/tri10_1minBIN565_30.bin",
    "/tri11_1minBIN565_30.bin"
};

const int fileNumberMax = (sizeof(vfrFileList) / sizeof(vfrFileList[0])) - 1;
const int crtImageHeight = 480; // 縦解像度　横縞本数なので480本は変えない
const int crtImageWidth = 120;  // CRT16bit表示するために横144
const int crtFrameSize = crtImageHeight * 2;

File dataFile;          // LittleFS内のファイルを開くためのFileクラス
static M5Canvas canvas; // ブラウン管表示前のスプライト
uint8_t buffer[crtFrameSize]; // 1列分の画像データ　rgb565は1データあたり2バイト　960バイト

int fileNumber = 0;    // 表示ファイル番号　構造体配列の番号
int frame = 0;         // 表示フレームカウント
int totalFrames = 0;          // ★追加：現在のファイルの総フレーム数（自動計算用）
uint32_t fpsCount = 0; // fps表示用
uint32_t fpsSec = 0;   // fps表示用

hw_timer_t *timer = NULL; // ハードウエアタイマ
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;     // ミューテックス
                                      // 変数に同時アクセスしないようにする
volatile SemaphoreHandle_t semaphore; // セマフォ　割込みのタイミング同期に使う
volatile uint32_t counter = 0;        // 割込み回数
uint32_t timerus = 33333;             // フレーム周期

// Preferences for storing last selected file number
Preferences prefs;

bool pendingFileSave = false;
uint32_t lastFileChangeMillis = 0;
bool lastBtnAPressed = false;
bool longPressHandled = false;
bool waitForBootButtonRelease = false;
const uint32_t fileChangeRepeatMs = 150;

bool fileCloseOpen(int fNum);   // 開いているファイルを閉じてfNum番目のファイルを開く
bool readFrameToBuffer();
void showFileNumber(int fNum);  // ファイル番号を表示
void updateScreen();            // スプライトバッファにセットされている画像を表示する
void updateScreenSlidein();     // ファイルから1画面分データを読み込んで左端からスライドイン　ファイル変更時に使う
void setAtomLED(uint16_t c);    // 16bitカラー(RGB565)を解析して内蔵LEDを光らせる

void saveCurrentFileSelection() {
  prefs.putInt("fileNumber", fileNumber);
}

// onTimer(): ハードウェアタイマ割り込み。フレーム同期用カウンタを増やし、セマフォを解放する
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux); // ミューテックスを利用して排他制御
  counter++;
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(semaphore, NULL); // セマフォを開放
}

// setup(): M5UnitRCA と LittleFS を初期化し、保存されたファイル番号を復元して
//             内蔵データファイルを開き、タイマーを開始する
void setup(void) {
  auto cfg = M5.config();
  cfg.external_display.unit_rca = true; // default=true. use UnitRCA VideoOutput
#if defined(__M5GFX_M5UNITRCA__)        // setting for Unit RCA.
  cfg.unit_rca.logical_width = crtImageWidth;
  cfg.unit_rca.logical_height = crtImageHeight;
  cfg.unit_rca.output_width = crtImageWidth;
  cfg.unit_rca.output_height = crtImageHeight;
  cfg.unit_rca.signal_type = M5UnitRCA::signal_type_t::NTSC_J; //  NTSC / NTSC_J / PAL_M / PAL_N
  cfg.unit_rca.use_psram = M5UnitRCA::use_psram_t::psram_use; // psram_no_use / psram_half_use
  cfg.unit_rca.pin_dac = GPIO_NUM_26;
  cfg.unit_rca.output_level = 128;
#endif

  M5.begin(cfg);

  Serial.begin(115200);

  M5.Display.setFont(&fonts::Font4);
  M5.Display.setColorDepth(16); // 16bitカラー
  M5.Display.setTextSize(1, 4);
  M5.Display.setTextColor(TFT_GREEN);
  M5.Display.setTextDatum(top_right); // ファイル番号右詰め表示

  canvas.setColorDepth(16);
  canvas.createSprite(1, 480);

  // 1. 内蔵ストレージ（LittleFS）をマウント
  if (!LittleFS.begin(true)) {
    Serial.println("❌ LittleFS Mount Failed!");
    return; 
  }
  Serial.println("⭕ LittleFS Mounted successfully.");

  prefs.begin("nicos", false);
  int savedFile = prefs.getInt("fileNumber", 0);

  M5.update();
  const bool forceFirstFileAtBoot = M5.BtnA.isPressed(); 
  waitForBootButtonRelease = forceFirstFileAtBoot;

  //  起動時にボタンが押されていたら0番に強制リセット
  if (forceFirstFileAtBoot) {
    fileNumber = 0;
  } else if (savedFile >= 0 && savedFile <= fileNumberMax) {
    fileNumber = savedFile;
  } else {
    fileNumber = 0;
  }

  // 決定したファイル番号でLittleFSからオープン
  if (!fileCloseOpen(fileNumber)) {
    Serial.println("❌ Error opening LittleFS dataFile");
  } else {
    Serial.println("⭕ File opened successfully from LittleFS!");
  }

  updateScreenSlidein();      // データを読み込んでスライドイン
  showFileNumber(fileNumber); // ファイル番号表示

  semaphore = xSemaphoreCreateBinary(); // バイナリセマフォ作成
  timer = timerBegin(0, 80, true);      // タイマ作成 80MHzを80分周
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timerus, true); // タイマ周期設定
  timerAlarmEnable(timer);               // タイマ有効化
}

// loop(): ボタン入力と長押しファイル切り替えを処理し、
//          フレーム読み込み・表示・LED更新を繰り返す
void loop(void) {
  if (dataFile) {
    while (dataFile.available()) {
      M5.update(); // ボタン情報更新

      // 長押しとリリースを判定して保存処理を制御
      const bool currentBtnAPressed = M5.BtnA.isPressed();

      if (waitForBootButtonRelease) {
        if (lastBtnAPressed && !currentBtnAPressed) {
          waitForBootButtonRelease = false;
          pendingFileSave = false;
          longPressHandled = false;
          saveCurrentFileSelection();
          Serial.println("boot button release confirmed");
        }
        lastBtnAPressed = currentBtnAPressed;
        showFileNumber(fileNumber);
        delayMicroseconds(500000);
      } else {
        if (lastBtnAPressed && !currentBtnAPressed) {
          // ボタンが離されたとき
          if (pendingFileSave) {
            saveCurrentFileSelection();
            pendingFileSave = false;
            Serial.printf("[LittleFS] saved fileNumber=%d\n", fileNumber);
          }
          longPressHandled = false;
        }
        lastBtnAPressed = currentBtnAPressed;

        if (currentBtnAPressed) {
          const uint32_t now = millis();
          // 長押しによる連続切り替えの判定
          const bool shouldRepeat = longPressHandled && (now - lastFileChangeMillis >= fileChangeRepeatMs);
          
          if ((M5.BtnA.pressedFor(200) && !longPressHandled) || shouldRepeat) {
            timerAlarmDisable(timer); // タイマー割り込みを一時停止して処理

            fileNumber++;
            if (fileNumber > fileNumberMax) {
              fileNumber = 0;
            }

            if (!fileCloseOpen(fileNumber)) {
              Serial.printf("❌ LittleFS file open failed at index: %d\n", fileNumber);
            }
            
            frame = 0; // フレームカウンターを初期化

            // 画面を更新して新しいファイル番号を表示
            updateScreenSlidein();
            showFileNumber(fileNumber);
            
            pendingFileSave = true;
            longPressHandled = true;
            lastFileChangeMillis = now;
            delayMicroseconds(400000);
            timerAlarmEnable(timer); // タイマー割り込みを再開
            
          } else if (!longPressHandled) { // クリック（短押し）時はファイル番号表示だけ
            showFileNumber(fileNumber);
            delayMicroseconds(500000);
          }
        }

        // ボタンが離された瞬間に最終的な保存を確定
        if (!currentBtnAPressed && pendingFileSave) {
          saveCurrentFileSelection();
          pendingFileSave = false;
          longPressHandled = false;
          Serial.printf("[LittleFS] Final save fileNumber=%d\n", fileNumber);
        }
      }

      if (!readFrameToBuffer()) {
        Serial.println(F("❌ LittleFS read error!"));
        frame = 0;
        continue; 
      }

      // 読み込んだ1列分をスプライトにセット
      canvas.setBuffer(buffer, 1, crtImageHeight, 16);
      canvas.setPivot(0, 0);
      xSemaphoreTake(semaphore, portMAX_DELAY); // タイマー割り込みでセマフォ開放されるのを待つ、解放されたらセマフォ取得
      updateScreen(); // bufferデータを表示

      int state;
      portENTER_CRITICAL(&timerMux);
      state = counter % 3;
      portEXIT_CRITICAL(&timerMux);

      // 使用している縞模様をすべて見たときに色が異なるラインを選び、
      // 点滅時にLEDの色を変えるためのサンプル位置
      const int ledSampleIndexA = 0;
      const int ledSampleIndexB = 442;

      uint16_t color = 0;
      if (state == 1) {
        color = (buffer[ledSampleIndexB] << 8) | buffer[ledSampleIndexB + 1];
      } else if (state == 2) {
        color = (buffer[ledSampleIndexA] << 8) | buffer[ledSampleIndexA + 1];
      }
      setAtomLED(color);

      // Count Frame rate
      fpsCount++;
      if (fpsSec != millis() / 1000) {
        fpsSec = millis() / 1000;
        // Serial.printf("fileNum:%d,fps:%d\r\n", fileNumber, fpsCount);
        fpsCount = 0;
      }
    }
  } else {
    Serial.println(F("error opening dataFile"));
  }
}

// updateScreen(): 現在のバッファをディスプレイに出力し、フレームインデックスを進める
void updateScreen() {
  // 1. 現在のバッファにたまっている1フレーム分を画面に描画
  canvas.pushRotateZoom( &M5.Display, crtImageWidth / 2, 0, 0, crtImageWidth, 1); 
  // 2. フレームを進める
  frame++;
  // 3. 自動計算された総フレーム数（totalFrames）に達したら、ファイルの先頭(0)に戻す
  if (frame >= totalFrames) {
    frame = 0;
    if (dataFile) {
      dataFile.seek(0); // ファイルの読み込み位置も先頭にリセット
    }
  }
}

// updateScreenSlidein(): 1フレーム分を読み込んでから、左からスライドイン表示する
void updateScreenSlidein() {
  readFrameToBuffer(); // 480バイト分バッファに読み込む
  canvas.setBuffer(buffer, 1, crtImageHeight, 16); // canvasバッファにセット 1x480
  canvas.setPivot(0, 0);
  for (int i = -(crtImageWidth * 3) / 2; i < (crtImageWidth / 2 + 1); i += 2) { // 浮動小数点演算(-1.5)を整数演算に最適化
    canvas.pushRotateZoom(&M5.Display, i, 0, 0, crtImageWidth, 1); // crt 120x480を、-180~61まで順にスライドさせる
  }
}

bool readFrameToBuffer() {
    if (!dataFile || totalFrames == 0) {
        return false;
    }

    // 総フレーム数に達したら自動で0（最初）に戻す（可変フレームレート対応）
    if (frame >= totalFrames) {
        frame = 0;
    }

    // 現在のフレーム位置へシーク
    if (!dataFile.seek(frame * crtFrameSize)) {
        dataFile.seek(0);
        frame = 0;
    }

    // 1フレーム分（960バイト）をLittleFSから読み込む
    const bool success = (dataFile.read(buffer, crtFrameSize) == crtFrameSize);
    return success;
}

// showFileNumber(): 現在のファイル番号を画面に表示
void showFileNumber(int fNum) {
  String label = String(fNum + 1);
  M5.Display.setTextSize(1, 4);
  M5.Display.setTextDatum(top_right);
  M5.Display.fillRect(0, 30, crtImageWidth, 40, TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN);
  M5.Display.drawString(label, 110, 50);
}

///LittleFSからファイルを開き、総フレーム数を自動計算する
bool fileCloseOpen(int fNum) {
    if (dataFile) {
        dataFile.close();
    }

    Serial.printf("Loading LittleFS File: %s\n", vfrFileList[fNum]);
    dataFile = LittleFS.open(vfrFileList[fNum], "r");    
    frame = 0;

    if (!dataFile) {
        Serial.println("❌ Failed to open file from LittleFS!");
        totalFrames = 0;
        return false;
    }

    size_t fileSize = dataFile.size();
    totalFrames = fileSize / crtFrameSize;

    Serial.printf("📊 File loaded. Size: %d bytes, Total Frames: %d\n", fileSize, totalFrames);
    return true;
}

// setAtomLED(): RGB565 値を NeoPixel 用 RGB に変換し、AtomLite の LED を点灯する
void setAtomLED(uint16_t c) {
  if (c == 0) {
    neopixelWrite(27, 0, 0, 0);
    return;
  }
  // RGB565から各色を抽出しつつ、計算を1回にまとめて直接「明るさ50」の8bit値(0-255)に変換する
  // 5bit(0-31)の赤・青は * 50 / 31
  // 6bit(0-63)の緑は * 50 / 63
  uint8_t r = ((c >> 11) & 0x1F) * 50 / 31;
  uint8_t g = ((c >> 5) & 0x3F) * 50 / 63;
  uint8_t b = (c & 0x1F) * 50 / 31;

  // AtomLiteのLED (GPIO27) に書き込み
  neopixelWrite(27, r, g, b);
}

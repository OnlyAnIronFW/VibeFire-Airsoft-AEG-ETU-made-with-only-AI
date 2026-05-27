// ============================================================
// 电动软弹枪火控单元 (FCU) v1.1
// 平台: Arduino Nano (ATmega328P, 16MHz, 5V)
// 电池: 6S LiPo (22.2V nominal, 25.2V full)
// 传感器: 897 霍尔开关 x2 (归位 + 快慢机) + MFRC522 NFC
// 显示: SSD1306 0.96" OLED I2C
//
// 依赖: Adafruit_SSD1306, Adafruit_GFX, MFRC522
// ============================================================

#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>

// ============================================================
// 引脚定义
// ============================================================
#define PIN_TRIGGER      2   // 扳机 (INPUT_PULLUP, LOW=按下)
#define PIN_HOME         3   // 归位霍尔 (INT1, FALLING 中断)
#define PIN_SELECTOR_A   4   // 快慢机霍尔 A
#define PIN_SELECTOR_B   5   // 快慢机霍尔 B
#define PIN_MOTOR_MAIN   6   // 主驱动 MOS (PWM)
#define PIN_BTN_UP       7   // UP 按钮
#define PIN_BTN_DOWN     8   // DOWN 按钮
#define PIN_BTN_SEL      9   // SELECT 按钮
#define PIN_NFC_SS       10  // MFRC522 片选 (SPI)
#define PIN_MOTOR_BRAKE  A1  // 刹车 MOS (从 D10 移至此)
#define PIN_NFC_RST      A2  // MFRC522 复位
#define PIN_LED           A3  // LED 指示灯 (移至 A3 避免 SPI SCK 冲突)
#define PIN_BAT_ADC       A0  // 电池分压 ADC

// ============================================================
// 参数常量
// ============================================================
#define PWM_FULL            255
#define BRAKE_DURATION_MS   120
#define STALL_TIMEOUT_MS    2500
#define SELECTOR_DEBOUNCE_MS 60
#define BTN_DEBOUNCE_MS     30
#define BTN_LONG_PRESS_MS   800
#define BTN_REPEAT_MS       200    // 长按连发间隔
#define MENU_TIMEOUT_MS     8000
#define DISPLAY_REFRESH_MS  200

#define CELL_COUNT          6
#define BATTERY_LOW_CELL    3.30f
#define BATTERY_LOW_V       (CELL_COUNT * BATTERY_LOW_CELL)
#define ADC_VREF            5.00f
#define ADC_MAX             1024.0f
#define VOLT_DIV_RATIO      11.0f
#define VOLT_FILTER_ALPHA   0.3f   // 电压低通滤波

#define PRE_COMP_MAX_MS     100
#define AMMO_MAX            999
#define AMMO_PAGE_SIZE      5

// NFC 弹匣
#define NFC_POLL_MS          250
#define NFC_MAG_TIMEOUT_MS   800
#define NFC_BLOCK_DATA       4      // MIFARE 数据块
#define NFC_MAX_CAPACITY     999

// EEPROM 布局
#define EE_PRE_COMP_HI      0
#define EE_PRE_COMP_LO      1
#define EE_AMMO_HI          2
#define EE_AMMO_LO          3
#define EE_MODE_MAP_BASE    4     // 4 bytes
#define EE_BURST_COUNTS     8     // 4 bytes
#define EE_EMPTY_LOCK       12    // 1 byte

// ============================================================
// 枚举
// ============================================================
enum FireMode  { SAFE = 0, SEMI, BINARY, BURST, AUTO };
enum FCUState : uint8_t { IDLE, FIRING, PRECOMP, BRAKING, DONE };
enum MenuPage  { MENU_OFF, MENU_MAIN, MENU_PRE_COMP,
                 MENU_EMPTY_LOCK, MENU_MODE_MAP, MENU_AMMO,
                 MENU_BATTERY, MENU_EDIT_BURST };
enum MagState  { MAG_ABSENT, MAG_PRESENT };

const char* MODE_NAMES[] = {"SAFE", "SEMI", "BIN", "BURST", "AUTO"};

// ============================================================
// 全局状态
// ============================================================
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// --- 状态机 ---
volatile FCUState state = IDLE;
volatile bool    home_isr_flag = false;
bool             cycle_active = false;
unsigned long    motor_start_ms = 0;
unsigned long    brake_start_ms = 0;
unsigned long    precomp_start_ms = 0;
bool             binary_armed = false;   // 双行程: 扣下已击发, 等松开再击发
uint8_t          burst_remaining = 0;     // x连发剩余发数
bool             error_stall = false;
bool             auto_brake_pending = false;  // 连发松扳机, 等下一发归位刹车

// --- 传感器 ---
bool         trigger_now = false;
bool         trigger_prev = false;
FireMode     fire_mode = SAFE;
uint8_t      selector_code = 0;
uint8_t      selector_stable = 0;
unsigned long selector_changed_ms = 0;
float        bat_voltage = 0.0;
bool         bat_low = false;

// --- NFC & 弹匣 ---
MFRC522 nfc(PIN_NFC_SS, PIN_NFC_RST);
MagState     mag_state = MAG_ABSENT;
uint16_t     mag_capacity = 30;       // 当前弹匣容量
unsigned long mag_last_seen_ms = 0;
unsigned long mag_last_poll_ms = 0;

// --- 设置 ---
uint16_t precomp_ms = 0;
uint16_t ammo = 30;
uint8_t  burst_counts[4] = {3, 3, 3, 3};  // 每档位x连发数
bool     empty_lock = true;               // 无弹锁定开关
float    rof_rps = 0.0;                   // 射速 (发/秒)
unsigned long last_home_time = 0;         // 上次归位时间
FireMode mode_map[4] = { SAFE, SEMI, BINARY, AUTO };  // 0=保险 1=单发 2=双行程 3=连发

// --- 按钮 (带时间戳的去抖状态机) ---
struct Button {
  uint8_t pin;
  bool    pressed;          // 当前稳定状态
  bool    prev;             // 上一帧状态
  unsigned long changed_at; // raw != pressed 的起始时间 (用于去抖)
  unsigned long stable_since; // 状态确认时间
  bool    long_press_fired;
};
Button btn_up   = { PIN_BTN_UP,   false, false, 0, 0, false };
Button btn_down = { PIN_BTN_DOWN, false, false, 0, 0, false };
Button btn_sel  = { PIN_BTN_SEL,  false, false, 0, 0, false };

// --- 菜单 ---
MenuPage     menu_page = MENU_OFF;
int8_t       menu_cursor = 0;
uint8_t       menu_edit_slot = 0;
unsigned long menu_idle_since = 0;

// --- 显示节流 ---
unsigned long last_draw_ms = 0;

// ============================================================
// 中断: 归位霍尔 FALLING
// ============================================================
void isr_home() {
  if (state == FIRING && !home_isr_flag) {
    home_isr_flag = true;
  }
}

// ============================================================
// 按钮扫描 (每 loop 调用一次)
// ============================================================
void scan_button(Button &b) {
  b.prev = b.pressed;
  bool raw = (digitalRead(b.pin) == LOW);

  if (raw != b.pressed) {
    // raw 与当前稳定值不同 → 启动/维持去抖计时
    if (b.changed_at == 0) {
      b.changed_at = millis();
    }
    if (millis() - b.changed_at >= BTN_DEBOUNCE_MS) {
      b.pressed = raw;
      b.stable_since = millis();
      b.changed_at = 0;
      b.long_press_fired = false;
    }
  } else {
    // raw == pressed，复位去抖计时
    b.changed_at = 0;
  }
}

// 上升沿 (刚按下)
bool btn_rose(Button &b) { return b.pressed && !b.prev; }
// 下降沿 (刚松开)
bool btn_fell(Button &b) { return !b.pressed && b.prev; }
// 长按中 (>=阈值)
bool btn_long(Button &b) {
  return b.pressed && (millis() - b.stable_since >= BTN_LONG_PRESS_MS);
}
// 长按刚触发 (仅一次)
bool btn_long_once(Button &b) {
  if (btn_long(b) && !b.long_press_fired) {
    b.long_press_fired = true;
    return true;
  }
  return false;
}

// ============================================================
// NFC 弹匣检测 (纯读取)
// ============================================================

void nfc_init() {
  SPI.begin();
  nfc.PCD_Init();
  byte ver = nfc.PCD_ReadRegister(MFRC522::VersionReg);
  if (ver == 0x00 || ver == 0xFF) {
    Serial.println(F("NFC: not found"));
    return;
  }
  Serial.print(F("NFC: MFRC522 v"));
  Serial.println(ver, HEX);
}

// 轮询 NFC 标签 — 读到有效标签就装弹，读不到就视为拔匣
void nfc_poll() {
  if (millis() - mag_last_poll_ms < NFC_POLL_MS) return;
  mag_last_poll_ms = millis();

  if (!nfc.PICC_IsNewCardPresent()) {
    if (mag_state == MAG_PRESENT && millis() - mag_last_seen_ms > NFC_MAG_TIMEOUT_MS) {
      mag_state = MAG_ABSENT;
      ammo = 0;
      Serial.println(F("NFC: mag removed"));
    }
    return;
  }

  if (!nfc.PICC_ReadCardSerial()) return;
  mag_last_seen_ms = millis();

  // 读取标签容量
  uint16_t capacity = 0;
  bool valid = nfc_read_capacity(capacity);

  nfc.PICC_HaltA();
  nfc.PCD_StopCrypto1();

  if (valid && capacity > 0) {
    mag_capacity = capacity;
    if (mag_state != MAG_PRESENT) {
      mag_state = MAG_PRESENT;
      ammo = capacity;
      Serial.print(F("NFC: mag in, cap="));
      Serial.println(capacity);
    }
  }
}

// 读取标签容量 (幻数 "FCUMAG" + 2字节容量)
bool nfc_read_capacity(uint16_t &capacity) {
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  byte trailer = (NFC_BLOCK_DATA / 4 + 1) * 4 - 1;
  if (nfc.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &key, &(nfc.uid)) != MFRC522::STATUS_OK)
    return false;

  byte buf[18];
  byte len = sizeof(buf);
  if (nfc.MIFARE_Read(NFC_BLOCK_DATA, buf, &len) != MFRC522::STATUS_OK) return false;

  if (buf[0] != 'F' || buf[1] != 'C' || buf[2] != 'U' ||
      buf[3] != 'M' || buf[4] != 'A' || buf[5] != 'G') return false;

  capacity = ((uint16_t)buf[8] << 8) | buf[9];
  if (capacity > NFC_MAX_CAPACITY) capacity = NFC_MAX_CAPACITY;
  return true;
}

// ============================================================
// 初始化
// ============================================================
void setup() {
  wdt_enable(WDTO_250MS);  // 看门狗: 卡死 250ms 后自动复位, 保护电机/电池

  // MOS 先关
  pinMode(PIN_MOTOR_MAIN, OUTPUT);   digitalWrite(PIN_MOTOR_MAIN, LOW);
  pinMode(PIN_MOTOR_BRAKE, OUTPUT);  digitalWrite(PIN_MOTOR_BRAKE, LOW);

  pinMode(PIN_TRIGGER,    INPUT_PULLUP);
  pinMode(PIN_HOME,       INPUT);
  pinMode(PIN_SELECTOR_A, INPUT);
  pinMode(PIN_SELECTOR_B, INPUT);
  pinMode(PIN_BTN_UP,     INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN,   INPUT_PULLUP);
  pinMode(PIN_BTN_SEL,    INPUT_PULLUP);
  pinMode(PIN_LED,        OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);

  // OLED 先初始化, 再 NFC
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(22, 20);
  oled.print("FCU v1.1 NFC");
  oled.setCursor(28, 36);
  oled.print("Booting...");
  oled.display();

  nfc_init();

  load_eeprom();
  attachInterrupt(digitalPinToInterrupt(PIN_HOME), isr_home, FALLING);

  // 初始无弹匣则弹量为0 (NFC 读到标签后会设回标签容量)
  if (mag_state == MAG_ABSENT) {
    ammo = 0;
  }

  delay(300);
  digitalWrite(PIN_LED, HIGH);

  Serial.println(F("FCU v1.1 Ready"));
  Serial.print(F("  Precomp: ")); Serial.print(precomp_ms); Serial.println(F("ms"));
  Serial.print(F("  Ammo: "));    Serial.println(ammo);
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  Pos")); Serial.print(i+1);
    Serial.print(F("=")); Serial.println(MODE_NAMES[mode_map[i]]);
  }
}

// ============================================================
// 主循环
// ============================================================
void loop() {
  wdt_reset();  // 喂狗: loop 必须在 250ms 内返回此处

  read_sensors();
  scan_button(btn_up);
  scan_button(btn_down);
  scan_button(btn_sel);
  nfc_poll();

  if (menu_page != MENU_OFF) {
    run_menu();
  } else {
    run_fcu();
  }

  if (millis() - last_draw_ms >= DISPLAY_REFRESH_MS) {
    last_draw_ms = millis();
    draw();
  }
  update_led();
}

// ============================================================
// 传感器
// ============================================================
void read_sensors() {
  // 扳机
  trigger_prev = trigger_now;
  trigger_now = (digitalRead(PIN_TRIGGER) == LOW);

  // 快慢机: 2-bit Gray-like 编码, 带稳定去抖
  uint8_t raw = 0;
  if (digitalRead(PIN_SELECTOR_A) == LOW) raw |= 0b01;
  if (digitalRead(PIN_SELECTOR_B) == LOW) raw |= 0b10;

  if (raw != selector_code) {
    selector_code = raw;
    selector_changed_ms = millis();
  }
  if (millis() - selector_changed_ms >= SELECTOR_DEBOUNCE_MS) {
    if (selector_stable != raw) {
      selector_stable = raw;
      binary_armed = false;  // 切模式时清除双行程状态
    }
  }

  // 快慢机 → fire_mode 映射
  // selector_stable: 0b00=SAFE, 0b01=SEMI, 0b10=BINARY, 0b11=AUTO
  if (selector_stable < 4) {
    fire_mode = mode_map[selector_stable];
  }

  // 电池电压 (低通滤波)
  int adc = analogRead(PIN_BAT_ADC);
  float inst_v = (adc * ADC_VREF / ADC_MAX) * VOLT_DIV_RATIO;
  bat_voltage = VOLT_FILTER_ALPHA * inst_v + (1.0f - VOLT_FILTER_ALPHA) * bat_voltage;
  bat_low = (bat_voltage < BATTERY_LOW_V && bat_voltage > 5.0f);
  // (bat_voltage > 5 防止未接电池时的误报)
}

// ============================================================
// 火控状态机
// ============================================================
void run_fcu() {
  // 长按 SELECT → 菜单 (仅在 IDLE 时)
  if (state == IDLE && btn_long_once(btn_sel)) {
    menu_page = MENU_MAIN;
    menu_cursor = 0;
    menu_idle_since = millis();
    return;
  }

  switch (state) {

  case IDLE: {
    auto_brake_pending = false;
    bool can_fire = (fire_mode != SAFE)
                 && (ammo > 0 || !empty_lock)  // 空仓锁
                 && !bat_low
                 && !error_stall;

    if (can_fire) {
      if (fire_mode == BINARY) {
        // 双行程: 扣下击发一发, 松开击发第二发 (松开不依赖边沿, 防止cycle中丢沿)
        if (trigger_now && !trigger_prev) {
          binary_armed = true;
          start_cycle();
        } else if (!trigger_now && binary_armed) {
          binary_armed = false;
          start_cycle();
        }
      } else if (fire_mode == BURST && trigger_now && !trigger_prev) {
        // x连发: 使用当前档位的连发数
        burst_remaining = burst_counts[selector_stable];
        start_cycle();
      } else {
        // 单发/连发: 扣下击发
        if (trigger_now && !trigger_prev) {
          start_cycle();
        }
      }
    } else {
      binary_armed = false;
    }
    break;
  }

  case FIRING: {
    // 堵转保护
    if (millis() - motor_start_ms > STALL_TIMEOUT_MS) {
      emergency_stop();
      error_stall = true;
      auto_brake_pending = false;
      state = IDLE;
      cycle_active = false;
      break;
    }

    // BURST 中再次扣扳机 → 重置连发计数
    if (fire_mode == BURST && trigger_now && !trigger_prev) {
      burst_remaining = burst_counts[selector_stable];
    }

    // 连发模式下松扳机 → 标记等待归位刹车
    if (fire_mode == AUTO && !trigger_now) {
      auto_brake_pending = true;
    }

    if (home_isr_flag) {
      // 归位确认 (897模块自带硬件去抖, ISR已锁存)
      // 每次归位 = 1发完成, 扣弹并算射速
      if (ammo > 0) ammo--;
      if (last_home_time > 0) {
        float instant = 1000.0f / (millis() - last_home_time);
        rof_rps = rof_rps * 0.3f + instant * 0.7f;
      }
      last_home_time = millis();

      // 还需继续运转? (AUTO扳机未松 或 BURST未打完)
      bool keep_running = false;
      if (fire_mode == AUTO && trigger_now && ammo > 0 && !auto_brake_pending) {
        keep_running = true;
      } else if (fire_mode == BURST && burst_remaining > 1 && ammo > 0 && !bat_low) {
        burst_remaining--;
        keep_running = true;
      }

      if (keep_running) {
        motor_start_ms = millis();
      } else {
        auto_brake_pending = false;
        if (precomp_ms > 0) {
          precomp_start_ms = millis();
          state = PRECOMP;
        } else {
          apply_brake();
          state = BRAKING;
        }
      }
      home_isr_flag = false;
    }
    break;
  }

  case PRECOMP: {
    if (millis() - precomp_start_ms >= precomp_ms) {
      apply_brake();
      state = BRAKING;
    }
    home_isr_flag = false;  // 忽略多余的归位脉冲
    break;
  }

  case BRAKING: {
    // 扳机按下 → 立刻打断刹车, 开始下一发
    if (trigger_now && !trigger_prev) {
      release_brake();
      start_cycle();
      break;
    }

    if (millis() - brake_start_ms >= BRAKE_DURATION_MS) {
      release_brake();
      finish_cycle();
      state = DONE;
    }
    break;
  }

  case DONE: {
    // BURST 残留发数 (非持续运转路径落入的)
    if (fire_mode == BURST && burst_remaining > 1 && ammo > 0 && !bat_low && !error_stall) {
      burst_remaining--;
      start_cycle();
      break;
    }
    burst_remaining = 0;
    state = IDLE;
    cycle_active = false;
    break;
  }

  } // switch
}

// ============================================================
// 循环控制
// ============================================================
void start_cycle() {
  cycle_active = true;
  home_isr_flag = false;
  auto_brake_pending = false;
  state = FIRING;
  motor_on();
  Serial.println(F("FIRE"));
}

void finish_cycle() {
  // ammo 已在 FIRING 的 home 检测时扣除
  Serial.print(F("DONE ammo="));
  Serial.println(ammo);
}

// ============================================================
// 电机驱动 (死区互锁)
// ============================================================
void motor_on() {
  digitalWrite(PIN_MOTOR_BRAKE, LOW);
  delayMicroseconds(80);
  analogWrite(PIN_MOTOR_MAIN, PWM_FULL);
  motor_start_ms = millis();
}

void motor_off() {
  analogWrite(PIN_MOTOR_MAIN, 0);
  digitalWrite(PIN_MOTOR_MAIN, LOW);
}

void apply_brake() {
  motor_off();
  delayMicroseconds(120);
  digitalWrite(PIN_MOTOR_BRAKE, HIGH);
  brake_start_ms = millis();
}

void release_brake() {
  digitalWrite(PIN_MOTOR_BRAKE, LOW);
}

void emergency_stop() {
  motor_off();
  delayMicroseconds(100);
  digitalWrite(PIN_MOTOR_BRAKE, HIGH);
  delay(250);
  digitalWrite(PIN_MOTOR_BRAKE, LOW);
  Serial.println(F("STALL!"));
}

// ============================================================
// 菜单逻辑
// ============================================================
void run_menu() {
  // 超时
  if (millis() - menu_idle_since > MENU_TIMEOUT_MS) {
    menu_page = MENU_OFF;
    save_eeprom();
    return;
  }

  // 长按 SEL → 退出菜单回 IDLE
  if (btn_long_once(btn_sel)) {
    menu_page = MENU_OFF;
    save_eeprom();
    menu_idle_since = millis();
    return;
  }

  bool up   = btn_rose(btn_up);
  bool down = btn_rose(btn_down);
  bool back = btn_rose(btn_sel);      // SEL 短按 = 返回上一级
  bool trig = trigger_now && !trigger_prev;  // 扳机 = 确认/选择
  // 长按连发 (用于调值)
  bool up_rep   = btn_long(btn_up)   && (millis() - btn_up.stable_since - BTN_LONG_PRESS_MS) % BTN_REPEAT_MS < (DISPLAY_REFRESH_MS + 10);
  bool down_rep = btn_long(btn_down) && (millis() - btn_down.stable_since - BTN_LONG_PRESS_MS) % BTN_REPEAT_MS < (DISPLAY_REFRESH_MS + 10);

  if (up || down || back || trig) menu_idle_since = millis();
  if (up_rep || down_rep) menu_idle_since = millis();

  // SEL 短按 = 返回 (子页面→MAIN, MAIN→退出)
  if (back) {
    if (menu_page == MENU_MAIN) {
      menu_page = MENU_OFF;
      save_eeprom();
    } else {
      menu_page = MENU_MAIN;
      menu_cursor = 0;
    }
    return;
  }

  switch (menu_page) {

  case MENU_MAIN: {
    const int N = 5;
    if (up)   menu_cursor = (menu_cursor - 1 + N) % N;
    if (down) menu_cursor = (menu_cursor + 1) % N;
    if (trig) {
      switch (menu_cursor) {
        case 0: menu_page = MENU_PRE_COMP;   break;
        case 1: menu_page = MENU_EMPTY_LOCK; break;
        case 2: menu_page = MENU_MODE_MAP;   break;
        case 3: menu_page = MENU_AMMO;       break;
        case 4: menu_page = MENU_BATTERY;    break;
      }
    }
    break;
  }

  case MENU_PRE_COMP: {
    int delta = 0;
    if (up || up_rep)       delta = +5;
    if (down || down_rep)   delta = -5;
    if (delta != 0) {
      int v = (int)precomp_ms + delta;
      if (v < 0) v = 0;
      if (v > PRE_COMP_MAX_MS) v = PRE_COMP_MAX_MS;
      precomp_ms = (uint16_t)v;
    }
    break;
  }

  case MENU_EMPTY_LOCK: {
    if (trig) { empty_lock = !empty_lock; }  // 切换开关
    break;
  }

  case MENU_MODE_MAP: {
    if (up)   menu_edit_slot = (menu_edit_slot + 1) % 4;
    if (down) menu_edit_slot = (menu_edit_slot - 1 + 4) % 4;
    if (trig) {
      // 循环切换模式 (5种)
      mode_map[menu_edit_slot] = (FireMode)(((int)mode_map[menu_edit_slot] + 1) % 5);
      // 如果切换到 BURST → 进入连发数编辑
      if (mode_map[menu_edit_slot] == BURST) {
        menu_page = MENU_EDIT_BURST;
      }
    }
    break;
  }

  case MENU_EDIT_BURST: {
    int delta = 0;
    if (up || up_rep)       delta = +1;
    if (down || down_rep)   delta = -1;
    if (delta != 0) {
      int v = (int)burst_counts[menu_edit_slot] + delta;
      if (v < 2) v = 2;
      if (v > 10) v = 10;
      burst_counts[menu_edit_slot] = (uint8_t)v;
    }
    if (trig || back) {
      menu_page = MENU_MODE_MAP;  // 确认或返回
    }
    break;
  }

  case MENU_AMMO: {
    int delta = 0;
    if (up || up_rep)       delta = +AMMO_PAGE_SIZE;
    if (down || down_rep)   delta = -AMMO_PAGE_SIZE;
    if (delta != 0) {
      int v = (int)ammo + delta;
      if (v < 0) v = 0;
      if (v > AMMO_MAX) v = AMMO_MAX;
      ammo = (uint16_t)v;
    }
    break;
  }

  case MENU_BATTERY: {
    // 只读
    break;
  }

  default: break;
  }
}

// ============================================================
// 显示
// ============================================================
void draw() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  if (menu_page != MENU_OFF) draw_menu();
  else                        draw_main();

  oled.display();
}

void draw_main() {
  char b[24];
  int16_t x1, y1;
  uint16_t w, h;

  // ── 顶栏: 电压 | ROF/预压 | 状态 ──
  oled.setCursor(0, 0);
  if (bat_low) oled.print("LOW!");
  else { snprintf(b, sizeof(b), "%.1fV", (double)bat_voltage); oled.print(b); }

  oled.setCursor(52, 0);
  if (cycle_active && rof_rps > 0.5f) {
    snprintf(b, sizeof(b), "%dRPS", (int)(rof_rps + 0.5f));
  } else if (precomp_ms > 0 && state == IDLE) {
    snprintf(b, sizeof(b), "PRE:%d", precomp_ms);
  } else {
    snprintf(b, sizeof(b), "%dRPS", 0);
  }
  oled.print(b);

  oled.setCursor(108, 0);
  if (error_stall)        oled.print("ERR");
  else if (bat_low)       oled.print("LOW");
  else if (cycle_active)  oled.print(">>");
  else                    oled.print("OK");

  oled.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  // ── 中央大字: 剩余弹量 / 满匣容量 ──
  oled.setTextSize(3);
  snprintf(b, sizeof(b), "%d", ammo);
  oled.getTextBounds(b, 0, 0, &x1, &y1, &w, &h);
  int cx = (128 - w - 24) / 2;  // 居中 (留空间给 "/容量")
  oled.setCursor(cx, 15);
  oled.print(b);

  oled.setTextSize(1);
  oled.setCursor(cx + w + 2, 34);
  snprintf(b, sizeof(b), "/%d", mag_capacity > 0 ? mag_capacity : 30);
  oled.print(b);

  // ── 进度条 ──
  int bar_y = 47;
  int cap = mag_capacity > 0 ? mag_capacity : 30;
  oled.drawRect(8, bar_y, 112, 7, SSD1306_WHITE);
  int bw = map(ammo, 0, cap, 0, 110);
  if (bw > 110) bw = 110;
  if (bw > 0) oled.fillRect(8, bar_y, bw, 7, SSD1306_WHITE);

  // ── 底栏: 当前模式 | 弹匣状态 ──
  oled.setCursor(0, 56);
  oled.print(MODE_NAMES[fire_mode]);

  if (ammo == 0 && mag_state == MAG_PRESENT) {
    oled.setCursor(70, 56); oled.print("EMPTY");
  } else if (mag_state == MAG_ABSENT) {
    oled.setCursor(70, 56); oled.print("NO MAG");
  } else {
    oled.setCursor(90, 56); oled.print("HOLD SEL");
  }
}

void draw_menu() {
  oled.setCursor(0, 0);

  switch (menu_page) {

  case MENU_MAIN: {
    oled.println("===== MENU =====");
    oled.println("");
    draw_menuitem(0, "Pre-Comp Delay");
    draw_menuitem(1, "Empty Lock");
    draw_menuitem(2, "Mode Config");
    draw_menuitem(3, "Ammo Count");
    draw_menuitem(4, "Battery Info");
    oled.println("");
    oled.println("TRIG=Sel SEL=Back");
    break;
  }

  case MENU_PRE_COMP: {
    oled.println("=== PRE-COMP ===");
    oled.println("");
    oled.print("Delay: ");
    oled.print(precomp_ms);
    oled.println(" ms");
    oled.println("");
    // 简易进度条
    oled.drawRect(0, 34, 100, 8, SSD1306_WHITE);
    int pw = map(precomp_ms, 0, PRE_COMP_MAX_MS, 0, 98);
    if (pw > 0) oled.fillRect(0, 34, pw, 8, SSD1306_WHITE);
    oled.println("");
    oled.println("UP +5  DOWN -5");
    oled.println("SEL=Back");
    break;
  }

  case MENU_EMPTY_LOCK: {
    oled.println("== EMPTY LOCK ==");
    oled.println("");
    oled.print("Status: ");
    oled.println(empty_lock ? "ON" : "OFF");
    oled.println("");
    oled.print(empty_lock ? "ammo=0 blocks" : "dry fire allowed");
    oled.println("");
    oled.println("TRIG=Toggle SEL=Back");
    break;
  }

  case MENU_MODE_MAP: {
    oled.println("=== MODE MAP ===");
    for (int i = 0; i < 4; i++) {
      oled.print(i == menu_edit_slot ? "> " : "  ");
      oled.print("P"); oled.print(i + 1); oled.print(":");
      oled.print(MODE_NAMES[mode_map[i]]);
      if (mode_map[i] == BURST) {
        oled.print(" x"); oled.print(burst_counts[i]);
      }
      oled.println("");
    }
    oled.println("UP/DN=Slot TRIG=Chg");
    break;
  }

  case MENU_EDIT_BURST: {
    oled.println("== BURST P");
    oled.print(menu_edit_slot + 1);
    oled.println(" CNT ==");
    oled.println("");
    oled.print("Shots: ");
    oled.println(burst_counts[menu_edit_slot]);
    oled.println("");
    oled.println("UP +1  DOWN -1");
    oled.println("TRIG/SEL=OK (2-10)");
    break;
  }

  case MENU_AMMO: {
    oled.println("=== AMMO ===");
    oled.println("");
    oled.print("Count: ");
    oled.println(ammo);
    oled.println("");
    oled.println("UP +5  DOWN -5");
    oled.println("SEL=Back");
    break;
  }

  case MENU_BATTERY: {
    oled.println("=== BATTERY ===");
    oled.println("");
    oled.print("Total: ");
    oled.print(bat_voltage, 1);
    oled.println("V");
    float cv = bat_voltage / CELL_COUNT;
    oled.print("Cell:  ");
    oled.print(cv, 2);
    oled.println("V");
    oled.println("");
    oled.println("SEL=Back");
    break;
  }

  default: break;
  }
}

void draw_menuitem(int idx, const char* label) {
  oled.print(idx == menu_cursor ? "> " : "  ");
  oled.println(label);
}

// ============================================================
// LED
// ============================================================
void update_led() {
  static unsigned long t;
  if (millis() - t < 250) return;
  t = millis();

  if (error_stall) {
    digitalWrite(PIN_LED, (millis() / 100) % 2);
  } else if (bat_low) {
    digitalWrite(PIN_LED, (millis() / 500) % 2);
  } else if (cycle_active) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, HIGH);
  }
}

// ============================================================
// EEPROM
// ============================================================
void load_eeprom() {
  uint16_t v;
  v  = ((uint16_t)EEPROM.read(EE_PRE_COMP_HI) << 8) | EEPROM.read(EE_PRE_COMP_LO);
  precomp_ms = (v <= PRE_COMP_MAX_MS) ? v : 0;

  v  = ((uint16_t)EEPROM.read(EE_AMMO_HI) << 8) | EEPROM.read(EE_AMMO_LO);
  ammo = (v <= AMMO_MAX) ? v : 30;

  for (int i = 0; i < 4; i++) {
    uint8_t m = EEPROM.read(EE_MODE_MAP_BASE + i);
    if (m <= AUTO) mode_map[i] = (FireMode)m;
  }

  for (int i = 0; i < 4; i++) {
    uint8_t bc = EEPROM.read(EE_BURST_COUNTS + i);
    if (bc >= 2 && bc <= 10) burst_counts[i] = bc;
  }
  empty_lock = EEPROM.read(EE_EMPTY_LOCK) != 0;
}

void save_eeprom() {
  EEPROM.write(EE_PRE_COMP_HI, precomp_ms >> 8);
  EEPROM.write(EE_PRE_COMP_LO, precomp_ms & 0xFF);
  EEPROM.write(EE_AMMO_HI, ammo >> 8);
  EEPROM.write(EE_AMMO_LO, ammo & 0xFF);
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EE_MODE_MAP_BASE + i, (uint8_t)mode_map[i]);
  }
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EE_BURST_COUNTS + i, burst_counts[i]);
  }
  EEPROM.write(EE_EMPTY_LOCK, empty_lock ? 1 : 0);
}

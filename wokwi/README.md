# FCU 火控 Wokwi 硬件仿真

在 [Wokwi.com](https://wokwi.com) 在线仿真 Arduino Nano 火控单元。

## 文件

| 文件 | 说明 |
|------|------|
| `diagram.json` | 电路图配置 |
| `fire_control.ino` | 火控固件 (与 firmware/ 同步) |
| `libraries.txt` | 依赖库列表 |

## 电路说明

### 输入 (模拟传感器)

| 元件 | 模拟 | Arduino 引脚 |
|------|------|-------------|
| 红色按钮 TRIG | 扳机微动 | D2 |
| 滑动开关 HOME | 归位霍尔传感器 | D3 |
| 滑动开关 SEL-A | 快慢机霍尔 A | D4 |
| 滑动开关 SEL-B | 快慢机霍尔 B | D5 |
| 绿色按钮 UP/DN/SEL | 菜单导航 | D7/D8/D9 |
| 电位器 BATT SIM | 电池电压检测 | A0 |

### 输出 (状态指示)

| 元件 | 表示 | Arduino 引脚 |
|------|------|-------------|
| 绿色 LED MOTOR | 电机运转 | D6 (PWM) |
| 红色 LED BRAKE | 刹车状态 | A1 |
| 黄色 LED RDY | 预压就绪/故障 | D13 |
| SSD1306 OLED | 主显示屏 | A4/A5 (I2C) |

### 快慢机编码

| SEL-B | SEL-A | 默认模式 |
|-------|-------|---------|
| HIGH | HIGH | SAFE (保险) |
| HIGH | LOW | SEMI (单发) |
| LOW | HIGH | BINARY (双行程) |
| LOW | LOW | AUTO (连发) |

## 限制

- **MFRC522 NFC** 模块 Wokwi 不支持，固件启动后 NFC 初始化会静默失败，弹匣功能不可用。可通过菜单手动设置弹量。
- **电机** 用 LED 代替，绿色亮=运转，红色亮=刹车
- 齿轮/归位时序由 HOME 开关手动控制

## 操作

1. 打开 [Wokwi.com](https://wokwi.com)
2. 新建 Arduino Nano 项目
3. 替换 `diagram.json` 和 `sketch.ino`
4. 添加 `libraries.txt`
5. 点击 ▶ 运行
6. OLED 显示主界面
7. 拨 SEL-A/SEL-B 选择模式
8. 按 TRIG 击发，拨 HOME 开关模拟归位信号

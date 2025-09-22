# ESP32 LCD 1602 I2C + Motor Driver Wiring Guide

## ESP32 30-pin DevKit Module Complete Wiring

### I2C LCD 1602 Connections:

| LCD 1602 I2C Pin | ESP32 30-pin Module | Description                     |
| ---------------- | ------------------- | ------------------------------- |
| VCC              | 3.3V hoặc 5V        | Power supply (3.3V recommended) |
| GND              | GND                 | Ground                          |
| SDA              | GPIO 21             | I2C Data line                   |
| SCL              | GPIO 22             | I2C Clock line                  |

### Motor Driver L298N Connections:

| L298N Pin | ESP32 30-pin Module | Motor Connection | Description                       |
| --------- | ------------------- | ---------------- | --------------------------------- |
| ENA       | GPIO 5              | Motor A Speed    | PWM speed control for right motor |
| IN1       | GPIO 16             | Motor A Dir 1    | Right motor direction pin 1       |
| IN2       | GPIO 17             | Motor A Dir 2    | Right motor direction pin 2       |
| IN3       | GPIO 19             | Motor B Dir 1    | Left motor direction pin 1        |
| IN4       | GPIO 18             | Motor B Dir 2    | Left motor direction pin 2        |
| ENB       | GPIO 23             | Motor B Speed    | PWM speed control for left motor  |
| VCC       | 5V/VIN              | -                | Power for motors (5V-12V)         |
| GND       | GND                 | -                | Ground                            |

### Complete Wiring Diagram:

```
ESP32 DevKit 30-pin        LCD 1602 I2C        L298N Motor Driver
┌─────────────────┐        ┌──────────────┐    ┌─────────────────┐
│                 │        │              │    │                 │
│ 3.3V     ┌──────┼────────┼─ VCC         │    │                 │
│ GND      │ ┌────┼────────┼─ GND         │    │ ┌─── GND        │
│ GPIO21   │ │ ┌──┼────────┼─ SDA         │    │ │               │
│ GPIO22   │ │ │  │        │ └─ SCL       │    │ │               │
│ GPIO5    │ │ │  │        │              │    │ │ ┌─ ENA        │
│ GPIO16   │ │ │  │        │              │    │ │ │ ┌─ IN1      │
│ GPIO17   │ │ │  │        │              │    │ │ │ │ ┌─ IN2    │
│ GPIO19   │ │ │  │        │              │    │ │ │ │ │ ┌─ IN3  │
│ GPIO18   │ │ │  │        │              │    │ │ │ │ │ │ ┌─ IN4│
│ GPIO23   │ │ │  │        │              │    │ │ │ │ │ │ │ ┌─ENB│
│ VIN      │ │ │  │        │              │    │ │ │ │ │ │ │ │ ┌VCC│
│          │ │ │  │        │              │    │ │ │ │ │ │ │ │ │ │
└──────────┼─┼─┼──┘        └──────────────┘    └─┼─┼─┼─┼─┼─┼─┼─┼─┼─┘
           │ │ │                                 │ │ │ │ │ │ │ │ │
           │ │ └─────────────────────────────────┼─┼─┼─┼─┼─┼─┼─┼─┘
           │ └───────────────────────────────────┼─┼─┼─┼─┼─┼─┼─┘
           └─────────────────────────────────────┼─┼─┼─┼─┼─┼─┘
                                                 │ │ │ │ │ │
                                Power & I2C     │ │ │ │ │ │ Motor Control
                                                 │ │ │ │ │ │
                                                 └─┼─┼─┼─┼─┘
                                                   │ │ │ │
                                                   └─┼─┼─┘
                                                     │ │
                                                     └─┘
```

### Pin Assignment Summary:

| Function  | ESP32 Pin | Previous Pin | Change Reason                    |
| --------- | --------- | ------------ | -------------------------------- |
| I2C SDA   | GPIO 21   | -            | Standard I2C pin                 |
| I2C SCL   | GPIO 22   | -            | Standard I2C pin                 |
| Motor IN1 | GPIO 16   | GPIO 22      | **CHANGED** - Avoid I2C conflict |
| Motor IN2 | GPIO 17   | GPIO 21      | **CHANGED** - Avoid I2C conflict |
| Motor IN3 | GPIO 19   | GPIO 19      | No change                        |
| Motor IN4 | GPIO 18   | GPIO 18      | No change                        |
| Motor ENA | GPIO 5    | GPIO 5       | No change                        |
| Motor ENB | GPIO 23   | GPIO 23      | No change                        |

### Important Notes:

1. **Power Supply**:

   - Sử dụng 3.3V từ ESP32 (khuyến nghị)
   - Hoặc có thể dùng 5V nếu module LCD hỗ trợ

2. **I2C Address**:

   - Địa chỉ I2C phổ biến: `0x27` hoặc `0x3F`
   - Code đã được cấu hình với địa chỉ `0x27`
   - Nếu không hoạt động, thay đổi thành `0x3F` trong code

3. **Pull-up Resistors**:
   - Module LCD I2C thường đã có pull-up resistors tích hợp
   - ESP32 cũng có pull-up resistors nội bộ được kích hoạt

### Pin Layout ESP32 30-pin DevKit:

```
      ESP32 DevKit 30-pin Layout
      ┌─────────────────────────┐
   EN │1                      30│ VIN
 VP36 │2                      29│ GND
 VN39 │3                      28│ D13
  D34 │4                      27│ D12
  D35 │5                      26│ D14
  D32 │6                      25│ D27
  D33 │7                      24│ D26
  D25 │8                      23│ D25
  D26 │9                      22│ D33
  D27 │10                     21│ D32
  D14 │11                     20│ D35
  D12 │12                     19│ D34
  GND │13                     18│ VN39
  D13 │14                     17│ VP36
  D15 │15     USB PORT      16│ EN
      └─────────────────────────┘

      Left Side:              Right Side:
      GPIO21 (SDA) ────┐     ┌──── GPIO22 (SCL)
      3.3V ────────────┼─────┼──── GND
                       │     │
                       └─────┘
                    Connection points
```

### Testing the Connection:

1. **Upload Code**: Upload the modified `main1.cpp`
2. **Check Serial Monitor**: Look for any I2C initialization messages
3. **LCD Display**: Should show "ESP32 Smart Car" on first line and status on second line

### Troubleshooting:

- **LCD không sáng**: Kiểm tra kết nối VCC và GND
- **Không hiển thị text**: Thử thay đổi địa chỉ I2C từ `0x27` thành `0x3F`
- **Hiển thị lỗi**: Kiểm tra kết nối SDA và SCL
- **Contrast quá thấm/cao**: Điều chỉnh potentiometer trên module I2C (nếu có)

### Code Configuration:

```cpp
#define LCD_SDA 21        // GPIO21 - SDA pin
#define LCD_SCL 22        // GPIO22 - SCL pin
#define LCD_ADDRESS 0x27  // I2C address (try 0x3F if doesn't work)
```

Nếu cần thay đổi địa chỉ I2C, sửa dòng `#define LCD_ADDRESS 0x27` thành `#define LCD_ADDRESS 0x3F` trong file main1.cpp.

# 🚗 ESP32 Car Control Tutorial: From Basic to Advanced

## Hướng dẫn điều khiển xe ESP32: Từ cơ bản đến nâng cao

---

## 📚 Mục lục / Table of Contents

1. [Giới thiệu](#giới-thiệu)
2. [Phần 1: Điều khiển motor đơn giản](#phần-1-điều-khiển-motor-đơn-giản)
3. [Phần 2: Điều khiển 4 motor với L298N](#phần-2-điều-khiển-4-motor-với-l298n)
4. [Phần 3: Tích hợp FreeRTOS](#phần-3-tích-hợp-freertos)
5. [Phần 4: Tích hợp E-Ra IoT Platform](#phần-4-tích-hợp-e-ra-iot-platform)
6. [Phần 5: Optimization và Best Practices](#phần-5-optimization-và-best-practices)

---

## Giới thiệu

Tutorial này sẽ hướng dẫn bạn từng bước xây dựng hệ thống điều khiển xe ESP32, từ những khái niệm cơ bản nhất đến việc tích hợp các công nghệ tiên tiến như FreeRTOS và E-Ra IoT Platform.

### 🎯 Mục tiêu học tập:

- Hiểu cách hoạt động của motor DC và PWM
- Làm chủ việc điều khiển nhiều motor đồng thời
- Áp dụng FreeRTOS cho hệ thống real-time
- Tích hợp IoT platform cho điều khiển từ xa
- Tối ưu hóa hiệu suất và độ ổn định

---

## Phần 1: Điều khiển motor đơn giản

### 1.1 Kiến thức cơ bản

#### Motor DC là gì?

- Motor DC (Direct Current) là loại motor quay bằng dòng điện một chiều
- Tốc độ quay tỷ lệ thuận với điện áp đặt vào
- Chiều quay phụ thuộc vào cực tính điện áp

#### PWM (Pulse Width Modulation)

- Kỹ thuật điều khiển công suất bằng cách thay đổi độ rộng xung
- Duty cycle: % thời gian HIGH trong một chu kỳ
- Frequency: số chu kỳ trong 1 giây (Hz)

### 1.2 Code cơ bản - Điều khiển 1 motor

```cpp
// File: basic_motor_control.ino
#include <Arduino.h>

// Pin definitions
#define MOTOR_PIN1 22  // Điều khiển chiều quay
#define MOTOR_PIN2 21  // Điều khiển chiều quay
#define MOTOR_EN   5   // PWM pin để điều khiển tốc độ

void setup() {
  Serial.begin(115200);

  // Cấu hình pin
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  Serial.println("Basic Motor Control Started");
}

void loop() {
  // Quay thuận với tốc độ tăng dần
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);

  for(int speed = 0; speed <= 255; speed += 5) {
    analogWrite(MOTOR_EN, speed);
    Serial.println("Speed: " + String(speed));
    delay(100);
  }

  delay(1000);

  // Dừng motor
  analogWrite(MOTOR_EN, 0);
  delay(1000);

  // Quay nghịch với tốc độ cố định
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, HIGH);
  analogWrite(MOTOR_EN, 150);
  delay(2000);

  // Dừng motor
  analogWrite(MOTOR_EN, 0);
  delay(2000);
}
```

### 1.3 Giải thích code:

1. **Setup()**: Khởi tạo pin và serial communication
2. **Loop()**: Thực hiện chu trình điều khiển
   - Quay thuận với tốc độ tăng dần (0-255)
   - Dừng 1 giây
   - Quay nghịch với tốc độ 150
   - Dừng 2 giây

### 1.4 Bài tập thực hành:

- [ ] Thay đổi tốc độ quay tối đa
- [ ] Thêm chức năng điều khiển qua Serial Monitor
- [ ] Thêm potentiometer để điều khiển tốc độ

---

## Phần 2: Điều khiển 4 motor với L298N

### 2.1 Sơ đồ kết nối L298N

```
ESP32          L298N Motor Driver
GPIO22   -->   IN1 (Motor A Direction)
GPIO21   -->   IN2 (Motor A Direction)
GPIO19   -->   IN3 (Motor B Direction)
GPIO18   -->   IN4 (Motor B Direction)
GPIO5    -->   ENA (Motor A Speed)
GPIO23   -->   ENB (Motor B Speed)
VIN      -->   12V+ (Motor Power)
GND      -->   GND
```

### 2.2 Code điều khiển 4 motor

```cpp
// File: four_motor_control.ino
#include <Arduino.h>

// Motor A pins (Left side)
#define IN1_PIN 22
#define IN2_PIN 21
#define ENA_PIN 5

// Motor B pins (Right side)
#define IN3_PIN 19
#define IN4_PIN 18
#define ENB_PIN 23

// Movement commands
#define STOP     0
#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4

class MotorController {
private:
  int defaultSpeed = 200;

public:
  void init() {
    // Configure direction pins
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    // Configure PWM pins
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENB_PIN, OUTPUT);

    Serial.println("Motor Controller Initialized");
  }

  void setSpeed(int speedA, int speedB) {
    speedA = constrain(speedA, 0, 255);
    speedB = constrain(speedB, 0, 255);

    analogWrite(ENA_PIN, speedA);
    analogWrite(ENB_PIN, speedB);
  }

  void move(int direction, int speed = -1) {
    if(speed == -1) speed = defaultSpeed;

    switch(direction) {
      case FORWARD:
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
        setSpeed(speed, speed);
        Serial.println("Moving FORWARD");
        break;

      case BACKWARD:
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
        setSpeed(speed, speed);
        Serial.println("Moving BACKWARD");
        break;

      case LEFT:
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
        setSpeed(speed, speed);
        Serial.println("Turning LEFT");
        break;

      case RIGHT:
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
        setSpeed(speed, speed);
        Serial.println("Turning RIGHT");
        break;

      case STOP:
      default:
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
        setSpeed(0, 0);
        Serial.println("STOPPED");
        break;
    }
  }

  void setDefaultSpeed(int speed) {
    defaultSpeed = constrain(speed, 0, 255);
  }
};

MotorController motors;

void setup() {
  Serial.begin(115200);
  motors.init();

  Serial.println("Four Motor Controller Ready");
  Serial.println("Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop");
}

void loop() {
  if(Serial.available()) {
    char command = Serial.read();

    switch(command) {
      case 'F':
      case 'f':
        motors.move(FORWARD);
        break;
      case 'B':
      case 'b':
        motors.move(BACKWARD);
        break;
      case 'L':
      case 'l':
        motors.move(LEFT);
        break;
      case 'R':
      case 'r':
        motors.move(RIGHT);
        break;
      case 'S':
      case 's':
        motors.move(STOP);
        break;
      default:
        Serial.println("Invalid command");
        break;
    }
  }

  // Demo sequence (comment out for manual control)
  /*
  motors.move(FORWARD, 150);
  delay(2000);
  motors.move(RIGHT, 180);
  delay(1000);
  motors.move(BACKWARD, 150);
  delay(2000);
  motors.move(LEFT, 180);
  delay(1000);
  motors.move(STOP);
  delay(2000);
  */
}
```

### 2.3 Tính năng nâng cao:

#### 2.3.1 Smooth Speed Control

```cpp
void smoothSpeedChange(int currentSpeed, int targetSpeed, int motorPin) {
  int step = (targetSpeed > currentSpeed) ? 5 : -5;

  for(int speed = currentSpeed;
      abs(speed - targetSpeed) > abs(step);
      speed += step) {
    analogWrite(motorPin, speed);
    delay(50);
  }

  analogWrite(motorPin, targetSpeed);
}
```

#### 2.3.2 Curved Movement

```cpp
void moveForwardLeft(int speedDiff = 50) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  setSpeed(defaultSpeed - speedDiff, defaultSpeed); // Left slower
}

void moveForwardRight(int speedDiff = 50) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);

  setSpeed(defaultSpeed, defaultSpeed - speedDiff); // Right slower
}
```

---

## Phần 3: Tích hợp FreeRTOS

### 3.1 Tại sao cần FreeRTOS?

#### Vấn đề với code tuần tự:

- Blocking operations (delay()) làm đình trệ toàn bộ hệ thống
- Khó xử lý nhiều task đồng thời
- Không có priority management
- Khó debug và maintain

#### Ưu điểm của FreeRTOS:

- **Multitasking**: Nhiều task chạy song song
- **Real-time**: Đảm bảo timing requirements
- **Priority-based scheduling**: Task quan trọng được ưu tiên
- **Inter-task communication**: Queue, Semaphore, Mutex

### 3.2 Kiến trúc FreeRTOS cho ESP32 Car

```
Core 0                    Core 1
├── WiFi Task            ├── Motor Control Task
├── E-Ra Communication   ├── Sensor Reading Task
└── Logging Task         └── Safety Monitor Task
```

### 3.3 Code FreeRTOS cơ bản

```cpp
// File: freertos_motor_control.ino
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Motor control pins
#define IN1_PIN 22
#define IN2_PIN 21
#define IN3_PIN 19
#define IN4_PIN 18
#define ENA_PIN 5
#define ENB_PIN 23

// Motor command structure
typedef struct {
  int direction;  // 0=stop, 1=forward, 2=backward, 3=left, 4=right
  int speedA;
  int speedB;
  int duration;   // milliseconds
} MotorCommand_t;

// FreeRTOS handles
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t commandTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;
QueueHandle_t motorCommandQueue;
SemaphoreHandle_t motorMutex;

// Global variables
volatile bool systemActive = true;
int currentSpeedA = 0;
int currentSpeedB = 0;

// Task functions prototypes
void motorControlTask(void *parameter);
void commandProcessingTask(void *parameter);
void systemMonitorTask(void *parameter);

// Motor control functions
void setupMotorPins();
void executeMotorCommand(MotorCommand_t *cmd);
void stopMotors();

void setup() {
  Serial.begin(115200);
  Serial.println("FreeRTOS Motor Control System Starting...");

  // Initialize hardware
  setupMotorPins();

  // Create FreeRTOS objects
  motorCommandQueue = xQueueCreate(10, sizeof(MotorCommand_t));
  motorMutex = xSemaphoreCreateMutex();

  if(motorCommandQueue == NULL || motorMutex == NULL) {
    Serial.println("Failed to create FreeRTOS objects!");
    while(1);
  }

  // Create tasks
  xTaskCreatePinnedToCore(
    motorControlTask,      // Task function
    "MotorControl",        // Task name
    4096,                  // Stack size
    NULL,                  // Parameters
    2,                     // Priority (0-25, higher is more priority)
    &motorTaskHandle,      // Task handle
    1                      // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(
    commandProcessingTask,
    "CommandProcessing",
    4096,
    NULL,
    1,
    &commandTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    systemMonitorTask,
    "SystemMonitor",
    2048,
    NULL,
    0,
    &monitorTaskHandle,
    0
  );

  Serial.println("FreeRTOS tasks created successfully");
}

void loop() {
  // Main loop is empty - all work done in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Task 1: Motor Control (High Priority)
void motorControlTask(void *parameter) {
  MotorCommand_t receivedCommand;
  TickType_t lastWakeTime = xTaskGetTickCount();

  Serial.println("Motor Control Task started on core " + String(xPortGetCoreID()));

  while(true) {
    // Check for new commands (non-blocking)
    if(xQueueReceive(motorCommandQueue, &receivedCommand, pdMS_TO_TICKS(50)) == pdTRUE) {

      // Take mutex to ensure thread-safe motor control
      if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        executeMotorCommand(&receivedCommand);
        xSemaphoreGive(motorMutex);
      }
    }

    // Run at 50Hz (20ms period)
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
  }
}

// Task 2: Command Processing (Medium Priority)
void commandProcessingTask(void *parameter) {
  Serial.println("Command Processing Task started on core " + String(xPortGetCoreID()));

  while(true) {
    if(Serial.available()) {
      char command = Serial.read();
      MotorCommand_t motorCmd;

      motorCmd.speedA = 200;
      motorCmd.speedB = 200;
      motorCmd.duration = 0; // Continuous

      switch(command) {
        case 'F':
        case 'f':
          motorCmd.direction = 1; // Forward
          break;
        case 'B':
        case 'b':
          motorCmd.direction = 2; // Backward
          break;
        case 'L':
        case 'l':
          motorCmd.direction = 3; // Left
          break;
        case 'R':
        case 'r':
          motorCmd.direction = 4; // Right
          break;
        case 'S':
        case 's':
          motorCmd.direction = 0; // Stop
          break;
        default:
          continue; // Skip invalid commands
      }

      // Send command to motor task
      if(xQueueSend(motorCommandQueue, &motorCmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        Serial.println("Failed to send command to motor task");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Check commands every 50ms
  }
}

// Task 3: System Monitor (Low Priority)
void systemMonitorTask(void *parameter) {
  Serial.println("System Monitor Task started on core " + String(xPortGetCoreID()));

  while(true) {
    // Print system statistics
    Serial.println("=== System Status ===");
    Serial.println("Free heap: " + String(esp_get_free_heap_size()) + " bytes");
    Serial.println("Queue messages waiting: " + String(uxQueueMessagesWaiting(motorCommandQueue)));
    Serial.println("Current Speed A: " + String(currentSpeedA));
    Serial.println("Current Speed B: " + String(currentSpeedB));
    Serial.println("High water mark - Motor Task: " + String(uxTaskGetStackHighWaterMark(motorTaskHandle)));
    Serial.println("High water mark - Command Task: " + String(uxTaskGetStackHighWaterMark(commandTaskHandle)));
    Serial.println("=====================");

    vTaskDelay(pdMS_TO_TICKS(5000)); // Print every 5 seconds
  }
}

// Hardware setup
void setupMotorPins() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);

  stopMotors();
  Serial.println("Motor pins initialized");
}

// Execute motor command
void executeMotorCommand(MotorCommand_t *cmd) {
  // Update global speed tracking
  currentSpeedA = cmd->speedA;
  currentSpeedB = cmd->speedB;

  // Set motor speeds
  analogWrite(ENA_PIN, cmd->speedA);
  analogWrite(ENB_PIN, cmd->speedB);

  // Set direction
  switch(cmd->direction) {
    case 1: // Forward
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);
      Serial.println("Moving FORWARD");
      break;

    case 2: // Backward
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, HIGH);
      digitalWrite(IN3_PIN, HIGH);
      digitalWrite(IN4_PIN, LOW);
      Serial.println("Moving BACKWARD");
      break;

    case 3: // Left
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
      digitalWrite(IN3_PIN, HIGH);
      digitalWrite(IN4_PIN, LOW);
      Serial.println("Turning LEFT");
      break;

    case 4: // Right
      digitalWrite(IN1_PIN, LOW);
      digitalWrite(IN2_PIN, HIGH);
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);
      Serial.println("Turning RIGHT");
      break;

    case 0: // Stop
    default:
      stopMotors();
      break;
  }
}

// Stop all motors
void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);

  currentSpeedA = 0;
  currentSpeedB = 0;

  Serial.println("Motors stopped");
}
```

### 3.4 FreeRTOS Best Practices:

#### 3.4.1 Task Priority Guidelines:

```
Priority 3: Critical safety tasks (emergency stop)
Priority 2: Motor control, sensor reading
Priority 1: Communication, user interface
Priority 0: Monitoring, logging, housekeeping
```

#### 3.4.2 Memory Management:

```cpp
// Check stack usage
void checkStackUsage() {
  UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(NULL);
  if(stackRemaining < 512) {
    Serial.println("WARNING: Low stack space!");
  }
}
```

#### 3.4.3 Task Communication:

```cpp
// Queue for commands
QueueHandle_t commandQueue = xQueueCreate(10, sizeof(Command_t));

// Mutex for shared resources
SemaphoreHandle_t sensorMutex = xSemaphoreCreateMutex();

// Binary semaphore for synchronization
SemaphoreHandle_t startSemaphore = xSemaphoreCreateBinary();
```

---

## Phần 4: Tích hợp E-Ra IoT Platform

### 4.1 Giới thiệu E-Ra Platform

E-Ra là một IoT platform mạnh mẽ cho phép:

- **Remote Control**: Điều khiển thiết bị từ xa qua app/web
- **Real-time Monitoring**: Giám sát dữ liệu real-time
- **Cloud Integration**: Lưu trữ và phân tích dữ liệu
- **Automation**: Tự động hóa dựa trên rules/conditions

### 4.2 Kiến trúc hệ thống với E-Ra

```
[Mobile App] --> [E-Ra Cloud] --> [MQTT] --> [ESP32 Car] --> [Motors]
     ^                                           |
     |                                           v
[Web Dashboard] <-- [Data Analytics] <-- [Sensor Data]
```

### 4.3 Setup E-Ra Project

#### 4.3.1 Tạo project trên E-Ra Dashboard:

1. Đăng ký tài khoản tại [era.eoh.io](https://era.eoh.io)
2. Tạo project mới: "ESP32 Car Control"
3. Lấy Auth Token
4. Cấu hình Virtual Pins:
   - V0: Forward control (Button)
   - V1: Backward control (Button)
   - V2: Left control (Button)
   - V3: Right control (Button)
   - V4: Speed A control (Slider 0-255)
   - V5: Speed B control (Slider 0-255)
   - V6: Battery voltage (Display)
   - V7: System status (Display)

### 4.4 Code tích hợp E-Ra với FreeRTOS

```cpp
// File: era_freertos_car.ino
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"

// E-Ra includes
#define ERA_DEBUG
#include <ERa.hpp>
#include <WiFi.h>

// WiFi credentials
const char ssid[] = "YOUR_WIFI_SSID";
const char pass[] = "YOUR_WIFI_PASSWORD";

// E-Ra configuration
#define ERA_AUTH_TOKEN "YOUR_ERA_TOKEN"

// Motor control pins
#define IN1_PIN 22
#define IN2_PIN 21
#define IN3_PIN 19
#define IN4_PIN 18
#define ENA_PIN 5
#define ENB_PIN 23

// PWM Configuration
#define PWM_FREQ 1000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define ENA_CHANNEL LEDC_CHANNEL_0
#define ENB_CHANNEL LEDC_CHANNEL_1
#define LEDC_TIMER_NUM LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

// Motor command structure
typedef struct {
  bool forward;
  bool backward;
  bool left;
  bool right;
  uint8_t speedA;
  uint8_t speedB;
} MotorCommand_t;

// FreeRTOS handles
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t eraConnectionTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
QueueHandle_t motorCommandQueue;
SemaphoreHandle_t motorMutex;

// Global variables
MotorCommand_t currentCommand = {false, false, false, false, 180, 180};
bool motorActive = false;
float batteryVoltage = 0.0;
String systemStatus = "Initializing";

// Function prototypes
void motorControlTask(void *parameter);
void eraConnectionTask(void *parameter);
void sensorReadingTask(void *parameter);
void setupMotorPins();
void executeMotorCommand(const MotorCommand_t *cmd);
void stopMotors();
void setMotorSpeed(ledc_channel_t channel, uint8_t speed);

void setup() {
  Serial.begin(115200);
  Serial.println("E-Ra FreeRTOS Car Control System Starting...");

  // Initialize hardware
  setupMotorPins();

  // Create FreeRTOS objects
  motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t));
  motorMutex = xSemaphoreCreateMutex();

  if(motorCommandQueue == NULL || motorMutex == NULL) {
    Serial.println("Failed to create FreeRTOS objects!");
    while(1);
  }

  // Create tasks
  xTaskCreatePinnedToCore(
    motorControlTask,
    "MotorControl",
    4096,
    NULL,
    2, // High priority for motor control
    &motorControlTaskHandle,
    1  // Pin to core 1
  );

  xTaskCreatePinnedToCore(
    eraConnectionTask,
    "ERAConnection",
    8192,
    NULL,
    1, // Lower priority for network operations
    &eraConnectionTaskHandle,
    0  // Pin to core 0
  );

  xTaskCreatePinnedToCore(
    sensorReadingTask,
    "SensorReading",
    2048,
    NULL,
    1,
    &sensorTaskHandle,
    0
  );

  systemStatus = "System Ready";
  Serial.println("FreeRTOS tasks created successfully");
}

void loop() {
  // Main loop is empty as all work is done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Task for handling motor control
void motorControlTask(void *parameter) {
  MotorCommand_t receivedCommand;
  TickType_t lastMotorUpdate = xTaskGetTickCount();
  const TickType_t motorTimeout = pdMS_TO_TICKS(200); // 200ms timeout

  Serial.println("Motor Control Task started on core " + String(xPortGetCoreID()));

  while(true) {
    // Check for new commands
    if(xQueueReceive(motorCommandQueue, &receivedCommand, pdMS_TO_TICKS(10)) == pdTRUE) {
      if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentCommand = receivedCommand;
        motorActive = (receivedCommand.forward || receivedCommand.backward ||
                      receivedCommand.left || receivedCommand.right);
        lastMotorUpdate = xTaskGetTickCount();

        executeMotorCommand(&currentCommand);
        xSemaphoreGive(motorMutex);
      }
    }

    // Check for motor timeout (safety feature)
    if(motorActive && (xTaskGetTickCount() - lastMotorUpdate) > motorTimeout) {
      if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motorActive = false;
        stopMotors();
        systemStatus = "Motor timeout - stopped";
        Serial.println("Motor timeout - stopping motors");
        xSemaphoreGive(motorMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
  }
}

// Task for handling E-Ra connection and communication
void eraConnectionTask(void *parameter) {
  Serial.println("E-Ra Connection Task started on core " + String(xPortGetCoreID()));

  // Initialize WiFi
  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    systemStatus = "Connecting WiFi";
  }

  Serial.println("WiFi Connected!");
  Serial.println("IP: " + WiFi.localIP().toString());
  systemStatus = "WiFi Connected";

  // Initialize E-Ra
  ERa.begin(ssid, pass);

  while(true) {
    ERa.run();
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to run
  }
}

// Task for reading sensors
void sensorReadingTask(void *parameter) {
  Serial.println("Sensor Reading Task started on core " + String(xPortGetCoreID()));

  while(true) {
    // Read battery voltage (example - adjust based on your setup)
    int adcValue = analogRead(A0);
    batteryVoltage = (adcValue * 3.3 * 4.0) / 4095.0; // Voltage divider calculation

    // Send data to E-Ra every 2 seconds
    static unsigned long lastSensorUpdate = 0;
    if(millis() - lastSensorUpdate > 2000) {
      ERa.virtualWrite(V6, batteryVoltage);
      ERa.virtualWrite(V7, systemStatus);
      lastSensorUpdate = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Read sensors every 500ms
  }
}

// E-Ra Virtual Pin handlers
ERA_WRITE(V0) {
  // Forward movement control
  uint8_t value = param.getInt();
  Serial.printf("V0 (Forward): %d\n", value);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.forward = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send forward command to queue");
  }

  ERa.virtualWrite(V0, value);
}

ERA_WRITE(V1) {
  // Backward movement control
  uint8_t value = param.getInt();
  Serial.printf("V1 (Backward): %d\n", value);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.backward = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send backward command to queue");
  }

  ERa.virtualWrite(V1, value);
}

ERA_WRITE(V2) {
  // Left movement control
  uint8_t value = param.getInt();
  Serial.printf("V2 (Left): %d\n", value);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.left = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send left command to queue");
  }

  ERa.virtualWrite(V2, value);
}

ERA_WRITE(V3) {
  // Right movement control
  uint8_t value = param.getInt();
  Serial.printf("V3 (Right): %d\n", value);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.right = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send right command to queue");
  }

  ERa.virtualWrite(V3, value);
}

ERA_WRITE(V4) {
  // Motor A speed control (ENA)
  uint8_t speed = param.getInt();
  Serial.printf("V4 (Speed A): %d\n", speed);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.speedA = constrain(speed, 0, 255);
    if(cmd.speedA > 0 && cmd.speedA < 80) {
      cmd.speedA = 80; // Minimum speed to overcome motor inertia
    }
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send speed A command to queue");
  }

  ERa.virtualWrite(V4, speed);
}

ERA_WRITE(V5) {
  // Motor B speed control (ENB)
  uint8_t speed = param.getInt();
  Serial.printf("V5 (Speed B): %d\n", speed);

  MotorCommand_t cmd;
  if(xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = currentCommand;
    cmd.speedB = constrain(speed, 0, 255);
    if(cmd.speedB > 0 && cmd.speedB < 80) {
      cmd.speedB = 80; // Minimum speed to overcome motor inertia
    }
    xSemaphoreGive(motorMutex);
  }

  if(xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("Failed to send speed B command to queue");
  }

  ERa.virtualWrite(V5, speed);
}

// E-Ra connection status handlers
ERA_CONNECTED() {
  Serial.println("E-Ra connected!");
  systemStatus = "E-Ra Connected";
  ERa.virtualWrite(V7, systemStatus);
}

ERA_DISCONNECTED() {
  Serial.println("E-Ra disconnected!");
  systemStatus = "E-Ra Disconnected";
}

// Hardware setup functions
void setupMotorPins() {
  // Configure motor direction pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Configure LEDC timer for PWM
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = LEDC_TIMER_NUM,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Configure LEDC channels
  ledc_channel_config_t ena_channel = {
    .gpio_num = ENA_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ENA_CHANNEL,
    .timer_sel = LEDC_TIMER_NUM,
    .duty = 0,
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ena_channel));

  ledc_channel_config_t enb_channel = {
    .gpio_num = ENB_PIN,
    .speed_mode = LEDC_MODE,
    .channel = ENB_CHANNEL,
    .timer_sel = LEDC_TIMER_NUM,
    .duty = 0,
    .hpoint = 0
  };
  ESP_ERROR_CHECK(ledc_channel_config(&enb_channel));

  stopMotors();
  Serial.println("Motor pins initialized with LEDC PWM");
}

void setMotorSpeed(ledc_channel_t channel, uint8_t speed) {
  esp_err_t err = ledc_set_duty(LEDC_MODE, channel, speed);
  if(err == ESP_OK) {
    ledc_update_duty(LEDC_MODE, channel);
  } else {
    Serial.printf("Error setting PWM duty for channel %d: %s\n", channel, esp_err_to_name(err));
  }
}

void executeMotorCommand(const MotorCommand_t *cmd) {
  // Set motor speeds
  setMotorSpeed(ENA_CHANNEL, cmd->speedA);
  setMotorSpeed(ENB_CHANNEL, cmd->speedB);

  // Debug output
  Serial.printf("Motor speeds - ENA: %d, ENB: %d\n", cmd->speedA, cmd->speedB);

  // Determine movement direction
  if(cmd->forward && !cmd->backward && !cmd->left && !cmd->right) {
    // Forward only
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    systemStatus = "Moving Forward";
  }
  else if(cmd->backward && !cmd->forward && !cmd->left && !cmd->right) {
    // Backward only
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    systemStatus = "Moving Backward";
  }
  else if(cmd->left && !cmd->right && !cmd->forward && !cmd->backward) {
    // Left turn only
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    systemStatus = "Turning Left";
  }
  else if(cmd->right && !cmd->left && !cmd->forward && !cmd->backward) {
    // Right turn only
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    systemStatus = "Turning Right";
  }
  else if(cmd->forward && cmd->left) {
    // Forward + Left
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    systemStatus = "Forward-Left";
  }
  else if(cmd->forward && cmd->right) {
    // Forward + Right
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    systemStatus = "Forward-Right";
  }
  else if(cmd->backward && cmd->left) {
    // Backward + Left
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    systemStatus = "Backward-Left";
  }
  else if(cmd->backward && cmd->right) {
    // Backward + Right
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    systemStatus = "Backward-Right";
  }
  else {
    // Stop or invalid combination
    stopMotors();
  }
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  setMotorSpeed(ENA_CHANNEL, 0);
  setMotorSpeed(ENB_CHANNEL, 0);

  systemStatus = "Motors Stopped";
  Serial.println("Motors stopped");
}
```

### 4.5 E-Ra Mobile App Setup

#### 4.5.1 Tạo Dashboard:

1. **Control Panel**:

   - 4 buttons: Forward, Backward, Left, Right
   - 2 sliders: Speed A, Speed B
   - Emergency Stop button

2. **Monitoring Panel**:
   - Battery voltage gauge
   - System status display
   - Connection status indicator
   - Real-time speed indicators

#### 4.5.2 Widget Configuration:

```json
{
  "V0": {
    "type": "button",
    "label": "Forward",
    "color": "#4CAF50",
    "mode": "push"
  },
  "V4": {
    "type": "slider",
    "label": "Speed A",
    "min": 0,
    "max": 255,
    "step": 5
  },
  "V6": {
    "type": "gauge",
    "label": "Battery (V)",
    "min": 0,
    "max": 12,
    "color": "#FF9800"
  }
}
```

---

## Phần 5: Optimization và Best Practices

### 5.1 PWM Optimization

#### 5.1.1 Frequency Selection:

```cpp
// Motor control frequency optimization
#define PWM_FREQ_LOW    1000    // For smooth operation
#define PWM_FREQ_MID    5000    // For responsive control
#define PWM_FREQ_HIGH   10000   // For high precision (may cause noise)

// Recommended: 1000Hz for most DC motors
```

#### 5.1.2 Resolution vs Performance:

```cpp
// 8-bit resolution (0-255) - Good balance
#define PWM_8BIT  LEDC_TIMER_8_BIT   // 256 steps
// 10-bit resolution (0-1023) - Higher precision
#define PWM_10BIT LEDC_TIMER_10_BIT  // 1024 steps
// 12-bit resolution (0-4095) - Maximum precision
#define PWM_12BIT LEDC_TIMER_12_BIT  // 4096 steps
```

### 5.2 Motor Control Best Practices

#### 5.2.1 Gradual Speed Changes:

```cpp
void gradualSpeedChange(int currentSpeed, int targetSpeed, ledc_channel_t channel) {
  const int STEP_SIZE = 10;
  const int STEP_DELAY = 20; // ms

  if(currentSpeed < targetSpeed) {
    for(int speed = currentSpeed; speed < targetSpeed; speed += STEP_SIZE) {
      setMotorSpeed(channel, speed);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
    }
  } else {
    for(int speed = currentSpeed; speed > targetSpeed; speed -= STEP_SIZE) {
      setMotorSpeed(channel, speed);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY));
    }
  }

  setMotorSpeed(channel, targetSpeed);
}
```

#### 5.2.2 Motor Calibration:

```cpp
typedef struct {
  uint8_t minSpeed;     // Minimum speed to overcome friction
  uint8_t maxSpeed;     // Maximum safe speed
  float calibration;    // Speed correction factor
} MotorCalibration_t;

MotorCalibration_t motorA = {80, 255, 1.0};
MotorCalibration_t motorB = {85, 255, 0.95}; // Motor B runs 5% faster

uint8_t calibrateSpeed(uint8_t speed, MotorCalibration_t* cal) {
  if(speed == 0) return 0;

  // Apply minimum speed threshold
  if(speed < cal->minSpeed) speed = cal->minSpeed;

  // Apply calibration factor
  speed = (uint8_t)(speed * cal->calibration);

  // Ensure within limits
  return constrain(speed, cal->minSpeed, cal->maxSpeed);
}
```

### 5.3 FreeRTOS Optimization

#### 5.3.1 Task Stack Monitoring:

```cpp
void monitorTaskStacks() {
  const int MIN_STACK_WORDS = 512;

  UBaseType_t motorStack = uxTaskGetStackHighWaterMark(motorTaskHandle);
  UBaseType_t eraStack = uxTaskGetStackHighWaterMark(eraTaskHandle);

  if(motorStack < MIN_STACK_WORDS) {
    Serial.println("WARNING: Motor task low stack!");
  }

  if(eraStack < MIN_STACK_WORDS) {
    Serial.println("WARNING: E-Ra task low stack!");
  }

  Serial.printf("Stack usage - Motor: %d, E-Ra: %d\n", motorStack, eraStack);
}
```

#### 5.3.2 CPU Usage Monitoring:

```cpp
void printCPUUsage() {
  static uint32_t lastCheck = 0;
  static uint32_t lastIdleTime = 0;

  uint32_t currentTime = millis();
  uint32_t currentIdleTime = esp_timer_get_time();

  if(currentTime - lastCheck >= 1000) { // Every second
    uint32_t totalTime = currentTime - lastCheck;
    uint32_t idleTime = (currentIdleTime - lastIdleTime) / 1000;

    uint32_t cpuUsage = 100 - ((idleTime * 100) / totalTime);
    Serial.printf("CPU Usage: %d%%\n", cpuUsage);

    lastCheck = currentTime;
    lastIdleTime = currentIdleTime;
  }
}
```

### 5.4 Power Management

#### 5.4.1 Battery Monitoring:

```cpp
class BatteryMonitor {
private:
  const float VOLTAGE_DIVIDER = 4.0;  // Resistor divider ratio
  const float ADC_REF = 3.3;          // ADC reference voltage
  const int ADC_RESOLUTION = 4095;    // 12-bit ADC

  const float BATTERY_MIN = 9.0;      // Minimum operating voltage
  const float BATTERY_MAX = 12.6;     // Maximum voltage (fully charged)
  const float BATTERY_LOW = 10.5;     // Low battery warning

public:
  float readVoltage() {
    int adcValue = analogRead(A0);
    return (adcValue * ADC_REF * VOLTAGE_DIVIDER) / ADC_RESOLUTION;
  }

  int getBatteryPercentage() {
    float voltage = readVoltage();
    return map(voltage * 100, BATTERY_MIN * 100, BATTERY_MAX * 100, 0, 100);
  }

  bool isLowBattery() {
    return readVoltage() < BATTERY_LOW;
  }

  bool isCriticalBattery() {
    return readVoltage() < BATTERY_MIN;
  }
};
```

#### 5.4.2 Power Saving Mode:

```cpp
void enterPowerSaveMode() {
  // Stop all motors
  stopMotors();

  // Reduce CPU frequency
  setCpuFrequencyMhz(80); // Down from 240MHz

  // Turn off unnecessary peripherals
  WiFi.mode(WIFI_OFF);

  // Enter light sleep when idle
  esp_sleep_enable_timer_wakeup(1000000); // 1 second
  esp_light_sleep_start();
}
```

### 5.5 Error Handling và Debugging

#### 5.5.1 Comprehensive Error Handling:

```cpp
typedef enum {
  ERROR_NONE = 0,
  ERROR_MOTOR_FAULT,
  ERROR_COMMUNICATION,
  ERROR_LOW_BATTERY,
  ERROR_OVERHEAT,
  ERROR_SENSOR_FAULT
} SystemError_t;

class ErrorHandler {
private:
  SystemError_t currentError = ERROR_NONE;
  String errorMessages[6] = {
    "No Error",
    "Motor Fault Detected",
    "Communication Lost",
    "Low Battery Warning",
    "System Overheating",
    "Sensor Malfunction"
  };

public:
  void setError(SystemError_t error) {
    currentError = error;
    Serial.println("ERROR: " + errorMessages[error]);

    // Send error to E-Ra dashboard
    ERa.virtualWrite(V8, error);
    ERa.virtualWrite(V9, errorMessages[error]);

    // Take appropriate action
    handleError(error);
  }

  void handleError(SystemError_t error) {
    switch(error) {
      case ERROR_MOTOR_FAULT:
        stopMotors();
        break;
      case ERROR_LOW_BATTERY:
        // Reduce maximum speed
        break;
      case ERROR_COMMUNICATION:
        // Retry connection
        break;
      case ERROR_OVERHEAT:
        stopMotors();
        enterPowerSaveMode();
        break;
    }
  }

  SystemError_t getCurrentError() {
    return currentError;
  }

  void clearError() {
    currentError = ERROR_NONE;
  }
};
```

#### 5.5.2 Advanced Debugging:

```cpp
#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_WARNING 2
#define DEBUG_LEVEL_INFO    3
#define DEBUG_LEVEL_DEBUG   4

#define DEBUG_LEVEL DEBUG_LEVEL_INFO

#define DEBUG_ERROR(fmt, ...)   if(DEBUG_LEVEL >= DEBUG_LEVEL_ERROR)   Serial.printf("[ERROR] " fmt "\n", ##__VA_ARGS__)
#define DEBUG_WARNING(fmt, ...) if(DEBUG_LEVEL >= DEBUG_LEVEL_WARNING) Serial.printf("[WARN]  " fmt "\n", ##__VA_ARGS__)
#define DEBUG_INFO(fmt, ...)    if(DEBUG_LEVEL >= DEBUG_LEVEL_INFO)    Serial.printf("[INFO]  " fmt "\n", ##__VA_ARGS__)
#define DEBUG_DEBUG(fmt, ...)   if(DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG)   Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

// Usage examples:
DEBUG_ERROR("Motor fault detected on channel %d", channel);
DEBUG_WARNING("Battery voltage low: %.2fV", voltage);
DEBUG_INFO("E-Ra connected successfully");
DEBUG_DEBUG("PWM duty set to %d", duty);
```

### 5.6 Performance Metrics

#### 5.6.1 System Performance Monitoring:

```cpp
class PerformanceMonitor {
private:
  uint32_t commandsProcessed = 0;
  uint32_t errorsCount = 0;
  uint32_t uptimeSeconds = 0;
  float averageResponseTime = 0.0;

public:
  void updateMetrics() {
    uptimeSeconds = millis() / 1000;

    // Calculate commands per second
    static uint32_t lastCommandCount = 0;
    static uint32_t lastTime = 0;

    uint32_t currentTime = millis();
    if(currentTime - lastTime >= 1000) {
      uint32_t commandsPerSecond = commandsProcessed - lastCommandCount;

      DEBUG_INFO("Performance: %d cmd/s, %d errors, %.2fms avg response",
                 commandsPerSecond, errorsCount, averageResponseTime);

      lastCommandCount = commandsProcessed;
      lastTime = currentTime;

      // Send to E-Ra dashboard
      ERa.virtualWrite(V10, commandsPerSecond);
      ERa.virtualWrite(V11, errorsCount);
      ERa.virtualWrite(V12, averageResponseTime);
    }
  }

  void recordCommand(uint32_t responseTime) {
    commandsProcessed++;

    // Update moving average
    averageResponseTime = (averageResponseTime * 0.9) + (responseTime * 0.1);
  }

  void recordError() {
    errorsCount++;
  }
};
```

---

## 📋 Checklist và Troubleshooting

### Checklist trước khi chạy:

- [ ] Kiểm tra kết nối phần cứng
- [ ] Cấu hình WiFi credentials
- [ ] Thiết lập E-Ra Auth Token
- [ ] Test motor direction
- [ ] Kiểm tra battery voltage
- [ ] Verify FreeRTOS task stack sizes

### Common Issues và Solutions:

#### 1. Motor không quay:

```
Kiểm tra:
- Nguồn cấp cho motor (12V)
- Kết nối L298N
- PWM frequency (đề xuất 1000Hz)
- Minimum speed threshold (>= 80)
```

#### 2. Motor giật:

```
Solutions:
- Sử dụng LEDC driver thay vì analogWrite
- Giảm PWM frequency xuống 1000Hz
- Thêm capacitor lọc nguồn
- Implement gradual speed changes
```

#### 3. FreeRTOS crash:

```
Debug:
- Kiểm tra stack overflow
- Verify queue sizes
- Check mutex usage
- Monitor heap memory
```

#### 4. E-Ra connection issues:

```
Troubleshoot:
- Verify WiFi credentials
- Check Auth Token
- Monitor network stability
- Implement reconnection logic
```

---

## 🎓 Kết luận

Tutorial này đã hướng dẫn bạn từ những kiến thức cơ bản nhất về điều khiển motor đến việc xây dựng một hệ thống IoT car hoàn chỉnh với:

### Kiến thức đã học:

1. **Basic Motor Control**: PWM, direction control
2. **Multi-Motor System**: Coordination, timing
3. **FreeRTOS Integration**: Real-time, multitasking
4. **IoT Platform**: Remote control, monitoring
5. **Optimization**: Performance, reliability

### Next Steps:

- [ ] Thêm sensors (ultrasonic, camera)
- [ ] Implement autonomous navigation
- [ ] Add voice control
- [ ] Integrate machine learning
- [ ] Build swarm robotics

### 📚 Tài liệu tham khảo:

- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [E-Ra Platform Guide](https://era.eoh.io/docs)

---

**Happy Coding! 🚗💨**

_Được tạo bởi ESP32 IoT Development Team_

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h" // ESP32 LEDC driver for PWM

// Enable debug console
#define ERA_DEBUG

/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "9a2815e5-efc5-47a5-bf43-36c87785a121"

#if defined(BUTTON_PIN)
// Active low (false), Active high (true)
#define BUTTON_INVERT false
#define BUTTON_HOLD_TIMEOUT 5000UL

// This directive is used to specify whether the configuration should be erased.
// If it's set to true, the configuration will be erased.
#define ERA_ERASE_CONFIG false
#endif

#include <ERa.hpp>
#include <Automation/ERaSmart.hpp>
#include <Time/ERaEspTime.hpp>
const char ssid[] = "eoh.io";
const char pass[] = "Eoh@2020";

WiFiClient mbTcpClient;

ERaEspTime syncTime;
ERaSmart smart(ERa, syncTime);

#if defined(BUTTON_PIN)
#include <ERa/ERaButton.hpp>

ERaButton button;

#if ERA_VERSION_NUMBER >= ERA_VERSION_VAL(1, 6, 0)
static void eventButton(uint16_t pin, ButtonEventT event)
{
  if (event != ButtonEventT::BUTTON_ON_HOLD)
  {
    return;
  }
  ERa.switchToConfig(ERA_ERASE_CONFIG);
  (void)pin;
}
#elif ERA_VERSION_NUMBER >= ERA_VERSION_VAL(1, 2, 0)
static void eventButton(uint8_t pin, ButtonEventT event)
{
  if (event != ButtonEventT::BUTTON_ON_HOLD)
  {
    return;
  }
  ERa.switchToConfig(ERA_ERASE_CONFIG);
  (void)pin;
}
#else
static void eventButton(ButtonEventT event)
{
  if (event != ButtonEventT::BUTTON_ON_HOLD)
  {
    return;
  }
  ERa.switchToConfig(ERA_ERASE_CONFIG);
}
#endif

#if defined(ESP32)
#include <pthread.h>

pthread_t pthreadButton;

static void *handlerButton(void *args)
{
  for (;;)
  {
    button.run();
    ERaDelay(10);
  }
  pthread_exit(NULL);
}

void initButton()
{
  pinMode(BUTTON_PIN, INPUT);
  button.setButton(BUTTON_PIN, digitalRead, eventButton,
                   BUTTON_INVERT)
      .onHold(BUTTON_HOLD_TIMEOUT);
  pthread_create(&pthreadButton, NULL, handlerButton, NULL);
}
#elif defined(ESP8266)
#include <Ticker.h>

Ticker ticker;

static void handlerButton()
{
  button.run();
}

void initButton()
{
  pinMode(BUTTON_PIN, INPUT);
  button.setButton(BUTTON_PIN, digitalRead, eventButton,
                   BUTTON_INVERT)
      .onHold(BUTTON_HOLD_TIMEOUT);
  ticker.attach_ms(100, handlerButton);
}
#elif defined(ARDUINO_AMEBA)
#include <GTimer.h>

const uint32_t timerIdButton{0};

static void handlerButton(uint32_t data)
{
  button.run();
  (void)data;
}

void initButton()
{
  pinMode(BUTTON_PIN, INPUT);
  button.setButton(BUTTON_PIN, digitalReadArduino, eventButton,
                   BUTTON_INVERT)
      .onHold(BUTTON_HOLD_TIMEOUT);
  GTimer.begin(timerIdButton, (100 * 1000), handlerButton);
}
#endif
#endif

/* This function will run every time ERa is connected */
ERA_CONNECTED()
{
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED()
{
  ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

/* This function print uptime every second */
void timerEvent()
{
  ERA_LOG(ERA_PSTR("Timer"), ERA_PSTR("Uptime: %d"), ERaMillis() / 1000L);
}

#if defined(USE_BASE_TIME)
unsigned long getTimeCallback()
{
  // Please implement your own function
  // to get the current time in seconds.
  return 0;
}
#endif

// Motor control pins
#define IN1_PIN 22
#define IN2_PIN 21
#define IN3_PIN 19
#define IN4_PIN 18
#define ENA_PIN 5
#define ENB_PIN 23

// PWM Configuration - optimized for motor control
#define PWM_FREQ 1000                   // 1kHz - optimal for motor drivers
#define PWM_RESOLUTION LEDC_TIMER_8_BIT // 8-bit resolution (0-255)

// LEDC channels for motor control
#define ENA_CHANNEL LEDC_CHANNEL_0
#define ENB_CHANNEL LEDC_CHANNEL_1
#define LEDC_TIMER_NUM LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

// Motor control structure
typedef struct
{
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
QueueHandle_t motorCommandQueue;
SemaphoreHandle_t motorMutex;

// Global variables (removed volatile to fix compilation issues)
MotorCommand_t currentCommand = {false, false, false, false, 180, 180}; // Reduced from 150 to 180 for better performance
MotorCommand_t lastCommand = {false, false, false, false, 0, 0};        // Track last command for smooth transitions
bool motorActive = false;

// Motor control helper functions
void setMotorSpeed(ledc_channel_t channel, uint8_t speed);
void smoothSpeedTransition(uint8_t currentSpeed, uint8_t targetSpeed, ledc_channel_t channel);

// Function prototypes
void motorControlTask(void *parameter);
void eraConnectionTask(void *parameter);
void setupMotorPins();
void executeMotorCommand(const MotorCommand_t *cmd);
void stopMotors();

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting E-RA IoT Car Control System...");

  // Initialize motor pins
  setupMotorPins();

  // Create FreeRTOS objects
  motorCommandQueue = xQueueCreate(5, sizeof(MotorCommand_t));
  motorMutex = xSemaphoreCreateMutex();

  if (motorCommandQueue == NULL || motorMutex == NULL)
  {
    Serial.println("Failed to create FreeRTOS objects!");
    while (1)
      ;
  }

  // Create tasks
  xTaskCreatePinnedToCore(
      motorControlTask,
      "MotorControl",
      4096,
      NULL,
      2, // High priority for motor control
      &motorControlTaskHandle,
      1 // Pin to core 1
  );

  xTaskCreatePinnedToCore(
      eraConnectionTask,
      "ERAConnection",
      8192,
      NULL,
      1, // Lower priority for network operations
      &eraConnectionTaskHandle,
      0 // Pin to core 0
  );

  Serial.println("FreeRTOS tasks created successfully");
}

void loop()
{
  // Main loop is empty as all work is done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Task for handling motor control
void motorControlTask(void *parameter)
{
  MotorCommand_t receivedCommand;
  TickType_t lastMotorUpdate = xTaskGetTickCount();
  const TickType_t motorTimeout = pdMS_TO_TICKS(100); // 100ms timeout

  Serial.println("Motor Control Task started");

  while (true)
  {
    // Check for new commands
    if (xQueueReceive(motorCommandQueue, &receivedCommand, pdMS_TO_TICKS(10)) == pdTRUE)
    {
      if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        currentCommand = receivedCommand;
        motorActive = (receivedCommand.forward || receivedCommand.backward ||
                       receivedCommand.left || receivedCommand.right);
        lastMotorUpdate = xTaskGetTickCount();

        executeMotorCommand(&currentCommand);
        xSemaphoreGive(motorMutex);
      }
    }

    // Check for motor timeout (safety feature)
    if (motorActive && (xTaskGetTickCount() - lastMotorUpdate) > motorTimeout)
    {
      if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)) == pdTRUE)
      {
        motorActive = false;
        stopMotors();
        Serial.println("Motor timeout - stopping motors");
        xSemaphoreGive(motorMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
  }
}

// Task for handling E-RA connection and communication
void eraConnectionTask(void *parameter)
{
  /* Setup debug console */
#if defined(ERA_DEBUG)
  Serial.begin(115200);
#endif

#if defined(BUTTON_PIN)
  /* Initializing button. */
  initButton();
  /* Enable read/write WiFi credentials */
  ERa.setPersistent(true);
#endif

#if defined(USE_BASE_TIME)
  syncTime.setGetTimeCallback(getTimeCallback);
#endif

  /* Setup Client for Modbus TCP/IP */
  ERa.setModbusClient(mbTcpClient);

  Serial.println("E-RA Connection Task started");
  // Initialize E-RA (corrected parameter order)
  ERa.setScanWiFi(true);

  /* Initializing the ERa library. */
  ERa.begin(ssid, pass);

  while (true)
  {
    ERa.run();
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to run
  }
}

// E-RA Virtual Pin handlers
ERA_WRITE(V0)
{
  // Forward movement control
  uint8_t value = param.getInt();
  Serial.printf("V0 (Forward): %d\n", value);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    cmd.forward = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send forward command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V0, value);
}

ERA_WRITE(V1)
{
  // Backward movement control
  uint8_t value = param.getInt();
  Serial.printf("V1 (Backward): %d\n", value);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    cmd.backward = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send backward command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V1, value);
}

ERA_WRITE(V2)
{
  // Left movement control
  uint8_t value = param.getInt();
  Serial.printf("V2 (Left): %d\n", value);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    cmd.left = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send left command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V2, value);
}

ERA_WRITE(V3)
{
  // Right movement control
  uint8_t value = param.getInt();
  Serial.printf("V3 (Right): %d\n", value);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    cmd.right = (value == 1);
    xSemaphoreGive(motorMutex);
  }

  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send right command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V3, value);
}

ERA_WRITE(V4)
{
  // Motor A speed control (ENA)
  uint8_t speed = param.getInt();
  Serial.printf("V4 (Speed A): %d\n", speed);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    // Constrain speed to safe range and add minimum threshold
    cmd.speedA = constrain(speed, 0, 255);
    if (cmd.speedA > 0 && cmd.speedA < 80)
    {
      cmd.speedA = 80; // Minimum speed to overcome motor inertia
    }
    currentCommand.speedA = cmd.speedA; // Update global state
    xSemaphoreGive(motorMutex);
  }

  // Send updated command to motor queue for immediate effect
  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send speed A command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V4, speed);
}

ERA_WRITE(V5)
{
  // Motor B speed control (ENB)
  uint8_t speed = param.getInt();
  Serial.printf("V5 (Speed B): %d\n", speed);

  // Create a copy and modify it (thread-safe way)
  MotorCommand_t cmd;
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    cmd = currentCommand;
    // Constrain speed to safe range and add minimum threshold
    cmd.speedB = constrain(speed, 0, 255);
    if (cmd.speedB > 0 && cmd.speedB < 80)
    {
      cmd.speedB = 80; // Minimum speed to overcome motor inertia
    }
    currentCommand.speedB = cmd.speedB; // Update global state
    xSemaphoreGive(motorMutex);
  }

  // Send updated command to motor queue for immediate effect
  if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE)
  {
    Serial.println("Failed to send speed B command to queue");
  }

  // Send status back to E-RA
  ERa.virtualWrite(V5, speed);
}

// Initialize motor control pins
void setupMotorPins()
{
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
      .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Configure LEDC channels for motor speed control
  ledc_channel_config_t ena_channel = {
      .gpio_num = ENA_PIN,
      .speed_mode = LEDC_MODE,
      .channel = ENA_CHANNEL,
      .timer_sel = LEDC_TIMER_NUM,
      .duty = 0,
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ena_channel));

  ledc_channel_config_t enb_channel = {
      .gpio_num = ENB_PIN,
      .speed_mode = LEDC_MODE,
      .channel = ENB_CHANNEL,
      .timer_sel = LEDC_TIMER_NUM,
      .duty = 0,
      .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&enb_channel));

  // Initialize all pins to LOW/OFF state
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Set initial PWM duty to 0
  ledc_set_duty(LEDC_MODE, ENA_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, ENA_CHANNEL);
  ledc_set_duty(LEDC_MODE, ENB_CHANNEL, 0);
  ledc_update_duty(LEDC_MODE, ENB_CHANNEL);

  Serial.println("Motor pins initialized with LEDC PWM");
}

// Helper function to set motor speed with error checking
void setMotorSpeed(ledc_channel_t channel, uint8_t speed)
{
  esp_err_t err = ledc_set_duty(LEDC_MODE, channel, speed);
  if (err == ESP_OK)
  {
    ledc_update_duty(LEDC_MODE, channel);
  }
  else
  {
    Serial.printf("Error setting PWM duty for channel %d: %s\n", channel, esp_err_to_name(err));
  }
}

// Smooth speed transition to reduce motor jitter
void smoothSpeedTransition(uint8_t currentSpeed, uint8_t targetSpeed, ledc_channel_t channel)
{
  if (currentSpeed == targetSpeed)
  {
    return;
  }

  int step = (targetSpeed > currentSpeed) ? 5 : -5; // Gradual change
  int speed = currentSpeed;

  while (abs(speed - targetSpeed) > abs(step))
  {
    speed += step;
    setMotorSpeed(channel, speed);
    vTaskDelay(pdMS_TO_TICKS(2)); // Small delay for smooth transition
  }

  // Final set to exact target
  setMotorSpeed(channel, targetSpeed);
}

// Execute motor command based on current state
void executeMotorCommand(const MotorCommand_t *cmd)
{
  // Apply smooth speed transitions to reduce jitter
  if (cmd->speedA != lastCommand.speedA)
  {
    smoothSpeedTransition(lastCommand.speedA, cmd->speedA, ENA_CHANNEL);
  }
  if (cmd->speedB != lastCommand.speedB)
  {
    smoothSpeedTransition(lastCommand.speedB, cmd->speedB, ENB_CHANNEL);
  }

  // Update last command for next comparison
  lastCommand.speedA = cmd->speedA;
  lastCommand.speedB = cmd->speedB;

  // Debug output for speed values
  Serial.printf("Motor speeds - ENA: %d, ENB: %d\n", cmd->speedA, cmd->speedB);

  // Determine movement direction
  if (cmd->forward && !cmd->backward && !cmd->left && !cmd->right)
  {
    // Forward only
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    Serial.printf("Moving: Forward (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->backward && !cmd->forward && !cmd->left && !cmd->right)
  {
    // Backward only
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    Serial.printf("Moving: Backward (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->left && !cmd->right && !cmd->forward && !cmd->backward)
  {
    // Left turn only
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    Serial.printf("Moving: Left (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->right && !cmd->left && !cmd->forward && !cmd->backward)
  {
    // Right turn only
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    Serial.printf("Moving: Right (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->forward && cmd->left)
  {
    // Forward + Left
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    Serial.printf("Moving: Forward-Left (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->forward && cmd->right)
  {
    // Forward + Right
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    Serial.printf("Moving: Forward-Right (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->backward && cmd->left)
  {
    // Backward + Left
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    Serial.printf("Moving: Backward-Left (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else if (cmd->backward && cmd->right)
  {
    // Backward + Right
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    Serial.printf("Moving: Backward-Right (Speed A:%d, B:%d)\n", cmd->speedA, cmd->speedB);
  }
  else
  {
    // Stop or invalid combination
    stopMotors();
  }
}

// Stop all motors
void stopMotors()
{
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Stop PWM using helper function
  setMotorSpeed(ENA_CHANNEL, 0);
  setMotorSpeed(ENB_CHANNEL, 0);

  // Update last command state
  lastCommand.speedA = 0;
  lastCommand.speedB = 0;

  Serial.println("Motors stopped");
}
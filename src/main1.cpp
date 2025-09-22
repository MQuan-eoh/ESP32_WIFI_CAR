/*Nodemcu ESP8266 WIFI control car.
 * This code created by sritu hobby team.
 * https://srituhobby.com
 */

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
#include <Arduino.h>
#include <ERa.hpp>
#include <Automation/ERaSmart.hpp>
#include <Time/ERaEspTime.hpp>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
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
void carforward();
void carbackward();
void carturnleft();
void carturnright();
void cartankleft();
void cartankright();
void carStop();
void smartcar();
void updateLCDWithDistance();
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

// Motor PINs
#define ENA 5
#define IN1 16 // Changed from 22 to avoid I2C SCL conflict
#define IN2 17 // Changed from 21 to avoid I2C SDA conflict
#define IN3 19
#define IN4 18
#define ENB 23

// Fixed speed settings
#define MOTOR_SPEED 255     // Full speed for straight movement (0-255)
#define TURN_SPEED_HIGH 255 // Higher speed for outer wheel during turn
#define TURN_SPEED_LOW 120  // Lower speed for inner wheel during turn

// Tank turn speed settings for stationary rotation
#define TANK_SPEED_HIGH 255 // Higher speed motor for tank turn
#define TANK_SPEED_LOW 120  // Lower speed motor for tank turn (balanced force)

// LCD I2C settings - ESP32 30pin module default I2C pins
#define LCD_SDA 21 // GPIO21 - SDA pin
#define LCD_SCL 22 // GPIO22 - SCL pin
// #define LCD_ADDRESS 0x27 // Common I2C address for LCD1602, try 0x3F if 0x27 doesn't work

// HC-SR04 Ultrasonic Sensor pins
#define TRIG_PIN 2 // GPIO2 - Trigger pin
#define ECHO_PIN 4 // GPIO4 - Echo pin

// Warning LED pin
#define WARNING_LED 15 // GPIO15 - Warning LED pin

// Obstacle detection settings
#define MIN_DISTANCE 20      // Minimum safe distance in cm
#define CRITICAL_DISTANCE 10 // Critical distance for immediate stop

// Initialize LCD (columns, rows, I2C address)
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool car_forward = 0;
bool car_backward = 0;
bool car_left = 0;
bool car_right = 0;
bool car_tank_left = 0;
bool car_tank_right = 0;

// Variable to track current display status to avoid flickering
String currentStatus = "";

// Obstacle detection variables
float currentDistance = 0;
bool obstacleDetected = false;
bool ledState = false;
unsigned long lastLedBlink = 0;
unsigned long lastDistanceCheck = 0;
const unsigned long LED_BLINK_INTERVAL = 300;      // LED blink interval in ms
const unsigned long DISTANCE_CHECK_INTERVAL = 100; // Check distance every 100ms

// Function to display car status on LCD
void displayCarStatus(String status)
{
    // Only update LCD if status has changed to prevent flickering
    if (currentStatus != status)
    {
        currentStatus = status;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ESP32 Smart Car");
        lcd.setCursor(0, 1);
        lcd.print("Status: " + status);
    }
}

// Function to measure distance using HC-SR04
float measureDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    if (duration == 0)
    {
        return 999; // Return large value if no echo received
    }

    float distance = (duration * 0.034) / 2; // Convert to cm
    return distance;
}

// Function to handle LED warning for obstacles
void handleWarningLED()
{
    unsigned long currentTime = millis();

    if (obstacleDetected)
    {
        // Blink LED when obstacle detected
        if (currentTime - lastLedBlink >= LED_BLINK_INTERVAL)
        {
            ledState = !ledState;
            digitalWrite(WARNING_LED, ledState);
            lastLedBlink = currentTime;
        }
    }
    else
    {
        // Turn off LED when no obstacle
        digitalWrite(WARNING_LED, LOW);
        ledState = false;
    }
}

// Function to check for obstacles
void checkObstacles()
{
    unsigned long currentTime = millis();

    if (currentTime - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL)
    {
        currentDistance = measureDistance();

        if (currentDistance <= MIN_DISTANCE && currentDistance > 0)
        {
            obstacleDetected = true;
        }
        else
        {
            obstacleDetected = false;
        }

        lastDistanceCheck = currentTime;

        // Update LCD with distance info
        updateLCDWithDistance();
    }
}

// Function to update LCD with distance information
void updateLCDWithDistance()
{
    lcd.setCursor(0, 1);
    if (obstacleDetected)
    {
        lcd.print("WARN: " + String((int)currentDistance) + "cm     ");
    }
    else
    {
        lcd.print("Status: " + currentStatus + "    ");
    }
}

// Function to initialize LCD

void setup()
{
    Serial.begin(15200);

    // Initialize LCD first
    // Wire.begin(LCD_SDA, LCD_SCL); // Initialize I2C with custom pins
    lcd.init(); // initialize the lcd
    lcd.init();
    lcd.backlight();
    displayCarStatus("READY");
    delay(1000);

    // Initialize motor pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize HC-SR04 pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize Warning LED pin
    pinMode(WARNING_LED, OUTPUT);
    digitalWrite(WARNING_LED, LOW);

    displayCarStatus("CONNECTING");
    ERa.begin(ssid, pass);
    displayCarStatus("CONNECTED");
}

ERA_WRITE(V0)
{
    car_left = param.getInt();
    if (car_left == 1)
    {
        carturnleft();
    }
    else if (car_left == 0)
    {
        carStop();
    }
}

ERA_WRITE(V1)
{
    car_right = param.getInt();
    if (car_right == 1)
    {
        carturnright();
    }
    else if (car_right == 0)
    {
        carStop();
    }
}

ERA_WRITE(V2)
{
    car_backward = param.getInt();
    if (car_backward == 1)
    {
        carbackward();
    }
    else if (car_backward == 0)
    {
        carStop();
    }
}

ERA_WRITE(V3)
{
    car_forward = param.getInt();
    if (car_forward == 1)
    {
        carforward();
    }
    else if (car_forward == 0)
    {
        carStop();
    }
}

ERA_WRITE(V4)
{
    car_tank_left = param.getInt();
    if (car_tank_left == 1)
    {
        cartankleft();
    }
    else if (car_tank_left == 0)
    {
        carStop();
    }
}

ERA_WRITE(V5)
{
    car_tank_right = param.getInt();
    if (car_tank_right == 1)
    {
        cartankright();
    }
    else if (car_tank_right == 0)
    {
        carStop();
    }
}

void smartcar()
{
    if (car_forward == 0 && car_backward == 0 && car_left == 0 && car_right == 0 && car_tank_left == 0 && car_tank_right == 0)
    {
        // Only call displayCarStatus, don't call carStop to avoid redundant motor control
        displayCarStatus("STOPPED");
        // Set all motor pins to LOW (stop motors)
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        Serial.println("carstop");
    }
}
void loop()
{
    ERa.run();
    checkObstacles();   // Check for obstacles
    handleWarningLED(); // Handle LED warning
    smartcar();
}

void carforward()
{
    // Check for obstacles before moving forward
    if (currentDistance <= CRITICAL_DISTANCE && currentDistance > 0)
    {
        // Stop immediately if critical distance reached
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        displayCarStatus("BLOCKED!");
        return;
    }

    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    displayCarStatus("FORWARD");
}
void carbackward()
{
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    displayCarStatus("BACKWARD");
}
void carturnright()
{
    // Check for obstacles before turning (turning forward motion)
    if (currentDistance <= CRITICAL_DISTANCE && currentDistance > 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        displayCarStatus("BLOCKED!");
        return;
    }

    // Right turn while moving forward: right motor slower, left motor faster
    analogWrite(ENA, TURN_SPEED_LOW);  // Right motor (IN1,IN2) slower
    analogWrite(ENB, TURN_SPEED_HIGH); // Left motor (IN3,IN4) faster
    digitalWrite(IN1, LOW);            // Right motor forward (same as carforward)
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); // Left motor forward (same as carforward)
    digitalWrite(IN4, HIGH);
    displayCarStatus("TURN RIGHT");
}
void carturnleft()
{
    // Check for obstacles before turning (turning forward motion)
    if (currentDistance <= CRITICAL_DISTANCE && currentDistance > 0)
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        displayCarStatus("BLOCKED!");
        return;
    }

    // Left turn while moving forward: left motor slower, right motor faster
    analogWrite(ENA, TURN_SPEED_HIGH); // Right motor (IN1,IN2) faster
    analogWrite(ENB, TURN_SPEED_LOW);  // Left motor (IN3,IN4) slower
    digitalWrite(IN1, LOW);            // Right motor forward (same as carforward)
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); // Left motor forward (same as carforward)
    digitalWrite(IN4, HIGH);
    displayCarStatus("TURN LEFT");
}
void carStop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    displayCarStatus("STOPPED");
}

void cartankleft()
{
    // Tank turn right: left motor forward, right motor backward
    // Differential speed creates rotation while keeping car stationary
    analogWrite(ENA, TANK_SPEED_LOW);  // Left motor forward (faster)
    analogWrite(ENB, TANK_SPEED_HIGH); // Right motor backward (slower)
    digitalWrite(IN1, LOW);            // Left motor forward
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); // Right motor backward
    digitalWrite(IN4, LOW);
    displayCarStatus("TANK LEFT");
}

void cartankright()
{
    // Tank turn left: left motor backward, right motor forward
    // Differential speed creates rotation while keeping car stationary
    analogWrite(ENA, TANK_SPEED_LOW);  // Left motor backward (slower)
    analogWrite(ENB, TANK_SPEED_HIGH); // Right motor forward (faster)
    digitalWrite(IN1, HIGH);           // Left motor backward
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); // Right motor forward
    digitalWrite(IN4, HIGH);
    displayCarStatus("TANK RIGHT");
}
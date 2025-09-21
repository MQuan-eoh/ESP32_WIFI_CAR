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
#define IN1 22
#define IN2 21
#define IN3 19
#define IN4 18
#define ENB 23

// Fixed speed settings
#define MOTOR_SPEED 255     // Full speed for straight movement (0-255)
#define TURN_SPEED_HIGH 225 // Higher speed for outer wheel during turn
#define TURN_SPEED_LOW 125  // Lower speed for inner wheel during turn

bool car_forward = 0;
bool car_backward = 0;
bool car_left = 0;
bool car_right = 0;
bool car_tank_left = 0;
bool car_tank_right = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    ERa.begin(ssid, pass);
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
        carStop();
        Serial.println("carstop");
    }
}
void loop()
{
    ERa.run();
    smartcar();
}

void carforward()
{
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void carbackward()
{
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void carturnleft()
{
    // Left turn: left motor slower, right motor faster
    analogWrite(ENA, TURN_SPEED_LOW);  // Left motor slower
    analogWrite(ENB, TURN_SPEED_HIGH); // Right motor faster
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void carturnright()
{
    // Right turn: right motor slower, left motor faster
    analogWrite(ENA, TURN_SPEED_HIGH); // Left motor faster
    analogWrite(ENB, TURN_SPEED_LOW);  // Right motor slower
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void carStop()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void cartankleft()
{
    // Tank turn left: left motor backward, right motor forward
    analogWrite(ENA, MOTOR_SPEED); // Left motor full speed
    analogWrite(ENB, MOTOR_SPEED); // Right motor full speed
    digitalWrite(IN1, LOW);        // Left motor backward
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); // Right motor forward
    digitalWrite(IN4, LOW);
}

void cartankright()
{
    // Tank turn right: left motor forward, right motor backward
    analogWrite(ENA, MOTOR_SPEED); // Left motor full speed
    analogWrite(ENB, MOTOR_SPEED); // Right motor full speed
    digitalWrite(IN1, HIGH);       // Left motor forward
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); // Right motor backward
    digitalWrite(IN4, HIGH);
}
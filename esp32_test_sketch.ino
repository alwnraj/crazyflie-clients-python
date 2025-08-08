/*
 * ESP32S3 - Crazyflie Connection Test Sketch
 * This sketch helps test the connection between ESP32S3 and Crazyflie
 * Upload this to your Xiao Sense ESP32S3
 */

#include <Arduino.h>

// Pin definitions for power monitoring
#define POWER_MONITOR_PIN 1 // GPIO1 for power monitoring
#define LED_PIN 2           // Built-in LED

// Variables
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds
bool powerTestMode = false;

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    // Initialize pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(POWER_MONITOR_PIN, INPUT);

    // Turn on LED to indicate startup
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    Serial.println("ESP32S3 - Crazyflie Connection Test");
    Serial.println("Ready for commands:");
    Serial.println("  PING - Respond with PONG");
    Serial.println("  POWER_TEST - Enter power test mode");
    Serial.println("  STATUS - Send current status");
    Serial.println("  HEARTBEAT - Send periodic status");
}

void loop()
{
    // Handle serial commands
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();

        handleCommand(command);
    }

    // Send heartbeat
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL)
    {
        sendHeartbeat();
        lastHeartbeat = millis();
    }

    // Power test mode
    if (powerTestMode)
    {
        runPowerTest();
    }

    delay(100);
}

void handleCommand(String command)
{
    if (command == "PING")
    {
        Serial.println("PONG");
    }
    else if (command == "POWER_TEST")
    {
        Serial.println("Entering power test mode");
        powerTestMode = true;
        digitalWrite(LED_PIN, HIGH);
    }
    else if (command == "STATUS")
    {
        sendStatus();
    }
    else if (command == "HEARTBEAT")
    {
        Serial.println("Heartbeat enabled");
    }
    else if (command == "STOP_POWER_TEST")
    {
        Serial.println("Exiting power test mode");
        powerTestMode = false;
        digitalWrite(LED_PIN, LOW);
    }
    else if (command == "LED_ON")
    {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON");
    }
    else if (command == "LED_OFF")
    {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF");
    }
    else if (command == "BLINK")
    {
        for (int i = 0; i < 5; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
        Serial.println("Blink complete");
    }
    else
    {
        Serial.print("Unknown command: ");
        Serial.println(command);
    }
}

void sendStatus()
{
    // Read analog values
    int powerPinValue = analogRead(POWER_MONITOR_PIN);

    // Get system info
    unsigned long uptime = millis();
    int freeHeap = ESP.getFreeHeap();

    Serial.print("STATUS:");
    Serial.print(" UPTIME=");
    Serial.print(uptime);
    Serial.print(" FREE_HEAP=");
    Serial.print(freeHeap);
    Serial.print(" POWER_PIN=");
    Serial.print(powerPinValue);
    Serial.print(" LED=");
    Serial.print(digitalRead(LED_PIN));
    Serial.print(" POWER_TEST=");
    Serial.print(powerTestMode ? "ON" : "OFF");
    Serial.println();
}

void sendHeartbeat()
{
    Serial.print("HEARTBEAT:");
    Serial.print(" UPTIME=");
    Serial.print(millis());
    Serial.print(" FREE_HEAP=");
    Serial.print(ESP.getFreeHeap());
    Serial.println();
}

void runPowerTest()
{
    // Simulate power draw by toggling LED rapidly
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    if (millis() - lastToggle > 100)
    { // Toggle every 100ms
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastToggle = millis();

        // Send power test data
        Serial.print("POWER_TEST:");
        Serial.print(" LED=");
        Serial.print(ledState ? "ON" : "OFF");
        Serial.print(" POWER_PIN=");
        Serial.print(analogRead(POWER_MONITOR_PIN));
        Serial.println();
    }
}

// Additional utility functions
void printSystemInfo()
{
    Serial.println("=== System Information ===");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("Chip Revision: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Chip Cores: ");
    Serial.println(ESP.getChipCores());
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(" MB");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    Serial.println("========================");
}

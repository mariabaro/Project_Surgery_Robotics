#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G3_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 31);
IPAddress receiverComputerIP(192, 168, 1, 55);
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

// Pins
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0;
float roll = 0, pitch = 0, yaw = 0;
int s1 = 1, s2 = 1;
//New variables for yaw control
float referenceYaw;
float incrementYaw;
boolean firstTime = true;
float currentYawServo;

void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void receiveOrientationUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      JsonDocument doc;  // ✅ Versió 7
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G3_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}

float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

// Modified function, in order to move servos based on received orientation data
void moveServos() {
  float delta = 0;
  if (s1 == 0) {
    delta = 40;
    Serial.println("S1 premut → Obrint");
  }

// Apply Roll
if (Gri_roll >= 270 && Gri_roll <= 360){
    int rollServo1 = constrain(90 + Gri_roll - 360 + delta, 0, 180);
    int rollServo2 = constrain(90 - Gri_roll + 360, 0, 180);
    servo_roll1.write(rollServo1);
    servo_roll2.write(rollServo2);
  }

  else {
    int rollServo1 = constrain(90 + Gri_roll + delta, 0, 180);
    int rollServo2 = constrain(90 - Gri_roll, 0, 180);
    servo_roll1.write(rollServo1);
    servo_roll2.write(rollServo2);
  }

  // Apply Pitch
  int pitchServo = constrain(90 + Gri_pitch, 0, 180);
  servo_pitch.write(pitchServo);

  // Apply Yaw 
  if (firstTime) {
    referenceYaw = Gri_yaw;   // Initial reference from IMU
    currentYawServo = 90;     // Neutral position
    servo_yaw.write(currentYawServo);
    firstTime = false;
  } else {
    // Compute angular difference
    float deltaYaw = Gri_yaw - referenceYaw;
    if (deltaYaw > 180)  deltaYaw -= 360;
    if (deltaYaw < -180) deltaYaw += 360;

    // Apply incremental movement
    currentYawServo += deltaYaw;

    // Constrain to servo range
    currentYawServo = constrain(currentYawServo, 0, 180);

    // Move the servo
    servo_yaw.write(currentYawServo);

    // Update reference for next iteration
    referenceYaw = Gri_yaw;
  }

}

// Function to update torque readings
void updateTorques() {
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw   = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);

  Serial.print("Torques → Roll1: "); Serial.print(Torque_roll1, 3);
  Serial.print(" Roll2: "); Serial.print(Torque_roll2, 3);
  Serial.print(" Pitch: "); Serial.print(Torque_pitch, 3);
  Serial.print(" Yaw: "); Serial.println(Torque_yaw, 3);
}

// Function to send torque data via UDP
void sendTorquesUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["Torque_roll1"] = Torque_roll1;
  doc["Torque_roll2"] = Torque_roll2;
  doc["Torque_pitch"] = Torque_pitch;
  doc["Torque_yaw"]   = Torque_yaw;

  char jsonBuffer[256];
  size_t len = serializeJson(doc, jsonBuffer);

- udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((uint8_t*)jsonBuffer, len);
  udp.endPacket();

  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((uint8_t*)jsonBuffer, len);
  udp.endPacket();

  Serial.println("Sent torque data to gripper and PC.");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

// Modified Loop function
void loop() {
  receiveOrientationUDP();  
  moveServos();             
  updateTorques();          // Measure current-based torques
  sendTorquesUDP();         // Send torque values to gripper and PC
  delay(50);                
}
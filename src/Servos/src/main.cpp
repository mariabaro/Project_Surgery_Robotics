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
IPAddress receiverComputerIP(192, 168, 1, 35);
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

// Reference yaw (for independence from North)
float initialYaw = 0.0;
bool yawReferenceSet = false;

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
      if (strcmp(device, "G5_Gri") == 0) {
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

void moveServos() {
  // Set reference yaw on first reading
  if (!yawReferenceSet) {
  initialYaw = Gri_yaw;
  yawReferenceSet = true;
  Serial.print("Initial Yaw reference set to: ");
  Serial.println(initialYaw);
  }

  // Apply angles relative to 90º neutral position
  float rollCmd = 90 + Gri_roll;
  float pitchCmd = 90 + Gri_pitch;
  float yawCmd = 90 + (Gri_yaw - initialYaw);

  // Constrain servo commands to valid range
  rollCmd = constrain(rollCmd, 0, 180);
  pitchCmd = constrain(pitchCmd, 0, 180);
  yawCmd = constrain(yawCmd, 0, 180);

  float delta = 0;
  if (s1 == 0) {
    delta = 40;
    Serial.println("S1 premut → Obrint");
  }

  // Apply servo positions
  servo_roll1.write(rollCmd + delta);
  servo_roll2.write(180 - rollCmd);
  servo_pitch.write(pitchCmd);
  servo_yaw.write(yawCmd);

  // Debug info
  Serial.print("RollCmd: "); Serial.print(rollCmd);
  Serial.print(" PitchCmd: "); Serial.print(pitchCmd);
  Serial.print(" YawCmd: "); Serial.println(yawCmd);
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

  Serial.println("Servos initialized to 90º neutral position");

}



void loop() {
  receiveOrientationUDP();
  moveServos();
  // Read torques
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);

  // Prepare JSON payload
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["Torque_roll1"] = Torque_roll1;
  doc["Torque_roll2"] = Torque_roll2;
  doc["Torque_pitch"] = Torque_pitch;
  doc["Torque_yaw"] = Torque_yaw;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  // Send to gripper ESP32
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((uint8_t*)buffer, n);
  udp.endPacket();

  // Send to computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((uint8_t*)buffer, n);
  udp.endPacket();

  // Optional: print torque values to serial
  Serial.print("Torque R1: "); Serial.print(Torque_roll1);
  Serial.print(" R2: "); Serial.print(Torque_roll2);
  Serial.print(" Pitch: "); Serial.print(Torque_pitch);
  Serial.print(" Yaw: "); Serial.println(Torque_yaw);
  delay(20);
}

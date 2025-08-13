#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
#include <Servo.h>

#define LED_PIN LED_BUILTIN  // Change to a specific pin if using an external LED

// ========================= WiFi / MQTT =========================
char ssid[] = "USG-Mobility";
char pass[] = "shadygrove9631";
int  status = WL_IDLE_STATUS;

const char broker[] = "broker.hivemq.com";
int        port     = 1883;
const char topicCmd[] = "jumpstart/rover_command"; // subscribe here
const char topicLocation[] = "jumpstart/rover_location";

Servo servoA; // A0
Servo servoB; // A1


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Forward decls
void connectWiFi();
void printWifiStatus();
void connectMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
inline void pumpNetwork() { mqttClient.loop(); delay(1); }

// ========================= Motors / IMU =========================
const int L_IN1 = 2;  
const int L_IN2 = 3;  
const int R_IN1 = 5;  
const int R_IN2 = 4;  

MPU6050 mpu(Wire);
const bool INVERT_YAW = false; 

inline void setPWM(int pin, int duty) { analogWrite(pin, duty); }
inline void pinInitPWM(int pin){ pinMode(pin, OUTPUT); setPWM(pin, 0); }
inline int  clamp255(int v){ return v<0?0:(v>255?255:v); }

void setSignedMotor(int pwmFwd, int pwmRev, int duty){
  duty = (duty > 255 ? 255 : (duty < -255 ? -255 : duty));
  if (duty >= 0) { setPWM(pwmFwd, duty); setPWM(pwmRev, 0); }
  else           { setPWM(pwmFwd, 0);    setPWM(pwmRev, -duty); }
}

void setSignedLR(int dutyL, int dutyR){
  setSignedMotor(L_IN1, L_IN2, dutyL);
  setSignedMotor(R_IN1, R_IN2, dutyR);
}

float angleDiffDeg(float a, float b){
  float d = a - b;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

float readYaw(){
  mpu.update();
  return mpu.getAngleZ();
}

void driveStop(){
  setSignedLR(0,0);
  Serial.println("[drive] Stop");
}

// -------- Drive X feet with heading hold --------
void driveForwardFeet(float feet, int baseDuty=90, int corrDuty=255, float deadbandDeg=3.0f,
                      uint16_t ctrlHz=50, float feetPerSec=0.90f) {
  baseDuty = clamp255(baseDuty);
  corrDuty = clamp255(corrDuty);
  if (feetPerSec <= 0.01f) feetPerSec = 0.01f;
  const uint32_t stepMs = max<uint32_t>(1, 1000UL / ctrlHz);
  const uint32_t duration = (uint32_t)((fabs(feet) / feetPerSec) * 1000.0f);
  const uint32_t t0 = millis();
  float targetYawDeg = readYaw();
  int dirSign = (feet >= 0) ? 1 : -1;

  while ((millis() - t0) < duration) {
    pumpNetwork();
    float yaw = readYaw();
    float err = angleDiffDeg(targetYawDeg, yaw);
    if (INVERT_YAW) err = -err;

    bool onTrack = (fabs(err) <= deadbandDeg);
    int dutyL = baseDuty, dutyR = baseDuty;
    if (!onTrack) {
      if (err < 0) dutyL = corrDuty; 
      else         dutyR = corrDuty;
    }
    setSignedLR(dirSign * dutyL, dirSign * dutyR);
    uint32_t elapsed = millis() - t0;
    if (elapsed < stepMs) delay(stepMs - elapsed);
  }
  driveStop();
}

// -------- In-place turn --------
void turnDegrees(float degRight, int turnDuty=140, float stopThresholdDeg=2.0f,
                 uint16_t ctrlHz=50, uint16_t maxSeconds=6) {
  turnDuty = clamp255(turnDuty);
  if (turnDuty < 60) turnDuty = 60;
  const float yaw0 = readYaw();
  const uint32_t stepMs = max<uint32_t>(1, 1000UL / ctrlHz);
  const uint32_t deadline = millis() + (uint32_t)maxSeconds * 1000UL;

  while (true) {
    pumpNetwork();
    const float yaw = readYaw();
    float rotated = yaw - yaw0;
    if (INVERT_YAW) rotated = -rotated;
    float remaining = degRight - (-rotated);
    float aRem = fabs(remaining);
    if (aRem <= stopThresholdDeg || millis() > deadline) break;
    int sgn = (remaining >= 0 ? +1 : -1);
    float scale = aRem / 30.0f;
    if (scale > 1.0f) scale = 1.0f;
    if (scale < 0.20f) scale = 0.20f;
    int duty = (int)(turnDuty * scale);
    if (duty < 60) duty = 60;
    setSignedLR(+sgn * duty, -sgn * duty);
    delay(stepMs);
  }
  driveStop();
}

// ========================= Command Parsing =========================
void handleCommand(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  for (char* p = line; *p && *p != ','; p++) *p = tolower(*p);
  char buf[160];
  strncpy(buf, line, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '\0';
  char* tokens[10];
  int ntok = 0;
  char* tok = strtok(buf, ",");
  while (tok && ntok < 10) {
    while (*tok==' '||*tok=='\t') tok++;
    char* end = tok + strlen(tok) - 1;
    while (end>tok && isspace(*end)) { *end = '\0'; end--; }
    tokens[ntok++] = tok;
    tok = strtok(NULL, ",");
  }
  if (ntok == 0) return;

  if (strcmp(tokens[0], "drive") == 0) {
    float feet = (ntok>1) ? atof(tokens[1]) : 0.0f;
    int baseDuty = (ntok>2) ? atoi(tokens[2]) : 90;
    int corrDuty = (ntok>3) ? atoi(tokens[3]) : 255;
    float deadband = (ntok>4) ? atof(tokens[4]) : 3.0f;
    int hz = (ntok>5) ? atoi(tokens[5]) : 50;
    float fps = (ntok>6) ? atof(tokens[6]) : 0.90f;
    driveForwardFeet(feet, baseDuty, corrDuty, deadband, hz, fps);
  }
  else if (strcmp(tokens[0], "turn") == 0) {
    float degRight = (ntok>1) ? atof(tokens[1]) : 0.0f;
    int turnDuty = (ntok>2) ? atoi(tokens[2]) : 140;
    float stopThr = (ntok>3) ? atof(tokens[3]) : 2.0f;
    int hz = (ntok>4) ? atoi(tokens[4]) : 50;
    int maxSec = (ntok>5) ? atoi(tokens[5]) : 6;
    turnDegrees(degRight, turnDuty, stopThr, hz, maxSec);
  }
  else if (strcmp(tokens[0], "probe") == 0) {
    probe(500);
  }
  else if (strcmp(tokens[0], "stop") == 0) {
    driveStop();
  }
}

void stopServos() {
  servoA.write(90); // stop
  servoB.write(90);
}

void probe(int timeMs) {
  // Down: A forward, B backward
  servoA.write(120); // forward
  servoB.write(60);  // reverse
  delay(timeMs);

  // Up: A backward, B forward
  servoA.write(60);  // reverse
  servoB.write(120); // forward
  delay(timeMs);

  // Stop
  stopServos();
}

// ========================= Setup / Loop =========================
void setup() {
  Serial.begin(115200);
  while (!Serial) {;}
  pinMode(LED_PIN, OUTPUT);
  analogWriteResolution(8);
  pinInitPWM(L_IN1); pinInitPWM(L_IN2);
  pinInitPWM(R_IN1); pinInitPWM(R_IN2);

  servoA.attach(A0);
  servoB.attach(A1);

  stopServos();

  Wire.begin(); delay(100);
  if (mpu.begin() != 0) Serial.println("MPU6050 init failed!");
  mpu.calcGyroOffsets();

  connectWiFi();
  mqttClient.setServer(broker, port);
  mqttClient.setCallback(onMqttMessage);
  connectMQTT();
}

void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();
  delay(1);
}

// ========================= WiFi / MQTT Impl =========================
void connectWiFi() {
  Serial.println("Connecting to WiFi…");
  while (status != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(3000);
  }
  digitalWrite(LED_PIN, HIGH); // solid on when connected
  printWifiStatus();
}

void connectMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker);
  byte mac[6];
  WiFi.macAddress(mac);
  String clientId = "GIGAClient-" + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);

  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection… ");
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");
      mqttClient.subscribe(topicCmd);

      // Send initial location/status message
      mqttClient.publish(topicLocation, "Rover connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" — retry in 2s");
      delay(2000);
    }
  }
}


void onMqttMessage(char* t, byte* payload, unsigned int length) {
  static char msg[200];
  unsigned int n = min(length, (unsigned int)(sizeof(msg)-1));
  for (unsigned int i = 0; i < n; i++) msg[i] = (char)payload[i];
  msg[n] = '\0';
  if (strcmp(t, topicCmd) == 0) handleCommand(msg);
}

void printWifiStatus() {
  Serial.print("SSID: "); Serial.println(WiFi.SSID());
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

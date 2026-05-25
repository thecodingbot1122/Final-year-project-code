/*********
  Project: ESP32 Landmine Robot - High-Speed Scan (Optimized + GPS + Thread Safe + Direct Servo)
*********/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h> 
#include <ESP32Servo.h> // Added Direct ESP32 Servo Library

// --- GPS & Serial Configuration ---
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART1 for GPS
HardwareSerial mySerial(2);  // Keep UART2 for Communication

// --- Direct Scan Servo ---
Servo scanServo;
#define SCAN_SERVO_PIN 5 // CRITICAL: Changed from 23 to 5 to prevent I2C crash!

// --- Mutex for Thread Safety ---
SemaphoreHandle_t messageMutex;
QMC5883LCompass compass;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// --- Configuration ---
#define SERVOMIN 150   
#define SERVOMAX 600   
#define SERVO_FREQ 50  
#define NUM_SERVOS 7

// Pins
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14    
#define ENB 32
#define analogPin 13

// Hardware Mapping
int servoChannels[NUM_SERVOS] = {0, 10, 11, 12, 13, 14, 15};
int initialAngles[7] = {60, 110, 90, 40, 120, 50, 140};
int baseservo = 0; // No longer used for scanning, kept for indexing
int pos ;
int currentStep = 0; 

// Speed & Scan Settings
int scanStepSize = 32;
int scanTickDelay = 15;
int slowSpeed = 130;
int slowSpeed1 = 100;
int fastSpeed = 255;
int angle = 30;

// State Variables (Volatile for Dual Core Safety)
String message = "";
volatile bool autoModeActive = false;
volatile bool manualScanActive = false;
volatile bool metalDetectedLock = false;
volatile bool autodetected = false;

// Mode Variables
volatile bool automode = false;
volatile bool menualmode = true; 
volatile bool gpsPathGenerated = false; 

volatile bool ON = true;

// Task Handles
TaskHandle_t ManualTask;
TaskHandle_t ScanTask;
TaskHandle_t AutoTask;

// ==========================================
// THREAD-SAFE I2C WRAPPERS (Crash Fix)
// ==========================================
void safePWM(int num, int on, int off) {
  if(xSemaphoreTake(messageMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
    pwm.setPWM(num, on, off);
    xSemaphoreGive(messageMutex);
  } else {
    Serial.println("I2C Error: PWM skipped to prevent crash");
  }
}

void safeCompassRead() {
  if(xSemaphoreTake(messageMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
    compass.read();
    xSemaphoreGive(messageMutex);
  } else {
    Serial.println("I2C Error: Compass read skipped to prevent crash");
  }
}

// --- Helper Functions ---
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void carstop() {
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void carforwardA() {
  if (metalDetectedLock) return;
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, slowSpeed); analogWrite(ENB, slowSpeed);
}
void carforward() {
  if (metalDetectedLock) return;
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, slowSpeed); analogWrite(ENB, slowSpeed);
}

void carbackwardA() {
  if (metalDetectedLock) return;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, slowSpeed); analogWrite(ENB, slowSpeed);
}

void carleftA() {
  if (metalDetectedLock) return;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, fastSpeed); analogWrite(ENB, fastSpeed);
}

void carrightA() {
  if (metalDetectedLock) return;
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); 
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, fastSpeed); analogWrite(ENB, fastSpeed);
}

// --- GPS DATA FUNCTION ---
void displayGPS() {
  if (gps.location.isValid()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
    Serial.print(" | Speed: "); Serial.println(gps.speed.kmph());
  } else {
    Serial.println("GPS: Waiting for Fix...");
  }
}

// --- Compass Code ---
void CAMPASS() {
  int a;
  char myArray[3];
  safeCompassRead();
  a = compass.getAzimuth();
  compass.getDirection(myArray, a);
  
  Serial.print("Azimuth: "); Serial.print(a);
  Serial.print(" | Dir: "); Serial.print(myArray[0]); Serial.print(myArray[1]); Serial.println(myArray[2]);
}

// --- Servo Functions ---
void jaropen() { for (int i = 150; i >= 80; i--) { safePWM(servoChannels[6], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void jarclose() { for (int i = 80; i <= 150; i++) { safePWM(servoChannels[6], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo2up() { for (int i = 100; i >= 80; i--) { safePWM(servoChannels[4], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo2down() { for (int i = 80; i <= 120; i++) { safePWM(servoChannels[4], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo3up() { for (int i = 40; i <= 100; i++) { safePWM(servoChannels[3], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo3down() { for (int i = 100; i >= 40; i--) { safePWM(servoChannels[3], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo4up() { for (int i = 20; i <= 90; i++) { safePWM(servoChannels[2], 0, angleToPulse(i)); vTaskDelay(50 / portTICK_PERIOD_MS); } }
void servo4dropup() { for (int i = 20; i <= 90; i++) { safePWM(servoChannels[2], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo4downPick() { for (int i = 90; i >= 20; i--) { safePWM(servoChannels[2], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }
void servo4downDrop() { for (int i = 90; i >= 20; i--) { safePWM(servoChannels[2], 0, angleToPulse(i)); vTaskDelay(20 / portTICK_PERIOD_MS); } }

void pickanddrop() { 
  safePWM(servoChannels[1], 0, angleToPulse(pos));
  jaropen(); servo3up(); servo4downPick(); jarclose(); servo4up(); //servo3down();
  
  for(int i = pos; i >= 40; i--) { safePWM(servoChannels[1], 0, angleToPulse(i)); vTaskDelay(10 / portTICK_PERIOD_MS); }
  servo2up(); servo4downDrop();  jaropen(); servo4dropup(); /*servo3up();*/ jarclose(); servo3down(); servo2down();
  for(int i = 40; i <= 110; i++) { safePWM(servoChannels[1], 0, angleToPulse(i)); vTaskDelay(10 / portTICK_PERIOD_MS); }
}

void SafetyAngel() {  
   int safetyAngle = (angle <= 80) ? 180 : 20;
   //Serial.println(safetyAngle);
   scanServo.write(safetyAngle); // Direct ESP32 Control
}

//slow scan
void slowscan() {
  int startAngle = -1;
  int endAngle = -1;
  int detectionCount = 0;
  Serial.println(">>> Precision Scanning for Center...");

  for (int i = 20; i <= 180; i ++) {
    scanServo.write(i); // Direct ESP32 Control
    vTaskDelay(50 / portTICK_PERIOD_MS); 
      
    int val = analogRead(analogPin);
   
    if (val > 3000) {
      detectionCount++;
      //Serial.println(val);
       Serial.println(i);
      if (detectionCount == 2 && startAngle == -1) startAngle = i; 
      endAngle = i;                         
    } 
   // else if (startAngle != -1) {
    //  break; 
    //}
  }
  Serial.println(startAngle);

  if (startAngle != -1) {
    angle = (startAngle + endAngle) / 2; 
    pos = map(angle, 20, 180, 140, 90); 
    SafetyAngel();
  }
}

// --- COMPASS HELPER FUNCTIONS ---
int getAngle() {
  safeCompassRead();
  int a = compass.getAzimuth();
  if (a < 0) { a = a + 360; }
  return a;
}

void turnRightUntil(int targetAngle) {
  carrightA();
  
 Serial.print("right run");
  while (true) {
    if (message != "T" && message != "Y") { carstop(); return; } 

    if (metalDetectedLock || autodetected) {
        carstop();
        while (metalDetectedLock) { vTaskDelay(50 / portTICK_PERIOD_MS); }
        if (autodetected) { pickanddrop(); autodetected = false; }
        autoModeActive = true;
        carrightA(); 
    }

    int currentAngle = getAngle();
    Serial.println(currentAngle);
    if (currentAngle == targetAngle || 
        currentAngle == targetAngle + 1 || currentAngle == targetAngle - 1 ||
        currentAngle == targetAngle + 2 || currentAngle == targetAngle - 2 ||
        currentAngle == targetAngle + 3 || currentAngle == targetAngle - 3) {
      break; 
    }
    vTaskDelay( 50 / portTICK_PERIOD_MS);
  }
  carstop();
}

void turnLeftUntil(int targetAngle) {
  carleftA(); 
 Serial.print("left run");
  while (true) {
    if (message != "T" && message != "Y") { carstop(); return; } 

    if (metalDetectedLock || autodetected) {
        carstop();
        while (metalDetectedLock) { vTaskDelay(50 / portTICK_PERIOD_MS); }
        if (autodetected) { pickanddrop(); autodetected = false; }
        autoModeActive = true;
        carleftA(); 
    }

    int currentAngle = getAngle();
     Serial.println(currentAngle);
    if (currentAngle == targetAngle || 
        currentAngle == targetAngle + 1 || currentAngle == targetAngle - 1 ||
        currentAngle == targetAngle + 2 || currentAngle == targetAngle - 2 ||
        currentAngle == targetAngle + 3 || currentAngle == targetAngle - 3) {
      break; 
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  carstop();
}

void autoAlign(int targetAngle) {
  int currentAngle = getAngle();
  Serial.println(currentAngle);
  int diff = targetAngle - currentAngle;

  if (diff > 180) diff -= 360;
  else if (diff < -180) diff += 360;

  if (abs(diff) <= 3) return; 

  if (diff > 0) turnRightUntil(targetAngle);
  else turnLeftUntil(targetAngle);
}

// ==========================================
// SMART FORWARD FUNCTION (Timer Fix)
// ==========================================
void smartForward(unsigned long duration) {
    unsigned long elapsed = 0;
    carforward();

    while (elapsed < duration) {
        if (message != "T" && message != "Y") { carstop(); return; } 

        if (metalDetectedLock || autodetected) {
            carstop(); 

        while (metalDetectedLock) { vTaskDelay(50 / portTICK_PERIOD_MS); }
        if (autodetected) { pickanddrop(); autodetected = false; }

            autoModeActive = true; 
            carforward(); 
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
        elapsed += 10; 
    }
    carstop();
}

// ==========================================
// AUTOMATIC MOVEMENT (Hardcoded Path - for T)
// ==========================================
void AutomaticCarMovement() {
    switch (currentStep) {
    case 0:
      Serial.println("Step 0: Auto-Aligning to start position...");
      autoAlign(120); 
      if(message != "T") return;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      currentStep = 1; 
      break;

    case 1:
      Serial.println("Step 1: Forward at 212 degrees");
      smartForward(2000); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 2; 
      break;

    case 2:
      Serial.println("Step 2: Turn right to 272 degrees");
      turnRightUntil(190); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      
      Serial.println("Step 2: Lane change forward");
      smartForward(700); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 3;
      break;

    case 3:
      Serial.println("Step 3: Turn right to 3 degrees");
      turnRightUntil(280); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 4;
      break;

    case 4:
      Serial.println("Step 4: Forward at 3 degrees");
      smartForward(2000); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 5;
      break;

    case 5:
      Serial.println("Step 5: Turn left to 272 degrees");
      turnLeftUntil(200); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      
      Serial.println("Step 5: Lane change forward");
      smartForward(700); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 6;
      break;

    case 6:
      Serial.println("Step 6: Turn left to 212 degrees");
      turnLeftUntil(120); 
      if(message != "T") return;
      vTaskDelay(500 / portTICK_PERIOD_MS);
      currentStep = 1; 
      break;
  }
}

// ==========================================
// AUTOMATIC MOVEMENT G (Live GPS Navigation - for Y)
// ==========================================
double G_Lat[4] = {0, 0, 0, 0}; 
double G_Lon[4] = {0, 0, 0, 0}; 

double waypointsLat[10]; 
double waypointsLon[10];
int totalWaypoints = 0;
int currentWaypointIndex = 0;

void generateZigZagPath() {
  totalWaypoints = 0;
  int numLanes = 3; 
  
  for (int i = 0; i < numLanes; i++) {
    float fraction = (float)i / (numLanes - 1);
    
    double leftLat = G_Lat[0] + (G_Lat[1] - G_Lat[0]) * fraction;
    double leftLon = G_Lon[0] + (G_Lon[1] - G_Lon[0]) * fraction;

    double rightLat = G_Lat[3] + (G_Lat[2] - G_Lat[3]) * fraction;
    double rightLon = G_Lon[3] + (G_Lon[2] - G_Lon[3]) * fraction;

    if (i % 2 == 0) {
      waypointsLat[totalWaypoints] = leftLat;  waypointsLon[totalWaypoints] = leftLon;  totalWaypoints++;
      waypointsLat[totalWaypoints] = rightLat; waypointsLon[totalWaypoints] = rightLon; totalWaypoints++;
    } else {
      waypointsLat[totalWaypoints] = rightLat; waypointsLon[totalWaypoints] = rightLon; totalWaypoints++;
      waypointsLat[totalWaypoints] = leftLat;  waypointsLon[totalWaypoints] = leftLon;  totalWaypoints++;
    }
  }
  Serial.println(">>> ZigZag Path Generated! Total Waypoints: " + String(totalWaypoints));
}

void AutomaticCarMovementG() {
    if (!gps.location.isValid()) {
      Serial.println("Waiting for GPS Fix to move...");
      carstop();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      return;
    }

    if (currentWaypointIndex >= totalWaypoints) {
      Serial.println(">>> SCAN COMPLETE! Area Covered.");
      carstop();
      autoModeActive = false;
      return;
    }

    double tLat = waypointsLat[currentWaypointIndex];
    double tLon = waypointsLon[currentWaypointIndex];

    double distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), tLat, tLon);
    double course = gps.courseTo(gps.location.lat(), gps.location.lng(), tLat, tLon);

    Serial.print("Target WP: "); Serial.print(currentWaypointIndex);
    Serial.print(" | Dist: "); Serial.print(distance);
    Serial.print("m | Angle: "); Serial.println(course);

    if (distance <= 1.5) {
      Serial.println("Reached Waypoint!");
      carstop();
      currentWaypointIndex++;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      return;
    }

    int currentHeading = getAngle();
    int diff = abs(currentHeading - (int)course);
    if (diff > 180) diff = 360 - diff;

    if (diff > 10) {
      autoAlign((int)course);
    } else {
      carforwardA();
    }
}

// ==========================================
// TASK 1: MANUAL & COMMUNICATION (Core 0)
// ==========================================
void ManualTaskCode(void * pvParameters) {
  for(;;) {
    if (mySerial.available()) {
      message = mySerial.readStringUntil('\n');
      message.trim();
      Serial.println(message);
      
      if (message == "M") { menualmode = true; automode = false; Serial.println(">>> Switched to MANUAL MODE"); }
      if (message == "A") { automode = true; menualmode = false; Serial.println(">>> Switched to AUTO MODE"); }

      if (menualmode == true) {
        if (!metalDetectedLock) {
          if (message == "F") carforwardA();
          else if (message == "B") carbackwardA();
          else if (message == "L") carleftA();
          else if (message == "R") carrightA();
          else if (message == "S") carstop();
          
          if (message.indexOf(':') != -1 && !message.startsWith("G")) {
            int colonIndex = message.indexOf(':');
            String id = message.substring(0, colonIndex);
            int rx_angle = message.substring(colonIndex + 1).toInt();
            
            if (id == "S0")      safePWM(servoChannels[6], 0, angleToPulse(rx_angle));
            else if (id == "S1") safePWM(servoChannels[5], 0, angleToPulse(rx_angle));
            else if (id == "S2") safePWM(servoChannels[4], 0, angleToPulse(rx_angle));
            else if (id == "S3") safePWM(servoChannels[3], 0, angleToPulse(rx_angle));
            else if (id == "S4") safePWM(servoChannels[2], 0, angleToPulse(rx_angle));
            else if (id == "S5") safePWM(servoChannels[1], 0, angleToPulse(rx_angle));
          }
        }
        if (message == "SCAN") manualScanActive = !manualScanActive;
        if (message == "GPS") displayGPS(); 
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// TASK 2: SCANNING & DETECTION (Core 1)
// ==========================================
void ScanTaskCode(void * pvParameters) {
  bool directionUp = true;

  for(;;) {
    if (autoModeActive || manualScanActive) {

      scanServo.write(angle); // Direct ESP32 Control
      // Serial.println(angle);
    if (directionUp) { 
        angle += scanStepSize; 
        if (angle >= 180) { angle = 180;  directionUp = false; } 
      } else { 
        angle -= scanStepSize; 
        if (angle <= 20) { angle = 20;    directionUp = true; } 
      } 
     // Serial.println(analogRead(analogPin));
      if (analogRead(analogPin) > 3000) {
        carstop();
        Serial.println("!!! MINE DETECTED !!!");

        metalDetectedLock = true; 
        if(autoModeActive) {
          Serial.println("inside slow scan");
          slowscan();
          autodetected = true;
          metalDetectedLock = false; 
        }

        SafetyAngel();
        autoModeActive = false;
        manualScanActive = false;
       // autodetected = true;
        vTaskDelay(3000 / portTICK_PERIOD_MS); 
        metalDetectedLock = false; 
      }


      
      vTaskDelay(scanTickDelay / portTICK_PERIOD_MS); 
      
    } else {
      vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
  }
}

// ==========================================
// TASK 3: AUTOMATIC & GPS (Core 1)
// ==========================================
void AutoTaskCode(void * pvParameters) {
  for(;;) {
    if (automode == true) {
      
      while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
      }
      
      if (message.startsWith("G") && message.indexOf(':') != -1) {
        int colonIndex = message.indexOf(':');
        int commaIndex = message.indexOf(',');

        if (colonIndex != -1 && commaIndex != -1) {
          String id = message.substring(0, colonIndex);
          double lat = message.substring(colonIndex + 1, commaIndex).toDouble();
          double lon = message.substring(commaIndex + 1).toDouble();

          if (id == "G1") { G_Lat[0] = lat; G_Lon[0] = lon; Serial.println("G1 Saved"); }
          else if (id == "G2") { G_Lat[1] = lat; G_Lon[1] = lon; Serial.println("G2 Saved"); }
          else if (id == "G3") { G_Lat[2] = lat; G_Lon[2] = lon; Serial.println("G3 Saved"); }
          else if (id == "G4") { G_Lat[3] = lat; G_Lon[3] = lon; Serial.println("G4 Saved"); }
        }
        message = ""; 
      } 
      else if (message == "Y") {
        if (gpsPathGenerated == false) {
          Serial.println(">>> Starting GPS Guided Scan!");
          generateZigZagPath();
          currentWaypointIndex = 0;
          gpsPathGenerated = true;
        }

        if (autodetected == true) {
          pickanddrop();
          autodetected = false;
        } else {
          autoModeActive = true;
          AutomaticCarMovementG(); 
        }
      } 
      else if (message == "T") {
        /*if (autodetected == true) {
          pickanddrop();
          autodetected = false;
        } else {
          autoModeActive = true;
          //AutomaticCarMovement(); 
        }*/

        if(ON == true){ autoModeActive = true; ON = false; }
        else {
          //Serial.println(getAngle());
          
    //vTaskDelay(250 / portTICK_PERIOD_MS);
          AutomaticCarMovement();
        }
      } 
      else if (message == "Z") {
        autoModeActive = false;
        gpsPathGenerated = false;
        ON = true ;
        carstop();
        currentStep = 0; 
        scanServo.write(20);
        for(int i = 0; i < NUM_SERVOS; i++) {
          // Do not write to base servo index since it's removed from PCA
          if(i != 0) safePWM(servoChannels[i], 0, angleToPulse(initialAngles[i]));
        }
        message = "";
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 2, 4); 
  mySerial.begin(115200, SERIAL_8N1, 21, 19);
  
  Wire.begin(22, 23); 
  Wire.setClock(400000);
  
  messageMutex = xSemaphoreCreateMutex(); 
  compass.init();
    // Aap ki final correct calibration values
  compass.setCalibration(4036, 6213, -7885, -5492, -3267, -2691);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // --- NEW: Hardware PWM Setup for Scan Servo ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  scanServo.setPeriodHertz(50);
  scanServo.attach(SCAN_SERVO_PIN, 500, 2400); 
  scanServo.write(angle); // Initial position

  for(int i = 0; i < NUM_SERVOS; i++) {
    if(i != 0) safePWM(servoChannels[i], 0, angleToPulse(initialAngles[i]));
  }

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  carstop();

  xTaskCreatePinnedToCore(ManualTaskCode, "Manual", 7096, NULL, 2, &ManualTask, 0);
  xTaskCreatePinnedToCore(ScanTaskCode, "Scan", 7096, NULL, 1, &ScanTask, 1);
  xTaskCreatePinnedToCore(AutoTaskCode, "Auto", 7096, NULL, 1, &AutoTask, 1);
  // 3. Delete the default Arduino loop task permanently
  vTaskDelete(NULL);
}

void loop() {
  // FreeRTOS tasks handle everything.
  
}

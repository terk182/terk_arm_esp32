#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>  // สำหรับฟังก์ชันทางคณิตศาสตร์
#include "RobotArmIK.h"
#include <EEPROM.h>
const char* ssid = "Terk_2.4GHz";  // ชื่อ Wi-Fi ของคุณ
const char* password = "08110171188";  // รหัสผ่าน Wi-Fi ของคุณ

// สร้างวัตถุ WebServer (ใช้พอร์ต 80)
WebServer server(80);

// สร้างวัตถุมอเตอร์และเซอร์โวเหมือนในโค้ดเดิม
AccelStepper stepper1(1, 17, 16);  // ข้อต่อ 1 หมุนรอบแกน Z
AccelStepper stepper2(1, 18, 5);   // ข้อต่อ 2
AccelStepper stepper3(1, 21, 19);  // ข้อต่อ 3
Servo gripperServo;
// กำหนด Limit Switch
#define limitSwitch1 26  // Limit switch สำหรับมอเตอร์ 1
#define limitSwitch2 25  // Limit switch สำหรับมอเตอร์ 2
#define limitSwitch3 33  // Limit switch สำหรับมอเตอร์ 3
#define DEBOUNCE_DELAY 10  // เวลาในการหน่วง debounce (50 มิลลิวินาที)

// ประกาศ Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define MAX_STEPS 1  // จำนวนขั้นตอนสูงสุดที่ต้องการสำหรับ trajectory

// ตัวแปรจัดเก็บตำแหน่งมุมของแต่ละขั้นตอน
int theta1Steps[MAX_STEPS];
int theta2Steps[MAX_STEPS];
int theta3Steps[MAX_STEPS];

int currentStep = 0;   // ขั้นตอนปัจจุบัน
int totalSteps = 0;    // จำนวนขั้นตอนทั้งหมด
bool isMoving = false; // สถานะของการเคลื่อนที่

// ความยาวของข้อต่อ (สมมุติ)
double L1 = 135;  // ความยาวของข้อต่อแรก L1 = 135 มม.
double L2 = 157;  // ความยาวของข้อต่อที่สอง L2 = 147 มม.
double baseHeight = 156; // ความสูงจากพื้น 156 มม.

int stepper1Position, stepper2Position, stepper3Position;
const float theta1AngleToSteps = 77.222222;
const float theta2AngleToSteps = 77.222222;
const float phiAngleToSteps = 23.222222;

// ตัวแปรเก็บค่าที่ถูกส่งไปล่าสุด
int lastTheta1 = 250;
int lastTheta2 = 210;
int lastTheta3 = 50;

int targetTheta1_i = 0;
int targetTheta2_i = 0;
int targetTheta3_i = 0;

int lastTheta1_home = 25;
int lastTheta2_home = 10;
int lastTheta3_home = 158;
int lastGripper = 180;
int lastSpeed = 2000;
int lastAcceleration = 100;



// ตัวแปรสำหรับตำแหน่งปลายแขนกล (forward kinematics result)
double endEffectorX = 0;
double endEffectorY = 0;
double endEffectorZ = 0;  // เพิ่มแกน Z
g_Code cmd;
RobotArmIK robotArmIK(L1, L2, baseHeight, 125, 125, 420, 50);


  String savedSSID = "";
  String savedPassword = "";

bool isFirstTime = false; // เช็คว่าคือการเริ่มต้นครั้งแรกหรือไม่
  char data[32];
  int len = 0;
  unsigned char k;



String css = "<style>body {font-family: Arial, sans-serif;padding: 20px;background-color: #f9f9f9;}.content {background-color: #fff;padding: 20px;border: 1px solid #ddd;box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);max-width: 800px;margin: 0 auto;}a {text-decoration: none;padding: 5px 10px;background-color: #007bff;color: white;border-radius: 3px;}a:hover {background-color: #0056b3;}</style>";






// ตั้งค่า flag เพื่อตรวจสอบการทำงานของ Timer
volatile bool timerFlag = false;

// ฟังก์ชัน Timer ISR
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    timerFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}
// ฟังก์ชัน debounce สำหรับ Limit Switch
bool debounce(int pin) {
    bool state = digitalRead(pin);
    delay(DEBOUNCE_DELAY);  // รอช่วงเวลาหน่วง
    bool newState = digitalRead(pin);
    return state == newState ? state : HIGH;  // ตรวจสอบว่าค่าที่อ่านได้เป็นค่าเดิมหรือไม่
}

void setStepperSpeedAndAcceleration(int speed, int acceleration) {
    stepper1.setMaxSpeed(speed);
    stepper1.setAcceleration(acceleration);
    stepper2.setMaxSpeed(speed);
    stepper2.setAcceleration(acceleration);
    stepper3.setMaxSpeed(speed);
    stepper3.setAcceleration(acceleration);
}
// ฟังก์ชันสำหรับการตั้งค่าตำแหน่งเริ่มต้นโดยใช้ Limit Switch พร้อม debounce
void setZero() {

   // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
    // กำหนดทิศทางการเคลื่อนที่ให้เคลื่อนไปยัง Limit Switch
    stepper1.setSpeed(-2000); // เคลื่อนที่ไปทิศทางตรงข้ามเพื่อหาจุดเริ่มต้น
    //stepper1.setAcceleration(500);
    stepper2.setSpeed(-2000);
    //stepper2.setAcceleration(500);
    stepper3.setSpeed(-2000);
   // stepper3.setAcceleration(500);
    while (debounce(limitSwitch2) == HIGH) {
        stepper2.runSpeed();
    }
    stepper2.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
    // รันมอเตอร์จนกว่าจะถึง Limit Switch (พร้อม debounce)
    while (debounce(limitSwitch1) == HIGH) {
        stepper1.runSpeed();
    }
    stepper1.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch



    while (debounce(limitSwitch3) == HIGH) {
        stepper3.runSpeed();
    }
    stepper3.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

    Serial.println("Set Zero: มอเตอร์ทั้งหมดถูกตั้งค่าที่ตำแหน่ง 0");
}
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
bool checkLimitSwitch() {
  bool limit1 = digitalRead(limitSwitch1) == LOW; // LOW เมื่อถูกกด
  bool limit2 = digitalRead(limitSwitch2) == LOW;
  bool limit3 = digitalRead(limitSwitch3) == LOW;
  
  // หากมี Limit Switch ใดถูกกด, ให้หยุดการเคลื่อนที่
  if (limit1 || limit2 || limit3) {
    Serial.println("Limit switch triggered! Motion stopped.");
    return true; // มี Limit Switch ถูกกด
  }
  return false; // ไม่มี Limit Switch ถูกกด
}

float lerp(float start, float end, float t) {
    return start + (end - start) * t;
}
// ฟังก์ชัน Trajectory Planning
void trajectoryPlanning(int targetTheta1, int targetTheta2, int targetTheta3, int duration) {
    totalSteps = MAX_STEPS;
    float timePerStep = (float)duration / totalSteps;

    // คำนวณตำแหน่งมุมในแต่ละขั้นตอน
    for (int i = 0; i < totalSteps; ++i) {
        float t = (float)i / totalSteps;

        // คำนวณมุมปัจจุบันโดยใช้ Linear Interpolation
        int currentTheta1 = lerp(lastTheta1, targetTheta1, t);
        int currentTheta2 = lerp(lastTheta2, targetTheta2, t);
        int currentTheta3 = lerp(lastTheta3, targetTheta3, t);

        // เรียกใช้ฟังก์ชัน runIK เพื่อคำนวณตำแหน่ง x, y, z ที่ต้องการ
        g_Code stepCmd = robotArmIK.runIK(currentTheta1, currentTheta2, currentTheta3, cmd);

        // จัดเก็บค่าที่คำนวณได้
        theta1Steps[i] = stepCmd.y * theta1AngleToSteps;
        theta2Steps[i] = stepCmd.z * theta2AngleToSteps;
        theta3Steps[i] = stepCmd.x * phiAngleToSteps;
    }

    // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
    currentStep = 0;
    isMoving = true;
    lastTheta1 = targetTheta1;
    lastTheta2 = targetTheta2;
    lastTheta3 = targetTheta3;
}
void handleSetup() {
  if (isFirstTime) {
    // แสดงหน้าเว็บสำหรับการตั้งค่า Wi-Fi
    String html = "<html>";
    html = html + css;
    html += "<body>";
    html += "<h1>WiFi Setup</h1>";
    html += "<form action=\"/save-wifi\" method=\"POST\">";
    html += "SSID: <input type=\"text\" name=\"ssid\"><br>";
    html += "Password: <input type=\"password\" name=\"password\"><br>";
    html += "<input type=\"submit\" value=\"Save\">";
    html += "</form>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  } else {
    // โค้ดของคุณสำหรับหน้าหลัก (การควบคุมแขนกล)
    handleSetup();
  }
}

void handleSaveWiFi() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");

  if (ssid.length() > 0 && password.length() > 0) {
    // บันทึก SSID และ Password ลง EEPROM
    writeStringToEEPROM(0, ssid);
    writeStringToEEPROM(32, password);
    EEPROM.commit();

    server.send(200, "text/html", "WiFi settings saved. Restarting...");
    delay(2000);
    ESP.restart(); // รีสตาร์ท ESP32 เพื่อเชื่อมต่อ Wi-Fi ใหม่
  } else {
    server.send(200, "text/html", "Please provide both SSID and Password.");
  }
}
// ฟังก์ชันสำหรับแสดงหน้าเว็บพร้อมค่าสุดท้ายที่ผู้ใช้ส่ง
void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Robot Arm Control</h1>";
  html += "<form action=\"/move\" method=\"GET\">";
  html += "Joint 1 Angle: <input type=\"number\" name=\"theta1\" value=\"" + String(lastTheta1) + "\"><br>";
  html += "Joint 2 Angle: <input type=\"number\" name=\"theta2\" value=\"" + String(lastTheta2) + "\"><br>";
  html += "Joint 3 Angle: <input type=\"number\" name=\"theta3\" value=\"" + String(lastTheta3) + "\"><br>";
  html += "Gripper: <input type=\"number\" name=\"gripper\" value=\"" + String(lastGripper) + "\"><br>";
  html += "Speed: <input type=\"number\" name=\"speed\" value=\"" + String(lastSpeed) + "\"><br>";
  html += "Acceleration: <input type=\"number\" name=\"acceleration\" value=\"" + String(lastAcceleration) + "\"><br>";
  html += "<input type=\"submit\" value=\"Move\">";
  html += "</form>";

  // แสดงผลตำแหน่งปลายแขนกล
  html += "<h2>End Effector Position</h2>";
  html += "X: " + String(endEffectorX) + " mm<br>";
  html += "Y: " + String(endEffectorY) + " mm<br>";
  html += "Z: " + String(endEffectorZ) + " mm<br>";  // แสดงแกน Z

  // กำหนดพื้นที่สำหรับพล็อตกราฟ
  html += "<canvas id=\"myChart\" width=\"400\" height=\"400\" style=\"border:1px solid #000000;\"></canvas>";

  // เพิ่ม JavaScript สำหรับการวาดกราฟ
  html += "<script>";
  html += "function drawChart() {";
  html += "  var canvas = document.getElementById('myChart');";
  html += "  var ctx = canvas.getContext('2d');";
  html += "  ctx.clearRect(0, 0, canvas.width, canvas.height);";  // ล้างกราฟเก่า

  // คำนวณตำแหน่ง X, Y, Z โดยการสเกลให้เหมาะสมกับ canvas
  html += "  var scale = 2;";
  html += "  var x = " + String(endEffectorX) + " * scale + 200;";
  html += "  var y = " + String(endEffectorY) + " * scale + 200;";
  html += "  var z = " + String(endEffectorZ) + " * scale;";

  // วาดจุดปลายแขนกล
  html += "  ctx.beginPath();";
  html += "  ctx.arc(x, y, 5, 0, 2 * Math.PI);";  // วาดวงกลมที่ตำแหน่ง X, Y
  html += "  ctx.fillStyle = 'red';";
  html += "  ctx.fill();";

  // วาดข้อความแสดงค่า X, Y, Z ใกล้จุดที่วาด
  html += "  ctx.font = '12px Arial';";
  html += "  ctx.fillText('X: ' + Math.round(" + String(endEffectorX) + ") + ' mm', x + 10, y);";
  html += "  ctx.fillText('Y: ' + Math.round(" + String(endEffectorY) + ") + ' mm', x + 10, y + 15);";
  html += "  ctx.fillText('Z: ' + Math.round(" + String(endEffectorZ) + ") + ' mm', x + 10, y + 30);";

  html += "}";
  html += "drawChart();";  // เรียกฟังก์ชันวาดกราฟเมื่อโหลดหน้าเว็บ
  html += "</script>";

  html += "</body></html>";
  
  server.send(200, "text/html", html);
}
void setHomePosition(){
    // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ

setStepperSpeedAndAcceleration(3000, 500);
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  stepper1.moveTo(lastTheta1_home * theta1AngleToSteps); // theta1AngleToSteps
  stepper2.moveTo(lastTheta2_home * theta2AngleToSteps); // theta2AngleToSteps
  stepper3.moveTo(lastTheta3_home * phiAngleToSteps);  // phiAngleToSteps
  
  // เคลื่อนที่ทั้งหมด
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
stepper1.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
stepper2.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
stepper3.setCurrentPosition(0); // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // ควบคุม Gripper
  gripperServo.write(lastGripper);
}
// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleMove() {
 // รับค่าจากฟอร์ม
  targetTheta1_i  = server.arg("theta1").toInt();
  targetTheta2_i  = server.arg("theta2").toInt();
  targetTheta3_i  = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();

  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);



trajectoryPlanning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);  // 303: See Other (Redirect to GET)
}
void powerOffMotors() {
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    stepper3.disableOutputs();
}
// ฟังก์ชันสำหรับบันทึกข้อความลงใน EEPROM
void writeStringToEEPROM(int addr, const String &data) {
  int len = data.length();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, 0); // Null terminator
}

// ฟังก์ชันสำหรับอ่านข้อความจาก EEPROM
String readStringFromEEPROM(int addr) {
  
   len = 0;
  
  k = EEPROM.read(addr);
  while (k != 0 && len < 32) {
    k = EEPROM.read(addr + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  return String(data);
}
void setup() {
  Serial.begin(115200);

    EEPROM.begin(64);

  // ตรวจสอบว่าเคยบันทึก SSID และ Password หรือไม่
  //  savedSSID = readStringFromEEPROM(0);
  //  savedPassword = readStringFromEEPROM(32);



    // ตั้งค่า Limit Switch เป็น INPUT_PULLUP
    pinMode(limitSwitch1, INPUT_PULLUP);
    pinMode(limitSwitch2, INPUT_PULLUP);
    pinMode(limitSwitch3, INPUT_PULLUP);
  // เชื่อมต่อ Wi-Fi
  // เชื่อมต่อ Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  // // แสดง IP Address บน Serial Monitor
  // Serial.println("Connected to WiFi");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());  // แสดง IP Address ของ ESP32



  if (savedSSID.length() == 0 || savedPassword.length() == 0) {
    // ไม่มีการตั้งค่า Wi-Fi บันทึกไว้ ใช้งานโหมด AP
    Serial.println("Starting in AP mode...");
    isFirstTime = true;
    WiFi.softAP("RobotArmAP", "12345678"); // ชื่อและรหัสผ่าน Wi-Fi ของ AP

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } else {
    // มีการตั้งค่า Wi-Fi บันทึกไว้แล้ว เชื่อมต่อกับ Wi-Fi นั้น
    Serial.println("Connecting to WiFi...");
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }

    Serial.println("");
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }











  // เริ่มต้นการตั้งค่าเซอร์โวและมอเตอร์
  gripperServo.attach(13);
  gripperServo.write(180);
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  // เริ่มต้น Web Server
  server.on("/", handleRoot);       // หน้าเว็บหลัก
  server.on("/move", handleMove);   // รับคำสั่งการเคลื่อนที่
  server.on("/setup", handleSetup);       // หน้าเว็บหลัก
  server.on("/save-wifi", handleSaveWiFi);
  server.begin();
  Serial.println("Web server started");
  // setZero();
  // setHomePosition();

      
    // ตั้งค่า Timer ให้เรียกใช้ `onTimer` ทุกๆ 1 ms
    timer = timerBegin(0, 80, true);  // Timer 0, Prescaler 80, count up
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);  // เรียกทุกๆ 1000 ticks = 1 ms
    timerAlarmEnable(timer);  // เปิดใช้งาน Timer
}

void loop() {

      if (timerFlag) {
        // Reset flag
   portENTER_CRITICAL(&timerMux);
        timerFlag = false;
        portEXIT_CRITICAL(&timerMux);

        if (isMoving) {
            // ตั้งค่ามอเตอร์ให้เคลื่อนที่ไปยังตำแหน่งที่เก็บไว้
            stepper1.moveTo(theta1Steps[currentStep]);
            stepper2.moveTo(theta2Steps[currentStep]);
            stepper3.moveTo(theta3Steps[currentStep]);

            // รันมอเตอร์
            stepper1.run();
            stepper2.run();
            stepper3.run();

            // ตรวจสอบว่าไปยังตำแหน่งที่กำหนดแล้วหรือยัง
            if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
                currentStep++;  // ไปยังขั้นตอนถัดไป
                if (currentStep >= totalSteps) {
                    isMoving = false;  // หยุดการเคลื่อนที่เมื่อถึงขั้นตอนสุดท้าย
                    Serial.println("Movement completed");
                }
            }
        }
    }
  // จัดการคำร้องขอ HTTP
  server.handleClient();
}
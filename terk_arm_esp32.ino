#include <queue>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>  // สำหรับฟังก์ชันทางคณิตศาสตร์
#include "RobotArmIK.h"
#include <EEPROM.h>
const char *ssid = "Terk_2.4GHz";      // ชื่อ Wi-Fi ของคุณ
const char *password = "08110171188";  // รหัสผ่าน Wi-Fi ของคุณ

struct Command {
  String type;
  int theta1;
  int theta2;
  int theta3;
  int speed;
  int acceleration;
};

// สร้างคิวสำหรับจัดเก็บคำสั่ง
std::queue<Command> commandQueue;
// สร้างวัตถุ WebServer (ใช้พอร์ต 80)
WebServer server(80);
int len = 0;
int wifi_connecy_count = 0;
// สร้างวัตถุมอเตอร์และเซอร์โวเหมือนในโค้ดเดิม
AccelStepper stepper1(1, 17, 16);  // ข้อต่อ 1 หมุนรอบแกน Z
AccelStepper stepper2(1, 18, 5);   // ข้อต่อ 2
AccelStepper stepper3(1, 21, 19);  // ข้อต่อ 3
Servo gripperServo;
// กำหนด Limit Switch
#define limitSwitch1 26    // Limit switch สำหรับมอเตอร์ 1
#define limitSwitch2 25    // Limit switch สำหรับมอเตอร์ 2
#define limitSwitch3 33    // Limit switch สำหรับมอเตอร์ 3
#define DEBOUNCE_DELAY 10  // เวลาในการหน่วง debounce (50 มิลลิวินาที)

// ประกาศ Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define MAX_STEPS 1  // จำนวนขั้นตอนสูงสุดที่ต้องการสำหรับ trajectory

// ตัวแปรจัดเก็บตำแหน่งมุมของแต่ละขั้นตอน
int theta1Steps[MAX_STEPS];
int theta2Steps[MAX_STEPS];
int theta3Steps[MAX_STEPS];

int currentStep = 0;    // ขั้นตอนปัจจุบัน
int totalSteps = 0;     // จำนวนขั้นตอนทั้งหมด
bool isMoving = false;  // สถานะของการเคลื่อนที่

// ความยาวของข้อต่อ (สมมุติ)
double L1 = 135;          // ความยาวของข้อต่อแรก L1 = 135 มม.
double L2 = 157;          // ความยาวของข้อต่อที่สอง L2 = 147 มม.
double baseHeight = 156;  // ความสูงจากพื้น 156 มม.

int stepper1Position, stepper2Position, stepper3Position;
float theta1AngleToSteps = 77.222222;
float theta2AngleToSteps = 77.222222;
float phiAngleToSteps = 23.222222;

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

String Arm_mode = "JOINT";
String savedSSID = "";
String savedPassword = "";
String html = "";
String content = "";
String state = "";
bool isFirstTime = false;  // เช็คว่าคือการเริ่มต้นครั้งแรกหรือไม่
char data[32];

unsigned char k;



String ip = "";
String gateway = "";
String subnet = "";
String urlMain = "http://183.88.230.236:12000";
int delay_time;

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
  stepper1.setSpeed(-2000);  // เคลื่อนที่ไปทิศทางตรงข้ามเพื่อหาจุดเริ่มต้น
  // stepper1.setAcceleration(500);
  stepper2.setSpeed(-2000);
  // stepper2.setAcceleration(500);
  stepper3.setSpeed(-2000);
  // stepper3.setAcceleration(500);
  while (debounce(limitSwitch2) == HIGH) {
    stepper2.runSpeed();
  }
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // รันมอเตอร์จนกว่าจะถึง Limit Switch (พร้อม debounce)
  while (debounce(limitSwitch1) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  while (debounce(limitSwitch3) == HIGH) {
    stepper3.runSpeed();
  }
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch

  Serial.println("Set Zero: มอเตอร์ทั้งหมดถูกตั้งค่าที่ตำแหน่ง 0");
}
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
// ฟังก์ชันสำหรับตรวจสอบสถานะ Limit Switch
bool checkLimitSwitch() {
  bool limit1 = digitalRead(limitSwitch1) == LOW;  // LOW เมื่อถูกกด
  bool limit2 = digitalRead(limitSwitch2) == LOW;
  bool limit3 = digitalRead(limitSwitch3) == LOW;

  // หากมี Limit Switch ใดถูกกด, ให้หยุดการเคลื่อนที่
  if (limit1 || limit2 || limit3) {
    Serial.println("Limit switch triggered! Motion stopped.");
    return true;  // มี Limit Switch ถูกกด
  }
  return false;  // ไม่มี Limit Switch ถูกกด
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
    Command cmd;
    cmd.type = "move";
    cmd.theta1 = theta1Steps[i];
    cmd.theta2 = theta2Steps[i];
    cmd.theta3 = theta3Steps[i];
    cmd.speed = lastSpeed;
    cmd.acceleration = lastAcceleration;
    commandQueue.push(cmd);
  }

  // ตั้งค่าเริ่มต้นสำหรับการเคลื่อนที่
  currentStep = 0;
  isMoving = true;
  lastTheta1 = targetTheta1;
  lastTheta2 = targetTheta2;
  lastTheta3 = targetTheta3;
}
void handleSetup() {

  // แสดงหน้าเว็บสำหรับการตั้งค่า Wi-Fi
  html = "<html><body>";
  html += "<h1>WiFi and Network Setup</h1>";
  html += "<form action=\"/save-setup\" method=\"POST\">";
  html += "SSID: <input type=\"text\" name=\"ssid\"><br>";
  html += "Password: <input type=\"password\" name=\"password\"><br>";
  html += "Static IP Address: <input type=\"text\" name=\"ip\" value=\"" + readStringFromEEPROM(64) + "\"><br>";
  html += "Gateway: <input type=\"text\" name=\"gateway\" value=\"" + readStringFromEEPROM(96) + "\"><br>";
  html += "Subnet Mask: <input type=\"text\" name=\"subnet\" value=\"" + readStringFromEEPROM(128) + "\"><br>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void executeCommand(Command cmd) {
  // Set speed and acceleration
  stepper1.setMaxSpeed(cmd.speed);
  stepper1.setAcceleration(cmd.acceleration);
  stepper2.setMaxSpeed(cmd.speed);
  stepper2.setAcceleration(cmd.acceleration);
  stepper3.setMaxSpeed(cmd.speed);
  stepper3.setAcceleration(cmd.acceleration);

  // Move to target positions
  stepper1.moveTo(cmd.theta1);
  stepper2.moveTo(cmd.theta2);
  stepper3.moveTo(cmd.theta3);

  // Run motors
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}
void loadSettingsFromEEPROM() {
  theta1AngleToSteps = EEPROM.readFloat(0);
  theta2AngleToSteps = EEPROM.readFloat(4);
  phiAngleToSteps = EEPROM.readFloat(8);
  L1 = EEPROM.readFloat(12);
  L2 = EEPROM.readFloat(16);
  baseHeight = EEPROM.readFloat(20);

  // ตรวจสอบว่ามีการตั้งค่าไว้แล้วหรือไม่ ถ้าไม่ให้ใช้ค่าที่ตั้งไว้ในโค้ด
  if (theta1AngleToSteps == 0)
    theta1AngleToSteps = 77.222222;
  if (theta2AngleToSteps == 0)
    theta2AngleToSteps = 77.222222;
  if (phiAngleToSteps == 0)
    phiAngleToSteps = 23.222222;
  if (L1 == 0)
    L1 = 135;
  if (L2 == 0)
    L2 = 157;
  if (baseHeight == 0)
    baseHeight = 156;
}
void handleSaveWiFi() {
  savedSSID = server.arg("ssid");
  savedPassword = server.arg("password");
  ip = server.arg("ip");
  gateway = server.arg("gateway");
  subnet = server.arg("subnet");

  if (savedSSID.length() > 0 && savedPassword.length() > 0) {
    // บันทึก SSID และ Password
    writeStringToEEPROM(0, savedSSID);
    writeStringToEEPROM(32, savedPassword);
    EEPROM.commit();

    // บันทึก IP Address, Gateway และ Subnet
    writeStringToEEPROM(64, ip);
    writeStringToEEPROM(96, gateway);
    writeStringToEEPROM(128, subnet);
    EEPROM.commit();

    server.send(200, "text/html", "Settings saved. Restarting...");
    delay(2000);
    ESP.restart();
  } else {
    server.send(200, "text/html", "Please provide all required fields.");
  }
}
// ฟังก์ชันสำหรับแสดงหน้าเว็บพร้อมค่าสุดท้ายที่ผู้ใช้ส่ง
void handleRoot() {
  html = "<html><body>";
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
  // html = "<html><head>";
  // html += "<style>";
  // html += "body { font-family: Arial, sans-serif; background-color: #f0f0f0; display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; }";
  // html += ".container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0px 0px 15px rgba(0, 0, 0, 0.2); width: 400px; text-align: center; }";
  // html += "h1 { color: #333; }";
  // html += "button { padding: 10px 20px; margin: 5px; border: none; background-color: #007bff; color: white; border-radius: 5px; cursor: pointer; }";
  // html += "button:hover { background-color: #0056b3; }";
  // html += "form { margin-top: 20px; }";
  // html += "input[type='number'] { width: 100%; padding: 8px; margin: 5px 0; border: 1px solid #ccc; border-radius: 4px; }";
  // html += "input[type='submit'] { padding: 10px 20px; background-color: #28a745; color: white; border: none; border-radius: 5px; cursor: pointer; }";
  // html += "input[type='submit']:hover { background-color: #218838; }";
  // html += "</style></head><body>";

  // html += "<div class='container'>";
  // html += "<h1>Robot Arm Control</h1>";

  // // ปุ่มลิงก์ไปยังหน้าอื่นๆ
  // html += "<button onclick=\"location.href='/'\">Home</button>";
  // html += "<button onclick=\"location.href='/setup'\">Setup</button>";
  // html += "<button><a href=\"https://www.terkrobotics.com.z225694-4o13ru.ps03.zwhhosting.com/blockly?ip=" + ip + "\">Blockly</a>";

  // // เนื้อหาฟอร์มควบคุมแขนกล
  // html += "<form action=\"/move\" method=\"GET\">";
  // html += "Joint 1 Angle: <input type=\"number\" name=\"theta1\" value=\"" + String(lastTheta1) + "\"><br>";
  // html += "Joint 2 Angle: <input type=\"number\" name=\"theta2\" value=\"" + String(lastTheta2) + "\"><br>";
  // html += "Joint 3 Angle: <input type=\"number\" name=\"theta3\" value=\"" + String(lastTheta3) + "\"><br>";
  // html += "Gripper: <input type=\"number\" name=\"gripper\" value=\"" + String(lastGripper) + "\"><br>";
  // html += "Speed: <input type=\"number\" name=\"speed\" value=\"" + String(lastSpeed) + "\"><br>";
  // html += "Acceleration: <input type=\"number\" name=\"acceleration\" value=\"" + String(lastAcceleration) + "\"><br>";
  // html += "<input type=\"submit\" value=\"Move\">";
  // html += "</form>";

  // html += "</div></body></html>";
  // html = "<!DOCTYPE html><html lang=\"en\"><head>";
  // html += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  // html += "<title>3DOF Robotic Arm Joystick Control</title><style>";
  // html += "body { background-color: #f0f8ff; font-family: Arial, sans-serif; text-align: center; padding: 20px; }";
  // html += ".joystick { display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px); gap: 5px; justify-content: center; align-items: center; margin: 20px auto; }";
  // html += ".joystick button { width: 80px; height: 80px; font-size: 16px; border: none; background-color: #4CAF50; color: white; border-radius: 10px; transition: background-color 0.3s, transform 0.1s; }";
  // html += ".joystick button:hover { background-color: #45a049; }";
  // html += ".joystick button:active { background-color: #367636; transform: scale(0.95); }";
  // html += ".controls { margin-top: 20px; }";
  // html += ".controls button { margin: 5px; padding: 10px 20px; font-size: 16px; border: none; background-color: #008CBA; color: white; border-radius: 5px; transition: background-color 0.3s, transform 0.1s; }";
  // html += ".controls button:hover { background-color: #007bb5; }";
  // html += ".controls button:active { background-color: #005f87; transform: scale(0.95); }";
  // html += "input[type=\"number\"] { padding: 5px; font-size: 16px; border-radius: 5px; border: 1px solid #ccc; margin-right: 10px; }";
  // html += "h1, h2 { color: #333; }";
  // html += "@media (max-width: 600px) { .joystick { grid-template-columns: repeat(3, 60px); grid-template-rows: repeat(3, 60px); }";
  // html += ".joystick button { width: 60px; height: 60px; font-size: 14px; } .controls button { padding: 8px 15px; font-size: 14px; } input[type=\"number\"] { font-size: 14px; } }";
  // html += ".message { margin-top: 20px; color: #333; font-size: 18px; }</style>";
  // html += "<script>function sendCommand(command) {";
  // html += "const messageBox = document.getElementById('message'); messageBox.textContent = 'Sending command: ' + command + '...';";
  // html += "fetch(`/${command}`)";
  // html += ".then(response => { if (response.ok) { messageBox.textContent = 'Command sent successfully!'; } else { messageBox.textContent = 'Failed to send command!'; } })";
  // html += ".catch(error => { messageBox.textContent = 'Error sending command: ' + error; }); }";
  // html += "function adjustGripper() { const angle = document.getElementById('gripperAngle').value; sendCommand('set_gripper_angle?angle=' + angle); }</script></head><body>";
  // html += "<h1>3DOF Robotic Arm Joystick Control</h1><div class=\"joystick\">";
  // html += "<button onclick=\"sendCommand('up')\">↑</button><button onclick=\"sendCommand('stop')\">Stop</button><button onclick=\"sendCommand('down')\">↓</button>";
  // html += "<button onclick=\"sendCommand('left')\">←</button><button onclick=\"sendCommand('stop')\">Stop</button><button onclick=\"sendCommand('right')\">→</button></div>";
  // html += "<div class=\"controls\"><h2>Gripper Control</h2><button onclick=\"sendCommand('open_gripper')\">Open Gripper</button>";
  // html += "<button onclick=\"sendCommand('close_gripper')\">Close Gripper</button><br><br>";
  // html += "<label for=\"gripperAngle\">Set Gripper Angle:</label>";
  // html += "<input type=\"number\" id=\"gripperAngle\" min=\"0\" max=\"180\" value=\"90\">";
  // html += "<button onclick=\"adjustGripper()\">Set Angle</button></div><div id=\"message\" class=\"message\"></div></body></html>";
  // server.send(200, "text/html", html);
}

void handleMain() {
  html = "<!DOCTYPE html>";
  html += "<html lang=\"en\">";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>Robot Arm Learning Platform</title>";
  html += "<style>";
  html += "body {";
  html += "font-family: Arial, sans-serif;";
  html += " text-align: center;";
  html += "background-color: #f0f0f0;";
  html += "}";
  html += ".container {";
  html += "margin: 20px auto;";
  html += "padding: 20px;";
  html += "background-color: #fff;";
  html += "box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.1);";
  html += "max-width: 600px;";
  html += "}";
  html += " a {";
  html += "text-decoration: none;";
  html += "display: block;";
  html += " margin: 10px;";
  html += "padding: 10px;";
  html += "background-color: #4CAF50;";
  html += "color: white;";
  html += "border-radius: 5px;";
  html += "}";
  html += " a:hover {";
  html += "background-color: #45a049;";
  html += " }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += " <div class=\"container \">";
  html += " <h1>ยินดีต้อนรับสู่แพลตฟอร์มการเรียนรู้แขนหุ่นยนต์</h1>";
  html += " <h3>Select a Lesson:</h3>";
  html += " <a href=\"index\">การควบคุม x y z</a>";
  html += " <a href=\"" + urlMain + "/blockly?ip=" + ip + "\">Blockly</a>";
  html += " <a href=\"" + urlMain + "\">เนื้อหาการสร้างแขนกล</a>";
  html += " <h3>Control the Robot Arm:</h3>";
  html += " <a href=\"setup\">ตั้งค่า wifi</a>";
  html += " <a href=\"setuparm\">ตั้งค่า แขนกล</a>";
  html += " <a href=\"ota\">update firmware</a>";
  html += " </div>";
  html += "</body>";
  html += "</html>";
  server.send(200, "text/html", html);
}
// ฟังก์ชันการแสดงหน้าเว็บสำหรับตั้งค่า
void handleSetupPage() {
  html = "<html><body>";
  html += "<h1>Robot Arm Configuration</h1>";
  html += "<form action=\"/save-param\" method=\"POST\">";
  html += "Theta1 AngleToSteps: <input type=\"number\" step=\"0.1\" name=\"theta1AngleToSteps\" value=\"" + String(theta1AngleToSteps) + "\"><br>";
  html += "Theta2 AngleToSteps: <input type=\"number\" step=\"0.1\" name=\"theta2AngleToSteps\" value=\"" + String(theta2AngleToSteps) + "\"><br>";
  html += "Phi AngleToSteps: <input type=\"number\" step=\"0.1\" name=\"phiAngleToSteps\" value=\"" + String(phiAngleToSteps) + "\"><br>";
  html += "L1 (mm): <input type=\"number\" step=\"0.1\" name=\"L1\" value=\"" + String(L1) + "\"><br>";
  html += "L2 (mm): <input type=\"number\" step=\"0.1\" name=\"L2\" value=\"" + String(L2) + "\"><br>";
  html += "Base Height (mm): <input type=\"number\" step=\"0.1\" name=\"baseHeight\" value=\"" + String(baseHeight) + "\"><br>";
  html += "Static IP Address: <input type=\"text\" name=\"ip\" value=\"" + readStringFromEEPROM(64) + "\"><br>";
  html += "Gateway: <input type=\"text\" name=\"gateway\" value=\"" + readStringFromEEPROM(96) + "\"><br>";
  html += "Subnet Mask: <input type=\"text\" name=\"subnet\" value=\"" + readStringFromEEPROM(128) + "\"><br>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


void handleOta() {
  html = "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>";
  html += "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>";
  html += "<input type='file' name='update'>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<div id='prg'>progress: 0%</div>";
  html += "<script>";
  html += "$('form').submit(function(e){";
  html += "e.preventDefault();";
  html += "var form = $('#upload_form')[0];";
  html += "var data = new FormData(form);";
  html += " $.ajax({";
  html += "url: '/update',";
  html += "type: 'POST',";
  html += "data: data,";
  html += "contentType: false,";
  html += "processData:false,";
  html += "xhr: function() {";
  html += "var xhr = new window.XMLHttpRequest();";
  html += "xhr.upload.addEventListener('progress', function(evt) {";
  html += "if (evt.lengthComputable) {";
  html += "var per = evt.loaded / evt.total;";
  html += "$('#prg').html('progress: ' + Math.round(per*100) + '%');";
  html += "}";
  html += "}, false);";
  html += "return xhr;";
  html += "},";
  html += "success:function(d, s) {";
  html += "console.log('success!')";
  html += "},";
  html += "error: function (a, b, c) {";
  html += "}";
  html += "});";
  html += "});";
  html += "</script>";
  server.send(200, "text/html", html);
}
void handleSaveSetupArm() {
  // อ่านค่าจากฟอร์ม
  theta1AngleToSteps = server.arg("theta1AngleToSteps").toFloat();
  theta2AngleToSteps = server.arg("theta2AngleToSteps").toFloat();
  phiAngleToSteps = server.arg("phiAngleToSteps").toFloat();
  L1 = server.arg("L1").toFloat();
  L2 = server.arg("L2").toFloat();
  baseHeight = server.arg("baseHeight").toFloat();
  String ip = server.arg("ip");
  String gateway = server.arg("gateway");
  String subnet = server.arg("subnet");

  // บันทึกค่าลงใน EEPROM
  EEPROM.writeFloat(0, theta1AngleToSteps);
  EEPROM.writeFloat(4, theta2AngleToSteps);
  EEPROM.writeFloat(8, phiAngleToSteps);
  EEPROM.writeFloat(12, L1);
  EEPROM.writeFloat(16, L2);
  EEPROM.writeFloat(20, baseHeight);
  writeStringToEEPROM(64, ip);
  writeStringToEEPROM(96, gateway);
  writeStringToEEPROM(128, subnet);
  EEPROM.commit();

  server.send(200, "text/html", "<html><body><h1>Settings Saved</h1><a href=\"/setup\">Go Back</a></body></html>");
}

void setHomePosition() {
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ

  setStepperSpeedAndAcceleration(3000, 500);
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  stepper1.moveTo(lastTheta1_home * theta1AngleToSteps);  // theta1AngleToSteps
  stepper2.moveTo(lastTheta2_home * theta2AngleToSteps);  // theta2AngleToSteps
  stepper3.moveTo(lastTheta3_home * phiAngleToSteps);     // phiAngleToSteps

  // เคลื่อนที่ทั้งหมด
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  stepper1.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper2.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  stepper3.setCurrentPosition(0);  // ตั้งค่าเป็นศูนย์เมื่อถึง Limit Switch
  // ควบคุม Gripper
  gripperServo.write(lastGripper);
}

void setHomeArm() {
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ

  setStepperSpeedAndAcceleration(3000, 500);
  // แปลงมุมเป็นสเต็ปและเคลื่อนมอเตอร์
  stepper1.moveTo(0);  // theta1AngleToSteps
  stepper2.moveTo(0);  // theta2AngleToSteps
  stepper3.moveTo(0);  // phiAngleToSteps

  // เคลื่อนที่ทั้งหมด
  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {

    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}
// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleMove() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("theta1").toInt();
  targetTheta2_i = server.arg("theta2").toInt();
  targetTheta3_i = server.arg("theta3").toInt();
  lastGripper = server.arg("gripper").toInt();
  lastSpeed = server.arg("speed").toInt();
  lastAcceleration = server.arg("acceleration").toInt();
  Arm_mode = String(server.arg("mode"));
  Serial.println("------handleMove-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);
  Serial.println(lastGripper);
  Serial.println(lastSpeed);
  Serial.println(lastAcceleration);
  Serial.println(Arm_mode);
  Serial.println("-------------");
  // คำนวณตำแหน่งปลายแขนกลด้วย Forward Kinematics
  // calculateForwardKinematics(lastTheta1, lastTheta2, lastTheta3);

  // ตั้งค่าความเร็วและความเร่งที่ได้รับจากเว็บ
  setStepperSpeedAndAcceleration(lastSpeed, lastAcceleration);

  trajectoryPlanning(targetTheta1_i, targetTheta2_i, targetTheta3_i, 7);

  // ควบคุม Gripper
  gripperServo.write(lastGripper);

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}

void handleOutput() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------Output-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "output";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleIf() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("io").toInt();
  state = server.arg("state");
  Serial.println("------if-------");
  Serial.println(targetTheta1_i);
  Serial.println(state);

  Command cmd;
  cmd.type = "if";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleGripper() {
  // รับค่าจากฟอร์ม
  lastGripper = server.arg("gripper").toInt();


  Serial.println("------Gripper-------");

  Serial.println(lastGripper);
  // ควบคุม Gripper
  //
  Command cmd;
  cmd.type = "Gripper";
  cmd.theta1 = lastGripper;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleDelay() {
  // รับค่าจากฟอร์ม
  delay_time = server.arg("time").toInt();



  Serial.println("------delay_time-------");
  Serial.println(delay_time);
  Command cmd;
  cmd.type = "delay";
  cmd.theta1 = delay_time;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ควบคุม Gripper

  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}

// ฟังก์ชันสำหรับรับคำสั่งการเคลื่อนที่จากเว็บเบราว์เซอร์
void handleFor() {
  // รับค่าจากฟอร์ม
  targetTheta1_i = server.arg("BY").toInt();
  targetTheta2_i = server.arg("FROM").toInt();
  targetTheta3_i = server.arg("TO").toInt();

  Serial.println("------for-------");
  Serial.println(targetTheta1_i);
  Serial.println(targetTheta2_i);
  Serial.println(targetTheta3_i);


  Command cmd;
  cmd.type = "for";
  cmd.theta1 = targetTheta1_i;
  cmd.theta2 = targetTheta2_i;
  cmd.theta3 = targetTheta3_i;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);
  // ทำการ Redirect กลับไปที่หน้าแรก (ฟอร์มควบคุม)
  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleHome() {
  Serial.println("------home-------");

  Command cmd;
  cmd.type = "home";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void handleZero() {
  Serial.println("------zero-------");

  Command cmd;
  cmd.type = "zero";
  cmd.theta1 = 0;
  cmd.theta2 = 0;
  cmd.theta3 = 0;
  cmd.speed = 0;
  cmd.acceleration = 0;
  commandQueue.push(cmd);

  server.sendHeader("Location", "/");  // กลับไปที่ URL "/"
  server.send(303);                    // 303: See Other (Redirect to GET)
}
void powerOffMotors() {
  stepper1.disableOutputs();
  stepper2.disableOutputs();
  stepper3.disableOutputs();
}
// ฟังก์ชันสำหรับบันทึกข้อความลงใน EEPROM
void writeStringToEEPROM(int addr, const String &data) {
  len = data.length();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, 0);  // Null terminator
  EEPROM.commit();              // บันทึกข้อมูลลง EEPROM จริงๆ
}

// ฟังก์ชันสำหรับอ่านข้อความจาก EEPROM
String readStringFromEEPROM(int addr) {
  memset(data, 0, sizeof(data));

  len = 0;

  k = EEPROM.read(addr);
  Serial.println("-------------");
  while (k != 0 && len < 32) {
    data[len] = k;  // เก็บค่าที่อ่านได้ในตัวแปร data
    len++;
    k = EEPROM.read(addr + len);  // อ่านตัวอักษรถัดไปจาก EEPROM
    Serial.println(k);            // แสดงค่าที่อ่านจาก EEPROM เพื่อวิเคราะห์
  }
  Serial.println("-------------");
  data[len] = '\0';  // สิ้นสุด string ด้วย null terminator

  return String(data);  // แปลง char array เป็น String และส่งคืน
}
void setup() {
  Serial.begin(115200);

  EEPROM.begin(500);

  // ตรวจสอบว่าเคยบันทึก SSID และ Password หรือไม่

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
  // writeStringToEEPROM(0, "0");
  // writeStringToEEPROM(32 ,"0");
  savedSSID = readStringFromEEPROM(0);
  savedPassword = readStringFromEEPROM(32);
  ip = readStringFromEEPROM(64);
  gateway = readStringFromEEPROM(96);
  subnet = readStringFromEEPROM(128);

  Serial.println(savedSSID);
  Serial.println(savedPassword);

  if (savedSSID.length() < 2) {
    Serial.println("Starting in AP mode...");
    isFirstTime = true;
    WiFi.softAP("RobotArmAP", "12345678");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } else {
    // ตรวจสอบและตั้งค่า IP แบบคงที่จากข้อมูลที่บันทึก
    if (ip.length() > 0 && gateway.length() > 0 && subnet.length() > 0) {
      IPAddress local_IP, local_gateway, local_subnet;
      local_IP.fromString(ip);
      local_gateway.fromString(gateway);
      local_subnet.fromString(subnet);

      if (!WiFi.config(local_IP, local_gateway, local_subnet)) {
        Serial.println("STA Failed to configure");
      }
    }

    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
      wifi_connecy_count++;
      if (wifi_connecy_count > 20) {
        writeStringToEEPROM(0, "0");
        writeStringToEEPROM(32, "0");
        ESP.restart();
      }
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

  setZero();
  setHomePosition();
  server.enableCORS();
  // เริ่มต้น Web Server
  server.on("/", handleMain);       // หน้าเว็บหลัก
  server.on("/index", handleRoot);  // หน้าเว็บหลัก
  server.on("/move", handleMove);   // รับคำสั่งการเคลื่อนที่

  server.on("/setup", handleSetup);         // หน้าเว็บหลัก
  server.on("/setuparm", handleSetupPage);  // หน้าเว็บหลัก
  server.on("/save-setup", handleSaveWiFi);
  server.on("/save-param", handleSaveSetupArm);
  server.on("/controlGripper", handleGripper);
  server.on("/delay", handleDelay);
  server.on("/for", handleFor);
  server.on("/home", handleHome);
  server.on("/zero", handleZero);
  server.on("/ota", handleOta);  //
  server.on("/output", handleOutput);
  server.on("/if", handleIf);
  /*handling uploading firmware file */
  server.on(
    "/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    []() {
      HTTPUpload &upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {  //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });
  server.begin();
  Serial.println("Web server started");
  // ตั้งค่า Timer ให้เรียกใช้ `onTimer` ทุกๆ 1 ms
  timer = timerBegin(0, 80, true);  // Timer 0, Prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);  // เรียกทุกๆ 1000 ticks = 1 ms
  timerAlarmEnable(timer);             // เปิดใช้งาน Timer
}

void loop() {
  serial();
  if (!commandQueue.empty()) {
    Command cmd = commandQueue.front();
    if (!commandQueue.empty()) {

      if (cmd.type == "move") {
        executeCommand(cmd);
      } else if (cmd.type == "delay") {
        delay(cmd.theta1);
      } else if (cmd.type == "Gripper") {
        gripperServo.write(cmd.theta1);
      } else if (cmd.type == "home") {
        setHomeArm();
      } else if (cmd.type == "zero") {
        setZero();
      } else if (cmd.type == "output") {

      } else {
        // Handle other cases if needed
      }
    }
    commandQueue.pop();
  }



  // จัดการคำร้องขอ HTTP
  server.handleClient();
}

void serial() {
  if (Serial.available()) {
    content = Serial.readString();  // อ่านข้อมูลจาก Processing

    Serial.println(content);
    // แยกข้อมูลจากสตริงและใส่ในตัวแปรอาร์เรย์ data[]
    for (int i = 0; i < 10; i++) {
      int index = content.indexOf(",");                     // ค้นหาเครื่องหมาย ","
      data[i] = atol(content.substring(0, index).c_str());  // แยกตัวเลขจากข้อมูล
      content = content.substring(index + 1);               // ลบตัวเลขออกจากสตริง
    }
  }
}
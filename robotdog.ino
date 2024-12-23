#include "ROBOT.h"
#include <ArduinoJson.h>
#include "ROBOT.h"
#include "FKIK.h"
#include <Wire.h>
#include "Attitude.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> 
#include "IMU.h"
#include "LittleFS.h"


// 定义舵机
float Servo0_Init = 97.0;
float Servo1_Init = 92.0;
float Servo2_Init = 90.0;
float Servo3_Init = 91.0;
float Servo4_Init = 88.0;
float Servo5_Init = 98.0;
float Servo6_Init = 85.0;
float Servo7_Init = 95.0;
float Servo8_Init = 85.0;
float Servo9_Init = 100.0;
float Servo10_Init = 82.0;
float Servo11_Init = 80.0;





// 全局变量
float currentX = 0;
float currentY = 0;
float currentZ = 0;
float currentRoll = 0;
float currentPitch = 0;
float currentYaw = 0;

// 目标全局变量
float targetX = 0;
float targetY = 0;
float targetZ = 0;
float targetRoll = 0;
float targetPitch = 0;
float targetYaw = 0;

// 定义舵机角度
float servoAngles[4][3] = {
    {90, 90, 90}, 
    {90, 90, 90}, 
    {90, 90, 90}, 
    {90, 90, 90}   
};



const char *ssid = "ESP01AP";   
const char *password = "12345678";  

WebServer server(80);
WebSocketsServer webSocket(81);
bool isMarkingTime = false;  
bool isRobotBusy = false;    


unsigned long lastMoveTime = 0;
unsigned long moveInterval = 10;  // 10 毫秒调用一次舵机移动


void gradualMove(float targetX, float targetY, float targetZ, float targetRoll, float targetPitch, float targetYaw);

float lerp(float start, float end, float t) {
    return start + (end - start) * t;
}


void gradualMoveX() {
    float stepRatio = 0.2; 
    currentX = lerp(currentX, targetX, stepRatio); 

    //控制舵机的X轴
    attitude_control(currentX, currentY, currentZ, currentRoll, currentPitch, currentYaw);
}


void gradualMove(float targetX, float targetY, float targetZ, float targetRoll, float targetPitch, float targetYaw) {
    float stepRatio = 0.3; 

    
    currentX = lerp(currentX, targetX, stepRatio);
    currentY = lerp(currentY, targetY, stepRatio);
    currentZ = lerp(currentZ, targetZ, stepRatio);
    currentRoll = lerp(currentRoll, targetRoll, stepRatio);
    currentPitch = lerp(currentPitch, targetPitch, stepRatio);
    currentYaw = lerp(currentYaw, targetYaw, stepRatio);

    
    attitude_control(currentX, currentY, currentZ, currentRoll, currentPitch, currentYaw);
}



float calculateDynamicStepRatio(float current, float target) {
    float distance = abs(target - current);
    if (distance > 10) {
        return 0.2; 
    } else if (distance > 2) {
        return 0.1; 
    } else {
        return 0.1; 
    }
}



// 发送 HTML 页面
void handleRoot() {
  Serial.println("Handling root request for /index.html");

  // 检查文件是否存在于 LittleFS 中
  if (LittleFS.exists("/index.html")) {
    Serial.println("index.html found in LittleFS");
    File file = LittleFS.open("/index.html", "r");  
    server.streamFile(file, "text/html");           
    file.close();                                   
  } else {
    Serial.println("index.html not found in LittleFS");
    server.send(404, "text/plain", "File Not Found");  
  }
}

// 处理PNG
void handleFileRequest() {
  String path = server.uri();
  Serial.print("Handling file request for: ");
  Serial.println(path);

  if (path.endsWith("/")) path += "index.html"; 


  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    String contentType = "text/plain";


    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg")) contentType = "image/jpeg";
    else if (path.endsWith(".ico")) contentType = "image/x-icon";

    Serial.print("File found: ");
    Serial.println(path);
    server.streamFile(file, contentType);  
    file.close();
  } else {
    Serial.print("File not found: ");
    Serial.println(path);
    server.send(404, "text/plain", "File Not Found");
  }
}

// 处理控制命令
void handleCommand() {
  String command = server.uri();
  command.remove(0, 1); 

  Serial.print("Received command: ");
  Serial.println(command);


  if (isRobotBusy) {
    Serial.println("Robot is busy, ignoring command.");
    server.send(200, "text/plain", "Robot is busy");
    return;
  }

  isRobotBusy = true;  // 标记机器人

  // 命令
  if (command == "forward") {
    Trot(10, 1);  
  } else if (command == "backward") {
    Trot(10, 0);  
  } else if (command == "turnleft") {
    TrotTurn(10, 0);  
  } else if (command == "turnright") {
    TrotTurn(10, 1);  
  } else if (command == "marktime") {
    if (isMarkingTime) {
      Robot_IK_Stand();  
      isMarkingTime = false;
    } else {
      TrotMark(10, 1);  
      isMarkingTime = true;
    }
  } else if (command == "moveleft") {
    TrotRL(10, 0);  
  } else if (command == "moveright") {
    TrotRL(10, 1); 
  }

  Serial.println("Command executed.");
  server.send(200, "text/plain", "OK");

}



// 姿态控制
void handleAttitudeControl() {
    
    if (isRobotBusy) {
        server.send(200, "text/plain", "Robot is busy");
        return;
    }

    
    if (server.hasArg("x") && server.hasArg("y") && server.hasArg("z") &&
        server.hasArg("roll") && server.hasArg("pitch") && server.hasArg("yaw")) {

        
        targetX = constrain(server.arg("x").toFloat(), -25, 25);
        targetY = constrain(server.arg("y").toFloat(), -25, 25);
        targetZ = constrain(server.arg("z").toFloat(), -25, 25);
        targetRoll = constrain(server.arg("roll").toFloat(), -25, 25);
        targetPitch = constrain(server.arg("pitch").toFloat(), -25, 25);
        targetYaw = constrain(server.arg("yaw").toFloat(), -25, 25);

        
        isRobotBusy = true;

        server.send(200, "text/plain", "Control command received");
    } else {
        server.send(400, "text/plain", "Missing arguments");
    }
}



bool isCalibrationMode = false;  

int mapToPulse(float angle) {
    return map(angle, 0, 300, 40, 983);
}



void loadCalibrationData() {
    if (LittleFS.exists("/servo_config.json")) {
        File file = LittleFS.open("/servo_config.json", "r");
        if (file) {
            StaticJsonDocument<512> jsonDoc;
            DeserializationError error = deserializeJson(jsonDoc, file);
            if (!error) {
                Servo0_Init = jsonDoc["servo_0"] | Servo0_Init;
                Servo1_Init = jsonDoc["servo_1"] | Servo1_Init;
                Servo2_Init = jsonDoc["servo_2"] | Servo2_Init;
                Servo3_Init = jsonDoc["servo_3"] | Servo3_Init;
                Servo4_Init = jsonDoc["servo_4"] | Servo4_Init;
                Servo5_Init = jsonDoc["servo_5"] | Servo5_Init;
                Servo6_Init = jsonDoc["servo_6"] | Servo6_Init;
                Servo7_Init = jsonDoc["servo_7"] | Servo7_Init;
                Servo8_Init = jsonDoc["servo_8"] | Servo8_Init;
                Servo9_Init = jsonDoc["servo_9"] | Servo9_Init;
                Servo10_Init = jsonDoc["servo_10"] | Servo10_Init;
                Servo11_Init = jsonDoc["servo_11"] | Servo11_Init;

                Serial.println("校准数据已加载，并应用到舵机初始化角度。");
                serializeJsonPretty(jsonDoc, Serial);
            } else {
                Serial.print("校准数据解析错误: ");
                Serial.println(error.c_str());
            }
            file.close();
        } else {
            Serial.println("无法打开校准数据文件");
        }
    } else {
        Serial.println("未找到校准数据文件，使用默认设置");
    }
}






void saveCalibrationData() {
    StaticJsonDocument<512> jsonDoc;

    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            String key = "servo_" + String(i * 3 + j);
            float angle = servoAngles[i][j];
            jsonDoc[key] = angle;

            // 更新全局变量
            switch (i * 3 + j) {
                case 0: Servo0_Init = angle; break;
                case 1: Servo1_Init = angle; break;
                case 2: Servo2_Init = angle; break;
                case 3: Servo3_Init = angle; break;
                case 4: Servo4_Init = angle; break;
                case 5: Servo5_Init = angle; break;
                case 6: Servo6_Init = angle; break;
                case 7: Servo7_Init = angle; break;
                case 8: Servo8_Init = angle; break;
                case 9: Servo9_Init = angle; break;
                case 10: Servo10_Init = angle; break;
                case 11: Servo11_Init = angle; break;
            }
        }
    }

    
    File file = LittleFS.open("/servo_config.json", "w");
    if (file) {
        serializeJson(jsonDoc, file);
        file.close();
        Serial.println("校准数据已保存，文件内容:");
        serializeJsonPretty(jsonDoc, Serial);
    } else {
        Serial.println("无法打开文件进行保存");
    }
}



void setup() {
    Serial.begin(115200);
    IMU_INIT();  
    servo_init();  
    delay(500);  
    Serial.println("IMU 和舵机初始化完成");
    if (!isCalibrationMode) {
        Robot_IK_Stand();  
        Serial.println("机器人进入站立状态。");
    }
    if (!LittleFS.begin()) {
        Serial.println("挂载LittleFS文件系统时出错");
        return;  
    }
    Serial.println("LittleFS文件系统挂载成功");
    loadCalibrationData(); 
    WiFi.softAP(ssid, password);  
    IPAddress IP = WiFi.softAPIP(); 
    Serial.print("AP IP地址: ");
    Serial.println(IP);
    server.on("/control", handleAttitudeControl);  
    server.on("/turnleft", handleCommand);  
    server.on("/turnright", handleCommand); 
    server.onNotFound(handleFileRequest);  
    server.begin();
    Serial.println("Web服务器已启动");
    webSocket.begin(); 
    webSocket.onEvent(webSocketEvent);  
    Serial.println("WebSocket服务器已启动");
}






void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.print("JSON 解析错误: ");
            Serial.println(error.c_str());
            return;
        }

        
        if (doc.containsKey("action") && doc["action"] == "saveCalibration") {
            saveCalibrationData();
            Serial.println("收到保存校准数据请求");
            return;
        }

        
        if (doc.containsKey("page")) {
            String page = doc["page"].as<String>();

            
            if ((page == "page3" && !isCalibrationMode) || (page != "page3" && isCalibrationMode)) {
                handlePageChange(page);
            } else {
                Serial.println("页面状态未改变，忽略页面切换命令。");
            }
            return;  
        }

        
        Serial.print("当前操作模式: ");
        Serial.println(isCalibrationMode ? "校准模式" : "正常模式");

        
        if (isCalibrationMode) {
            handleCalibrationMode(doc);
        } else {
            handleNormalMode(doc);
        }
    }
}


void handlePageChange(String page) {
    if (page == "page3") {
        isCalibrationMode = true;
        Serial.println("进入第三页：舵机校准模式");

        
        for (int i = 0; i < 12; i++) {
            int legIndex = i / 3;
            int servoOnLeg = i % 3;

            
            switch (i) {
                case 0: servoAngles[legIndex][servoOnLeg] = Servo0_Init; break;
                case 1: servoAngles[legIndex][servoOnLeg] = Servo1_Init; break;
                case 2: servoAngles[legIndex][servoOnLeg] = Servo2_Init; break;
                case 3: servoAngles[legIndex][servoOnLeg] = Servo3_Init; break;
                case 4: servoAngles[legIndex][servoOnLeg] = Servo4_Init; break;
                case 5: servoAngles[legIndex][servoOnLeg] = Servo5_Init; break;
                case 6: servoAngles[legIndex][servoOnLeg] = Servo6_Init; break;
                case 7: servoAngles[legIndex][servoOnLeg] = Servo7_Init; break;
                case 8: servoAngles[legIndex][servoOnLeg] = Servo8_Init; break;
                case 9: servoAngles[legIndex][servoOnLeg] = Servo9_Init; break;
                case 10: servoAngles[legIndex][servoOnLeg] = Servo10_Init; break;
                case 11: servoAngles[legIndex][servoOnLeg] = Servo11_Init; break;
            }

            
            int pulse = mapToPulse(servoAngles[legIndex][servoOnLeg]);
            sc.WritePosEx(i, pulse, 100, 0);
            delay(50);
        }

        Serial.println("舵机角度已按照初始值设置进入校准模式。");
    } else {
        isCalibrationMode = false;
        Serial.println("退出第三页：恢复正常控制模式");

        
        saveCalibrationData();

        
        applyCalibrationData();
    }
}


void applyCalibrationData() {
    Serial.println("根据校准数据设置舵机位置");
    for (int i = 0; i < 12; i++) {
        int legIndex = i / 3;
        int servoOnLeg = i % 3;
        float angle = servoAngles[legIndex][servoOnLeg];

        int pulse = mapToPulse(angle);
        sc.WritePosEx(i, pulse, 100, 0);
        delay(50);  
    }
}



void handleCalibrationMode(const StaticJsonDocument<200>& doc) {
    if (doc.containsKey("servoIndex") && doc.containsKey("increment")) {
        int servoIndex = doc["servoIndex"];
        int increment = doc["increment"];

        if (servoIndex >= 0 && servoIndex < 12) {
            adjustLegServo(servoIndex, increment);
            Serial.print("第三页舵机校准模式：调整舵机 ");
            Serial.print(servoIndex);
            Serial.print(" 增量: ");
            Serial.println(increment);
        } else {
            Serial.println("舵机索引无效，忽略命令。");
        }
    } else {
        Serial.println("缺少舵机控制参数，无法处理校准命令。");
    }
}


void handleNormalMode(const StaticJsonDocument<200>& doc) {
    if (doc.containsKey("servoIndex") && doc.containsKey("increment")) {
        int servoIndex = doc["servoIndex"];
        int increment = doc["increment"];

        if (isRobotBusy) {
            Serial.println("机器人忙碌中，忽略舵机调整命令。");
            return;
        }

        if (servoIndex >= 0 && servoIndex < 12) {
            isRobotBusy = true;  // 标记机器人为忙碌状态
            adjustLegServo(servoIndex, increment);
            Serial.println("舵机调整完成。");
            isRobotBusy = false;  // 重置状态为不忙碌
        } else {
            Serial.println("输入无效，舵机编号超出范围。");
        }
    }

    // 处理姿态调整（X、Y、Z、Roll、Pitch、Yaw）
    if (doc.containsKey("x")) {
        float xValue = doc["x"].as<float>();
        targetX = constrain(xValue, -25, 25);
        Serial.print("X轴: ");
        Serial.println(targetX);
    }
    if (doc.containsKey("y")) {
        float yValue = doc["y"].as<float>();
        targetY = constrain(yValue, -25, 25);
        Serial.print("Y轴: ");
        Serial.println(targetY);
    }
    if (doc.containsKey("z")) {
        float zValue = doc["z"].as<float>();
        targetZ = constrain(zValue, -25, 25);
        Serial.print("Z轴: ");
        Serial.println(targetZ);
    }
    if (doc.containsKey("roll")) {
        float rollValue = doc["roll"].as<float>();
        targetRoll = constrain(rollValue, -25, 25);
        Serial.print("横滚: ");
        Serial.println(targetRoll);
    }
    if (doc.containsKey("pitch")) {
        float pitchValue = doc["pitch"].as<float>();
        targetPitch = constrain(pitchValue, -25, 25);
        Serial.print("俯仰: ");
        Serial.println(targetPitch);
    }
    if (doc.containsKey("yaw")) {
        float yawValue = doc["yaw"].as<float>();
        targetYaw = constrain(yawValue, -25, 25);
        Serial.print("偏航: ");
        Serial.println(targetYaw);
    }


    gradualMove(targetX, targetY, targetZ, targetRoll, targetPitch, targetYaw);


    if (doc.containsKey("command")) {
        String command = doc["command"].as<String>();

        if (command == "forward") {
            Serial.println("执行前进命令");
            Trot(10, 1);  
        } else if (command == "backward") {
            Serial.println("执行后退命令");
            Trot(10, 0);  
        } else if (command == "turnleft") {
            Serial.println("执行左转命令");
            TrotTurn(10, 0);  
        } else if (command == "turnright") {
            Serial.println("执行右转命令");
            TrotTurn(10, 1);  
        } else if (command == "marktime") {
            Serial.println("执行原地踏步命令");
            if (isMarkingTime) {
                Robot_IK_Stand();  
                isMarkingTime = false;
            } else {
                TrotMark(10, 1);  
                isMarkingTime = true;
            }
        } else if (command == "moveleft") {
            Serial.println("执行左横移命令");
            TrotRL(10, 0);  
        } else if (command == "moveright") {
            Serial.println("执行右横移命令");
            TrotRL(10, 1);  
        }
    }
}



void loop() {
    server.handleClient();
    webSocket.loop();

    if (!isCalibrationMode) {
        unsigned long currentTime = millis();
        if (currentTime - lastMoveTime >= moveInterval) {
            gradualMove(targetX, targetY, targetZ, targetRoll, targetPitch, targetYaw);
            lastMoveTime = currentTime;
        }
    }
}

unsigned long previousServoMillis = 0;  
const long servoInterval = 50;          

void adjustLegServo(int servoIndex, int increment) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousServoMillis < servoInterval) {
        Serial.println("操作过于频繁，请稍后再试。");
        return;
    }
    previousServoMillis = currentMillis;  

    int legIndex = servoIndex / 3;  
    int servoOnLeg = servoIndex % 3;

    
    servoAngles[legIndex][servoOnLeg] = constrain(servoAngles[legIndex][servoOnLeg] + increment, 0, 300);
    Serial.print("调整腿 ");
    Serial.print(legIndex);
    Serial.print(" 的舵机 ");
    Serial.print(servoOnLeg);
    Serial.print(" 新角度为: ");
    Serial.println(servoAngles[legIndex][servoOnLeg]);

    int pulse = mapToPulse(servoAngles[legIndex][servoOnLeg]);
    sc.WritePosEx(servoIndex, pulse, 100, 0);  

    
    isRobotBusy = false;  
    Serial.println("isRobotBusy 已重置为 false");
}




void setServoAngle(int legIndex, int servoOnLeg, float angle) {
    Serial.print("设置腿 ");
    Serial.print(legIndex);
    Serial.print(" 的舵机 ");
    Serial.print(servoOnLeg);
    Serial.print(" 角度为: ");
    Serial.println(angle);

    switch(legIndex) {
        case 0: // 
            Serial.println(" 控制左前腿舵机");
            FK_LUMove(servoAngles[legIndex][0], servoAngles[legIndex][1], servoAngles[legIndex][2], 1000);
            break;
        case 1: // 
            Serial.println("控制右前腿舵机");
            FK_RUMove(servoAngles[legIndex][0], servoAngles[legIndex][1], servoAngles[legIndex][2], 1000);
            break;
        case 2: // 
            Serial.println("控制左后腿舵机");
            FK_LBMove(servoAngles[legIndex][0], servoAngles[legIndex][1], servoAngles[legIndex][2], 1000);
            break;
        case 3: // 
            Serial.println("控制右后腿舵机");
            FK_RBMove(servoAngles[legIndex][0], servoAngles[legIndex][1], servoAngles[legIndex][2], 1000);
            break;
    }
}















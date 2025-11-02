
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoJson.h> 
#include <ESPAsyncWebServer.h>

Servo esc;
const char* ssid = "Top_2G";
const char* password = "Toptop@1";
//int LED=LED_BUILTIN;

/*void setup() {
  esc.attach(9); // connect ESC signal to digital pin 9
  esc.writeMicroseconds(1000); // zero throttle
  delay(2000); // wait for ESC to arm (some ESCs need this)
}

void loop() {
  // Example: slowly increase throttle then back down
  for (int pw = 1000; pw <= 1700; pw += 10) {
    esc.writeMicroseconds(pw);
    delay(50);
  }
  delay(500);
  for (int pw = 1700; pw >= 1000; pw -= 10) {
    esc.writeMicroseconds(pw);
    delay(50);
  }
  delay(1000);
}*/
///Controlling all 4 motors
//int motorPins[4]={9,10,11,12};
int motorSpeeds[4] = {1000, 1000, 1000, 1000};  // Current μs for each (FL, FR, RL, RR)
int frontMotors[2] = {0, 1};  // Front-left & front-right
int backMotors[2]  = {2, 3};  // Back-left & back-right
int leftMotors[2]  = {0, 2};  // Front-left & back-left
int rightMotors[2] = {1, 3};  // Front-right & back-ri
const int MIN_SPEED = 1000;  // Arm/idle (no spin)
const int MAX_SPEED = 2000;  // Full throttle
int newFrontSpeed ;
  int newBackSpeed ;
  int newLeftMotors;
  int newRightMotors;
const int DELTA = 50;
const int DELTA2=50; // Ramp step (tune: 25=snappy, 100=slow)
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
int ledPin=2;
Servo motors[4];
 String command="";
 String lastCommand = "";
int motorPins[4]={15,17,19,22};
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        String msg = "";
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
      command=msg;
     
    }
     Serial.println(" Processing::\n"+ msg);
     
       controllMotors();
       
       
    }
}

 void speedStabilizer(){
    

 }
void controllMotors() {
  StaticJsonDocument<200> json;

  json["type"] = "motorSpeeds";
  JsonArray speeds = json.createNestedArray("values");
   String jsonString;

  /*if (command.length() == 0 || command == lastCommand) return;  // Skip empties/repeats
  lastCommand = command;*/
  

  // Helper: Clamp & write (reusable)
  auto updateMotor = [](Servo& motor, int& speed) {
      speed = constrain(speed, MIN_SPEED, MAX_SPEED);
      motor.writeMicroseconds(speed);
  };
  
  if (command == "throttleUp") {  // Gradual all-up
    Serial.println("Throttle Up All =="+ DELTA);

    ///Speed stabilizer
     if(motorSpeeds[frontMotors[0]] != motorSpeeds[frontMotors[1]]||
   motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]]){
    motorSpeeds[frontMotors[0]]=motorSpeeds[frontMotors[1]];
     motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]];

     for (int i = 0; i < 2; i++) {
    
    motorSpeeds[frontMotors[i]] += DELTA;
    motorSpeeds[backMotors[i]]  += DELTA;
     updateMotor(motors[frontMotors[0]], motorSpeeds[frontMotors[1]]);
    updateMotor(motors[backMotors[0]],  motorSpeeds[backMotors[1]]);
     Serial.printf("[%d] satbilised front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] satbilised back motors = %d\n", i, motorSpeeds[backMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
  }

   }

    for (int i = 0; i < 4; i++) {
     if (motorSpeeds[i]>= 2000) {
          for(int i=0;i<4;i++){
            motorSpeeds[i]={1950};
            updateMotor(motors[i], motorSpeeds[i]);
           speeds.add(motorSpeeds[i]);
           Serial.printf("[%d] = %d  ", i, motorSpeeds[i]);    
             ws.textAll("Cannot Pitch Up motor... limit reached\n");
          }
       
        return;
        }
    
      motorSpeeds[i] += DELTA;
      updateMotor(motors[i], motorSpeeds[i]);
      speeds.add(motorSpeeds[i]);
    }
    
    
    
   
      serializeJson(json, jsonString);
    ws.textAll(jsonString);
    //ws.textAll("Throttle Up - Ramping!");
     Serial.println("Motor Speeds==" +jsonString);


  } else if (command == "throttleDown") {  // Gradual all-down
    Serial.printf(" %d Throttle Down All ==\n"+ DELTA);
    if(motorSpeeds[frontMotors[0]] != motorSpeeds[frontMotors[1]]||
   motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]]){
    motorSpeeds[frontMotors[0]]=motorSpeeds[frontMotors[1]];
     motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]];

     for (int i = 0; i < 2; i++) {
    
    motorSpeeds[frontMotors[i]] -= DELTA;
    motorSpeeds[backMotors[i]]  -= DELTA;
     updateMotor(motors[frontMotors[0]], motorSpeeds[frontMotors[1]]);
    updateMotor(motors[backMotors[0]],  motorSpeeds[backMotors[1]]);
     Serial.printf("[%d] satbilised front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] satbilised back motors = %d\n", i, motorSpeeds[backMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
  }

   }
    for (int i = 0; i < 4; i++) {
      if (motorSpeeds[i]<=1000) {
           for(int i=0;i<4;i++){
            motorSpeeds[i]={1050};
            updateMotor(motors[i], motorSpeeds[i]);
           speeds.add(motorSpeeds[i]);
           Serial.printf("[%d] = %d  ", i, motorSpeeds[i]);    
          }
           serializeJson(json, jsonString);
        ws.textAll(jsonString);
        ws.textAll("Cannot Pitch Down motor min limit reached\n");
         Serial.printf("[%d] = %d  ", i, motorSpeeds[i]);  
          return;
        }
      motorSpeeds[i] -= DELTA;
      updateMotor(motors[i], motorSpeeds[i]);
      speeds.add(motorSpeeds[i]);
      Serial.printf("[%d] speeds throttledown= %d \n ", i, motorSpeeds[i]);  
    }
    Serial.println("Motor Speeds==" +jsonString);
    ws.textAll("Throttle Down - Deceling!");
  serializeJson(json, jsonString);
  ws.textAll(jsonString);
  }
  
  ///Pitching up nose up drone moving backwards
   else if (command == "PitchUp") { 
     // Nose up: Fronts +, Rears s
     int speedError;

    /*We need to make sure the speed of a pair of motors is 
    //same before making changes so we dont have un balanced movements*/
 for (int i = 0; i < 2; i++) {
  // Predict new speeds before applying
 
  if( motorSpeeds[frontMotors[0]] != motorSpeeds[frontMotors[1]]||
   motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]]){
    motorSpeeds[frontMotors[0]] =motorSpeeds[frontMotors[1]];
     motorSpeeds[backMotors[0]]= motorSpeeds[backMotors[1]];
     updateMotor(motors[frontMotors[0]], motorSpeeds[frontMotors[1]]);
    updateMotor(motors[backMotors[0]],  motorSpeeds[backMotors[1]]);
   Serial.printf("[%d] satbilised front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] satbilised back motors = %d\n", i, motorSpeeds[backMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
   }
   if(motorSpeeds[frontMotors[i]]>=2000||motorSpeeds[backMotors[i]]>=2000){
    Serial.println("Speeds cant exceed 2000");
    return;
   }
 }
  // newFrontSpeed = motorSpeeds[frontMotors[i]] + DELTA;
  // newBackSpeed  = motorSpeeds[backMotors[i]]  + DELTA2;
  // Check bounds for any motor in the pair
  /*if (newFrontSpeed < MIN_SPEED || newBackSpeed > MAX_SPEED) {
    ws.textAll("⚠️ Pitch Up Blocked: speed limit reached");
    Serial.println("Pitch Up Blocked: limit reached");
    return;  // Stop the whole function safely
  }*/
 // If safe, apply changes to both pairs
   for (int i = 0; i < 2; i++) {
    
  motorSpeeds[frontMotors[i]] += DELTA+10;
  motorSpeeds[backMotors[i]]  += DELTA;
  updateMotor(motors[frontMotors[i]], motorSpeeds[frontMotors[i]]);
  updateMotor(motors[backMotors[i]],  motorSpeeds[backMotors[i]]);
  speeds.add(motorSpeeds[frontMotors[i]]);
  speeds.add(motorSpeeds[backMotors[i]]);
 Serial.printf("[%d] front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] back motors = %d\n", i, motorSpeeds[backMotors[i]]);
}
serializeJson(json, jsonString);
  ws.textAll(jsonString);
    ws.textAll("Pitch Up - Nose lifting!");
     Serial.println("Motor Speeds==" +jsonString);

  
  } 
  //Pitching down nose down drone moving forward..
  else if (command == "PitchDown") {  // Nose down: Fronts -, Rears +

  /*We need to make sure the speed of a pair of motors is 
    //same before making changes so we dont have un balanced movements*/
   for (int i = 0; i < 2; i++) {
  // Predict new speeds before applying
   if( motorSpeeds[frontMotors[0]] != motorSpeeds[frontMotors[1]]||
   motorSpeeds[backMotors[1]]!= motorSpeeds[backMotors[0]]){
    motorSpeeds[frontMotors[0]] =motorSpeeds[frontMotors[1]];
     motorSpeeds[backMotors[0]]= motorSpeeds[backMotors[1]];
     updateMotor(motors[frontMotors[0]], motorSpeeds[frontMotors[1]]);
    updateMotor(motors[backMotors[0]],  motorSpeeds[backMotors[1]]);
 // speeds.add(motorSpeeds[frontMotors[i]]);
  //speeds.add(motorSpeeds[backMotors[i]]);
   Serial.printf("[%d] satbilised front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] satbilised back motors = %d\n", i, motorSpeeds[backMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
   }
    if(motorSpeeds[frontMotors[i]]>=1000||motorSpeeds[backMotors[i]]>=1000){
    Serial.println("Speeds cant exceed 2000");
    return;
   }
   }

// If safe, apply changes to both pairs
   for (int i = 0; i < 2; i++) {
  motorSpeeds[frontMotors[i]] += DELTA;
  motorSpeeds[backMotors[i]]  += DELTA+10;
  updateMotor(motors[frontMotors[i]], motorSpeeds[frontMotors[i]]);
  updateMotor(motors[backMotors[i]],  motorSpeeds[backMotors[i]]);
  speeds.add(motorSpeeds[frontMotors[i]]);
  speeds.add(motorSpeeds[backMotors[i]]);
  Serial.printf("[%d] front motors = %d\n", i, motorSpeeds[frontMotors[i]]);
 Serial.printf("[%d] back motors = %d\n", i, motorSpeeds[backMotors[i]]);
}
      serializeJson(json, jsonString);
  ws.textAll(jsonString);
    ws.textAll("Pitch DOwn - Nose Dipping!");
     Serial.println("Motor Speeds==" +jsonString);
   
  } 
  
  //Rolling Left
  else if (command == "RollLeft") {  // Bank left: Lefts +, Rights -
     /*We need to make sure the speed of a pair of motors is 
    //same before making changes so we dont have un balanced movements*/
   for (int i = 0; i < 2; i++) {

    if( motorSpeeds[leftMotors[0]] != motorSpeeds[leftMotors[1]]||
   motorSpeeds[rightMotors[1]]!= motorSpeeds[rightMotors[0]]){

    motorSpeeds[leftMotors[0]] =motorSpeeds[leftMotors[1]];
     motorSpeeds[rightMotors[0]]= motorSpeeds[rightMotors[1]];
     updateMotor(motors[leftMotors[0]], motorSpeeds[leftMotors[1]]);
    updateMotor(motors[rightMotors[0]],  motorSpeeds[rightMotors[1]]);
 // speeds.add(motorSpeeds[rightMotors[i]]);
 // speeds.add(motorSpeeds[leftMotors[i]]);
   Serial.printf("[%d] satbilised left motors = %d\n", i, motorSpeeds[leftMotors[i]]);
 Serial.printf("[%d] satbilised back motors = %d\n", i, motorSpeeds[rightMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
   }
   }
   for (int i = 0; i < 2; i++) {
    
  motorSpeeds[leftMotors[i]] +=DELTA;
  motorSpeeds[rightMotors[i]] +=DELTA+10;
  updateMotor(motors[leftMotors[i]], motorSpeeds[leftMotors[i]]);
  updateMotor(motors[rightMotors[i]] ,  motorSpeeds[rightMotors[i]] );
  speeds.add(motorSpeeds[leftMotors[i]]);
  speeds.add(motorSpeeds[rightMotors[i]]);
   Serial.printf("[%d] left motors = %d\n", i, motorSpeeds[leftMotors[i]]);
 Serial.printf("[%d] right motors = %d\n", i, motorSpeeds[rightMotors[i]]);
}
    

    serializeJson(json, jsonString);
    ws.textAll(jsonString);
   Serial.println("Motor Speeds==" +jsonString);
    ws.textAll("Roll Left - Banking left!");


  } 
  //Rolling Right
  else if (command == "RollRight") {  // Bank right: Rights +, Lefts -
  /*We need to make sure the speed of a pair of motors is 
    //same before making changes so we dont have un balanced movements*/
 for (int i = 0; i < 2; i++) {
    if( motorSpeeds[leftMotors[0]] != motorSpeeds[leftMotors[1]]||
   motorSpeeds[rightMotors[1]]!= motorSpeeds[rightMotors[0]]){
    motorSpeeds[leftMotors[0]] =motorSpeeds[leftMotors[1]];
     motorSpeeds[rightMotors[0]]= motorSpeeds[rightMotors[1]];
     updateMotor(motors[leftMotors[0]], motorSpeeds[leftMotors[1]]);
    updateMotor(motors[rightMotors[0]],  motorSpeeds[rightMotors[1]]);
  //speeds.add(motorSpeeds[rightMotors[i]]);
  //speeds.add(motorSpeeds[leftMotors[i]]);
   Serial.printf("[%d] satbilised left motors = %d\n", i, motorSpeeds[leftMotors[i]]);
 Serial.printf("[%d] satbilised right motors = %d\n", i, motorSpeeds[rightMotors[i]]);
     Serial.println(" Motors were not eauqal hence stabilized");
   }
 }
   for (int i = 0; i < 2; i++) {
  motorSpeeds[leftMotors[i]] +=DELTA+10;
  motorSpeeds[rightMotors[i]] +=DELTA;;
  updateMotor(motors[leftMotors[i]], motorSpeeds[leftMotors[i]]);
  updateMotor(motors[rightMotors[i]] ,  motorSpeeds[rightMotors[i]] );
  speeds.add(motorSpeeds[leftMotors[i]]);
  speeds.add(motorSpeeds[rightMotors[i]]);
   Serial.printf("[%d] left motors = %d\n", i, motorSpeeds[leftMotors[i]]);
 Serial.printf("[%d] right motors = %d\n", i, motorSpeeds[rightMotors[i]]);
}
      
    serializeJson(json, jsonString);
    ws.textAll(jsonString);
   Serial.println("Motor Speeds==" +jsonString);
    ws.textAll("Rolling Right - Banking Right!");
  } else if (command == "YawLeft") {  // CCW turn: Lefts +, Rights - (torque)
    Serial.println("Yaw Left: Lefts +delta, Rights -delta");
    motorSpeeds[0] += DELTA;  // FL +
    motorSpeeds[2] += DELTA;  // RL +
    motorSpeeds[1] -= DELTA;  // FR -
    motorSpeeds[3] -= DELTA;  // RR -
    for (int i = 0; i < 4; i++) {
      if (motorSpeeds[0] >= MAX_SPEED || motorSpeeds[1] >= MAX_SPEED ||
        motorSpeeds[2] >= MAX_SPEED || motorSpeeds[3] >= MAX_SPEED) {
        ws.textAll("Cannot Yaw Left: motor  limit reached");
          return;
        }
    updateMotor(motors[i], motorSpeeds[i]);
    speeds.add(motorSpeeds[i]);
    }
    serializeJson(json, jsonString);
  ws.textAll(jsonString);
   Serial.println("Motor Speeds==" +jsonString);
    ws.textAll("Yaw Left - Turning CCW!");
  } else if (command == "YawRight") {  // CW turn: Rights +, Lefts - (torque)
    Serial.println("Yaw Right: Rights +delta, Lefts -delta");
    motorSpeeds[0] -= DELTA;  // FL -
    motorSpeeds[2] -= DELTA;  // RL -
    motorSpeeds[1] += DELTA;  // FR +
    motorSpeeds[3] += DELTA;  // RR +
    for (int i = 0; i < 4; i++){
      if (motorSpeeds[0] <= MIN_SPEED || motorSpeeds[1] <= MIN_SPEED ||
        motorSpeeds[2] <= MIN_SPEED || motorSpeeds[3] <= MIN_SPEED) {
        ws.textAll("Cannot  Yaw right: motor min limit reached");
          return;
        }
     updateMotor(motors[i], motorSpeeds[i]);
     speeds.add(motorSpeeds[i]);
     }
     serializeJson(json, jsonString);
  ws.textAll(jsonString);
   Serial.println("Motor Speeds==" +jsonString);
    ws.textAll("Yaw Right - Turning CW!");
  } else if (command == "ON") {  // Legacy quick-start
    Serial.println("Quick ON: Jump to mid-throttle");
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = 1500;
      updateMotor(motors[i], motorSpeeds[i]);
       speeds.add(motorSpeeds[i]);
    }
    digitalWrite(ledPin, HIGH);
    serializeJson(json, jsonString);
  ws.textAll(jsonString);
    ws.textAll("ON - Mid-throttle engaged");
  } else if (command == "OFF") {  // Full stop
    Serial.println("OFF: Disarm all");
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] = MIN_SPEED;
      updateMotor(motors[i], motorSpeeds[i]);
    }
    digitalWrite(ledPin, LOW);
    ws.textAll("OFF - Disarmed");
  } else {
    Serial.printf("Unknown command: %s\n", command.c_str());
    ws.textAll("Unknown: " + command);
  }
  
  command = "";  // Reset
}
///
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            Serial.printf("💡 Client %u connected\n", client->id());
            client->text("Connected to ESP32 WebSocket");
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("❌ Client %u disconnected\n", client->id());
            break;
        case WS_EVT_DATA:
            handleWebSocketMessage(arg, data, len);
            break;
        default: break;
    }
}


void setup(){
  Serial.begin(115200);
  delay(2000);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }
   Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
 pinMode(ledPin, OUTPUT);
  // Start WebSocket server
  ws.onEvent(onEvent);
  server.begin();
  
  server.addHandler(&ws);
  Serial.println("WebSocket server started on port 81");
  Serial.println("WiFi connected");
  Serial.println("Arming ESCs...now");
   
  for (int i = 0; i < 4; i++) {
    motors[i].attach(motorPins[i], 1000, 2000); // min and max pulse
    motors[i].writeMicroseconds(1000);          // send minimum to arm
    delay(2000);                                 // wait for ESC to arm
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.println(" armed.");
  }

  Serial.println("All ESCs armed!");
}
void loop() {
   ///getting commands
  //function that contolls motors;


}
/*
void setup() {

  pinMode( LED_BUILTIN, OUTPUT);

}

void loop() {
  Serial.begin(115200);
  Serial.println("Arming ESCs...");
  digitalWrite(LED_BUILTIN, HIGH); // Turn LED on

  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);  // Turn LED off
  delay(5000);
}*/
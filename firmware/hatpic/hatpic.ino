/*
  Project: hatpic device
  Description: This sketch handles the interface of the smart servo with
  the computer throug serial communication.

  Authors: Julien Mellet, and Simon Le Berre
  Date: May 26, 2024

  Additional Notes: This is a basic example to demonstrate digital output.
*/

#include <SCServo.h> 
SMS_STS motor; 
#define RXD2 16 
#define TXD2 17 
 
int load_M1; 
int pos_M1 = 1000;
int pos_M2 = 1000;
int pos_M3 = 1000;
int pos_M4 = 1000;
int vel_M1;
int tmp;

// Torque control parameters
const float Kp = -1500.0; // Proportional gain
const float Ki = 20.0; // Integral gain
const float Ke = 0.50; // Elastic coefficient

// Control loop variables
float torqueSetpoint = 0.0; // Set your desired torque value
float torqueFeedback = 0.0;
float integralTerm = 0.0;
float integralLimit = 2000.0;  // Adjust as needed

// Base motor speed (adjust as needed)
int baseSpeed = 100; // Set your base speed value
int interval = 200;

unsigned long loopEndTime = millis();
unsigned long loopStartTime = millis();

char serial_data;
String serial_trame;
unsigned long data_a, data_b, data_c, data_d;

void setup() {
  Serial.begin(115200);
  //Serial.setTimeout(1);
  Serial.println(" ");
  Serial.println("STS3032 Torque control");
 
  Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);
  motor.pSerial = &Serial2;
  delay(1000);
 
  Serial.print("Current pos motor : ");
  Serial.println(motor.ReadPos(1));
 
  pos_M1 = motor.ReadPos(1);
  Serial.print("Current pos motor : ");
  Serial.println(pos_M1);
 
  //Wheel mode for ID1
  //motor.WriteMode(1, 1);
  //motor.WheelMode(1);
  //motor.WriteTorqueLimit(1, 1);
  //motor.EnableTorque(1, 0);  // 0 to disconnect the motor     1 to activate torque 
  
  Serial.println(motor.FeedBack(1));
 
  delay(100);
}

void loop() {
  if (Serial.available()) {
    serial_data = Serial.read();
    if (String(serial_data) == "i"){
      serial_trame = "i";
      while (String(serial_data)!= "o") {
        serial_data = Serial.read();
        serial_trame = serial_trame + String(serial_data);
      }
      check_trame(serial_trame);
    }
    else{Serial.flush();}
  }
  // Record the start time of the control loop in microseconds
  loopStartTime = millis();

  // Read torque feedback from the sensor
  torqueFeedback = motor.ReadLoad(1);

  // Calculate torque error
  float torqueError = torqueSetpoint - torqueFeedback;

  // Calculate integral term
  integralTerm += Ki * torqueError;

  // Anti-windup - limit the integral term to prevent excessive accumulation
  integralTerm = constrain(integralTerm, -integralLimit, integralLimit);

  // Calculate spring effect
  torqueSetpoint = Ke * (motor.ReadPos(1)-1000);

  // Calculate speed control output using PI control
  float speedControlOutput = Kp * torqueError + Ki * integralTerm;
  if (speedControlOutput > 20000){
    speedControlOutput = 20000;
  }
  if (speedControlOutput < -20000){
    speedControlOutput = -20000;
  }

  // Update motor speed
  int motorSpeed = motor.ReadSpeed(1);   
  motor.WriteSpe(1, speedControlOutput, 240);

  if (loopStartTime - loopEndTime > interval) { 
    loopEndTime = millis();
    tmp = motor.ReadPos(1);
    if (tmp != -1) {
      pos_M1 = tmp;
    }
    send_cmd();
  }
}

void check_trame(String trame){
  if (trame.substring(0,2) == "ia"){trame2data(trame);}
  else {Serial.println("error");}
}

void trame2data(String trame){
  int aIndex = trame.indexOf('a');
  int bIndex = trame.indexOf('b');
  int cIndex = trame.indexOf('c');
  int dIndex = trame.indexOf('d');
  int oIndex = trame.indexOf('o');

  if((aIndex != -1) and (bIndex != -1) and (cIndex != -1) and (dIndex != -1) and (oIndex != -1)) {
    data_a = trame.substring(aIndex + 1, bIndex).toInt();
    data_b = trame.substring(bIndex + 1, cIndex).toInt();
    data_c = trame.substring(cIndex + 1, dIndex).toInt();
    data_d = trame.substring(dIndex + 1, oIndex).toInt();
  }
}

void send_cmd(){
  Serial.print("ia");
  Serial.print(String(pos_M1));
  Serial.print("b");
  Serial.print(String(pos_M2));
  Serial.print("c");
  Serial.print(String(pos_M3));
  Serial.print("d");
  Serial.print(String(pos_M4));
  Serial.println("o");
}
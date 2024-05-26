/*
  Project: hatpic device
  Description: This sketch handles the interface of the smart servo with
  the computer throug serial communication.

  Created by: Julien Mellet, and Simon Le Berre
  Date: May 26, 2024

  Additional Notes: This is a basic example to demonstrate digital output.
*/

char serial_data;
String serial_trame;
unsigned long data_a, data_b, data_c, data_d;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
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
}

void check_trame(String trame){
  
  if (trame.substring(0,2) == "ia"){ set_led_color(trame);}

  else {Serial.println("error");}
}

void set_led_color(String trame){
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
  Serial.print("ia");
  Serial.print(String(data_a));
  Serial.print("b");
  Serial.print(String(data_b));
  Serial.print("c");
  Serial.print(String(data_c));
  Serial.print("d");
  Serial.print(String(data_d));
  Serial.println("o");
}
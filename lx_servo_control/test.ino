#include "lx_servo_control.hpp"

/****************************/
/* TESTING/DEBUGING SECTION */
/****************************/
void test_move_time_write(){
  lx_servo_serial_set_move_time(Serial, ID00, 0, 0);
  delay(1000);
  lx_servo_serial_set_move_time(Serial, ID00, 500, 1000);
  delay(1500);
  lx_servo_serial_set_move_time(Serial, ID00, 1000, 0);
  delay(1000);
}

void test_move_time_read(){
  int16_t* pos;
  int16_t* time;
  delay(1000);
  lx_servo_serial_set_move_time(Serial, ID00, 0, 0);
  lx_servo_serial_get_move_time(Serial, ID00, pos, time);
  Serial.print("Pos:");
  Serial.println(*pos);
  delay(1000);
  lx_servo_serial_set_move_time(Serial, ID00, 1000, 0);
  lx_servo_serial_get_move_time(Serial, ID00, pos, time);
  Serial.print("Pos:");
  Serial.println(*pos);
}

void test_get_id(){
  lx_servo_serial_get_id(Serial);
  delay(1000);
  lx_servo_serial_set_move_time(Serial, ID00, 0, 0);
  delay(2000);
  lx_servo_serial_set_move_time(Serial, ID00, 1000, 0);
  delay(2000);
}

void test_read_temp(){
  int temp;
  int pos;
  temp = lx_servo_serial_get_temp(Serial, ID00);
  Serial.print("Temp: ");
  Serial.println(temp);
  delay(1000);
  lx_servo_serial_set_move_time(Serial, ID00, 0, 0);
  for (uint8_t i = 0; i < 20; i++){
    pos = lx_servo_serial_get_pos(Serial, ID00);
    Serial.print("Pos: ");
    Serial.println(pos);
    delay(10);
  }
  delay(1000);
  pos = lx_servo_serial_get_pos(Serial, ID00);
  Serial.print("Pos: ");
  Serial.println(pos);
  delay(10);
  lx_servo_serial_set_move_time(Serial, ID00, 1000, 0);
  for (uint8_t i = 0; i < 20; i++){
    pos = lx_servo_serial_get_pos(Serial, ID00);
    Serial.print("Pos: ");
    Serial.println(pos);
    delay(10);
  }
  delay(1000);
  pos = lx_servo_serial_get_pos(Serial, ID00);
  Serial.print("Pos: ");
  Serial.println(pos);
  delay(10);
}

// put your setup code here, to run once:
// void setup(){
//   Serial.begin(115200);
//   delay(1000);
// }

// put your main code here, to run repeatedly:
// void loop(){
//   test_read_temp();
// }

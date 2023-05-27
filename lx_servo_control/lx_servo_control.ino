#include "lx_servo_control.h"

void debug_buf(byte buf[], const char* text){
#ifdef LX_SERVO_DEBUG
  Serial.println(text);
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++){
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println("");
#endif
}

byte lx_servo_check_sum(byte buf []){
  byte i;
  uint16_t temp = 0;
  for (i=2; i < buf[3]+2; i++){
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

int lx_servo_serial_recieve_handle(HardwareSerial &SerialX, byte* ret){
  bool frameStarted = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i = 0;

  while (SerialX.available()){
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if(!frameStarted){
      if(rxBuf == LX_SERVO_FRAME_HEADER){
        recvBuf[dataCount] = (uint8_t)rxBuf;
        frameCount++;
        dataCount++;
        if (frameCount == 2){
          frameCount = 0;
          frameStarted = true;
        }
      }else{
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }else{
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3){
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7){
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3){

#ifdef LX_SERVO_DEBUG
        Serial.print("RECEIVED DATA:");
        for(i = 0; i < dataCount; i++){
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif
        if (lx_servo_check_sum(recvBuf) == recvBuf[dataCount-1]){
#ifdef LX_SERVO_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println(" ");
#endif
          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}
// TODO: TEST
void lx_servo_serial_set_move_time(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time){
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LX_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO MOVE");
  SerialX.write(buf, 10);
}
// TODO: TEST
int lx_servo_serial_get_move_time(HardwareSerial &SerialX, uint8_t id, int16_t* position, uint16_t* time){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_MOVE_TIME_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ MOVE_TIME");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0){
    *position = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    *time = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    ret = 0;
  }else{
    ret = -1;
  }

#ifdef LX_SERVO_DEBUG
  Serial.print("Position: ");
  Serial.println(*position);
  Serial.println("time: ");
  Serial.println(*time);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_wait_move_time(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time){
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LX_SERVO_MOVE_TIME_WAIT_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO MOVE");
  SerialX.write(buf, 10);
}
 // TODO: TEST
int lx_servo_serial_get_wait_move_time(HardwareSerial &SerialX, uint8_t id, int16_t* position, uint16_t* time){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_MOVE_TIME_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ WAIT_OVE_TIME");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0){
    *position = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    *time = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    ret = 0;
  }else{
    ret = -1;
  }
#ifdef LX_SERVO_DEBUG
  Serial.print("Position: ");
  Serial.println(*position);
  Serial.println("time: ");
  Serial.println(*time);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_move_start(HardwareSerial &SerialX, uint8_t id){
  byte buf[6];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_MOVE_START;
  buf[5] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO START MOVE");
  SerialX.write(buf, 6);
}
// TODO: TEST
void lx_servo_serial_stop_move(HardwareSerial &SerialX, uint8_t id){
  byte buf[6];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_MOVE_STOP;
  buf[5] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO STOP MOVE");
  SerialX.write(buf, 6);
}
// TODO: TEST
void lx_servo_serial_set_id(HardwareSerial &SerialX, uint8_t oldId, uint8_t newId){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = oldId;
  buf[3] = 4;
  buf[4] = LX_SERVO_ID_WRITE;
  buf[5] = newId;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO SET ID");
  SerialX.write(buf, 7);
}
// TODO: TEST
int lx_servo_serial_get_id(HardwareSerial &SerialX){
  int count = 10000;
  int ret;
  byte buf[6];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = 254;
  buf[3] = 3;
  buf[4] = LX_SERVO_ID_READ;
  buf[5] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO READ ID");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(0, buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.print("SERIVO ID: ");
  Serial.println(ret);
#endif
  return ret;
} 
// TODO: TEST
void lx_servo_serial_angle_offset_adjuist(HardwareSerial &SerialX, uint8_t id, uint8_t adjust){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LX_SERVO_ANGLE_OFFSET_ADJUST;
  buf[5] = adjust;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO ADJUST ANGLE OFFSET");
  SerialX.write(buf, 7);
}
// TODO: TEST
void lx_servo_serial_set_angle_offset(HardwareSerial &SerialX, uint8_t id){
  byte buf[6];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_ANGLE_OFFSET_WRITE;
  buf[5] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO WRITE ANGLE OFFSET");
  SerialX.write(buf, 6);
}
// TODO: TEST
int lx_servo_serial_get_angle_offset(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_ANGLE_OFFSET_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ OFFSET ANGLE");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = (int)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_ang_limit(HardwareSerial &SerialX, uint8_t id, uint16_t min, uint16_t max){
  byte buf[10];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LX_SERVO_ANGLE_LIMIT_WRITE;
  buf[5] = GET_LOW_BYTE(min);
  buf[6] = GET_HIGH_BYTE(min);
  buf[7] = GET_LOW_BYTE(max);
  buf[8] = GET_HIGH_BYTE(max);
  buf[9] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO SET ANGLE LIMIT");
  SerialX.write(buf, 10);
}
// TODO: TEST
int lx_servo_serial_get_ang_limit(HardwareSerial &SerialX, uint8_t id, uint16_t* min, uint16_t* max){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_ANGLE_LIMIT_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ ANGLE LIMIT");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0){
    *min = BYTE_TO_HW(buf[2], buf[1]);
    *max = BYTE_TO_HW(buf[2], buf[1]);
    ret = 0;
  }else{
    ret = -1;
  }
#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_vin_limit(HardwareSerial &SerialX, uint8_t id, uint16_t min_v, uint16_t max_v){
  byte buf[10];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LX_SERVO_VIN_LIMIT_WRITE;
  buf[5] = GET_LOW_BYTE(min_v);
  buf[6] = GET_HIGH_BYTE(min_v);
  buf[7] = GET_LOW_BYTE(max_v);
  buf[8] = GET_HIGH_BYTE(max_v);
  buf[9] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO MOVE");
  SerialX.write(buf, 10);
}
// TODO: TEST
int lx_servo_serial_get_vin_lim(HardwareSerial &SerialX, uint8_t id, uint16_t* min_v, uint16_t* max_v){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_VIN_LIMIT_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ VIN LIM");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0){
    *min_v = BYTE_TO_HW(buf[2], buf[1]);
    *max_v = BYTE_TO_HW(buf[2], buf[1]);
    ret = 0;
  }else{
    ret = -1;
  }
#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO TEST
void lx_servo_serial_set_temp_max(HardwareSerial &SerialX, uint8_t id, uint8_t temp){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LX_SERVO_TEMP_MAX_LIMIT_WRITE;
  buf[5] = temp;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO SET ID");
  SerialX.write(buf, 7);
}
// TODO: TEST
int lx_servo_serial_get_temp_max(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_TEMP_MAX_LIMIT_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ TEMP MAX");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
int lx_servo_serial_get_temp(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_TEMP_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ TMP");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
int lx_servo_serial_get_vin(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_VIN_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ VIN");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
int lx_servo_serial_get_pos(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_POS_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ_POS");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_mode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t speed){
  byte buf[10];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LX_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)speed);
  buf[9] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO SET MODE");
  SerialX.write(buf, 10);
}
// TODO: TEST
int lx_servo_serial_get_mode(HardwareSerial &SerialX, uint8_t id, uint8_t* mode, uint8_t* speed){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_OR_MOTOR_MODE_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ MODE");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0){
    *mode = BYTE_TO_HW(buf[2], buf[1]);
    *speed = BYTE_TO_HW(buf[4], buf[3]);
    ret = 0;
  }else{
    ret = -1;
  }

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_load_or_unload(HardwareSerial &SerialX, uint8_t id, uint8_t mode){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LX_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = mode;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO UNLOAD");
  SerialX.write(buf, 7);
}
// TODO: TEST
int lx_servo_serial_get_load_or_unload(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_LOAD_OR_UNLOAD_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ LOAD");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_led_ctrl(HardwareSerial &SerialX, uint8_t id, uint8_t mode){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LX_SERVO_LED_CTRL_WRITE;
  buf[5] = mode;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO WRITE CONTROL LED");
  SerialX.write(buf, 7);
}
// TODO: TEST
int lx_servo_serial_get_led_ctrl(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_LED_CTRL_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ CONTROL LED");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
// TODO: TEST
void lx_servo_serial_set_led_err(HardwareSerial &SerialX, uint8_t id, uint8_t mode){
  byte buf[7];
  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LX_SERVO_LED_ERROR_WRITE;
  buf[5] = mode;
  buf[6] = lx_servo_check_sum(buf);
  debug_buf(buf, "LX_SERVO WRITE ERROR LED");
  SerialX.write(buf, 7);
}
// TODO: TEST
int lx_servo_serial_get_led_err(HardwareSerial &SerialX, uint8_t id){
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LX_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LX_SERVO_LED_ERROR_READ;
  buf[5] = lx_servo_check_sum(buf);

  debug_buf(buf, "LX_SERVO READ ERROR LED");

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while(!SerialX.available()){
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (lx_servo_serial_recieve_handle(SerialX, buf) > 0)
    ret = BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -1;

#ifdef LX_SERVO_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

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


void setup(){
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop(){
  // put your main code here, to run repeatedly:
  test_move_time_write();
}

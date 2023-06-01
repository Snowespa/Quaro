#ifndef __LX_SERVO_CONTROL_H__
#define __LX_SERVO_CONTROL_H__

#define GET_LOW_BYTE(A) (uint8_t)(A) //Macro function get lower bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8) //Macro function get higher 8 bites of A
//Put A as higher 8 bits, A as lower 8 bits, returns 
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) 


#define LX_SERVO_FRAME_HEADER         0x55
#define LX_SERVO_MOVE_TIME_WRITE      1
#define LX_SERVO_MOVE_TIME_READ       2
#define LX_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LX_SERVO_MOVE_TIME_WAIT_READ  8
#define LX_SERVO_MOVE_START           11
#define LX_SERVO_MOVE_STOP            12
#define LX_SERVO_ID_WRITE             13
#define LX_SERVO_ID_READ              14
#define LX_SERVO_ANGLE_OFFSET_ADJUST  17
#define LX_SERVO_ANGLE_OFFSET_WRITE   18
#define LX_SERVO_ANGLE_OFFSET_READ    19
#define LX_SERVO_ANGLE_LIMIT_WRITE    20
#define LX_SERVO_ANGLE_LIMIT_READ     21
#define LX_SERVO_VIN_LIMIT_WRITE      22
#define LX_SERVO_VIN_LIMIT_READ       23
#define LX_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LX_SERVO_TEMP_MAX_LIMIT_READ  25
#define LX_SERVO_TEMP_READ            26
#define LX_SERVO_VIN_READ             27
#define LX_SERVO_POS_READ             28
#define LX_SERVO_OR_MOTOR_MODE_WRITE  29
#define LX_SERVO_OR_MOTOR_MODE_READ   30
#define LX_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LX_SERVO_LOAD_OR_UNLOAD_READ  32
#define LX_SERVO_LED_CTRL_WRITE       33
#define LX_SERVO_LED_CTRL_READ        34
#define LX_SERVO_LED_ERROR_WRITE      35
#define LX_SERVO_LED_ERROR_READ       36

//#define LX_SERVO_DEBUG 0 /*Debug: print debug value*/

/* The 12 servos that will be used on quaro */
#define ID00 0
#define ID01 1
#define ID02 2
#define ID03 3
#define ID04 4
#define ID05 5
#define ID06 6
#define ID07 7
#define ID08 8
#define ID09 9
#define ID10 10
#define ID11 11
#define IDX 254

#define BAUD 115200

int poses_vec[3];

/*
 * Computes the checksum for 
 * the command sent to the servos
 * input:
 * ------
 *  - byte[]: the byte array to use
 *      to compute the checksum.
 * output:
 * -------
 *  - byte containing the checksum
 */
byte lx_servo_check_sum(byte buf []);

/*
 * Recieves message from the serial port
 * 
 * input:
 * ------
 *  - HardwareSerial, the device for the serial communication
 *  - byte* return array.
 * 
 * output:
 * -------
 *  - Array containing the return value.
 */
int lx_servo_serial_recieve_handle(HardwareSerial &SerialX, byte* ret);

/*
 * Rotates a servo contected to the serial 
 * port with the given id to a given angle.
 * Tries to perform the rotation within the
 * time value. Command executed immidiatly
 * upon reception.
 * 
 * input:
 * ------
 *  - HardwareSerial: the serial device 
 *      to use for communication.
 *  - uint8_t id: the id servo to control.
 *  - uint16_t position: the angle position to set
 *      between [0-1000] corresponding to [0-240] degrees.
 *  - uint16_t time: time range between [0-30000]ms. 
 */
void lx_servo_serial_set_move_time(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);

/*
 * Gets the angle and time value sent by command move time.
 * 
 * input:
 * ------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - int16_t* position, pointer to the angle input varialbe
 *  - uint16_t* time. pointer to the time output varialbe
 */
int lx_servo_serial_get_move_time(HardwareSerial &SerialX, uint8_t id, int16_t* position, uint16_t* time);

// TODO: implement
/*
 * Similar to set_move_time but isn't immitadly executed. The servo will
 * rotate from its current angle to the desired angle at uniform speed
 * within time set by parameter time upon reception of 
 * SERVO_MOVE_START commang
 * 
 * input:
 * ------
 *  - HardwareSerial, the device used for the serial communication.
 *  - uint8_t id: the servo id.
 *  - uint16_t angle: the servo angle [0-1000] -> [0-240] deg.
 *  - uint16_t time: the preset time [0-30.000]ms
 */
void lx_servo_serial_set_wait_move_time(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);

// TODO: implement
/*
 * Gets the angle and time value sent by command move time.
 * 
 * input:
 * ------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - int16_t* position, pointer to the angle input varialbe
 *  - uint16_t* time. pointer to the time output varialbe
 */
int lx_servo_serial_get_wait_move_time(HardwareSerial &SerialX, uint8_t id, int16_t* position, uint16_t* time);

// TODO: implement
/*
 * Starts the execution of a set_wait_move_time command.
 * 
 * input:
 * ------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 */
void lx_servo_serial_move_start(HardwareSerial &SerialX, uint8_t id);

/*
 * Stops the servo's execution immediately.
 * 
 * input:
 * ------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 */
void lx_servo_serial_move_stop(HardwareSerial &SerialX, uint8_t id);

/*
 * Set the servo id to a new id.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t oldId: the old servo id.
 *  - uint8_t newId: the new servo id.
 */
void lx_servo_serial_set_id(HardwareSerial &SerialX, uint8_t oldId, uint8_t newId);

/*
 * Get the servo id that is connected on the bus.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 */
int lx_servo_serial_get_id(HardwareSerial &SerialX);

// TODO: implement
/*
 * Sets a deviation for the servo (without saving it). When the command
 * is sent, the servo will automatically rotate to the adjusted deviation.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - signed char dev, the deviation value [-125, 125] -> [-30, 30] deg.
 */
void lx_servo_serial_angle_offset_adjuist(HardwareSerial &SerialX);

// TODO: implement
/*
 * Sets the deviation value for the servo and saves it set with the
 * adjust method.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 */
void lx_servo_serial_set_angle_offset(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Reads the deviation value set for the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 * 
 * output:
 * -------
 *  - int, the offset value assigned to the servo.
 */
int lx_servo_serial_get_angle_offset(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Sets the limits of the angle range. Min should always be smaller than max
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint16_t min: the minimal value of the angle [0, 1000] -> [0, 240]deg
 *  - uint16_t max: the max value of the angle [0, 1000] -> [0, 240]deg
 */
void lx_servo_serial_set_ang_limit(HardwareSerial &SerialX, uint8_t id, uint16_t min, uint16_t max);

// TODO: implement
/*
 * Read the limit angle range.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint16_t* min: the minimal value of the angle [0, 1000] -> [0, 240]deg
 *  - uint16_t* max: the max value of the angle [0, 1000] -> [0, 240]deg
 * 
 * outputs:
 * --------
 *  - 0 on sucess, -1 on failure
 */
int lx_servo_serial_get_ang_limit(HardwareSerial &SerialX, uint8_t id, uint16_t* min, uint16_t* max);

// TODO: implement
/*
 * Sets the min and max voltage accepted by the servo. If the actual voltage is not within the range, 
 * the flash and alarm will activate.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint16_t min_v: the min voltage value [4500, 12000] mV
 *  - uint16_t max_v: the max voltage value [4500, 12000] mV
 */
void lx_servo_serial_set_vin_limit(HardwareSerial &SerialX, uint8_t id, uint16_t min_v, uint16_t max_v);

// TODO: implement
/*
 * Reads the min and max voltage accepted by the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint16_t* min_v: the min voltage value [4500, 12000] mV
 *  - uint16_t* max_v: the max voltage value [4500, 12000] mV
 *  
 * outputs:
 * --------
 *  - 0 on sucess, -1 on failure
 */
int lx_servo_serial_get_vin_lim(HardwareSerial &SerialX, uint8_t id, uint16_t* min_v, uint16_t* max_v);

// TODO: implement
/*
 * Sets the temperature limit of the servo. If temp is reached the error led will flahs
 * and the alarm will ring. The motor will also go into unload mode until the temperature goes
 * below the limit.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t temp: the max temperature value [50-100]deg Celsius
 */
void lx_servo_serial_set_temp_max(HardwareSerial &SerialX, uint8_t id, uint8_t temp);

// TODO: implement
/*
 * Reads the temperature limit of the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *
 * output:
 * -------
 *  - int temp. The max temperature
 */
int lx_servo_serial_get_temp_max(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Reads the temperature of the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *
 * output:
 * -------
 *  - int temp. The servo temperature
 */
int lx_servo_serial_get_temp(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Reads the voltage value of the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *
 * output:
 * -------
 *  - int V. The current voltage
 */
int lx_servo_serial_get_vin(HardwareSerial &SerialX, uint8_t id);

/*
 * Reads the position of the servo.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *
 * output:
 * -------
 *  - int pos. The current position of the srevo
 */
int lx_servo_serial_get_pos(HardwareSerial &SerialX, uint8_t id);

/*
 * Selects the actuator mode. Does not support power down save.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t mode: the mode of the servo 0 for servo mode 1 for motor mode.
 *  - int16_t rot_speed: [-1000; 1000] only valid in motor mode.
 */
void lx_servo_serial_set_mode(HardwareSerial &SerialX, uint8_t id, uint8_t mode, uint16_t speed);

// TODO: implement
/*
 * Reads the actuator mode.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t mode: the mode of the servo 0 for servo mode 1 for motor mode.
 *  - int16_t rot_speed: [-1000; 1000] 
 */
int lx_servo_serial_get_mode(HardwareSerial &SerialX, uint8_t id);

/*
 * Sets Load or unload servo mode. In unload mode, the servo isn't 
 * generating any torque.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t mode: the mode of the servo 0 unload mode (no torque), 1 for loaded mode (active torque).
 *      default mode is 0
 */
void lx_servo_serial_set_load_or_unload(HardwareSerial &SerialX, uint8_t id, uint8_t mode);

// TODO: implement
/*
 * Reads the load Load or unload servo mode.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 * 
 * outputs:
 * --------
 *  - int the current mode.
 */
int lx_servo_serial_get_load_or_unload(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Sets the current value of the control led. Support power-down save.
 * 
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t mode: 0 off, 1 on.
 */
void lx_servo_serial_set_led_ctrl(HardwareSerial &SerialX, uint8_t id, uint8_t mode);

// TODO: implement
/*
 * Reads the current value of the control led.
 *
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 * 
 * outputs:
 * --------
 *  - int mode: the current control led mode.
 */
int lx_servo_serial_get_led_ctrl(HardwareSerial &SerialX, uint8_t id);

// TODO: implement
/*
 * Sets the value for the error led
 *
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 *  - uint8_t mode: [0-7]
 *      - 0 no alarm
 *      - 1 Over temperature.
 *      - 2 Over voltage.
 *      - 3 Over temperature and voltage.
 *      - 4 Locked-rotor.
 *      - 5 Over temperature and stalled.
 *      - 6 Over voltage and stalled.
 *      - 7 Over temp-volt and stalleed
 * 
 * outputs:
 * --------
 */
void lx_servo_serial_set_led_err(HardwareSerial &SerialX, uint8_t id, uint8_t mode);

// TODO: implement
/*
 * Reads the value of the error led.
 *
 * inputs:
 * -------
 *  - HardwareSerial, the device for the serial communication
 *  - uint8_t id: the servo id.
 * 
 * outputs:
 * --------
 *  - int mode: the current error led mode
 */
int lx_servo_serial_get_led_err(HardwareSerial &SerialX, uint8_t id);

#endif

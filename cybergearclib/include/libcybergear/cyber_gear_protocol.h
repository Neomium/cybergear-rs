//
// Created by Eric Wu on 2023/9/4.
//

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "visibility.h"

typedef enum {
/// Get device ID (communication type 0); Get device ID and 64-bit MCU unique identifier
COMMUNICATION_FETCH_DEVICE_ID = 0,
/// Operation mode motor control command (communication type 1) is used to send control commands to the motor
COMMUNICATION_MOTION_CONTROL_COMMAND = 1,
/// Motor feedback data (communication type 2) is used to feedback the motor operation status to the host
COMMUNICATION_STATUS_REPORT = 2,
/// Motor enable operation (communication type 3)
COMMUNICATION_ENABLE_DEVICE = 3,
/// Motor stop operation (communication type 4)
COMMUNICATION_DISABLE_DEVICE = 4,
/// Setting the motor mechanical zero position (communication type 6) will set the current motor position to the mechanical zero position (power-off loss)
COMMUNICATION_SET_MECHANICAL_ZERO_POSITION = 6,
/// Set the motor CAN_ID (communication type 7) to change the current motor CAN_ID, effective immediately.
COMMUNICATION_SET_CAN_ID = 7,
/// Request Logging data.
COMMUNICATION_LOGGING = 10,
/// Single parameter read (communication type 17)
COMMUNICATION_READ_SINGLE_PARAM = 17,
/// Single parameter write (communication type 18) (power off loss)
COMMUNICATION_WRITE_SINGLE_PARAM = 18,
/// Fault feedback frame (communication type 21)
COMMUNICATION_ERROR_REPORT = 21
} cyber_gear_can_communication_type_t;

typedef enum {
/// Name Motor name
/// Parameter type String
CONFIG_WR_NAME = 0x0000,
/// BarCode
/// Parameter type String
CONFIG_R_BAR_CODE = 0x0001,
/// BootCodeVersion
/// Parameter type String
CONFIG_R_BOOT_CODE_VERSION = 0x1000,
/// BootBuildDate
/// Parameter type String
CONFIG_R_BOOT_BUILD_DATE = 0x1001,
/// BootBuildTime
/// Parameter type String
CONFIG_R_BOOT_BUILD_TIME = 0x1002,
/// AppCodeVersion
/// Parameter type String
CONFIG_R_APP_CODE_VERSION = 0x1003,
/// AppGitVersion
/// Parameter type String
CONFIG_R_APP_GIT_VERSION = 0x1004,
/// AppBuildDate
/// Parameter type String
CONFIG_R_APP_BUILD_DATE = 0x1005,
/// AppBuildTime
/// Parameter type
CONFIG_R_APP_BUILD_TIME = 0x1006,
/// AppCodeName
/// Parameter type
CONFIG_R_APP_CODE_NAME = 0x1007,
/// echoPara1
/// Parameter type
CONFIG_R_ECHO_PARA1 = 0x2000,
/// echoPara2
/// Parameter type
CONFIG_R_ECHO_PARA2 = 0x2001,
/// echoPara3
/// Parameter type
CONFIG_R_ECHO_PARA3 = 0x2002,
/// echoPara4
/// Parameter type
CONFIG_R_ECHO_PARA4 = 0x2003,
/// echoFreHz
/// Parameter type
CONFIG_WR_ECHO_FRE_HZ = 0x2004,
/// MechOffset Motor encoder angle offset
/// Parameter type float [-7, 7]
CONFIG_R_MECH_OFFSET = 0x2005,
/// MechPos_init Reference angle for initial multi-turn
/// Parameter type float [-50, 50]
CONFIG_WR_MECH_POS_INIT = 0x2006,
/// limit_torque torque limit
/// Parameter type float [0, 12]
CONFIG_WR_LIMIT_TORQUE = 0x2007,
/// I_FW_MAX weak magnetic current value, default 0
/// Parameter type float [0, 33]
CONFIG_WR_I_FW_MAX = 0x2008,
/// motor_index motor index, marking the motor joint position
/// Parameter type uint8_t [0, 20]
CONFIG_WR_MOTOR_INDEX = 0x2009,
/// CAN_ID local node id
/// Parameter type uint8_t [0, 127]
CONFIG_WR_CAN_ID = 0x200a,
/// CAN_MASTER can Host id
/// Parameter type uint8_t [0, 127]
CONFIG_WR_CAN_MASTER = 0x200b,
/// CAN_TIMEOUT can timeout threshold, default 0
/// Parameter type uint32_t [0, 10000]
CONFIG_WR_CAN_TIMEOUT = 0x200c,
/// motorOverTemp motor protection temperature value, temp (degrees) *10
/// Parameter type uint16_t [0, 1500]
CONFIG_WR_MOTOR_OVER_TEMP = 0x200d,
/// overTempTime over temperature time
/// Parameter type uint32_t [0, 100000]
CONFIG_WR_OVER_TEMP_TIME = 0x200e,
/// GearRatio gear ratio
/// Parameter type float [1, 64]
CONFIG_WR_GEAR_RATIO = 0x200f,
/// Tq_caliType torque calibration method setting
/// parameter type uint8_t 0, 1
CONFIG_WR_TQ_CALI_TYPE = 0x2010,
/// cur_filt_gain current filter parameter
/// parameter type float [0, 1]
CONFIG_WR_CUR_FILT_GAIN = 0x2011,
/// cur_kp current kp
/// parameter type float [0, 200]
CONFIG_WR_CUR_KP = 0x2012,
/// cur_ki current ki
/// parameter type float [0, 200]
CONFIG_WR_CUR_KI = 0x2013,
/// spd_kp speed kp
/// parameter type float [0, 200]
CONFIG_WR_SPD_KP = 0x2014,
/// spd_ki speed ki
/// parameter type float [0, 200]
CONFIG_WR_SPD_KI = 0x2015,
/// loc_kp position kp
/// parameter type float [0, 200]
CONFIG_WR_LOC_KP = 0x2016,
/// spd_filt_gain speed filter parameter
/// parameter type float [0, 1]
CONFIG_WR_SPD_FILT_GAIN = 0x2017,
/// limit_spd position loop speed limit
/// parameter type float [0, 200]
CONFIG_WR_LIMIT_SPD = 0x2018,
/// limit_cur position speed control current limit
/// parameter type float [0, 27]
CONFIG_WR_LIMIT_CUR = 0x2019,
/// timeUse0
/// parameter type uint16_t
CONFIG_R_TIME_USE0 = 0x3000,
/// timeUse1
/// parameter type uint16_t
CONFIG_R_TIME_USE1 = 0x3001,
/// timeUse1
/// parameter type uint16_t
CONFIG_R_TIME_USE2 = 0x3002,
/// timeUse1
/// parameter type uint16_t
CONFIG_R_TIME_USE3 = 0x3003,
/// encoderRaw magnetic encoder sampling value
/// parameter type uint16_t
CONFIG_R_ENCODER_RAW = 0x3004,
/// mcuTemp mcu internal temperature, *10
/// parameter type uint16_t
CONFIG_R_MCU_TEMP = 0x3005,
/// motorTemp motor ntc temperature, *10
/// parameter type uint16_t
CONFIG_R_MOTOR_TEMP = 0x3006,
/// vBus(mv) bus voltage
/// parameter type uint16_t
CONFIG_R_VBUS_MV = 0x3007,
/// adc1Offset adc sampling channel 1 zero current offset
/// parameter type int32_t
CONFIG_R_ADC1_OFFSET = 0x3008,
 /// adc2Offset adc sampling channel 2 zero current offset
/// parameter type int32_t
CONFIG_R_ADC2_OFFSET = 0x3009,
/// adc1Raw adc sampling value 1
/// parameter type uint32_t
CONFIG_R_ADC1_RAW = 0x300a,
/// adc2Raw adc sampling value 2
/// parameter type uint32_t
CONFIG_R_ADC2_RAW = 0x300b,
/// VBUS bus voltage V
/// parameter type float
CONFIG_R_VBUS_V = 0x300c,
/// cmdId id ring instruction, A
/// parameter type float
CONFIG_R_CMD_ID = 0x300d,
/// cmdIq iq ring instruction, A
/// parameter type float
CONFIG_R_CMD_IQ = 0x300e,
/// cmdlocref Position loop command, rad
/// Parameter type float
CONFIG_R_CMD_LOC_REF = 0x300f,
/// cmdspdref Speed ​​loop command, rad/s
/// Parameter type float
CONFIG_R_CMD_SPD_REF = 0x3010,
/// cmdTorque Torque command, nm
/// Parameter type float
CONFIG_R_CMD_TORQUE = 0x3011,
/// cmdPos mit protocol angle command
/// Parameter type float
CONFIG_R_CMD_POS = 0x3012,
/// cmdVel mit protocol velocity command
/// Parameter type float
CONFIG_R_CMD_VEL = 0x3013,
/// rotation Number of revolutions
/// Parameter type int16_t
CONFIG_R_ROTATION = 0x3014,
/// modPos Mechanical angle of motor without counting turns, rad
/// Parameter type float
CONFIG_R_MOD_POS = 0x3015,
/// mechPos Mechanical angle of load end counting turns, rad
/// Parameter type float
CONFIG_R_MECH_POS = 0x3016,
/// mechVel Load end speed, rad/s
/// Parameter type float
CONFIG_R_MECH_VEL = 0x3017,
/// elecPos Electrical angle
/// Parameter type float
CONFIG_R_ELEC_POS = 0x3018,
/// ia U line current, A
/// Parameter type float
CONFIG_R_IA = 0x3019,
/// ib V line current, A
/// Parameter type float
CONFIG_R_IB = 0x301a,
/// ic W line current, A
/// parameter type float
CONFIG_R_IC = 0x301b,
/// tick
/// parameter type uint32_t
CONFIG_R_TICK = 0x301c,
/// phaseOrder calibration direction mark
/// parameter type uint8_t
CONFIG_R_PHASE_ORDER = 0x301d,
/// iqf iq filter value, A
/// parameter type float
CONFIG_R_IQF = 0x301e,
/// boardTemp board temperature, *10
/// parameter type int16_t
CONFIG_R_BOARD_TEMP = 0x301f,
/// iq iq original value, A
/// parameter type float
CONFIG_R_IQ = 0x3020,
/// id id original value, A
/// parameter type float
CONFIG_R_ID = 0x3021,
/// faultSta fault status value
/// parameter type uint32_t
CONFIG_R_FAULT_STATUS = 0x3022,
/// warnSta warning status value
/// parameter type uint32_t
CONFIG_R_WARN_STATUS = 0x3023,
/// drv_fault driver chip fault value
/// parameter type uint16_t
CONFIG_R_DRV_FAULT = 0x3024,
/// drv_temp driver chip temperature value, degrees
/// parameter type int16_t
CONFIG_R_DRV_TEMP = 0x3025,
/// Uq q axis voltage
/// parameter type float
CONFIG_R_UQ = 0x3026,
/// Ud d-axis voltage
/// Parameter type float
CONFIG_R_UD = 0x3027,
/// dtc_u U-phase output duty cycle
/// Parameter type float
CONFIG_R_DTC_U = 0x3028,
/// dtc_v V-phase output duty cycle
/// Parameter type float
CONFIG_R_DTC_V = 0x3029,
/// dtc_w W-phase output duty cycle
/// Parameter type float
CONFIG_R_DTC_W = 0x302a,
/// v_bus closed loop vbus
/// Parameter type
CONFIG_R_CLOSED_LOOP_V_BUS = 0x302b,
/// v_ref closed loop vq,vd composite voltage
/// Parameter type float
CONFIG_R_CLOSED_LOOP_V_REF = 0x302c,
/// torque_fdb torque feedback value, nm
/// parameter type float
CONFIG_R_TORQUE_FDB = 0x302d,
/// rated_i motor rated current
/// parameter type float
CONFIG_R_RATED_I = 0x302e,
/// limit_i motor limit maximum current
/// parameter type float
CONFIG_R_LIMIT_I = 0x302f,
} cyber_gear_config_index_t;

typedef enum {
/// operation control mode
/// 0: operation control mode 1: position mode 2: speed mode 3: current mode
/// parameter type uint8_t
/// parameter byte number 1
/// parameter description
PARAMETER_RUN_MODE = 0x7005,
/// Current mode Iq command
/// Parameter type float
/// Parameter byte count 4
/// Parameter description -27~27A
PARAMETER_IQ_REF = 0x7006,
/// Speed ​​mode speed command
/// Parameter type float
/// Parameter byte count 4
/// Parameter description -30~30rad/s
PARAMETER_SPD_REF = 0x700A,
/// Torque limit
/// Parameter type float
/// Parameter byte count 4
/// Parameter description 0~12Nm
PARAMETER_LIMIT_TORQUE = 0x700B,
/// Current Kp
/// Parameter type float
/// Parameter byte count 4
/// Parameter description Default value 0.125
PARAMETER_CUR_KP = 0x7010,
/// Current Ki
/// Parameter type float
/// Parameter byte count 4
/// Parameter description Default value 0.0158
PARAMETER_CUR_KI = 0x7011,
/// Current filter coefficient filt_gain
/// Parameter type float
/// Parameter byte number 4
/// Parameter description Default value 0~1.0, default value W/R 0.1
PARAMETER_CUR_FILT_GAIN = 0x7014,
/// Position mode angle command
/// Parameter type float
/// Parameter byte number 4
/// Parameter description rad
PARAMETER_LOC_REF = 0x7016,
/// Position mode speed setting
/// Parameter type float
/// Parameter byte number 4
/// Parameter description 0~30rad/s
PARAMETER_LIMIT_SPD = 0x7017,
/// Speed ​​position mode current setting
/// Parameter type float
/// Parameter byte number 4
/// Parameter description 0~27A
PARAMETER_LIMIT_CUR = 0x7018,
} cyber_gear_read_write_parameter_index_t;

/*
* CyberGear CAN packet definition
*/
typedef struct {
    union {
    uint32_t value;
    uint8_t bytes[4];
    } can_id; // CAN ID, 29 bits of valid data

    union {
    uint64_t value;
    uint8_t bytes[8];
    } can_data; // data area, 8 bytes
} cyber_gear_can_t;

/*
* CyberGear communication mode 1 send control instructions
*/
typedef struct {
uint8_t motor_can_id; // target motor CAN ID
float target_location; // target angle (-4π~4π)
float target_speed; // target angular velocity (-30rad/s~30rad/s)
float kp; // Kp (0.0~500.0)
float kd; // Kd (0.0~5.0)
float torque; // Torque correspondence (-12Nm~12Nm)
} cyber_gear_motion_control_t;

/*
* CyberGear communication mode 2 motor feedback data
*/
typedef enum {
MOTOR_MODE_RESET, // Reset mode
MOTOR_MODE_CALI, // Calibration mode
MOTOR_MODE_MOTOR // Operation mode
} cyber_gear_motor_mode_t;
typedef struct {
uint8_t host_can_id; // Host CAN_ID
uint8_t motor_can_id; // Target motor CAN ID
float current_torque; // Torque [-12, 12] Unit N/m
float current_location; // Current angle [-4pi, 4pi]
float current_speed; // Current angular velocity [-30rad/s, 30rad/s]
float current_temperature; // Current temperature: Temp (degrees Celsius) * 10
int has_calibration_error; // Calibration error
int has_hall_encode_error; // HALL encoding error
int has_magnetic_encoding_error; // Magnetic encoding error
int has_over_temperature; // Overtemperature error
int has_over_current; // Overcurrent error
int has_undervoltage; // Undervoltage error
cyber_gear_motor_mode_t mode_type; // Run mode
} cyber_gear_motor_status_t;

/*
* CyberGear communication mode 17 single parameter read
*/
typedef struct {
uint8_t host_can_id; // host CAN_ID
uint8_t motor_can_id; // target motor CAN ID

cyber_gear_read_write_parameter_index_t index; // parameter index

union {
uint32_t value;
uint8_t bytes[4];
} data; // parameter value
} cyber_gear_single_parameter_t;

/* Initialize a CyberGear CAN frame */
void cyber_gear_can_init(const cyber_gear_can_t *frame);

/* Dump a CyberGear CAN frame */
void cyber_gear_can_dump(const cyber_gear_can_t * const frame);

/* Assign integer values ​​to specific bits in the CAN ID area
* @param: frame frame to be set
* @param: bit_start bit start position
* @param: bit_length bit length
* @param: value set value (integer)
* */
void cyber_gear_set_can_id_int_value(const cyber_gear_can_t *frame, int bit_start, int bit_length, int value);

/* Get specific bit data in the CAN ID area
* @param: frame frame to be set
* @param: bit_start bit start position
* @param: bit_length bit length
* @return: int value data
* */
int cyber_gear_get_can_id_int_value(const cyber_gear_can_t *frame, int bit_start, int bit_length);

/* Set the communication type for the CAN ID
* @param: frame frame to be set
* @param: type Communication type
* */
void cyber_gear_set_can_id_communication_type(const cyber_gear_can_t *frame, cyber_gear_can_communication_type_t type);

/* Set the communication type variant for the CAN ID
* @param: frame frame to be set
* @param: type variant
* */
void cyber_gear_set_can_id_communication_type_variant(const cyber_gear_can_t *frame, uint8_t type);

/* Set host CANID for CAN ID
* @param: frame Frame to be set
* @param: value Host CANID
* */
void cyber_gear_set_can_id_host_can_id(const cyber_gear_can_t *frame, int value);

/* Set target motor CANID for CAN ID
* @param: frame Frame to be set
* @param: value Motor CANID
* */
void cyber_gear_set_can_id_target_can_id(const cyber_gear_can_t *frame, int value);

/* Construct a CAN packet (communication type 18) for parameter writing, parameter value is integer
* @param: frame Frame to be set
* @param: index Parameter index
* @param: value parameter value
* */
void cyber_gear_build_parameter_write_frame_with_int_value(const cyber_gear_can_t *frame, cyber_gear_read_write_parameter_index_t index, int value);

/* Construct a CAN packet for parameter writing (communication type 18), the parameter value is a floating point
* @param: frame frame to be set
* @param: index parameter index
* @param: value parameter value
* */
void cyber_gear_build_parameter_write_frame_with_float_value(const cyber_gear_can_t *frame, cyber_gear_read_write_parameter_index_t index, float value);

/* Construct a CAN packet for parameter reading (communication type 17)
* @param: frame frame to be set
*/
void cyber_gear_build_parameter_read_frame(const cyber_gear_can_t *frame, cyber_gear_read_write_parameter_index_t index);

/* Parse a parameter read CAN packet (communication type 17)
* @param: frame frame to be set
*/
cyber_gear_single_parameter_t cyber_gear_parse_parameter_read_frame(const cyber_gear_can_t *frame);

/* Get the communication type of the frame
* @param: frame parsed frame
* @return: cyber_gear_can_communication_type_t communication type
* */
cyber_gear_can_communication_type_t cyber_gear_get_can_id_communication_type(const cyber_gear_can_t * const frame);

/* Get the target CAN_ID of the frame
* @param: frame frame to be set
* @return: int target CAN_ID
* */
uint8_t cyber_gear_get_can_id_target_id(const cyber_gear_can_t * const frame);

/* Get the host CAN_ID of the frame
* @param: frame The frame to be set
* @return: int host CAN_ID
* */
uint8_t cyber_gear_get_can_id_host_id(const cyber_gear_can_t * const frame);

/* Operation mode motor control instructions (communication type 1) are used to send control instructions to the motor
* @param: control_param control parameters
* */
void cyber_gear_build_motion_control_frame(const cyber_gear_can_t *frame, const cyber_gear_motion_control_t control_param);

/* Parse communication type 2 motor operation status frame
* @param: frame The frame to be parsed
* @return: cyber_gear_can_communication_type_t Communication type
* */
cyber_gear_motor_status_t cyber_gear_parse_motor_status_frame(const cyber_gear_can_t * const frame);

/* Parse communication type 6 to build a mechanical zero position frame
* @param: frame frame to be parsed
* @return: cyber_gear_can_communication_type_t communication type
* */
void cyber_gear_build_set_mechanical_zero_position_frame(const cyber_gear_can_t * frame);

/* Parse communication type 7 to build a setting CAN ID frame
* @param: frame frame to be parsed
* @return: cyber_gear_can_communication_type_t communication type
* */
void cyber_gear_build_set_can_id_frame(const cyber_gear_can_t * frame, int setting_can_id);

/* Dump a CyberGear motor running status frame frame */
void cyber_gear_dump_motor_status_frame(const cyber_gear_motor_status_t status);
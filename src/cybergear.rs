use crate::bindings::{
    cyber_gear_can_communication_type_t, cyber_gear_can_init, cyber_gear_can_t,
    cyber_gear_motor_status_t, cyber_gear_set_can_id_communication_type,
    cyber_gear_set_can_id_host_can_id, cyber_gear_set_can_id_target_can_id,
};
use embedded_can::nb::Can;

#[derive(Debug, Default)]
struct LogParameters {
    pub last_rcv: u32,
    pub param_count: u16,
    pub freq: u16, // Hz
    pub param_slot_pid: [u16; 8],
    pub param_slot_value_ptr: [f32; 8],
    pub expected_frame_number: u8,
    pub data_buffer: [u8; 4], // Temporary buffer of half values, 4 bytes long

    pub torque_fdb: f32, // CONFIG_R_TORQUE_FDB
    pub dtc_u: f32,      // CONFIG_R_DTC_U - for arming/enable checking
    pub v_bus: f32,      // CONFIG_R_VBUS_V - input voltage monitoring
    pub vel: f32,        // CONFIG_R_MECH_VEL
    pub pos: f32,        // CONFIG_R_MECH_POS
}

#[derive(Debug, Default)]
struct Homing {
    pub homing: u32,
    pub timeout: u32,
    pub current: f32,
    pub position: f32,
    pub state: i32,
    pub is_homed: bool,
}

#[derive(Debug)]
pub struct CyberGear<C: Can> {
    can: C,
    host_id: u8,
    motor_id: u8,
    uuid: u64,
    homing: Homing,
    log_parameters: LogParameters,

    target_position: f32,

    default_params: Parameters,
    current_params: Parameters,

    last_cmd_sent: u32,
    last_response: u32,
    last_status_frame_req: u32,
    last_status_frame_rec: u32,

    enabled: bool,
    logging_active: bool,

    can_send_error: u32,
    fault_time: u32,

    wait_for_response: u32,

    current_mode: MotorMode,           // Custom type
    status: cyber_gear_motor_status_t, // Custom type
    direction: i32,

    temperature: f32,
    voltage: f32,
    current_position: f32,
}

impl<C: Can> CyberGear<C> {
    pub fn new(id: u8, can_dev: C) -> Self {
        Self {
            motor_id: id,
            host_id: 0x7D,
            direction: 1,
            can: can_dev,
            current_mode: Default::default(),
            logging_active: false,
            wait_for_response: 0,
            default_params: Parameters::default(),
            current_position: 0.0,
            target_position: 0.0,
            last_response: 0,
            last_cmd_sent: 0,
            last_status_frame_rec: 0,
            fault_time: 0,
            enabled: false,
            voltage: 0.0,
            last_status_frame_req: 0,
            uuid: 0,
            temperature: 0.0,
            status: cyber_gear_motor_status_t::default(),
            can_send_error: 0,
            current_params: Parameters::default(),
            homing: Homing {
                current: 0.6,
                ..Default::default()
            },
            log_parameters: LogParameters {
                freq: 50,
                ..Default::default()
            },
        }
    }

    pub(crate) fn init_frame(
        &self,
        frame: *mut cyber_gear_can_t,
        can_type: cyber_gear_can_communication_type_t,
    ) {
        unsafe {
            cyber_gear_can_init(frame);
            cyber_gear_set_can_id_host_can_id(frame, self.host_id as i32);
            cyber_gear_set_can_id_target_can_id(frame, self.motor_id as i32);
            cyber_gear_set_can_id_communication_type(frame, can_type);
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Parameters {
    speed: f32,
    torque: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    current_limit: f32,
    speed_limit: f32,
    torque_limit: f32,
    filter_gain: f32,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            speed: 2.0,
            torque: 4.0,
            kp: 1.0,
            ki: 0.01,
            kd: 0.01,
            current_limit: 5.0,
            speed_limit: 1.0,
            torque_limit: 4.0,
            filter_gain: 1.0,
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MotorCommand {
    MotorStop = 0,
    EnableMotor,

    FetchStatus,
    SetZeroPos,

    SetModeCurrent,
    SetModeSpeed,
    SetModePosition,
    SetModeOperation,

    SetCurrentLimit,
    SetSpeedLimit,
    SetTorqueLimit,
    SetKi,
    SetKp,
    SetFilterGain,

    SetTargetPosition,
    SetTargetSpeed,
    SetTargetCurrent,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum MotorMode {
    #[default]
    None = -1,
    Operation = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum MotorState {
    Idle,
    Start,
    Homing,
    HomeFound,
    Wait1,
    MoveZero,
    Wait2,
    Timeout,
    End,
}

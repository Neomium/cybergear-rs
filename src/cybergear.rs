#![allow(async_fn_in_trait)]

use crate::bindings::{
    cyber_gear_build_motion_control_frame, cyber_gear_build_parameter_write_frame_with_float_value,
    cyber_gear_build_parameter_write_frame_with_int_value, cyber_gear_can_communication_type_t,
    cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
    cyber_gear_can_communication_type_t_COMMUNICATION_ENABLE_DEVICE,
    cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID,
    cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING,
    cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
    cyber_gear_can_communication_type_t_COMMUNICATION_READ_SINGLE_PARAM,
    cyber_gear_can_communication_type_t_COMMUNICATION_SET_MECHANICAL_ZERO_POSITION,
    cyber_gear_can_communication_type_t_COMMUNICATION_STATUS_REPORT,
    cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM, cyber_gear_can_init,
    cyber_gear_can_t, cyber_gear_can_t__bindgen_ty_1, cyber_gear_can_t__bindgen_ty_2,
    cyber_gear_config_index_t_CONFIG_R_MECH_POS, cyber_gear_config_index_t_CONFIG_R_MECH_VEL,
    cyber_gear_config_index_t_CONFIG_R_TORQUE_FDB, cyber_gear_get_can_id_communication_type,
    cyber_gear_motion_control_t, cyber_gear_motor_status_t, cyber_gear_parse_motor_status_frame,
    cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_FILT_GAIN,
    cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_KI,
    cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_KP,
    cyber_gear_read_write_parameter_index_t_PARAMETER_IQ_REF,
    cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_CUR,
    cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_SPD,
    cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_TORQUE,
    cyber_gear_read_write_parameter_index_t_PARAMETER_LOC_REF,
    cyber_gear_read_write_parameter_index_t_PARAMETER_RUN_MODE,
    cyber_gear_read_write_parameter_index_t_PARAMETER_SPD_REF,
    cyber_gear_set_can_id_communication_type, cyber_gear_set_can_id_communication_type_variant,
    cyber_gear_set_can_id_host_can_id, cyber_gear_set_can_id_target_can_id,
};
use crate::frame::{CanFrameAdapter, CyberGearFrame};
use core::fmt::Debug;
use defmt::{Debug2Format, Format, Formatter, error, info, warn, write};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
#[cfg(not(test))]
use embassy_time::{Instant, Timer};
use embedded_can::nb::Can;
use embedded_can::{Frame, Id};
use nb::{Error, block};

#[derive(Debug)]
#[allow(dead_code)]
pub struct LogParameters {
    last_rcv: u32,
    param_count: u16,
    freq: u16, // Hz
    param_slot_pid: [u16; 8],
    param_slot_value_ptr: [*mut f32; 8],
    expected_frame_number: u8,
    data_buffer: [u8; 4], // Temporary buffer of half-values, 4 bytes long

    torque_fdb: f32, // CONFIG_R_TORQUE_FDB
    dtc_u: f32,      // CONFIG_R_DTC_U - for arming/enable checking
    v_bus: f32,      // CONFIG_R_VBUS_V - input voltage monitoring
    vel: f32,        // CONFIG_R_MECH_VEL
    pos: f32,        // CONFIG_R_MECH_POS
}

impl Default for LogParameters {
    fn default() -> Self {
        Self {
            last_rcv: 0,
            param_count: 0,
            freq: 0,
            param_slot_pid: [0; 8],
            param_slot_value_ptr: [0 as *mut f32; 8],
            expected_frame_number: 0,
            data_buffer: [0; 4],
            torque_fdb: 0.0,
            dtc_u: 0.0,
            v_bus: 0.0,
            vel: 0.0,
            pos: 0.0,
        }
    }
}

#[derive(Debug, Default)]
#[allow(dead_code)]
pub struct Homing {
    pub homing_ms: u32,
    pub timeout: u32,
    pub current: f32,
    pub hstate_change: u32,
    pub position: f32,
    pub state: MotorState,
    pub is_homed: bool,
    pub home_found: bool,
}

#[derive(Debug, Copy, Clone)]
pub enum GearError<C>
where
    C: Can,
    C::Error: Debug,
{
    CanWriteError(Error<C::Error>),

    MotorStateConversionError,

    StartLoggingError,
}

impl<C> Format for GearError<C>
where
    C: Can,
    C::Error: Debug,
{
    fn format(&self, fmt: Formatter) {
        match self {
            Self::CanWriteError(e) => write!(fmt, "CanWriteError: {:?}", Debug2Format(&e)),
            Self::MotorStateConversionError => write!(fmt, "MotorStateConversionError"),
            Self::StartLoggingError => write!(fmt, "StartLoggingError"),
        }
    }
}

pub trait ServoMotor {
    type Error;
    type Frame: Frame;
    async fn init(&mut self) -> Result<(), Self::Error>;

    fn set_host_id(&mut self, id: u8);

    fn set_home_position(&mut self, position: f32);

    fn set_homed(&mut self, homed: bool);

    fn reverse_direction(&mut self);

    async fn set_parameters(
        &mut self,
        current_limit: f32,
        speed_limit: f32,
        torque_limit: f32,
        ki: f32,
        kp: f32,
        kd: f32,
        filter_gain: f32,
    ) -> Result<(), Self::Error>;

    /// Set current limit for servo motor
    async fn set_current_limit(&mut self, limit: f32) -> Result<(), Self::Error>;

    /// Run motor to position
    async fn run_position_safe(&mut self, position: f32) -> Result<(), Self::Error>;

    /// Process incoming CAN frame
    async fn process_inframe(&mut self, frame: &Self::Frame) -> bool;

    /// Set target position for motor
    fn set_target_position(&mut self, position: f32);

    /// Get target position for motor
    fn get_target_position(&self) -> f32;

    /// Returns true if motor is homed
    fn is_homed(&self) -> bool;

    /// Home motor
    fn home(&mut self, timeout: u32, current: f32);

    /// Disable motor
    async fn disable(&mut self) -> Result<(), Self::Error>;

    /// Enable motor
    async fn enable(&mut self) -> Result<(), Self::Error>;

    /// Get logged torque value for motor
    fn get_torque_fdb(&self) -> f32;

    /// Get logged motor position
    fn get_position(&self) -> f32;

    /// Runs homing sequence for motor
    async fn execute(&mut self);

    /// Returns instant of last homing command, if > 0 homing is in progress
    fn homing_instant(&self) -> u32;

    /// Returns true if motor is enabled
    fn is_enabled(&self) -> bool;

    /// Returns instant in ms of last command sent
    fn last_cmd_sent(&self) -> u32;
}

/// Xiaomi Cybergear servo motor
#[allow(dead_code)]
pub struct CyberGear<C: Can + 'static, A: CanFrameAdapter<C::Frame>> {
    /// CAN Bus
    can: &'static Mutex<CriticalSectionRawMutex, Option<C>>,

    /// CAN frame adapter to convert types from and to CyberGearFrame
    adapter: A,

    /// Host CAN ID
    host_id: u8,

    /// Motor CAN ID
    motor_id: u8,

    /// Servo uuid
    uuid: u64,

    /// Homing parameters
    homing: Homing,

    /// Logging parameters
    log_parameters: LogParameters,

    /// Servo target position
    target_position: f32,

    /// Default servo parameters
    default_params: Parameters,

    /// Current servo parameters
    current_params: Parameters,

    /// Time instant in ms of last command sent
    last_cmd_sent: u32,

    /// Time instant in ms of last response received
    last_response: u32,

    /// Time instant in ms of last status frame request sent
    last_status_frame_req: u32,

    /// Time instant in ms of last status frame received
    last_status_frame_rec: u32,

    /// If true, servo is enabled
    enabled: bool,

    /// If true, logging is activated
    logging_active: bool,

    /// Time instant in ms of last CAN send error
    can_send_error: u32,

    /// Time instant in of ms of last fault
    fault_time: u32,

    /// Cybergear operating mode
    current_mode: MotorMode,

    /// Cybergear motor status
    status: cyber_gear_motor_status_t,

    /// Direction of rotation
    direction: i32,
}

impl<C, A> ServoMotor for CyberGear<C, A>
where
    C: Can + 'static,
    A: CanFrameAdapter<C::Frame>,
    <C as Can>::Error: Debug,
{
    type Error = GearError<C>;
    type Frame = C::Frame;
    async fn init(&mut self) -> Result<(), Self::Error> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID,
        );
        self.send_frame(frame, true).await
    }

    fn set_host_id(&mut self, id: u8) {
        self.host_id = id;
    }

    fn set_home_position(&mut self, position: f32) {
        self.homing.position = position;
    }

    fn set_homed(&mut self, homed: bool) {
        self.homing.is_homed = homed;
    }

    fn reverse_direction(&mut self) {
        self.direction = -1 * self.direction;
    }

    async fn set_parameters(
        &mut self,
        current_limit: f32,
        speed_limit: f32,
        torque_limit: f32,
        ki: f32,
        kp: f32,
        kd: f32,
        filter_gain: f32,
    ) -> Result<(), Self::Error> {
        self.default_params = Parameters {
            current_limit,
            speed_limit,
            torque_limit,
            ki,
            kp,
            kd,
            filter_gain,
            ..Default::default()
        };

        self.set_current_limit(current_limit).await?;
        self.set_speed_limit(speed_limit).await?;
        self.set_torque_limit(torque_limit).await?;
        delay_ms(5).await;
        self.set_ki(ki).await?;
        self.set_kp(kp).await?;
        self.set_kd(kd)?;
        self.set_filter_gain(filter_gain).await?;
        delay_ms(50).await;
        Ok(())
    }

    async fn set_current_limit(&mut self, limit: f32) -> Result<(), Self::Error> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );
        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_CUR,
                limit,
            );
        }
        self.current_params.current_limit = limit;
        self.send_frame(frame, false).await
    }
    async fn run_position_safe(&mut self, position: f32) -> Result<(), Self::Error> {
        if self.is_ready().await && self.homing.homing_ms == 0 {
            return self.run_position(position).await;
        }

        Ok(())
    }

    async fn process_inframe(&mut self, frame: &Self::Frame) -> bool {
        self.process_frame(frame).await
    }

    fn set_target_position(&mut self, position: f32) {
        self.target_position = position;
    }

    fn get_target_position(&self) -> f32 {
        self.target_position
    }

    fn is_homed(&self) -> bool {
        self.homing.is_homed
    }
    fn home(&mut self, timeout: u32, current: f32) {
        self.homing.state = MotorState::Start;
        self.homing.homing_ms = now();
        self.homing.timeout = timeout;
        self.homing.current = current;
    }

    async fn disable(&mut self) -> Result<(), Self::Error> {
        self.disable_motor().await
    }

    async fn enable(&mut self) -> Result<(), Self::Error> {
        self.enable_motor().await
    }

    fn get_torque_fdb(&self) -> f32 {
        self.log_parameters.torque_fdb * self.direction as f32
    }

    fn get_position(&self) -> f32 {
        self.log_parameters.pos
    }

    async fn execute(&mut self) {
        let _ = self.request_status_report().await;

        // homing process:
        // 0. logging for speed and force feedback needs to be enabled
        // 1. set to low speed and low torque
        // 2. run in speed mode with a timeout (5s)
        // 3. watch for velocity below threshold (0.1), give some time in the beginning for the motor to start rotating (1s)
        // 4. if vel < threshold, we found our home pos. save it and
        // 5. move to requested/current target position with medium speed.
        // 6. restore the old current and speed settings
        // error handling. on timeout stop the motor and set the current position as zero, as the motor will otherwise rewind to its starting position on the next position command
        //info!("homing state: {:?}", self.homing.state);
        match self.homing.state {
            MotorState::Idle => {}
            MotorState::Start => {
                let _ = self.set_current_limit(self.homing.current).await;
                let _ = self.set_speed_limit(4.0).await;
                let _ = self.run_speed(4.0).await;
                info!("Started homing");
                self.homing.hstate_change = now();
                self.homing.home_found = false;
                self.homing.state = MotorState::Homing;
            }
            MotorState::Homing => {
                if now() - self.log_parameters.last_rcv < 100 {
                    if now() - self.homing.hstate_change > 1000
                        && self.log_parameters.vel.abs() < 0.1
                    {
                        let _ = self.run_speed(0.0).await;
                        info!("Home position found");
                        self.homing.home_found = true;

                        self.homing.state = MotorState::HomeFound;
                    }
                }

                if now() - self.homing.hstate_change > self.homing.timeout {
                    info!("Homing timeout");
                    self.homing.state = MotorState::Timeout;
                }
            }
            MotorState::HomeFound => {
                if self.set_zero().await.is_ok() {
                    self.homing.hstate_change = now();
                    self.homing.state = MotorState::Wait1;
                }
            }
            MotorState::Wait1 => {
                if now() - self.homing.hstate_change > 1500 {
                    let _ = self.set_current_limit(4.0).await;
                    let _ = self.set_speed_limit(4.0).await;
                    let _ = self.run_position(self.target_position).await;
                    self.homing.state = MotorState::MoveZero;
                    self.homing.hstate_change = now();
                }
            }
            MotorState::MoveZero => {
                if now() - self.homing.hstate_change > 1000 && self.log_parameters.vel.abs() < 0.1 {
                    info!("Moved to target position");
                    self.homing.state = MotorState::End;
                    self.homing.hstate_change = now();
                }
            }
            MotorState::Wait2 => {}
            MotorState::Timeout => {
                let _ = self.run_speed(0.0).await;
                delay_ms(3u64).await;
                let _ = self.set_zero().await;
                self.homing.state = MotorState::End;
            }
            MotorState::End => {
                let _ = self
                    .set_current_limit(self.default_params.current_limit)
                    .await;
                let _ = self.set_speed_limit(self.default_params.speed_limit).await;
                self.homing.homing_ms = 0;
                self.homing.state = MotorState::Idle;
                self.homing.is_homed = self.homing.home_found;
                info!("Ending homing sequence");
            }
        }

        if self.can_send_error != 0 && now() - self.can_send_error > 2000 {
            warn!("CAN error, restoring settings");
            let _ = self.clear_fault().await;
            let _ = self.restore_config().await;
            self.homing.is_homed = false;
            self.can_send_error = 0;
        }
    }

    fn homing_instant(&self) -> u32 {
        self.homing.homing_ms
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }

    fn last_cmd_sent(&self) -> u32 {
        self.last_cmd_sent
    }
}

impl<C, A> CyberGear<C, A>
where
    C: Can + 'static,
    A: CanFrameAdapter<C::Frame>,
    <C as Can>::Error: Debug,
{
    pub fn new(
        can_id: u8,
        can_dev: &'static Mutex<CriticalSectionRawMutex, Option<C>>,
        adapter: A,
    ) -> Self {
        Self {
            motor_id: can_id,
            adapter,
            host_id: 0x7D,
            direction: 1,
            can: can_dev,
            current_mode: Default::default(),
            logging_active: false,
            default_params: Parameters::default(),
            target_position: 0.0,
            last_response: 0,
            last_cmd_sent: 0,
            last_status_frame_rec: 0,
            fault_time: 0,
            enabled: false,
            last_status_frame_req: 0,
            uuid: 0,
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

    async fn set_mode(&mut self, mode: MotorMode) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_int_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_RUN_MODE,
                mode as i32,
            );
        }
        self.send_frame(frame, false).await?;
        self.current_mode = mode;
        info!("Changed mode to: {:?}", mode);
        Ok(())
    }

    async fn enable_motor(&mut self) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_ENABLE_DEVICE,
        );
        self.send_frame(frame, false).await?;
        self.enabled = true;

        if !self.start_logging().await {
            error!("Failed to start logging");
            return Err(GearError::StartLoggingError);
        }

        Ok(())
    }

    #[allow(unused)]
    async fn check_enabled(&mut self) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_READ_SINGLE_PARAM,
        );
        frame.can_data.value = cyber_gear_config_index_t_CONFIG_R_TORQUE_FDB as u64;
        self.send_frame(frame, false).await?;
        self.enabled = true;
        Ok(())
    }

    async fn clear_fault(&mut self) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
        );
        unsafe {
            frame.can_data.bytes[0] = 0x1;
        }
        self.send_frame(frame, false).await?;
        if self.enabled {
            self.enable_motor().await?;
        }
        info!("Fault cleared");
        Ok(())
    }

    async fn request_status_report(&mut self) -> bool {
        if now() - self.last_status_frame_req < 100 {
            return false;
        }

        let mut frame = cyber_gear_can_t::new();

        if self.enabled {
            self.init_frame(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_can_communication_type_t_COMMUNICATION_ENABLE_DEVICE,
            );
        } else {
            self.init_frame(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
            );
        }

        if self.send_frame(frame, false).await.is_ok() {
            self.last_status_frame_req = now();
            return true;
        }
        false
    }

    async fn disable_motor(&mut self) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
        );
        self.send_frame(frame, false).await?;
        self.enabled = false;
        self.stop_logging().await;
        Ok(())
    }

    async fn set_speed_limit(&mut self, limit: f32) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );
        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_SPD,
                limit,
            );
        }
        self.current_params.speed_limit = limit;
        self.send_frame(frame, false).await
    }

    async fn set_torque_limit(&mut self, limit: f32) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );
        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_LIMIT_TORQUE,
                limit,
            );
        }
        self.current_params.torque_limit = limit;
        self.send_frame(frame, false).await
    }

    async fn set_ki(&mut self, ki: f32) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_KI,
                ki,
            );
        }

        self.current_params.ki = ki;
        self.send_frame(frame, false).await
    }

    async fn set_kp(&mut self, kp: f32) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_KP,
                kp,
            );
        }

        self.current_params.kp = kp;
        self.send_frame(frame, false).await
    }

    // No kd value for cyber gear?
    fn set_kd(&mut self, kd: f32) -> Result<(), GearError<C>> {
        //  let mut frame = cyber_gear_can_t::new();
        //  self.init_frame(
        //      &mut frame as *mut cyber_gear_can_t,
        //      cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        //  );
        //  unsafe {
        //      cyber_gear_build_parameter_write_frame_with_float_value(
        //          &mut frame as *mut cyber_gear_can_t,
        //          cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_KD,
        //          kd,
        //      );
        //  }
        //  self.send_frame(frame,false)?;
        self.current_params.kd = kd;
        Ok(())
    }

    async fn set_filter_gain(&mut self, gain: f32) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_CUR_FILT_GAIN,
                gain,
            );
        }

        self.current_params.filter_gain = gain;
        self.send_frame(frame, false).await
    }

    async fn run_speed(&mut self, speed: f32) -> Result<(), GearError<C>> {
        if self.current_mode != MotorMode::Speed {
            self.set_mode(MotorMode::Speed).await?;
        }
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
        );
        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_SPD_REF,
                speed * self.direction as f32,
            );
        }
        self.send_frame(frame, false).await
    }

    #[allow(unused)]
    async fn run_current(&mut self, current: f32) -> Result<(), GearError<C>> {
        if self.current_mode != MotorMode::Current {
            self.set_mode(MotorMode::Current).await?;
        }

        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_IQ_REF,
                current * self.direction as f32,
            );
        }

        self.send_frame(frame, false).await
    }

    async fn run_position(&mut self, position: f32) -> Result<(), GearError<C>> {
        if self.current_mode != MotorMode::Position {
            self.set_mode(MotorMode::Position).await?;
        }

        let mut frame = cyber_gear_can_t::new();

        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
        );

        unsafe {
            cyber_gear_build_parameter_write_frame_with_float_value(
                &mut frame as *mut cyber_gear_can_t,
                cyber_gear_read_write_parameter_index_t_PARAMETER_LOC_REF,
                position * self.direction as f32,
            );
        }

        self.send_frame(frame, false).await
    }

    #[allow(clippy::wrong_self_convention)]
    async fn is_ready(&mut self) -> bool {
        if now() - self.last_status_frame_rec < 5000 || now() - self.log_parameters.last_rcv < 100 {
            if self.status_error() {
                if self.fault_time == 0 {
                    self.fault_time = now();
                }

                info!(
                    "Motor Error:\r\nOC:{}\r\nUV:{}\r\nOT:{}\r\nCAL:{}\r\nHALL:{}\r\nMAG:{}",
                    self.status.has_over_current,
                    self.status.has_undervoltage,
                    self.status.has_over_temperature,
                    self.status.has_calibration_error,
                    self.status.has_hall_encode_error,
                    self.status.has_magnetic_encoding_error,
                );

                delay_ms(5).await;

                if let Err(_e) = self.clear_fault().await {
                    error!("CyberGear clear_fault error");
                }

                return false;
            }

            if self.fault_time != 0 {
                self.fault_time = 0;
            }

            return true;
        }

        self.request_status_report().await
    }

    #[allow(unused)]
    pub(crate) async fn run_op(
        &mut self,
        speed: f32,
        position: f32,
        torque: f32,
        kd: f32,
        kp: f32,
    ) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();

        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
        );

        let control_params = cyber_gear_motion_control_t {
            motor_can_id: self.motor_id,
            target_speed: speed,
            target_location: position * self.direction as f32,
            kd,
            kp,
            torque,
        };

        unsafe {
            cyber_gear_build_motion_control_frame(
                &mut frame as *mut cyber_gear_can_t,
                control_params,
            );
        }

        self.send_frame(frame, false).await
    }

    pub async fn set_parameters(
        &mut self,
        current_limit: f32,
        speed_limit: f32,
        torque_limit: f32,
        ki: f32,
        kp: f32,
        kd: f32,
        filter_gain: f32,
    ) -> Result<(), GearError<C>> {
        self.default_params = Parameters {
            current_limit,
            speed_limit,
            torque_limit,
            ki,
            kp,
            kd,
            filter_gain,
            ..Default::default()
        };

        self.set_current_limit(current_limit).await?;
        self.set_speed_limit(speed_limit).await?;
        self.set_torque_limit(torque_limit).await?;
        delay_ms(5).await;
        self.set_kp(kp).await?;
        self.set_ki(ki).await?;
        self.set_kd(kd)?;
        self.set_filter_gain(filter_gain).await?;
        delay_ms(50).await;
        Ok(())
    }

    pub async fn set_zero(&mut self) -> Result<(), GearError<C>> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_SET_MECHANICAL_ZERO_POSITION,
        );

        unsafe {
            frame.can_data.bytes[0] = 1;
        }

        self.send_frame(frame, false).await
    }

    async fn restore_config(&mut self) -> Result<(), GearError<C>> {
        self.set_current_limit(self.current_params.current_limit)
            .await?;
        self.set_speed_limit(self.current_params.speed_limit)
            .await?;
        self.set_torque_limit(self.current_params.torque_limit)
            .await?;

        delay_ms(5).await;

        self.set_kp(self.current_params.kp).await?;
        self.set_ki(self.current_params.ki).await?;
        self.set_kd(self.current_params.kd)?;
        self.set_filter_gain(self.current_params.filter_gain)
            .await?;
        self.set_mode(self.current_mode).await?;

        delay_ms(5).await;

        if self.enabled {
            self.enable_motor().await?;
        }
        Ok(())
    }

    async fn process_frame(&mut self, frame: &C::Frame) -> bool {
        let frame_cyber = self.adapter.from_frame(&frame);
        let mut cybergear_frame = cyber_gear_can_t::from(frame_cyber);
        let id_bytes = unsafe { cybergear_frame.can_id.bytes };

        let id = u32::from_le_bytes(id_bytes);

        if ((id & 0xFF00) >> 8) as u8 != self.motor_id {
            return false;
        }
        self.last_response = now();
        let comm_type = unsafe {
            cyber_gear_get_can_id_communication_type(&mut cybergear_frame as *mut cyber_gear_can_t)
        };

        match comm_type {
            _ if comm_type == cyber_gear_can_communication_type_t_COMMUNICATION_STATUS_REPORT => {
                self.status = unsafe {
                    cyber_gear_parse_motor_status_frame(
                        &mut cybergear_frame as *mut cyber_gear_can_t,
                    )
                };

                self.last_status_frame_rec = now();
                return true;
            }
            _ if comm_type == cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING
                && (id & 0x0F_00_00) >> 16 == 0x02 =>
            {
                let frame_number = ((id & 0xF0_00_00) >> 20) as u8;
                if !self.logging_active {
                    let _ = self.stop_logging().await;
                    return true;
                }

                if frame_number != self.log_parameters.expected_frame_number {
                    self.log_parameters.expected_frame_number = 0;
                } else {
                    match frame_number {
                        0 => {
                            self.log_parameters.last_rcv = now();
                            let mut float_slice = [0u8; 4];
                            float_slice
                                .copy_from_slice(unsafe { &cybergear_frame.can_data.bytes[2..6] });
                            unsafe {
                                *self.log_parameters.param_slot_value_ptr[0] =
                                    f32::from_le_bytes(float_slice);
                            }
                            self.log_parameters.data_buffer[..2]
                                .copy_from_slice(unsafe { &cybergear_frame.can_data.bytes[6..] });

                            if self.log_parameters.param_count > 1 {
                                self.log_parameters.expected_frame_number = 1;
                            }
                        }
                        _ if frame_number > 0 => {
                            self.log_parameters.data_buffer[2..]
                                .copy_from_slice(unsafe { &cybergear_frame.can_data.bytes[0..2] });
                            unsafe {
                                *self.log_parameters.param_slot_value_ptr
                                    [(frame_number as usize * 2) - 1] =
                                    f32::from_le_bytes(self.log_parameters.data_buffer);
                            }

                            if self.log_parameters.param_count
                                > (frame_number as usize * 2 - 1) as u16
                            {
                                let mut float_slice = [0u8; 4];
                                float_slice.copy_from_slice(unsafe {
                                    &cybergear_frame.can_data.bytes[2..6]
                                });
                                unsafe {
                                    *self.log_parameters.param_slot_value_ptr
                                        [frame_number as usize * 2] =
                                        f32::from_le_bytes(float_slice);
                                }

                                self.log_parameters.data_buffer[..2].copy_from_slice(unsafe {
                                    &cybergear_frame.can_data.bytes[6..8]
                                });
                            }

                            self.log_parameters.expected_frame_number = 0;

                            if self.log_parameters.param_count
                                > (frame_number as usize * 2 + 1) as u16
                            {
                                self.log_parameters.expected_frame_number = frame_number + 1;
                            }
                        }
                        _ => {}
                    }
                }
            }
            _ if comm_type == cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID => {
                if self.uuid == 0 {
                    info!("CyberGear UUID: {}", id);
                }
                self.uuid = unsafe { cybergear_frame.can_data.value };
                return true;
            }
            _ => info!(
                "> {:x} : {:x}",
                unsafe { cybergear_frame.can_id.value },
                unsafe { cybergear_frame.can_id.bytes }
            ),
        }

        true
    }

    async fn start_logging(&mut self) -> bool {
        info!("Starting logging");
        let mut frame1 = cyber_gear_can_t::new();

        self.log_parameters.param_count = 3;
        self.log_parameters.param_slot_pid[0] =
            cyber_gear_config_index_t_CONFIG_R_TORQUE_FDB as u16;
        self.log_parameters.param_slot_value_ptr[0] =
            &mut self.log_parameters.torque_fdb as *mut f32;
        self.log_parameters.param_slot_pid[1] = cyber_gear_config_index_t_CONFIG_R_MECH_VEL as u16;
        self.log_parameters.param_slot_value_ptr[1] = &mut self.log_parameters.vel as *mut f32;
        self.log_parameters.param_slot_pid[2] = cyber_gear_config_index_t_CONFIG_R_MECH_POS as u16;
        self.log_parameters.param_slot_value_ptr[2] = &mut self.log_parameters.pos as *mut f32;

        self.init_frame(
            &mut frame1 as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING,
        );
        let bytes = [
            self.log_parameters.freq as u8,
            (self.log_parameters.freq >> 8) as u8,
            0x11,
            0x00,
            0x00,
            0x10,
            0x0E,
            0x00,
        ];

        frame1.can_data.bytes = bytes.clone();

        if self.send_frame(frame1, true).await.is_ok() {
            delay_ms(10).await;
            let mut frame2 = cyber_gear_can_t::new();
            self.init_frame(
                &mut frame2 as *mut cyber_gear_can_t,
                cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING,
            );

            frame2.can_data.bytes = bytes.clone();

            let comm_type_variant = 0x01 + ((self.log_parameters.param_count - 1) << 4) as u8;

            unsafe {
                cyber_gear_set_can_id_communication_type_variant(
                    &mut frame2 as *mut cyber_gear_can_t,
                    comm_type_variant,
                );
            }

            for i in 0..4 {
                self.set_logging_pid(&mut frame2, self.log_parameters.param_slot_pid[i], i);
            }

            if self.send_frame(frame2, true).await.is_ok() {
                delay_ms(10).await;
                let mut frame3 = cyber_gear_can_t::new();

                self.init_frame(
                    &mut frame3 as *mut cyber_gear_can_t,
                    cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING,
                );

                frame3.can_data.bytes = bytes.clone();

                let comm_type_variant = if self.log_parameters.param_count > 4 {
                    0x05 + ((self.log_parameters.param_count - 1) << 4) as u8
                } else {
                    0x02
                };

                unsafe {
                    cyber_gear_set_can_id_communication_type_variant(
                        &mut frame3 as *mut cyber_gear_can_t,
                        comm_type_variant,
                    );
                }

                for i in 4..8 {
                    self.set_logging_pid(&mut frame3, self.log_parameters.param_slot_pid[i], i - 4);
                }

                if self.send_frame(frame3, true).await.is_ok() {
                    self.logging_active = true;
                    return true;
                }
            }
        }

        false
    }

    async fn stop_logging(&mut self) -> bool {
        info!("Stopping logging");
        let mut cyber_gear_frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut cyber_gear_frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_LOGGING,
        );

        unsafe {
            cyber_gear_set_can_id_communication_type_variant(
                &mut cyber_gear_frame as *mut cyber_gear_can_t,
                0x03,
            );
        }

        if self.send_frame(cyber_gear_frame, true).await.is_ok() {
            self.logging_active = false;
            self.log_parameters.param_count = 0;
            return true;
        }

        false
    }

    fn set_logging_pid(&self, frame: &mut cyber_gear_can_t, pid: u16, pos: usize) {
        let [lo, hi] = pid.to_le_bytes();
        unsafe {
            frame.can_data.bytes[pos * 2 + 1] = hi;
            frame.can_data.bytes[pos * 2] = lo;
        }
    }

    async fn send_frame(&mut self, frame: cyber_gear_can_t, log: bool) -> Result<(), GearError<C>> {
        let cybergear_frame = CyberGearFrame::from(frame);
        let hal_frame = self.adapter.to_frame(&cybergear_frame);
        {
            let mut guard = self.can.lock().await;
            let driver = guard.as_mut().unwrap();

            let tx_res: Result<_, C::Error> = block!(driver.transmit(&hal_frame));

            tx_res.map_err(|e| {
                // record when it failed
                self.can_send_error = now();
                self.last_cmd_sent = now();
                GearError::CanWriteError(Error::from(e))
            })?;
        }
        self.last_cmd_sent = now();

        let id_raw = match hal_frame.id() {
            Id::Standard(standard_id) => standard_id.as_raw() as u32,
            Id::Extended(extended_id) => extended_id.as_raw(),
        };

        #[cfg(feature = "debug-log")]
        info!("out message >> {=u32:08x} : {:x}", id_raw, hal_frame.data());
        if log {
            #[cfg(not(feature = "debug-log"))]
            info!("out message >> {=u32:08x} : {:x}", id_raw, hal_frame.data());
        }

        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn get_status(&self) -> cyber_gear_motor_status_t {
        self.status
    }

    fn status_error(&self) -> bool {
        if self.status.has_hall_encode_error != 0 {
            return true;
        }
        if self.status.has_calibration_error != 0 {
            return true;
        }
        if self.status.has_magnetic_encoding_error != 0 {
            return true;
        }
        if self.status.has_undervoltage != 0 {
            return true;
        }
        if self.status.has_over_current != 0 {
            return true;
        }
        if self.status.has_over_temperature != 0 {
            return true;
        }

        false
    }
}

pub(crate) fn now() -> u32 {
    #[cfg(test)]
    {
        return 0;
    }

    #[cfg(not(test))]
    {
        Instant::now().as_millis() as u32
    }
}

pub(crate) async fn delay_ms(_ms: u64) {
    #[cfg(test)]
    {
        return;
    }

    #[cfg(not(test))]
    {
        Timer::after_millis(_ms).await;
    }
}

impl cyber_gear_can_t {
    pub(crate) fn new() -> Self {
        Self {
            can_id: cyber_gear_can_t__bindgen_ty_1 { value: 0 },
            can_data: cyber_gear_can_t__bindgen_ty_2 { bytes: [0u8; 8] },
        }
    }
}

pub fn map_float(x: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

#[allow(dead_code)]
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

/// Cybergear operating modes
#[repr(C)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default, Format)]
pub enum MotorMode {
    #[default]
    None = -1,
    Operation = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default, Format)]
#[allow(dead_code)]
#[repr(u8)]
pub enum MotorState {
    #[default]
    Idle = 0,
    Start,
    Homing,
    HomeFound,
    Wait1,
    MoveZero,
    Wait2,
    Timeout,
    End,
}

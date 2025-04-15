use crate::bindings::{
    cyber_gear_build_motion_control_frame, cyber_gear_build_parameter_write_frame_with_float_value,
    cyber_gear_build_parameter_write_frame_with_int_value, cyber_gear_can_communication_type_t,
    cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
    cyber_gear_can_communication_type_t_COMMUNICATION_ENABLE_DEVICE,
    cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID,
    cyber_gear_can_communication_type_t_COMMUNICATION_MOTION_CONTROL_COMMAND,
    cyber_gear_can_communication_type_t_COMMUNICATION_READ_SINGLE_PARAM,
    cyber_gear_can_communication_type_t_COMMUNICATION_SET_MECHANICAL_ZERO_POSITION,
    cyber_gear_can_communication_type_t_COMMUNICATION_STATUS_REPORT,
    cyber_gear_can_communication_type_t_COMMUNICATION_WRITE_SINGLE_PARAM, cyber_gear_can_init,
    cyber_gear_can_t, cyber_gear_can_t__bindgen_ty_1, cyber_gear_can_t__bindgen_ty_2,
    cyber_gear_config_index_t_CONFIG_R_MECH_POS, cyber_gear_config_index_t_CONFIG_R_MECH_VEL,
    cyber_gear_config_index_t_CONFIG_R_TORQUE_FDB, cyber_gear_get_can_id_communication_type,
    cyber_gear_get_can_id_int_value, cyber_gear_motion_control_t, cyber_gear_motor_status_t,
    cyber_gear_parse_motor_status_frame,
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
    cyber_gear_set_can_id_communication_type, cyber_gear_set_can_id_host_can_id,
    cyber_gear_set_can_id_int_value, cyber_gear_set_can_id_target_can_id,
};
use crate::frame::{CanFrameAdapter, CyberGearFrame};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use defmt::{Format, error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
#[cfg(not(test))]
use embassy_time::{Instant, Timer};
use embedded_can::nb::Can;
use embedded_can::{Frame, Id};

#[derive(Debug, Default)]
#[allow(dead_code)]
pub struct LogParameters {
    pub last_rcv: u32,
    pub param_count: u16,
    pub freq: u16, // Hz
    pub param_slot_pid: [u16; 8],
    pub param_slot_value: [f32; 8],
    pub expected_frame_number: u8,
    pub data_buffer: [u8; 4], // Temporary buffer of half values, 4 bytes long

    pub torque_fdb: f32, // CONFIG_R_TORQUE_FDB
    pub dtc_u: f32,      // CONFIG_R_DTC_U - for arming/enable checking
    pub v_bus: f32,      // CONFIG_R_VBUS_V - input voltage monitoring
    pub vel: f32,        // CONFIG_R_MECH_VEL
    pub pos: f32,        // CONFIG_R_MECH_POS
}

#[derive(Debug, Default)]
#[allow(dead_code)]
pub struct Homing {
    pub homing: u32,
    pub timeout: u32,
    pub current: f32,
    pub position: f32,
    pub state: MotorState,
    pub is_homed: bool,
}

#[derive(Debug, Format)]
pub enum GearError {
    CanWriteError,

    MotorStateConversionError,

    StartLoggingError,
}

#[allow(dead_code)]
pub struct CyberGear<C: Can + 'static, A: CanFrameAdapter<C::Frame>> {
    can: &'static Mutex<CriticalSectionRawMutex, Option<C>>,
    adapter: A,
    /// Host CAN ID
    host_id: u8,
    /// Motor CAN ID
    motor_id: u8,
    uuid: u64,
    pub homing: Homing,
    pub log_parameters: LogParameters,

    pub target_position: f32,

    default_params: Parameters,
    current_params: Parameters,

    pub last_cmd_sent: u32,
    pub last_response: u32,
    pub last_status_frame_req: u32,
    pub last_status_frame_rec: u32,

    pub enabled: bool,
    logging_active: bool,

    can_send_error: u32,
    pub fault_time: u32,

    pub wait_for_response: u32,

    pub current_mode: MotorMode,       // Custom type
    status: cyber_gear_motor_status_t, // Custom type
    pub direction: i32,

    temperature: f32,
    voltage: f32,
    current_position: f32,
}

impl<C, A> CyberGear<C, A>
where
    C: Can + 'static,
    A: CanFrameAdapter<C::Frame>,
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

    pub async fn init(&mut self) -> Result<(), GearError> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID,
        );
        self.send_frame(frame, true).await
    }

    pub fn set_host_id(&mut self, id: u8) {
        self.host_id = id;
    }

    pub async fn set_mode(&mut self, mode: MotorMode) -> Result<(), GearError> {
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
        info!("Change mode to: {:?}", mode);
        Ok(())
    }

    pub fn reverse_direction(&mut self) {
        self.direction = -self.direction;
    }

    pub async fn enable(&mut self) -> Result<(), GearError> {
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

    pub async fn check_enabled(&mut self) -> Result<(), GearError> {
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

    pub async fn clear_fault(&mut self) -> Result<(), GearError> {
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
            self.enable().await?;
        }
        Ok(())
    }

    pub async fn request_status_report(&mut self) -> Result<(), GearError> {
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
        self.send_frame(frame, false).await?;
        self.last_status_frame_req = now();
        Ok(())
    }

    pub async fn disable(&mut self) -> Result<(), GearError> {
        let mut frame = cyber_gear_can_t::new();
        self.init_frame(
            &mut frame as *mut cyber_gear_can_t,
            cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
        );
        self.send_frame(frame, false).await?;
        self.enabled = false;
        // Todo: add stop logging
        Ok(())
    }

    pub async fn set_current_limit(&mut self, limit: f32) -> Result<(), GearError> {
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
        self.send_frame(frame, false).await?;
        self.current_params.current_limit = limit;
        Ok(())
    }

    pub async fn set_speed_limit(&mut self, limit: f32) -> Result<(), GearError> {
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
        self.send_frame(frame, false).await?;
        self.current_params.speed_limit = limit;
        Ok(())
    }

    pub async fn set_torque_limit(&mut self, limit: f32) -> Result<(), GearError> {
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
        self.send_frame(frame, false).await?;
        self.current_params.torque_limit = limit;
        Ok(())
    }

    pub async fn set_ki(&mut self, ki: f32) -> Result<(), GearError> {
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

        self.send_frame(frame, false).await?;
        self.current_params.ki = ki;
        Ok(())
    }

    pub async fn set_kp(&mut self, kp: f32) -> Result<(), GearError> {
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

        self.send_frame(frame, false).await?;
        self.current_params.kp = kp;
        Ok(())
    }

    // No kd value for cyber gear?
    pub fn set_kd(&mut self, kd: f32) -> Result<(), GearError> {
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

    pub async fn set_filter_gain(&mut self, gain: f32) -> Result<(), GearError> {
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

        self.send_frame(frame, false).await?;
        self.current_params.filter_gain = gain;
        Ok(())
    }

    pub async fn run_speed(&mut self, speed: f32) -> Result<(), GearError> {
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
                speed,
            );
        }
        self.send_frame(frame, false).await?;
        self.current_params.speed = speed;
        Ok(())
    }

    pub async fn run_current(&mut self, current: f32) -> Result<(), GearError> {
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
                current,
            );
        }

        self.send_frame(frame, false).await?;
        self.current_params.speed = current;
        Ok(())
    }

    pub async fn run_position_safe(&mut self, position: f32) -> Result<(), GearError> {
        if self.is_ready().await && self.homing.homing == 0 {
            return self.run_position(position).await;
        }

        Ok(())
    }

    pub async fn run_position(&mut self, position: f32) -> Result<(), GearError> {
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
    pub async fn is_ready(&mut self) -> bool {
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

                delay_ms(5u64).await;

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

        self.request_status_report().await.is_ok()
    }

    pub async fn run_op(
        &mut self,
        speed: f32,
        position: f32,
        torque: f32,
        kd: f32,
        kp: f32,
    ) -> Result<(), GearError> {
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

    pub async fn execute(&mut self, hstate_change_mut: &AtomicU32, found_home_mut: &AtomicBool) {
        if now() - self.last_status_frame_rec > 100 {
            let _ = self.request_status_report();
        }

        // homing process:
        // 0. logging for speed and force feedback needs to be enabled
        // 1. set to low speed and low torque
        // 2. run in speed mode with a timeout (5s)
        // 3. watch for velocity below threshold (0.1), give some time in the beginning for the motor to start rotating (1s)
        // 4. if vel < threshold, we found our home pos. save it and
        // 5. move to requested/current target position with medium speed.
        // 6. restore the old current and speed settings
        // error handling. on timeout stop the motor and set the current position as zero, as the motor will otherwise rewind to its starting position on the next position command

        match self.homing.state {
            MotorState::Idle => {}
            MotorState::Start => {
                self.set_current_limit(self.homing.current).await.unwrap();
                self.set_speed_limit(4.0).await.unwrap();
                self.run_speed(4.0).await.unwrap();
                info!("Started homing");
                hstate_change_mut.store(now(), Ordering::Relaxed);
                self.homing.state = MotorState::Homing;
            }
            MotorState::Homing => {
                if now() - self.log_parameters.last_rcv < 100 {
                    if now() - hstate_change_mut.load(Ordering::Relaxed) > 1000
                        && self.log_parameters.vel.abs() < 0.1
                    {
                        self.run_speed(0.0).await.unwrap();
                        info!("Home position found");
                        found_home_mut.store(true, Ordering::Relaxed);

                        self.homing.state = MotorState::Homing;
                    }
                }

                if now() - hstate_change_mut.load(Ordering::Relaxed) > self.homing.timeout {
                    info!("Homing timeout");
                    self.homing.state = MotorState::Timeout;
                }
            }
            MotorState::HomeFound => {
                if self.set_zero().await.is_ok() {
                    hstate_change_mut.store(now(), Ordering::Relaxed);
                    self.homing.state = MotorState::Wait1;
                }
            }
            MotorState::Wait1 => {
                if now() - hstate_change_mut.load(Ordering::Relaxed) > 1500 {
                    self.set_current_limit(4.0).await.unwrap();
                    self.set_speed_limit(4.0).await.unwrap();
                    self.run_position(self.target_position).await.unwrap();
                    self.homing.state = MotorState::MoveZero;
                    hstate_change_mut.store(now(), Ordering::Relaxed);
                }
            }
            MotorState::MoveZero => {
                if now() - hstate_change_mut.load(Ordering::Relaxed) > 1000
                    && self.log_parameters.vel.abs() < 0.1
                {
                    info!("Moved to target position");
                    self.homing.state = MotorState::End;
                    hstate_change_mut.store(now(), Ordering::Relaxed);
                }
            }
            MotorState::Wait2 => {}
            MotorState::Timeout => {
                self.run_speed(0.0).await.unwrap();
                delay_ms(3u64).await;
                self.set_zero().await.unwrap();
                self.homing.state = MotorState::End;
            }
            MotorState::End => {
                self.set_current_limit(self.default_params.current_limit)
                    .await
                    .unwrap();
                self.set_speed_limit(self.default_params.speed_limit)
                    .await
                    .unwrap();
                self.homing.homing = 0;
                self.homing.is_homed = found_home_mut.load(Ordering::Relaxed);
                info!("Ending homing sequence");
            }
        }

        if self.can_send_error != 0 && now() - self.can_send_error > 2000 {
            info!("CAN error, restoring settings");
            self.clear_fault().await.unwrap();
            self.restore_config().await.unwrap();
            self.homing.is_homed = false;
            self.can_send_error = 0;
        }
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
    ) -> Result<(), GearError> {
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
        delay_ms(5u64).await;
        self.set_kp(kp).await?;
        self.set_ki(ki).await?;
        self.set_kd(kd)?;
        self.set_filter_gain(filter_gain).await?;
        delay_ms(50u64).await;
        Ok(())
    }

    pub async fn set_zero(&mut self) -> Result<(), GearError> {
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

    pub async fn restore_config(&mut self) -> Result<(), GearError> {
        self.set_current_limit(self.current_params.current_limit)
            .await?;
        self.set_speed_limit(self.current_params.speed_limit)
            .await?;
        self.set_torque_limit(self.current_params.torque_limit)
            .await?;

        delay_ms(5u64).await;

        self.set_kp(self.current_params.kp).await?;
        self.set_ki(self.current_params.ki).await?;
        self.set_kd(self.current_params.kd)?;
        self.set_filter_gain(self.current_params.filter_gain)
            .await?;
        self.set_mode(self.current_mode).await?;

        delay_ms(5u64).await;

        if self.enabled {
            self.enable().await?;
        }
        Ok(())
    }

    pub fn process_inframe(&mut self, frame: &C::Frame) -> bool {
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
                // According to documentation -> status feedback frame id B0-B7: host id
                // and B8-B15: motor id, all other frames have it switched and cyber_gear_parse_motor_status_frame
                // uses other notion
                self.status.motor_can_id = unsafe {
                    cyber_gear_get_can_id_int_value(
                        &cybergear_frame as *const cyber_gear_can_t,
                        8,
                        8,
                    ) as u8
                };
                self.status.host_can_id = unsafe {
                    cyber_gear_get_can_id_int_value(
                        &cybergear_frame as *const cyber_gear_can_t,
                        0,
                        8,
                    ) as u8
                };

                self.last_response = now();
                return true;
            }
            _ if comm_type == cyber_gear_can_communication_type_t_COMMUNICATION_FETCH_DEVICE_ID => {
                if self.uuid == 0 {
                    info!("CyberGear UUID: {}", id);
                }
                self.uuid = unsafe { cybergear_frame.can_data.value };
                return true;
            }
            _ if comm_type
                == cyber_gear_can_communication_type_t_COMMUNICATION_READ_SINGLE_PARAM =>
            {
                // TODO: Not sure about implementation
                let _param = unsafe {
                    cybergear_frame.can_data.bytes[0] as u16
                        | ((cybergear_frame.can_data.bytes[1] as u16) << 8)
                };
            }
            _ => info!(
                "> {:x} : {:x}",
                unsafe { cybergear_frame.can_id.value },
                unsafe { cybergear_frame.can_id.bytes }
            ),
        }

        true
    }

    pub fn home(&mut self, timeout: u32, current: f32) {
        self.homing.state = MotorState::Homing;
        self.homing.homing = now();
        self.homing.timeout = timeout;
        self.homing.current = current;
    }

    pub async fn start_logging(&mut self) -> bool {
        info!("Starting logging");
        let mut frame1 = cyber_gear_can_t::new();

        self.log_parameters.param_count = 3;
        self.log_parameters.param_slot_pid[0] =
            cyber_gear_config_index_t_CONFIG_R_TORQUE_FDB as u16;
        self.log_parameters.param_slot_value[0] = self.log_parameters.torque_fdb;
        self.log_parameters.param_slot_pid[1] = cyber_gear_config_index_t_CONFIG_R_MECH_VEL as u16;
        self.log_parameters.param_slot_value[1] = self.log_parameters.vel;
        self.log_parameters.param_slot_pid[2] = cyber_gear_config_index_t_CONFIG_R_MECH_POS as u16;
        self.log_parameters.param_slot_value[2] = self.log_parameters.pos;

        // 21 is custom COMMUNICATION_LOGGING type value
        self.init_frame(&mut frame1 as *mut cyber_gear_can_t, 21);
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
            delay_ms(10u64).await;
            let mut frame2 = cyber_gear_can_t::new();
            self.init_frame(&mut frame2 as *mut cyber_gear_can_t, 21);

            frame2.can_data.bytes = bytes.clone();

            let comm_type_variant = 0x01 + ((self.log_parameters.param_count - 1) << 4) as u8;

            unsafe {
                cyber_gear_set_can_id_int_value(
                    &mut frame2 as *mut cyber_gear_can_t,
                    16,
                    8,
                    comm_type_variant as i32,
                );
            }

            for i in 0..4 {
                self.set_logging_pid(&mut frame2, self.log_parameters.param_slot_pid[i], i);
            }

            if self.send_frame(frame2, true).await.is_ok() {
                delay_ms(10u64).await;
                let mut frame3 = cyber_gear_can_t::new();

                self.init_frame(&mut frame3 as *mut cyber_gear_can_t, 21);

                frame3.can_data.bytes = bytes.clone();

                let comm_type_variant = 0x05 + ((self.log_parameters.param_count - 1) << 4) as u8;

                if self.log_parameters.param_count > 4 {
                    unsafe {
                        cyber_gear_set_can_id_int_value(
                            &mut frame3 as *mut cyber_gear_can_t,
                            16,
                            8,
                            comm_type_variant as i32,
                        );
                    }
                } else {
                    unsafe {
                        cyber_gear_set_can_id_int_value(
                            &mut frame3 as *mut cyber_gear_can_t,
                            16,
                            8,
                            0x02,
                        );
                    }
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

    pub async fn stop_logging(&mut self) -> bool {
        info!("Stopping logging");
        let mut cyber_gear_frame = cyber_gear_can_t::new();
        self.init_frame(&mut cyber_gear_frame as *mut cyber_gear_can_t, 21);

        unsafe {
            cyber_gear_set_can_id_int_value(
                &mut cyber_gear_frame as *mut cyber_gear_can_t,
                16,
                8,
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

    pub fn set_logging_pid(&self, frame: &mut cyber_gear_can_t, pid: u16, pos: usize) {
        unsafe {
            frame.can_data.bytes[pos * 2 + 1] = (pid >> 8) as u8;
            frame.can_data.bytes[pos * 2] = pid as u8;
        }
    }

    async fn send_frame(&mut self, frame: cyber_gear_can_t, log: bool) -> Result<(), GearError> {
        let cybergear_frame = CyberGearFrame::from(frame);
        let hal_frame = self.adapter.to_frame(&cybergear_frame);
        let mut lock = self.can.lock().await;
        lock.as_mut()
            .unwrap()
            .transmit(&hal_frame)
            .map_err(|_| GearError::CanWriteError)?;

        self.last_cmd_sent = now();

        let id_raw = match hal_frame.id() {
            Id::Standard(standard_id) => standard_id.as_raw() as u32,
            Id::Extended(extended_id) => extended_id.as_raw(),
        };

        if log {
            info!("> {:x} : {:x}", id_raw, hal_frame.data());
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

pub async fn test_logging_and_time() {
    loop {
        info!("logging is working yay");
        delay_ms(2000).await;
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

#[repr(C)]
#[allow(dead_code)]
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

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
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

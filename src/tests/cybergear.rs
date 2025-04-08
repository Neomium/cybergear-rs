use crate::bindings::{
    cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE, cyber_gear_can_t,
};
use crate::cybergear::CyberGear;
use crate::tests::mocks::{MockAdapter, MockCan, MockFrame};
use core::f32::consts::PI;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
static CAN_MUTEX: Mutex<CriticalSectionRawMutex, Option<MockCan>> =
    Mutex::<CriticalSectionRawMutex, Option<MockCan>>::new(None);

async fn setup_cybergear(id: u8) -> CyberGear<MockCan, MockAdapter> {
    let mock_can = MockCan::new();
    let mock_adapter = MockAdapter::default();

    {
        *(CAN_MUTEX.lock().await) = Some(mock_can);
    }

    let cyber_gear = CyberGear::new(id, &CAN_MUTEX, mock_adapter);
    cyber_gear
}

#[tokio::test]
async fn test_init_frame() {
    let cyber_gear = setup_cybergear(0x3).await;

    let mut frame = cyber_gear_can_t::new();
    cyber_gear.init_frame(
        &mut frame as *mut cyber_gear_can_t,
        cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE,
    );

    let raw_eid = unsafe { frame.can_id.value };

    println!("raw_eid: {:x}", raw_eid);

    assert_eq!(raw_eid & 0xFF, 0x3);
    assert_eq!((raw_eid >> 8) & 0xFF, 0x7D);
    assert_eq!(
        (raw_eid >> 24) & 0x1F,
        cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE
    );
}

#[tokio::test]
async fn test_build_motion_control_frame() {
    let mut mock_can = MockCan::new();
    mock_can.expect_transmit().times(1).returning(move |x| {
        let torque_val = map_float(3.1, -12.0, 12.0, 0.0, 65535.0);
        let torque_bytes = u16::to_le_bytes(torque_val as u16);
        let le_torque = torque_bytes[0] as u16 | (torque_bytes[1] as u16) << 8;
        assert_eq!(x.id as u8 & 0xFF, 0x3);
        assert_eq!((x.id >> 8) as u16, le_torque);
        assert_eq!((x.id >> 24) as u8, 0x1);

        let kp_val = map_float(2.1, 0.0, 500.0, 0.0, 65535.0);
        let bytes4_5 = x.data[4] as u16 | (x.data[5] as u16) << 8;
        assert_eq!(kp_val as u16, bytes4_5);
        Ok(Some(MockFrame::default()))
    });

    {
        *(CAN_MUTEX.lock().await) = Some(mock_can);
    }

    let mock_adapter = MockAdapter::default();

    let mut cyber_gear = CyberGear::new(0x3, &CAN_MUTEX, mock_adapter);

    cyber_gear.run_op(20.0, 3.0, 3.1, 1.1, 2.1).await.unwrap();
}

#[tokio::test]
async fn test_process_inframe() {
    let mock_frame = MockFrame {
        id: 0x0240037D,
        data: [0x00, 0x00, 0x5E, 0x91, 0xFF, 0x9F, 0x00, 0x00],
    };

    let mut cyber_gear = setup_cybergear(0x03).await;

    let res = cyber_gear.process_inframe(&mock_frame);
    let status = cyber_gear.get_status();

    assert_eq!(status.host_can_id, 0x7D);
    assert_eq!(status.motor_can_id, 0x03);
    assert_eq!(status.mode_type, 0x2);

    let speed_val = mock_frame.data[2] as u16 | (mock_frame.data[3] as u16) << 8;
    let speed = map_float(
        rebound_value(speed_val as i16, 32768) as f32,
        -32768.0,
        32768.0,
        -30.0,
        30.0,
    );
    assert_eq!(status.current_speed, speed);

    let torque_val = mock_frame.data[4] as u16 | (mock_frame.data[5] as u16) << 8;
    let torque = map_float(
        rebound_value(torque_val as i16, 32768) as f32,
        -32768.0,
        32768.0,
        -12.0,
        12.0,
    );
    assert_eq!(status.current_torque, torque);

    assert!(res);
}

fn map_float(x: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
}

fn rebound_value(input: i16, max: i32) -> i16 {
    if input == 0 {
        return 0;
    }

    let abs_val = input.abs();
    let signed_val = abs_val / input;
    let out = (abs_val as i32 - max) * signed_val as i32;
    out as i16
}

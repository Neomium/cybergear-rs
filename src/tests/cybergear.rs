use crate::bindings::{
    cyber_gear_can_communication_type_t_COMMUNICATION_DISABLE_DEVICE, cyber_gear_can_t,
    cyber_gear_can_t__bindgen_ty_1, cyber_gear_can_t__bindgen_ty_2,
    cyber_gear_get_can_id_target_id,
};
use crate::cybergear::CyberGear;
use crate::tests::mocks::MockDev;
#[test]
fn test_bindings() {
    let cybergear_frame = cyber_gear_can_t {
        can_id: cyber_gear_can_t__bindgen_ty_1 { value: 1 },
        can_data: cyber_gear_can_t__bindgen_ty_2 { bytes: [0x1; 8] },
    };

    unsafe {
        let x: u8 = cyber_gear_get_can_id_target_id(&cybergear_frame) as u8;
        println!("{:?}", x);
        assert_ne!(x, 0u8);
    }
}

#[test]
fn test_init_frame() {
    let mock_can = MockDev::new();
    let cyber_gear = CyberGear::new(0x3, mock_can);

    let mut frame = cyber_gear_can_t {
        can_id: cyber_gear_can_t__bindgen_ty_1 { value: 1 },
        can_data: cyber_gear_can_t__bindgen_ty_2 { value: 0 },
    };
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

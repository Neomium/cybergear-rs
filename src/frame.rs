use crate::bindings::{
    cyber_gear_can_t, cyber_gear_can_t__bindgen_ty_1, cyber_gear_can_t__bindgen_ty_2,
};
use defmt::Format;
use embedded_can::{ExtendedId, Frame, Id};

#[derive(Debug, Default, Format)]
pub struct CyberGearFrame {
    pub id: u32,
    pub data: [u8; 8],
}

impl Frame for CyberGearFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let eid = id.into();
        let raw_id = match eid {
            Id::Extended(eid) => Some(eid.as_raw()),
            Id::Standard(_) => None,
        };

        let mut buffer = [0u8; 8];
        buffer[..data.len()].copy_from_slice(&data[..data.len()]);
        Some(Self {
            id: raw_id?,
            data: buffer,
        })
    }

    fn new_remote(id: impl Into<Id>, _dlc: usize) -> Option<Self> {
        Self::new(id, &[0u8; 8])
    }

    fn is_extended(&self) -> bool {
        true
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> Id {
        Id::Extended(ExtendedId::new(self.id).unwrap())
    }

    fn dlc(&self) -> usize {
        8
    }

    fn data(&self) -> &[u8] {
        &self.data
    }
}

impl From<cyber_gear_can_t> for CyberGearFrame {
    fn from(value: cyber_gear_can_t) -> Self {
        Self {
            id: unsafe { value.can_id.value },
            data: unsafe { value.can_data.bytes },
        }
    }
}

impl From<CyberGearFrame> for cyber_gear_can_t {
    fn from(value: CyberGearFrame) -> Self {
        Self {
            can_id: cyber_gear_can_t__bindgen_ty_1 { value: value.id },
            can_data: cyber_gear_can_t__bindgen_ty_2 { bytes: value.data },
        }
    }
}

pub trait CanFrameAdapter<F: Frame> {
    fn to_frame(&self, frame: &CyberGearFrame) -> F;
    fn from_frame(&self, frame: &F) -> CyberGearFrame;
}

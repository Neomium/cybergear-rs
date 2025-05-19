use crate::frame::{CanFrameAdapter, CyberGearFrame};
use embedded_can::nb::Can;
use embedded_can::{ErrorKind, ExtendedId, Frame, Id};
use mockall::mock;

#[derive(Debug, Default)]
pub struct MockFrame {
    pub(crate) id: u32,
    pub(crate) data: [u8; 8],
}

impl Frame for MockFrame {
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

#[derive(Default)]
pub(crate) struct MockAdapter;

impl CanFrameAdapter<MockFrame> for MockAdapter {
    fn to_frame(&self, frame: &CyberGearFrame) -> MockFrame {
        MockFrame {
            id: frame.id,
            data: frame.data,
        }
    }

    fn from_frame(&self, frame: &MockFrame) -> CyberGearFrame {
        CyberGearFrame {
            id: frame.id,
            data: frame.data,
        }
    }
}

#[derive(Debug)]
pub enum MockCanError {}

impl embedded_can::Error for MockCanError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

mock! {
    #[derive(Debug)]
    pub Can {}

    impl Can for Can {
        type Frame = MockFrame;
        type Error = MockCanError;

        fn transmit(&mut self, frame: &MockFrame) -> nb::Result<Option<MockFrame>, MockCanError>;

        fn receive(&mut self) -> nb::Result<MockFrame, MockCanError>;
    }
}

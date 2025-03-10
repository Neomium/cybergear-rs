use embedded_can::nb::Can;
use embedded_can::{ErrorKind, ExtendedId, Frame, Id};
use mockall::mock;

#[derive(Debug, Default)]
pub struct MockFrame {
    id: u32,
    data: [u8; 8],
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

    fn is_standard(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn is_data_frame(&self) -> bool {
        true
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

#[derive(Debug)]
pub enum MockCanError {}

impl embedded_can::Error for MockCanError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

mock! {
    pub Dev {}

    impl Can for Dev {
        type Frame = MockFrame;
        type Error = MockCanError;

        fn transmit(&mut self, frame: &MockFrame) -> nb::Result<Option<MockFrame>, MockCanError>;

        fn receive(&mut self) -> nb::Result<MockFrame, MockCanError>;
    }
}

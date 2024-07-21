use crate::Args;
use serialport::SerialPort;
pub struct Serial {
    port: Box<dyn SerialPort>,
}

impl Serial {
    pub fn establish_connection(args: &Args) -> Result<Self, Box<dyn std::error::Error>> {
        let port = serialport::new(&args.serial, args.baud_rate)
            .timeout(std::time::Duration::from_millis(10))
            .open()?;
        Ok(Self { port })
    }
}

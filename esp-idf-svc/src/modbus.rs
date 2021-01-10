use anyhow::*;

pub trait Modbus {
    fn read_discrete(&mut self, coil: bool, address: u16, registers: &mut [bool]) -> Result<()>;
    fn write_discrete(&mut self, address: u16, registers: &[bool]) -> Result<()>;

    fn read(&mut self, holding: bool, address: u16, registers: &mut [u16]) -> Result<()>;
    fn write(&mut self, address: u16, registers: &[u16]) -> Result<()>;
}

pub trait Slave {
    fn process() -> Result<()>;
}

// struct ModbusRTU<IO: io::Read + io::Write>(IO);

// impl<IO: io::Read + io::Write> ModbusRTU<IO> {
//     fn write(&mut self, data: &[u8], crc: &mut u16) -> Result<()> {
//         self.0.write(data)?;

//         *crc = update_crc(data, *crc);

//         Ok(())
//     }

//     fn write8(&mut self, data: u8, crc: &mut u16) -> Result<()> {
//         self.write(&u8_to_array(data), &mut crc)?;

//         Ok(())
//     }

//     fn write16(&mut self, data: u16, crc: &mut u16) -> Result<()> {
//         self.write(&u16_to_array(data), &mut crc)?;

//         Ok(())
//     }

//     fn read(&mut self, buf: &mut [u8], crc: &mut u16) -> Result<()> {
//         self.0.read_exact(&mut buf)?;

//         *crc = update_crc(buf, *crc);

//         Ok(())
//     }

//     fn read8(&mut self, crc: &mut u16) -> Result<u8> {
//         let mut buf: [u8; 1] = [0; 1];

//         self.read(&mut buf, crc)?;

//         Ok(buf[0])
//     }

//     fn read16(&mut self, crc: &mut u16) -> Result<u16> {
//         let mut buf: [u8; 2] = [0; 2];

//         self.read(&mut buf, crc)?;

//         Ok(((buf[0] as u16) << 8) | (buf[1] as u16))
//     }
// }

// fn u8_to_array(data: u8) -> [u8; 1] {
//     [data; 1]
// }

// fn u16_to_array(data: u16) -> [u8; 2] {
//     [((data >> 8) & 0xff) as u8, (data & 0xff) as u8]
// }

// fn initial_crc() -> u16 {
//     0xffff
// }

// fn update_crc(data: &[u8], crc: u16) -> u16 {
//     for b in data {
//         crc ^= *b as u16;

//         for _ in 0..8 {
//             if (crc & 0x0001) != 0 {
//                 crc >>= 1;
//                 crc ^= 0xa001;
//             } else {
//             crc >>= 1;
//             }
//         }
//     }

//     crc
// }

// impl<IO: io::Read + io::Write> Modbus for ModbusRTU<IO> {
//     fn read_discrete(&mut self, coil: bool, address: u16, registers: &mut [bool]) -> Result<()> {
//         let mut crc: u16 = initial_crc();

//         self.write8(if coil {1} else {2}, &mut crc)?;
//         self.write16(address, &mut crc)?;
//         self.write16(registers.len() as u16, &mut crc)?;

//         self.0.write(&u16_to_array(crc))?;

//         self.0.flush()?;

//         thread::sleep(Duration::from_millis(150));
        
//         crc = initial_crc();

//         let command = self.read8(&mut crc)?;

//         Ok(())
//     }

//     fn write_discrete(&mut self, address: u16, registers: &[bool]) -> Result<()> {
//         Ok(())
//     }

//     fn read(&mut self, holding: bool, address: u16, registers: &mut [u16]) -> Result<()> {
//         Ok(())
//     }

//     fn write(&mut self, address: u16, registers: &[u16]) -> Result<()> {
//         Ok(())
//     }
// }

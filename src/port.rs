//! 串口传输层：真实 Dynamixel 总线 + 仿真实现（同一 trait）。

use crate::dxl;
use serialport::SerialPort;
use std::collections::BTreeMap;
use std::io::{ErrorKind, Read, Write};
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum IoError {
    Port(String),
    Timeout { expected: Vec<u8>, got: Vec<u8> },
    Crc,
    Device { id: u8, error: u8 },
    Busy,
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IoError::Port(s) => write!(f, "serial error: {s}"),
            IoError::Timeout { expected, got } => {
                write!(f, "timeout: expected {expected:?}, got {got:?}")
            }
            IoError::Crc => write!(f, "crc mismatch"),
            IoError::Device { id, error } => write!(f, "motor {id} hardware error 0x{error:02X}"),
            IoError::Busy => write!(f, "port busy"),
        }
    }
}

impl std::error::Error for IoError {}

pub type IoResult<T> = Result<T, IoError>;

/// 一条串口总线（一个机械臂）的读写接口。
pub trait ArmIo: Send {
    /// 同步读多个舵机的寄存器。缺失的舵机不出现在结果里（总线上个别掉线不算整臂失败）。
    fn sync_read(
        &mut self,
        addr: u16,
        len: u16,
        ids: &[u8],
        timeout: Duration,
    ) -> IoResult<BTreeMap<u8, Vec<u8>>>;

    /// 单个舵机写寄存器（等它回状态包确认）。
    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()>;

    /// 同步写多个舵机（广播式，舵机不回包）。
    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()>;

    /// ping 一个舵机，返回 model number。
    fn ping(&mut self, id: u8) -> IoResult<u16>;

    fn describe(&self) -> String;
}

// ---------------------------------------------------------------- 真实串口

pub struct DxlPort {
    path: String,
    baud: u32,
    port: Box<dyn SerialPort>,
    buf: Vec<u8>,
}

impl DxlPort {
    pub fn open(path: &str, baud: u32) -> IoResult<Self> {
        let port = serialport::new(path, baud)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| IoError::Port(format!("open {path} @ {baud}: {e}")))?;
        Ok(Self { path: path.to_string(), baud, port, buf: Vec::with_capacity(256) })
    }

    fn drain(&mut self) {
        let _ = self.port.clear(serialport::ClearBuffer::Input);
        self.buf.clear();
    }

    /// 读一个状态包。`want` 为 Some(id) 时跳过其它 id 的包。
    fn read_status(&mut self, want: Option<u8>, deadline: Instant) -> IoResult<dxl::Status> {
        loop {
            if !self.buf.is_empty() {
                match dxl::decode(&self.buf) {
                    Ok((st, used)) => {
                        self.buf.drain(..used);
                        if st.error != 0 {
                            return Err(IoError::Device { id: st.id, error: st.error });
                        }
                        if want.is_none() || want == Some(st.id) {
                            return Ok(st);
                        }
                        continue;
                    }
                    Err(dxl::DecodeError::BadHeader) | Err(dxl::DecodeError::Corrupt) => {
                        self.buf.remove(0);
                        continue;
                    }
                    Err(dxl::DecodeError::NeedMore) => {}
                }
            }
            if Instant::now() >= deadline {
                return Err(IoError::Timeout { expected: want.map(|i| vec![i]).unwrap_or_default(), got: Vec::new() });
            }
            let mut chunk = [0u8; 512];
            match self.port.read(&mut chunk) {
                Ok(0) => {}
                Ok(n) => self.buf.extend_from_slice(&chunk[..n]),
                Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::Interrupted) => {}
                Err(e) => return Err(IoError::Port(format!("read {path}: {e}", path = self.path))),
            }
        }
    }
}

impl ArmIo for DxlPort {
    fn sync_read(
        &mut self,
        addr: u16,
        len: u16,
        ids: &[u8],
        timeout: Duration,
    ) -> IoResult<BTreeMap<u8, Vec<u8>>> {
        self.drain();
        let pkt = dxl::sync_read_packet(dxl::BROADCAST_ID, addr, len, ids);
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
        let deadline = Instant::now() + timeout;
        let mut out = BTreeMap::new();
        while out.len() < ids.len() && Instant::now() < deadline {
            match self.read_status(None, deadline) {
                Ok(st) if st.data.len() >= len as usize => {
                    out.insert(st.id, st.data);
                }
                Ok(_) => {}
                Err(IoError::Timeout { .. }) => break,
                Err(IoError::Device { id, error }) => {
                    return Err(IoError::Device { id, error });
                }
                Err(e) => return Err(e),
            }
        }
        if out.is_empty() {
            return Err(IoError::Timeout { expected: ids.to_vec(), got: Vec::new() });
        }
        Ok(out)
    }

    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()> {
        self.drain();
        let pkt = dxl::write_packet(id, addr, data);
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
        let st = self.read_status(Some(id), Instant::now() + Duration::from_millis(50))?;
        if st.id != id {
            return Err(IoError::Timeout { expected: vec![id], got: vec![st.id] });
        }
        Ok(())
    }

    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()> {
        self.drain();
        let mut pkt = dxl::sync_write_packet(dxl::BROADCAST_ID, addr, values.first().map(|v| v.1.len()).unwrap_or(0) as u16, values);
        // 同步写不需要应答：写完清空输入缓冲即可
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
        pkt.clear();
        std::thread::sleep(Duration::from_micros(300));
        let _ = self.port.clear(serialport::ClearBuffer::Input);
        self.buf.clear();
        Ok(())
    }

    fn ping(&mut self, id: u8) -> IoResult<u16> {
        self.drain();
        let pkt = dxl::ping_packet(id);
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
        let st = self.read_status(Some(id), Instant::now() + Duration::from_millis(50))?;
        Ok((st.data.first().copied().unwrap_or(0) as u16)
            | ((st.data.get(1).copied().unwrap_or(0) as u16) << 8))
    }

    fn describe(&self) -> String {
        format!("{}@{}", self.path, self.baud)
    }
}

// ---------------------------------------------------------------- 仿真

/// 没有硬件时用：位置按正弦缓慢变化，收到 goal 写入后跟随。
pub struct SimPort {
    positions: BTreeMap<u8, i64>,
    goals: BTreeMap<u8, i64>,
    start: Instant,
    pub model: u16,
}

impl SimPort {
    pub fn new(ids: &[u8]) -> Self {
        let mut positions = BTreeMap::new();
        for (i, id) in ids.iter().enumerate() {
            positions.insert(*id, 2048 + (i as i64) * 40);
        }
        Self { positions, goals: BTreeMap::new(), start: Instant::now(), model: 1060 }
    }

    fn tick(&self) -> BTreeMap<u8, i64> {
        let t = self.start.elapsed().as_secs_f64();
        let mut out = BTreeMap::new();
        for (id, base) in &self.positions {
            let goal = self.goals.get(id).copied().unwrap_or(*base);
            let v = goal + ((t * (0.7 + (*id as f64) * 0.11)).sin() * 120.0) as i64;
            out.insert(*id, v);
        }
        out
    }
}

impl ArmIo for SimPort {
    fn sync_read(
        &mut self,
        addr: u16,
        len: u16,
        ids: &[u8],
        _timeout: Duration,
    ) -> IoResult<BTreeMap<u8, Vec<u8>>> {
        if addr == 144 {
            // 仿真电压/温度：120 = 12.0V，35℃
            let mut out = BTreeMap::new();
            for id in ids {
                out.insert(*id, vec![120, 0, 35]);
            }
            return Ok(out);
        }
        let tick = self.tick();
        let mut out = BTreeMap::new();
        for id in ids {
            if let Some(v) = tick.get(id) {
                out.insert(*id, dxl::le_bytes(*v, len as usize));
            }
        }
        Ok(out)
    }

    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()> {
        if addr == dxl::ADDR_GOAL_POSITION {
            self.goals.insert(id, dxl::le_int(data));
        }
        Ok(())
    }

    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()> {
        if addr == dxl::ADDR_GOAL_POSITION {
            for (id, data) in values {
                self.goals.insert(*id, dxl::le_int(data));
            }
        }
        Ok(())
    }

    fn ping(&mut self, _id: u8) -> IoResult<u16> {
        Ok(self.model)
    }

    fn describe(&self) -> String {
        "sim".to_string()
    }
}

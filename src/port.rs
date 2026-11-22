//! 串口传输层：真实 Dynamixel 总线 + 仿真实现（同一 trait），以及串口句柄生命周期（Link）。
//!
//! 关键安全设计（见 `docs/SAFETY-DESIGN.md`）：
//! - `sync_read` 只有**传输层**失败才返回 Err；单个舵机的硬件 error 字节记入
//!   [`SyncReadResult::errors`]，不再让整条臂判死。
//! - [`Link::open_with`] 在打开新串口前**必须先 drop 旧 handle**：串口是独占资源
//!   （TIOCEXCL），旧 fd 未释放时再次 open 会永远 `Device or resource busy`。
//!   v0.1.0 的 fd 泄漏事故就是这一步缺失导致的。

use crate::dxl;
use serialport::SerialPort;
use std::collections::{BTreeMap, HashSet};
use std::io::{ErrorKind, Read, Write};
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum IoError {
    Port(String),
    Timeout { expected: Vec<u8>, got: Vec<u8> },
    Crc,
    Device { id: u8, error: u8 },
    /// 软限位拒绝（mode=reject）
    SoftLimit { motor: u8, value: i64, min: Option<i64>, max: Option<i64> },
    /// 当前传输层不支持（例如对真机做 sim 注入）
    Unsupported,
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
            IoError::SoftLimit { motor, value, min, max } => write!(
                f,
                "motor {motor} goal {value} 超出软限位 [{:?}..{:?}]，已拒绝（soft_limit_mode=reject）",
                min, max
            ),
            IoError::Unsupported => write!(f, "unsupported operation for this transport"),
            IoError::Busy => write!(f, "port busy"),
        }
    }
}

impl std::error::Error for IoError {}

impl IoError {
    /// 是否是传输层（整条总线）错误；单电机 Device/SoftLimit/Unsupported 不算。
    pub fn is_transport(&self) -> bool {
        matches!(
            self,
            IoError::Port(_) | IoError::Timeout { .. } | IoError::Crc | IoError::Busy
        )
    }
}

pub type IoResult<T> = Result<T, IoError>;

/// 一次 sync_read 的结果：按电机拆开数据与硬件错误，个别舵机异常不拖垮整次读取。
#[derive(Debug, Clone, Default)]
pub struct SyncReadResult {
    pub data: BTreeMap<u8, Vec<u8>>,
    pub errors: BTreeMap<u8, u8>,
}

impl SyncReadResult {
    pub fn is_empty(&self) -> bool {
        self.data.is_empty() && self.errors.is_empty()
    }

    /// 有应答的电机数（有数据或有错误状态包）
    pub fn responded(&self) -> usize {
        self.data.len() + self.errors.len()
    }
}

/// 一条串口总线（一个机械臂）的读写接口。
pub trait ArmIo: Send {
    /// 同步读多个舵机的寄存器。返回值按电机拆分：`data` 是正常数据，
    /// `errors` 是带硬件 error 位的状态包；完全没响应的电机两边都不出现。
    fn sync_read(
        &mut self,
        addr: u16,
        len: u16,
        ids: &[u8],
        timeout: Duration,
    ) -> IoResult<SyncReadResult>;

    /// 单个舵机写寄存器（等它回状态包确认）。
    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()>;

    /// 同步写多个舵机（广播式，舵机不回包）。
    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()>;

    /// ping 一个舵机，返回 model number。
    fn ping(&mut self, id: u8) -> IoResult<u16>;

    /// Dynamixel Reboot（0x08）：清除 latched 硬件错误。`id=0xFE` 为广播。
    fn reboot(&mut self, id: u8) -> IoResult<()>;

    /// 仿真故障注入（真串口不支持）。
    fn inject_fault(&mut self, kind: &str, ids: &[u8], enable: bool) -> IoResult<()> {
        let _ = (kind, ids, enable);
        Err(IoError::Unsupported)
    }

    fn describe(&self) -> String;
}

// ---------------------------------------------------------------- 句柄生命周期

/// 串口句柄的唯一持有者：**open 前先 close（drop 旧 handle 释放 fd）**。
///
/// v0.1.0 的事故：读失败后 `io` 仍为 Some，重连直接再 open → TIOCEXCL 让新 open
/// 永远 EBUSY，旧 fd 永不释放。这里把"先关后开"固化成唯一入口，并用单元测试锁死。
pub struct Link {
    io: Option<Box<dyn ArmIo>>,
}

impl Default for Link {
    fn default() -> Self {
        Self::new()
    }
}

impl Link {
    pub fn new() -> Self {
        Self { io: None }
    }

    pub fn is_open(&self) -> bool {
        self.io.is_some()
    }

    pub fn io(&mut self) -> Option<&mut (dyn ArmIo + 'static)> {
        match self.io.as_mut() {
            Some(b) => Some(b.as_mut()),
            None => None,
        }
    }

    /// 关闭串口（drop handle → 释放 fd）。返回是否真的关了一个。
    pub fn close(&mut self) -> bool {
        self.io.take().is_some()
    }

    /// 先关闭旧句柄，再打开新句柄。打开失败时保证 `self.io == None`（不会把旧 fd 留在手里）。
    pub fn open_with<F>(&mut self, open: F) -> IoResult<()>
    where
        F: FnOnce() -> IoResult<Box<dyn ArmIo>>,
    {
        // 关键顺序：旧 handle 必须在新 open 之前 drop。用 take() 显式表达，
        // 不依赖 `self.io = Some(new)` 覆盖时才 drop 的隐式行为。
        self.close();
        let io = open()?;
        self.io = Some(io);
        Ok(())
    }
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
    ///
    /// 注意：**带 error 位的状态包也照常返回**——由调用方决定是"单电机错误"
    /// （sync_read 记入 errors 继续）还是"写失败"（write_register/ping 转 Err(Device)）。
    fn read_status(&mut self, want: Option<u8>, deadline: Instant) -> IoResult<dxl::Status> {
        loop {
            if !self.buf.is_empty() {
                match dxl::decode(&self.buf) {
                    Ok((st, used)) => {
                        self.buf.drain(..used);
                        if let Some(w) = want {
                            if st.id != w {
                                continue;
                            }
                        }
                        return Ok(st);
                    }
                    Err(dxl::DecodeError::BadHeader) | Err(dxl::DecodeError::Corrupt) => {
                        self.buf.remove(0);
                        continue;
                    }
                    Err(dxl::DecodeError::NeedMore) => {}
                }
            }
            if Instant::now() >= deadline {
                return Err(IoError::Timeout {
                    expected: want.map(|i| vec![i]).unwrap_or_default(),
                    got: Vec::new(),
                });
            }
            let mut chunk = [0u8; 512];
            match self.port.read(&mut chunk) {
                Ok(0) => {}
                Ok(n) => self.buf.extend_from_slice(&chunk[..n]),
                Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::Interrupted | ErrorKind::WouldBlock) => {}
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
    ) -> IoResult<SyncReadResult> {
        self.drain();
        let pkt = dxl::sync_read_packet(dxl::BROADCAST_ID, addr, len, ids);
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
        let deadline = Instant::now() + timeout;
        let mut out = SyncReadResult::default();
        let mut seen: HashSet<u8> = HashSet::new();
        while seen.len() < ids.len() && Instant::now() < deadline {
            match self.read_status(None, deadline) {
                Ok(st) => {
                    if !ids.contains(&st.id) || !seen.insert(st.id) {
                        continue;
                    }
                    if st.error != 0 {
                        out.errors.insert(st.id, st.error);
                    }
                    if st.data.len() >= len as usize {
                        out.data.insert(st.id, st.data);
                    }
                }
                Err(IoError::Timeout { .. }) => break,
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
        if st.error != 0 {
            return Err(IoError::Device { id: st.id, error: st.error });
        }
        Ok(())
    }

    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()> {
        self.drain();
        let len = values.first().map(|v| v.1.len()).unwrap_or(0) as u16;
        let pkt = dxl::sync_write_packet(dxl::BROADCAST_ID, addr, len, values);
        // 同步写不需要应答：写完清空输入缓冲即可
        self.port.write_all(&pkt).map_err(|e| IoError::Port(format!("write: {e}")))?;
        self.port.flush().ok();
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
        if st.error != 0 {
            return Err(IoError::Device { id: st.id, error: st.error });
        }
        Ok((st.data.first().copied().unwrap_or(0) as u16)
            | ((st.data.get(1).copied().unwrap_or(0) as u16) << 8))
    }

    fn reboot(&mut self, id: u8) -> IoResult<()> {
        self.drain();
        let pkt = dxl::reboot_packet(id);
        self.port
            .write_all(&pkt)
            .map_err(|e| IoError::Port(format!("write reboot: {e}")))?;
        self.port.flush().ok();
        if id != dxl::BROADCAST_ID {
            // 单机 Reboot：舵机通常先回状态包再重启；回包带 error 位/超时都正常，忽略之。
            // 只有串口本身读写失败（传输层）才算 reboot 失败。
            match self.read_status(Some(id), Instant::now() + Duration::from_millis(50)) {
                Err(IoError::Port(e)) => return Err(IoError::Port(e)),
                _ => {}
            }
        }
        Ok(())
    }

    fn describe(&self) -> String {
        format!("{}@{}", self.path, self.baud)
    }
}

// ---------------------------------------------------------------- 仿真

/// 一个仿真舵机的状态。默认 torque ON（仿真便利；真机上电 torque=OFF）。
struct SimMotor {
    position: i64,
    goal: i64,
    torque: bool,
    min_limit: i64,
    max_limit: i64,
    hardware_error: u8,
    stall: bool,
    no_response: bool,
    velocity: i64,
}

impl SimMotor {
    fn new(position: i64) -> Self {
        Self {
            position,
            goal: position,
            torque: true,
            min_limit: 0,
            max_limit: 4095,
            hardware_error: 0,
            stall: false,
            no_response: false,
            velocity: 0,
        }
    }
}

/// 没有硬件时用：位置以固定速度跟随 goal；支持注入堵转 / 硬件错误 / 掉线，
/// 以及 Reboot 清除 latched 错误，用于无硬件验证安全层。
pub struct SimPort {
    motors: BTreeMap<u8, SimMotor>,
    last_tick: Instant,
    /// 注入的整条总线传输错误（模拟 USB 掉线/串口读失败，用于验证 fd 释放路径）
    transport_error: bool,
    pub model: u16,
}

/// 仿真舵机满速（counts/s）；1 velocity 单位 ≈ 15.6 counts/s（X 系列 0.229rpm）
const SIM_SPEED_COUNTS_PER_SEC: f64 = 1500.0;
const COUNTS_PER_VELOCITY_UNIT: f64 = 15.6;

impl SimPort {
    pub fn new(ids: &[u8]) -> Self {
        let mut motors = BTreeMap::new();
        for (i, id) in ids.iter().enumerate() {
            motors.insert(*id, SimMotor::new(2048 + (i as i64) * 40));
        }
        Self { motors, last_tick: Instant::now(), transport_error: false, model: 1060 }
    }

    /// 推进仿真时间：torque ON 时位置向 goal 移动，堵转时冻结。
    fn tick(&mut self) {
        let now = Instant::now();
        let dt = now.saturating_duration_since(self.last_tick).as_secs_f64().clamp(0.0, 0.5);
        self.last_tick = now;
        for m in self.motors.values_mut() {
            if m.stall || m.hardware_error != 0 || m.no_response || !m.torque {
                m.velocity = 0;
                continue;
            }
            let diff = m.goal - m.position;
            if diff == 0 {
                m.velocity = 0;
                continue;
            }
            let max_step = (SIM_SPEED_COUNTS_PER_SEC * dt) as i64;
            let step = diff.clamp(-max_step.max(1), max_step.max(1));
            m.position += step;
            m.velocity = if dt > 0.0 {
                (step as f64 / dt / COUNTS_PER_VELOCITY_UNIT).round() as i64
            } else {
                0
            };
        }
    }

    fn read_data(m: &SimMotor, addr: u16, len: u16) -> Option<Vec<u8>> {
        match (addr, len) {
            (dxl::ADDR_MAX_POSITION_LIMIT, 4) => {
                let mut v = dxl::le_bytes(m.max_limit, 2);
                v.extend_from_slice(&dxl::le_bytes(m.min_limit, 2));
                Some(v)
            }
            (dxl::ADDR_MAX_POSITION_LIMIT, 2) => Some(dxl::le_bytes(m.max_limit, 2)),
            (dxl::ADDR_MIN_POSITION_LIMIT, 2) => Some(dxl::le_bytes(m.min_limit, 2)),
            (dxl::ADDR_OPERATING_MODE, 1) => Some(dxl::le_bytes(3, 1)), // 仿真默认 POSITION(3)
            (dxl::ADDR_TORQUE_ENABLE, 1) => Some(dxl::le_bytes(m.torque as i64, 1)),
            (dxl::ADDR_HARDWARE_ERROR_STATUS, 1) => Some(dxl::le_bytes(m.hardware_error as i64, 1)),
            (dxl::ADDR_GOAL_POSITION, 4) => Some(dxl::le_bytes(m.goal, 4)),
            (dxl::ADDR_PRESENT_VELOCITY, 8) => {
                let mut v = dxl::le_bytes(m.velocity, 4);
                v.extend_from_slice(&dxl::le_bytes(m.position, 4));
                Some(v)
            }
            (dxl::ADDR_PRESENT_VELOCITY, 4) => Some(dxl::le_bytes(m.velocity, 4)),
            (dxl::ADDR_PRESENT_POSITION, 4) => Some(dxl::le_bytes(m.position, 4)),
            (dxl::ADDR_PRESENT_INPUT_VOLTAGE, 3) => Some(vec![120, 0, 35]), // 12.0V, 35℃
            (dxl::ADDR_PRESENT_TEMPERATURE, 1) => Some(vec![35]),
            (_, l) => Some(vec![0u8; l as usize]),
        }
    }

    /// 故障注入/reboot 的目标电机：空列表或广播 id 都表示全部电机。
    fn targets(&self, ids: &[u8]) -> Vec<u8> {
        if ids.is_empty() || ids.contains(&dxl::BROADCAST_ID) {
            self.motors.keys().copied().collect()
        } else {
            ids.to_vec()
        }
    }
}

impl ArmIo for SimPort {
    fn sync_read(
        &mut self,
        addr: u16,
        len: u16,
        ids: &[u8],
        _timeout: Duration,
    ) -> IoResult<SyncReadResult> {
        if self.transport_error {
            return Err(IoError::Port("sim: injected transport error".into()));
        }
        self.tick();
        let mut out = SyncReadResult::default();
        for id in ids {
            let Some(m) = self.motors.get(id) else { continue };
            if m.no_response {
                continue;
            }
            if m.hardware_error != 0 {
                // latched 硬件错误：状态包只带 error（与真机一致，不给数据）；
                // 例外：读 Hardware_Error_Status(70) 用于 clear_error 诊断。
                out.errors.insert(*id, m.hardware_error);
                if addr == dxl::ADDR_HARDWARE_ERROR_STATUS {
                    if let Some(data) = Self::read_data(m, addr, len) {
                        out.data.insert(*id, data);
                    }
                }
                continue;
            }
            if let Some(data) = Self::read_data(m, addr, len) {
                if data.len() >= len as usize {
                    out.data.insert(*id, data);
                }
            }
        }
        Ok(out)
    }

    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()> {
        if self.transport_error {
            return Err(IoError::Port("sim: injected transport error".into()));
        }
        self.tick();
        let m = self
            .motors
            .get_mut(&id)
            .ok_or(IoError::Timeout { expected: vec![id], got: Vec::new() })?;
        if m.no_response {
            return Err(IoError::Timeout { expected: vec![id], got: Vec::new() });
        }
        if m.hardware_error != 0 {
            return Err(IoError::Device { id, error: m.hardware_error });
        }
        Self::apply_write(m, addr, data);
        Ok(())
    }

    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()> {
        if self.transport_error {
            return Err(IoError::Port("sim: injected transport error".into()));
        }
        self.tick();
        for (id, data) in values {
            if let Some(m) = self.motors.get_mut(id) {
                if m.no_response || m.hardware_error != 0 {
                    continue;
                }
                Self::apply_write(m, addr, data);
            }
        }
        Ok(())
    }

    fn ping(&mut self, id: u8) -> IoResult<u16> {
        if self.transport_error {
            return Err(IoError::Port("sim: injected transport error".into()));
        }
        let Some(m) = self.motors.get(&id) else {
            return Err(IoError::Timeout { expected: vec![id], got: Vec::new() });
        };
        if m.no_response {
            return Err(IoError::Timeout { expected: vec![id], got: Vec::new() });
        }
        if m.hardware_error != 0 {
            return Err(IoError::Device { id, error: m.hardware_error });
        }
        Ok(self.model)
    }

    fn reboot(&mut self, id: u8) -> IoResult<()> {
        let targets = self.targets(&[id]);
        for mid in targets {
            let Some(m) = self.motors.get_mut(&mid) else { continue };
            if m.no_response {
                continue;
            }
            // Reboot 后 RAM 恢复默认：torque OFF、goal 回到当前位置（仿真便利，避免复电即飞车）
            m.hardware_error = 0;
            m.torque = false;
            m.velocity = 0;
            m.goal = m.position;
        }
        Ok(())
    }

    fn inject_fault(&mut self, kind: &str, ids: &[u8], enable: bool) -> IoResult<()> {
        if kind == "transport_error" {
            self.transport_error = enable;
            return Ok(());
        }
        let targets = self.targets(ids);
        for id in targets {
            let Some(m) = self.motors.get_mut(&id) else { continue };
            match kind {
                "hardware_error" => {
                    m.hardware_error = if enable { 0x80 } else { 0 };
                    if enable {
                        m.velocity = 0;
                    }
                }
                "stall" => {
                    m.stall = enable;
                    m.velocity = 0;
                }
                "no_response" => {
                    m.no_response = enable;
                    m.velocity = 0;
                }
                "clear" => {
                    m.hardware_error = 0;
                    m.stall = false;
                    m.no_response = false;
                    m.velocity = 0;
                }
                other => {
                    return Err(IoError::Port(format!("unknown sim fault '{other}'")));
                }
            }
        }
        Ok(())
    }

    fn describe(&self) -> String {
        format!("sim({} motors)", self.motors.len())
    }
}

impl SimPort {
    fn apply_write(m: &mut SimMotor, addr: u16, data: &[u8]) {
        match addr {
            dxl::ADDR_TORQUE_ENABLE => m.torque = dxl::le_int(data) != 0,
            dxl::ADDR_GOAL_POSITION => m.goal = dxl::le_int(data),
            dxl::ADDR_MAX_POSITION_LIMIT => m.max_limit = dxl::le_int(data),
            dxl::ADDR_MIN_POSITION_LIMIT => m.min_limit = dxl::le_int(data),
            _ => {}
        }
    }
}

// ---------------------------------------------------------------- 测试

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    /// 假串口：Drop 时把 live 计数减一，用于证明 open 前旧 handle 已释放。
    struct FakeIo {
        live: Arc<AtomicUsize>,
        id: usize,
    }

    impl Drop for FakeIo {
        fn drop(&mut self) {
            self.live.fetch_sub(1, Ordering::SeqCst);
        }
    }

    impl ArmIo for FakeIo {
        fn sync_read(
            &mut self,
            _addr: u16,
            _len: u16,
            _ids: &[u8],
            _timeout: Duration,
        ) -> IoResult<SyncReadResult> {
            Ok(SyncReadResult::default())
        }
        fn write_register(&mut self, _id: u8, _addr: u16, _data: &[u8]) -> IoResult<()> {
            Ok(())
        }
        fn sync_write(&mut self, _addr: u16, _values: &[(u8, Vec<u8>)]) -> IoResult<()> {
            Ok(())
        }
        fn ping(&mut self, _id: u8) -> IoResult<u16> {
            Ok(1060)
        }
        fn reboot(&mut self, _id: u8) -> IoResult<()> {
            Ok(())
        }
        fn describe(&self) -> String {
            format!("fake-{}", self.id)
        }
    }

    fn fake_opener(
        live: Arc<AtomicUsize>,
        id: usize,
    ) -> impl FnOnce() -> IoResult<Box<dyn ArmIo>> {
        move || {
            // 关键断言：进入 open 时进程里不能还有旧 FakeIo 存活（旧 fd 必须已关闭）
            assert_eq!(
                live.load(Ordering::SeqCst),
                0,
                "open 前旧串口 handle 必须已释放（否则 TIOCEXCL 会 EBUSY 并泄漏 fd）"
            );
            let io = FakeIo { live: live.clone(), id };
            live.fetch_add(1, Ordering::SeqCst);
            Ok(Box::new(io) as Box<dyn ArmIo>)
        }
    }

    #[test]
    fn link_reopen_100_times_never_leaks_handle() {
        let live = Arc::new(AtomicUsize::new(0));
        let mut link = Link::new();
        for i in 0..100 {
            link.open_with(fake_opener(live.clone(), i)).expect("open ok");
            assert_eq!(live.load(Ordering::SeqCst), 1, "第 {i} 次 open 后应恰好 1 个 live handle");
            assert!(link.is_open());
            assert_eq!(link.io().expect("io").describe(), format!("fake-{i}"));
        }
        assert!(link.close());
        assert_eq!(live.load(Ordering::SeqCst), 0);
        assert!(!link.is_open());
        assert!(!link.close(), "重复 close 是幂等的");
    }

    #[test]
    fn link_failed_open_releases_old_handle_and_stays_closed() {
        let live = Arc::new(AtomicUsize::new(0));
        let mut link = Link::new();
        link.open_with(fake_opener(live.clone(), 1)).unwrap();
        assert_eq!(live.load(Ordering::SeqCst), 1);
        let res = link.open_with(|| {
            Err::<Box<dyn ArmIo>, IoError>(IoError::Port("open: Device or resource busy".into()))
        });
        assert!(res.is_err());
        assert!(!link.is_open());
        assert_eq!(live.load(Ordering::SeqCst), 0, "open 失败也必须释放旧 handle，不能留下 fd");
        // 失败后还能正常重开
        link.open_with(fake_opener(live.clone(), 2)).unwrap();
        assert_eq!(live.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn link_close_then_open_same_path() {
        // 模拟真实串口：第二个 open 只有在第一个 fd 已关闭时才可能成功。
        let live = Arc::new(AtomicUsize::new(0));
        let mut link = Link::new();
        link.open_with(fake_opener(live.clone(), 1)).unwrap();
        link.close();
        link.open_with(fake_opener(live.clone(), 2)).unwrap();
        assert_eq!(live.load(Ordering::SeqCst), 1);
    }

    // ------------------------------------------------------------ 仿真行为

    fn sim() -> SimPort {
        SimPort::new(&[1, 2, 3])
    }

    fn read_pos(sim: &mut SimPort, id: u8) -> i64 {
        let out = sim
            .sync_read(dxl::ADDR_PRESENT_POSITION, 4, &[id], Duration::from_millis(10))
            .unwrap();
        dxl::le_int(&out.data[&id])
    }

    #[test]
    fn sim_goal_write_moves_position() {
        let mut s = sim();
        let before = read_pos(&mut s, 1);
        s.sync_write(dxl::ADDR_GOAL_POSITION, &[(1, dxl::le_bytes(before + 100, 4))]).unwrap();
        std::thread::sleep(Duration::from_millis(120));
        let after = read_pos(&mut s, 1);
        assert!(after > before, "goal 写入后位置应前进: {before} -> {after}");
    }

    #[test]
    fn sim_stall_freezes_position_and_reports_zero_velocity() {
        let mut s = sim();
        let before = read_pos(&mut s, 3);
        s.sync_write(dxl::ADDR_GOAL_POSITION, &[(3, dxl::le_bytes(before + 500, 4))]).unwrap();
        s.inject_fault("stall", &[3], true).unwrap();
        std::thread::sleep(Duration::from_millis(100));
        let out = s
            .sync_read(dxl::ADDR_PRESENT_VELOCITY, 8, &[3], Duration::from_millis(10))
            .unwrap();
        let (vel, pos) = dxl::velocity_position(&out.data[&3]).unwrap();
        assert_eq!(pos, before, "堵转时位置冻结");
        assert_eq!(vel, 0, "堵转时速度视为 0");
        s.inject_fault("stall", &[3], false).unwrap();
    }

    #[test]
    fn sim_hardware_error_isolated_per_motor() {
        let mut s = sim();
        s.inject_fault("hardware_error", &[3], true).unwrap();
        let out = s
            .sync_read(dxl::ADDR_PRESENT_VELOCITY, 8, &[1, 2, 3], Duration::from_millis(10))
            .unwrap();
        assert_eq!(out.errors.get(&3), Some(&0x80));
        assert!(out.data.contains_key(&1) && out.data.contains_key(&2), "其他电机仍可读");
        // 硬件错误下的 ping 返回 Device 错误，而不是"整条臂超时"
        match s.ping(3) {
            Err(IoError::Device { id, error }) => {
                assert_eq!((id, error), (3, 0x80));
            }
            other => panic!("期望 Device 错误: {other:?}"),
        }
        // Reboot 清除 latched 错误，torque 复位为 OFF
        s.reboot(3).unwrap();
        let out = s
            .sync_read(dxl::ADDR_TORQUE_ENABLE, 1, &[3], Duration::from_millis(10))
            .unwrap();
        assert_eq!(dxl::le_int(&out.data[&3]), 0);
        let out = s
            .sync_read(dxl::ADDR_HARDWARE_ERROR_STATUS, 1, &[3], Duration::from_millis(10))
            .unwrap();
        assert_eq!(dxl::le_int(&out.data[&3]), 0);
    }

    #[test]
    fn sim_no_response_motor_missing_but_others_ok() {
        let mut s = sim();
        s.inject_fault("no_response", &[2], true).unwrap();
        let out = s
            .sync_read(dxl::ADDR_PRESENT_POSITION, 4, &[1, 2, 3], Duration::from_millis(10))
            .unwrap();
        assert!(!out.data.contains_key(&2));
        assert!(out.data.contains_key(&1) && out.data.contains_key(&3));
        assert_eq!(out.errors.get(&2), None);
    }

    #[test]
    fn sim_limits_readable_and_writable() {
        let mut s = sim();
        s.sync_write(dxl::ADDR_MAX_POSITION_LIMIT, &[(1, dxl::le_bytes(3309, 2))]).unwrap();
        s.sync_write(dxl::ADDR_MIN_POSITION_LIMIT, &[(1, dxl::le_bytes(1607, 2))]).unwrap();
        let out = s
            .sync_read(dxl::ADDR_MAX_POSITION_LIMIT, 4, &[1], Duration::from_millis(10))
            .unwrap();
        assert_eq!(dxl::position_limits(&out.data[&1]), Some((1607, 3309)));
    }

    #[test]
    fn sim_transport_error_injection() {
        let mut s = sim();
        s.inject_fault("transport_error", &[], true).unwrap();
        assert!(matches!(
            s.sync_read(dxl::ADDR_PRESENT_POSITION, 4, &[1], Duration::from_millis(10)),
            Err(IoError::Port(_))
        ));
        assert!(s.ping(1).is_err());
        s.inject_fault("transport_error", &[], false).unwrap();
        assert!(s.sync_read(dxl::ADDR_PRESENT_POSITION, 4, &[1], Duration::from_millis(10)).is_ok());
    }

    #[test]
    fn sim_reboot_broadcast_clears_all() {
        let mut s = sim();
        s.inject_fault("hardware_error", &[1, 2], true).unwrap();
        s.reboot(dxl::BROADCAST_ID).unwrap();
        let out = s
            .sync_read(dxl::ADDR_HARDWARE_ERROR_STATUS, 1, &[1, 2, 3], Duration::from_millis(10))
            .unwrap();
        for id in [1u8, 2, 3] {
            assert_eq!(dxl::le_int(&out.data[&id]), 0);
        }
    }
}

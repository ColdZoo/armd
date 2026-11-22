//! Dynamixel Protocol 2.0：帧构造与解析。
//!
//! 帧格式（真机字节流验证过）：
//! - 指令包：`FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAM... | CRC_L CRC_H`，LEN = params + 3
//! - 状态包：`FF FF FD 00 | ID | LEN_L LEN_H | 0x55 | ERR | PARAM... | CRC_L CRC_H`，LEN = params + 4
//! - CRC 为 CRC-16（poly 0x8005，init 0，非反射），覆盖整帧除最后 2 字节 CRC 外全部

pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
pub const STATUS_MARKER: u8 = 0x55;
pub const BROADCAST_ID: u8 = 0xFE;

pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_SYNC_READ: u8 = 0x82;
pub const INST_SYNC_WRITE: u8 = 0x83;

// 常用控制表地址（X 系列 / XL330 / XL430 通用）
pub const ADDR_DRIVE_MODE: u16 = 10;
pub const ADDR_OPERATING_MODE: u16 = 11;
pub const ADDR_HOMING_OFFSET: u16 = 20;
pub const ADDR_TORQUE_ENABLE: u16 = 64;
pub const ADDR_GOAL_POSITION: u16 = 116;
pub const ADDR_PRESENT_POSITION: u16 = 132;
pub const ADDR_PRESENT_TEMPERATURE: u16 = 146;

const fn crc_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x8005 } else { crc << 1 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

static CRC_TABLE: [u16; 256] = crc_table();

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in data {
        crc = (crc << 8) ^ CRC_TABLE[((crc >> 8) ^ (b as u16)) as usize & 0xFF];
    }
    crc
}

/// 构造指令包。
pub fn packet(inst: u8, id: u8, params: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(params.len() + 10);
    v.extend_from_slice(&HEADER);
    v.push(id);
    let len = (params.len() + 3) as u16;
    v.push((len & 0xFF) as u8);
    v.push((len >> 8) as u8);
    v.push(inst);
    v.extend_from_slice(params);
    let crc = crc16(&v);
    v.push((crc & 0xFF) as u8);
    v.push((crc >> 8) as u8);
    v
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Status {
    pub id: u8,
    pub error: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    /// 数据还不够一帧，继续读
    NeedMore,
    /// 不是以合法帧头开始（可丢弃 1 字节后重试）
    BadHeader,
    /// 帧头对但状态标记/CRC 不对（丢弃该帧）
    Corrupt,
}

/// 从字节流开头解析一帧状态包，返回 (状态包, 消耗字节数)。
pub fn decode(buf: &[u8]) -> Result<(Status, usize), DecodeError> {
    if buf.len() < HEADER.len() {
        return Err(DecodeError::NeedMore);
    }
    if buf[..4] != HEADER {
        return Err(DecodeError::BadHeader);
    }
    if buf.len() < 7 {
        return Err(DecodeError::NeedMore);
    }
    let id = buf[4];
    let len = (buf[5] as usize) | ((buf[6] as usize) << 8);
    let total = 7 + len;
    if buf.len() < total {
        return Err(DecodeError::NeedMore);
    }
    if len < 4 || buf[7] != STATUS_MARKER {
        return Err(DecodeError::Corrupt);
    }
    let error = buf[8];
    let data = buf[9..total - 2].to_vec();
    let crc = (buf[total - 2] as u16) | ((buf[total - 1] as u16) << 8);
    if crc16(&buf[..total - 2]) != crc {
        return Err(DecodeError::Corrupt);
    }
    Ok((Status { id, error, data }, total))
}

pub fn ping_packet(id: u8) -> Vec<u8> {
    packet(INST_PING, id, &[])
}

pub fn read_packet(id: u8, addr: u16, len: u16) -> Vec<u8> {
    packet(INST_READ, id, &[(addr & 0xFF) as u8, (addr >> 8) as u8, (len & 0xFF) as u8, (len >> 8) as u8])
}

pub fn write_packet(id: u8, addr: u16, data: &[u8]) -> Vec<u8> {
    let mut p = vec![(addr & 0xFF) as u8, (addr >> 8) as u8];
    p.extend_from_slice(data);
    packet(INST_WRITE, id, &p)
}

pub fn sync_read_packet(id: u8, addr: u16, len: u16, motors: &[u8]) -> Vec<u8> {
    let mut p = vec![(addr & 0xFF) as u8, (addr >> 8) as u8, (len & 0xFF) as u8, (len >> 8) as u8];
    p.extend_from_slice(motors);
    packet(INST_SYNC_READ, id, &p)
}

pub fn sync_write_packet(id: u8, addr: u16, len: u16, values: &[(u8, Vec<u8>)]) -> Vec<u8> {
    let mut p = vec![(addr & 0xFF) as u8, (addr >> 8) as u8, (len & 0xFF) as u8, (len >> 8) as u8];
    for (mid, v) in values {
        p.push(*mid);
        p.extend_from_slice(v);
    }
    packet(INST_SYNC_WRITE, id, &p)
}

/// 有符号小端编码（长度 1/2/4 字节）。
pub fn le_bytes(value: i64, len: usize) -> Vec<u8> {
    let mut out = vec![0u8; len];
    let raw = (value as u64).to_le_bytes();
    for i in 0..len.min(8) {
        out[i] = raw[i];
    }
    out
}

/// 有符号小端解码。
pub fn le_int(data: &[u8]) -> i64 {
    if data.is_empty() {
        return 0;
    }
    let mut raw = [0u8; 8];
    for (i, b) in data.iter().take(8).enumerate() {
        raw[i] = *b;
    }
    let v = i64::from_le_bytes(raw);
    // 符号扩展
    let bits = (data.len().min(8) * 8) as u32;
    if bits < 64 && v & (1i64 << (bits - 1)) != 0 {
        v - (1i64 << bits)
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_real_device() {
        // 真机抓包：ping ID1 的指令包，CRC = 0x4E19
        let p = ping_packet(1);
        assert_eq!(p, vec![0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E]);
        assert_eq!(crc16(&p[..p.len() - 2]), 0x4E19);
    }

    #[test]
    fn decode_real_ping_reply() {
        // 真机抓包：ping 回复（XL430，model 1060=0x0424，固件 50）
        let bytes = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x24, 0x04, 0x32, 0xB5, 0x5F,
        ];
        let (st, used) = decode(&bytes).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(st.id, 1);
        assert_eq!(st.error, 0);
        assert_eq!(le_int(&st.data[0..2]), 1060);
    }

    #[test]
    fn decode_real_position_reply() {
        // 真机抓包：读 Present_Position(ID1) → 0x05E7 = 1511
        let bytes = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xE7, 0x05, 0x00, 0x00, 0xD6, 0xD4,
        ];
        let (st, used) = decode(&bytes).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(le_int(&st.data), 1511);
    }

    #[test]
    fn decode_is_incremental() {
        let bytes = [
            0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xE7, 0x05, 0x00, 0x00, 0xD6, 0xD4,
        ];
        for n in 0..bytes.len() {
            assert_eq!(decode(&bytes[..n]), Err(DecodeError::NeedMore), "n={n}");
        }
        assert!(decode(&bytes).is_ok());
    }

    #[test]
    fn sync_read_packet_layout() {
        let p = sync_read_packet(BROADCAST_ID, ADDR_PRESENT_POSITION, 4, &[1, 2, 3, 4, 5, 6]);
        // 帧头 + ID + LEN(2) + INST + addr(2)+len(2)+ids(6) + CRC
        assert_eq!(&p[..4], &HEADER);
        assert_eq!(p[4], BROADCAST_ID);
        assert_eq!(p[7], INST_SYNC_READ);
        assert_eq!(&p[8..12], &[0x84, 0x00, 0x04, 0x00]);
        assert_eq!(&p[12..18], &[1, 2, 3, 4, 5, 6]);
        assert_eq!(&p[p.len() - 2..], &crc16(&p[..p.len() - 2]).to_le_bytes());
    }

    #[test]
    fn le_roundtrip() {
        assert_eq!(le_int(&le_bytes(-1511, 4)), -1511);
        assert_eq!(le_int(&le_bytes(2047, 4)), 2047);
        assert_eq!(le_bytes(-1, 1), vec![0xFF]);
    }
}

//! Bluetooth Low Energy link layer (LE 1M PHY): channel mapping, whitening,
//! CRC-24, and PDU interpretation for human-readable output.

pub mod decoder;
pub mod remoteid;

use crate::bits::{self, Lfsr7};

/// Access address used on the primary advertising channels.
pub const ADV_AA: u32 = 0x8E89_BED6;
/// CRC initialisation value used on the advertising channels.
pub const ADV_CRC_INIT: u32 = 0x55_5555;

/// RF channel number (0..=39, 2402 MHz + 2 MHz * n) for a centre frequency.
pub fn rf_channel_from_mhz(mhz: u32) -> Option<u8> {
    if (2402..=2480).contains(&mhz) && (mhz - 2402) % 2 == 0 {
        Some(((mhz - 2402) / 2) as u8)
    } else {
        None
    }
}

pub fn mhz_from_rf_channel(rf: u8) -> u32 {
    2402 + 2 * rf as u32
}

/// Link-layer channel index (0..=36 data, 37..=39 advertising) for an RF channel.
pub fn channel_index(rf: u8) -> u8 {
    match rf {
        0 => 37,
        12 => 38,
        39 => 39,
        1..=11 => rf - 1,
        _ => rf - 2,
    }
}

pub fn is_primary_adv(rf: u8) -> bool {
    matches!(rf, 0 | 12 | 39)
}

/// Whiten / de-whiten a bit stream (header, payload and CRC) in place.
pub fn whiten(bits: &mut [u8], channel_index: u8) {
    Lfsr7::ble(channel_index).apply(bits);
}

/// Bit-reverse a 24-bit CRC value. The CRC register below runs in air order,
/// which is the bit reversal of the natural (spec / CONNECT_IND) value.
pub fn crc_init_to_air(natural: u32) -> u32 {
    bits::reverse_bits(natural as u64, 24) as u32
}

pub fn crc_init_from_air(air: u32) -> u32 {
    bits::reverse_bits(air as u64, 24) as u32
}

const CRC_MASK: u32 = 0x5A_6000; // x^24+x^10+x^9+x^6+x^4+x^3+x+1, reflected, minus top bit

/// Compute the LE CRC-24 over a (de-whitened) bit stream. `init_air` is the
/// air-order initial value (see [`crc_init_to_air`]). The returned value is in
/// air order, i.e. it equals the 24 received CRC bits assembled LSB first.
pub fn crc24(init_air: u32, data_bits: &[u8]) -> u32 {
    let mut state = init_air & 0xFF_FFFF;
    for &b in data_bits {
        let fb = (state ^ b as u32) & 1;
        state >>= 1;
        if fb != 0 {
            state |= 1 << 23;
            state ^= CRC_MASK;
        }
    }
    state
}

/// Run the CRC register backwards: given the received CRC (air order) and the
/// PDU bits, recover the air-order initial value. Used to learn the CRCInit
/// of connections whose CONNECT_IND was not observed.
pub fn crc24_reverse(crc_air: u32, data_bits: &[u8]) -> u32 {
    let mut state = crc_air & 0xFF_FFFF;
    for &b in data_bits.iter().rev() {
        let fb = (state >> 23) & 1;
        if fb != 0 {
            state ^= CRC_MASK;
        }
        state &= 0x7F_FFFF;
        state = (state << 1) | ((fb ^ b as u32) & 1);
    }
    state
}

/// Validity rules for access addresses (Core spec Vol 6, Part B, 2.1.2),
/// used to reject noise in promiscuous connection discovery.
pub fn access_address_plausible(aa: u32) -> bool {
    if aa == ADV_AA || (aa ^ ADV_AA).count_ones() == 1 {
        return false;
    }
    // No more than six consecutive zeros or ones.
    let mut run = 1;
    for i in 1..32 {
        if (aa >> i) & 1 == (aa >> (i - 1)) & 1 {
            run += 1;
            if run > 6 {
                return false;
            }
        } else {
            run = 1;
        }
    }
    // No more than 24 transitions.
    let transitions = ((aa ^ (aa >> 1)) & 0x7FFF_FFFF).count_ones();
    if transitions > 24 {
        return false;
    }
    // At least two transitions in the six most significant bits.
    let top = aa >> 26;
    ((top ^ (top >> 1)) & 0x1F).count_ones() >= 2
}

/// Who sent a data-channel PDU (affects the Wireshark PDU-type flags).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Unknown,
    CentralToPeripheral,
    PeripheralToCentral,
}

/// A fully decoded LE packet.
#[derive(Clone, Debug)]
pub struct BlePacket {
    /// Seconds since the start of the capture (first preamble bit).
    pub time: f64,
    pub rf_channel: u8,
    pub access_address: u32,
    /// De-whitened PDU: 2-byte header followed by the payload.
    pub pdu: Vec<u8>,
    /// CRC bytes as received (air bit order, LSB first per byte).
    pub crc: [u8; 3],
    pub crc_checked: bool,
    pub crc_valid: bool,
    /// Bit errors corrected/tolerated in the access address.
    pub aa_errors: u8,
    pub signal_dbm: i8,
    pub noise_dbm: i8,
    pub direction: Direction,
    pub cfo_hz: f32,
    /// Bits flipped by the soft-decision CRC repair (0 = received intact).
    pub bits_corrected: u8,
}

impl BlePacket {
    pub fn is_advertising(&self) -> bool {
        self.access_address == ADV_AA
    }

    /// Bytes for LINKTYPE_BLUETOOTH_LE_LL: AA + PDU + CRC.
    pub fn ll_bytes(&self) -> Vec<u8> {
        let mut v = Vec::with_capacity(4 + self.pdu.len() + 3);
        v.extend_from_slice(&self.access_address.to_le_bytes());
        v.extend_from_slice(&self.pdu);
        v.extend_from_slice(&self.crc);
        v
    }

    /// One-line human readable summary of the PDU.
    pub fn describe(&self) -> String {
        if self.is_advertising() {
            describe_adv_pdu(&self.pdu)
        } else {
            describe_data_pdu(&self.pdu)
        }
    }
}

/// Parameters carried in a CONNECT_IND (or AUX_CONNECT_REQ) PDU.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConnectInd {
    pub initiator: [u8; 6],
    pub advertiser: [u8; 6],
    pub access_address: u32,
    pub crc_init: u32,
    pub win_size: u8,
    pub win_offset: u16,
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
    pub channel_map: [u8; 5],
    pub hop: u8,
    pub sca: u8,
}

pub fn parse_connect_ind(pdu: &[u8]) -> Option<ConnectInd> {
    let pdu_type = pdu.first()? & 0x0F;
    if pdu_type != 0x05 || pdu.len() < 2 + 34 {
        return None;
    }
    let p = &pdu[2..];
    let mut initiator = [0u8; 6];
    let mut advertiser = [0u8; 6];
    initiator.copy_from_slice(&p[0..6]);
    advertiser.copy_from_slice(&p[6..12]);
    let ll = &p[12..];
    let mut channel_map = [0u8; 5];
    channel_map.copy_from_slice(&ll[16..21]);
    Some(ConnectInd {
        initiator,
        advertiser,
        access_address: u32::from_le_bytes([ll[0], ll[1], ll[2], ll[3]]),
        crc_init: u32::from_le_bytes([ll[4], ll[5], ll[6], 0]),
        win_size: ll[7],
        win_offset: u16::from_le_bytes([ll[8], ll[9]]),
        interval: u16::from_le_bytes([ll[10], ll[11]]),
        latency: u16::from_le_bytes([ll[12], ll[13]]),
        timeout: u16::from_le_bytes([ll[14], ll[15]]),
        channel_map,
        hop: ll[21] & 0x1F,
        sca: ll[21] >> 5,
    })
}

pub fn fmt_addr(a: &[u8]) -> String {
    // Device addresses are transmitted least significant byte first.
    a.iter()
        .rev()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(":")
}

fn adv_pdu_name(t: u8) -> &'static str {
    match t {
        0x0 => "ADV_IND",
        0x1 => "ADV_DIRECT_IND",
        0x2 => "ADV_NONCONN_IND",
        0x3 => "SCAN_REQ",
        0x4 => "SCAN_RSP",
        0x5 => "CONNECT_IND",
        0x6 => "ADV_SCAN_IND",
        0x7 => "ADV_EXT_IND",
        0x8 => "AUX_CONNECT_RSP",
        _ => "ADV_RESERVED",
    }
}

/// Information extracted from advertising data (AD structures).
#[derive(Default, Debug, Clone)]
pub struct AdvInfo {
    pub name: Option<String>,
    pub company: Option<u16>,
    pub uuids16: Vec<u16>,
    /// Drone Remote ID message carried as service data (UUID 0xFFFA).
    pub remote_id: Option<remoteid::Message>,
}

pub fn parse_ad(data: &[u8]) -> AdvInfo {
    let mut info = AdvInfo::default();
    let mut i = 0;
    while i < data.len() {
        let len = data[i] as usize;
        if len == 0 || i + 1 + len > data.len() {
            break;
        }
        let ad_type = data[i + 1];
        let val = &data[i + 2..i + 1 + len];
        match ad_type {
            0x08 | 0x09 => {
                let s: String = String::from_utf8_lossy(val)
                    .chars()
                    .filter(|c| !c.is_control())
                    .collect();
                if ad_type == 0x09 || info.name.is_none() {
                    info.name = Some(s);
                }
            }
            0x02 | 0x03 => {
                for c in val.chunks_exact(2) {
                    info.uuids16.push(u16::from_le_bytes([c[0], c[1]]));
                }
            }
            0xFF if val.len() >= 2 => {
                info.company = Some(u16::from_le_bytes([val[0], val[1]]));
            }
            0x16 => {
                if let Some(m) = remoteid::from_service_data(val) {
                    info.remote_id = Some(m);
                }
            }
            _ => {}
        }
        i += 1 + len;
    }
    info
}

/// Address of the device that transmitted a legacy advertising-channel PDU
/// (AdvA, ScanA or InitA - always the first address field) and whether it is
/// random (TxAdd).
pub fn adv_address(pdu: &[u8]) -> Option<([u8; 6], bool)> {
    if pdu.len() < 8 || (pdu[0] & 0x0F) > 0x6 {
        return None;
    }
    let mut a = [0u8; 6];
    a.copy_from_slice(&pdu[2..8]);
    Some((a, pdu[0] & 0x40 != 0))
}

pub fn describe_adv_pdu(pdu: &[u8]) -> String {
    if pdu.len() < 2 {
        return "truncated".into();
    }
    let t = pdu[0] & 0x0F;
    let tx_add = pdu[0] & 0x40 != 0;
    let rx_add = pdu[0] & 0x80 != 0;
    let p = &pdu[2..];
    let kind = |r: bool| if r { "random" } else { "public" };
    let mut s = adv_pdu_name(t).to_string();
    match t {
        0x0 | 0x2 | 0x4 | 0x6 if p.len() >= 6 => {
            s += &format!(" AdvA={} ({})", fmt_addr(&p[0..6]), kind(tx_add));
            let info = parse_ad(&p[6..]);
            if let Some(n) = info.name {
                s += &format!(" name=\"{n}\"");
            }
            if let Some(c) = info.company {
                s += &format!(" company=0x{c:04x}");
            }
            if !info.uuids16.is_empty() {
                let u: Vec<String> = info.uuids16.iter().map(|u| format!("{u:04x}")).collect();
                s += &format!(" uuid16=[{}]", u.join(","));
            }
            if let Some(m) = &info.remote_id {
                s += &m.describe();
            }
        }
        0x1 if p.len() >= 12 => {
            s += &format!(
                " AdvA={} ({}) TargetA={} ({})",
                fmt_addr(&p[0..6]),
                kind(tx_add),
                fmt_addr(&p[6..12]),
                kind(rx_add)
            );
        }
        0x3 if p.len() >= 12 => {
            s += &format!(
                " ScanA={} ({}) AdvA={} ({})",
                fmt_addr(&p[0..6]),
                kind(tx_add),
                fmt_addr(&p[6..12]),
                kind(rx_add)
            );
        }
        0x5 => {
            if let Some(c) = parse_connect_ind(pdu) {
                s += &format!(
                    " InitA={} AdvA={} AA={:08x} CRCInit={:06x} interval={:.2}ms hop={}",
                    fmt_addr(&c.initiator),
                    fmt_addr(&c.advertiser),
                    c.access_address,
                    c.crc_init,
                    c.interval as f64 * 1.25,
                    c.hop
                );
            }
        }
        _ => {}
    }
    s
}

pub fn ll_control_name(op: u8) -> &'static str {
    match op {
        0x00 => "LL_CONNECTION_UPDATE_IND",
        0x01 => "LL_CHANNEL_MAP_IND",
        0x02 => "LL_TERMINATE_IND",
        0x03 => "LL_ENC_REQ",
        0x04 => "LL_ENC_RSP",
        0x05 => "LL_START_ENC_REQ",
        0x06 => "LL_START_ENC_RSP",
        0x07 => "LL_UNKNOWN_RSP",
        0x08 => "LL_FEATURE_REQ",
        0x09 => "LL_FEATURE_RSP",
        0x0A => "LL_PAUSE_ENC_REQ",
        0x0B => "LL_PAUSE_ENC_RSP",
        0x0C => "LL_VERSION_IND",
        0x0D => "LL_REJECT_IND",
        0x0E => "LL_PERIPHERAL_FEATURE_REQ",
        0x0F => "LL_CONNECTION_PARAM_REQ",
        0x10 => "LL_CONNECTION_PARAM_RSP",
        0x11 => "LL_REJECT_EXT_IND",
        0x12 => "LL_PING_REQ",
        0x13 => "LL_PING_RSP",
        0x14 => "LL_LENGTH_REQ",
        0x15 => "LL_LENGTH_RSP",
        0x16 => "LL_PHY_REQ",
        0x17 => "LL_PHY_RSP",
        0x18 => "LL_PHY_UPDATE_IND",
        0x19 => "LL_MIN_USED_CHANNELS_IND",
        0x1A => "LL_CTE_REQ",
        0x1B => "LL_CTE_RSP",
        0x1C => "LL_PERIODIC_SYNC_IND",
        0x1D => "LL_CLOCK_ACCURACY_REQ",
        0x1E => "LL_CLOCK_ACCURACY_RSP",
        0x1F => "LL_CIS_REQ",
        0x20 => "LL_CIS_RSP",
        0x21 => "LL_CIS_IND",
        0x22 => "LL_CIS_TERMINATE_IND",
        0x23 => "LL_POWER_CONTROL_REQ",
        0x24 => "LL_POWER_CONTROL_RSP",
        0x25 => "LL_POWER_CHANGE_IND",
        _ => "LL_CTRL",
    }
}

pub fn describe_data_pdu(pdu: &[u8]) -> String {
    if pdu.len() < 2 {
        return "truncated".into();
    }
    let llid = pdu[0] & 0x03;
    let nesn = (pdu[0] >> 2) & 1;
    let sn = (pdu[0] >> 3) & 1;
    let md = (pdu[0] >> 4) & 1;
    let len = pdu[1] as usize;
    let flags = format!("SN={sn} NESN={nesn} MD={md}");
    match llid {
        1 if len == 0 => format!("LL Empty PDU {flags}"),
        1 => format!("LL Data (continuation) len={len} {flags}"),
        2 => {
            if pdu.len() >= 6 {
                let l2_len = u16::from_le_bytes([pdu[2], pdu[3]]);
                let cid = u16::from_le_bytes([pdu[4], pdu[5]]);
                let what = match cid {
                    0x0004 => "ATT",
                    0x0005 => "LE Signaling",
                    0x0006 => "SMP",
                    _ => "L2CAP",
                };
                format!("LL Data (start) {what} cid=0x{cid:04x} l2len={l2_len} {flags}")
            } else {
                format!("LL Data (start) len={len} {flags}")
            }
        }
        3 if pdu.len() >= 3 => format!("{} {flags}", ll_control_name(pdu[2])),
        _ => format!("LL reserved LLID len={len}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::{pack_lsb, unpack_lsb};

    /// Reference implementation operating MSB-first on the natural register
    /// (as found in many bit-banging LE transmitters).
    fn natural_crc(data: &[u8], init: u32) -> [u8; 3] {
        let mut d = [(init >> 16) as u8, (init >> 8) as u8, init as u8];
        for &byte in data {
            let mut x = byte;
            for _ in 0..8 {
                let t = d[0] >> 7;
                d[0] = (d[0] << 1) | (d[1] >> 7);
                d[1] = (d[1] << 1) | (d[2] >> 7);
                d[2] <<= 1;
                if t != (x & 1) {
                    d[2] ^= 0x5B;
                    d[1] ^= 0x06;
                }
                x >>= 1;
            }
        }
        // transmitted MSB first => bytes on air are bit-reversed
        [
            d[0].reverse_bits(),
            d[1].reverse_bits(),
            d[2].reverse_bits(),
        ]
    }

    #[test]
    fn crc_matches_reference() {
        let pdu = [0x40u8, 0x06, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let reference = natural_crc(&pdu, ADV_CRC_INIT);
        let crc = crc24(crc_init_to_air(ADV_CRC_INIT), &unpack_lsb(&pdu));
        let crc_bytes = pack_lsb(&crate::bits::from_u64(crc as u64, 24));
        assert_eq!(crc_bytes, reference);
    }

    #[test]
    fn crc_reverse_recovers_init() {
        let pdu = unpack_lsb(&[0x0Eu8, 0x00]);
        for init in [0x555555u32, 0x123456, 0xABCDEF, 0x000001] {
            let a = crc_init_to_air(init);
            let crc = crc24(a, &pdu);
            assert_eq!(crc_init_from_air(crc24_reverse(crc, &pdu)), init);
        }
    }

    #[test]
    fn channel_mapping() {
        assert_eq!(channel_index(0), 37);
        assert_eq!(channel_index(1), 0);
        assert_eq!(channel_index(11), 10);
        assert_eq!(channel_index(12), 38);
        assert_eq!(channel_index(13), 11);
        assert_eq!(channel_index(38), 36);
        assert_eq!(channel_index(39), 39);
        assert_eq!(rf_channel_from_mhz(2426), Some(12));
        assert_eq!(rf_channel_from_mhz(2427), None);
    }

    #[test]
    fn aa_rules() {
        assert!(!access_address_plausible(ADV_AA));
        assert!(!access_address_plausible(0x0000_0000));
        assert!(!access_address_plausible(0x5555_5555)); // too many transitions
        assert!(access_address_plausible(0x50654A3C));
    }

    #[test]
    fn describe_adv_with_remote_id() {
        use crate::synth::{remote_id_adv, rid_basic_id, rid_location};
        let addr = [0x01, 0x02, 0x03, 0x04, 0x05, 0xC6];
        let pdu = remote_id_adv(addr, 3, &rid_basic_id(1, 2, "1581F5FKD229400EQ5"));
        assert_eq!(pdu.len(), 2 + 6 + 31);
        assert_eq!(
            describe_adv_pdu(&pdu),
            "ADV_NONCONN_IND AdvA=c6:05:04:03:02:01 (random) \
             rid_id=\"1581F5FKD229400EQ5\" rid_id_type=serial rid_ua=multirotor"
        );
        let loc = rid_location(52.229676, 21.012229, 152.5, 45.0, 5.25, 270);
        let info = parse_ad(&remote_id_adv(addr, 4, &loc)[8..]);
        assert_eq!(
            info.remote_id.unwrap().describe(),
            " rid_status=airborne rid_pos=52.2296760,21.0122290 rid_alt=152.5m \
             rid_height_takeoff=45.0m rid_speed=5.25m/s rid_vspeed=0.0m/s rid_track=270"
        );
        // other service data is not mistaken for Remote ID
        let info = parse_ad(&[0x05, 0x16, 0x6F, 0xFD, 0x01, 0x02]);
        assert!(info.remote_id.is_none());
    }
}

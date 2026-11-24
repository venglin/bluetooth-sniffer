//! Drone Remote ID (ASTM F3411-22a, ASD-STAN EN 4709-002) in Bluetooth 4
//! legacy advertising.
//!
//! A broadcast module puts one 25-byte message into each advertisement, as
//! service data for the 16-bit UUID 0xFFFA (ASTM International): the
//! application code 0x0D, a message counter and the message. Successive
//! advertisements cycle through the message types. Bluetooth 5 Long Range and
//! Wi-Fi broadcasts carry the same messages, often in a message pack, but
//! btsdr does not receive them.

/// 16-bit service UUID assigned to ASTM International for Remote ID.
pub const SERVICE_UUID: u16 = 0xFFFA;
/// Application code of Open Drone ID in the service data.
pub const APP_CODE: u8 = 0x0D;
/// Every Remote ID message is 25 bytes long.
pub const MESSAGE_LEN: usize = 25;

/// One decoded Remote ID message. Fields the transmitter marks as unknown
/// are `None`.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// Basic ID (type 0): who the aircraft is.
    BasicId {
        id_type: u8,
        ua_type: u8,
        id: String,
    },
    /// Location/Vector (type 1).
    Location(Location),
    /// Authentication (type 2); only the page number is reported.
    Auth { page: u8 },
    /// Self-ID (type 3): free text, e.g. the purpose of the flight.
    SelfId { text: String },
    /// System (type 4): where the operator is.
    System {
        operator_pos: Option<(f64, f64)>,
        operator_alt: Option<f32>,
    },
    /// Operator ID (type 5), e.g. an EU operator registration number.
    OperatorId { id: String },
    /// Message pack (type 15) or a reserved type.
    Other { msg_type: u8 },
}

/// Location/Vector message.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Location {
    /// 0 undeclared, 1 ground, 2 airborne, 3 emergency, 4 Remote ID failure.
    pub status: u8,
    /// Latitude and longitude in degrees.
    pub pos: Option<(f64, f64)>,
    /// Geodetic (WGS-84) altitude in metres.
    pub alt_geo: Option<f32>,
    /// Pressure altitude in metres.
    pub alt_baro: Option<f32>,
    /// Height in metres, and whether it is above ground (true) or above the
    /// takeoff point (false).
    pub height: Option<(f32, bool)>,
    /// Ground speed in m/s.
    pub speed: Option<f32>,
    /// Vertical speed in m/s, positive up.
    pub vspeed: Option<f32>,
    /// Track over ground in degrees clockwise from true north.
    pub track: Option<u16>,
}

/// Decodes the value of a service data AD structure (type 0x16, starting with
/// the UUID). Returns `None` if it is not Remote ID.
pub fn from_service_data(val: &[u8]) -> Option<Message> {
    if val.len() < 4 + MESSAGE_LEN
        || u16::from_le_bytes([val[0], val[1]]) != SERVICE_UUID
        || val[2] != APP_CODE
    {
        return None;
    }
    parse_message(&val[4..4 + MESSAGE_LEN])
}

/// Decodes one 25-byte message.
pub fn parse_message(m: &[u8]) -> Option<Message> {
    let m: &[u8; MESSAGE_LEN] = m.get(..MESSAGE_LEN)?.try_into().ok()?;
    Some(match m[0] >> 4 {
        0 => Message::BasicId {
            id_type: m[1] >> 4,
            ua_type: m[1] & 0x0F,
            id: id_string(&m[2..22]),
        },
        1 => Message::Location(location(m)),
        2 => Message::Auth { page: m[1] & 0x0F },
        3 => Message::SelfId {
            text: text(&m[2..25]),
        },
        4 => Message::System {
            operator_pos: position(&m[2..6], &m[6..10]),
            operator_alt: altitude(&m[18..20]),
        },
        5 => Message::OperatorId {
            id: text(&m[2..22]),
        },
        t => Message::Other { msg_type: t },
    })
}

fn location(m: &[u8; MESSAGE_LEN]) -> Location {
    let flags = m[1];
    let track = m[2] as u16 + if flags & 0x02 != 0 { 180 } else { 0 };
    let speed = match (m[3], flags & 0x01 != 0) {
        (255, true) => None,
        (v, true) => Some(v as f32 * 0.75 + 63.75),
        (v, false) => Some(v as f32 * 0.25),
    };
    let vspeed = m[4] as i8;
    Location {
        status: flags >> 4,
        pos: position(&m[5..9], &m[9..13]),
        alt_baro: altitude(&m[13..15]),
        alt_geo: altitude(&m[15..17]),
        height: altitude(&m[17..19]).map(|h| (h, flags & 0x04 != 0)),
        speed,
        vspeed: (vspeed != 126).then_some(vspeed as f32 * 0.5),
        track: (track < 360).then_some(track),
    }
}

/// Latitude and longitude, each an int32 in units of 1e-7 degree; 0,0 means
/// unknown.
fn position(lat: &[u8], lon: &[u8]) -> Option<(f64, f64)> {
    let lat = i32::from_le_bytes(lat.try_into().ok()?) as f64 * 1e-7;
    let lon = i32::from_le_bytes(lon.try_into().ok()?) as f64 * 1e-7;
    let known = (lat, lon) != (0.0, 0.0) && lat.abs() <= 90.0 && lon.abs() <= 180.0;
    known.then_some((lat, lon))
}

/// Altitudes and heights: uint16 in half metres from -1000 m; -1000 m means
/// unknown.
fn altitude(v: &[u8]) -> Option<f32> {
    let raw = u16::from_le_bytes(v.try_into().ok()?);
    (raw != 0).then_some(raw as f32 * 0.5 - 1000.0)
}

/// A NUL-padded ASCII field.
fn text(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    b[..end]
        .iter()
        .map(|&c| {
            if (0x20..0x7F).contains(&c) {
                c as char
            } else {
                '?'
            }
        })
        .collect::<String>()
        .trim()
        .to_string()
}

/// A UAS ID: ASCII for serial numbers and registrations, hex for the binary
/// UTM UUID and session IDs.
fn id_string(b: &[u8]) -> String {
    let end = b.iter().rposition(|&c| c != 0).map_or(0, |i| i + 1);
    let b = &b[..end];
    if b.iter().all(|c| (0x20..0x7F).contains(c)) {
        text(b)
    } else {
        b.iter().map(|c| format!("{c:02x}")).collect()
    }
}

pub fn id_type_name(t: u8) -> &'static str {
    match t {
        0 => "none",
        1 => "serial",
        2 => "registration",
        3 => "utm",
        4 => "session",
        _ => "reserved",
    }
}

pub fn ua_type_name(t: u8) -> &'static str {
    const NAMES: [&str; 16] = [
        "none",
        "aeroplane",
        "multirotor",
        "gyroplane",
        "vtol",
        "ornithopter",
        "glider",
        "kite",
        "free-balloon",
        "captive-balloon",
        "airship",
        "parachute",
        "rocket",
        "tethered",
        "ground-obstacle",
        "other",
    ];
    NAMES[(t & 0x0F) as usize]
}

fn status_name(s: u8) -> &'static str {
    match s {
        0 => "undeclared",
        1 => "ground",
        2 => "airborne",
        3 => "emergency",
        4 => "rid-failure",
        _ => "reserved",
    }
}

impl Message {
    pub fn msg_type(&self) -> u8 {
        match self {
            Message::BasicId { .. } => 0,
            Message::Location(_) => 1,
            Message::Auth { .. } => 2,
            Message::SelfId { .. } => 3,
            Message::System { .. } => 4,
            Message::OperatorId { .. } => 5,
            Message::Other { msg_type } => *msg_type,
        }
    }

    /// `key=value` fields for the advertising line, each with a leading space.
    pub fn describe(&self) -> String {
        let pos = |(lat, lon): (f64, f64)| format!("{lat:.7},{lon:.7}");
        let mut s = String::new();
        match self {
            Message::BasicId {
                id_type,
                ua_type,
                id,
            } => {
                s += &format!(
                    " rid_id=\"{id}\" rid_id_type={} rid_ua={}",
                    id_type_name(*id_type),
                    ua_type_name(*ua_type)
                );
            }
            Message::Location(l) => {
                s += &format!(" rid_status={}", status_name(l.status));
                if let Some(p) = l.pos {
                    s += &format!(" rid_pos={}", pos(p));
                }
                if let Some(a) = l.alt_geo {
                    s += &format!(" rid_alt={a:.1}m");
                }
                if let Some(a) = l.alt_baro {
                    s += &format!(" rid_alt_baro={a:.1}m");
                }
                if let Some((h, agl)) = l.height {
                    let over = if agl { "agl" } else { "takeoff" };
                    s += &format!(" rid_height_{over}={h:.1}m");
                }
                if let Some(v) = l.speed {
                    s += &format!(" rid_speed={v:.2}m/s");
                }
                if let Some(v) = l.vspeed {
                    s += &format!(" rid_vspeed={v:.1}m/s");
                }
                if let Some(t) = l.track {
                    s += &format!(" rid_track={t}");
                }
            }
            Message::Auth { page } => s += &format!(" rid_auth_page={page}"),
            Message::SelfId { text } => s += &format!(" rid_desc=\"{text}\""),
            Message::System {
                operator_pos,
                operator_alt,
            } => {
                if let Some(p) = operator_pos {
                    s += &format!(" rid_operator_pos={}", pos(*p));
                }
                if let Some(a) = operator_alt {
                    s += &format!(" rid_operator_alt={a:.1}m");
                }
            }
            Message::OperatorId { id } => s += &format!(" rid_operator_id=\"{id}\""),
            Message::Other { msg_type } => s += &format!(" rid_msg_type={msg_type}"),
        }
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Service data as transmitted: UUID, application code, counter, message.
    fn service_data(msg: [u8; MESSAGE_LEN]) -> Vec<u8> {
        let mut v = vec![0xFA, 0xFF, APP_CODE, 0x07];
        v.extend_from_slice(&msg);
        v
    }

    #[test]
    fn basic_id() {
        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x02; // type 0, protocol version 2
        m[1] = 0x12; // serial number, multirotor
        m[2..20].copy_from_slice(b"1581F5FKD229400EQ5");
        let msg = from_service_data(&service_data(m)).unwrap();
        assert_eq!(
            msg,
            Message::BasicId {
                id_type: 1,
                ua_type: 2,
                id: "1581F5FKD229400EQ5".into()
            }
        );
        assert_eq!(
            msg.describe(),
            " rid_id=\"1581F5FKD229400EQ5\" rid_id_type=serial rid_ua=multirotor"
        );
        // a binary session ID is shown in hex
        m[1] = 0x42;
        m[2..22].copy_from_slice(&[
            0x01, 0xAB, 0x00, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ]);
        let Message::BasicId { id, .. } = parse_message(&m).unwrap() else {
            panic!()
        };
        assert_eq!(id, "01ab00ff");
    }

    #[test]
    fn location() {
        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x12;
        m[1] = 0x20 | 0x02; // airborne, height above takeoff, track >= 180
        m[2] = 90; // 270 degrees
        m[3] = 21; // 5.25 m/s
        m[4] = (-1i8) as u8; // -0.5 m/s
        m[5..9].copy_from_slice(&522_296_760i32.to_le_bytes());
        m[9..13].copy_from_slice(&210_122_290i32.to_le_bytes());
        m[13..15].copy_from_slice(&0u16.to_le_bytes()); // pressure altitude unknown
        m[15..17].copy_from_slice(&2305u16.to_le_bytes()); // 152.5 m
        m[17..19].copy_from_slice(&2090u16.to_le_bytes()); // 45.0 m
        let msg = from_service_data(&service_data(m)).unwrap();
        let Message::Location(l) = &msg else { panic!() };
        assert_eq!(l.status, 2);
        let (lat, lon) = l.pos.unwrap();
        assert!((lat - 52.229676).abs() < 1e-9 && (lon - 21.012229).abs() < 1e-9);
        assert_eq!(l.alt_baro, None);
        assert_eq!(l.alt_geo, Some(152.5));
        assert_eq!(l.height, Some((45.0, false)));
        assert_eq!(l.speed, Some(5.25));
        assert_eq!(l.vspeed, Some(-0.5));
        assert_eq!(l.track, Some(270));
        assert_eq!(
            msg.describe(),
            " rid_status=airborne rid_pos=52.2296760,21.0122290 rid_alt=152.5m \
             rid_height_takeoff=45.0m rid_speed=5.25m/s rid_vspeed=-0.5m/s rid_track=270"
        );
    }

    #[test]
    fn location_unknown_values() {
        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x12;
        m[1] = 0x03; // track >= 180, speed multiplier 0.75
        m[2] = 181; // 361 = unknown
        m[3] = 255; // unknown with the 0.75 multiplier
        m[4] = 126; // 63 m/s = unknown
        let Message::Location(l) = parse_message(&m).unwrap() else {
            panic!()
        };
        assert_eq!(l, Location::default());
        // with the 0.75 multiplier a known speed starts at 63.75 m/s
        m[3] = 10;
        let Message::Location(l) = parse_message(&m).unwrap() else {
            panic!()
        };
        assert_eq!(l.speed, Some(71.25));
    }

    #[test]
    fn system_self_id_operator() {
        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x42;
        m[2..6].copy_from_slice(&522_290_000i32.to_le_bytes());
        m[6..10].copy_from_slice(&210_110_000i32.to_le_bytes());
        m[18..20].copy_from_slice(&2220u16.to_le_bytes()); // 110.0 m
        assert_eq!(
            parse_message(&m).unwrap().describe(),
            " rid_operator_pos=52.2290000,21.0110000 rid_operator_alt=110.0m"
        );

        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x32;
        m[2..15].copy_from_slice(b"Survey flight");
        assert_eq!(
            parse_message(&m).unwrap().describe(),
            " rid_desc=\"Survey flight\""
        );

        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x52;
        m[2..18].copy_from_slice(b"FIN87astrdge12k8");
        assert_eq!(
            parse_message(&m).unwrap().describe(),
            " rid_operator_id=\"FIN87astrdge12k8\""
        );
    }

    #[test]
    fn rejects_other_service_data() {
        let mut m = [0u8; MESSAGE_LEN];
        m[0] = 0x02;
        let mut v = service_data(m);
        assert!(from_service_data(&v).is_some());
        v[2] = 0x0E; // another application code
        assert!(from_service_data(&v).is_none());
        let mut v = service_data(m);
        v[0] = 0xFE; // another UUID
        assert!(from_service_data(&v).is_none());
        let v = service_data(m);
        assert!(from_service_data(&v[..v.len() - 1]).is_none()); // truncated
    }
}

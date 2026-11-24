//! Human-readable console output and end-of-capture summary.

use crate::ble::decoder::BleDecoder;
use crate::ble::{self, BlePacket, Direction};
use crate::bredr::decoder::{BrDecoder, BrPacket};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

#[derive(Default)]
struct BleDevice {
    random: bool,
    name: Option<String>,
    company: Option<u16>,
    packets: u64,
    best_rssi: i8,
    pdu_types: BTreeSet<&'static str>,
    /// Remote ID messages received, and the latest one of each type.
    rid_messages: u64,
    remote_id: BTreeMap<u8, ble::remoteid::Message>,
}

#[derive(Default)]
struct BrDevice {
    class_of_device: u32,
    seen: u64,
    name: Option<String>,
}

pub struct Reporter {
    out: Box<dyn Write + Send>,
    quiet: bool,
    show_bad: bool,
    print_summary: bool,
    ble_devices: BTreeMap<[u8; 6], BleDevice>,
    br_devices: BTreeMap<String, BrDevice>,
    pub ble_count: u64,
    pub br_count: u64,
}

fn pdu_short(p: &BlePacket) -> &'static str {
    match p.pdu.first().map(|b| b & 0x0F) {
        Some(0) => "ADV_IND",
        Some(1) => "ADV_DIRECT_IND",
        Some(2) => "ADV_NONCONN_IND",
        Some(3) => "SCAN_REQ",
        Some(4) => "SCAN_RSP",
        Some(5) => "CONNECT_IND",
        Some(6) => "ADV_SCAN_IND",
        Some(7) => "ADV_EXT_IND",
        _ => "OTHER",
    }
}

impl Reporter {
    pub fn new(out: Box<dyn Write + Send>, quiet: bool, show_bad: bool) -> Self {
        Self {
            out,
            quiet,
            show_bad,
            print_summary: true,
            ble_devices: BTreeMap::new(),
            br_devices: BTreeMap::new(),
            ble_count: 0,
            br_count: 0,
        }
    }

    pub fn note(&mut self, s: &str) {
        let _ = writeln!(self.out, "# {s}");
    }

    pub fn ble(&mut self, p: &BlePacket) {
        self.ble_count += 1;
        if p.is_advertising() && p.crc_valid {
            if let Some((addr, random)) = ble::adv_address(&p.pdu) {
                let d = self.ble_devices.entry(addr).or_insert_with(|| BleDevice {
                    best_rssi: i8::MIN,
                    ..Default::default()
                });
                d.random = random;
                d.packets += 1;
                d.best_rssi = d.best_rssi.max(p.signal_dbm);
                d.pdu_types.insert(pdu_short(p));
                let t = p.pdu[0] & 0x0F;
                if matches!(t, 0 | 2 | 4 | 6) && p.pdu.len() > 8 {
                    let info = ble::parse_ad(&p.pdu[8..]);
                    if info.name.is_some() {
                        d.name = info.name;
                    }
                    if info.company.is_some() {
                        d.company = info.company;
                    }
                    if let Some(m) = info.remote_id {
                        d.rid_messages += 1;
                        d.remote_id.insert(m.msg_type(), m);
                    }
                }
            }
        }
        if self.quiet || (!p.crc_valid && !self.show_bad) {
            return;
        }
        let dir = match p.direction {
            Direction::CentralToPeripheral => "C->P ",
            Direction::PeripheralToCentral => "P->C ",
            Direction::Unknown => "",
        };
        let ci = ble::channel_index(p.rf_channel);
        let _ = writeln!(
            self.out,
            "{:>11.6} BLE {} MHz ch{:<2} {:>4} dB AA={:08x} {}{}{}",
            p.time,
            ble::mhz_from_rf_channel(p.rf_channel),
            ci,
            p.signal_dbm,
            p.access_address,
            dir,
            p.describe(),
            match (p.crc_valid, p.bits_corrected) {
                (false, _) => " [BAD CRC]".to_string(),
                (true, 0) => String::new(),
                (true, n) => format!(" [{n} bit(s) corrected]"),
            }
        );
    }

    pub fn bredr(&mut self, p: &BrPacket) {
        self.br_count += 1;
        if let Some(f) = p.fhs() {
            let d = self.br_devices.entry(f.bd_addr()).or_default();
            d.class_of_device = f.class_of_device;
            d.seen += 1;
        }
        if self.quiet {
            return;
        }
        let _ = writeln!(
            self.out,
            "{:>11.6} BR  {} MHz ch{:<2} {:>4} dB {}",
            p.time,
            2402 + p.rf_channel as u32,
            p.rf_channel,
            p.signal_dbm,
            p.describe()
        );
    }

    pub fn status(&mut self, line: &str) {
        eprintln!("{line}");
    }

    pub fn flush(&mut self) {
        let _ = self.out.flush();
    }

    /// Disable the end-of-capture summary on stderr (extcap mode).
    pub fn set_print_summary(&mut self, on: bool) {
        self.print_summary = on;
    }

    pub fn summary(&mut self, ble: Option<&BleDecoder>, br: Option<&BrDecoder>) {
        if !self.print_summary {
            return;
        }
        let mut s = String::new();
        s += "\n=== Summary ===\n";
        if let Some(b) = ble {
            s += &format!(
                "BLE: {} advertising + {} data packets (CRC ok, {} after bit repair), {} bad CRC, {} connections\n",
                b.stats.adv_packets,
                b.stats.data_packets,
                b.stats.crc_repaired,
                b.stats.bad_crc,
                b.stats.connections_seen
            );
            if !self.ble_devices.is_empty() {
                s += &format!(
                    "  {:<17} {:<6} {:>6} {:>5}  {:<28} {}\n",
                    "address", "type", "pkts", "rssi", "name", "PDUs"
                );
                let mut devs: Vec<_> = self.ble_devices.iter().collect();
                devs.sort_by_key(|d| std::cmp::Reverse(d.1.packets));
                for (addr, d) in devs.iter().take(50) {
                    let mut name = d.name.clone().unwrap_or_default();
                    if let Some(ble::remoteid::Message::BasicId { id, .. }) = d.remote_id.get(&0) {
                        if name.is_empty() {
                            name = format!("Remote ID {id}");
                        }
                    }
                    if let Some(c) = d.company {
                        if name.is_empty() {
                            name = format!("(company 0x{c:04x})");
                        }
                    }
                    let types: Vec<&str> = d.pdu_types.iter().copied().collect();
                    s += &format!(
                        "  {:<17} {:<6} {:>6} {:>5}  {:<28} {}\n",
                        ble::fmt_addr(*addr),
                        if d.random { "random" } else { "public" },
                        d.packets,
                        d.best_rssi,
                        name.chars().take(28).collect::<String>(),
                        types.join(",")
                    );
                }
                if devs.len() > 50 {
                    s += &format!("  ... and {} more\n", devs.len() - 50);
                }
                // every drone, with the latest message of each informative type
                for (addr, d) in devs.iter().filter(|d| d.1.rid_messages > 0) {
                    let fields: String = d
                        .remote_id
                        .values()
                        .filter(|m| matches!(m.msg_type(), 0 | 1 | 3 | 4 | 5))
                        .map(|m| m.describe())
                        .collect();
                    s += &format!(
                        "  Remote ID {}: {} messages{fields}\n",
                        ble::fmt_addr(*addr),
                        d.rid_messages
                    );
                }
            }
            let conns: Vec<_> = b.connections().collect();
            for c in conns {
                s += &format!(
                    "  connection AA={:08x} CRCInit={:06x} {:?} packets={}{}\n",
                    c.access_address,
                    c.crc_init,
                    c.source,
                    c.packets,
                    c.interval_ms
                        .map(|i| format!(" interval={i:.2}ms"))
                        .unwrap_or_default()
                );
            }
        }
        if let Some(b) = br {
            s += &format!(
                "BR:  {} ID packets, {} packets with header, {} headers de-whitened, {} payload CRC ok; EDR: {} detected, {} CRC ok\n",
                b.stats.id_packets,
                b.stats.header_packets,
                b.stats.decoded_headers,
                b.stats.crc_ok,
                b.stats.edr_detected,
                b.stats.edr_crc_ok
            );
            let mut pns: Vec<_> = b.piconets().collect();
            pns.sort_by_key(|p| std::cmp::Reverse(p.packets));
            if !pns.is_empty() {
                s += &format!(
                    "  {:<8} {:<4} {:>7} {:>8}  {}\n",
                    "LAP", "UAP", "pkts", "decoded", "channels"
                );
            }
            for p in pns.iter().take(50) {
                let ch: Vec<String> = p.channels.iter().map(|c| c.to_string()).collect();
                s += &format!(
                    "  {:06x}   {:<4} {:>7} {:>8}  {}{}\n",
                    p.lap,
                    match (p.uap, crate::bredr::is_inquiry_lap(p.lap)) {
                        (_, true) => "--".to_string(),
                        (Some(u), _) => format!("{u:02x}"),
                        (None, _) => "??".to_string(),
                    },
                    p.packets,
                    p.decoded,
                    ch.join(","),
                    if crate::bredr::is_inquiry_lap(p.lap) {
                        "  (inquiry)"
                    } else {
                        ""
                    }
                );
            }
            for (addr, d) in &self.br_devices {
                s += &format!(
                    "  FHS: BD_ADDR={addr} CoD=0x{:06x} seen {}x{}\n",
                    d.class_of_device,
                    d.seen,
                    d.name
                        .as_ref()
                        .map(|n| format!(" name=\"{n}\""))
                        .unwrap_or_default()
                );
            }
        }
        let _ = self.out.flush();
        eprint!("{s}");
    }
}

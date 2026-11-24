//! End-to-end test: synthesise a wideband capture with LE and BR traffic,
//! run it through the complete pipeline and check what comes out.

use btsdr::ble::BlePacket;
use btsdr::ble::decoder::BleConfig;
use btsdr::bredr::decoder::{BrConfig, BrKind, BrPacket};
use btsdr::pcap::PacketSink;
use btsdr::pipeline::{self, CaptureConfig};
use btsdr::report::Reporter;
use btsdr::source::MemorySource;
use btsdr::synth::{self, ScenarioInfo};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};

#[derive(Default)]
struct Collected {
    ble: Vec<BlePacket>,
    br: Vec<BrPacket>,
}

struct Collect(Arc<Mutex<Collected>>);

impl PacketSink for Collect {
    fn write_ble(&mut self, _ts: u64, p: &BlePacket) -> std::io::Result<()> {
        self.0.lock().unwrap().ble.push(p.clone());
        Ok(())
    }
    fn write_bredr(&mut self, _ts: u64, p: &BrPacket) -> std::io::Result<()> {
        self.0.lock().unwrap().br.push(p.clone());
        Ok(())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn run(snr_db: f64, seed: u64) -> (Collected, ScenarioInfo) {
    let (data, info) =
        synth::demo_scenario(2426, 20, 0.25, snr_db, seed, synth::Impairments::IDEAL);
    let cfg = CaptureConfig {
        center_mhz: 2426,
        sample_rate_mhz: 20,
        channels_mhz: CaptureConfig::default_channels(2426, 20),
        squelch_db: 8.0,
        ble: Some(BleConfig::default()),
        bredr: Some(BrConfig::default()),
        follow: vec![],
        max_duration: None,
        record: None,
        wall_start_ns: 0,
        status_interval: None,
    };
    let out = Arc::new(Mutex::new(Collected::default()));
    let mut rep = Reporter::new(Box::new(std::io::sink()), true, false);
    pipeline::run(
        Box::new(MemorySource::new(data)),
        cfg,
        vec![Box::new(Collect(out.clone()))],
        &mut rep,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    let c = std::mem::take(&mut *out.lock().unwrap());
    (c, info)
}

fn ratio(got: usize, expected: usize) -> f64 {
    got as f64 / expected.max(1) as f64
}

#[test]
fn decodes_demo_scenario() {
    let (c, info) = run(25.0, 7);

    // LE advertising, all with a valid CRC
    let adv = c
        .ble
        .iter()
        .filter(|p| p.is_advertising() && p.crc_valid)
        .count();
    assert!(
        adv + 1 >= info.adv_packets,
        "advertising {adv}/{}",
        info.adv_packets
    );

    // connection announced by CONNECT_IND
    let conn: Vec<_> = c
        .ble
        .iter()
        .filter(|p| p.access_address == info.conn_aa)
        .collect();
    assert!(
        ratio(conn.len(), info.conn_packets) > 0.95,
        "connection {}/{}",
        conn.len(),
        info.conn_packets
    );
    assert!(conn.iter().all(|p| p.crc_valid));
    // central and peripheral alternate within each event
    let c2p = conn
        .iter()
        .filter(|p| p.direction == btsdr::ble::Direction::CentralToPeripheral)
        .count();
    assert!(
        (c2p as i64 - (conn.len() / 2) as i64).abs() <= 2,
        "directions {c2p}/{}",
        conn.len()
    );

    // connection discovered promiscuously (no CONNECT_IND)
    let hidden = c
        .ble
        .iter()
        .filter(|p| p.access_address == info.hidden_aa)
        .count();
    assert!(
        ratio(hidden, info.hidden_packets) > 0.9,
        "hidden {hidden}/{}",
        info.hidden_packets
    );

    // BR piconet: UAP and clock discovered, headers de-whitened, CRCs valid
    let pico: Vec<_> = c.br.iter().filter(|p| p.lap == info.br_lap).collect();
    let decoded = pico
        .iter()
        .filter(|p| p.clk6.is_some() && p.hec_valid)
        .count();
    assert!(
        ratio(decoded, info.br_packets) > 0.95,
        "BR decoded {decoded}/{}",
        info.br_packets
    );
    assert!(
        pico.iter()
            .filter(|p| p.clk6.is_some())
            .all(|p| p.uap == Some(info.br_uap))
    );
    let crc_ok = pico.iter().filter(|p| p.crc_valid == Some(true)).count();
    assert!(
        crc_ok + 1 >= info.br_crc_packets,
        "BR CRC {crc_ok}/{}",
        info.br_crc_packets
    );

    // EDR: 2-DH1 / 2-DH3 (pi/4-DQPSK) and 3-DH1 (8DPSK) payloads with valid CRC
    let edr: Vec<_> = pico
        .iter()
        .filter(|p| p.rate > 0 && p.crc_valid == Some(true))
        .collect();
    assert!(
        ratio(edr.len(), info.br_edr_packets) > 0.9,
        "EDR {}/{}",
        edr.len(),
        info.br_edr_packets
    );
    for p in &edr {
        let expected_rate = if p.ptype() == Some(8) { 2 } else { 1 };
        assert_eq!(p.rate, expected_rate, "{}", p.describe());
        assert_eq!(p.transport, 0x30 | expected_rate);
    }

    // inquiry
    // inquiry
    let ids =
        c.br.iter()
            .filter(|p| p.kind == BrKind::Id && p.lap == btsdr::bredr::GIAC)
            .count();
    assert!(
        ratio(ids, info.inquiry_ids) > 0.9,
        "inquiry IDs {ids}/{}",
        info.inquiry_ids
    );
    let fhs: Vec<_> = c.br.iter().filter_map(|p| p.fhs()).collect();
    assert_eq!(fhs.len(), 1);
    assert_eq!(fhs[0].bd_addr(), info.fhs_bd_addr);
}

#[test]
fn degrades_gracefully_at_low_snr() {
    let (c, info) = run(14.0, 11);
    let adv = c
        .ble
        .iter()
        .filter(|p| p.is_advertising() && p.crc_valid)
        .count();
    assert!(
        ratio(adv, info.adv_packets) > 0.6,
        "advertising {adv}/{}",
        info.adv_packets
    );
    // no false CRC passes: every LE packet reported valid really is one of ours
    let known = [btsdr::ble::ADV_AA, info.conn_aa, info.hidden_aa];
    assert!(c.ble.iter().all(|p| known.contains(&p.access_address)));
    // the piconet must still be identified correctly
    assert!(
        c.br.iter()
            .filter(|p| p.lap == info.br_lap && p.clk6.is_some())
            .all(|p| p.uap == Some(info.br_uap))
    );
    let decoded =
        c.br.iter()
            .filter(|p| p.lap == info.br_lap && p.clk6.is_some())
            .count();
    assert!(
        ratio(decoded, info.br_packets) > 0.5,
        "BR decoded {decoded}/{}",
        info.br_packets
    );
}

/// A sweeping receiver retunes in the middle of the stream. Packets before
/// the retune belong to the old band and packets after it to the new one;
/// the settling samples after the retune are discarded, and the time base
/// continues across it.
#[test]
fn follows_a_retune() {
    use btsdr::ble;
    use btsdr::synth::{BLE_DEV, Scene, adv_ind, ble_packet_bits};
    let fs = 20e6;
    let step = 0.04; // seconds per sweep step
    let settle = 0.008; // seconds discarded after the retune
    let period = 0.004;
    let (addr_a, addr_b, addr_s) = (
        [0x0A, 0, 0, 0, 0, 0xC0],
        [0x0B, 0, 0, 0, 0, 0xC0],
        [0x05, 0, 0, 0, 0, 0xC0],
    );
    let adv = |addr, ch| ble_packet_bits(ble::ADV_AA, ble::ADV_CRC_INIT, &adv_ind(addr, "x"), ch);
    // step 1: centre 2426 MHz, advertising on channel 38 (2426 MHz)
    let mut a = Scene::new(fs, 2426, step);
    let mut times_a = Vec::new();
    let mut t = 0.002;
    while t < step - 0.002 {
        a.add(t, 2426, &adv(addr_a, 38), BLE_DEV, 0.25, 0.0);
        times_a.push(t);
        t += period;
    }
    // step 2: centre 2472 MHz, advertising on channel 39 (2480 MHz). The
    // packet inside the settling time (after the burst detectors' 2 ms
    // warm-up) would decode if it were not dropped.
    let mut b = Scene::new(fs, 2472, step);
    b.add(0.005, 2480, &adv(addr_s, 39), BLE_DEV, 0.25, 0.0);
    let mut times_b = Vec::new();
    let mut t = settle + 0.002;
    while t < step - 0.002 {
        b.add(t, 2480, &adv(addr_b, 39), BLE_DEV, 0.25, 0.0);
        times_b.push(step + t);
        t += period;
    }
    let noise_rms = 0.25 / 10f32.powf(25.0 / 20.0) * (fs as f32 / 1e6).sqrt();
    let mut data = a.to_ci8(noise_rms, 1);
    data.extend(b.to_ci8(noise_rms, 2));
    let source = MemorySource::new(data).retune_at((step * fs) as u64, 2472, (settle * fs) as u64);

    let cfg = CaptureConfig {
        center_mhz: 2426,
        sample_rate_mhz: 20,
        channels_mhz: CaptureConfig::default_channels(2426, 20),
        squelch_db: 8.0,
        ble: Some(BleConfig::default()),
        bredr: None,
        follow: vec![],
        max_duration: None,
        record: None,
        wall_start_ns: 0,
        status_interval: None,
    };
    let out = Arc::new(Mutex::new(Collected::default()));
    let mut rep = Reporter::new(Box::new(std::io::sink()), true, false);
    pipeline::run(
        Box::new(source),
        cfg,
        vec![Box::new(Collect(out.clone()))],
        &mut rep,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    let c = out.lock().unwrap();
    let from = |addr: [u8; 6]| -> Vec<&BlePacket> {
        c.ble
            .iter()
            .filter(|p| p.crc_valid && ble::adv_address(&p.pdu).is_some_and(|(a, _)| a == addr))
            .collect()
    };
    let (pa, pb) = (from(addr_a), from(addr_b));
    assert_eq!(pa.len(), times_a.len(), "packets before the retune");
    assert_eq!(pb.len(), times_b.len(), "packets after the retune");
    assert!(
        from(addr_s).is_empty(),
        "a packet in the settling time was decoded"
    );
    for (p, &t) in pa.iter().zip(&times_a) {
        assert_eq!(ble::mhz_from_rf_channel(p.rf_channel), 2426);
        assert!((p.time - t).abs() < 2e-6, "time {} vs {t}", p.time);
    }
    for (p, &t) in pb.iter().zip(&times_b) {
        assert_eq!(ble::mhz_from_rf_channel(p.rf_channel), 2480);
        assert!((p.time - t).abs() < 2e-6, "time {} vs {t}", p.time);
    }
}

/// A HackRF can lose a USB transfer without telling the host. The time base
/// then jumps by 131072 samples (6.55 ms at 20 Msps, about 10.5 slots), and
/// the predicted clock of every locked piconet is wrong. With only POLL and
/// NULL packets (or encrypted payloads) no CRC can confirm a clock, so the
/// decoder has to notice the failures and search for the clock again.
#[test]
fn relocks_after_lost_samples() {
    use btsdr::bredr;
    use btsdr::synth::{BR_DEV, Scene, XorShift, br_packet_bits};
    let fs = 20e6;
    let duration = 0.8;
    let cut_at = 0.4;
    let lost = 131072usize;
    let (lap, uap) = (0xB13350u32, 0xFBu8);
    let slot = 625e-6;
    let mut sc = Scene::new(fs, 2426, duration);
    let mut rng = XorShift(7);
    let freqs: Vec<u32> = (2418..=2434).collect();
    let mut sent_after = 0;
    // sniff-like traffic: master POLL every 20 slots, slave NULL right after;
    // every third exchange the master sends a 17-byte DM1 whose payload is
    // unreadable (encrypted): it decodes without FEC errors but fails the CRC,
    // and under the complementary clock its header reads as an FHS.
    let mut n = 4u32;
    let mut exchange = 0;
    while (n as f64 + 2.0) * slot < duration {
        let master = if exchange % 3 == 2 { 3u8 } else { 1 };
        exchange += 1;
        for (k, ptype) in [(0u32, master), (1, 0)] {
            let t = (n + k) as f64 * slot + 0.0001;
            let f = freqs[(rng.next_u64() % freqs.len() as u64) as usize];
            let clk6 = ((n + k) & 63) as u8;
            let mut bits = br_packet_bits(lap, uap, clk6, ptype, 1, false, 2, &[0; 17]);
            if ptype == 3 {
                let key: Vec<u8> = (0..160).map(|_| (rng.next_u64() & 1) as u8).collect();
                for (b, k) in bits[126..].iter_mut().zip(bredr::fec23_encode(&key)) {
                    *b ^= k;
                }
            }
            sc.add(t, f, &bits, BR_DEV, 0.25, 0.0);
            if t > cut_at + 0.01 {
                sent_after += 1;
            }
        }
        n += 20;
    }
    let noise_rms = 0.25 / 10f32.powf(25.0 / 20.0) * (fs as f32 / 1e6).sqrt();
    let mut data = sc.to_ci8(noise_rms, 3);
    let cut = 2 * (cut_at * fs) as usize;
    data.drain(cut..cut + 2 * lost);

    let cfg = CaptureConfig {
        center_mhz: 2426,
        sample_rate_mhz: 20,
        channels_mhz: CaptureConfig::default_channels(2426, 20),
        squelch_db: 8.0,
        ble: None,
        bredr: Some(BrConfig::default()),
        follow: vec![],
        max_duration: None,
        record: None,
        wall_start_ns: 0,
        status_interval: None,
    };
    let out = Arc::new(Mutex::new(Collected::default()));
    let mut rep = Reporter::new(Box::new(std::io::sink()), true, false);
    pipeline::run(
        Box::new(MemorySource::new(data)),
        cfg,
        vec![Box::new(Collect(out.clone()))],
        &mut rep,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    let c = out.lock().unwrap();
    let shift = lost as f64 / fs;
    let after: Vec<&BrPacket> =
        c.br.iter()
            .filter(|p| p.lap == lap && p.time > cut_at + 0.01 - shift)
            .collect();
    let decoded: Vec<&&BrPacket> = after.iter().filter(|p| p.ptype().is_some()).collect();
    for p in &decoded {
        assert_eq!(p.lt_addr(), 1, "wrong clock: {}", p.describe());
        assert!(
            matches!(p.ptype().unwrap(), 0 | 1 | 3),
            "wrong clock: {}",
            p.describe()
        );
    }
    assert!(
        after.len() + 2 >= sent_after,
        "{} of {sent_after} packets found",
        after.len()
    );
    assert!(
        ratio(decoded.len(), sent_after) >= 0.8,
        "only {} of {sent_after} packets decoded after the lost samples",
        decoded.len()
    );
}

#[test]
fn decodes_remote_id_adverts() {
    use btsdr::ble;
    use btsdr::synth::{
        BLE_DEV, Scene, ble_packet_bits, remote_id_adv, rid_basic_id, rid_location,
    };
    let fs = 20e6;
    let duration = 0.3;
    // a drone module alternating Basic ID and Location every 20 ms on channel 38
    let addr = [0x9C, 0x8B, 0x7A, 0x69, 0x58, 0xC7];
    let messages = [
        rid_basic_id(1, 2, "1581F5FKD229400EQ5"),
        rid_location(52.229676, 21.012229, 152.5, 45.0, 5.25, 270),
    ];
    let mut sc = Scene::new(fs, 2426, duration);
    let mut sent = 0usize;
    let mut t = 0.005;
    while t < duration - 0.005 {
        let pdu = remote_id_adv(addr, sent as u8, &messages[sent % 2]);
        let bits = ble_packet_bits(ble::ADV_AA, ble::ADV_CRC_INIT, &pdu, 38);
        sc.add(t, 2426, &bits, BLE_DEV, 0.25, 0.0);
        sent += 1;
        t += 0.020;
    }
    let noise_rms = 0.25 / 10f32.powf(20.0 / 20.0) * (fs as f32 / 1e6).sqrt();
    let data = sc.to_ci8(noise_rms, 5);

    let cfg = CaptureConfig {
        center_mhz: 2426,
        sample_rate_mhz: 20,
        channels_mhz: CaptureConfig::default_channels(2426, 20),
        squelch_db: 8.0,
        ble: Some(BleConfig::default()),
        bredr: None,
        follow: vec![],
        max_duration: None,
        record: None,
        wall_start_ns: 0,
        status_interval: None,
    };
    let out = Arc::new(Mutex::new(Collected::default()));
    let mut rep = Reporter::new(Box::new(std::io::sink()), true, false);
    pipeline::run(
        Box::new(MemorySource::new(data)),
        cfg,
        vec![Box::new(Collect(out.clone()))],
        &mut rep,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    let c = out.lock().unwrap();
    let lines: Vec<String> = c
        .ble
        .iter()
        .filter(|p| p.crc_valid)
        .map(|p| p.describe())
        .collect();
    let count = |s: &str| lines.iter().filter(|l| l.contains(s)).count();
    let ids = count(" rid_id=\"1581F5FKD229400EQ5\" rid_id_type=serial rid_ua=multirotor");
    let positions = count(" rid_status=airborne rid_pos=52.2296760,21.0122290 rid_alt=152.5m");
    assert!(
        ids + positions + 1 >= sent && ids > 0 && positions > 0,
        "{ids} Basic ID and {positions} Location of {sent} adverts decoded: {lines:?}"
    );
}

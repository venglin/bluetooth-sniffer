# btsdr

[![CI](https://github.com/venglin/bluetooth-sniffer/actions/workflows/ci.yml/badge.svg)](https://github.com/venglin/bluetooth-sniffer/actions/workflows/ci.yml)

**Bluetooth LE and Bluetooth Classic (BR) sniffer for the HackRF One, with PCAP/PCAPNG output for Wireshark.**

btsdr captures up to 20 MHz of the 2.4 GHz band at once, splits it into
1 MHz Bluetooth channels and decodes every packet that lands on any of them.
It doesn't follow frequency hopping. It listens to up to 19 channels at the
same time, which is enough to see advertising, ongoing LE connections and
Classic piconets in that slice of the band. Decoded packets go to the console
and to capture files that Wireshark dissects natively.

```
HackRF One ──► channelizer (PFB) ──► burst detector ──► GFSK demod ──┬─► LE link layer ──┐
 20 Msps        17-19 × 1 MHz         per channel        per burst    └─► BR baseband ────┴─► console
                                                                                            PCAPNG / PCAP
```

## Features

**Wideband capture**
- Polyphase filter-bank channelizer: one HackRF covers up to 19 channels
  (default 17) simultaneously.
- Packets from every monitored channel are decoded, including Classic
  piconets hopping at 1600 hops/s and LE connections hopping on all 37 data
  channels.
- Wi-Fi and other wideband interference is detected and discarded early.
- Band sweep (`--sweep`): the HackRF retunes through the whole 2.4 GHz band,
  and piconet clocks and LE connections are kept across the steps.

**Bluetooth Low Energy (LE 1M PHY)**
- Advertising on the primary and secondary channels: `ADV_IND`,
  `ADV_NONCONN_IND`, `SCAN_REQ`/`SCAN_RSP`, `CONNECT_IND`, `ADV_EXT_IND`/`AUX_*`.
- Connections announced by a `CONNECT_IND` are followed by access address and
  CRCInit on every monitored data channel.
- Connections established *before* the capture started are discovered
  promiscuously: the CRCInit is recovered by running the CRC backwards.
- CRC-24 verification on every packet, plus central/peripheral direction
  inference from the 150 µs inter-frame spacing.
- Soft-decision CRC repair: packets with up to two bit errors are fixed
  using the least reliable bits.
- Device names, company IDs and service UUIDs are extracted from
  advertising data.
- Drone Remote ID (ASTM F3411, EN 4709-002) broadcast over Bluetooth 4 is
  decoded: the UAS ID and aircraft type, position, altitude, speed and
  track, the operator's position and ID, and the self-ID text, e.g.
  `rid_id="1581F5FKD229400EQ5" rid_id_type=serial rid_ua=multirotor`.
  The summary lists each drone with its latest messages.

**Bluetooth Classic (BR/EDR)**
- The LAP is read directly from the access code: a BCH(64,30) sync word with
  error correction.
- Inquiry (GIAC/DIAC) and page ID packets.
- The UAP and master clock (CLK6-1) of each piconet are discovered
  automatically, so headers are de-whitened and their HEC verified. When a
  piconet's clock stops fitting, for example after samples were lost, it is
  searched for again.
- Payload decoding with CRC check for FHS, DM1, DH1, DM3, DH3, DM5 and DH5.
- Soft-decision FEC: soft majority voting for the header and Chase decoding
  for the rate-2/3 payload code.
- An FHS packet (inquiry or page response) reveals the full BD_ADDR, class of
  device and clock.
- EDR payload demodulation: π/4-DQPSK (2 Mbit/s) and 8DPSK (3 Mbit/s) for the
  ACL packets 2-DH1, 3-DH1, 2-DH3, 3-DH3, 2-DH5 and 3-DH5, with CRC check.

**Output**
- PCAPNG with both link types in one file, or separate PCAP files:
  `LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR` (256) and `LINKTYPE_BLUETOOTH_BREDR_BB` (255).
- Live streaming into Wireshark through stdout.
- Wireshark extcap plugin: choose the HackRF in Wireshark's interface list,
  configure it in the GUI and start or stop captures from there.
- Raw IQ recording and offline re-analysis of `ci8`, `cu8`, `ci16` and
  `cf32` files.
- A built-in synthetic signal generator lets you try everything without
  hardware.

**Portable**
- Windows, Linux and macOS.
- libhackrf is loaded at run time, so no SDK is needed to build.

## Hardware

- **HackRF One.** Tested with firmware 2023.01.1 and libhackrf 0.6.
- **A 2.4 GHz antenna.** The ANT500 works; a Wi-Fi antenna works better.
- **A USB 2.0 port** that sustains 20 Msps (40 MB/s).
- **A reasonably modern CPU.** Decoding 17 channels at 20 Msps runs at about
  2× real time on a current desktop.

## Installation

### 1. Install libhackrf and check the device

| OS | How |
|----|-----|
| Windows | Install [PothosSDR](https://downloads.myriadrf.org/builds/PothosSDR/) or [radioconda](https://github.com/ryanvolz/radioconda) (both ship `hackrf.dll` and `libusb-1.0.dll`). If `hackrf_info` cannot open the device, install the WinUSB driver for "HackRF One" with [Zadig](https://zadig.akeo.ie/). |
| Debian/Ubuntu | `sudo apt install hackrf` (installs `libhackrf0` and udev rules; log in again after adding yourself to the `plugdev` group if needed) |
| Fedora | `sudo dnf install hackrf` |
| macOS | `brew install hackrf` |

Verify with `hackrf_info`. btsdr looks for the library next to its own
executable, on the system search path, in the PothosSDR/radioconda/Homebrew
locations, and in `$HACKRF_LIB` or `--hackrf-lib <path>`.

### 2. Build btsdr

You need [Rust](https://rustup.rs/) 1.85 or newer. On Windows use the MSVC
toolchain, which needs the *Visual Studio Build Tools* with the
"Desktop development with C++" / VC tools workload.

```bash
git clone https://github.com/venglin/bluetooth-sniffer.git
cd bluetooth-sniffer
cargo build --release
```

The binary is `target/release/btsdr` (`btsdr.exe` on Windows). Run
`cargo install --path .` to put it on your `PATH`.

## Quick start

List devices:

```bash
btsdr devices
```

Capture for 60 s around 2426 MHz (LE advertising channel 38 plus its neighbours):

```bash
btsdr capture -t 60 -w capture.pcapng
```

Watch packets live in Wireshark:

```bash
btsdr capture -w - | wireshark -k -i -
```

On Windows, run the live pipe from `cmd.exe` or PowerShell 7.4+. Windows
PowerShell 5.1 corrupts binary data in pipelines.

No HackRF at hand? Generate a synthetic recording and decode it:

```bash
btsdr synth -o demo.ci8
```

```bash
btsdr capture -i demo.ci8 -w demo.pcapng
```

Example console output (from the synthetic recording):

```
   0.012500 BLE 2426 MHz ch38  -12 dB AA=8e89bed6 CONNECT_IND InitA=46:05:04:03:02:01 AdvA=c1:22:33:44:55:66 AA=50654a3c CRCInit=7a3b5c interval=7.50ms hop=7
   0.014500 BLE 2418 MHz ch7   -12 dB AA=50654a3c C->P LL Empty PDU SN=0 NESN=0 MD=0
   0.014730 BLE 2418 MHz ch7   -14 dB AA=50654a3c P->C LL Data (start) ATT cid=0x0004 l2len=3 SN=0 NESN=1 MD=0
# BLE discovered connection AA=6a7c4f31 CRCInit=13579b (promiscuous)
   0.016000 BR  2429 MHz ch27  -13 dB LAP=9e8b33 ID (GIAC inquiry)
# BR piconet LAP=a1b2c3: UAP=5d and clock discovered after 2 packets
   0.016975 BR  2419 MHz ch17  -12 dB LAP=a1b2c3 UAP=5d clk6=53 LT_ADDR=1 DH1 F-- len=20 L2CAP start CRC ok
   0.017600 BR  2434 MHz ch32  -15 dB LAP=a1b2c3 UAP=5d clk6=54 LT_ADDR=1 NULL F--
   0.022000 BLE 2426 MHz ch38  -12 dB AA=8e89bed6 ADV_IND AdvA=c1:22:33:44:55:66 (random) name="btsdr-demo"
```

Each line shows:
- capture time in seconds;
- PHY (`BLE` or `BR`), centre frequency and channel number (LE channel index
  or BR RF channel);
- signal level in dBFS;
- the decoded packet.

The BR flags `F`, `A` and `S` are FLOW, ARQN and SEQN. Lines starting with
`#` are events. A summary of devices, connections and piconets is printed at
the end.

## Capturing directly from Wireshark (extcap)

btsdr can register itself as a Wireshark *extcap* interface, so HackRF
captures are started, configured and stopped from Wireshark itself:

```bash
btsdr extcap install
```

This copies the btsdr executable into Wireshark's personal extcap folder.
btsdr asks Wireshark for the location with `tshark -G folders`; typical
locations are `%APPDATA%\Wireshark\extcap` on Windows and
`~/.local/lib/wireshark/extcap` on Linux and macOS (Wireshark 4.4+).

Restart Wireshark or use *Capture > Refresh Interfaces*. Three interfaces
appear:

| Interface | Captures |
|-----------|----------|
| Bluetooth LE + BR/EDR sniffer (HackRF One) | both, as two interfaces in one PCAPNG stream |
| Bluetooth LE sniffer (HackRF One) | LE only |
| Bluetooth BR/EDR sniffer (HackRF One) | BR only |

The gear icon next to an interface opens its settings:
- centre frequency, sample rate and channel list, or a sweep of the whole
  band with its dwell time;
- LNA/VGA gain and the RF amplifier;
- squelch and decoder options;
- HackRF serial number, bias tee, libhackrf path and an optional raw IQ
  recording.

Start the capture as usual. Stopping it in Wireshark stops btsdr and
releases the HackRF.

The same works from the command line:

```bash
tshark -i btsdr -o extcap.btsdr.center:2472 -o extcap.btsdr.lna:24
```

Notes:
- **Updates:** the extcap folder holds a *copy* of the binary. Run
  `btsdr extcap install` again after updating btsdr.
- **Other commands:** `btsdr extcap uninstall` removes the copy and
  `btsdr extcap path` shows the folder. `--dir <folder>` installs elsewhere,
  for example into the global extcap folder.
- **tshark preferences:** for the LE-only and BR-only interfaces they are
  named `extcap.btsdr_le.*` and `extcap.btsdr_bredr.*`.
- **Filters:** capture filters are not supported; use display filters.
- **Errors:** if the HackRF cannot be opened, Wireshark shows btsdr's error
  message. On Linux the user running Wireshark needs access to the HackRF
  (udev rules, `plugdev` group).
- **Manual installation:** copy the btsdr executable into the folder shown
  under *Help > About Wireshark > Folders > Personal Extcap path*.

## Choosing the frequency

A HackRF sees at most 20 MHz, so pick the slice of the band you care about.
The centre frequency must be an integer number of MHz.

| `-f` (MHz) | Channels decoded (default) | Contains |
|-----------:|----------------------------|----------|
| 2410 | 2402–2418 MHz | LE advertising ch 37, LE data 0–7, BR 0–16 |
| 2426 (default) | 2418–2434 MHz | LE advertising ch 38, LE data 7–14, BR 16–32 |
| 2441 | 2433–2449 MHz | LE data 14–21, BR 31–47 |
| 2472 | 2464–2480 MHz | LE advertising ch 39, LE data 29–36, BR 62–78 |

- Channels closer than 2 MHz to the band edge are skipped by default because
  the HackRF's anti-aliasing filter attenuates them. Override with
  `-c 2420-2432` or `-c 2402,2426,2480`.
- A lower sample rate (`-s 10`) monitors fewer channels with less CPU.
- A Classic piconet hops over all 79 channels, so a 17-channel window sees
  about a fifth of its packets. That is enough to identify the piconet,
  discover its UAP and clock, and decode what you see.
- An LE connection is visible whenever one of its connection events falls
  into the window.

### Sweeping the whole band

To see everything around you, let the HackRF retune through the band:

```bash
btsdr capture --sweep -w sweep.pcapng
```

At 20 Msps this visits 2410, 2427, 2444, 2461 and 2472 MHz, 2 s each
(`--dwell`), and covers all 79 channels. The first, second and last steps
contain the LE advertising channels.

- **Trade-off:** each channel is monitored only a fifth of the time. A fixed
  window is better for following one device; the sweep gives an overview of
  all devices.
- **State is kept:** one decoder runs across all steps, so a piconet's UAP and
  clock and an LE connection's parameters survive a retune. The time base
  stays continuous.
- **Settling time:** after each retune about 35 ms of samples are discarded
  at 20 Msps. These are buffered samples from the old frequency and the
  synthesizer settling. At lower sample rates up to about 0.2 s is
  discarded, so `--dwell` must be at least 0.5 s.
- **Custom steps:** `--sweep 2410,2427,2472` visits only the given centre
  frequencies.
- **Limits:** a sweep needs a live HackRF. It cannot be combined with
  `--record`, `--center` or `--channels`.
- **Wireshark:** the extcap settings have the same option ("Sweep the whole
  band").

## Command reference

### `btsdr capture`

Capture and decode, live from a HackRF or from a file (`-i`).

| Option | Default | Description |
|--------|---------|-------------|
| `-f, --center <MHz>` | 2426 | Centre frequency |
| `-s, --rate <Msps>` | 20 | Sample rate, an even number from 4 to 20; one channel per Msps |
| `-c, --channels <list>` | auto | Channels to decode, e.g. `2402,2426,2430-2434` |
| `--sweep [MHz,...]` | | Sweep the whole band, or the given centre frequencies |
| `--dwell <s>` | 2 | Time on each centre frequency when sweeping (at least 0.5) |
| `-w, --pcapng <file>` | | Write LE and BR packets to one PCAPNG file (`-` = stdout) |
| `--ble-pcap <file>` | | LE only, PCAP with link type 256 |
| `--bredr-pcap <file>` | | BR only, PCAP with link type 255 |
| `--record <file>` | | Save the raw IQ stream (signed 8-bit) for later |
| `-i, --input <file>` | | Decode an IQ recording instead of a live HackRF |
| `--format <fmt>` | auto | `ci8`, `cu8`, `ci16`, `cf32` (guessed from the extension) |
| `-t, --duration <s>` | | Stop after this many seconds (Ctrl-C stops any time) |
| `--lna <dB>` | 32 | LNA (IF) gain, 0–40 in steps of 8 |
| `--vga <dB>` | 20 | VGA (baseband) gain, 0–62 in steps of 2 |
| `--amp` | off | RF amplifier (+14 dB) |
| `--bias-tee` | off | Antenna port power |
| `--bb-filter <MHz>` | 0.75 × rate | Baseband filter bandwidth |
| `--serial <sn>` | | Select a HackRF by serial number |
| `--hackrf-lib <path>` | | Explicit path to libhackrf |
| `--squelch <dB>` | 8 | Burst detection threshold above the noise floor |
| `--no-ble`, `--no-bredr` | | Disable one of the decoders |
| `--no-promiscuous` | | Don't discover LE connections without a `CONNECT_IND` |
| `--follow <AA:CRCINIT>` | | Follow a known LE connection (hex, repeatable) |
| `--uap <LAP:UAP>` | | Known UAP of a BR piconet (hex, repeatable) |
| `--max-ac-errors <n>` | 2 | Bit errors corrected in a BR sync word |
| `--max-aa-errors <n>` | 2 | Bit errors tolerated in an LE access address |
| `--keep-bad-crc` | | Also output LE packets with a bad CRC |
| `--hard-decisions` | | Disable soft-decision decoding (LE CRC repair, BR soft FEC), for comparison |
| `--rssi-offset <dB>` | 0 | Calibration offset added to signal/noise levels |
| `-q, --quiet` | | Print only the summary |
| `--status <s>` | 5 | Status line interval on stderr (0 = off) |

### `btsdr devices`

Lists the connected HackRF devices and shows which libhackrf was loaded.

### `btsdr synth`

Writes a synthetic `ci8` recording containing:
- LE advertising;
- a `CONNECT_IND` and the connection it announces;
- a second, pre-existing connection;
- a Classic piconet with POLL, NULL, DM1, DH1 and EDR (2-DH1, 3-DH1,
  2-DH3) packets;
- inquiry ID packets and an FHS response.

Options: `-o <file>`, `-f <MHz>`, `-s <Msps>`, `-t <seconds>`, `--snr <dB>`,
`--seed <n>`, and `--impaired` to add carrier offset and drift, modulation
index spread and phase noise.

### `btsdr extcap`

| Command | Description |
|---------|-------------|
| `btsdr extcap install [--dir <folder>]` | Copy btsdr into Wireshark's personal extcap folder |
| `btsdr extcap uninstall [--dir <folder>]` | Remove it again |
| `btsdr extcap path` | Show the extcap folder |

When Wireshark starts btsdr with `--extcap-*` options, btsdr speaks the
extcap protocol instead of its normal command line.

## Working with the captures in Wireshark

Open the `.pcapng` file directly; no plugins are needed. Useful display filters:

| Filter | Shows |
|--------|-------|
| `btle` | all LE packets |
| `btle.advertising_header.pdu_type == 0x05` | `CONNECT_IND` |
| `btle.access_address == 0x50654a3c` | one LE connection |
| `btatt` / `btsmp` / `btl2cap` | LE upper layers |
| `btbredr_rf` | all BR packets |
| `btbredr_rf.lower_address_part == 0x9e8b33` | general inquiry (GIAC) |
| `btbredr_fhs` | FHS packets (BD_ADDR, class of device) |
| `btbredr_rf.flags.crc_pass == 1` | BR packets with a verified payload |

Notes on the pseudo-headers:
- **Signal and noise levels** are in dBFS, not dBm. Use `--rssi-offset` if
  you have a calibration.
- **LE packets** are written de-whitened with the CRC check result. Data
  packets also carry the inferred direction (central → peripheral or
  peripheral → central).
- **BR packets** carry the LAP, the UAP (once known), the de-whitened header
  (once the clock is known) and, when the CRC passes, the de-whitened and
  FEC-decoded payload. Wireshark then dissects LMP and L2CAP.
- **EDR packets** with a valid CRC are marked "Enhanced Data Rate, ACL" with
  the 2 or 3 Mbit/s modulation. Wireshark names them 2-DH1 … 3-DH5 and
  dissects their payload. When the EDR modulation is found but the CRC does
  not match (usually an encrypted link), only the header is written, with
  the EDR rate set.
- **BR packets captured before their piconet's UAP was discovered** are kept
  back for a few seconds and written decoded afterwards. Their timestamps
  stay correct, but they can appear slightly out of order in the file.

## Tuning tips

- **Too few packets:** raise `--vga` in steps of 4–6 dB, then `--lna`. Try
  `--amp` only for weak signals.
- **"ADC CLIPPING" in the status line, or many bad CRCs on strong signals
  (above -10 dBFS):** a transmitter close to the antenna saturates the
  HackRF's 8-bit ADC. This distorts the whole band, including weak signals,
  and EDR (8DPSK) suffers first. Lower the gain (e.g. `--lna 16 --vga 16`)
  or move the antenna away.
- **"samples DROPPED" in the summary:** the machine can't keep up. Reduce the
  sample rate (`-s 10`) or the channel list.
- **Busy Wi-Fi channel inside your window:** consider another centre
  frequency. Wideband bursts are discarded automatically, but they still
  cost air time.
- **Repeatable experiments:** record the IQ once with `--record` and re-run
  the decoder offline with different settings.

## Sensitivity

`examples/benchmark.rs` measures the fraction of packets decoded versus SNR
on synthetic captures. SNR is per 1 MHz channel, which equals Eb/N0 at
1 Mbit/s. The channel is impaired within the specification limits: ±100 kHz
carrier offset, up to 20 Hz/µs drift, ±10% modulation index spread and phase
noise.

```bash
cargo run --release --example benchmark -- --impaired
```

SNR at which 50% of the packets are decoded (lower is better):

| Packets | v0.1.0 (hard decisions) | current |
|---------|------------------------:|--------:|
| LE advertising | 11.4 dB | 9.2 dB |
| LE data (empty PDUs and short payloads) | 9.6 dB | 8.6 dB |
| BR header de-whitened (piconet locked) | 13.4 dB | 10.0 dB |
| BR payload with valid CRC | 14.0 dB | 12.8 dB |
| BR inquiry ID | 10.9 dB | 9.9 dB |

The gains come from three changes:
- **A narrower channel filter for GFSK** (±0.6 MHz instead of ±0.8 MHz).
- **Soft decisions.** CRC repair for LE; soft majority and Chase decoding of
  the FEC for BR.
- **A more robust piconet lock** for BR.

The channelizer itself is ±0.7 MHz wide, so that EDR signals pass. This
costs GFSK about 0.3 dB compared with a ±0.6 MHz channelizer.

EDR payloads, measured with `--edr` on a scenario with only EDR traffic and
maximum-length payloads:

| Packet | Modulation | 50% decoded at |
|--------|------------|---------------:|
| 2-DH1 / 2-DH3 / 2-DH5 | π/4-DQPSK | 13.0 / 14.1 / 14.6 dB |
| 3-DH1 / 3-DH3 / 3-DH5 | 8DPSK | 18.9 / 20.8 / 21.1 dB |

This is within about 1.5–2 dB of ideal differential detection. 8DPSK needs a
strong signal, as in any receiver.

On two indoor recordings, the same changes increased the number of
CRC-valid LE advertising packets by 13%, and cut the remaining bad-CRC
packets by 55–60%.

At high SNR the curves saturate at 96–99%, because the synthetic scenario
contains overlapping transmissions.

## Limitations

- **LE:** only the LE 1M PHY. LE 2M and LE Coded are not demodulated.
- **Remote ID:** only the Bluetooth 4 (legacy advertising) broadcast. The
  same messages sent over Bluetooth 5 Long Range (LE Coded) or Wi-Fi are not
  received. Authentication messages are reported with their page number
  only, and signatures are not checked.
- **Classic EDR:**
  - ACL payloads (2-DHx, 3-DHx) are demodulated, but eSCO EDR (2-EVx, 3-EVx)
    and SCO/eSCO voice payloads are not; their headers are.
  - Symbol timing is not tracked within an EDR packet. Very long 8DPSK
    packets from a transmitter with a large clock error may lose bits
    towards the end.
  - Encrypted EDR traffic (most audio streams) cannot be CRC-checked. It is
    reported with its header and the EDR rate only.
  - In tests, one nearby device sent EDR-typed packets whose DPSK part did
    not match the standard synchronisation sequence. btsdr reports such
    packets with their header only.
  - Unencrypted EDR traffic for a test can be generated with BlueZ, e.g.
    `l2ping -s 600 <bdaddr>`.
- **No hop following:** btsdr monitors a fixed window of up to 20 MHz, or
  sweeps the band in steps (`--sweep`). It does not follow the hopping of
  one piconet or connection, so traffic outside the current window is not
  seen.
- **No decryption:** encrypted payloads stay encrypted. On E0-encrypted
  links the CRC cannot be checked either, so such BR/EDR packets are written
  with their header only. A valid BR CRC
  normally means the payload is readable, so such packets are marked
  "decrypted" so that Wireshark dissects them.
- **UAP discovery needs data:** a piconet's UAP and clock are found after a
  few packets (one CRC-protected packet plus one more is usually enough).
  Until then its packets are reported with the LAP only.
- **Timestamps** are the host clock at start plus the sample count, so
  relative timing is precise to well under a microsecond.
- **Silently lost samples:** a HackRF can lose samples without telling the
  host, e.g. a whole USB transfer (131072 samples, 6.55 ms at 20 Msps).
  Later timestamps are then early by that amount. Locked piconets lose
  their clock, notice it after a few packets and search for it again
  (`clock lost …, searching again`), which usually takes 0.5–10 s. In an
  8-hour sweep this happened about twice an hour.
- **Validation:** the following has been verified on live traffic with a
  HackRF One:
  - LE advertising, and LE connections both followed from `CONNECT_IND` and
    discovered promiscuously;
  - Classic inquiry;
  - piconet UAP/clock discovery and BR payloads with valid CRC;
  - EDR 2-DH1 (π/4-DQPSK) and 3-DH1 (8DPSK) packets with valid CRC and
    readable L2CAP;
  - the band sweep: in a 40 s test all 1063 decoded packets were assigned to
    the channels of the step they were received in.

  The end-to-end tests with synthetic signals and the codec cross-checks
  against reference tables cover the rest. Reports from other environments
  are welcome.

## How it works

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the signal chain:
- the channelizer math;
- burst detection and the GFSK demodulator;
- LE connection discovery;
- Classic UAP/clock recovery;
- the exact capture formats.

## Development

Run the tests:

```bash
cargo test --release
```

The test suite includes:
- **Codec unit tests:** sync words, whitening, FEC, HEC and CRC, cross-checked
  against libbtbb tables, Wireshark's checks and a reference LE CRC.
- **Channelizer and detector tests.**
- **An end-to-end test** that synthesises LE and BR traffic and runs it
  through the whole pipeline.

`cargo run --release --example benchmark` measures sensitivity; see
[Sensitivity](#sensitivity).

Source layout:

```
src/
  main.rs            command line
  pipeline.rs        threads: source -> DSP -> decoders -> outputs
  source/            HackRF (run-time loaded libhackrf), IQ files
  dsp/               channelizer, burst detector, GFSK demodulator, filters
  ble/               LE link layer: codec, connection tracking
  bredr/             BR baseband: codec, piconet UAP/clock discovery
  pcap.rs            PCAP / PCAPNG writers
  extcap.rs          Wireshark extcap interface and installer
  report.rs          console output and summary
  synth.rs           packet encoders, GFSK modulator, demo scenario
tests/synthetic.rs   end-to-end test
examples/benchmark.rs  packet success rate vs SNR
```

## Legal

btsdr is a passive receiver. Bluetooth traffic can contain personal data, and
intercepting communications you are not a party to may be illegal where you
live. Only capture traffic you are authorised to analyse, and handle captures
in line with applicable privacy law.

## Acknowledgements

- **Bluetooth Core Specification**: Vol 2 Part B (Baseband) and Vol 6 Part B
  (LE Link Layer).
- **Wireshark's `btle_rf` / `btbredr_rf` dissectors** and the
  [tcpdump.org link-type registry](https://www.tcpdump.org/linktypes.html)
  define the capture formats.
- **[libbtbb](https://github.com/greatscottgadgets/libbtbb) / Ubertooth**:
  their published tables are used as independent test vectors.
- **[ice9-bluetooth-sniffer](https://github.com/mikeryan/ice9-bluetooth-sniffer)**
  showed that wideband SDR sniffing with a filter bank is practical.

## License

MIT, see [LICENSE](LICENSE).

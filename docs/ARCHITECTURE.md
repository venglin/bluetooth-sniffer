# btsdr architecture

This document describes the signal chain and the protocol logic of btsdr in
enough detail to review or extend it. Section and module names refer to the
source tree.

```
 ┌────────────┐  IqBlock   ┌──────────────────────────────┐  Burst   ┌──────────────────────────────────────┐
 │ source     │──────────► │ DSP thread                   │────────► │ decoder thread                        │
 │ HackRF/file│  (8-bit IQ)│ DC removal → PFB channelizer │          │ GFSK demod → BR decoder → LE decoder  │
 └────────────┘            │ → burst detectors            │          │ → console, PCAPNG/PCAP sinks          │
                           │ → wideband (Wi-Fi) rejection │          └──────────────────────────────────────┘
                           └──────────────────────────────┘
```

Three threads are connected by bounded channels (`pipeline.rs`):

- **Source.** The HackRF callback copies each USB buffer and `try_send`s it.
  If the queue is full, the buffer is dropped and a `Gap(n)` marker is sent
  ahead of the next block. The time base stays exact and drops are reported
  at the end.
- **DSP.** Channelizes, detects bursts, sorts them by onset time and discards
  wideband interference.
- **Decoder.** Demodulates each burst, runs the link-layer decoders and
  writes the output.

All timestamps are derived from the sample counter: `t = n / fs`, corrected
for filter delays. They are converted to wall-clock time only when written
to a capture file.

## 1. Source (`source/`)

`hackrf.rs` loads libhackrf at run time (`dynlib.rs`: `LoadLibraryExW` or
`dlopen`) and binds about 20 functions. On Windows the DLL is loaded with
`LOAD_WITH_ALTERED_SEARCH_PATH`, so `libusb-1.0.dll` next to `hackrf.dll` is
found. Device settings:
- sample rate;
- baseband filter (`hackrf_compute_baseband_filter_bw(0.75·fs)` unless
  overridden);
- frequency;
- LNA, VGA and amp gain, and the bias tee.

The callback context outlives `hackrf_close()`, which joins the USB thread.

### Band sweep

With a `Sweep` (centre frequencies and a dwell time) the source thread
retunes the HackRF while it streams:

- **Schedule:** it follows the sample counter on a fixed grid, so step `k`
  starts at sample `k · dwell · fs`.
- **Retune order:** the source first marks a retune as pending and then
  calls `hackrf_set_freq`. The callback puts a
  `Retune { center_mhz, settle }` marker in the stream, in front of the
  first USB buffer it delivers after that.
- **Why that order:** every sample from the new frequency comes after the
  marker. The samples that follow the marker can still be from the old
  frequency: the buffer being filled, and any filled buffers not yet
  delivered (at most the 4 libhackrf transfers).
- **`settle`:** 5 buffers plus 1 ms of synthesizer settling, about 34 ms at
  20 Msps (buffers are 131072 samples).
- **Measured:** alternating between an LTE downlink and the quiet Bluetooth
  band, the power changed within the first buffer after the marker, and
  never before it.

On a marker the DSP thread:
- selects the PFB bins of the new channel set
  (`CaptureConfig::default_channels` of the new centre);
- resets the DC estimate;
- drops the filter history (`skip(0)`);
- creates new burst detectors with the new frequencies;
- skips the next `settle` input samples like a gap.

The sample counter, and with it the time base, continues across the retune.
The decoders are not touched, so piconet clocks, UAPs and LE connections
survive the steps. `pipeline::sweep_centers` chooses the steps: as few
centres as needed to cover 2402–2480 MHz with full channel sets, e.g. 2410,
2427, 2444, 2461 and 2472 MHz at 20 Msps.

`file.rs` reads `ci8` (HackRF native), `cu8`, `ci16` and `cf32` recordings.

The DSP thread also counts samples whose I or Q is at full scale. ADC
clipping from a strong transmitter near the antenna distorts the whole band
(PSK first), so the status line and the summary warn about it.

## 2. Channelizer (`dsp/channelizer.rs`)

The input of `M = fs / 1 MHz` complex samples per microsecond is split into
`M` channels spaced 1 MHz apart. Each channel is produced at `2 MHz`
(oversampling factor `os = 2`, decimation `D = M/2`). For the newest input
index `t` of each window:

```
y_k[t] = Σ_l h[l] · x[t−l] · e^{−j2πk(t−l)/M}
       = e^{−j2πk t/M} · Σ_m e^{+j2πkm/M} · u_m,      u_m = Σ_p h[pM+m] · x[t−pM−m]
```

So each output vector costs:
- one dot product of length `L = 16·M` with the prototype filter `h`, folded
  into `M` partial sums `u_m`;
- one `M`-point inverse FFT (rustfft);
- a per-channel phase rotation that depends on `t mod M`.

Only the bins that map to selected channels are kept.

- **Prototype filter:** Kaiser-windowed sinc, 65 dB stop band, −6 dB at
  ±0.7 MHz, which passes the EDR spectrum (root-raised cosine, roll-off 0.4,
  ±0.7 MHz). The GFSK demodulator narrows the band further to ±0.6 MHz (see
  below). Against a single ±0.6 MHz filter this costs GFSK ~0.3 dB of
  burst-detection sensitivity, and gains 8DPSK about 1 dB. The output
  Nyquist band is 2 MHz.
- **DC removal:** the HackRF LO leakage would land in the centre channel.
  It is removed with a block-mean estimate smoothed across blocks.
- **Edge channels:** channels within 2 MHz of the band edge are skipped by
  default because of the analogue anti-aliasing filter.

## 3. Burst detection (`dsp/burst.rs`)

One detector runs per channel on `|y|²`:

- **Noise floor:** initialised from the first 2 ms, then tracked with a slow
  average of quiet samples only.
- **Start:** a burst starts when the smoothed power (IIR, α = 0.3) exceeds
  `noise × squelch` (default +8 dB). The onset is then refined backwards
  using the instantaneous power, because the smoothed power lags.
- **End:** the burst ends after 6 µs below half the threshold (hysteresis).
  The end of the "core" is the last sample whose *instantaneous* power was
  above the threshold; GFSK has a constant envelope, so this edge is sharp.
  An accurate end matters: the LE promiscuous mode and the BR packet-type
  check compare the decoded length with it.
- **Padding:** 10 µs before and 6 µs after the core are kept.
- **Length limits:** bursts shorter than 50 µs are dropped. Bursts longer
  than 3.2 ms are treated as continuous interference and raise the noise
  floor.

### Wideband rejection (`pipeline::wideband_filter`)

Wi-Fi frames switch on in many channels within the same microsecond with a
flat spectrum. A strong Bluetooth packet also leaks into neighbouring
channels, but its power falls off quickly away from its own channel.

Bursts with onsets within 2 µs of each other are grouped. A group is dropped
when at least 5 of its bursts are within 6 dB of the strongest one. In a
typical indoor capture this removes about 40% of all bursts and loses no
Bluetooth packets.

## 4. GFSK demodulation (`dsp/demod.rs`)

LE 1M and BR both use 1 Msym/s GFSK (BT = 0.5). The modulation index is
h ≈ 0.5 for LE and 0.28–0.35 for BR. Per burst:

0. **Channel filter:** a 25-tap zero-phase low-pass at the channel rate,
   −6 dB at ±0.6 MHz. Measured with the benchmark, this is the best
   bandwidth for both LE and BR: narrower than the LE signal's ~1.5 MHz
   occupied bandwidth, but it passes the GFSK main lobe and keeps noise out
   of the discriminator. Zero-phase, so burst edges and timing stay aligned.
1. **Interpolation:** 4× polyphase interpolation, 2 → 8 samples/symbol
   (49-tap Kaiser filter).
2. **FM discriminator:** `arg(z[n]·z*[n−1])`, scaled to Hz.
3. **Smoothing:** a one-symbol moving average, which approximates the
   matched filter for the frequency pulse.
4. **Carrier offset:** the mean frequency over the first 128 µs of the
   burst core (whitened data is balanced). The window covers the LE
   preamble/access address/header and the BR access code and header. These
   are GFSK even when an EDR (DPSK) payload follows, which would otherwise
   corrupt the CFO, timing and threshold estimates.
5. **Symbol timing:** the sampling phase (1 of 8) with the largest mean
   |f − CFO| over the core, i.e. the widest eye.
6. **Threshold refinement:** the midpoint between the mean of the positive
   and of the negative decisions; their half-difference is the deviation
   estimate.
7. **Slicing:** a positive deviation is a 1, as both specifications define.

The output carries:
- hard bits and soft values (the CFO-corrected frequency at each decision
  point, in Hz; its magnitude is the bit's reliability);
- the time of bit 0;
- the bit range of the energy core.

Clock drift (≤ 50 ppm) is below 0.15 symbol over the longest packet, so no
timing tracking is needed.

Many radios transmit tens of microseconds of unmodulated carrier before the
first preamble bit. Both decoders therefore search for packet starts from
just before the energy onset up to 80 bits after it.

## 5. Bluetooth LE (`ble/`)

**Codec (`ble/mod.rs`).**
- **Whitening:** the 7-bit LFSR x⁷+x⁴+1. Position 0 is set to 1 and
  positions 1–6 hold the channel index, MSB first.
- **CRC-24:** x²⁴+x¹⁰+x⁹+x⁶+x⁴+x³+x+1. The register runs in *air order*, so
  it equals the received CRC bits assembled LSB-first, with
  `init_air = bitreverse24(CRCInit)`.
- **CRC reversal:** the CRC register can be run backwards
  (`crc24_reverse`) to recover the initial value from a received packet.

**Remote ID (`ble/remoteid.rs`).** A Bluetooth 4 Remote ID module fills
the 31 bytes of advertising data with a single AD structure:
- service data (type 0x16) for UUID 0xFFFA;
- the application code 0x0D;
- a message counter;
- one 25-byte ASTM F3411 message.

The high nibble of the message's first byte is its type. Successive adverts
cycle through Basic ID, Location/Vector, Self-ID, System and Operator ID;
a message pack does not fit into a legacy advert. Fields follow the
standard's encodings:
- positions are int32 in units of 1e-7°;
- altitudes and heights are uint16 in half metres from −1000 m;
- speed is in 0.25 m/s steps, or 0.75 m/s above 63.75 m/s;
- the track is 0–179° plus an "east" flag that adds 180°.

The standard's "unknown" values (0,0 position, −1000 m, 255 with the 0.75
multiplier, 63 m/s vertical, track 361) are left out of the output.
`report.rs` keeps the latest message of each type per advertiser for the
summary.

**Decoder (`ble/decoder.rs`)**, for each burst on an LE channel:

1. **Known access addresses.** Correlate the 32-bit window at every
   candidate offset with the advertising AA `0x8E89BED6` and with the AAs of
   all tracked connections (≤ 2 bit errors). Each match is de-whitened with
   the channel's LFSR. The length is read from the header (data PDUs: plus
   one CTEInfo byte when CP = 1), and the CRC is checked with the
   advertising init `0x555555` or the connection's CRCInit. The first match
   that passes the CRC wins.
2. **CONNECT_IND.** A valid `CONNECT_IND` registers its connection: AA,
   CRCInit, interval and addresses, with an expiry of 4 × the supervision
   timeout, at least 30 s. `LL_TERMINATE_IND` shortens the expiry.
3. **Promiscuous discovery** runs on data channels for bursts no known AA
   matched:
   - require 8 alternating preamble bits, followed by an AA whose first bit
     continues the alternation;
   - the AA must satisfy the spec's validity rules (≤ 6 equal bits in a row,
     ≤ 24 transitions, ≥ 2 transitions in the top six bits, not the
     advertising AA or one bit away from it);
   - the de-whitened length must make the packet end within 5 µs of the end
     of the burst's energy;
   - run the CRC backwards to get the candidate CRCInit.

   A (AA, CRCInit) pair seen 3 times becomes a tracked connection. Its
   buffered packets are released with valid-CRC flags (they are consistent
   by construction).
4. **Soft-decision CRC repair.** If no candidate passes the CRC, the
   decoder takes the 12 bits of the packet with the smallest |soft| value
   (header to CRC). It flips them one at a time, then in pairs (78 trials),
   until the CRC passes. Header bits are included, so a corrupted length
   field can be repaired as well. With a 24-bit CRC the probability of
   accepting a wrong packet is about 78 × 2⁻²⁴ ≈ 5·10⁻⁶ per repaired packet.
   Repaired packets report the number of corrected bits.
5. **Direction.** Within a connection event the peripheral answers 150 µs
   (T_IFS) after the central's packet on the same channel. A packet starting
   150 ± 10 µs after the previous packet of the same connection on the same
   channel is attributed to the other side; anything else starts a new
   event and is attributed to the central.

## 6. Bluetooth BR (`bredr/`)

### Access code

The 64-bit sync word is a BCH(64,30) code word XOR-ed with a fixed PN
sequence (`0x83848D96BBCC54FC` in air order). It is built from 30
information bits: the 24-bit LAP followed by the 6-bit "barker" sequence
`001101` or `110010` (chosen by the LAP's MSB). The generator polynomial is
octal `260534236651`.

A candidate is accepted when:
- the syndrome `(sync ⊕ PN) mod g(D)` is zero, or matches one of the
  precomputed 1- and 2-bit error patterns (the code's minimum distance is
  14, so these are unique);
- and the barker bits agree with the LAP.

The LAP is then read from bits 34–57. Packets whose energy lasts less than
100 µs from the preamble are ID packets (access code only).

### Header

The 18-bit header (LT_ADDR, TYPE, FLOW, ARQN, SEQN, HEC) is sent with
rate-1/3 repetition coding. It is decoded by soft majority: each bit is the
sign of the sum of its three soft values. It is whitened with the
same 7-bit LFSR, initialised with the master's clock bits CLK6-1 (CLK1 in
position 0, position 6 = 1). The header error check is an 8-bit CRC
(polynomial 0xA7) initialised with the UAP. Bit orders match Wireshark's
`btbredr_rf` checks; the whitening offsets match libbtbb's tables. Both are
unit-tested.

### UAP and clock discovery (`bredr/decoder.rs`)

The UAP and CLK6-1 are not transmitted in clear, but they can be recovered:

- **One UAP per clock value.** For a de-whitened header the HEC relation is
  linear in the UAP. For each of the 64 clock values `c` there is exactly
  one UAP `u(c)` that makes the HEC valid (computed with a 256-entry inverse
  table).
- **The slot grid ties packets together.** Packets of one piconet start on
  its 625 µs slot grid, and CLK6-1 advances by one per slot. For packet *i*
  at slot index `s_i` (relative to the first packet), the hypothesis
  "clock offset `c₀`" implies `c = (c₀ + s_i) mod 64`.
- **Voting.** Every packet votes for the 64 pairs `(u(c), c − s_i)` in a
  256 × 64 table. Only types whose length agrees with the observed air time
  vote. The measured length runs from the preamble to the end of the energy,
  so it includes the transmitter's power ramp-down: live devices add up to
  ~20 µs. Accepted ranges:

  | Types | Range |
  |-------|-------|
  | NULL/POLL | 116–152 µs |
  | FHS | 352–395 µs |
  | DM1 | 165–400 µs |
  | other single-slot | 145–400 µs |
  | 3-slot | up to 1660 µs |
  | 5-slot | up to 2910 µs |

  The minima keep NULL/POLL apart from DM1/FHS, which the clock aliases
  below would otherwise turn them into.
  A pair whose payload decodes with a valid CRC-16 gets weight 5 instead
  of 1.
- **Protocol rule.** POLL is only sent by the master, and the master only
  transmits in slots with CLK1 = 0. Hypotheses that would put a POLL into a
  slave slot do not vote.
- **Partial aliases.** Whitening is linear: for clock pairs `c`, `c + δ` the
  mask difference is constant for a subset of `c` values. A few wrong
  (UAP, clock) pairs therefore collect votes from a fixed fraction of the
  packets, up to ~40% in tests.
- **Locking.** The true pair gains one vote from nearly every packet. The
  piconet locks when the best cell either has ≥ 6 votes and 3× the
  runner-up, or has ≥ 8 votes, at least 60% of the packets that voted, and
  1.5× the runner-up. Typically this takes one CRC packet plus one more, or
  about six to eight header-only packets.
- **Restart.** Votes are reset after 8 s without a lock, because the slot
  grid drifts against the receiver clock.

Packets seen before the lock are buffered (up to 4 s) and decoded
retroactively.

Once locked, each new packet is de-whitened with the predicted clock
`c_ref + round(Δt / 625 µs)`, searching outward:
- the search widens with the time since the last good packet;
- a CRC-confirmed clock is preferred;
- a header-only clock is accepted within ±1 slot.

The reference is re-anchored on every decoded packet, which absorbs clock
drift. If nothing fits (for example an FHS sent during page response with
its special whitening, or after a long gap), all 64 clocks are tried and
only a CRC-confirmed result is accepted.

**Complementary clock.** The whitening sequences of `c` and `63 − c` differ
by a valid header code word, so every header also passes the HEC under the
complementary clock. It then reads with TYPE bit 0 flipped (NULL↔POLL,
FHS↔DM1), LT_ADDR XORed with 6 and ARQN/SEQN flipped. No other pair of
clocks aliases. This only matters when the prediction is already wrong,
and it has two consequences:
- An FHS is accepted only with a valid CRC. FHS packets are never
  encrypted, and an encrypted 17-byte DM1 has the same length, so an
  unconfirmed "FHS" is a DM1 read with the wrong clock.
- Live, the aliases showed up as POLL/NULL/FHS packets with LT_ADDR 7 on a
  piconet that only used LT_ADDR 1.

**Losing the lock.** A HackRF can lose samples without telling the host,
for example a whole USB transfer of 131072 samples (6.55 ms at 20 Msps,
about 10.5 slots). All later timestamps are then early by that amount; live,
jumps of 6.55 ms and 2.4 ms were seen. On a link that carries only POLL/NULL
and encrypted payloads, nothing can be confirmed by CRC, and the old clock
never fits again. Live, such a piconet stayed undecodable for 2–12 minutes,
apart from occasional complementary-clock aliases.

The decoder therefore watches the last 8 packets of a locked piconet. When
5 of them fail to decode, it drops the clock, keeps the UAP and starts
discovery again, with the votes restricted to the known UAP. The packets of
the new search are buffered and decoded once the new lock is found. The
event is logged as `clock lost (…), searching again`. In a 15-minute live
sweep a piconet lost its clock twice and was locked again after 0.5 s and
10 s; 96% of its headers were de-whitened, against 81–94% before.

**Inquiry.** Packets with an inquiry access code (LAP `0x9E8B00`–`0x9E8B3F`)
use the DCI `0x00` as UAP. Their whitening clock is brute-forced per packet,
with HEC plus CRC confirmation. This decodes FHS inquiry responses, which
carry the responding device's full BD_ADDR, class of device and clock.

### Payload

Supported types are FHS, DM1, DH1, DM3, DH3, DM5 and DH5:

1. **FEC.** Decode the rate-2/3 FEC ((15,10) shortened Hamming code,
   g(D) = D⁵+D⁴+D²+1) where the type uses it. Single errors are corrected
   algebraically. Blocks with an uncorrectable syndrome are Chase-decoded:
   all 7 combinations of their three least reliable bits are flipped, and
   the candidate code word nearest to the soft values wins. Only as many
   blocks as the payload header announces are decoded.
2. **De-whitening.** Continue the header's whitening sequence (offset 18).
3. **Payload header.** Parse it: 1 byte with a 5-bit length for single-slot
   packets, 2 bytes with a 10-bit length for multi-slot packets.
4. **CRC.** Check the CRC-16 (CCITT, reflected, initialised with the
   reversed UAP).

Types without a CRC (HV, AUX1, DV), eSCO packets, and EDR packets (whose
payload is DPSK) are reported with their header only.

### EDR payloads (`dsp/edr.rs`)

An EDR packet sends its access code and header in GFSK. Then follow:
- a guard time of 4.75–5.25 µs;
- a DPSK synchronisation sequence: a reference symbol and ten symbols with
  phase changes 3π/4, −3π/4, 3π/4, −3π/4, 3π/4, −3π/4, −3π/4, 3π/4, 3π/4,
  3π/4;
- the payload at 1 Msym/s, root-raised-cosine pulses with roll-off 0.4:
  π/4-DQPSK (2 bits/symbol, Gray: 00→π/4, 01→3π/4, 11→−3π/4, 10→−π/4) or
  8DPSK (3 bits/symbol, Gray: 000→0, 001→π/4, 011→π/2, 010→3π/4, 110→π,
  111→−3π/4, 101→−π/2, 100→−π/4);
- a two-symbol trailer.

The demodulator works on the burst's channel samples:
1. **Interpolation and offset removal:** interpolate ×4 and remove the
   carrier offset measured on the GFSK part of the same packet.
2. **Matched filter:** apply the RRC matched filter.
3. **Timing:** try reference-symbol positions for guard times of 4.5–5.5 µs
   in 1/8-symbol steps. Keep the one whose ten differential phases best
   match the sync sequence (mean cos of the error; at least 0.7 is
   required). Real EDR packets score 0.99–1.0, and about 0.9 at the
   decoding threshold. Signals that are not standard EDR score around 0.5.
4. **Residual frequency error:** estimate it from the sync sequence.
5. **Detection:** detect each payload symbol differentially (nearest
   constellation phase change) and track the residual error decision-
   directed.

The BR decoder tries this for packets whose basic-rate decoding fails the
CRC and whose TYPE has an EDR ACL meaning: 4 = 2-DH1, 8 = 3-DH1, 10 = 2-DH3,
11 = 3-DH3, 14 = 2-DH5, 15 = 3-DH5. The DPSK bits are computed once per
packet and rate, then de-whitened, continuing the header's whitening
sequence, for every (UAP, clock) hypothesis. Their 2-byte payload header and
CRC are checked exactly as for basic rate. A valid CRC also feeds the
piconet discovery votes.

## 7. Output formats (`pcap.rs`)

PCAPNG files contain two interfaces, both with nanosecond timestamps
(`if_tsresol = 9`):
- 0 = `LINKTYPE_BLUETOOTH_LE_LL_WITH_PHDR` (256);
- 1 = `LINKTYPE_BLUETOOTH_BREDR_BB` (255).

`--ble-pcap` and `--bredr-pcap` write classic nanosecond PCAP files (magic
`0xA1B23C4D`) with one link type each. All multi-byte fields are
little-endian.

**LE record** (10-byte header + packet):

| Field | Value |
|-------|-------|
| rf_channel | RF channel 0–39 ((f − 2402)/2) |
| signal_power, noise_power | dBFS (+ `--rssi-offset`) |
| access_address_offenses | AA bit errors |
| reference_access_address | the AA |
| flags | de-whitened, signal and noise valid, reference AA valid, AA offenses valid, CRC checked, CRC valid; PDU type: 0 for advertising, 2/3 for central→peripheral / peripheral→central data; PHY = LE 1M |
| packet | AA (4) + PDU + CRC (3, air bit order) |

**BR/EDR record** (22-byte header + payload):

| Field | Value |
|-------|-------|
| rf_channel | 0–78 |
| signal_power, noise_power | dBFS |
| access_code_offenses | corrected sync word bits |
| payload_transport_rate | `0xFF` for ID packets, `0x30` / `0x31` / `0x32` (ACL, BR / EDR 2M / EDR 3M) for CRC-verified payloads, `0x01` / `0x02` for EDR detected without a valid CRC, `0x00` otherwise |
| corrected_header_bits, corrected_payload_bits | FEC corrections |
| lap | LAP |
| ref_lap_uap | LAP \| UAP << 24 |
| bt_header | the 18-bit header, de-whitened once the clock is known |
| flags | de-whitened, signal and noise valid, reference LAP valid, reference UAP valid, HEC checked and valid, CRC checked and valid, payload present, and decrypted (set for CRC-valid payloads so Wireshark dissects them) |
| payload | payload header + body + CRC, air bit order |

### Wireshark extcap (`extcap.rs`)

When started with any `--extcap-*` option, btsdr implements Wireshark's
extcap protocol instead of its normal command line:

| Call | Answer |
|------|--------|
| `--extcap-interfaces` | three interfaces: `btsdr` (LE + BR/EDR), `btsdr-le`, `btsdr-bredr` |
| `--extcap-interface X --extcap-dlts` | the interface's (first) link type |
| `--extcap-interface X --extcap-config` | 13 options grouped in the "Radio", "Gain", "Decoder" and "Device" tabs |
| `--extcap-interface X --capture --fifo P [options]` | a PCAPNG stream written into the FIFO / named pipe `P` |

How the capture behaves:
- **Format:** the stream is PCAPNG. The combined interface carries two
  interface description blocks; dumpcap (Wireshark 4.x) keeps them, so LE
  and BR/EDR packets keep their own link types.
- **Silence:** nothing is printed on stderr during a normal capture.
  Wireshark shows stderr output as an error, so only real problems (for
  example, no HackRF) reach the user.
- **Unknown options are ignored,** so newer Wireshark versions can add their
  own.

Wireshark stops an extcap tool by sending SIGTERM (Unix) or by closing the
pipe. The `termination` feature of `ctrlc` turns SIGTERM into the normal
stop path. A closed pipe surfaces as a write error. Even when no packets
are captured, an empty Interface Statistics Block is written every second
as a keep-alive, so a closed pipe is noticed within a second. Either way
the pipeline shuts down and the HackRF is closed properly.

`btsdr extcap install` asks `tshark -G folders` for the personal extcap
folder, falling back to the platform default. It copies the running
executable there and checks that the copy answers `--extcap-interfaces`.

## 8. Validation

- **Codec cross-checks** (`cargo test`):
  - sync words against libbtbb's generator matrix;
  - BR whitening against libbtbb's m-sequence table, for all 64 clocks;
  - FEC 2/3 against libbtbb's generator matrix;
  - HEC against Wireshark's check;
  - CRC-16 residue;
  - LE whitening and CRC against an independent byte-oriented
    implementation.
- **DSP tests:** a tone lands in the right channel at the right frequency
  with the expected leakage, and burst edges are found accurately.
- **End-to-end test** (`tests/synthetic.rs`): the scenario generator in
  `synth.rs` produces LE advertising, an announced and a hidden connection,
  a BR piconet and inquiry traffic. The whole pipeline must decode it at
  25 dB SNR and degrade gracefully at 14 dB. A second test inserts a retune
  marker: packets before it must keep the old channels, packets after it
  must get the new ones with continuous timestamps, and a packet inside the
  settling time must be dropped. A third test removes one USB transfer
  (131072 samples) from a piconet that sends only POLL, NULL and encrypted
  DM1 packets. At least 80% of the packets after the cut must decode again,
  all with the true LT_ADDR and packet types. A fourth test sends Remote ID
  adverts (Basic ID and Location) and checks the decoded fields.
- **Remote ID unit tests:** messages built byte by byte from the standard's
  layout, including the "unknown" values and the 0.75 m/s speed multiplier.
  They have not been checked against a real drone.
- **Sensitivity benchmark** (`examples/benchmark.rs`): packet success rate
  versus SNR on synthetic captures with impairments within the
  specification limits (carrier offset and drift, modulation index spread,
  phase noise). `--edr` measures each EDR packet type separately. The
  results are in the README.
- **Live tests with a HackRF One:**
  - LE advertising (CRC-valid packets from phones, printers and other nearby
    devices);
  - Classic inquiry trains from a PC (GIAC ID packets);
  - LE connections followed from `CONNECT_IND` and discovered
    promiscuously, with CRC-valid data packets;
  - Classic piconets: UAP/clock discovery, header de-whitening (95%+ of the
    packets of locked piconets), BR payloads with valid CRC;
  - EDR 2-DH1 and 3-DH1 packets from a phone with valid CRC and consistent
    L2CAP lengths;
  - live streaming into Wireshark;
  - the extcap interfaces through tshark: listing, option passing, LE and
    BR/EDR link types in one stream, clean stop and error reporting;
  - the band sweep: in 40 s, 1063 packets, each on a channel of the step it
    was received in, and LE advertising decoded on all three advertising
    channels (their whitening depends on the channel, so a wrong channel map
    would fail the CRC).

## 9. Ideas for future work

- LE 2M PHY (a second filter bank with 2 MHz channels) and LE Coded.
- EDR symbol timing tracking for long packets, eSCO EDR payloads.
- Following LE connections across the whole band with channel-selection
  algorithms #1/#2 and several receivers.
- Better timing recovery and soft-decision CRC repair for EDR payloads.
- Multi-threaded decoding for very busy environments.

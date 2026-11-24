# Changelog

## 0.1.0 (unreleased)

First release.

- HackRF One live capture (libhackrf loaded at run time) and IQ file input
  (`ci8`, `cu8`, `ci16`, `cf32`).
- Polyphase filter-bank channelizer for up to 19 simultaneous 1 MHz channels,
  with wideband (Wi-Fi) rejection.
- Bluetooth LE 1M:
  - advertising decoding;
  - following connections announced by `CONNECT_IND`;
  - promiscuous discovery of existing connections (CRCInit recovery);
  - direction inference.
- Bluetooth BR:
  - LAP extraction with sync word error correction;
  - inquiry and page ID packets;
  - per-piconet UAP and clock discovery;
  - header de-whitening and HEC check;
  - FHS and DM/DH payload decoding with CRC check.
- PCAPNG (LE + BR/EDR) and PCAP output for Wireshark, including live
  streaming to stdout.
- `synth` command and an end-to-end test suite built on synthetic signals.
- Wireshark extcap integration: `btsdr extcap install` registers three
  capture interfaces (LE + BR/EDR, LE only, BR only) with a settings dialog.
- Sensitivity improvements, measured with the new `examples/benchmark.rs`.
  At 50% decoding they gain about 2.5 dB for LE advertising and about 3 dB
  for BR headers:
  - narrower channel filter (±0.6 MHz);
  - soft-decision CRC repair for LE (up to two bit errors);
  - soft majority header decoding and Chase decoding of the rate-2/3 FEC
    for BR;
  - more robust piconet UAP/clock lock (partial-alias aware, POLL slot
    rule).

  `--hard-decisions` restores the previous behaviour for comparison.
- `btsdr synth --impaired` adds realistic radio impairments.
- EDR payload demodulation (π/4-DQPSK and 8DPSK) for 2-DH1 … 3-DH5, with
  CRC check and Wireshark transport/rate flags.
  - The channelizer is widened to ±0.7 MHz for EDR; GFSK keeps a ±0.6 MHz
    filter in its demodulator.
  - GFSK CFO/timing estimation is limited to the start of the burst.
  - `synth` generates EDR packets; `examples/benchmark.rs --edr` measures
    them.
- Live validation with a phone and earbuds:
  - EDR 2-DH1/3-DH1 decoded with valid CRC;
  - LE connections followed and discovered;
  - piconets locked.
- ADC clipping detection: the status line and summary warn and suggest
  lowering the gain.
- BR length plausibility accounts for the transmitter's power ramp-down.
  Idle piconets (only POLL/NULL) of some devices were rejected before;
  locked piconets now de-whiten 95%+ of their headers live.
- The EDR sync threshold was raised to 0.7, so non-standard DPSK packets are
  no longer reported as EDR.
- Band sweep: `btsdr capture --sweep` retunes the HackRF through the whole
  2.4 GHz band, every `--dwell` seconds (default 2).
  - At 20 Msps there are 5 steps of 17 channels.
  - The decoder state and the time base are kept across steps.
  - `--sweep 2410,2427,2472` visits only the given centre frequencies.
  - The extcap settings have a matching option.
  - The status line shows the current centre frequency.
- BR piconet clock recovery: a locked piconet whose clock fails on 5 of its
  last 8 packets is searched for again, with its UAP kept.
  - This happens when a HackRF silently loses samples, e.g. a USB transfer
    (about 10.5 slots). Links that carry only POLL/NULL and encrypted
    payloads stayed undecodable for 2–12 minutes before; live they now
    recover in 0.5–10 s.
  - An FHS is only reported with a valid CRC. Without one it is an
    encrypted DM1 read with the complementary clock (63 − CLK6-1), which
    turns DM1 into FHS and LT_ADDR 1 into 7.
- Drone Remote ID (ASTM F3411-22a, EN 4709-002) in Bluetooth 4 advertising
  (service data UUID 0xFFFA) is decoded without new dependencies.
  - The advertising line shows the Basic ID, Location/Vector, Self-ID,
    System and Operator ID fields, e.g. `rid_id=…`, `rid_pos=…`, `rid_alt=…`,
    `rid_operator_pos=…`; values the drone marks as unknown are left out.
  - The summary names drones by their UAS ID and lists each with its latest
    messages.
  - `synth` has encoders for Remote ID adverts, and an end-to-end test
    decodes them from IQ.

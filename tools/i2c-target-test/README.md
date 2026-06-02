# i2c-target-test

Host-side I2C controller test app that exercises the
`embedded_mcu_hal::i2c::target` trait implementation on an RT685 board.

It connects to a [Pico de Gallo](https://github.com/OpenDevicePartnership/pico-de-gallo)
USB bridge, configures it as an I2C controller, and drives a sequence of
transactions designed to hit every `Request` variant (`Write`, `Read`,
`RepeatedStart`, `Stop`/probe) and every `WriteStatus`/`ReadStatus` variant
(`Stopped`/`Restarted`/`BufferFull`, `Complete`/`NeedMore`/`EarlyStop`) that
the trait impl in `embassy-imxrt` is supposed to surface.

## Hardware setup

1. Flash one of the target-trait example binaries to your RT685S-EVK:
   - `cd ../../examples/rt685s-evk`
   - `cargo run --release --bin i2c-slave-async-target-trait` (async)
   - or `cargo run --release --bin i2c-slave-target-trait` (blocking)
2. Connect the Pico de Gallo board to your host PC via USB.
3. Wire:
   - Pico de Gallo I2C SCL -> RT685S-EVK pin `P0_18` (FC2 SCL)
   - Pico de Gallo I2C SDA -> RT685S-EVK pin `P0_17` (FC2 SDA)
   - common GND between the two boards
4. The Pico de Gallo has built-in pull-ups; the embassy-imxrt slave
   configures its SCL/SDA pads as open-drain with no internal pull, so no
   extra resistors are required.

The on-device example listens at 7-bit address `0x20` by default — change
`--address` if you patched `SLAVE_ADDR` in the example.

## Usage

```bash
cargo run -- --help            # show all subcommands and flags
cargo run -- list              # enumerate connected Pico de Gallo boards
cargo run -- scan              # bus scan; must see 0x20 ACK
cargo run -- all               # run every functional test + a short soak

# Functional scenarios:
cargo run -- write             # 8-byte write -> WriteStatus::Stopped(8)
cargo run -- read              # 8-byte read  -> ReadStatus::Complete(8)
cargo run -- write-read        # write+restart+read -> RepeatedStart event
cargo run -- buffer-full       # 24-byte write -> 2 BufferFull cycles + Stop
cargo run -- early-stop        # 4-byte read of 8-byte source -> EarlyStop
cargo run -- need-more         # 16-byte read -> NeedMore + Complete

# Stress / regression hunting:
cargo run -- soak --iterations 5000
cargo run -- soak --iterations 100000 --fail-fast
cargo run -- write-read-soak --iterations 100000
cargo run -- write-read-soak --iterations 1000000 --timeout-ms 200 --keep-going
```

If you have multiple Pico de Gallo boards plugged in, target a specific one
with `--serial-number ABCD1234` (use `cargo run -- list` to find serial
numbers).

## Per-call timeout

Every host-side I2C transaction is wrapped in `--per-call-timeout-ms`
(default 1000 ms). A wedged slave that stops clocking SCL would otherwise
block the harness indefinitely; the timeout converts that into a clean
classified failure that prints the iteration number and the wall-clock
budget that was exceeded.

A successful 100 kHz I2C transaction of < 32 bytes finishes in < 10 ms on
the wire, so 1 s is comfortably generous for everyday runs. The
`write-read-soak` subcommand defaults to a tighter 200 ms because the
race-hunt scenario benefits from failing fast.

## Stress: `soak` vs `write-read-soak`

Two soak modes target different failure classes.

### `soak`

Cycles through four transaction shapes (1-byte write, 4-byte read, 2-byte
write + 4-byte read combined-format, 2-byte write + 2-byte read batched).
Records a histogram of outcomes — ok / timeout / payload-mismatch / usb-error
— and prints it at the end. Use this as a broad regression sweep.

By default it runs to completion and reports a failure if any class is
non-zero. Pass `--fail-fast` to abort on the first timeout (useful when you
want a stack trace correlated with a defmt-print RTT capture; a wedged
slave can't recover on its own, so continuing produces no new information).

### `write-read-soak`

Hammers only `i2c_write_read` (controller writes N bytes, repeated START,
reads M bytes). This is the transaction shape that exposes the
`slv_state -> addressed` mid-DMA HW race described in
[PR #565 discussion](https://github.com/OpenDevicePartnership/embassy-imxrt/pull/565#discussion_r3337586759):
under back-to-back combined-format transactions, the FC peripheral
occasionally reports `SlaveAddress` instead of the expected `SlaveReceive`
while a DMA receive is still in flight, which the driver currently maps to
`WriteStatus::Restarted(_)` + a queued `RepeatedStart` edge. When this
fires, the slave's `respond_to_*` future for the subsequent leg blocks and
the host-side `i2c_write_read` times out.

This mode fails fast by default and prints:

- The iteration number that wedged.
- The wall-clock elapsed time from the start of the soak run.
- The write payload that was on the wire.
- A reproducer command line.

Pass `--keep-going` to measure how often the race fires across a long run
without stopping. Pass `--write-len` / `--read-len` to vary the transaction
shape (both default to 4 bytes).

## What to watch on the slave side

Run `probe-rs attach` or `defmt-print` against the RT685 RTT channel while
the host app runs. The slave should log one event per transaction, e.g.:

```text
Write @ 0x20
Write stopped after 8 bytes
Read @ 0x20
Read complete with 8 bytes
Write @ 0x20
Write restarted after 4 bytes — next listen will surface RepeatedStart
RepeatedStart from prev @ 0x20
(consumed expected RepeatedStart edge before Read)
Read @ 0x20
Read complete with 8 bytes
```

The target-trait examples also emit `WARN` lines for two suspicious shapes
associated with the HW race:

1. **`WriteStatus::Restarted(0)`** — a zero-byte restart should not occur
   on a healthy bus; a real repeated START is preceded by at least one
   ACKed payload byte. Seeing this strongly suggests the slave
   mis-classified an in-flight receive as a restart.
2. **`Request::RepeatedStart(_)` without a prior `Restarted(_)`** — the
   queued edge that surfaces as `RepeatedStart` should always have a
   matching upstream `Restarted` event.

When `write-read-soak --fail-fast` fires, capture the surrounding 50 ms of
RTT log: the on-target `WARN` line tells you which mis-classification the
slave made; the host-side timeout tells you which leg of the transaction
the controller was on when the slave stopped clocking.

If the slave logs an unexpected error or a wildly different status than
what the test case expects, you've found a regression in the trait impl.

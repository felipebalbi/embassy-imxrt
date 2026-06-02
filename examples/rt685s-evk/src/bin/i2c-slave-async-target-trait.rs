//! I2C slave async example using the `embedded_mcu_hal::i2c::target` trait.
//!
//! This is the trait-based counterpart to `i2c-slave-async.rs`. It drives
//! the same FC2 hardware in the same listen → respond → re-listen loop, but
//! it goes through the `embedded_mcu_hal::i2c::target::asynch::I2c` trait
//! instead of the inherent `I2cSlave` methods. Differences from the
//! inherent example:
//!
//! * `listen()` returns a `Request<SevenBitAddress>` carrying the matched
//!   address; the example logs it at `debug!` (silent at `DEFMT_LOG=info`).
//! * `respond_to_write()` / `respond_to_read()` return `WriteStatus` /
//!   `ReadStatus` enums that distinguish `Stopped` / `Restarted` /
//!   `BufferFull` (for writes) and `Complete` / `NeedMore` / `EarlyStop`
//!   (for reads).
//! * `RepeatedStart(prev_addr)` is surfaced as a separate `listen()` event
//!   between a write and a read on the same controller transaction.
//! * `recover()` is **not** called on the happy path. The `Restarted`
//!   branch deliberately leaves the in-flight transaction alone so the
//!   queued `RepeatedStart` edge surfaces on the next `listen()`. Reserve
//!   `recover()` for wedged / cancelled transfers — e.g. after dropping a
//!   `respond_to_*` future mid-transaction.
//!
//! ## Race-watching telemetry
//!
//! Two `warn!` emissions in this example flag known-suspicious shapes
//! associated with the `slv_state -> addressed` mid-DMA HW race tracked
//! on PR #565:
//!
//! 1. `WriteStatus::Restarted(0)` — a zero-byte restart should not occur
//!    on a healthy bus: a real repeated START is preceded by at least one
//!    ACKed payload byte. Seeing this strongly suggests the slave
//!    mis-classified an in-progress receive as a restart.
//! 2. `Request::RepeatedStart(_)` arriving when the prior `respond_to_*`
//!    did **not** report `Restarted(_)`. The queued edge that produces
//!    `RepeatedStart` should always have a matching upstream `Restarted`.
//!
//! ## Soak workflow (Mole rig)
//!
//! Pair this binary with the `i2c-soak.moleasm` program in the sibling
//! `mole` repository — a Mole bit-cycle-engine controller-role soak that
//! drives back-to-back combined-format write(32) / Sr / read(32)
//! transactions at 400 kHz with the tightest legal tBUF, looping until
//! the slave NACKs (which is how a wedged slave manifests to the
//! controller). The Mole HALT status + MARK count identify the wedge
//! iteration; the slave-side RACE WATCH `warn!` lines (if any) identify
//! the mis-classification that fired.
//!
//! Build this example with `DEFMT_LOG=info` (the default for `cargo run`
//! via `examples/rt685s-evk/.cargo/config.toml` is `trace`, which keeps
//! per-transaction `debug!` chatter on the wire — that adds ~5 RTT lines
//! per Mole transaction and at ~1300 tx/sec risks blocking the slave
//! between STOP and the next address phase inside Mole's 1.875 µs tBUF
//! window). At `info` level: only the 255-transaction heartbeat and the
//! two RACE WATCH `warn!` lines are emitted; the rest is compile-time
//! no-ops.
//!
//! ```sh
//! # Override the trace default for the soak build:
//! set DEFMT_LOG=info
//! cargo run --release --bin i2c-slave-async-target-trait
//! ```
//!
//! The companion `tools/i2c-target-test write-read-soak` host harness
//! (USB-bound, ~1 tx/ms ceiling) remains useful for the broader regression
//! sweep where per-transaction logging IS desired. Use Mole for the tight
//! reproducer; use the host harness for the variety pass.
//!
//! Tested against the same Raspberry Pi 5 master rig as the existing
//! `i2c-slave-async.rs` example
//! (https://github.com/jerrysxie/pi5-i2c-test).

#![no_std]
#![no_main]

use defmt::{debug, info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_imxrt::i2c::slave::{Address, I2cSlave};
use embassy_imxrt::i2c::{self, Async};
use embassy_imxrt::{bind_interrupts, peripherals};
use embassy_imxrt_examples as _;
// Bring the target trait methods into scope so we go through the trait
// instead of the inherent API.
use embedded_mcu_hal::i2c::SevenBitAddress;
use embedded_mcu_hal::i2c::target::Request;
use embedded_mcu_hal::i2c::target::asynch::I2c as TargetI2c;
use panic_probe as _;

const SLAVE_ADDR: Option<Address> = Address::new(0x20);
const BUFLEN: usize = 32;

/// Emit a heartbeat `info!` every `HEARTBEAT_EVERY` completed transactions.
/// Mirrors the Mole-side `MARK label=0xAA` cadence (also 255) so the two
/// streams correlate 1:1 in the captured logs.
const HEARTBEAT_EVERY: u32 = 255;

bind_interrupts!(struct Irqs {
    FLEXCOMM2 => i2c::InterruptHandler<peripherals::FLEXCOMM2>;
});

#[embassy_executor::task]
async fn slave_service(mut i2c: I2cSlave<'static, Async>) {
    // Tracks whether the most recent respond_to_* terminator was
    // `Restarted(_)`. The very next `listen()` is expected to return
    // `Request::RepeatedStart(_)`; any other shape indicates an event
    // mismatch worth investigating. See the module-level "Race-watching
    // telemetry" docs.
    let mut expect_repeated_start = false;

    // Counter for completed transactions (write + Sr + read on the
    // combined-format path; or just a single Write / Read / Stop on the
    // simple paths). Drives the heartbeat `info!` used to correlate
    // with the Mole `MARK label=0xAA` heartbeat in the soak workflow.
    // u32 covers ~46 days of continuous 1000 tx/sec soak before wrap;
    // for the v1 PR this is plenty.
    let mut tx_count: u32 = 0;

    info!(
        "i2cs target-trait soak listening @ 0x20; heartbeat every {} transactions",
        HEARTBEAT_EVERY
    );

    loop {
        let mut buf: [u8; BUFLEN] = [0u8; BUFLEN];

        for (i, e) in buf.iter_mut().enumerate() {
            *e = i as u8;
        }

        // Go through the target trait — note `<_ as TargetI2c<SevenBitAddress>>`
        // disambiguates between the 7-bit and 10-bit trait impls. The
        // address mode is checked at runtime against the address the slave
        // was constructed with; a mismatch returns `ErrorKind::Other`.
        let req: Request<SevenBitAddress> = match TargetI2c::<SevenBitAddress>::listen(&mut i2c).await {
            Ok(r) => r,
            Err(e) => {
                warn!("listen error: {:?}", defmt::Debug2Format(&e));
                expect_repeated_start = false;
                continue;
            }
        };

        let was_expecting_restart = expect_repeated_start;
        expect_repeated_start = false;

        match req {
            Request::Stop(addr) => {
                // A probe (address-only transaction terminated by STOP)
                // surfaces here. The inherent API reports the same event
                // as `Command::Probe { addr }`.
                debug!("Stop @ 0x{:02X} (probe)", addr);
                if was_expecting_restart {
                    warn!(
                        "RACE WATCH: prior respond_to_* reported Restarted but listen() \
                         returned Stop(0x{:02X}); expected RepeatedStart",
                        addr
                    );
                }
            }
            Request::RepeatedStart(prev_addr) => {
                // Surfaced when a previous respond_to_* observed a Sr.
                debug!("RepeatedStart from prev @ 0x{:02X}", prev_addr);
                if !was_expecting_restart {
                    warn!(
                        "RACE WATCH: RepeatedStart(0x{:02X}) surfaced without a prior \
                         Restarted(_) — likely a spurious edge synthesised from a \
                         mid-DMA SlaveAddress mis-classification",
                        prev_addr
                    );
                }
            }
            Request::Read(addr) => {
                debug!("Read @ 0x{:02X}", addr);
                if was_expecting_restart {
                    // A Read after a Restarted is a normal combined-format
                    // transaction; the RepeatedStart event was consumed
                    // implicitly by the trait impl.
                    debug!("(consumed expected RepeatedStart edge before Read)");
                }
                loop {
                    use embedded_mcu_hal::i2c::target::ReadStatus;
                    match TargetI2c::<SevenBitAddress>::respond_to_read(&mut i2c, &buf).await {
                        Ok(ReadStatus::Complete(n)) => {
                            debug!("Read complete with {} bytes", n);
                            tx_count = tx_count.wrapping_add(1);
                            break;
                        }
                        Ok(ReadStatus::EarlyStop(n)) => {
                            debug!("Read terminated by controller after {} bytes", n);
                            tx_count = tx_count.wrapping_add(1);
                            break;
                        }
                        Ok(ReadStatus::NeedMore(n)) => {
                            debug!("Read NeedMore: sent {} bytes so far, more requested", n);
                            // Loop and supply more bytes. In a real
                            // application you would prepare the next chunk
                            // here; for the demo we just resend `buf`.
                        }
                        Ok(_) => {
                            // ReadStatus is `#[non_exhaustive]`; future
                            // variants are gracefully ignored.
                            warn!("Read: unknown status variant");
                            break;
                        }
                        Err(e) => {
                            warn!("respond_to_read error: {:?}", defmt::Debug2Format(&e));
                            break;
                        }
                    }
                }
            }
            Request::Write(addr) => {
                debug!("Write @ 0x{:02X}", addr);
                if was_expecting_restart {
                    debug!("(consumed expected RepeatedStart edge before Write)");
                }
                loop {
                    use embedded_mcu_hal::i2c::target::WriteStatus;
                    match TargetI2c::<SevenBitAddress>::respond_to_write(&mut i2c, &mut buf).await {
                        Ok(WriteStatus::Stopped(n)) => {
                            debug!("Write stopped after {} bytes", n);
                            tx_count = tx_count.wrapping_add(1);
                            break;
                        }
                        Ok(WriteStatus::Restarted(n)) => {
                            debug!(
                                "Write restarted after {} bytes — next listen will surface RepeatedStart",
                                n
                            );
                            if n == 0 {
                                warn!(
                                    "RACE WATCH: WriteStatus::Restarted(0) — zero-byte restart \
                                     should not occur on a healthy bus (Sr is preceded by at \
                                     least one ACKed payload byte). Likely the slv_state -> \
                                     addressed mid-DMA HW race noted in PR #565."
                                );
                            }
                            // Do NOT call recover() here: a Restarted is a
                            // healthy continuation of an in-flight master
                            // transaction (Sr + ADDR+R/W is queued on the
                            // wire). recover() would NAK the new address
                            // byte and drop the queued RepeatedStart edge,
                            // causing the master to see a spurious NACK.
                            // Reserve recover() for wedged/cancelled
                            // transfers — e.g. after dropping a
                            // respond_to_* future mid-transaction.
                            expect_repeated_start = true;
                            // Don't count: the read leg of the combined
                            // transaction will increment tx_count when it
                            // completes. Counting here would double-count
                            // every write+read pair.
                            break;
                        }
                        Ok(WriteStatus::BufferFull(n)) => {
                            debug!("Write BufferFull after {} bytes — supplying more buffer space", n);
                            // Loop and continue draining.
                        }
                        Ok(_) => {
                            // WriteStatus is `#[non_exhaustive]`; future
                            // variants are gracefully ignored.
                            warn!("Write: unknown status variant");
                            break;
                        }
                        Err(e) => {
                            warn!("respond_to_write error: {:?}", defmt::Debug2Format(&e));
                            break;
                        }
                    }
                }
            }
            // GeneralCall / SmbusAlert are not produced by this peripheral
            // in v1; the catch-all covers any future variants.
            _ => {
                warn!("unhandled request variant");
            }
        }

        // Heartbeat: one info! per HEARTBEAT_EVERY completed transactions.
        // Aligns 1:1 with Mole's `MARK label=0xAA` cadence so log streams
        // correlate. Uses `is_multiple_of` for readability; the modulo
        // path is identical machine code.
        if tx_count > 0 && tx_count.is_multiple_of(HEARTBEAT_EVERY) {
            info!("soak heartbeat: {} transactions completed", tx_count);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_imxrt::init(Default::default());

    info!("i2cs target-trait example - I2c::new");
    let i2c = I2cSlave::new_async(p.FLEXCOMM2, p.PIO0_18, p.PIO0_17, Irqs, SLAVE_ADDR.unwrap(), p.DMA0_CH4).unwrap();

    spawner.spawn(slave_service(i2c).unwrap());
}

//! Host-side I2C controller test app for the embassy-imxrt I2C target trait
//! examples.
//!
//! Drives the `i2c-slave-async-target-trait` and `i2c-slave-target-trait`
//! examples (flashed onto an RT685 board) by acting as the I2C controller
//! through a Pico de Gallo USB bridge. Each test case exercises a specific
//! [`embedded_mcu_hal::i2c::target::Request`] / `WriteStatus` / `ReadStatus`
//! variant that the on-device trait impl should surface.
//!
//! # Wiring
//!
//! - Pico de Gallo I2C SCL -> RT685S-EVK pin P0_18 (FC2 SCL)
//! - Pico de Gallo I2C SDA -> RT685S-EVK pin P0_17 (FC2 SDA)
//! - Common GND between the two boards
//! - Pull-ups: rely on the Pico de Gallo's built-in pull-ups (the
//!   embassy-imxrt slave configures its SCL/SDA pads as open-drain with no
//!   internal pull).
//!
//! # Slave address
//!
//! The target-trait examples both listen on the 7-bit address `0x20`. Change
//! `--address` if you rebuild them with a different `SLAVE_ADDR` constant.
//!
//! # Usage
//!
//! ```text
//! cargo run -- --help
//! cargo run -- all
//! cargo run -- scan
//! cargo run -- write
//! cargo run -- read
//! cargo run -- write-read
//! cargo run -- buffer-full
//! cargo run -- early-stop
//! cargo run -- need-more
//! cargo run -- soak --iterations 5000
//! cargo run -- write-read-soak --iterations 100000 --timeout-ms 200
//! ```
//!
//! Connect to a specific device via its USB serial number with
//! `--serial-number ABCD1234`. List connected boards with `cargo run -- list`.
//!
//! # Per-call timeout
//!
//! Every host-side I2C call is wrapped in `--per-call-timeout-ms` (default
//! 1000 ms). A wedged slave that stops clocking SCL will surface as a clean
//! timeout failure with the iteration number instead of blocking the harness
//! indefinitely. This matters for catching the
//! [pre-existing hardware race](https://github.com/OpenDevicePartnership/embassy-imxrt/pull/565#discussion_r3337586759)
//! that briefly mis-reports `SlaveAddress` mid-DMA under stress: when it
//! fires, the next host transaction times out cleanly instead of hanging.

use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use clap::{Parser, Subcommand};
use pico_de_gallo_lib::{I2cBatchOp, I2cFrequency, PicoDeGallo, list_devices};
use tokio::time::{sleep, timeout};

/// Default 7-bit I2C address advertised by the embassy-imxrt target-trait
/// example binaries (`i2c-slave-async-target-trait`, `i2c-slave-target-trait`).
const DEFAULT_SLAVE_ADDR: u8 = 0x20;

/// Slave's outgoing buffer used by the example tasks: `t_buf[i] = i as u8`
/// for `i` in `0..8`. This is what we expect to receive on a `Read`.
const EXPECTED_SLAVE_READ_PATTERN: [u8; 8] = [0, 1, 2, 3, 4, 5, 6, 7];

/// Default iteration count for the `all` subcommand's soak phase. Small
/// enough to finish in under a minute as a smoke test; bump
/// `cargo run -- soak --iterations N` for longer runs.
const DEFAULT_ALL_SOAK_ITERATIONS: u32 = 1_000_000;

/// Default per-call timeout in milliseconds. A single 100 kHz I2C transaction
/// of < 32 bytes finishes in < 10 ms on the wire, plus USB round-trip; 1 s
/// is comfortably generous without letting a wedged slave block the harness
/// for noticeable time.
const DEFAULT_PER_CALL_TIMEOUT_MS: u64 = 1000;

/// Default per-call timeout for the dedicated `write-read-soak` mode. Tighter
/// than the general default because the bug we are hunting wedges the slave
/// instantly and we want to fail fast.
const DEFAULT_WRSOAK_TIMEOUT_MS: u64 = 200;

#[derive(Parser)]
#[command(name = "i2c-target-test", version, about, long_about = None)]
struct Cli {
    /// USB serial number of the Pico de Gallo to connect to. Useful when
    /// multiple boards are plugged in. Without this flag, the first
    /// matching device is used.
    #[arg(short, long)]
    serial_number: Option<String>,

    /// 7-bit I2C address the slave is listening on. Must match the
    /// `SLAVE_ADDR` constant baked into the on-target example binary.
    #[arg(short, long, default_value_t = DEFAULT_SLAVE_ADDR)]
    address: u8,

    /// I2C bus frequency to drive from the controller side.
    #[arg(short, long, value_enum, default_value_t = FreqArg::Standard)]
    frequency: FreqArg,

    /// Per-test setup delay in milliseconds. Gives the slave time to loop
    /// back to `listen()` between sub-tests.
    #[arg(long, default_value_t = 50)]
    settle_ms: u64,

    /// Per-call timeout in milliseconds. Every host-side I2C transaction is
    /// wrapped in this budget. A timeout indicates the slave wedged (failed
    /// to clock the next byte / NAK the next address) and the bus is no
    /// longer making forward progress.
    #[arg(long, default_value_t = DEFAULT_PER_CALL_TIMEOUT_MS)]
    per_call_timeout_ms: u64,

    #[command(subcommand)]
    command: Cmd,
}

#[derive(clap::ValueEnum, Clone, Copy, Debug)]
enum FreqArg {
    Standard,
    Fast,
    FastPlus,
}

impl From<FreqArg> for I2cFrequency {
    fn from(v: FreqArg) -> Self {
        match v {
            FreqArg::Standard => I2cFrequency::Standard,
            FreqArg::Fast => I2cFrequency::Fast,
            FreqArg::FastPlus => I2cFrequency::FastPlus,
        }
    }
}

#[derive(Subcommand)]
enum Cmd {
    /// List all connected Pico de Gallo devices and exit.
    List,
    /// Scan the I2C bus for responding addresses (sanity check).
    Scan,
    /// Run every test case in sequence (uses a short soak phase).
    All,
    /// Write 8 bytes and expect `Request::Write(addr) + WriteStatus::Stopped(8)`.
    Write,
    /// Read 8 bytes and expect `Request::Read(addr) + ReadStatus::Complete(8)`.
    Read,
    /// Write-then-read in a single transaction with repeated START.
    /// Expects `Write(addr) -> Restarted(n) -> RepeatedStart(addr) -> Read(addr) -> Complete(m)`.
    WriteRead,
    /// Write more bytes than the slave's per-loop buffer (BUFLEN = 8) to
    /// exercise `WriteStatus::BufferFull` followed by a continuation.
    BufferFull,
    /// Read fewer bytes than the slave is prepared to send, terminating
    /// with NACK+STOP early. Expects `ReadStatus::EarlyStop`.
    EarlyStop,
    /// Read more bytes than the slave's per-loop buffer (BUFLEN = 8) to
    /// exercise `ReadStatus::NeedMore` followed by a continuation.
    NeedMore,
    /// Hammer the slave with a mix of writes/reads/write-reads to look for
    /// state-machine regressions. Records a histogram of outcomes
    /// (ok / timeout / payload-mismatch / usb-error) and prints it at the
    /// end; non-zero non-ok counts cause a non-zero exit.
    Soak {
        /// How many transactions to run.
        #[arg(long, default_value_t = 100)]
        iterations: u32,

        /// Stop on the first timeout (a wedged slave cannot recover on its
        /// own; further iterations will all fail). Other error classes
        /// (payload mismatch, USB error) are always counted but never
        /// short-circuit.
        #[arg(long, default_value_t = false)]
        fail_fast: bool,
    },
    /// Dedicated soak that hammers only `i2c_write_read` (write -> repeated
    /// START -> read), the transaction shape most likely to expose the
    /// `slv_state -> addressed` mid-DMA HW race noted in PR #565. Fails
    /// fast on the first timeout by default and surfaces the iteration
    /// number, the call leg (write vs read), and the wall-clock time it
    /// took to wedge.
    WriteReadSoak {
        /// How many transactions to run.
        #[arg(long, default_value_t = 100_000)]
        iterations: u32,

        /// Per-call timeout in milliseconds for *this* run (overrides the
        /// global `--per-call-timeout-ms` for the write-read-soak only).
        /// Defaults to 200 ms — tight enough to fail fast on a wedge.
        #[arg(long, default_value_t = DEFAULT_WRSOAK_TIMEOUT_MS)]
        timeout_ms: u64,

        /// Keep running past the first timeout (useful for measuring how
        /// often the race fires across a long run). Default is to fail
        /// fast.
        #[arg(long, default_value_t = false)]
        keep_going: bool,

        /// Number of payload bytes the controller writes to the slave on
        /// each iteration (1..=8).
        #[arg(long, default_value_t = 4)]
        write_len: usize,

        /// Number of payload bytes the controller reads back from the
        /// slave on each iteration (1..=8). Must not exceed the slave's
        /// 8-byte `t_buf` pattern.
        #[arg(long, default_value_t = 4)]
        read_len: usize,
    },
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    // `Cmd::List` is special: no device handle required.
    if let Cmd::List = cli.command {
        for dev in list_devices() {
            println!("{:?}", dev);
        }
        return Ok(());
    }

    let gallo = match cli.serial_number.as_deref() {
        Some(sn) => PicoDeGallo::new_with_serial_number(sn),
        None => PicoDeGallo::new(),
    };

    let freq: I2cFrequency = cli.frequency.into();
    gallo
        .i2c_set_config(freq)
        .await
        .map_err(|e| anyhow!("i2c_set_config failed: {:?}", e))?;
    println!("I2C frequency set to {:?}", freq);

    let ctx = Ctx {
        gallo,
        address: cli.address,
        settle: Duration::from_millis(cli.settle_ms),
        per_call_timeout: Duration::from_millis(cli.per_call_timeout_ms),
    };

    let result: Result<()> = match cli.command {
        Cmd::List => unreachable!("handled above"),
        Cmd::Scan => scan(&ctx).await,
        Cmd::All => run_all(&ctx).await,
        Cmd::Write => test_write(&ctx).await,
        Cmd::Read => test_read(&ctx).await,
        Cmd::WriteRead => test_write_read(&ctx).await,
        Cmd::BufferFull => test_buffer_full(&ctx).await,
        Cmd::EarlyStop => test_early_stop(&ctx).await,
        Cmd::NeedMore => test_need_more(&ctx).await,
        Cmd::Soak { iterations, fail_fast } => test_soak(&ctx, iterations, fail_fast).await,
        Cmd::WriteReadSoak {
            iterations,
            timeout_ms,
            keep_going,
            write_len,
            read_len,
        } => {
            test_write_read_soak(
                &ctx,
                iterations,
                Duration::from_millis(timeout_ms),
                keep_going,
                write_len,
                read_len,
            )
            .await
        }
    };

    match result {
        Ok(()) => {
            println!("\nAll requested tests PASSED.");
            Ok(())
        }
        Err(e) => {
            eprintln!("\nTEST FAILED: {e:#}");
            Err(e)
        }
    }
}

struct Ctx {
    gallo: PicoDeGallo,
    address: u8,
    settle: Duration,
    per_call_timeout: Duration,
}

impl Ctx {
    async fn settle(&self) {
        sleep(self.settle).await;
    }
}

/// Wrap a single host I2C call in the per-call timeout, mapping the three
/// possible outcomes (success / timeout / library error) into an
/// `anyhow::Result<T>` with informative messages.
async fn with_timeout<T, E>(
    budget: Duration,
    label: &str,
    fut: impl std::future::Future<Output = Result<T, E>>,
) -> Result<T>
where
    E: std::fmt::Debug,
{
    match timeout(budget, fut).await {
        Err(_) => bail!(
            "{label}: host call did not complete within {:?} — slave likely wedged",
            budget
        ),
        Ok(Err(e)) => bail!("{label}: {:?}", e),
        Ok(Ok(v)) => Ok(v),
    }
}

// ---------- Tests ----------

async fn run_all(ctx: &Ctx) -> Result<()> {
    scan(ctx).await?;
    test_write(ctx).await?;
    test_read(ctx).await?;
    test_write_read(ctx).await?;
    test_buffer_full(ctx).await?;
    test_early_stop(ctx).await?;
    test_need_more(ctx).await?;
    test_soak(ctx, DEFAULT_ALL_SOAK_ITERATIONS, false).await?;
    Ok(())
}

async fn scan(ctx: &Ctx) -> Result<()> {
    section("scan: probe all standard 7-bit addresses");
    let addrs = with_timeout(ctx.per_call_timeout, "i2c_scan", ctx.gallo.i2c_scan(false)).await?;
    println!("Addresses that ACKed: {}", format_addrs(&addrs));
    if !addrs.contains(&ctx.address) {
        bail!(
            "Slave address 0x{:02X} did not ACK during scan — is the target board powered, \
             flashed with one of the i2c-slave-*-target-trait examples, and wired up?",
            ctx.address
        );
    }
    println!("Slave address 0x{:02X} responded as expected.", ctx.address);
    ctx.settle().await;
    Ok(())
}

async fn test_write(ctx: &Ctx) -> Result<()> {
    section("write: send 8 bytes, expect WriteStatus::Stopped(8)");
    let payload: Vec<u8> = (0..8u8).collect();
    with_timeout(
        ctx.per_call_timeout,
        "8-byte write",
        ctx.gallo.i2c_write(ctx.address, &payload),
    )
    .await?;
    println!("Wrote {:?}", payload);
    ctx.settle().await;
    Ok(())
}

async fn test_read(ctx: &Ctx) -> Result<()> {
    section("read: request 8 bytes, expect ReadStatus::Complete(8)");
    let got = with_timeout(
        ctx.per_call_timeout,
        "8-byte read",
        ctx.gallo
            .i2c_read(ctx.address, EXPECTED_SLAVE_READ_PATTERN.len() as u16),
    )
    .await?;
    println!("Got {:?}", got);
    if got.as_slice() != EXPECTED_SLAVE_READ_PATTERN {
        bail!(
            "Read payload mismatch: expected {:?}, got {:?}",
            EXPECTED_SLAVE_READ_PATTERN,
            got,
        );
    }
    println!("Read payload matched expected pattern.");
    ctx.settle().await;
    Ok(())
}

async fn test_write_read(ctx: &Ctx) -> Result<()> {
    section("write-read: 4 bytes write -> repeated START -> 8 bytes read");
    let payload: Vec<u8> = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let got = with_timeout(
        ctx.per_call_timeout,
        "write_read",
        ctx.gallo
            .i2c_write_read(ctx.address, &payload, EXPECTED_SLAVE_READ_PATTERN.len() as u16),
    )
    .await?;
    println!("Wrote {:?}, then read {:?}", payload, got);
    if got.as_slice() != EXPECTED_SLAVE_READ_PATTERN {
        bail!(
            "write_read read payload mismatch: expected {:?}, got {:?}",
            EXPECTED_SLAVE_READ_PATTERN,
            got,
        );
    }
    println!(
        "Slave should have surfaced: Write(0x{addr:02X}) -> Restarted(n) -> RepeatedStart(0x{addr:02X}) -> \
         Read(0x{addr:02X}) -> Complete({n})",
        addr = ctx.address,
        n = EXPECTED_SLAVE_READ_PATTERN.len(),
    );
    ctx.settle().await;
    Ok(())
}

async fn test_buffer_full(ctx: &Ctx) -> Result<()> {
    section("buffer-full: write 24 bytes (3x the slave's 8-byte buffer)");
    // The slave's inner respond_to_write loop drains into an 8-byte buf,
    // so 3*8=24 bytes forces two `BufferFull` continuations followed by a
    // final `Stopped`. The slave example loops on BufferFull and re-arms
    // a fresh buffer, so this should still ACK every byte.
    let payload: Vec<u8> = (0..24u8).collect();
    with_timeout(
        ctx.per_call_timeout,
        "24-byte write",
        ctx.gallo.i2c_write(ctx.address, &payload),
    )
    .await?;
    println!("Wrote {} bytes spanning 3 BufferFull cycles.", payload.len());
    ctx.settle().await;
    Ok(())
}

async fn test_early_stop(ctx: &Ctx) -> Result<()> {
    section("early-stop: read 4 of 8 bytes, controller NACK+STOP early");
    let got = with_timeout(ctx.per_call_timeout, "short read", ctx.gallo.i2c_read(ctx.address, 4)).await?;
    println!("Got {:?}", got);
    if got != EXPECTED_SLAVE_READ_PATTERN[..4] {
        bail!(
            "Short read payload mismatch: expected {:?}, got {:?}",
            &EXPECTED_SLAVE_READ_PATTERN[..4],
            got,
        );
    }
    println!(
        "Slave should have surfaced ReadStatus::EarlyStop(4) (or Complete(4) — both are valid \
         depending on whether the slave saw the NACK before the buffer ran out)."
    );
    ctx.settle().await;
    Ok(())
}

async fn test_need_more(ctx: &Ctx) -> Result<()> {
    section("need-more: read 16 bytes (2x the slave's 8-byte buffer)");
    let got = with_timeout(
        ctx.per_call_timeout,
        "16-byte read",
        ctx.gallo.i2c_read(ctx.address, 16),
    )
    .await?;
    println!("Got {:?}", got);
    // The slave example resends the same 8-byte pattern when NeedMore
    // fires, so we expect the buffer to be the pattern twice in a row.
    let mut expected = Vec::with_capacity(16);
    expected.extend_from_slice(&EXPECTED_SLAVE_READ_PATTERN);
    expected.extend_from_slice(&EXPECTED_SLAVE_READ_PATTERN);
    if got != expected {
        bail!(
            "16-byte read payload mismatch:\n  expected {:?}\n  got      {:?}",
            expected,
            got,
        );
    }
    println!("Slave should have surfaced NeedMore(8) once, then Complete(8) on the second pass.");
    ctx.settle().await;
    Ok(())
}

/// Outcome of a single soak iteration. Keeps timeout (wedge) distinct from
/// generic USB / protocol errors so the harness can fail fast on wedges and
/// merely tally non-fatal errors.
#[derive(Debug, Clone)]
enum SoakOutcome {
    Ok,
    Timeout,
    PayloadMismatch { expected: Vec<u8>, got: Vec<u8> },
    UsbError(String),
}

#[derive(Default, Debug, Clone, Copy)]
struct SoakStats {
    ok: u32,
    timeouts: u32,
    mismatches: u32,
    usb_errors: u32,
}

impl SoakStats {
    fn record(&mut self, outcome: &SoakOutcome) {
        match outcome {
            SoakOutcome::Ok => self.ok += 1,
            SoakOutcome::Timeout => self.timeouts += 1,
            SoakOutcome::PayloadMismatch { .. } => self.mismatches += 1,
            SoakOutcome::UsbError(_) => self.usb_errors += 1,
        }
    }

    fn total_errors(&self) -> u32 {
        self.timeouts + self.mismatches + self.usb_errors
    }
}

/// Run a single soak call with the supplied timeout. Optionally verify the
/// returned payload against `expected`. Used by both `test_soak` and
/// `test_write_read_soak`.
async fn run_soak_call<T, E>(
    budget: Duration,
    fut: impl std::future::Future<Output = Result<T, E>>,
    expected: Option<&[u8]>,
    extract_payload: impl FnOnce(&T) -> Option<&[u8]>,
) -> SoakOutcome
where
    E: std::fmt::Debug,
{
    match timeout(budget, fut).await {
        Err(_) => SoakOutcome::Timeout,
        Ok(Err(e)) => SoakOutcome::UsbError(format!("{:?}", e)),
        Ok(Ok(v)) => {
            if let (Some(expected), Some(got)) = (expected, extract_payload(&v))
                && got != expected
            {
                return SoakOutcome::PayloadMismatch {
                    expected: expected.to_vec(),
                    got: got.to_vec(),
                };
            }
            SoakOutcome::Ok
        }
    }
}

async fn test_soak(ctx: &Ctx, iterations: u32, fail_fast: bool) -> Result<()> {
    section(&format!("soak: {iterations} mixed transactions back-to-back"));
    let mut stats = SoakStats::default();
    let started_at = std::time::Instant::now();

    for i in 0..iterations {
        let (label, outcome) = match i % 4 {
            0 => {
                let outcome = run_soak_call(
                    ctx.per_call_timeout,
                    ctx.gallo.i2c_write(ctx.address, &[(i & 0xFF) as u8]),
                    None,
                    |_: &()| None,
                )
                .await;
                ("write(1)", outcome)
            }
            1 => {
                let outcome = run_soak_call(
                    ctx.per_call_timeout,
                    ctx.gallo.i2c_read(ctx.address, 4),
                    None,
                    |_: &Vec<u8>| None,
                )
                .await;
                ("read(4)", outcome)
            }
            2 => {
                let outcome = run_soak_call(
                    ctx.per_call_timeout,
                    ctx.gallo.i2c_write_read(ctx.address, &[(i & 0xFF) as u8, 0xA5], 4),
                    None,
                    |_: &Vec<u8>| None,
                )
                .await;
                ("write_read(2,4)", outcome)
            }
            _ => {
                let ops = [I2cBatchOp::Write { data: &[0x55, 0xAA] }, I2cBatchOp::Read { len: 2 }];
                let outcome = run_soak_call(
                    ctx.per_call_timeout,
                    ctx.gallo.i2c_batch(ctx.address, &ops),
                    None,
                    |_: &Vec<u8>| None,
                )
                .await;
                ("batch(write+read)", outcome)
            }
        };

        stats.record(&outcome);
        match outcome {
            SoakOutcome::Ok => {}
            SoakOutcome::Timeout => {
                eprintln!("  iteration {i:>6} ({label}) TIMEOUT after {:?}", ctx.per_call_timeout);
                if fail_fast {
                    print_soak_summary("soak", &stats, iterations, started_at.elapsed());
                    bail!(
                        "Soak aborted on first timeout at iteration {i} (--fail-fast). \
                         A wedged slave cannot recover without recover() being called; \
                         further iterations would all time out."
                    );
                }
            }
            SoakOutcome::PayloadMismatch { expected, got } => {
                eprintln!(
                    "  iteration {i:>6} ({label}) payload mismatch: expected {:?}, got {:?}",
                    expected, got
                );
            }
            SoakOutcome::UsbError(msg) => {
                eprintln!("  iteration {i:>6} ({label}) usb/protocol error: {msg}");
            }
        }
    }

    print_soak_summary("soak", &stats, iterations, started_at.elapsed());
    if stats.total_errors() > 0 {
        bail!(
            "Soak test recorded {} errors across {iterations} iterations.",
            stats.total_errors()
        );
    }
    Ok(())
}

async fn test_write_read_soak(
    ctx: &Ctx,
    iterations: u32,
    timeout_budget: Duration,
    keep_going: bool,
    write_len: usize,
    read_len: usize,
) -> Result<()> {
    if !(1..=EXPECTED_SLAVE_READ_PATTERN.len()).contains(&write_len) {
        bail!(
            "write_len {write_len} out of range 1..={}",
            EXPECTED_SLAVE_READ_PATTERN.len()
        );
    }
    if !(1..=EXPECTED_SLAVE_READ_PATTERN.len()).contains(&read_len) {
        bail!(
            "read_len {read_len} out of range 1..={}",
            EXPECTED_SLAVE_READ_PATTERN.len()
        );
    }

    section(&format!(
        "write-read-soak: {iterations} write({write_len})+restart+read({read_len}) \
         transactions, per-call timeout {:?}{}",
        timeout_budget,
        if keep_going {
            " (--keep-going)"
        } else {
            " (fail-fast on first timeout)"
        }
    ));

    println!(
        "Hunting for the slv_state -> addressed mid-DMA HW race noted in PR #565. \
         A wedged slave will manifest as a timeout on either the write leg or the \
         read leg of i2c_write_read; the iteration number and the leg will be \
         printed on the first failure."
    );

    let mut stats = SoakStats::default();
    let expected = &EXPECTED_SLAVE_READ_PATTERN[..read_len];
    let started_at = std::time::Instant::now();

    for i in 0..iterations {
        // Cycle the write payload so consecutive iterations don't look
        // identical on the wire (just in case any controller-side caching
        // is masking the bug).
        let write_payload: Vec<u8> = (0..write_len).map(|k| ((i as usize + k) & 0xFF) as u8).collect();

        let outcome = run_soak_call(
            timeout_budget,
            ctx.gallo.i2c_write_read(ctx.address, &write_payload, read_len as u16),
            Some(expected),
            |v: &Vec<u8>| Some(v.as_slice()),
        )
        .await;

        stats.record(&outcome);

        match outcome {
            SoakOutcome::Ok => {
                if i.is_multiple_of(1000) && i > 0 {
                    println!(
                        "  ...iteration {i:>6} OK ({:.0} tx/s)",
                        i as f64 / started_at.elapsed().as_secs_f64()
                    );
                }
            }
            SoakOutcome::Timeout => {
                let elapsed = started_at.elapsed();
                eprintln!(
                    "  iteration {i:>6} TIMEOUT after {:?} — slave likely wedged on the \
                     write or read leg of i2c_write_read. Reproducer: \
                     `cargo run -- write-read-soak --iterations {} --timeout-ms {} \
                     --write-len {write_len} --read-len {read_len}`.",
                    timeout_budget,
                    iterations,
                    timeout_budget.as_millis(),
                );
                eprintln!("    write payload was: {:?}", write_payload);
                eprintln!("    elapsed since soak start: {:?}", elapsed);

                if !keep_going {
                    print_soak_summary("write-read-soak", &stats, iterations, elapsed);
                    bail!(
                        "Write-read soak wedged at iteration {i}/{iterations} (after {:?}). \
                         Re-run with --keep-going to measure how often the race fires; pair \
                         with `defmt-print` against the slave RTT channel to capture the \
                         on-target event history.",
                        elapsed
                    );
                }
            }
            SoakOutcome::PayloadMismatch { expected, got } => {
                eprintln!(
                    "  iteration {i:>6} payload mismatch: expected {:?}, got {:?}",
                    expected, got
                );
            }
            SoakOutcome::UsbError(msg) => {
                eprintln!("  iteration {i:>6} usb/protocol error: {msg}");
            }
        }
    }

    print_soak_summary("write-read-soak", &stats, iterations, started_at.elapsed());
    if stats.total_errors() > 0 {
        bail!(
            "Write-read soak recorded {} errors across {iterations} iterations \
             (timeouts: {}, mismatches: {}, usb_errors: {}).",
            stats.total_errors(),
            stats.timeouts,
            stats.mismatches,
            stats.usb_errors,
        );
    }
    Ok(())
}

// ---------- Helpers ----------

fn section(title: &str) {
    println!("\n=== {title} ===");
}

fn print_soak_summary(label: &str, stats: &SoakStats, iterations: u32, elapsed: Duration) {
    let attempted = stats.ok + stats.total_errors();
    let rate = if elapsed.as_secs_f64() > 0.0 {
        attempted as f64 / elapsed.as_secs_f64()
    } else {
        0.0
    };
    println!(
        "\n{label} summary: {ok} ok / {timeouts} timeout / {mismatches} mismatch / \
         {usb} usb-error out of {attempted}/{iterations} attempted in {:?} ({rate:.0} tx/s)",
        elapsed,
        ok = stats.ok,
        timeouts = stats.timeouts,
        mismatches = stats.mismatches,
        usb = stats.usb_errors,
        attempted = attempted,
        iterations = iterations,
        rate = rate,
    );
}

fn format_addrs(addrs: &[u8]) -> String {
    if addrs.is_empty() {
        "(none)".to_string()
    } else {
        let mut s = String::new();
        for (i, a) in addrs.iter().enumerate() {
            if i > 0 {
                s.push(' ');
            }
            s.push_str(&format!("0x{:02X}", a));
        }
        s
    }
}

//! Cyclic Redundancy Check (CRC)

use core::marker::PhantomData;

use embassy_hal_internal::into_ref;

use crate::clocks::{enable_and_reset, SysconPeripheral};
use crate::{peripherals, Peripheral};

/// CRC driver.
pub struct Crc<'d> {
    info: Info,
    _config: Config,
    _lifetime: PhantomData<&'d ()>,
}

/// CRC configuration
pub struct Config {
    /// Polynomial to be used
    pub polynomial: Polynomial,

    /// Reverse bit order of input?
    pub bit_order_input_reverse: bool,

    /// 1's complement input?
    pub input_complement: bool,

    /// Reverse CRC bit order?
    pub bit_order_crc_reverse: bool,

    /// 1's complement CRC?
    pub crc_complement: bool,

    /// CRC Seed
    pub seed: u32,
}

impl Config {
    /// Create a new CRC config.
    #[must_use]
    pub fn new(
        polynomial: Polynomial,
        bit_order_input_reverse: bool,
        input_complement: bool,
        bit_order_crc_reverse: bool,
        crc_complement: bool,
        seed: u32,
    ) -> Self {
        Config {
            polynomial,
            bit_order_input_reverse,
            input_complement,
            bit_order_crc_reverse,
            crc_complement,
            seed,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            polynomial: Polynomial::default(),
            bit_order_input_reverse: false,
            input_complement: false,
            bit_order_crc_reverse: false,
            crc_complement: false,
            seed: 0xffff,
        }
    }
}

/// CRC polynomial
#[derive(Debug, Copy, Clone, Default)]
pub enum Polynomial {
    /// CRC-32: 0x04C11DB7
    Crc32,
    /// CRC-16: 0x8005
    Crc16,
    /// CRC-CCITT: 0x1021
    #[default]
    CrcCcitt,
}

impl From<Polynomial> for u8 {
    fn from(polynomial: Polynomial) -> u8 {
        match polynomial {
            Polynomial::Crc16 => 1,
            Polynomial::CrcCcitt => 0,
            _ => 2,
        }
    }
}

impl<'d> Crc<'d> {
    /// Instantiates new CRC peripheral and initializes to default values.
    pub fn new<T: Instance>(_peripheral: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        // enable CRC clock
        enable_and_reset::<T>();

        into_ref!(_peripheral);

        let mut instance = Self {
            info: T::info(),
            _config: config,
            _lifetime: PhantomData,
        };

        instance.reconfigure();
        instance
    }

    /// Reconfigured the CRC peripheral.
    fn reconfigure(&mut self) {
        self.info.regs.mode().write(|w| {
            unsafe { w.crc_poly().bits(self._config.polynomial.into()) }
                .bit_rvs_wr()
                .variant(self._config.bit_order_input_reverse)
                .cmpl_wr()
                .variant(self._config.input_complement)
                .bit_rvs_sum()
                .variant(self._config.bit_order_crc_reverse)
                .cmpl_sum()
                .variant(self._config.crc_complement)
        });

        // Init CRC value
        self.info
            .regs
            .seed()
            .write(|w| unsafe { w.crc_seed().bits(self._config.seed) });
    }

    /// Feeds a byte into the CRC peripheral. Returns the computed checksum.
    pub fn feed_byte(&mut self, byte: u8) -> u32 {
        self.info.regs.wr_data8().write(|w| unsafe { w.bits(byte) });

        self.info.regs.sum().read().bits()
    }

    /// Feeds an slice of bytes into the CRC peripheral. Returns the computed checksum.
    pub fn feed_bytes(&mut self, bytes: &[u8]) -> u32 {
        let (prefix, data, suffix) = unsafe { bytes.align_to::<u32>() };

        for b in prefix {
            self.info.regs.wr_data8().write(|w| unsafe { w.bits(*b) });
        }

        for d in data {
            self.info.regs.wr_data32().write(|w| unsafe { w.bits(*d) });
        }

        for b in suffix {
            self.info.regs.wr_data8().write(|w| unsafe { w.bits(*b) });
        }

        self.info.regs.sum().read().bits()
    }

    /// Feeds a halfword into the CRC peripheral. Returns the computed checksum.
    pub fn feed_halfword(&mut self, halfword: u16) -> u32 {
        self.info.regs.wr_data16().write(|w| unsafe { w.bits(halfword) });

        self.info.regs.sum().read().bits()
    }

    /// Feeds an slice of halfwords into the CRC peripheral. Returns the computed checksum.
    pub fn feed_halfwords(&mut self, halfwords: &[u16]) -> u32 {
        for halfword in halfwords {
            self.info.regs.wr_data16().write(|w| unsafe { w.bits(*halfword) });
        }

        self.info.regs.sum().read().bits()
    }

    /// Feeds a words into the CRC peripheral. Returns the computed checksum.
    pub fn feed_word(&mut self, word: u32) -> u32 {
        self.info.regs.wr_data32().write(|w| unsafe { w.bits(word) });

        self.info.regs.sum().read().bits()
    }

    /// Feeds an slice of words into the CRC peripheral. Returns the computed checksum.
    pub fn feed_words(&mut self, words: &[u32]) -> u32 {
        for word in words {
            self.info.regs.wr_data32().write(|w| unsafe { w.bits(*word) });
        }

        self.info.regs.sum().read().bits()
    }
}

struct Info {
    regs: crate::pac::CrcEngine,
}

trait SealedInstance {
    fn info() -> Info;
}

/// CRC instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Peripheral<P = Self> + SysconPeripheral + 'static + Send {}

impl Instance for peripherals::CRC {}

impl SealedInstance for peripherals::CRC {
    fn info() -> Info {
        // SAFETY: safe from single executor
        Info {
            regs: unsafe { crate::pac::CrcEngine::steal() },
        }
    }
}

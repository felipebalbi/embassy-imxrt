//! Hashcrypt
use core::marker::PhantomData;

use hasher::Hasher;

use crate::clocks::enable_and_reset;
use crate::dma::AnyChannel;
use crate::peripherals::{DMA0_CH30, HASHCRYPT};
use crate::{dma, pac, Peri};

/// Hasher module
pub mod hasher;

trait Sealed {}

/// Asynchronous or blocking mode
#[allow(private_bounds)]
pub trait Mode: Sealed {}

/// Blocking mode
pub struct Blocking {}
impl Sealed for Blocking {}
impl Mode for Blocking {}

/// Asynchronous mode
pub struct Async {}
impl Sealed for Async {}
impl Mode for Async {}

/// Trait for compatible DMA channels
#[allow(private_bounds)]
pub trait HashcryptDma: dma::Channel {}
impl Sealed for DMA0_CH30 {}
impl HashcryptDma for DMA0_CH30 {}

/// Hashcrypt driver
pub struct Hashcrypt<'d, M: Mode> {
    hashcrypt: pac::Hashcrypt,
    dma_ch: Option<Peri<'d, AnyChannel>>,
    _peripheral: Peri<'d, HASHCRYPT>,
    _mode: PhantomData<M>,
}

/// Hashcrypt mode
#[derive(Debug, Copy, Clone)]
#[non_exhaustive]
enum Algorithm {
    /// SHA256
    SHA256,
}

impl From<Algorithm> for u8 {
    fn from(value: Algorithm) -> Self {
        match value {
            Algorithm::SHA256 => 0x2,
        }
    }
}

impl<'d, M: Mode> Hashcrypt<'d, M> {
    /// Instantiate new Hashcrypt peripheral
    fn new_inner(peripheral: Peri<'d, HASHCRYPT>, dma_ch: Option<Peri<'d, AnyChannel>>) -> Self {
        enable_and_reset::<HASHCRYPT>();

        Self {
            _peripheral: peripheral,
            _mode: PhantomData,
            dma_ch,
            hashcrypt: unsafe { pac::Hashcrypt::steal() },
        }
    }

    // Safety: unsafe for writing algorithm type to register
    fn start_algorithm(&mut self, algorithm: Algorithm, dma: bool) {
        self.hashcrypt.ctrl().write(|w| w.mode().disabled().new_hash().start());
        self.hashcrypt.ctrl().write(|w| {
            unsafe { w.mode().bits(algorithm.into()) }.new_hash().start();
            if dma {
                w.dma_i().set_bit();
            }
            w
        });
    }
}

impl<'d> Hashcrypt<'d, Blocking> {
    /// Create a new instance
    pub fn new_blocking(peripheral: Peri<'d, HASHCRYPT>) -> Self {
        Self::new_inner(peripheral, None)
    }

    /// Start a new SHA256 hash
    pub fn new_sha256<'a>(&'a mut self) -> Hasher<'d, 'a, Blocking> {
        self.start_algorithm(Algorithm::SHA256, false);
        Hasher::new_blocking(self)
    }
}

impl<'d> Hashcrypt<'d, Async> {
    /// Create a new instance
    pub fn new_async(peripheral: Peri<'d, HASHCRYPT>, dma_ch: Peri<'d, impl HashcryptDma>) -> Self {
        Self::new_inner(peripheral, Some(dma_ch.into()))
    }

    /// Start a new SHA256 hash
    pub fn new_sha256<'a>(&'a mut self) -> Hasher<'d, 'a, Async> {
        self.start_algorithm(Algorithm::SHA256, true);
        Hasher::new_async(self)
    }
}

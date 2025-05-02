#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_imxrt::dma;
use {defmt_rtt as _, panic_probe as _};

const TEST_LEN: usize = 16;

macro_rules! test_dma_channel {
    ($peripherals:expr, $src:ident, $dst:ident, $number:expr) => {
        unsafe { dma::copy($peripherals, &$src, &mut $dst) }.await;

        if $src == $dst {
            info!("DMA copy on channel {} completed successfully", $number);
        } else {
            error!("DMA copy on channel {} failed!", $number);
        }
    };
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_imxrt::init(Default::default());

    let srcbuf = [0x55_u8; TEST_LEN];
    let mut dstbuf = [0u8; TEST_LEN];

    info!("Test memory-to-memory DMA transfers");

    test_dma_channel!(p.DMA0_CH0, srcbuf, dstbuf, 0);
    test_dma_channel!(p.DMA0_CH1, srcbuf, dstbuf, 1);
    test_dma_channel!(p.DMA0_CH2, srcbuf, dstbuf, 2);
    test_dma_channel!(p.DMA0_CH3, srcbuf, dstbuf, 3);
    test_dma_channel!(p.DMA0_CH4, srcbuf, dstbuf, 4);
    test_dma_channel!(p.DMA0_CH5, srcbuf, dstbuf, 5);
    test_dma_channel!(p.DMA0_CH6, srcbuf, dstbuf, 6);
    test_dma_channel!(p.DMA0_CH7, srcbuf, dstbuf, 7);
    test_dma_channel!(p.DMA0_CH8, srcbuf, dstbuf, 8);
    test_dma_channel!(p.DMA0_CH9, srcbuf, dstbuf, 9);
    test_dma_channel!(p.DMA0_CH10, srcbuf, dstbuf, 10);
    test_dma_channel!(p.DMA0_CH11, srcbuf, dstbuf, 11);
    test_dma_channel!(p.DMA0_CH12, srcbuf, dstbuf, 12);
    test_dma_channel!(p.DMA0_CH13, srcbuf, dstbuf, 13);
    test_dma_channel!(p.DMA0_CH14, srcbuf, dstbuf, 14);
    test_dma_channel!(p.DMA0_CH15, srcbuf, dstbuf, 15);
    test_dma_channel!(p.DMA0_CH16, srcbuf, dstbuf, 16);
    test_dma_channel!(p.DMA0_CH17, srcbuf, dstbuf, 17);
    test_dma_channel!(p.DMA0_CH18, srcbuf, dstbuf, 18);
    test_dma_channel!(p.DMA0_CH19, srcbuf, dstbuf, 19);
    test_dma_channel!(p.DMA0_CH20, srcbuf, dstbuf, 20);
    test_dma_channel!(p.DMA0_CH21, srcbuf, dstbuf, 21);
    test_dma_channel!(p.DMA0_CH22, srcbuf, dstbuf, 22);
    test_dma_channel!(p.DMA0_CH23, srcbuf, dstbuf, 23);
    test_dma_channel!(p.DMA0_CH24, srcbuf, dstbuf, 24);
    test_dma_channel!(p.DMA0_CH25, srcbuf, dstbuf, 25);
    test_dma_channel!(p.DMA0_CH26, srcbuf, dstbuf, 26);
    test_dma_channel!(p.DMA0_CH27, srcbuf, dstbuf, 27);
    test_dma_channel!(p.DMA0_CH28, srcbuf, dstbuf, 28);
    test_dma_channel!(p.DMA0_CH29, srcbuf, dstbuf, 29);
    test_dma_channel!(p.DMA0_CH30, srcbuf, dstbuf, 30);
    test_dma_channel!(p.DMA0_CH31, srcbuf, dstbuf, 31);

    info!("DMA transfer tests completed");
}

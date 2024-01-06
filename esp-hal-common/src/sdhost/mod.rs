use core::sync::atomic::compiler_fence;

use esp32s3::sdhost::{bmod::BMOD_SPEC, cmd::CMD_SPEC, RegisterBlock};

use crate::{
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal, ONE_INPUT, ZERO_INPUT},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::GPIO,
    soc::is_valid_ram_address,
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
    systimer::SystemTimer,
    Timer,
};

#[derive(PartialEq)]
enum Slot {
    Slot0,
    Slot1,
}

#[derive(PartialEq)]
enum ClockSource {
    Xtal,
    PLL160M,
}

mod command;
mod dma;

#[cfg_attr(esp32s3, path = "sdhost_ll/esp32s3.rs")]
pub(crate) mod sdhost_ll;

use core::{
    mem::{size_of, MaybeUninit},
    pin::Pin,
    ptr::{addr_of, addr_of_mut},
    task::{Context, Poll},
};

use command::Command;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::waitqueue::AtomicWaker;
use esp32s3::{
    generic::{Readable, Reg, Resettable, Writable},
    sdhost::cmd::W,
};
use procmacros::interrupt;

use self::dma::{DmaLinkedListDes0, DmaLinkedListDes1};
use crate::clock::Clocks;

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct SdHost<'d, D3: InputPin + OutputPin> {
    _instance: PeripheralRef<'d, crate::peripherals::SDHOST>,
    sdhost: &'d crate::peripherals::sdhost::RegisterBlock,
    slot: Slot,
    rca: u16,
    d3: PeripheralRef<'d, D3>,
    sector_size: u16,
    num_blocks: u32,
}

// Steps:
// inspired by https://github.com/espressif/esp-idf/blob/692c1fcc52b9b936c73dead4ef0c2ea1fbdfb602/components/fatfs/vfs/vfs_fat_sdmmc.c#L233
// sdmmc_host_init -> https://github.com/espressif/esp-idf/blob/692c1fcc52b9b936c73dead4ef0c2ea1fbdfb602/components/esp_driver_sdmmc/src/sdmmc_host.c#L380

extern "C" {
    fn ets_delay_us(delay: u32);
}

macro_rules! do_command {
    ($self:ident, $code:block) => {
        async {
            let error = SdhostFuture::new(Event::ResponseError, $self.sdhost);
            let timeout = SdhostFuture::new(Event::Timeout, $self.sdhost);
            let done = SdhostFuture::new(Event::CommandDone, $self.sdhost);

            // Your provided code block
            $code

            match select3(
                error,
                timeout,
                done,
            )
            .await
            {
                Either3::First(_) => Err(Error::InvalidResponse),
                Either3::Second(_) => Err(Error::Timeout),
                Either3::Third(_) => Ok(()),
            }
        }
    };
}

macro_rules! do_command_meh {
    ($self:ident, $done:expr, $code:block) => {
        async {
            let error = SdhostFuture::new(Event::ResponseError, $self.sdhost);
            let timeout = SdhostFuture::new(Event::Timeout, $self.sdhost);
            let done = SdhostFuture::new($done, $self.sdhost);

            // Your provided code block
            $code

            match select3(
                error,
                timeout,
                done,
            )
            .await
            {
                Either3::First(_) => Err(Error::InvalidResponse),
                Either3::Second(_) => Err(Error::Timeout),
                Either3::Third(_) => Ok(()),
            }
        }
    };
}

// #[cfg(feature = "async")]
impl<'d, D3: InputPin + OutputPin> SdHost<'d, D3> {
    pub fn new<
        CLK: OutputPin,
        CMD: InputPin + OutputPin,
        D0: InputPin + OutputPin,
        D1: InputPin + OutputPin,
        D2: InputPin + OutputPin,
    >(
        _instance: impl Peripheral<P = crate::peripherals::SDHOST> + 'd,
        clk: impl Peripheral<P = CLK> + 'd,
        cmd: impl Peripheral<P = CMD> + 'd,
        d0: impl Peripheral<P = D0> + 'd,
        d1: impl Peripheral<P = D1> + 'd,
        d2: impl Peripheral<P = D2> + 'd,
        d3: impl Peripheral<P = D3> + 'd,
    ) -> Self {
        crate::into_ref!(_instance);

        let sdhost = unsafe { &*crate::peripherals::SDHOST::ptr() };
        let slot = Slot::Slot0;

        crate::into_ref!(clk);
        clk.enable_output(true);
        clk.connect_peripheral_to_output(slot.clk_output_signal());

        crate::into_ref!(cmd);
        cmd.enable_output(true);
        cmd.connect_peripheral_to_output(slot.cmd_output_signal());
        cmd.enable_input(true);
        cmd.connect_input_to_peripheral(slot.cmd_input_signal());

        crate::into_ref!(d0);
        d0.enable_output(true);
        d0.connect_peripheral_to_output(slot.d0_output_signal());
        d0.enable_input(true);
        d0.connect_input_to_peripheral(slot.d0_input_signal());

        crate::into_ref!(d1);
        d1.enable_output(true);
        d1.connect_peripheral_to_output(slot.d1_output_signal());
        d1.enable_input(true);
        d1.connect_input_to_peripheral(slot.d1_input_signal());

        crate::into_ref!(d2);
        d2.enable_output(true);
        d2.connect_peripheral_to_output(slot.d2_output_signal());
        d2.enable_input(true);
        d2.connect_input_to_peripheral(slot.d2_input_signal());

        crate::into_ref!(d3);
        d3.enable_output(true);
        d3.connect_peripheral_to_output(slot.d3_output_signal());
        d3.enable_input(true);
        d3.connect_input_to_peripheral(slot.d3_input_signal());

        // set this and card_detect to high to enable sdio interrupt
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(slot.int_input_signal() as usize)
            .modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(ONE_INPUT)
            });

        // Default to CD low (card present)
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(slot.cd_input_signal() as usize)
            .modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(ZERO_INPUT)
            });

        // Default to WP high (not write protected)
        unsafe { &*GPIO::PTR }
            .func_in_sel_cfg(slot.wp_input_signal() as usize)
            .modify(|_, w| unsafe {
                w.sel()
                    .set_bit()
                    .in_inv_sel()
                    .bit(false)
                    .in_sel()
                    .bits(ONE_INPUT)
            });

        let sdh = SdHost {
            _instance,
            sdhost,
            slot,
            d3,
            rca: 0,
            sector_size: 0,
            num_blocks: 0,
        };

        sdh.host_init();

        sdh
    }

    pub fn num_blocks(&self) -> u32 {
        self.num_blocks
    }

    fn host_init(&self) {
        PeripheralClockControl::enable(PeripheralEnable::SdHost);

        // Enable clock to peripheral. Use smallest divider first.
        self.set_host_clock_divider(2);

        self.host_reset();

        // Clear interrupt status and set interrupt mask to known state
        self.sdhost
            .rintsts()
            .write(|w| unsafe { w.bits(0xffffffff) });
        self.sdhost
            .intmask()
            .write(|w| unsafe { w.int_mask().bits(0) });
        self.sdhost.ctrl().write(|w| w.int_enable().clear_bit());

        // Enable interrupts
        self.sdhost.intmask().write(|w| unsafe {
            w.bits(
                (SDMMC_INTMASK_CD
                    | SDMMC_INTMASK_CMD_DONE
                    | SDMMC_INTMASK_DATA_OVER
                    | SDMMC_INTMASK_RCRC
                    | SDMMC_INTMASK_DCRC
                    | SDMMC_INTMASK_RTO
                    | SDMMC_INTMASK_DTO
                    | SDMMC_INTMASK_HTO
                    | SDMMC_INTMASK_SBE
                    | SDMMC_INTMASK_EBE
                    | SDMMC_INTMASK_RESP_ERR
                    | SDMMC_INTMASK_HLE) as u32,
            )
        }); // sdio is enabled only when use.
        self.sdhost.ctrl().write(|w| w.int_enable().set_bit());
        self.sdhost
            .idinten()
            .write(|w| w.ai().clear_bit().ni().set_bit().ces().clear_bit());

        // Disable generation of Busy Clear Interrupt
        self.sdhost
            .cardthrctl()
            .write(|w| w.cardclrinten().clear_bit());

        self.dma_init();
    }

    async fn card_init(&mut self, clocks: &Clocks<'_>) -> Result<(), Error> {
        // TODO: Not required? Not using SDIO
        // self.io_reset().await?;

        // GO_IDLE_STATE (CMD0) command resets the card
        self.send_go_idle().await?;

        // SEND_IF_COND (CMD8) command is used to identify SDHC/SDXC cards.
        self.init_sd_if_cond().await?;

        // TODO: Not required? Not using SDIO
        // IO_SEND_OP_COND(CMD5), Determine if the card is an IO card.
        // self.init_io().await?;

        // Use SEND_OP_COND to set up card OCR
        let ocr = self.init_ocr().await?;
        info!("ocr: {:?}", ocr);

        // Read the contents of CID register
        self.init_cid().await?;

        // Assign RCA
        self.init_rca().await?;
        info!("rca {:?}", self.rca);

        // Read and decode the contents of CSD register
        let sector_size = self.init_csd().await?;
        self.sector_size = sector_size;

        // Switch the card from stand-by mode to data transfer mode (not needed if
        // SPI interface is used). This is needed to issue SET_BLOCKLEN and
        // SEND_SCR commands.
        self.init_select_card().await?;

        // Set block len for SDSC cards to 512 bytes (same as SDHC)
        // Read SCR
        // Wait to enter data transfer state
        self.init_sd_blocklen(sector_size).await?;
        self.init_sd_scr().await?;
        self.init_sd_wait_data_ready().await?;

        // Try to switch card to HS mode if the card supports it.
        // Set card->max_freq_khz value accordingly.
        self.init_card_hs_mode().await?;

        self.init_card_bus_width().await?;
        self.set_bus_width(4)?;

        Ok(())
    }

    fn dma_init(&self) {
        self.sdhost
            .bmod()
            .write(|w| unsafe { w.bits(BMOD_SPEC::RESET_VALUE).swr().set_bit() });
    }

    fn sdmmc_host_dma_prepare(&self, addr: u32, block_size: u16, buffer_size: u32) {
        // Set size of data and DMA descriptor pointer
        sdhost_ll::set_data_transfer_len(buffer_size);
        sdhost_ll::set_block_size(block_size);
        sdhost_ll::set_desc_addr(addr);

        // Enable everything needed to use DMA
        sdhost_ll::enable_dma(true);
        sdhost_ll::poll_demand(true);
    }

    fn host_reset(&self) {
        self.sdhost.ctrl().write(|w| unsafe {
            w.controller_reset()
                .set_bit()
                .dma_reset()
                .set_bit()
                .fifo_reset()
                .set_bit()
        });

        while self.sdhost.ctrl().read().controller_reset().bit_is_set()
            || self.sdhost.ctrl().read().dma_reset().bit_is_set()
            || self.sdhost.ctrl().read().fifo_reset().bit_is_set()
        {
            // TODO
            // Check if stopwatch has run out, if so timeout
            // Otherwise wait 1 tick (maybe longer? seems pretty short)
        }
    }

    // async fn start_command(&self) -> Result<(), Error> {
    //     // TODO: Check card detect and write protect

    //     self.sdhost
    //         .cmd()
    //         .read()
    //         .self
    //         .sdhost
    //         .ctrl()
    //         .modify(|r, w| unsafe {
    //             w.controller_reset()
    //                 .set_bit()
    //                 .dma_reset()
    //                 .set_bit()
    //                 .fifo_reset()
    //                 .set_bit()
    //         });

    //     while (self.sdhost.ctrl().read().controller_reset().bit_is_set()
    //         || self.sdhost.ctrl().read().dma_reset().bit_is_set()
    //         || self.sdhost.ctrl().read().fifo_reset().bit_is_set())
    //     {
    //         // Check if stopwatch has run out, if so timeout
    //         // Otherwise wait 1 tick (maybe longer? seems pretty short)
    //     }
    // }

    pub async fn init(&mut self, clocks: &Clocks<'_>) -> Result<(), Error> {
        self.host_init();

        self.set_card_clock(clocks, 400).await?;
        self.set_bus_width(1);

        self.card_init(clocks).await
    }

    fn set_bus_width(&mut self, width: usize) -> Result<(), Error> {
        let mask = self.slot.mask();

        let (w4, w8) = match width {
            1 => (false, false),
            4 => (true, false),
            8 => (false, true),
            _ => unreachable!(),
        };

        self.sdhost.ctype().modify(|r, w| unsafe {
            w.card_width4()
                .bits(if w4 {
                    r.card_width4().bits() | mask
                } else {
                    r.card_width4().bits() & !mask
                })
                .card_width8()
                .bits(if w8 {
                    r.card_width8().bits() | mask
                } else {
                    r.card_width8().bits() & !mask
                })
        });

        if w4 || w8 {
            self.d3.set_output_high(w4 || w8);
        }

        Ok(())
    }

    async fn set_card_clock(&self, clocks: &Clocks<'_>, freq_khz: u32) -> Result<(), Error> {
        // Disable clock first
        sdhost_ll::enable_card_clock(false, self.slot.mask());
        self.send_clock_update().await?;

        // Program card clock settings, send them to the CIU
        let (host_div, card_div) = self.get_clk_dividers(clocks, freq_khz);
        sdhost_ll::set_card_clock_divider(self.slot.mask(), card_div);

        self.set_host_clock_divider(host_div);
        self.send_clock_update().await?;

        // Re-enable clocks
        sdhost_ll::enable_card_clock(true, self.slot.mask());
        sdhost_ll::enable_card_clock_low_power(true, self.slot.mask());
        self.send_clock_update().await?;

        // set data timeout
        let data_timeout_ms = 100;
        let data_timeout_cycles: u32 = data_timeout_ms * freq_khz;
        sdhost_ll::set_data_timeout(data_timeout_cycles);
        // always set response timeout to highest value, it's small enough anyway
        sdhost_ll::set_response_timeout(255);

        Ok(())
    }

    fn set_host_clock_divider(&self, host_div: u8) {
        critical_section::with(|_cs| {
            sdhost_ll::set_clock_divider(host_div);
            sdhost_ll::select_clock_source();
            sdhost_ll::init_phase_delay();
        });

        // Wait for the clock to propagate
        unsafe {
            ets_delay_us(10);
        }
    }

    async fn send_clock_update(&self) -> Result<(), Error> {
        loop {
            self.set_clock_update_command();

            // TODO: Timeout after some time
            let res = select(
                SdhostFuture::new(Event::CommandAccepted, self.sdhost),
                SdhostFuture::new(Event::HleError, self.sdhost),
            )
            .await;

            match res {
                Either::First(_) => break Ok(()),
                Either::Second(_) => {}
            }
        }
    }

    async fn init_rca(&mut self) -> Result<(), Error> {
        info!("SdHost::init_rca");

        do_command!(self, {
            self.sdhost.cmdarg().write(|w| unsafe { w.bits(0) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(SD_SEND_RELATIVE_ADDR)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await?;

        self.rca = (self.sdhost.resp0().read().bits() >> 16) as u16;

        Ok(())
    }

    async fn init_csd(&mut self) -> Result<u16, Error> {
        info!("SdHost::init_csd");
        self.sdhost
            .cmdarg()
            .write(|w| unsafe { w.bits((self.rca as u32) << 16) });
        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .index()
                .bits(MMC_SEND_CSD)
                .card_number()
                .bits(self.slot.number())
                .check_response_crc()
                .set_bit()
                .response_expect()
                .set_bit()
                .response_length()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .use_hole()
                .set_bit()
                .start_cmd()
                .set_bit()
        });

        self.wait_for_command().await?;

        let mut resp = [0u8; 128];
        resp[0..4].copy_from_slice(&self.sdhost.resp0().read().bits().to_le_bytes());
        resp[4..8].copy_from_slice(&self.sdhost.resp1().read().bits().to_le_bytes());
        resp[8..12].copy_from_slice(&self.sdhost.resp2().read().bits().to_le_bytes());
        resp[12..16].copy_from_slice(&self.sdhost.resp3().read().bits().to_le_bytes());
        let csd_ver = Self::read_csd_bits(resp, 126, 2);

        let (capacity, block_len) = match csd_ver {
            1 => {
                // v2.0
                info!("csd v2");
                let size = Self::read_csd_bits(resp, 48, 22);
                let cap = ((size + 1) << 10);
                (cap, 0x9)
            }
            0 => {
                // v1.0
                info!("csd v1");
                let size = Self::read_csd_bits(resp, 62, 12);
                let mult = Self::read_csd_bits(resp, 47, 3);
                let cap = ((size + 1) << (mult + 2));
                (cap, 0x9)
            }
            _ => return Err(Error::NotSupported),
        };

        self.num_blocks = capacity;
        info!("capacity {:?}MB", (capacity as u64) * 512 / 1024 / 1024);

        let sector_size = (1 << block_len).min(512);
        info!("sector_size: {:?}", sector_size);

        let speed = Self::read_csd_bits(resp, 96, 8);
        let tr_speed = if speed == 0x5a { 50000000 } else { 25000000 };
        info!("tr_speed: {:?}", tr_speed);

        Ok(sector_size)
    }

    async fn init_select_card(&self) -> Result<(), Error> {
        info!("SdHost::init_select_card");
        do_command!(self, {
            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits((self.rca as u32) << 16) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_SELECT_CARD)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .response_expect()
                    .bit(self.rca != 0)
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await
    }

    async fn init_sd_blocklen(&self, sector_size: u16) -> Result<(), Error> {
        info!("SdHost::init_sd_blocklen");

        do_command!(self, {
            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits(sector_size as u32) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_SET_BLOCKLEN)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await
    }

    async fn init_sd_scr(&self) -> Result<(), Error> {
        info!("SdHost::init_sd_scr");
        self.start_app_cmd().await;

        self.sdhost.cmdarg().reset();

        let mut descriptors = [0u32; 4];
        let mut buffer = [0u32; 2];

        descriptors[0].set_owner(dma::Owner::Dma);
        descriptors[0].set_is_first(true);
        descriptors[0].set_is_last(true);
        descriptors[0].set_is_chained(true);

        descriptors[1].set_size(8);

        // pointer to current data
        descriptors[2] = buffer.as_mut_ptr() as u32;

        // self.sdmmc_host_dma_prepare(descriptors.as_mut_ptr() as u32, 8, 8);

        // self.sdhost.bufaddr().read()
        sdhost_ll::set_block_size(8);
        sdhost_ll::set_data_transfer_len(8);
        // self.sdhost
        //     .fifoth()
        //     .write(|w| unsafe { w.rx_wmark().bits(4) });
        // self.sdhost
        //     .buffifo()
        //     .write(|w| unsafe { w.buffifo().bits(buffer.as_mut_ptr() as u32) });
        // self.sdhost
        //     .cardthrctl()
        //     .write(|w| unsafe { w.cardrdthren().set_bit().cardthreshold().bits(4) });

        let mut data_over = SdhostFuture::new(Event::DataOver, self.sdhost);
        do_command!(self, {
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(SD_APP_SEND_SCR)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .data_expected()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await;
        data_over.await;

        let mut buffer = [0u8; 8];
        self.get_buffifo_bytes(&mut buffer);

        // sdhost_ll::set_block_size(0);
        // self.sdhost.buffifo().reset();

        // sdhost_ll::poll_demand(true);
        // data_received.await;

        info!("{:?}", &buffer);

        Ok(())
    }

    pub async fn read_sector<'w>(
        &self,
        start_block: u32,
        buffer: &'w mut [u8],
    ) -> Result<&'w [u8], Error> {
        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        assert_eq!(buffer.len(), self.sector_size as usize);

        let mut descriptors = [0u32; 4];

        descriptors[0].set_owner(dma::Owner::Dma);
        descriptors[0].set_is_first(true);
        descriptors[0].set_is_last(true);
        descriptors[0].set_is_chained(true);

        descriptors[1].set_size(self.sector_size);

        // pointer to current data
        // let mut data_buf = [255u8; 512];
        let buf_addr = &buffer[0] as *const _ as u32;
        descriptors[2] = buf_addr;

        info!("buf_addr: {:?}", buf_addr);
        assert!(is_valid_ram_address(buf_addr));
        assert!((buf_addr % 4) == 0);

        // assert_eq!(data_buf, [255u8; 512]);

        do_command_meh!(self, Event::DataReceived, {
            sdhost_ll::set_block_size(self.sector_size);
            sdhost_ll::set_data_transfer_len(self.sector_size as u32);
            self.sdmmc_host_dma_prepare(
                descriptors.as_mut_ptr() as u32,
                self.sector_size,
                self.sector_size as u32,
            );

            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits(start_block) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_READ_BLOCK_SINGLE)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .data_expected()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await;

        sdhost_ll::set_desc_addr(0);
        assert_eq!(descriptors[0].get_owner(), dma::Owner::Cpu);
        assert_eq!(descriptors[0].get_ces(), false);

        // assert_ne!(data_buf, [255u8; 512]);
        info!("buf: {:?}", &buffer);

        // buffer.copy_from_slice(&data_buf);

        // self.get_buffifo_bytes(buffer);

        // let buffifo = self.sdhost.cda;
        // info!("{:?}", buffifo);

        self.init_sd_wait_data_ready().await;

        Ok(buffer)
    }

    pub async fn write_sector(&self, start_block: u32, buffer: &[u8]) -> Result<(), Error> {
        assert_eq!(buffer.len(), self.sector_size as usize);

        let mut descriptors = [0u32; 4];

        descriptors[0].set_owner(dma::Owner::Dma);
        descriptors[0].set_is_first(true);
        descriptors[0].set_is_last(true);
        descriptors[0].set_is_chained(true);

        descriptors[1].set_size(self.sector_size);

        // pointer to current data
        let buf_addr = buffer.as_ptr() as u32;
        descriptors[2] = buf_addr;

        assert!(is_valid_ram_address(buf_addr));
        assert!((buf_addr % 4) == 0);

        do_command_meh!(self, Event::DataTransmitted, {
            sdhost_ll::set_block_size(self.sector_size);
            sdhost_ll::set_data_transfer_len(self.sector_size as u32);
            self.sdmmc_host_dma_prepare(
                descriptors.as_mut_ptr() as u32,
                self.sector_size,
                self.sector_size as u32,
            );

            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits(start_block) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_WRITE_BLOCK_SINGLE)
                    .card_number()
                    .bits(self.slot.number())
                    .check_response_crc()
                    .set_bit()
                    .data_expected()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
                    .read_write()
                    .set_bit()
            });
        })
        .await;

        assert_eq!(descriptors[0].get_owner(), dma::Owner::Cpu);
        assert_eq!(descriptors[0].get_ces(), false);

        // self.set_buffifo_bytes(buffer);

        // let buffifo = self.sdhost.cda;
        // info!("{:?}", buffifo);

        self.init_sd_wait_data_ready().await;

        // let mut check_buf = [0u8; 512];
        // self.read_sector(start_block, &mut check_buf).await?;

        // assert_eq!(buffer, check_buf);

        Ok(())
    }

    async fn init_sd_wait_data_ready(&self) -> Result<(), Error> {
        const TIMEOUT_TICKS: u64 = 5 * SystemTimer::TICKS_PER_SECOND;
        let timeout_at = SystemTimer::now() + TIMEOUT_TICKS;

        info!("SdHost::init_sd_wait_data_ready");
        loop {
            do_command!(self, {
                self.sdhost
                    .cmdarg()
                    .write(|w| unsafe { w.bits((self.rca as u32) << 16) });
                self.sdhost.cmd().write(|w| unsafe {
                    w.bits(CMD_SPEC::RESET_VALUE)
                        .index()
                        .bits(MMC_SEND_STATUS)
                        .card_number()
                        .bits(self.slot.number())
                        .check_response_crc()
                        .set_bit()
                        .response_expect()
                        .set_bit()
                        .wait_prvdata_complete()
                        .set_bit()
                        .use_hole()
                        .set_bit()
                        .start_cmd()
                        .set_bit()
                });
            })
            .await;

            let status = self.sdhost.resp0().read().bits();
            const READY_FOR_DATA: u32 = 1 << 8;

            if status & READY_FOR_DATA != 0 {
                return Ok(());
            }

            if SystemTimer::now() > timeout_at {
                return Err(Error::Timeout);
            }
        }
    }

    async fn init_card_hs_mode(&self) -> Result<(), Error> {
        const SD_ACCESS_MODE: u32 = 1;
        self.send_cmd_switch_func(0, SD_ACCESS_MODE, 0).await?;

        const SD_ACCESS_MODE_SDR25: u32 = 1;
        self.send_cmd_switch_func(1, SD_ACCESS_MODE, SD_ACCESS_MODE_SDR25)
            .await?;

        Ok(())
    }

    async fn send_cmd_switch_func(
        &self,
        mode: u32,
        group: u32,
        function: u32,
    ) -> Result<(), Error> {
        let group_shift = (group - 1) << 2;
        // all functions which should not be affected are set to 0xf (no change)
        let other_func_mask = (0x00ffffff & !(0xf << group_shift));
        let func_val = (function << group_shift) | other_func_mask;

        let mut descriptors = [0u32; 4];
        let mut buffer = [0u8; 64];

        descriptors[0].set_owner(dma::Owner::Dma);
        descriptors[0].set_is_first(true);
        descriptors[0].set_is_last(true);
        descriptors[0].set_is_chained(true);

        descriptors[1].set_size(64);

        // pointer to current data
        descriptors[2] = buffer.as_mut_ptr() as u32;

        sdhost_ll::set_block_size(64);
        sdhost_ll::set_data_transfer_len(64);
        // self.sdmmc_host_dma_prepare(descriptors.as_mut_ptr() as u32, 64, 64);

        self.sdhost
            .cmdarg()
            .write(|w| unsafe { w.bits((!!mode << 31) | func_val) });
        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .index()
                .bits(MMC_SWITCH)
                .card_number()
                .bits(self.slot.number())
                .check_response_crc()
                .set_bit()
                .data_expected()
                .set_bit()
                .response_expect()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .use_hole()
                .set_bit()
                .start_cmd()
                .set_bit()
        });

        let mut data_over = SdhostFuture::new(Event::DataOver, self.sdhost);
        self.wait_for_command().await?;
        data_over.await;

        self.get_buffifo_bytes(&mut buffer);

        Ok(())
    }

    async fn init_card_bus_width(&self) -> Result<(), Error> {
        self.start_app_cmd().await?;

        const ARG_BUS_WIDTH_4: u32 = 2;

        self.sdhost
            .cmdarg()
            .write(|w| unsafe { w.bits(ARG_BUS_WIDTH_4) });
        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .index()
                .bits(SD_APP_SET_BUS_WIDTH)
                .card_number()
                .bits(self.slot.number())
                .use_hole()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .response_expect()
                .set_bit()
                .start_cmd()
                .set_bit()
        });

        self.wait_for_command().await
    }

    fn read_csd_bits(resp: [u8; 128], index: u32, len: u32) -> u32 {
        // Check if the index and length are within bounds
        if index + len > 8 * resp.len() as u32 {
            panic!("Index and length are out of bounds");
        }

        // Calculate the starting byte and bit within that byte
        let start_byte = (index / 8) as usize;
        let start_bit = (index % 8);

        // Initialize the result
        let mut result = 0;

        // Iterate over the bytes and bits to construct the result
        for i in 0..len {
            let byte_index = (start_byte + (i / 8) as usize) % resp.len();
            let bit_index = (start_bit + (i % 8)) % 8;
            let bit_value = (((resp[byte_index] >> bit_index) & 1) as u32) << i;
            result |= bit_value;
        }

        result
    }

    async fn init_cid(&self) -> Result<(u32, u32, u32, u32), Error> {
        info!("SdHost::init_cid");
        self.sdhost.cmdarg().reset();

        do_command!(self, {
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_ALL_SEND_CID)
                    .card_number()
                    .bits(self.slot.number())
                    .wait_prvdata_complete()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .response_length()
                    .set_bit()
                    .check_response_crc()
                    .set_bit()
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await?;

        // TODO: Decode CID
        Ok((
            self.sdhost.resp0().read().bits(),
            self.sdhost.resp1().read().bits(),
            self.sdhost.resp2().read().bits(),
            self.sdhost.resp3().read().bits(),
        ))
    }

    fn set_clock_update_command(&self) {
        self.sdhost.cmdarg().reset();
        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .card_number()
                .bits(self.slot.number())
                .update_clock_registers_only()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .use_hole()
                .set_bit()
                .start_cmd()
                .set_bit()
        });
    }

    async fn send_go_idle(&self) -> Result<(), Error> {
        info!("SdHost::send_go_idle");

        self.sdhost.cmdarg().reset();

        do_command!(self, {
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_GO_IDLE_STATE)
                    .send_initialization()
                    .set_bit()
                    .card_number()
                    .bits(self.slot.number())
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await?;

        unsafe {
            ets_delay_us(20 * 1000);
        }

        Ok(())
    }

    async fn init_sd_if_cond(&self) -> Result<(), Error> {
        info!("SdHost::init_sd_if_cond");
        self.send_if_cond().await?; // For now this will error if the card is not SDHC

        Ok(())
    }

    async fn send_if_cond(&self) -> Result<(), Error> {
        let pattern: u32 = 0xaa; // any pattern will do here
        let voltage_supplied = 1 << 8;

        do_command!(self, {
            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits(voltage_supplied | pattern) });

            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(SD_SEND_IF_COND)
                    .card_number()
                    .bits(self.slot.number())
                    .use_hole()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
                    .check_response_crc()
                    .set_bit()
                    .response_expect()
                    .set_bit()
            });
        })
        .await?;

        let response = self.sdhost.resp0().read().response0().bits();

        let voltage_ok = (response & voltage_supplied) != 0;
        let pattern_match = (response & 0xff) == pattern;

        if voltage_ok && pattern_match {
            return Ok(());
        }

        Err(Error::InvalidResponse)
    }

    async fn io_reset(&self) -> Result<(), Error> {
        info!("SdHost::io_reset");
        let sd_io_cccr_ctl = 0x06;
        let sdio_reset = 1 << 3;

        match do_command!(self, {
            self.set_cmd_reg_io_rw_direct(0, sd_io_cccr_ctl, true, sdio_reset);
        })
        .await
        {
            Err(Error::Timeout) => Ok(()),
            other => other,
        }?;

        Ok(())
    }

    fn set_cmd_reg_io_rw_direct(&self, func: u8, reg: u32, is_write: bool, byte: u8) {
        self.sdhost.cmdarg().write(|w| unsafe {
            w.bits(
                ((if is_write { 1 << 31 } else { 0 }) | ((func as u32) & 0b111) << 28)
                    | ((reg & 0x1ffff) << 9)
                    | ((byte as u32) & 0xff),
            )
        });

        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .index()
                .bits(SD_IO_RW_DIRECT)
                .card_number()
                .bits(self.slot.number())
                .use_hole()
                .set_bit()
                .start_cmd()
                .set_bit()
                .response_expect()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .check_response_crc()
                .set_bit()
        });
    }

    async fn init_io(&self) -> Result<(), Error> {
        info!("SdHost::init_io");
        match self.send_io_op_cond(0).await {
            Ok(o) => unimplemented!("SDIO not implemented"),
            Err(e) => return Ok(()), // Fail is expected
        }
    }

    async fn wait_for_command(&self) -> Result<(), Error> {
        match select3(
            SdhostFuture::new(Event::ResponseError, self.sdhost),
            SdhostFuture::new(Event::Timeout, self.sdhost),
            SdhostFuture::new(Event::CommandDone, self.sdhost),
        )
        .await
        {
            Either3::First(_) => Err(Error::InvalidResponse),
            Either3::Second(_) => Err(Error::Timeout),
            Either3::Third(_) => Ok(()),
        }
    }

    async fn send_io_op_cond(&self, ocr: u32) -> Result<u32, Error> {
        do_command!(self, {
            self.set_cmd_reg_io_op_cond(ocr);
        })
        .await?;

        assert!(self.sdhost.status().read().response_index().bits() == SD_IO_SEND_OP_COND);

        Ok(self.sdhost.resp0().read().bits())
    }

    fn set_cmd_reg_io_op_cond(&self, ocr: u32) {
        self.sdhost.cmdarg().write(|w| unsafe { w.bits(ocr) });

        self.sdhost.cmd().write(|w| unsafe {
            w.bits(CMD_SPEC::RESET_VALUE)
                .index()
                .bits(SD_IO_SEND_OP_COND)
                .card_number()
                .bits(self.slot.number())
                .use_hole()
                .set_bit()
                .wait_prvdata_complete()
                .set_bit()
                .response_expect()
                .set_bit()
                .start_cmd()
                .set_bit()
        });
    }

    async fn init_ocr(&mut self) -> Result<u32, Error> {
        info!("SdHost::init_ocr");
        let vol_mask = 0xFF8000;
        let host_ocr = vol_mask | SD_OCR_SDHC_CAP;
        let card_ocr = self.sdmmc_send_cmd_send_op_cond(host_ocr).await?;

        Ok(host_ocr & (card_ocr | !vol_mask))
    }

    async fn sdmmc_send_cmd_send_op_cond(&self, ocr: u32) -> Result<u32, Error> {
        self.set_cclk_always_on(true).await?;

        let mut nretries = 100;
        let mut err_cnt = 3;
        let res = loop {
            match self.set_cmd_send_op_cond(ocr).await {
                Err(e) => {
                    err_cnt -= 1;

                    if err_cnt == 0 {
                        break Err(e);
                    } else if nretries == 0 {
                        info!("SdHost::sdmmc_send_cmd_send_op_cond Timeout");
                        break Err(Error::Timeout);
                    }
                }
                Ok(_) => {
                    let res = self.sdhost.resp0().read().bits();
                    let is_mem_ready = (res & (1 << 31)) != 0;
                    if is_mem_ready {
                        break Ok(res);
                    }
                }
            };

            nretries -= 1;
        };

        self.set_cclk_always_on(false).await;

        res
    }

    async fn set_cclk_always_on(&self, always_on: bool) -> Result<(), Error> {
        if always_on {
            sdhost_ll::enable_card_clock_low_power(!always_on, self.slot.mask())
        }

        self.send_clock_update().await
    }

    async fn start_app_cmd(&self) -> Result<(), Error> {
        do_command!(self, {
            self.sdhost
                .cmdarg()
                .write(|w| unsafe { w.bits((self.rca as u32) << 16) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(MMC_APP_CMD)
                    .card_number()
                    .bits(self.slot.number())
                    .use_hole()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
                    .check_response_crc()
                    .set_bit()
            });
        })
        .await
    }

    async fn set_cmd_send_op_cond(&self, ocr: u32) -> Result<(), Error> {
        self.start_app_cmd().await?;

        do_command!(self, {
            self.sdhost.cmdarg().write(|w| unsafe { w.bits(ocr) });
            self.sdhost.cmd().write(|w| unsafe {
                w.bits(CMD_SPEC::RESET_VALUE)
                    .index()
                    .bits(SD_APP_OP_COND)
                    .card_number()
                    .bits(self.slot.number())
                    .use_hole()
                    .set_bit()
                    .wait_prvdata_complete()
                    .set_bit()
                    .response_expect()
                    .set_bit()
                    .start_cmd()
                    .set_bit()
            });
        })
        .await
    }

    fn get_clk_dividers(&self, clocks: &Clocks, freq_khz: u32) -> (u8, u8) {
        let clk = clocks.sdmmc_clock.to_Hz();

        if freq_khz >= 40000 {
            (4, 0)
        } else if freq_khz == 20000 {
            (8, 0)
        } else if freq_khz == 400 {
            (10, 20)
        } else {
            let host_div = ((clk) / (freq_khz * 1000)) as u8;
            if (host_div > 15) {
                let mut card_div = ((clk / 2) / (2 * freq_khz * 1000)) as u8;

                if (((clk / 2) % (2 * freq_khz * 1000)) > 0) {
                    card_div += 1;
                }

                (2, card_div)
            } else if ((clk % (freq_khz * 1000)) > 0) {
                (host_div + 1, 0)
            } else {
                (host_div, 0)
            }
        }
    }

    fn get_buffifo_bytes(&self, buffer: &mut [u8]) {
        let mut index = 0;

        assert!(buffer.len() > 0);
        for chunk in buffer.chunks_mut(4) {
            let val = self.sdhost.buffifo().read().bits().to_le_bytes();

            let copy_len = val.len().min(chunk.len());
            chunk[..copy_len].copy_from_slice(&val[..copy_len]);
        }
    }

    fn set_buffifo_bytes(&self, buffer: &[u8]) {
        for chunk in buffer.array_chunks::<4>() {
            let value = u32::from_le_bytes(*chunk);
            self.sdhost.buffifo().write(|w| unsafe { w.bits(value) });

            // info!("fr {:}", self.sdhost.tbbcnt().read().tbbcnt().bits());
        }
    }
}

#[interrupt]
fn SDIO_HOST() {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    let sdio_interrupts = sdhost.mintsts().read().sdio_interrupt_msk().bits();
    let interrupts = sdhost.mintsts().read().int_status_msk().bits();

    let mut dma_pending = sdhost.idsts().read().bits();
    sdhost.idsts().write(|w| unsafe { w.bits(dma_pending) });

    sdhost
        .rintsts()
        .write(|w| unsafe { w.sdio_interrupt_raw().bits(sdio_interrupts) });
    sdhost
        .rintsts()
        .write(|w| unsafe { w.int_status_raw().bits(interrupts) });

    sdhost.intmask().modify(|r, w| unsafe {
        w.sdio_int_mask()
            .bits(r.sdio_int_mask().bits() ^ sdio_interrupts)
    });
    sdhost
        .intmask()
        .modify(|r, w| unsafe { w.int_mask().bits(r.int_mask().bits() ^ interrupts) });
    sdhost
        .idinten()
        .modify(|r, w| unsafe { w.bits(r.bits() ^ (dma_pending & 0x1f)) });

    WAKER.wake();

    info!(
        "interrupts (CMD_{:02?}) I {:#010b}_{:#010b} D {:#07b} SD {:#010b}",
        sdhost.cmd().read().index().bits(),
        interrupts >> 8,
        interrupts & 0xff,
        dma_pending,
        sdio_interrupts,
    );
}

#[derive(Debug)]
pub enum Error {
    InvalidArgument,
    NotFound,
    InvalidState,
    Timeout,
    InvalidResponse,
    Unknown,
    InvalidCrc,
    NotSupported,
    General,
}

#[derive(Debug)]
pub(crate) enum Event {
    CommandDone,
    ResponseError,
    Timeout,
    HleError,
    DataOver,
    CommandAccepted, // TODO: This shouldn't be a future, the interrupt won't trigger it
    DataReceived,
    DataTransmitted,
}

pub(crate) struct SdhostFuture<'a> {
    event: Event,
    instance: &'a RegisterBlock,
}

impl<'a> SdhostFuture<'a> {
    pub fn new(event: Event, instance: &'a RegisterBlock) -> Self {
        let mehhh = SDMMC_INTMASK_DTO
            | SDMMC_INTMASK_DCRC
            | SDMMC_INTMASK_HTO
            | SDMMC_INTMASK_SBE
            | SDMMC_INTMASK_EBE;
        match event {
            Event::CommandDone => instance.intmask().modify(|r, w| unsafe {
                w.int_mask()
                    .bits(r.int_mask().bits() | SDMMC_INTMASK_CMD_DONE)
            }),
            Event::ResponseError => instance.intmask().modify(|r, w| unsafe {
                w.int_mask()
                    .bits(r.int_mask().bits() | SDMMC_INTMASK_RESP_ERR | SDMMC_INTMASK_DCRC | mehhh)
            }),
            Event::DataOver => instance.intmask().modify(|r, w| unsafe {
                w.int_mask()
                    .bits(r.int_mask().bits() | SDMMC_INTMASK_DATA_OVER)
            }),
            Event::Timeout => instance.intmask().modify(|r, w| unsafe {
                w.int_mask().bits(
                    r.int_mask().bits() | SDMMC_INTMASK_RTO | SDMMC_INTMASK_DTO | SDMMC_INTMASK_HTO,
                )
            }),
            Event::HleError => instance.intmask().modify(|r, w| unsafe {
                w.int_mask().bits(r.int_mask().bits() | SDMMC_INTMASK_HLE)
            }),
            Event::DataReceived => {
                instance.intmask().modify(|r, w| unsafe {
                    w.int_mask().bits(
                        r.int_mask().bits()
                            | SDMMC_INTMASK_CMD_DONE
                            | SDMMC_INTMASK_DATA_OVER
                            | SDMMC_INTMASK_SBE,
                    )
                });
                instance.idinten().write(|w| unsafe {
                    w.ni()
                        .set_bit()
                        .ri()
                        .set_bit()
                        .fbe()
                        .set_bit()
                        .ai()
                        .set_bit()
                })
            }
            Event::DataTransmitted => {
                instance.intmask().modify(|r, w| unsafe {
                    w.int_mask().bits(
                        r.int_mask().bits()
                            | SDMMC_INTMASK_CMD_DONE
                            | SDMMC_INTMASK_DATA_OVER
                            | SDMMC_INTMASK_SBE,
                    )
                });
                instance
                    .idinten()
                    .write(|w| unsafe { w.ni().set_bit().ti().set_bit() });
            }
            Event::CommandAccepted => {}
        };

        Self { event, instance }
    }

    fn event_bit_is_clear(&self) -> bool {
        match self.event {
            Event::CommandDone => {
                (self.instance.intmask().read().int_mask().bits() & SDMMC_INTMASK_CMD_DONE) == 0
            }
            Event::ResponseError => {
                let error_bits = (SDMMC_INTMASK_RESP_ERR | SDMMC_INTMASK_DCRC);
                (self.instance.intmask().read().int_mask().bits() & error_bits) != error_bits
            }
            Event::Timeout => {
                let error_bits = (SDMMC_INTMASK_RTO | SDMMC_INTMASK_DTO | SDMMC_INTMASK_HTO);
                (self.instance.intmask().read().int_mask().bits() & error_bits) != error_bits
            }
            Event::HleError => {
                (self.instance.intmask().read().int_mask().bits() & SDMMC_INTMASK_HLE) == 0
            }
            Event::DataOver => {
                (self.instance.intmask().read().int_mask().bits() & SDMMC_INTMASK_DATA_OVER) == 0
            }
            Event::CommandAccepted => self.instance.cmd().read().start_cmd().bit_is_clear(),
            Event::DataReceived => {
                self.instance.idinten().read().ri().bit_is_clear()
                    && ((self.instance.intmask().read().int_mask().bits()
                        & (SDMMC_INTMASK_DATA_OVER | SDMMC_INTMASK_CMD_DONE))
                        == 0)
            }
            Event::DataTransmitted => {
                self.instance.idinten().read().ti().bit_is_clear()
                    && ((self.instance.intmask().read().int_mask().bits()
                        & (SDMMC_INTMASK_DATA_OVER | SDMMC_INTMASK_CMD_DONE))
                        == 0)
            }
        }
    }
}

impl<'a> core::future::Future for SdhostFuture<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, ctx: &mut Context<'_>) -> Poll<Self::Output> {
        info!("poll {:?}", self.event);
        WAKER.register(ctx.waker());

        if self.event_bit_is_clear() {
            info!("{:?} ready!", self.event);
            Poll::Ready(())
        } else {
            info!("{:?} pending..", self.event);
            Poll::Pending
        }
    }
}

impl Slot {
    fn number(&self) -> u8 {
        match self {
            Slot::Slot0 => 0,
            Slot::Slot1 => 1,
        }
    }

    fn mask(&self) -> u8 {
        match self {
            Slot::Slot0 => 1,
            Slot::Slot1 => 2,
        }
    }

    fn clk_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_CLK,
            Slot::Slot1 => OutputSignal::SDHOST1_CLK,
        }
    }

    fn cmd_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_CMD,
            Slot::Slot1 => OutputSignal::SDHOST1_CMD,
        }
    }

    fn cmd_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_CMD,
            Slot::Slot1 => InputSignal::SDHOST1_CMD,
        }
    }

    fn d0_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_D0,
            Slot::Slot1 => OutputSignal::SDHOST1_D0,
        }
    }

    fn d0_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_D0,
            Slot::Slot1 => InputSignal::SDHOST1_D0,
        }
    }

    fn d1_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_D1,
            Slot::Slot1 => OutputSignal::SDHOST1_D1,
        }
    }

    fn d1_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_D1,
            Slot::Slot1 => InputSignal::SDHOST1_D1,
        }
    }

    fn d2_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_D2,
            Slot::Slot1 => OutputSignal::SDHOST1_D2,
        }
    }

    fn d2_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_D2,
            Slot::Slot1 => InputSignal::SDHOST1_D2,
        }
    }

    fn d3_output_signal(&self) -> OutputSignal {
        match self {
            Slot::Slot0 => OutputSignal::SDHOST0_D3,
            Slot::Slot1 => OutputSignal::SDHOST1_D3,
        }
    }

    fn d3_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_D3,
            Slot::Slot1 => InputSignal::SDHOST1_D3,
        }
    }

    fn cd_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_CD,
            Slot::Slot1 => InputSignal::SDHOST1_CD,
        }
    }

    fn wp_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_WP,
            Slot::Slot1 => InputSignal::SDHOST1_WP,
        }
    }

    fn int_input_signal(&self) -> InputSignal {
        match self {
            Slot::Slot0 => InputSignal::SDHOST0_INT,
            Slot::Slot1 => InputSignal::SDHOST1_INT,
        }
    }
}

const SDMMC_INTMASK_IO_SLOT1: u32 = 1 << 17;
const SDMMC_INTMASK_IO_SLOT0: u32 = 1 << 16;
const SDMMC_INTMASK_EBE: u16 = 1 << 15;
const SDMMC_INTMASK_ACD: u16 = 1 << 14;
const SDMMC_INTMASK_SBE: u16 = 1 << 13;
const SDMMC_INTMASK_HLE: u16 = 1 << 12;
const SDMMC_INTMASK_FRUN: u16 = 1 << 11;
const SDMMC_INTMASK_HTO: u16 = 1 << 10;
const SDMMC_INTMASK_DTO: u16 = 1 << 9;
const SDMMC_INTMASK_RTO: u16 = 1 << 8;
const SDMMC_INTMASK_DCRC: u16 = 1 << 7;
const SDMMC_INTMASK_RCRC: u16 = 1 << 6;
const SDMMC_INTMASK_RXDR: u16 = 1 << 5;
const SDMMC_INTMASK_TXDR: u16 = 1 << 4;
const SDMMC_INTMASK_DATA_OVER: u16 = 1 << 3;
const SDMMC_INTMASK_CMD_DONE: u16 = 1 << 2;
const SDMMC_INTMASK_RESP_ERR: u16 = 1 << 1;
const SDMMC_INTMASK_CD: u16 = 1 << 0;

// MMC commands
// response type
const MMC_GO_IDLE_STATE: u8 = 0; // R0
const MMC_SEND_OP_COND: u8 = 1; // R3
const MMC_ALL_SEND_CID: u8 = 2; // R2
const MMC_SET_RELATIVE_ADDR: u8 = 3; // R1
const MMC_SWITCH: u8 = 6; // R1B
const MMC_SELECT_CARD: u8 = 7; // R1
const MMC_SEND_EXT_CSD: u8 = 8; // R1
const MMC_SEND_CSD: u8 = 9; // R2
const MMC_SEND_CID: u8 = 10; // R1
const MMC_READ_DAT_UNTIL_STOP: u8 = 11; // R1
const MMC_STOP_TRANSMISSION: u8 = 12; // R1B
const MMC_SEND_STATUS: u8 = 13; // R1
const MMC_SET_BLOCKLEN: u8 = 16; // R1
const MMC_READ_BLOCK_SINGLE: u8 = 17; // R1
const MMC_READ_BLOCK_MULTIPLE: u8 = 18; // R1
const MMC_WRITE_DAT_UNTIL_STOP: u8 = 20; // R1
const MMC_SET_BLOCK_COUNT: u8 = 23; // R1
const MMC_WRITE_BLOCK_SINGLE: u8 = 24; // R1
const MMC_WRITE_BLOCK_MULTIPLE: u8 = 25; // R1
const MMC_ERASE_GROUP_START: u8 = 35; // R1
const MMC_ERASE_GROUP_END: u8 = 36; // R1
const MMC_ERASE: u8 = 38; // R1B
const MMC_APP_CMD: u8 = 55; // R1

// SD commands
// response type
const SD_SEND_RELATIVE_ADDR: u8 = 3; // R6
const SD_SEND_SWITCH_FUNC: u8 = 6; // R1
const SD_SEND_IF_COND: u8 = 8; // R7
const SD_ERASE_GROUP_START: u8 = 32; // R1
const SD_ERASE_GROUP_END: u8 = 33; // R1
const SD_READ_OCR: u8 = 58; // R3
const SD_CRC_ON_OFF: u8 = 59; // R1

// SD application commands
// response type
const SD_APP_SET_BUS_WIDTH: u8 = 6; // R1
const SD_APP_SD_STATUS: u8 = 13; // R2
const SD_APP_OP_COND: u8 = 41; // R3
const SD_APP_SEND_SCR: u8 = 51; // R1

// SD IO commands
const SD_IO_SEND_OP_COND: u8 = 5; // R4
const SD_IO_RW_DIRECT: u8 = 52; // R5
const SD_IO_RW_EXTENDED: u8 = 53; // R5

const OCR_VDD_26_27: u32 = (1 << 14);
const OCR_VDD_27_28: u32 = (1 << 15);
const OCR_VDD_28_29: u32 = (1 << 16);
const OCR_VDD_29_30: u32 = (1 << 17);
const OCR_VDD_30_31: u32 = (1 << 18);
const OCR_VDD_31_32: u32 = (1 << 19);
const OCR_VDD_32_33: u32 = (1 << 20);

const SD_OCR_SDHC_CAP: u32 = (1 << 30);
const SD_MMC_VOLTAGE_SUPPORT: u32 =
    (OCR_VDD_27_28 | OCR_VDD_28_29 | OCR_VDD_29_30 | OCR_VDD_30_31 | OCR_VDD_31_32 | OCR_VDD_32_33);

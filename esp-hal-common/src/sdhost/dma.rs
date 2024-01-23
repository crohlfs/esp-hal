//! # Direct Memory Access Commons
//!
//! ## Overview
//!
//! The `DMA` driver provides an interface to efficiently transfer data between
//! different memory regions within the ESP microcontroller without involving
//! the CPU. The `Direct Memory Access` (DMA) controller is a hardware
//! block responsible for managing these data transfers.
//!
//! The driver is organized into several components and traits, each responsible
//! for handling specific functionalities of the `DMA` controller. Below is an
//! overview of the main components and their functionalities:
//!   * `Tx` and `Rx` traits:
//!      - These traits define the behaviors and functionalities required for
//!        DMA transmit and receive operations.<br> The `Tx` trait includes
//!        functions to start, stop, and check the completion status of an
//!        outbound DMA transfer.<br> On the other hand, the Rx trait provides
//!        similar functionalities for inbound DMA transfers.
//!   * `DmaTransfer` and `DmaTransferRxTx` traits:
//!      - The `DmaTransfer` trait and `DmaTransferRxTx` trait are used for
//!        in-progress DMA transfers.<br> They allow waiting for the transfer to
//!        complete and checking its status. Additionally, the `DmaTransferRxTx`
//!        trait extends the functionalities to support both receive and
//!        transmit operations in a single trait.
//!   * `RegisterAccess` trait:
//!      - This trait defines a set of methods that allow low-level access to
//!        the DMA controller's registers.<br> It provides functions to
//!        initialize DMA channels, configure burst mode, priority, and
//!        peripheral for both input and output data transfers.<br>Additionally,
//!        it supports clearing interrupts, resetting channels, setting
//!        descriptor addresses, and checking for descriptor errors.
//!
//! Notice, that this module is a common version of the DMA driver, `ESP32` and
//! `ESP32-S2` are using older `PDMA` controller, whenever other chips are using
//! newer `GDMA` controller.
//!
//! ## Example
//!
//! ### Initialize and utilize DMA controller in `SPI`
//!
//! ```no_run
//! let dma = Gdma::new(peripherals.DMA);
//! let dma_channel = dma.channel0;
//!
//! // For `ESP32` and `ESP32-S2` chips use `pdma::Dma` instead:
//! // let dma = Dma::new(system.dma);
//! // let dma_channel = dma.spi2channel;
//!
//! let mut descriptors = [0u32; 8 * 3];
//! let mut rx_descriptors = [0u32; 8 * 3];
//!
//! let mut spi = Spi::new(
//!     peripherals.SPI2,
//!     sclk,
//!     mosi,
//!     miso,
//!     cs,
//!     100u32.kHz(),
//!     SpiMode::Mode0,
//!     &clocks,
//! )
//! .with_dma(dma_channel.configure(
//!     false,
//!     &mut descriptors,
//!     &mut rx_descriptors,
//!     DmaPriority::Priority0,
//! ));
//! ```
//!
//! ⚠️ Note: Descriptors should be sized as `((CHUNK_SIZE + 4091) / 4092) * 3`.
//! I.e., to transfer buffers of size `1..=4092`, you need 3 descriptors. The
//! number of descriptors must be a multiple of 3.

use core::{marker::PhantomData, ptr::addr_of, sync::atomic::compiler_fence};

const CHUNK_SIZE: usize = 4092;

// /// Convenience macro to create DMA buffers and descriptors
// ///
// /// ## Usage
// /// ```rust,no_run
// /// // TX and RX buffers are 32000 bytes - passing only one parameter makes
// TX and RX the same size /// let (tx_buffer, mut tx_descriptors, rx_buffer,
// mut rx_descriptors) = dma_buffers!(32000, 32000); /// ```
// #[macro_export]
// macro_rules! dma_buffers {
//     ($tx_size:expr, $rx_size:expr) => {{
//         static mut TX_BUFFER: [u8; $tx_size] = [0u8; $tx_size];
//         static mut RX_BUFFER: [u8; $rx_size] = [0u8; $rx_size];
//         let tx_descriptors = [0u32; (($tx_size + 4091) / 4092) * 3];
//         let rx_descriptors = [0u32; (($rx_size + 4091) / 4092) * 3];
//         unsafe {
//             (
//                 &mut TX_BUFFER,
//                 tx_descriptors,
//                 &mut RX_BUFFER,
//                 rx_descriptors,
//             )
//         }
//     }};

//     ($size:expr) => {{
//         static mut TX_BUFFER: [u8; $size] = [0u8; $size];
//         static mut RX_BUFFER: [u8; $size] = [0u8; $size];
//         let tx_descriptors = [0u32; (($size + 4091) / 4092) * 3];
//         let rx_descriptors = [0u32; (($size + 4091) / 4092) * 3];
//         unsafe {
//             (
//                 &mut TX_BUFFER,
//                 tx_descriptors,
//                 &mut RX_BUFFER,
//                 rx_descriptors,
//             )
//         }
//     }};
// }

/// Convenience macro to create DMA descriptors
///
/// ## Usage
/// ```rust,no_run
/// // Create TX and RX descriptors for transactions up to 32000 bytes - passing only one parameter assumes TX and RX are the same size
/// let (mut tx_descriptors, mut rx_descriptors) = dma_descriptors!(32000, 32000);
/// ```
// #[macro_export]
// macro_rules! dma_descriptors {
//     ($tx_size:expr, $rx_size:expr) => {{
//         let tx_descriptors = [0u32; (($tx_size + 4091) / 4092) * 3];
//         let rx_descriptors = [0u32; (($rx_size + 4091) / 4092) * 3];
//         (tx_descriptors, rx_descriptors)
//     }};

//     ($size:expr) => {{
//         let tx_descriptors = [0u32; (($size + 4091) / 4092) * 3];
//         let rx_descriptors = [0u32; (($size + 4091) / 4092) * 3];
//         (tx_descriptors, rx_descriptors)
//     }};
// }

/// DMA Errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    InvalidAlignment,
    OutOfDescriptors,
    InvalidDescriptorSize,
    /// DescriptorError the DMA rejected the descriptor configuration. This
    /// could be because the source address of the data is not in RAM. Ensure
    /// your source data is in a valid address space, or try using
    /// [`crate::FlashSafeDma`] wrapper.
    DescriptorError,
    Overflow,
    Exhausted,
    BufferTooSmall,
}

#[derive(PartialEq, PartialOrd, Debug)]
pub(crate) enum Owner {
    Cpu = 0,
    Dma = 1,
}

impl From<u32> for Owner {
    fn from(value: u32) -> Self {
        match value {
            0 => Owner::Cpu,
            _ => Owner::Dma,
        }
    }
}

pub(crate) trait DmaLinkedListDes0 {
    fn set_is_chained(&mut self, is_chained: bool);
    fn get_is_chained(&self) -> bool;
    fn set_is_last(&mut self, is_last: bool);
    fn get_is_last(&self) -> bool;
    fn set_owner(&mut self, owner: Owner);
    fn get_owner(&self) -> Owner;
    fn set_is_first(&mut self, is_first: bool);
    fn get_is_first(&self) -> bool;
    fn set_ces(&mut self, ces: bool);
    fn get_ces(&self) -> bool;
    fn set_disable_interrupt(&mut self, disable_interrupt: bool);
    fn get_disable_interrupt(&self) -> bool;
}

pub(crate) trait DmaLinkedListDes1 {
    fn set_size(&mut self, len: u16);
    fn get_size(&self) -> u16;
}

impl DmaLinkedListDes0 for u32 {
    fn set_is_chained(&mut self, is_chained: bool) {
        let mask = 0b1;
        let bit_s = 4;
        *self = (*self & !(mask << bit_s)) | (is_chained as u32) << bit_s;
    }

    fn get_is_chained(&self) -> bool {
        let mask = 0b1;
        let bit_s = 4;
        ((*self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_is_first(&mut self, is_first: bool) {
        let mask = 0b1;
        let bit_s = 3;
        *self = (*self & !(mask << bit_s)) | (is_first as u32) << bit_s;
    }

    fn get_is_first(&self) -> bool {
        let mask = 0b1;
        let bit_s = 3;
        ((*self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_is_last(&mut self, is_last: bool) {
        let mask = 0b1;
        let bit_s = 2;
        *self = (*self & !(mask << bit_s)) | (is_last as u32) << bit_s;
    }

    fn get_is_last(&self) -> bool {
        let mask = 0b1;
        let bit_s = 2;
        ((*self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_ces(&mut self, ces: bool) {
        let mask = 0b1;
        let bit_s = 30;
        *self = (*self & !(mask << bit_s)) | (ces as u32) << bit_s;
    }

    fn get_ces(&self) -> bool {
        let mask = 0b1;
        let bit_s = 30;
        ((*self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_disable_interrupt(&mut self, disable_interrupt: bool) {
        let mask = 0b1;
        let bit_s = 1;
        *self = (*self & !(mask << bit_s)) | (disable_interrupt as u32) << bit_s;
    }

    fn get_disable_interrupt(&self) -> bool {
        let mask = 0b1;
        let bit_s = 1;
        ((*self & (mask << bit_s)) >> bit_s) != 0
    }

    fn set_owner(&mut self, owner: Owner) {
        let mask = 0b1;
        let bit_s = 31;
        *self = (*self & !(mask << bit_s)) | (owner as u32) << bit_s;
    }

    fn get_owner(&self) -> Owner {
        let mask = 0b1;
        let bit_s = 31;
        ((*self & (mask << bit_s)) >> bit_s).into()
    }
}

impl DmaLinkedListDes1 for u32 {
    fn set_size(&mut self, len: u16) {
        let mask = 0b1_1111_1111_1111;
        let bit_s = 0;
        *self = (*self & !(mask << bit_s)) | (len as u32) << bit_s;
    }

    fn get_size(&self) -> u16 {
        let mask = 0b1_1111_1111_1111;
        let bit_s = 0;
        ((*self & (mask << bit_s)) >> bit_s) as u16
    }
}

// /// The functions here are not meant to be used outside the HAL
// pub trait RxPrivate {
//     fn init(&mut self, burst_mode: bool);

//     fn init_channel(&mut self);

//     fn prepare_transfer_without_start(
//         &mut self,
//         circular: bool,
//         data: *mut u8,
//         len: usize,
//     ) -> Result<(), DmaError>;

//     fn start_transfer(&mut self) -> Result<(), DmaError>;

//     fn listen_ch_in_done(&self);

//     fn clear_ch_in_done(&self);

//     fn is_ch_in_done_set(&self) -> bool;

//     fn unlisten_ch_in_done(&self);

//     fn is_listening_ch_in_done(&self) -> bool;

//     fn is_done(&self) -> bool;

//     fn is_listening_eof(&self) -> bool;

//     fn listen_eof(&self);

//     fn unlisten_eof(&self);

//     fn available(&mut self) -> usize;

//     fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError>;

//     fn drain_buffer(&mut self, dst: &mut [u8]) -> Result<usize, DmaError>;

//     /// Descriptor error detected
//     fn has_error(&self) -> bool;

//     /// ERR_DSCR_EMPTY error detected
//     fn has_dscr_empty_error(&self) -> bool;

//     /// ERR_EOF error detected
//     fn has_eof_error(&self) -> bool;

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
// }

// pub trait RxChannel<R>
// where
//     R: RegisterAccess,
// {
//     fn init(&mut self, burst_mode: bool) {
//         R::set_in_burstmode(burst_mode);
//     }

//     fn prepare_transfer_without_start(
//         &mut self,
//         descriptors: &mut [u32],
//         circular: bool,
//         data: *mut u8,
//         len: usize,
//     ) -> Result<(), DmaError> {
//         descriptors.fill(0);

//         compiler_fence(core::sync::atomic::Ordering::SeqCst);

//         let mut processed = 0;
//         let mut descr = 0;
//         loop {
//             let chunk_size = usize::min(CHUNK_SIZE, len - processed);
//             let last = processed + chunk_size >= len;

//             // buffer flags
//             let des0 = &mut descriptors[descr];

//             des0.set_has_ces(false);
//             des0.set_owner(Owner::Dma);

//             let des1 = &mut descriptors[descr + 1];
//             des1.set_size(0); // hardware will fill in the received number of
// bytes

//             // pointer to current data
//             descriptors[descr + 2] = data as u32 + processed as u32;

//             // pointer to next descriptor
//             descriptors[descr + 3] = if last {
//                 if circular {
//                     addr_of!(descriptors[0]) as u32
//                 } else {
//                     0
//                 }
//             } else {
//                 addr_of!(descriptors[descr + 4]) as u32
//             };

//             if last {
//                 break;
//             }

//             processed += chunk_size;
//             descr += 4;
//         }

//         R::clear_in_interrupts();
//         R::reset_in();
//         R::set_in_descriptors(descriptors.as_ptr() as u32);

//         Ok(())
//     }

//     fn start_transfer(&mut self) -> Result<(), DmaError> {
//         R::start_in();

//         if R::has_in_descriptor_error() {
//             Err(DmaError::DescriptorError)
//         } else {
//             Ok(())
//         }
//     }

//     fn is_done(&self) -> bool {
//         R::is_in_done()
//     }

//     fn last_in_dscr_address(&self) -> usize {
//         R::last_in_dscr_address()
//     }

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
// }

// // DMA receive channel
// pub struct ChannelRx<'a, T, R>
// where
//     T: RxChannel<R>,
//     R: RegisterAccess,
// {
//     pub descriptors: &'a mut [u32],
//     pub burst_mode: bool,
//     pub rx_impl: T,
//     pub read_descr_ptr: *const u32,
//     pub available: usize,
//     pub last_seen_handled_descriptor_ptr: *const u32,
//     pub read_buffer_start: *const u8,
//     pub _phantom: PhantomData<R>,
// }

// impl<'a, T, R> crate::dma::Rx for ChannelRx<'a, T, R>
// where
//     T: RxChannel<R>,
//     R: RegisterAccess,
// {
// }

// impl<'a, T, R> crate::dma::RxPrivate for ChannelRx<'a, T, R>
// where
//     T: RxChannel<R>,
//     R: RegisterAccess,
// {
//     fn init(&mut self, burst_mode: bool) {
//         self.rx_impl.init(burst_mode);
//     }

//     fn prepare_transfer_without_start(
//         &mut self,
//         circular: bool,
//         data: *mut u8,
//         len: usize,
//     ) -> Result<(), DmaError> {
//         if self.descriptors.len() < (len + CHUNK_SIZE - 1) / CHUNK_SIZE * 3 {
//             return Err(DmaError::OutOfDescriptors);
//         }

//         if self.burst_mode && (len % 4 != 0 || data as u32 % 4 != 0) {
//             return Err(DmaError::InvalidAlignment);
//         }

//         if circular && len < CHUNK_SIZE * 2 {
//             return Err(DmaError::BufferTooSmall);
//         }

//         self.available = 0;
//         self.read_descr_ptr = self.descriptors.as_ptr();
//         self.last_seen_handled_descriptor_ptr = core::ptr::null();
//         self.read_buffer_start = data;

//         self.rx_impl
//             .prepare_transfer_without_start(self.descriptors, circular, data,
// len)     }

//     fn start_transfer(&mut self) -> Result<(), DmaError> {
//         self.rx_impl.start_transfer()
//     }

//     fn listen_ch_in_done(&self) {
//         R::listen_ch_in_done();
//     }

//     fn clear_ch_in_done(&self) {
//         R::clear_ch_in_done();
//     }

//     fn is_ch_in_done_set(&self) -> bool {
//         R::is_ch_in_done_set()
//     }

//     fn unlisten_ch_in_done(&self) {
//         R::unlisten_ch_in_done();
//     }

//     fn is_listening_ch_in_done(&self) -> bool {
//         R::is_listening_ch_in_done()
//     }

//     fn is_done(&self) -> bool {
//         self.rx_impl.is_done()
//     }

//     fn init_channel(&mut self) {
//         R::init_channel();
//     }

//     fn available(&mut self) -> usize {
//         if self.last_seen_handled_descriptor_ptr.is_null() {
//             self.last_seen_handled_descriptor_ptr =
// self.descriptors.as_mut_ptr();             return 0;
//         }

//         if self.available != 0 {
//             return self.available;
//         }

//         let descr_address = self.last_seen_handled_descriptor_ptr.cast_mut();
//         let mut dw0 = unsafe { descr_address.read_volatile() };

//         if dw0.get_owner() == Owner::Cpu && dw0.get_length() != 0 {
//             let descriptor_buffer =
//                 unsafe { descr_address.offset(1).cast::<*const
// u8>().read_volatile() };             let next_descriptor =
//                 unsafe { descr_address.offset(2).cast::<*const
// u32>().read_volatile() };

//             self.read_buffer_start = descriptor_buffer;
//             self.available = dw0.get_length();

//             dw0.set_owner(Owner::Dma);
//             dw0.set_length(0);
//             dw0.set_suc_eof(false);

//             unsafe {
//                 descr_address.write_volatile(dw0);
//             }

//             self.last_seen_handled_descriptor_ptr = if
// next_descriptor.is_null() {                 self.descriptors.as_ptr()
//             } else {
//                 next_descriptor
//             };
//         }

//         self.available
//     }

//     fn pop(&mut self, data: &mut [u8]) -> Result<usize, DmaError> {
//         let avail = self.available;

//         if avail < data.len() {
//             return Err(DmaError::Exhausted);
//         }

//         unsafe {
//             let dst = data.as_mut_ptr();
//             let src = self.read_buffer_start;
//             let count = self.available;
//             core::ptr::copy_nonoverlapping(src, dst, count);
//         }

//         self.available = 0;
//         Ok(data.len())
//     }

//     fn drain_buffer(&mut self, mut dst: &mut [u8]) -> Result<usize, DmaError>
// {         let mut len = 0;
//         let mut idx = 0;
//         loop {
//             let chunk_len =
// dst.len().min(self.descriptors[idx].get_length());             if chunk_len
// == 0 {                 break;
//             }

//             let buffer_ptr = addr_of!(self.descriptors[idx +
// 1]).cast::<u8>();             let next_dscr = addr_of!(self.descriptors[idx +
// 2]).cast::<u8>();

//             // Copy data to desination
//             let (dst_chunk, dst_remaining) = dst.split_at_mut(chunk_len);
//             dst = dst_remaining;

//             dst_chunk
//                 .copy_from_slice(unsafe {
// core::slice::from_raw_parts(buffer_ptr, chunk_len) });

//             len += chunk_len;

//             if next_dscr.is_null() {
//                 break;
//             }

//             idx += 3;
//         }

//         Ok(len)
//     }

//     fn is_listening_eof(&self) -> bool {
//         R::is_listening_in_eof()
//     }

//     fn listen_eof(&self) {
//         R::listen_in_eof()
//     }

//     fn unlisten_eof(&self) {
//         R::unlisten_in_eof()
//     }

//     fn has_error(&self) -> bool {
//         R::has_in_descriptor_error()
//     }

//     fn has_dscr_empty_error(&self) -> bool {
//         R::has_in_descriptor_error_dscr_empty()
//     }

//     fn has_eof_error(&self) -> bool {
//         R::has_in_descriptor_error_err_eof()
//     }

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
//         T::waker()
//     }
// }

// /// The functions here are not meant to be used outside the HAL
// pub trait TxPrivate {
//     fn init(&mut self, burst_mode: bool);

//     fn init_channel(&mut self);

//     fn prepare_transfer_without_start(
//         &mut self,
//         circular: bool,
//         data: *const u8,
//         len: usize,
//     ) -> Result<(), DmaError>;

//     fn start_transfer(&mut self) -> Result<(), DmaError>;

//     fn clear_ch_out_done(&self);

//     fn is_ch_out_done_set(&self) -> bool;

//     fn listen_ch_out_done(&self);

//     fn unlisten_ch_out_done(&self);

//     fn is_listening_ch_out_done(&self) -> bool;

//     fn is_done(&self) -> bool;

//     fn is_listening_eof(&self) -> bool;

//     fn listen_eof(&self);

//     fn unlisten_eof(&self);

//     fn available(&mut self) -> usize;

//     fn has_error(&self) -> bool;

//     fn push(&mut self, data: &[u8]) -> Result<usize, DmaError>;

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
// }

// pub trait TxChannel<R>
// where
//     R: RegisterAccess,
// {
//     fn init(&mut self, burst_mode: bool) {
//         R::set_out_burstmode(burst_mode);
//     }

//     fn prepare_transfer_without_start(
//         &mut self,
//         descriptors: &mut [u32],
//         circular: bool,
//         data: *const u8,
//         len: usize,
//     ) -> Result<(), DmaError> {
//         descriptors.fill(0);

//         compiler_fence(core::sync::atomic::Ordering::SeqCst);

//         let mut processed = 0;
//         let mut descr = 0;
//         loop {
//             let chunk_size = usize::min(CHUNK_SIZE, len - processed);
//             let last = processed + chunk_size >= len;

//             // buffer flags
//             let dw0 = &mut descriptors[descr];

//             // The `suc_eof` bit doesn't affect the transfer itself, but
// signals when the             // hardware should trigger an interrupt request.
// In circular mode,             // we set the `suc_eof` bit for every buffer we
// send. We use this for             // I2S to track progress of a transfer by
// checking OUTLINK_DSCR_ADDR.             dw0.set_suc_eof(circular || last);
//             dw0.set_owner(Owner::Dma);
//             dw0.set_size(chunk_size as u16); // align to 32 bits?
//             dw0.set_length(chunk_size as u16); // the hardware will transmit
// this many bytes

//             // pointer to current data
//             descriptors[descr + 1] = data as u32 + processed as u32;

//             // pointer to next descriptor
//             descriptors[descr + 2] = if last {
//                 if circular {
//                     addr_of!(descriptors[0]) as u32
//                 } else {
//                     0
//                 }
//             } else {
//                 addr_of!(descriptors[descr + 3]) as u32
//             };

//             if last {
//                 break;
//             }

//             processed += chunk_size;
//             descr += 3;
//         }

//         R::clear_out_interrupts();
//         R::reset_out();
//         R::set_out_descriptors(addr_of!(descriptors[0]) as u32);

//         Ok(())
//     }

//     fn start_transfer(&mut self) -> Result<(), DmaError> {
//         R::start_out();

//         if R::has_out_descriptor_error() {
//             Err(DmaError::DescriptorError)
//         } else {
//             Ok(())
//         }
//     }

//     fn clear_ch_out_done(&self) {
//         R::clear_ch_out_done();
//     }

//     fn is_ch_out_done_set(&self) -> bool {
//         R::is_ch_out_done_set()
//     }

//     fn listen_ch_out_done(&self) {
//         R::listen_ch_out_done();
//     }

//     fn unlisten_ch_out_done(&self) {
//         R::unlisten_ch_out_done();
//     }

//     fn is_listening_ch_out_done(&self) -> bool {
//         R::is_listening_ch_out_done()
//     }

//     fn is_done(&self) -> bool {
//         R::is_out_done()
//     }

//     fn descriptors_handled(&self) -> bool {
//         R::is_out_eof_interrupt_set()
//     }

//     fn reset_descriptors_handled(&self) {
//         R::reset_out_eof_interrupt();
//     }

//     fn last_out_dscr_address(&self) -> usize {
//         R::last_out_dscr_address()
//     }

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker;
// }

// /// DMA transmit channel
// pub struct ChannelTx<'a, T, R>
// where
//     T: TxChannel<R>,
//     R: RegisterAccess,
// {
//     pub descriptors: &'a mut [u32],
//     #[allow(unused)]
//     pub burst_mode: bool,
//     pub tx_impl: T,
//     pub write_offset: usize,
//     pub write_descr_ptr: *const u32,
//     pub available: usize,
//     pub last_seen_handled_descriptor_ptr: *const u32,
//     pub buffer_start: *const u8,
//     pub buffer_len: usize,
//     pub _phantom: PhantomData<R>,
// }

// impl<'a, T, R> crate::dma::Tx for ChannelTx<'a, T, R>
// where
//     T: crate::dma::TxChannel<R>,
//     R: RegisterAccess,
// {
// }

// impl<'a, T, R> TxPrivate for ChannelTx<'a, T, R>
// where
//     T: crate::dma::TxChannel<R>,
//     R: RegisterAccess,
// {
//     fn init(&mut self, burst_mode: bool) {
//         self.tx_impl.init(burst_mode);
//     }

//     fn init_channel(&mut self) {
//         R::init_channel();
//     }

//     fn prepare_transfer_without_start(
//         &mut self,
//         circular: bool,
//         data: *const u8,
//         len: usize,
//     ) -> Result<(), DmaError> {
//         if self.descriptors.len() < (len + CHUNK_SIZE - 1) / CHUNK_SIZE * 3 {
//             return Err(DmaError::OutOfDescriptors);
//         }

//         if circular && len < CHUNK_SIZE * 2 {
//             return Err(DmaError::BufferTooSmall);
//         }

//         self.write_offset = 0;
//         self.available = 0;
//         self.write_descr_ptr = self.descriptors.as_ptr();
//         self.last_seen_handled_descriptor_ptr = self.descriptors.as_ptr();
//         self.buffer_start = data;
//         self.buffer_len = len;

//         self.tx_impl
//             .prepare_transfer_without_start(self.descriptors, circular, data,
// len)     }

//     fn start_transfer(&mut self) -> Result<(), DmaError> {
//         self.tx_impl.start_transfer()
//     }

//     fn clear_ch_out_done(&self) {
//         self.tx_impl.clear_ch_out_done();
//     }

//     fn is_ch_out_done_set(&self) -> bool {
//         self.tx_impl.is_ch_out_done_set()
//     }

//     fn listen_ch_out_done(&self) {
//         self.tx_impl.listen_ch_out_done();
//     }

//     fn unlisten_ch_out_done(&self) {
//         self.tx_impl.unlisten_ch_out_done();
//     }

//     fn is_listening_ch_out_done(&self) -> bool {
//         self.tx_impl.is_listening_ch_out_done()
//     }

//     fn is_done(&self) -> bool {
//         self.tx_impl.is_done()
//     }

//     fn available(&mut self) -> usize {
//         if self.tx_impl.descriptors_handled() {
//             self.tx_impl.reset_descriptors_handled();
//             let descr_address = self.tx_impl.last_out_dscr_address() as
// *const u32;

//             if descr_address >= self.last_seen_handled_descriptor_ptr {
//                 let mut ptr = self.last_seen_handled_descriptor_ptr;

//                 unsafe {
//                     while ptr < descr_address {
//                         let dw0 = ptr.read_volatile();
//                         self.available += dw0.get_length();
//                         ptr = ptr.offset(3);
//                     }
//                 }
//             } else {
//                 let mut ptr = self.last_seen_handled_descriptor_ptr;

//                 unsafe {
//                     while ptr.offset(2).read_volatile() != 0 {
//                         let dw0 = ptr.read_volatile();
//                         self.available += dw0.get_length();
//                         ptr = ptr.offset(3);
//                     }
//                 }
//             }

//             if self.available >= self.buffer_len {
//                 unsafe {
//                     let segment_len =
// self.write_descr_ptr.read_volatile().get_length();
// self.available -= segment_len;                     self.write_offset =
// (self.write_offset + segment_len) % self.buffer_len;                     let
// next_descriptor = self                         .write_descr_ptr
//                         .offset(2)
//                         .cast::<*const u32>()
//                         .read_volatile();

//                     self.write_descr_ptr = if next_descriptor.is_null() {
//                         self.descriptors.as_ptr()
//                     } else {
//                         next_descriptor
//                     }
//                 }
//             }

//             self.last_seen_handled_descriptor_ptr = descr_address;
//         }

//         self.available
//     }

//     fn push(&mut self, data: &[u8]) -> Result<usize, DmaError> {
//         let avail = self.available();

//         if avail < data.len() {
//             return Err(DmaError::Overflow);
//         }

//         unsafe {
//             let src = data.as_ptr();
//             let dst = self.buffer_start.add(self.write_offset).cast_mut();
//             let count = usize::min(data.len(), self.buffer_len -
// self.write_offset);             core::ptr::copy_nonoverlapping(src, dst,
// count);         }

//         if self.write_offset + data.len() >= self.buffer_len {
//             let remainder = (self.write_offset + data.len()) %
// self.buffer_len;             let dst = self.buffer_start.cast_mut();
//             unsafe {
//                 let src = data.as_ptr().add(data.len() - remainder);
//                 core::ptr::copy_nonoverlapping(src, dst, remainder);
//             }
//         }

//         let mut forward = data.len();
//         loop {
//             unsafe {
//                 let next_descriptor = self
//                     .write_descr_ptr
//                     .offset(2)
//                     .cast::<*const u32>()
//                     .read_volatile();
//                 let segment_len =
// self.write_descr_ptr.read_volatile().get_length();
// self.write_descr_ptr = if next_descriptor.is_null() {
// self.descriptors.as_ptr()                 } else {
//                     next_descriptor
//                 };

//                 if forward <= segment_len {
//                     break;
//                 }

//                 forward -= segment_len;
//             }
//         }

//         self.write_offset = (self.write_offset + data.len()) %
// self.buffer_len;         self.available -= data.len();

//         Ok(data.len())
//     }

//     fn is_listening_eof(&self) -> bool {
//         R::is_listening_out_eof()
//     }

//     fn listen_eof(&self) {
//         R::listen_out_eof()
//     }

//     fn unlisten_eof(&self) {
//         R::unlisten_out_eof()
//     }

//     fn has_error(&self) -> bool {
//         R::has_out_descriptor_error()
//     }

//     #[cfg(feature = "async")]
//     fn waker() -> &'static embassy_sync::waitqueue::AtomicWaker {
//         T::waker()
//     }
// }

pub trait RegisterAccess {
    fn init_channel();
    fn set_out_burstmode(burst_mode: bool);
    fn clear_out_interrupts();
    fn reset_out();
    fn set_out_descriptors(address: u32);
    fn has_out_descriptor_error() -> bool;
    fn set_out_peripheral(peripheral: u8);
    fn start_out();
    fn clear_ch_out_done();
    fn is_ch_out_done_set() -> bool;
    fn listen_ch_out_done();
    fn unlisten_ch_out_done();
    fn is_listening_ch_out_done() -> bool;
    fn is_out_done() -> bool;
    fn is_out_eof_interrupt_set() -> bool;
    fn reset_out_eof_interrupt();
    fn last_out_dscr_address() -> usize;

    fn set_in_burstmode(burst_mode: bool);
    fn clear_in_interrupts();
    fn reset_in();
    fn set_in_descriptors(address: u32);
    fn has_in_descriptor_error() -> bool;
    fn has_in_descriptor_error_dscr_empty() -> bool;
    fn has_in_descriptor_error_err_eof() -> bool;
    fn set_in_peripheral(peripheral: u8);
    fn start_in();
    fn is_in_done() -> bool;
    fn last_in_dscr_address() -> usize;

    fn is_listening_in_eof() -> bool;
    fn is_listening_out_eof() -> bool;

    fn listen_in_eof();
    fn listen_out_eof();
    fn unlisten_in_eof();
    fn unlisten_out_eof();

    fn listen_ch_in_done();
    fn clear_ch_in_done();
    fn is_ch_in_done_set() -> bool;
    fn unlisten_ch_in_done();
    fn is_listening_ch_in_done() -> bool;
}

pub trait ChannelTypes {
    type Tx<'a>: crate::dma::Tx;
    type Rx<'a>: crate::dma::Rx;
}

/// DMA Channel
pub struct Channel<'d, C>
where
    C: ChannelTypes,
{
    pub(crate) tx: C::Tx<'d>,
    pub(crate) rx: C::Rx<'d>,
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransfer<B, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> Result<(B, T), (DmaError, B, T)>;
    /// Check if the transfer is finished.
    fn is_done(&self) -> bool;
}

/// Trait to be implemented for an in progress dma transfer.
#[allow(drop_bounds)]
pub trait DmaTransferRxTx<BR, BT, T>: Drop {
    /// Wait for the transfer to finish.
    fn wait(self) -> Result<(BR, BT, T), (DmaError, BR, BT, T)>;
    /// Check if the transfer is finished.
    fn is_done(&self) -> bool;
}

// #[cfg(feature = "async")]
pub(crate) mod asynch {
    use core::task::Poll;

    use super::*;
    use crate::macros::interrupt;

    pub struct DmaTxFuture<'a, TX> {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    impl<'a, TX> DmaTxFuture<'a, TX>
    where
        TX: crate::dma::Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            Self { tx, _a: () }
        }
    }

    impl<'a, TX> core::future::Future for DmaTxFuture<'a, TX>
    where
        TX: crate::dma::Tx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_eof() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    pub struct DmaRxFuture<'a, RX> {
        pub(crate) rx: &'a mut RX,
        _a: (),
    }

    impl<'a, RX> DmaRxFuture<'a, RX>
    where
        RX: crate::dma::Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            Self { rx, _a: () }
        }
    }

    impl<'a, RX> core::future::Future for DmaRxFuture<'a, RX>
    where
        RX: crate::dma::Rx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_eof() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaTxDoneChFuture<'a, TX> {
        pub(crate) tx: &'a mut TX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> DmaTxDoneChFuture<'a, TX>
    where
        TX: crate::dma::Tx,
    {
        pub fn new(tx: &'a mut TX) -> Self {
            tx.listen_ch_out_done();
            Self { tx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, TX> core::future::Future for DmaTxDoneChFuture<'a, TX>
    where
        TX: crate::dma::Tx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            TX::waker().register(cx.waker());
            if self.tx.is_listening_ch_out_done() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    pub struct DmaRxDoneChFuture<'a, RX> {
        pub(crate) rx: &'a mut RX,
        _a: (),
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> DmaRxDoneChFuture<'a, RX>
    where
        RX: crate::dma::Rx,
    {
        pub fn new(rx: &'a mut RX) -> Self {
            rx.listen_ch_in_done();
            Self { rx, _a: () }
        }
    }

    #[cfg(any(i2s0, i2s1))]
    impl<'a, RX> core::future::Future for DmaRxDoneChFuture<'a, RX>
    where
        RX: crate::dma::Rx,
    {
        type Output = (); // TODO handle DMA errors

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            RX::waker().register(cx.waker());
            if self.rx.is_listening_ch_in_done() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    // #[cfg(esp32c2)]
    // mod interrupt {
    //     use super::*;

    //     #[interrupt]
    //     fn DMA_CH0() {
    //         use crate::dma::gdma::{
    //             Channel0 as Channel,
    //             Channel0RxImpl as ChannelRxImpl,
    //             Channel0TxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }
    // }

    // #[cfg(esp32c3)]
    // mod interrupt {
    //     use super::*;

    //     #[interrupt]
    //     fn DMA_CH0() {
    //         use crate::dma::gdma::{
    //             Channel0 as Channel,
    //             Channel0RxImpl as ChannelRxImpl,
    //             Channel0TxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_CH1() {
    //         use crate::dma::gdma::{
    //             Channel1 as Channel,
    //             Channel1RxImpl as ChannelRxImpl,
    //             Channel1TxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_CH2() {
    //         use crate::dma::gdma::{
    //             Channel2 as Channel,
    //             Channel2RxImpl as ChannelRxImpl,
    //             Channel2TxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }
    // }

    // #[cfg(any(esp32c6, esp32h2))]
    // mod interrupt {
    //     use super::*;

    //     #[interrupt]
    //     fn DMA_IN_CH0() {
    //         use crate::dma::gdma::{Channel0 as Channel, Channel0RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH0() {
    //         use crate::dma::gdma::{Channel0 as Channel, Channel0TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_IN_CH1() {
    //         use crate::dma::gdma::{Channel1 as Channel, Channel1RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH1() {
    //         use crate::dma::gdma::{Channel1 as Channel, Channel1TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_IN_CH2() {
    //         use crate::dma::gdma::{Channel2 as Channel, Channel2RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH2() {
    //         use crate::dma::gdma::{Channel2 as Channel, Channel2TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }
    // }

    // #[cfg(esp32s3)]
    // mod interrupt {
    //     use super::*;

    //     #[interrupt]
    //     fn DMA_IN_CH0() {
    //         use crate::dma::gdma::{Channel0 as Channel, Channel0RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH0() {
    //         use crate::dma::gdma::{Channel0 as Channel, Channel0TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_IN_CH1() {
    //         use crate::dma::gdma::{Channel1 as Channel, Channel1RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH1() {
    //         use crate::dma::gdma::{Channel1 as Channel, Channel1TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_IN_CH3() {
    //         use crate::dma::gdma::{Channel3 as Channel, Channel3RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH3() {
    //         use crate::dma::gdma::{Channel3 as Channel, Channel3TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_IN_CH4() {
    //         use crate::dma::gdma::{Channel4 as Channel, Channel4RxImpl as
    // ChannelRxImpl};

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn DMA_OUT_CH4() {
    //         use crate::dma::gdma::{Channel4 as Channel, Channel4TxImpl as
    // ChannelTxImpl};

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }
    //     }
    // }

    // #[cfg(any(esp32, esp32s2))]
    // mod interrupt {
    //     use super::*;

    //     #[interrupt]
    //     fn SPI2_DMA() {
    //         use crate::dma::pdma::{
    //             Spi2DmaChannel as Channel,
    //             Spi2DmaChannelRxImpl as ChannelRxImpl,
    //             Spi2DmaChannelTxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn SPI3_DMA() {
    //         use crate::dma::pdma::{
    //             Spi3DmaChannel as Channel,
    //             Spi3DmaChannelRxImpl as ChannelRxImpl,
    //             Spi3DmaChannelTxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[interrupt]
    //     fn I2S0() {
    //         use crate::dma::pdma::{
    //             I2s0DmaChannel as Channel,
    //             I2s0DmaChannelRxImpl as ChannelRxImpl,
    //             I2s0DmaChannelTxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() && Channel::is_listening_out_eof() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }

    //     #[cfg(esp32)]
    //     #[interrupt]
    //     fn I2S1() {
    //         use crate::dma::pdma::{
    //             I2s1DmaChannel as Channel,
    //             I2s1DmaChannelRxImpl as ChannelRxImpl,
    //             I2s1DmaChannelTxImpl as ChannelTxImpl,
    //         };

    //         if Channel::is_in_done() && Channel::is_listening_in_eof() {
    //             Channel::clear_in_interrupts();
    //             Channel::unlisten_in_eof();
    //             ChannelRxImpl::waker().wake()
    //         }

    //         if Channel::is_out_done() && Channel::is_listening_out_eof() {
    //             Channel::clear_out_interrupts();
    //             Channel::unlisten_out_eof();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_out_done_set() {
    //             Channel::clear_ch_out_done();
    //             Channel::unlisten_ch_out_done();
    //             ChannelTxImpl::waker().wake()
    //         }

    //         if Channel::is_ch_in_done_set() {
    //             Channel::clear_ch_in_done();
    //             Channel::unlisten_ch_in_done();
    //             ChannelRxImpl::waker().wake()
    //         }
    //     }
    // }
}
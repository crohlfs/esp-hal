pub(crate) fn enable_card_clock(en: bool, slot_mask: u8) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost.clkena().modify(|r, w| unsafe {
        w.cclk_enable().bits(if en {
            r.cclk_enable().bits() | slot_mask
        } else {
            r.cclk_enable().bits() & !slot_mask
        })
    });
}

pub(crate) fn enable_card_clock_low_power(en: bool, slot_mask: u8) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost.clkena().modify(|r, w| unsafe {
        w.lp_enable().bits(if en {
            r.lp_enable().bits() | slot_mask
        } else {
            r.lp_enable().bits() & !slot_mask
        })
    });
}

pub(crate) fn set_data_timeout(timeout: u32) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .tmout()
        .write(|w| unsafe { w.data_timeout().bits(timeout.min(0xffffff)) })
}

pub(crate) fn set_response_timeout(timeout: u8) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .tmout()
        .write(|w| unsafe { w.response_timeout().bits(timeout) })
}

pub(crate) fn set_card_clock_divider(slot_mask: u8, div: u8) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    match slot_mask {
        1 => {
            // Set card 0 to clock divider 0 and then set clock divider 0 value
            sdhost
                .clksrc()
                .modify(|r, w| unsafe { w.clksrc().bits(r.clksrc().bits() & 0b1100) });
            sdhost
                .clkdiv()
                .write(|w| unsafe { w.clk_divider0().bits(div) });
        }
        2 => {
            // Set card 1 to clock divider 1 and then set clock divider 1 value
            sdhost
                .clksrc()
                .modify(|r, w| unsafe { w.clksrc().bits((r.clksrc().bits() & 0b0011) | 0b0100) });
            sdhost
                .clkdiv()
                .write(|w| unsafe { w.clk_divider1().bits(div) });
        }
        _ => unreachable!(),
    }
}

pub(crate) fn set_clock_divider(div: u8) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    let l = div - 1;
    let h = div / 2 - 1;
    sdhost.clk_edge_sel().write(|w| unsafe {
        w.ccllkin_edge_h()
            .bits(h)
            .ccllkin_edge_l()
            .bits(l)
            .ccllkin_edge_n()
            .bits(l)
    });
}

pub(crate) fn select_clock_source() {}

pub(crate) fn init_phase_delay() {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    // 180 degree phase on input and output clocks
    sdhost.clk_edge_sel().write(|w| unsafe {
        w.cclkin_edge_drv_sel()
            .bits(4)
            .cclkin_edge_sam_sel()
            .bits(4)
            .cclkin_edge_slf_sel()
            .bits(0)
    });
}

pub(crate) fn set_data_transfer_len(len: u32) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .bytcnt()
        .write(|w| unsafe { w.byte_count().bits(len) })
}

pub(crate) fn set_block_size(size: u16) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .blksiz()
        .write(|w| unsafe { w.block_size().bits(size) })
}

pub(crate) fn set_desc_addr(addr: u32) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost.dbaddr().write(|w| unsafe { w.dbaddr().bits(addr) })
}

pub(crate) fn enable_dma(en: bool) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .bmod()
        .write(|w| unsafe { w.de().bit(en).fb().bit(en) })
}

pub(crate) fn poll_demand(en: bool) {
    let sdhost = unsafe { &*crate::peripherals::SDHOST::PTR };

    sdhost
        .pldmnd()
        .write(|w| unsafe { w.pd().bits(if en { 1 } else { 0 }) })
}

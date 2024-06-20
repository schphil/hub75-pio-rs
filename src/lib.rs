//! # hub75-pio
//!
//! An **experimental** HUB75 driver for [RP2040](https://www.raspberrypi.com/products/rp2040/).
//! Utilizes Programmable I/O (PIO) units in combination with DMA to achieve high refresh rates,
//! true color depth and zero CPU overhead without sacrificing quality.
//!
//! ## Features
//!
//! - Supports LED matrices up to 64x32 pixels with 1:16 scanline
//! - High refresh rate (approx. 2100 Hz with 24 bit color depth on a 64x32
//!   display)
//! - Does not utilize CPU for clocking out data to the display – all the work is
//!   done by the PIOs and the DMA controller
//! - Uses binary color modulation
//! - Double buffered
//! - Implements [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) traits
//!
//! ## Requirements
//!
//! The current implementation assumes that the following groups of data outputs
//! are assigned to consecutive pins on the RP2040:
//!
//! - R1, G1, B1, R2, G2, B2
//! - ADDRA, ADDRB, ADDRC, ADDRD

#![no_std]
#![feature(generic_const_exprs)]
#![feature(const_for)]

// TODO: Implement the drop trait to release DMA & PIO?
// TODO: organize these
use core::convert::TryInto;
use embassy_rp::dma::Channel;
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::peripherals;
use embassy_rp::pio::{
    Config, Direction, FifoJoin, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::bind_interrupts;
use embedded_graphics::prelude::*;

pub mod lut;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<peripherals::PIO0>;
});

/// Framebuffer size in bytes
#[doc(hidden)]
pub const fn fb_bytes(w: usize, h: usize, b: usize) -> usize {
    w * h / 2 * b
}

/// Computes an array with number of clock ticks to wait for every n-th color bit
const fn delays<const B: usize>() -> [u32; B] {
    let mut arr = [0; B];
    let mut i = 0;
    while i < arr.len() {
        arr[i] = (1 << i) - 1;
        i += 1;
    }
    arr
}

/// Backing storage for the framebuffers
///
/// ## Memory layout
///
/// The pixel buffer is organized in a way way so that transmiting the content of it requires no
/// manipulation on the PIO.
///
/// PIO reads the pixel buffer one byte at a time from low to high, then shifts out the pixels from
/// left to right.
///
/// ### Pixel tuple
///
/// At the lowest level, every byte in the buffer contains a so called pixel tuple. Within a tuple
/// you will find the nth bit of the R/G/B-componts of two colors. The reason for packing two
/// colors together is the scan rate of the display. The display this library is targetting is has
/// a 1:16 scan rate, which means that when you are addressing a pixel at row 0, you are at the
/// same time addressing a pixel in the same column at row 16.
///
/// The tuple has the following structure:
///
/// ```
/// XXBGRBGR
/// --222111
/// ```
///
/// Currently we are wasting two bits per tuple (byte) – marked as X above.
///
/// ### Buffer structure
///
/// The diagram below attempts to visualize the structure of the buffer for a 64x32 display
/// configured for 24 bit color depth. X, Y are the pixel coordinates on the screen and N stands
/// for the big-endian position of the bit in the color to be displayed.
///
/// ```
///   N           8                                               1             0
///   X   |      63|   |       3        2|       1|       0|      63|   |       0|
/// Y   0 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
/// Y   1 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
///       .................................................|.....................|
///       .................................................|.....................|
///       .................................................|.....................|
/// Y  15 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
/// ```
pub struct DisplayMemory<const W: usize, const H: usize, const B: usize>
where
    [(); fb_bytes(W, H, B)]: Sized,
{
    fbptr: [u32; 1],
    fb0: [u8; fb_bytes(W, H, B)],
    fb1: [u8; fb_bytes(W, H, B)],
    delays: [u32; B],
    delaysptr: [u32; 1],
}

impl<const W: usize, const H: usize, const B: usize> DisplayMemory<W, H, B>
where
    [(); fb_bytes(W, H, B)]: Sized,
{
    pub const fn new() -> Self {
        let fb0 = [0; fb_bytes(W, H, B)];
        let fb1 = [0; fb_bytes(W, H, B)];
        let fbptr: [u32; 1] = [0];
        let delays = delays();
        let delaysptr: [u32; 1] = [0];
        DisplayMemory {
            fbptr,
            fb0,
            fb1,
            delays,
            delaysptr,
        }
    }
}

/// The HUB75 display driver
pub struct Display<'a, CH1, const W: usize, const H: usize, const B: usize, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: Channel,
    C: RgbColor,
{
    mem: &'static mut DisplayMemory<W, H, B>,
    fb_loop_ch: CH1,
    benchmark: bool,
    brightness: u8,
    lut: &'a dyn lut::Lut<B, C>,
}

fn setup_pio_task_data<'a, const W: usize>(
    pio: &mut embassy_rp::pio::Common<'a, peripherals::PIO0>,
    sm: &mut StateMachine<'a, peripherals::PIO0, 0>,
    r1: impl PioPin,
    g1: impl PioPin,
    b1: impl PioPin,
    r2: impl PioPin,
    g2: impl PioPin,
    b2: impl PioPin,
    clk: impl PioPin,
) {
    // Data program
    let prg = pio_proc::pio_asm!(
        ".side_set 1",
        "out isr, 32    side 0b0",
        ".wrap_target",
        "mov x isr      side 0b0",
        // Wait for the row program to set the ADDR pins
        "pixel:",
        "out pins, 8    side 0b0",
        "jmp x-- pixel  side 0b1", // clock out the pixel
        "irq 4          side 0b0", // tell the row program to set the next row
        "wait 1 irq 5   side 0b0",
        ".wrap",
    );

    let r1 = pio.make_pio_pin(r1);
    let g1 = pio.make_pio_pin(g1);
    let b1 = pio.make_pio_pin(b1);
    let r2 = pio.make_pio_pin(r2);
    let g2 = pio.make_pio_pin(g2);
    let b2 = pio.make_pio_pin(b2);
    let clk = pio.make_pio_pin(clk);

    sm.set_pin_dirs(Direction::Out, &[&r1, &g1, &b1, &r2, &g2, &b2, &clk]);

    let mut cfg = Config::default();
    cfg.use_program(&pio.load_program(&prg.program), &[&clk]);
    let divsor = fixed::FixedU32::from_bits(2 << 8);
    cfg.clock_divider = divsor.into();
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };
    cfg.set_out_pins(&[&r1, &g1, &b1, &r2, &g2, &b2]);
    cfg.fifo_join = FifoJoin::TxOnly;

    sm.set_config(&cfg);

    sm.tx().push(W as u32 - 1);
}

fn setup_pio_task_row<'a, const H: usize, const B: usize>(
    pio: &mut embassy_rp::pio::Common<'a, peripherals::PIO0>,
    sm: &mut StateMachine<'a, peripherals::PIO0, 1>,
    addra: impl PioPin,
    addrb: impl PioPin,
    addrc: impl PioPin,
    addrd: impl PioPin,
    lat: impl PioPin,
) {
    // Row program
    let prg = pio_proc::pio_asm!(
        ".side_set 1",
        "pull           side 0b0", // Pull the height / 2 into OSR
        "out isr, 32    side 0b0", // and move it to OSR
        "pull           side 0b0", // Pull the color depth - 1 into OSR
        ".wrap_target",
        "mov x, isr     side 0b0",
        "addr:",
        "mov pins, ~x   side 0b0", // Set the row address
        "mov y, osr     side 0b0",
        "row:",
        "wait 1 irq 4   side 0b0", // Wait until the data is clocked in
        "nop            side 0b1",
        "irq 6          side 0b1", // Display the latched data
        "irq 5          side 0b0", // Clock in next row
        "wait 1 irq 7   side 0b0", // Wait for the OE cycle to complete
        "jmp y-- row    side 0b0",
        "jmp x-- addr   side 0b0",
        ".wrap",
    );

    let addra = pio.make_pio_pin(addra);
    let addrb = pio.make_pio_pin(addrb);
    let addrc = pio.make_pio_pin(addrc);
    let addrd = pio.make_pio_pin(addrd);
    let lat = pio.make_pio_pin(lat);

    sm.set_pin_dirs(Direction::Out, &[&addra, &addrb, &addrc, &addrd, &lat]);

    let mut cfg = Config::default();
    cfg.use_program(&pio.load_program(&prg.program), &[&lat]);
    let divsor = fixed::FixedU32::from_bits(1 << 8 | 1);
    cfg.clock_divider = divsor.into();
    cfg.set_out_pins(&[&addra, &addrb, &addrc, &addrd]);

    sm.set_config(&cfg);

    sm.tx().push(H as u32 / 2 - 1);
    // Configure the color depth
    sm.tx().push(B as u32 - 1);
}

fn setup_pio_task_oe<'a>(
    pio: &mut embassy_rp::pio::Common<'a, peripherals::PIO0>,
    sm: &mut StateMachine<'a, peripherals::PIO0, 2>,
    oe: impl PioPin,
) {
    // Control the delay using DMA - buffer with 8 bytes specifying the length of the delays
    // Delay program (controls OE)
    let prg = pio_proc::pio_asm!(
        ".side_set 1"
        ".wrap_target",
        "out x, 32      side 0b1",
        "wait 1 irq 6   side 0b1",
        "delay:",
        "jmp x-- delay  side 0b0",
        "irq 7          side 0b1",
        ".wrap",
    );

    let oe = pio.make_pio_pin(oe);

    sm.set_pin_dirs(Direction::Out, &[&oe]);

    let mut cfg = Config::default();
    cfg.use_program(&pio.load_program(&prg.program), &[&oe]);
    let divsor = fixed::FixedU32::from_bits(1 << 8 | 1);
    cfg.clock_divider = divsor.into();
    cfg.set_out_pins(&[&oe]);
    cfg.fifo_join = FifoJoin::TxOnly;

    sm.set_config(&cfg);
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: Channel,
    C: RgbColor,
{
    /// Creates a new display
    ///
    /// Algo:
    /// 1. Enable output
    /// 2. Clock out the row:
    ///  1. Set rgb pins
    ///  2. Cycle the clock once
    ///  3. Go back to 2 and repeat 63 more times
    /// 4. Disable output
    /// 5. Cycle the latch
    /// 6. Select the row
    ///
    /// # Arguments
    ///
    /// * `pins`: Pins to use for the communication with the display
    /// * `pio_sms`: PIO state machines to be used to drive the display
    /// * `dma_chs`: DMA channels to be used to drive the PIO state machines
    pub fn new<'l, CH0, CH2, CH3>(
        buffer: &'static mut DisplayMemory<W, H, B>,
        r1: impl PioPin,
        g1: impl PioPin,
        b1: impl PioPin,
        r2: impl PioPin,
        g2: impl PioPin,
        b2: impl PioPin,
        addra: impl PioPin,
        addrb: impl PioPin,
        addrc: impl PioPin,
        addrd: impl PioPin,
        clk: impl PioPin,
        lat: impl PioPin,
        oe: impl PioPin,
        dma_0: CH0,
        dma_1: CH1,
        dma_2: CH2,
        dma_3: CH3,
        benchmark: bool,
        lut: &'a impl lut::Lut<B, C>,
    ) -> Self
    where
        CH0: Channel,
        CH2: Channel,
        CH3: Channel,
        C: RgbColor,
    {
        let pio = unsafe { peripherals::PIO0::steal() };
        let mut pio0 = Pio::new(pio, Irqs);
        let mut common = pio0.common;
        let mut data_sm = pio0.sm0;
        let mut row_sm = pio0.sm1;
        let mut oe_sm = pio0.sm2;

        setup_pio_task_data::<W>(&mut common, &mut data_sm, r1, g1, b1, r2, g2, b2, clk);
        setup_pio_task_row::<H, B>(&mut common, &mut row_sm, addra, addrb, addrc, addrd, lat);
        setup_pio_task_oe(&mut common, &mut oe_sm, oe);

        // Setup DMA
        let (fb_ch, fb_loop_ch, oe_ch, oe_loop_ch) = (dma_0, dma_1, dma_2, dma_3);

        // TODO: move this to a better place
        buffer.fbptr[0] = buffer.fb0.as_ptr() as u32;
        buffer.delaysptr[0] = buffer.delays.as_ptr() as u32;

        // Framebuffer channel
        fb_ch.regs().ctrl_trig().write(|w| {
            w.set_incr_read(true);
            w.set_incr_write(false);
            w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
            w.set_treq_sel(TreqSel(0 * 8 + 0 as u8));
            w.set_irq_quiet(!benchmark);
            w.set_chain_to(fb_loop_ch.number());
            w.set_en(true);

        });

        fb_ch.regs().read_addr().write_value(buffer.fbptr[0] as u32);
        fb_ch
            .regs()
            .trans_count()
            .write_value((fb_bytes(W, H, B) / 4) as u32);
        fb_ch
            .regs()
            .write_addr()
            .write_value(embassy_rp::pac::PIO0.txf(0).as_ptr() as u32);

        //// Framebuffer loop channel
        fb_loop_ch.regs().ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
            w.set_treq_sel(TreqSel::PERMANENT);
            w.set_irq_quiet(true);
            w.set_chain_to(fb_ch.number());
            w.set_en(true);
        });

        fb_loop_ch
            .regs()
            .read_addr()
            .write_value(buffer.fbptr.as_ptr() as u32);
        fb_loop_ch.regs().trans_count().write_value(1 as u32);
        fb_loop_ch
            .regs()
            .al2_write_addr_trig()
            .write_value(fb_ch.regs().read_addr().as_ptr() as u32);

        //// Output enable channel
        oe_ch.regs().ctrl_trig().write(|w| {
            w.set_incr_read(true);
            w.set_incr_write(false);
            w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
            w.set_treq_sel(TreqSel(0 * 8 + 2 as u8));
            w.set_irq_quiet(true);
            w.set_chain_to(oe_loop_ch.number());
            w.set_en(true);
        });

        oe_ch
            .regs()
            .read_addr()
            .write_value(buffer.delays.as_ptr() as u32);
        oe_ch
            .regs()
            .trans_count()
            .write_value(buffer.delays.len().try_into().unwrap());
        oe_ch
            .regs()
            .write_addr()
            .write_value(embassy_rp::pac::PIO0.txf(2).as_ptr() as u32);
 
        //// Output enable loop channel
        oe_loop_ch.regs().ctrl_trig().write(|w| {
            w.set_incr_read(false);
            w.set_incr_write(false);
            w.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_WORD);
            w.set_treq_sel(TreqSel::PERMANENT);
            w.set_irq_quiet(true);
            w.set_chain_to(oe_ch.number());
            w.set_en(true);
        });

        oe_loop_ch
            .regs()
            .read_addr()
            .write_value(buffer.delaysptr.as_ptr() as u32);
        oe_loop_ch
            .regs()
            .trans_count()
            .write_value(buffer.delaysptr.len().try_into().unwrap());
        oe_loop_ch
            .regs()
            .al2_write_addr_trig()
            .write_value(oe_ch.regs().read_addr().as_ptr() as u32);

        data_sm.set_enable(true);
        row_sm.set_enable(true);
        oe_sm.set_enable(true);

        defmt::info!("data sm is enabled: {}", data_sm.is_enabled());
        defmt::info!("row sm is enabled: {}", row_sm.is_enabled());
        defmt::info!("oe sm is enabled: {}", oe_sm.is_enabled());

        Display {
            mem: buffer,
            fb_loop_ch,
            benchmark,
            brightness: 255,
            lut,
        }
    }

    fn fb_loop_busy(&self) -> bool {
        self.fb_loop_ch.regs().ctrl_trig().read().busy()
    }

    /// Flips the display buffers
    ///
    /// Has to be called once you have drawn something onto the currently inactive buffer.
    pub fn commit(&mut self) {
        defmt::info!("commit");
        if self.mem.fbptr[0] == (self.mem.fb0.as_ptr() as u32) {
            self.mem.fbptr[0] = self.mem.fb1.as_ptr() as u32;
            //while !self.benchmark && !self.fb_loop_busy() {}
            while !self.fb_loop_busy() {}
            self.mem.fb0[0..].fill(0);
        } else {
            self.mem.fbptr[0] = self.mem.fb0.as_ptr() as u32;
            //while !self.benchmark && !self.fb_loop_busy() {}
            while !self.fb_loop_busy() {}
            self.mem.fb1[0..].fill(0);
        }

        defmt::info!("commit2");
    }

    /// Paints the given pixel coordinates with the given color
    ///
    /// Note that the coordinates are 0-indexed.
    pub fn set_pixel(&mut self, x: usize, y: usize, color: C) {
        // invert the screen
        let x = W - 1 - x;
        let y = H - 1 - y;
        // Half of the screen
        let h = y > (H / 2) - 1;
        let shift = if h { 3 } else { 0 };
        let (c_r, c_g, c_b) = self.lut.lookup(color);
        let c_r: u16 = ((c_r as f32) * (self.brightness as f32 / 255f32)) as u16;
        let c_g: u16 = ((c_g as f32) * (self.brightness as f32 / 255f32)) as u16;
        let c_b: u16 = ((c_b as f32) * (self.brightness as f32 / 255f32)) as u16;
        let base_idx = x + ((y % (H / 2)) * W * B);
        for b in 0..B {
            // Extract the n-th bit of each component of the color and pack them
            let cr = c_r >> b & 0b1;
            let cg = c_g >> b & 0b1;
            let cb = c_b >> b & 0b1;
            let packed_rgb = (cb << 2 | cg << 1 | cr) as u8;
            let idx = base_idx + b * W;
            if self.mem.fbptr[0] == (self.mem.fb0.as_ptr() as u32) {
                self.mem.fb1[idx] &= !(0b111 << shift);
                self.mem.fb1[idx] |= packed_rgb << shift;
            } else {
                self.mem.fb0[idx] &= !(0b111 << shift);
                self.mem.fb0[idx] |= packed_rgb << shift;
            }
        }
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness
    }
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> OriginDimensions
    for Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: Channel,
    C: RgbColor,
{
    fn size(&self) -> Size {
        Size::new(W as u32, H as u32)
    }
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> DrawTarget
    for Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: Channel,
    C: RgbColor,
{
    type Color = C;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if (coord.x as usize) < W && (coord.y as usize) < H {
                self.set_pixel(coord.x as usize, coord.y as usize, color);
            }
        }

        Ok(())
    }
}

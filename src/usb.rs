//! Embassy USB device driver for the CH58x USBFS controller at `0x4000_8000`.
//!
//! CH58x endpoints are backed by fixed 64-byte DMA banks. The driver owns those
//! banks, translates transfer interrupts into endpoint futures, and leaves
//! descriptor/class handling to `embassy-usb`. CH582 encodes DMA addresses as
//! 16-bit SRAM offsets while CH585 uses complete 32-bit addresses; the selected
//! chip backend keeps that incompatible register access out of the shared USB
//! state machine.

#![allow(async_fn_in_trait)]

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    self as driver, Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo,
    EndpointType, Event, Unsupported,
};
use portable_atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use crate::interrupt::typelevel::{Binding, Handler, USB as UsbInterrupt};

mod chip;

const USB_BASE: usize = 0x4000_8000;
const USB_IRQ: u8 = 22;
// EP0 plus the three independently DMA-backed non-control endpoint numbers.
// EP1 and EP2 can each be bidirectional while EP3 supplies another IN
// direction.
const ENDPOINT_COUNT: usize = 4;
const EP_PACKET_SIZE: usize = 64;

const UIF_BUS_RESET: u8 = 0x01;
const UIF_TRANSFER: u8 = 0x02;
const UIF_SUSPEND: u8 = 0x04;
const UIS_SETUP_ACT: u8 = 0x80;
const UIS_TOG_OK: u8 = 0x40;
const TOKEN_MASK: u8 = 0x30;
const TOKEN_OUT: u8 = 0x00;
const TOKEN_IN: u8 = 0x20;
const TOKEN_SETUP: u8 = 0x30;
const ENDPOINT_MASK: u8 = 0x0f;

// CH58x leaves the low endpoint-number nibble of INT_ST unspecified for a
// SETUP transaction. In particular, CH585 hardware has been observed
// reporting 0xbd for a valid EP0 SETUP. SETUP_ACT is the authoritative marker
// and must be decoded before validating the endpoint-number nibble.
const fn is_setup_status(status: u8) -> bool {
    status & UIS_SETUP_ACT != 0
}

const _: () = {
    assert!(is_setup_status(0xbd));
    assert!(is_setup_status(0xb0));
    assert!(!is_setup_status(0x3d));
};

const UEP_R_TOG: u8 = 0x80;
const UEP_T_TOG: u8 = 0x40;
const UEP_AUTO_TOG: u8 = 0x10;
const UEP_R_RES_MASK: u8 = 0x0c;
const UEP_R_RES_ACK: u8 = 0x00;
const UEP_R_RES_NAK: u8 = 0x08;
const UEP_R_RES_STALL: u8 = 0x0c;
const UEP_T_RES_MASK: u8 = 0x03;
const UEP_T_RES_ACK: u8 = 0x00;
const UEP_T_RES_NAK: u8 = 0x02;
const UEP_T_RES_STALL: u8 = 0x03;

#[repr(C, align(4))]
struct EndpointBuffer([u8; EP_PACKET_SIZE * 2]);

#[repr(C, align(4))]
struct ControlBuffer([u8; EP_PACKET_SIZE]);

/// Storage shared by the CPU and the USB DMA engine.
///
/// Access remains raw until the bounded copy itself. This avoids manufacturing
/// repeated `&'static mut` references to one DMA bank while a transfer may be
/// active. The USB peripheral token and endpoint state machine provide the
/// higher-level ownership; this cell is only the audited hardware boundary.
#[repr(transparent)]
struct UsbDmaCell<T>(UnsafeCell<T>);

unsafe impl<T: Send> Sync for UsbDmaCell<T> {}

impl<T> UsbDmaCell<T> {
    const fn new(value: T) -> Self {
        Self(UnsafeCell::new(value))
    }

    fn as_mut_ptr(&self) -> *mut T {
        self.0.get()
    }
}

static EP0_BUFFER: UsbDmaCell<ControlBuffer> = UsbDmaCell::new(ControlBuffer([0; EP_PACKET_SIZE]));
static EP0_SETUP_SHADOW: UsbDmaCell<[u8; 8]> = UsbDmaCell::new([0; 8]);
static EP_BUFFERS: UsbDmaCell<[EndpointBuffer; ENDPOINT_COUNT - 1]> =
    UsbDmaCell::new([const { EndpointBuffer([0; EP_PACKET_SIZE * 2]) }; ENDPOINT_COUNT - 1]);

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP0_WAKER: AtomicWaker = AtomicWaker::new();
static IN_WAKERS: [AtomicWaker; ENDPOINT_COUNT] = [const { AtomicWaker::new() }; ENDPOINT_COUNT];
static OUT_WAKERS: [AtomicWaker; ENDPOINT_COUNT] = [const { AtomicWaker::new() }; ENDPOINT_COUNT];
static ENABLED_IN: AtomicU8 = AtomicU8::new(0);
static ENABLED_OUT: AtomicU8 = AtomicU8::new(0);
static ALLOCATED_IN: AtomicU8 = AtomicU8::new(0);
static ALLOCATED_OUT: AtomicU8 = AtomicU8::new(0);
static IN_COMPLETE: AtomicU8 = AtomicU8::new(0);
static OUT_COMPLETE: AtomicU8 = AtomicU8::new(0);
// Non-control transfers use monotonically advancing generations instead of a
// set/clear completion latch. Only one transfer can be outstanding per
// endpoint, so inequality with the pre-arm snapshot is sufficient and cannot
// lose an ISR completion to a concurrent clear. EP0 keeps the bit latch above
// because its multi-stage control-transfer state machine consumes direction
// bits explicitly.
static IN_COMPLETION_GENERATIONS: [AtomicU8; ENDPOINT_COUNT] =
    [const { AtomicU8::new(0) }; ENDPOINT_COUNT];
static OUT_COMPLETION_GENERATIONS: [AtomicU8; ENDPOINT_COUNT] =
    [const { AtomicU8::new(0) }; ENDPOINT_COUNT];
// SET_CONFIGURATION and SET_INTERFACE may disable and re-enable an endpoint
// before its transfer future is polled again. Enabled masks alone cannot
// distinguish that lifecycle boundary after the endpoint is live once more,
// so each direction also carries a generation that invalidates the old
// future. This is separate from transfer-completion generations: a lifecycle
// change must return Disabled, never masquerade as a successful packet.
static IN_ENABLE_GENERATIONS: [AtomicU32; ENDPOINT_COUNT] =
    [const { AtomicU32::new(0) }; ENDPOINT_COUNT];
static OUT_ENABLE_GENERATIONS: [AtomicU32; ENDPOINT_COUNT] =
    [const { AtomicU32::new(0) }; ENDPOINT_COUNT];
static OUT_LENGTHS: [AtomicU8; ENDPOINT_COUNT] = [const { AtomicU8::new(0) }; ENDPOINT_COUNT];
static BUS_EVENTS: AtomicU8 = AtomicU8::new(0);
static SETUP_PENDING: AtomicBool = AtomicBool::new(false);
static RESET_GENERATION: AtomicU32 = AtomicU32::new(0);

/// Electrically disconnects the USB device by disabling its D+ pull-up.
///
/// A PFIC reset can preserve peripheral and pin state on CH58x. Acceptance
/// firmware that is reflashed while an older USB image is running should call
/// this function, wait for a host-visible detach interval, and only then start
/// the USB driver. `Bus::enable` restores the pull-up automatically.
pub fn force_disconnect() {
    chip::disable_usb_pullup();
}

#[derive(Clone, Copy)]
struct Reg8(*mut u8);

unsafe impl Send for Reg8 {}
unsafe impl Sync for Reg8 {}

impl Reg8 {
    const unsafe fn at(offset: usize) -> Self {
        Self((USB_BASE + offset) as *mut u8)
    }

    fn read(self) -> u8 {
        unsafe { read_volatile(self.0) }
    }

    fn write(self, value: u8) {
        unsafe { write_volatile(self.0, value) }
    }

    fn modify(self, f: impl FnOnce(u8) -> u8) {
        self.write(f(self.read()));
    }
}

const CTRL: Reg8 = unsafe { Reg8::at(0x00) };
const UDEV_CTRL: Reg8 = unsafe { Reg8::at(0x01) };
const INT_EN: Reg8 = unsafe { Reg8::at(0x02) };
const DEV_AD: Reg8 = unsafe { Reg8::at(0x03) };
const MIS_ST: Reg8 = unsafe { Reg8::at(0x05) };
const INT_FG: Reg8 = unsafe { Reg8::at(0x06) };
const INT_ST: Reg8 = unsafe { Reg8::at(0x07) };
const RX_LEN: Reg8 = unsafe { Reg8::at(0x08) };

fn endpoint_control(index: usize) -> Reg8 {
    let offset = 0x22 + (index * 4);
    unsafe { Reg8::at(offset) }
}

fn endpoint_tx_length(index: usize) -> Reg8 {
    let offset = 0x20 + (index * 4);
    unsafe { Reg8::at(offset) }
}

fn endpoint_buffer_ptr(index: usize, direction: Direction) -> *mut u8 {
    if index == 0 {
        return EP0_BUFFER.as_mut_ptr().cast::<u8>();
    }
    assert!(index < ENDPOINT_COUNT);
    let pointer = EP_BUFFERS.as_mut_ptr().cast::<EndpointBuffer>();
    let pointer = unsafe { pointer.add(index - 1).cast::<u8>() };
    // A unidirectional endpoint uses the 64-byte bank at its DMA base. The
    // second bank is the IN bank only when the same endpoint number has both
    // OUT and IN enabled (for example CDC data on a shared endpoint number).
    let offset = match direction {
        Direction::Out => 0,
        Direction::In if ALLOCATED_OUT.load(Ordering::Acquire) & endpoint_mask(index) != 0 => {
            EP_PACKET_SIZE
        }
        Direction::In => 0,
    };
    unsafe { pointer.add(offset) }
}

fn write_endpoint_buffer(index: usize, direction: Direction, data: &[u8]) {
    assert!(data.len() <= EP_PACKET_SIZE);
    unsafe {
        core::ptr::copy_nonoverlapping(
            data.as_ptr(),
            endpoint_buffer_ptr(index, direction),
            data.len(),
        );
    }
}

fn read_endpoint_buffer(index: usize, direction: Direction, output: &mut [u8]) {
    assert!(output.len() <= EP_PACKET_SIZE);
    unsafe {
        core::ptr::copy_nonoverlapping(
            endpoint_buffer_ptr(index, direction).cast_const(),
            output.as_mut_ptr(),
            output.len(),
        );
    }
}

const fn endpoint_mask(index: usize) -> u8 {
    1 << index
}

/// Full-speed USB device driver.
pub struct Driver<'d> {
    usb: crate::pac::USB,
    allocated_in: u8,
    allocated_out: u8,
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d> Driver<'d> {
    /// Creates the exclusive USBFS driver instance.
    ///
    /// The caller transfers ownership of the PAC USB peripheral to this
    /// driver; using that peripheral elsewhere while the driver lives is a
    /// logic error.
    pub fn new(usb: crate::pac::USB, _irq: impl Binding<UsbInterrupt, InterruptHandler>) -> Self {
        Self {
            usb,
            allocated_in: 0,
            allocated_out: 0,
            _lifetime: PhantomData,
        }
    }

    fn allocate(
        allocated: &mut u8,
        direction: Direction,
        ep_type: EndpointType,
        requested: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d>, EndpointAllocError> {
        if ep_type == EndpointType::Isochronous || max_packet_size == 0 || max_packet_size > 64 {
            return Err(EndpointAllocError);
        }
        let index = match requested {
            Some(address) if address.direction() == direction => address.index(),
            Some(_) => return Err(EndpointAllocError),
            None if direction == Direction::In && ep_type == EndpointType::Interrupt => (1
                ..ENDPOINT_COUNT)
                .rev()
                .find(|index| *allocated & endpoint_mask(*index) == 0)
                .ok_or(EndpointAllocError)?,
            None => (1..ENDPOINT_COUNT)
                .find(|index| *allocated & endpoint_mask(*index) == 0)
                .ok_or(EndpointAllocError)?,
        };
        if index == 0 || index >= ENDPOINT_COUNT || *allocated & endpoint_mask(index) != 0 {
            return Err(EndpointAllocError);
        }
        *allocated |= endpoint_mask(index);
        Ok(Endpoint {
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, direction),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            reset_generation: RESET_GENERATION.load(Ordering::Acquire),
            _lifetime: PhantomData,
        })
    }
}

impl<'d> driver::Driver<'d> for Driver<'d> {
    type EndpointOut = Endpoint<'d>;
    type EndpointIn = Endpoint<'d>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        Self::allocate(
            &mut self.allocated_out,
            Direction::Out,
            ep_type,
            ep_addr,
            max_packet_size,
            interval_ms,
        )
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        Self::allocate(
            &mut self.allocated_in,
            Direction::In,
            ep_type,
            ep_addr,
            max_packet_size,
            interval_ms,
        )
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(matches!(control_max_packet_size, 8 | 16 | 32 | 64));
        // CH58x has no USB VBUS-detect event wired into this driver. Report
        // power once when the device starts so embassy-usb can call
        // `Bus::enable()`. Emitting this from `enable()` would deadlock the
        // initial state machine and then continuously re-emit the event.
        BUS_EVENTS.store(1 << 3, Ordering::Release);
        (
            Bus {
                _usb: self.usb,
                allocated_in: self.allocated_in,
                allocated_out: self.allocated_out,
                enabled: false,
                _lifetime: PhantomData,
            },
            ControlPipe {
                max_packet_size: control_max_packet_size as usize,
                reset_generation: RESET_GENERATION.load(Ordering::Acquire),
                _lifetime: PhantomData,
            },
        )
    }
}

const fn endpoint_mode_bytes(allocated_in: u8, allocated_out: u8) -> [u8; 3] {
    let mut modes = [0u8; 3];
    let mut index = 1;
    while index < ENDPOINT_COUNT {
        let tx = allocated_in & endpoint_mask(index) != 0;
        let rx = allocated_out & endpoint_mask(index) != 0;
        match index {
            1 => modes[0] |= ((tx as u8) << 6) | ((rx as u8) << 7),
            2 => modes[1] |= ((tx as u8) << 2) | ((rx as u8) << 3),
            3 => modes[1] |= ((tx as u8) << 6) | ((rx as u8) << 7),
            _ => unreachable!(),
        }
        index += 1;
    }
    modes
}

const _: () = {
    let none = endpoint_mode_bytes(0, 0);
    assert!(none[0] == 0x00 && none[1] == 0x00 && none[2] == 0x00);
    let ep1_in = endpoint_mode_bytes(0x02, 0);
    assert!(ep1_in[0] == 0x40 && ep1_in[1] == 0x00 && ep1_in[2] == 0x00);
    let ep1_out = endpoint_mode_bytes(0, 0x02);
    assert!(ep1_out[0] == 0x80 && ep1_out[1] == 0x00 && ep1_out[2] == 0x00);
    let ep2_in = endpoint_mode_bytes(0x04, 0);
    assert!(ep2_in[0] == 0x00 && ep2_in[1] == 0x04 && ep2_in[2] == 0x00);
    let ep2_out = endpoint_mode_bytes(0, 0x04);
    assert!(ep2_out[0] == 0x00 && ep2_out[1] == 0x08 && ep2_out[2] == 0x00);
    let ep3_in = endpoint_mode_bytes(0x08, 0);
    assert!(ep3_in[0] == 0x00 && ep3_in[1] == 0x40 && ep3_in[2] == 0x00);
    let ep3_out = endpoint_mode_bytes(0, 0x08);
    assert!(ep3_out[0] == 0x00 && ep3_out[1] == 0x80 && ep3_out[2] == 0x00);
    let bidirectional = endpoint_mode_bytes(0x0e, 0x0e);
    assert!(bidirectional[0] == 0xc0 && bidirectional[1] == 0xcc && bidirectional[2] == 0x00);
};

fn configure_endpoint_modes(allocated_in: u8, allocated_out: u8) {
    let modes = endpoint_mode_bytes(allocated_in, allocated_out);
    unsafe { Reg8::at(0x0c) }.write(modes[0]);
    unsafe { Reg8::at(0x0d) }.write(modes[1]);
    unsafe { Reg8::at(0x0e) }.write(modes[2]);
}

fn configure_dma() {
    let ep0 = EP0_BUFFER.as_mut_ptr().cast::<u8>() as usize;
    chip::write_endpoint_dma(0, ep0);
    for index in 1..ENDPOINT_COUNT {
        let address = endpoint_buffer_ptr(index, Direction::Out) as usize;
        chip::write_endpoint_dma(index, address);
    }
}

/// USB bus owner returned after endpoint allocation.
pub struct Bus<'d> {
    // Keep the unique PAC token alive for the complete enabled/disabled bus
    // lifetime. This makes the register/DMA owner explicit instead of relying
    // on the token having been consumed and dropped in `Driver::new`.
    _usb: crate::pac::USB,
    allocated_in: u8,
    allocated_out: u8,
    enabled: bool,
    _lifetime: PhantomData<&'d mut ()>,
}

impl driver::Bus for Bus<'_> {
    async fn enable(&mut self) {
        ALLOCATED_IN.store(self.allocated_in, Ordering::Release);
        ALLOCATED_OUT.store(self.allocated_out, Ordering::Release);
        // Stop the SIE before programming endpoint mode and DMA registers.
        // Programming these in Driver::start() was too early: this reset can
        // discard the EP2/EP3 mode byte, leaving an allocated endpoint with a
        // valid descriptor but no DMA-backed data path.
        CTRL.write(0);
        configure_endpoint_modes(self.allocated_in, self.allocated_out);
        configure_dma();
        reset_endpoint_state(self.allocated_in, self.allocated_out, false);
        DEV_AD.write(0);
        CTRL.write(0x20 | 0x08 | 0x01);
        chip::enable_usb_pins();
        INT_FG.write(0xff);
        UDEV_CTRL.write(0x80 | 0x01);
        INT_EN.write(UIF_SUSPEND | UIF_TRANSFER | UIF_BUS_RESET);
        unsafe {
            qingke::pfic::set_priority(USB_IRQ, qingke::interrupt::Priority::P1.into());
            qingke::pfic::enable_interrupt(USB_IRQ);
        }
        self.enabled = true;
    }

    async fn disable(&mut self) {
        unsafe { qingke::pfic::disable_interrupt(USB_IRQ) };
        INT_EN.write(0);
        UDEV_CTRL.write(0x80);
        CTRL.write(0);
        self.enabled = false;
        RESET_GENERATION.fetch_add(1, Ordering::AcqRel);
        ENABLED_IN.store(0, Ordering::Release);
        ENABLED_OUT.store(0, Ordering::Release);
        wake_all_endpoints();
    }

    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());
            let events = BUS_EVENTS.load(Ordering::Acquire);
            for (bit, event) in [
                (0, Event::Reset),
                (1, Event::Suspend),
                (2, Event::Resume),
                (3, Event::PowerDetected),
            ] {
                if events & (1 << bit) != 0 {
                    BUS_EVENTS.fetch_and(!(1 << bit), Ordering::AcqRel);
                    return Poll::Ready(event);
                }
            }
            Poll::Pending
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let mask = endpoint_mask(ep_addr.index());
        let state = match ep_addr.direction() {
            Direction::In => &ENABLED_IN,
            Direction::Out => &ENABLED_OUT,
        };
        with_usb_irq_masked(|| {
            match ep_addr.direction() {
                Direction::In => {
                    IN_ENABLE_GENERATIONS[ep_addr.index()].fetch_add(1, Ordering::AcqRel)
                }
                Direction::Out => {
                    OUT_ENABLE_GENERATIONS[ep_addr.index()].fetch_add(1, Ordering::AcqRel)
                }
            };
            if enabled {
                state.fetch_or(mask, Ordering::AcqRel);
            } else {
                state.fetch_and(!mask, Ordering::AcqRel);
            }
            let control = endpoint_control(ep_addr.index());
            match ep_addr.direction() {
                Direction::In => {
                    if !enabled {
                        IN_COMPLETE.fetch_and(!mask, Ordering::AcqRel);
                    }
                    control.modify(|value| {
                        // A newly selected alternate endpoint starts at
                        // DATA0. Clear the old endpoint's manual/auto-toggle
                        // state before exposing the new instance.
                        let value = if enabled { value & !UEP_T_TOG } else { value };
                        (value & !UEP_T_RES_MASK)
                            | if enabled {
                                UEP_T_RES_NAK
                            } else {
                                UEP_T_RES_STALL
                            }
                    })
                }
                Direction::Out => {
                    if !enabled {
                        // An alternate-setting change cancels ownership of a
                        // packet received under the old endpoint instance.
                        OUT_COMPLETE.fetch_and(!mask, Ordering::AcqRel);
                    }
                    control.modify(|value| {
                        let value = if enabled { value & !UEP_R_TOG } else { value };
                        (value & !UEP_R_RES_MASK)
                            | if enabled {
                                UEP_R_RES_ACK
                            } else {
                                UEP_R_RES_STALL
                            }
                    })
                }
            }
        });
        endpoint_waker(ep_addr).wake();
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        with_usb_irq_masked(|| {
            let control = endpoint_control(ep_addr.index());
            match ep_addr.direction() {
                Direction::In => control.modify(|value| {
                    // USB CLEAR_FEATURE(ENDPOINT_HALT) resets the affected
                    // direction's data toggle to DATA0. Preserve it while
                    // entering STALL, but clear it when the halt is removed.
                    let value = if stalled { value } else { value & !UEP_T_TOG };
                    (value & !UEP_T_RES_MASK)
                        | if stalled {
                            UEP_T_RES_STALL
                        } else {
                            UEP_T_RES_NAK
                        }
                }),
                Direction::Out => control.modify(|value| {
                    let value = if stalled { value } else { value & !UEP_R_TOG };
                    (value & !UEP_R_RES_MASK)
                        | if stalled {
                            UEP_R_RES_STALL
                        } else if OUT_COMPLETE.load(Ordering::Acquire)
                            & endpoint_mask(ep_addr.index())
                            != 0
                        {
                            // One packet already owns this endpoint's only
                            // DMA bank. Clearing halt restores DATA0 but must
                            // keep NAK until EndpointOut::read consumes it.
                            UEP_R_RES_NAK
                        } else {
                            UEP_R_RES_ACK
                        }
                }),
            }
        });
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let value = endpoint_control(ep_addr.index()).read();
        match ep_addr.direction() {
            Direction::In => value & UEP_T_RES_MASK == UEP_T_RES_STALL,
            Direction::Out => value & UEP_R_RES_MASK == UEP_R_RES_STALL,
        }
    }

    fn force_reset(&mut self) -> Result<(), Unsupported> {
        CTRL.modify(|value| value | 0x04);
        for _ in 0..16 {
            core::hint::spin_loop();
        }
        CTRL.modify(|value| value & !0x04);
        Ok(())
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        if !self.enabled || MIS_ST.read() & 0x04 == 0 {
            return Err(Unsupported);
        }
        CTRL.modify(|value| (value & !0x30) | 0x30);
        embassy_time::Timer::after_millis(2).await;
        CTRL.modify(|value| (value & !0x30) | 0x20);
        Ok(())
    }
}

/// Allocated non-control endpoint.
pub struct Endpoint<'d> {
    info: EndpointInfo,
    reset_generation: u32,
    _lifetime: PhantomData<&'d mut ()>,
}

/// Restores NAK when an endpoint transfer future is cancelled before its
/// completion interrupt. This returns DMA ownership to software before a
/// later call is allowed to reuse the static endpoint bank.
struct TransferGuard {
    index: usize,
    direction: Direction,
    reset_generation: u32,
    enable_generation: u32,
    completion_generation: u8,
    armed: bool,
}

impl TransferGuard {
    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for TransferGuard {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }

        critical_section::with(|_| {
            if RESET_GENERATION.load(Ordering::Acquire) != self.reset_generation {
                return;
            }

            let (enabled, enable_generation, completion_generation) = match self.direction {
                Direction::In => (
                    ENABLED_IN.load(Ordering::Acquire),
                    IN_ENABLE_GENERATIONS[self.index].load(Ordering::Acquire),
                    IN_COMPLETION_GENERATIONS[self.index].load(Ordering::Acquire),
                ),
                Direction::Out => (
                    ENABLED_OUT.load(Ordering::Acquire),
                    OUT_ENABLE_GENERATIONS[self.index].load(Ordering::Acquire),
                    OUT_COMPLETION_GENERATIONS[self.index].load(Ordering::Acquire),
                ),
            };
            if enabled & endpoint_mask(self.index) == 0
                || enable_generation != self.enable_generation
                || completion_generation != self.completion_generation
            {
                return;
            }

            endpoint_control(self.index).modify(|value| match self.direction {
                Direction::In if value & UEP_T_RES_MASK == UEP_T_RES_ACK => {
                    (value & !UEP_T_RES_MASK) | UEP_T_RES_NAK
                }
                Direction::Out if value & UEP_R_RES_MASK == UEP_R_RES_ACK => {
                    (value & !UEP_R_RES_MASK) | UEP_R_RES_NAK
                }
                _ => value,
            });
        });
    }
}

impl driver::Endpoint for Endpoint<'_> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let mask = endpoint_mask(self.info.addr.index());
        let enabled = match self.info.addr.direction() {
            Direction::In => &ENABLED_IN,
            Direction::Out => &ENABLED_OUT,
        };
        poll_fn(|cx| {
            endpoint_waker(self.info.addr).register(cx.waker());
            if enabled.load(Ordering::Acquire) & mask != 0 {
                self.reset_generation = RESET_GENERATION.load(Ordering::Acquire);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl driver::EndpointIn for Endpoint<'_> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let index = self.info.addr.index();
        if data.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if ENABLED_IN.load(Ordering::Acquire) & endpoint_mask(index) == 0 {
            return Err(EndpointError::Disabled);
        }
        let generation = RESET_GENERATION.load(Ordering::Acquire);
        let enable_generation = IN_ENABLE_GENERATIONS[index].load(Ordering::Acquire);
        write_endpoint_buffer(index, Direction::In, data);
        let completion_generation = with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || IN_ENABLE_GENERATIONS[index].load(Ordering::Acquire) != enable_generation
                || ENABLED_IN.load(Ordering::Acquire) & endpoint_mask(index) == 0
            {
                return None;
            }
            let completion_generation = IN_COMPLETION_GENERATIONS[index].load(Ordering::Acquire);
            IN_COMPLETE.fetch_and(!endpoint_mask(index), Ordering::AcqRel);
            endpoint_tx_length(index).write(data.len() as u8);
            endpoint_control(index).modify(|value| (value & !UEP_T_RES_MASK) | UEP_T_RES_ACK);
            Some(completion_generation)
        })
        .ok_or(EndpointError::Disabled)?;

        let mut guard = TransferGuard {
            index,
            direction: Direction::In,
            reset_generation: generation,
            enable_generation,
            completion_generation,
            armed: true,
        };

        let result = poll_fn(|cx| {
            IN_WAKERS[index].register(cx.waker());
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || IN_ENABLE_GENERATIONS[index].load(Ordering::Acquire) != enable_generation
                || ENABLED_IN.load(Ordering::Acquire) & endpoint_mask(index) == 0
            {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if IN_COMPLETION_GENERATIONS[index].load(Ordering::Acquire)
                != completion_generation
            {
                IN_COMPLETE.fetch_and(!endpoint_mask(index), Ordering::AcqRel);
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await;
        if result.is_ok() {
            guard.disarm();
        }
        result
    }
}

impl driver::EndpointOut for Endpoint<'_> {
    async fn read(&mut self, output: &mut [u8]) -> Result<usize, EndpointError> {
        enum ReadStart {
            Disabled,
            Completed(usize),
            Armed(u8),
        }

        let index = self.info.addr.index();
        let mask = endpoint_mask(index);
        if ENABLED_OUT.load(Ordering::Acquire) & mask == 0 {
            return Err(EndpointError::Disabled);
        }
        let generation = RESET_GENERATION.load(Ordering::Acquire);
        let enable_generation = OUT_ENABLE_GENERATIONS[index].load(Ordering::Acquire);
        let start = with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || OUT_ENABLE_GENERATIONS[index].load(Ordering::Acquire) != enable_generation
                || ENABLED_OUT.load(Ordering::Acquire) & mask == 0
            {
                return ReadStart::Disabled;
            }
            if OUT_COMPLETE.fetch_and(!mask, Ordering::AcqRel) & mask != 0 {
                // endpoint_set_enabled/clear-halt may have allowed one OUT
                // token before the read future was first polled. The ISR has
                // already changed the endpoint to NAK, so consume that owned
                // DMA packet instead of clearing its completion and waiting
                // forever for a second host transfer.
                ReadStart::Completed(usize::from(OUT_LENGTHS[index].load(Ordering::Acquire)))
            } else {
                let completion_generation =
                    OUT_COMPLETION_GENERATIONS[index].load(Ordering::Acquire);
                endpoint_control(index).modify(|value| (value & !UEP_R_RES_MASK) | UEP_R_RES_ACK);
                ReadStart::Armed(completion_generation)
            }
        });
        let length = match start {
            ReadStart::Disabled => return Err(EndpointError::Disabled),
            ReadStart::Completed(length) => {
                if RESET_GENERATION.load(Ordering::Acquire) != generation
                    || OUT_ENABLE_GENERATIONS[index].load(Ordering::Acquire) != enable_generation
                    || ENABLED_OUT.load(Ordering::Acquire) & mask == 0
                {
                    return Err(EndpointError::Disabled);
                }
                length
            }
            ReadStart::Armed(completion_generation) => {
                let mut guard = TransferGuard {
                    index,
                    direction: Direction::Out,
                    reset_generation: generation,
                    enable_generation,
                    completion_generation,
                    armed: true,
                };
                let result = poll_fn(|cx| {
                    OUT_WAKERS[index].register(cx.waker());
                    if RESET_GENERATION.load(Ordering::Acquire) != generation
                        || OUT_ENABLE_GENERATIONS[index].load(Ordering::Acquire)
                            != enable_generation
                        || ENABLED_OUT.load(Ordering::Acquire) & mask == 0
                    {
                        Poll::Ready(Err(EndpointError::Disabled))
                    } else if OUT_COMPLETION_GENERATIONS[index].load(Ordering::Acquire)
                        != completion_generation
                    {
                        OUT_COMPLETE.fetch_and(!mask, Ordering::AcqRel);
                        Poll::Ready(Ok(usize::from(OUT_LENGTHS[index].load(Ordering::Acquire))))
                    } else {
                        Poll::Pending
                    }
                })
                .await;
                if result.is_ok() {
                    guard.disarm();
                }
                result?
            }
        };
        if length > output.len() {
            return Err(EndpointError::BufferOverflow);
        }
        read_endpoint_buffer(index, Direction::Out, &mut output[..length]);
        Ok(length)
    }
}

/// Endpoint-zero state machine.
pub struct ControlPipe<'d> {
    max_packet_size: usize,
    reset_generation: u32,
    _lifetime: PhantomData<&'d mut ()>,
}

struct ControlTransferGuard {
    direction: Direction,
    reset_generation: u32,
    armed: bool,
}

impl ControlTransferGuard {
    fn new(direction: Direction, reset_generation: u32) -> Self {
        Self {
            direction,
            reset_generation,
            armed: true,
        }
    }

    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for ControlTransferGuard {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }

        critical_section::with(|_| {
            if RESET_GENERATION.load(Ordering::Acquire) != self.reset_generation
                || SETUP_PENDING.load(Ordering::Acquire)
            {
                return;
            }

            let completed = match self.direction {
                Direction::In => IN_COMPLETE.load(Ordering::Acquire) & 1 != 0,
                Direction::Out => OUT_COMPLETE.load(Ordering::Acquire) & 1 != 0,
            };
            if completed {
                return;
            }

            endpoint_control(0).modify(|value| match self.direction {
                Direction::In if value & UEP_T_RES_MASK == UEP_T_RES_ACK => {
                    (value & !UEP_T_RES_MASK) | UEP_T_RES_NAK
                }
                Direction::Out if value & UEP_R_RES_MASK == UEP_R_RES_ACK => {
                    (value & !UEP_R_RES_MASK) | UEP_R_RES_NAK
                }
                _ => value,
            });
        });
    }
}

impl ControlPipe<'_> {
    async fn wait_ep0(&mut self, direction: Direction) -> Result<usize, EndpointError> {
        let generation = self.reset_generation;
        poll_fn(|cx| {
            EP0_WAKER.register(cx.waker());
            if RESET_GENERATION.load(Ordering::Acquire) != generation {
                self.reset_generation = RESET_GENERATION.load(Ordering::Acquire);
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if SETUP_PENDING.load(Ordering::Acquire) {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let completed = match direction {
                Direction::In => IN_COMPLETE.fetch_and(!1, Ordering::AcqRel) & 1 != 0,
                Direction::Out => OUT_COMPLETE.fetch_and(!1, Ordering::AcqRel) & 1 != 0,
            };
            if completed {
                Poll::Ready(Ok(usize::from(OUT_LENGTHS[0].load(Ordering::Acquire))))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn prepare_in(&self, data: &[u8], rx_status: u8, generation: u32) -> bool {
        with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || SETUP_PENDING.load(Ordering::Acquire)
            {
                return false;
            }
            IN_COMPLETE.fetch_and(!1, Ordering::AcqRel);
            write_endpoint_buffer(0, Direction::In, data);
            endpoint_tx_length(0).write(data.len() as u8);
            // SETUP primes EP0 for DATA1. Keep the current PID here: the
            // transfer ISR advances it after each successfully acknowledged
            // packet. Writing DATA1 unconditionally breaks descriptors longer
            // than one EP0 packet.
            endpoint_control(0).modify(|value| {
                (value & !(UEP_R_RES_MASK | UEP_T_RES_MASK)) | rx_status | UEP_T_RES_ACK
            });
            true
        })
    }

    async fn status_in(&mut self, address: Option<u8>) {
        let generation = self.reset_generation;
        if !self.prepare_in(&[], UEP_R_RES_ACK, generation) {
            return;
        }
        let mut guard = ControlTransferGuard::new(Direction::In, generation);
        if self.wait_ep0(Direction::In).await.is_err() {
            return;
        }
        let completed = with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || SETUP_PENDING.load(Ordering::Acquire)
            {
                return false;
            }
            endpoint_control(0).write(UEP_R_RES_ACK | UEP_T_RES_NAK);
            if let Some(address) = address {
                DEV_AD.modify(|value| (value & 0x80) | (address & 0x7f));
            }
            true
        });
        if completed {
            guard.disarm();
        }
    }
}

impl driver::ControlPipe for ControlPipe<'_> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|cx| {
            EP0_WAKER.register(cx.waker());
            critical_section::with(|_| {
                if SETUP_PENDING.swap(false, Ordering::AcqRel) {
                    let mut setup = [0; 8];
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            EP0_SETUP_SHADOW.as_mut_ptr().cast::<u8>().cast_const(),
                            setup.as_mut_ptr(),
                            setup.len(),
                        );
                    }
                    endpoint_control(0)
                        .write(UEP_R_TOG | UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK);
                    self.reset_generation = RESET_GENERATION.load(Ordering::Acquire);
                    Poll::Ready(setup)
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }

    async fn data_out(
        &mut self,
        output: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        let generation = self.reset_generation;
        with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || SETUP_PENDING.load(Ordering::Acquire)
            {
                return Err(EndpointError::Disabled);
            }
            endpoint_control(0).modify(|value| (value & !UEP_R_RES_MASK) | UEP_R_RES_ACK);
            Ok(())
        })?;
        let mut guard = ControlTransferGuard::new(Direction::Out, generation);
        let length = self.wait_ep0(Direction::Out).await?;
        if length > output.len() {
            return Err(EndpointError::BufferOverflow);
        }
        with_usb_irq_masked(|| {
            if RESET_GENERATION.load(Ordering::Acquire) != generation
                || SETUP_PENDING.load(Ordering::Acquire)
            {
                return Err(EndpointError::Disabled);
            }
            read_endpoint_buffer(0, Direction::Out, &mut output[..length]);
            Ok(())
        })?;
        guard.disarm();
        Ok(length)
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        last: bool,
    ) -> Result<(), EndpointError> {
        if data.len() > self.max_packet_size {
            return Err(EndpointError::BufferOverflow);
        }
        let generation = self.reset_generation;
        if !self.prepare_in(
            data,
            if last { UEP_R_RES_ACK } else { UEP_R_RES_NAK },
            generation,
        ) {
            return Err(EndpointError::Disabled);
        }
        let mut guard = ControlTransferGuard::new(Direction::In, generation);
        self.wait_ep0(Direction::In).await?;
        if last {
            let completed = with_usb_irq_masked(|| {
                if RESET_GENERATION.load(Ordering::Acquire) != generation
                    || SETUP_PENDING.load(Ordering::Acquire)
                {
                    return false;
                }
                endpoint_control(0).write(UEP_R_TOG | UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK);
                true
            });
            if !completed {
                return Err(EndpointError::Disabled);
            }
        }
        guard.disarm();
        Ok(())
    }

    async fn accept(&mut self) {
        self.status_in(None).await;
    }

    async fn reject(&mut self) {
        with_usb_irq_masked(|| {
            endpoint_control(0).write(UEP_R_RES_STALL | UEP_T_RES_STALL);
        });
    }

    async fn accept_set_address(&mut self, address: u8) {
        self.status_in(Some(address)).await;
    }
}

fn endpoint_waker(address: EndpointAddress) -> &'static AtomicWaker {
    match address.direction() {
        Direction::In => &IN_WAKERS[address.index()],
        Direction::Out => &OUT_WAKERS[address.index()],
    }
}

/// Protect a combined endpoint-control read/modify/write from the USB ISR.
///
/// RX/TX response bits and both DATA toggles share one byte. A short global
/// critical section prevents an interrupt-side update from being overwritten
/// by a task-side stale read.
#[inline]
fn with_usb_irq_masked<R>(f: impl FnOnce() -> R) -> R {
    critical_section::with(|_| f())
}

fn reset_endpoint_state(allocated_in: u8, allocated_out: u8, enabled: bool) {
    DEV_AD.write(0);
    ENABLED_IN.store(if enabled { allocated_in } else { 0 }, Ordering::Release);
    ENABLED_OUT.store(if enabled { allocated_out } else { 0 }, Ordering::Release);
    IN_COMPLETE.store(0, Ordering::Release);
    OUT_COMPLETE.store(0, Ordering::Release);
    SETUP_PENDING.store(false, Ordering::Release);
    endpoint_control(0).write(UEP_R_RES_ACK | UEP_T_RES_NAK);
    for index in 1..ENDPOINT_COUNT {
        let enabled_in = enabled && allocated_in & endpoint_mask(index) != 0;
        let enabled_out = enabled && allocated_out & endpoint_mask(index) != 0;
        let toggle_mode = UEP_AUTO_TOG;
        endpoint_control(index).write(
            toggle_mode
                | if enabled_out {
                    UEP_R_RES_ACK
                } else {
                    UEP_R_RES_STALL
                }
                | if enabled_in {
                    UEP_T_RES_NAK
                } else {
                    UEP_T_RES_STALL
                },
        );
        endpoint_tx_length(index).write(0);
    }
}

fn wake_all_endpoints() {
    EP0_WAKER.wake();
    for waker in IN_WAKERS.iter().chain(OUT_WAKERS.iter()) {
        waker.wake();
    }
}

/// Type-level USBFS handler used by [`crate::bind_interrupts!`].
pub struct InterruptHandler;

impl Handler<UsbInterrupt> for InterruptHandler {
    // `qingke_rt::interrupt` places the vector wrapper in SRAM `.highcode`.
    // Keeping this call boundary prevents fat LTO from pulling the complete
    // USB protocol state machine into that scarce section. Only the
    // vector/trampoline needs the runtime's fast placement; the unchanged
    // handler body executes from Flash.
    #[inline(never)]
    unsafe fn on_interrupt() {
        usb_interrupt();
    }
}

fn usb_interrupt() {
    let flags = INT_FG.read();

    // If transfer and reset are latched together, retire the transfer
    // snapshot first and leave BUS_RESET set for the next IRQ entry. Never
    // reset state and then interpret stale INT_ST/RX_LEN from the same entry.
    if flags & UIF_BUS_RESET != 0 && flags & UIF_TRANSFER == 0 {
        RESET_GENERATION.fetch_add(1, Ordering::AcqRel);
        reset_endpoint_state(
            ALLOCATED_IN.load(Ordering::Acquire),
            ALLOCATED_OUT.load(Ordering::Acquire),
            false,
        );
        BUS_EVENTS.fetch_or(1 << 0, Ordering::AcqRel);
        INT_FG.write(UIF_BUS_RESET);
        wake_all_endpoints();
        BUS_WAKER.wake();
    }

    if flags & UIF_TRANSFER != 0 {
        let status = INT_ST.read();
        let setup = is_setup_status(status);
        let index = if setup {
            // SETUP always targets EP0. Do not reject it because INT_ST kept a
            // stale or unspecified endpoint number in its low nibble.
            0
        } else {
            usize::from(status & ENDPOINT_MASK)
        };
        if index < ENDPOINT_COUNT {
            match if setup {
                TOKEN_SETUP
            } else {
                status & TOKEN_MASK
            } {
                TOKEN_SETUP => {
                    // Every SETUP packet starts a new control transfer and
                    // aborts any earlier data/status stage. In particular,
                    // do not let the previous transfer's status-OUT latch be
                    // consumed by ControlPipe::data_out(): EP0's shared DMA
                    // bank now contains this SETUP packet, so that stale
                    // completion would return the request header as payload.
                    IN_COMPLETE.fetch_and(!endpoint_mask(0), Ordering::AcqRel);
                    OUT_COMPLETE.fetch_and(!endpoint_mask(0), Ordering::AcqRel);
                    // EP0 has one shared DMA bank. A control-write data packet
                    // may follow SETUP before the async control task runs, so
                    // preserve the eight-byte SETUP packet in its own shadow
                    // while still in the ISR.
                    unsafe {
                        core::ptr::copy_nonoverlapping(
                            endpoint_buffer_ptr(0, Direction::Out).cast_const(),
                            EP0_SETUP_SHADOW.as_mut_ptr().cast::<u8>(),
                            8,
                        );
                    }
                    SETUP_PENDING.store(true, Ordering::Release);
                    OUT_LENGTHS[0].store(8, Ordering::Release);
                    // Preserve SETUP in the shared EP0 DMA bank until the
                    // async control task has copied it. The task arms the
                    // selected data/status direction explicitly afterwards.
                    endpoint_control(0)
                        .write(UEP_R_TOG | UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK);
                    EP0_WAKER.wake();
                }
                TOKEN_OUT if status & UIS_TOG_OK != 0 => {
                    let length = RX_LEN.read();
                    if index == 0 {
                        // EP0 has no automatic toggle mode. Advance the
                        // expected OUT PID after a valid data/status packet.
                        endpoint_control(0).modify(|value| value ^ UEP_R_TOG);
                    } else {
                        // Each OUT direction has one DMA bank. Hold NAK until
                        // the endpoint future has copied this packet, or a
                        // back-to-back packet can overwrite the bank before
                        // the task observes the first completion.
                        endpoint_control(index)
                            .modify(|value| (value & !UEP_R_RES_MASK) | UEP_R_RES_NAK);
                    }
                    OUT_LENGTHS[index].store(length, Ordering::Release);
                    OUT_COMPLETE.fetch_or(endpoint_mask(index), Ordering::AcqRel);
                    if index == 0 {
                        EP0_WAKER.wake();
                    } else {
                        OUT_COMPLETION_GENERATIONS[index].fetch_add(1, Ordering::AcqRel);
                        OUT_WAKERS[index].wake();
                    }
                }
                TOKEN_OUT => {}
                TOKEN_IN => {
                    endpoint_control(index).modify(|value| {
                        if index == 0 {
                            // EP0 has no automatic toggle mode. The next
                            // packet in this control transfer uses the other
                            // DATA PID.
                            (value ^ UEP_T_TOG) & !UEP_T_RES_MASK | UEP_T_RES_NAK
                        } else {
                            (value & !UEP_T_RES_MASK) | UEP_T_RES_NAK
                        }
                    });
                    IN_COMPLETE.fetch_or(endpoint_mask(index), Ordering::AcqRel);
                    if index == 0 {
                        EP0_WAKER.wake();
                    } else {
                        IN_COMPLETION_GENERATIONS[index].fetch_add(1, Ordering::AcqRel);
                        IN_WAKERS[index].wake();
                    }
                }
                _ => {}
            }
        }
        INT_FG.write(UIF_TRANSFER);
    }

    if flags & UIF_SUSPEND != 0 && flags & (UIF_TRANSFER | UIF_BUS_RESET) == 0 {
        let event = if MIS_ST.read() & 0x04 != 0 {
            Event::Suspend
        } else {
            Event::Resume
        };
        let bit = match event {
            Event::Suspend => 1,
            Event::Resume => 2,
            _ => unreachable!(),
        };
        BUS_EVENTS.fetch_or(1 << bit, Ordering::AcqRel);
        INT_FG.write(UIF_SUSPEND);
        BUS_WAKER.wake();
    }
}

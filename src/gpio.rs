//! Owned GPIO pins for CH581/CH582/CH583/CH584/CH585.

use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin as FuturePin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::digital::{ErrorType, InputPin};
use portable_atomic::{AtomicBool, Ordering};
use qingke::interrupt::Priority;

pub use crate::hal::gpio::{AnyPin, Drive, Flex, GpioPin, Level, Output, Pin, PinId, Pins, Pull};
use crate::pac;

/// GPIO interrupt trigger mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Trigger {
    LowLevel,
    HighLevel,
    FallingEdge,
    RisingEdge,
}

const GPIO_PIN_COUNT: usize = 40;
static GPIO_EVENTS: [AtomicBool; GPIO_PIN_COUNT] =
    [const { AtomicBool::new(false) }; GPIO_PIN_COUNT];
static GPIO_WAKERS: [AtomicWaker; GPIO_PIN_COUNT] = [const { AtomicWaker::new() }; GPIO_PIN_COUNT];
fn event_index(port: u8, number: u8) -> usize {
    match port {
        0 => number as usize,
        1 => 16 + number as usize,
        _ => unreachable!(),
    }
}

fn interrupt_mask(port: u8, number: u8) -> u16 {
    match (port, number) {
        (0, 0..=15) | (1, 0..=15) => 1 << number,
        // PB22/PB23 share the PB8/PB9 interrupt channels through INTX remap.
        (1, 22..=23) => 1 << (number - 14),
        _ => panic!("this CH58x pin has no GPIO interrupt channel"),
    }
}

fn configure_interrupt(port: u8, number: u8, trigger: Trigger) {
    let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
    let registers = match port {
        0 => unsafe { &*pac::GPIOA::PTR },
        1 => unsafe { &*pac::GPIOB::PTR },
        _ => unreachable!(),
    };
    let pin_mask = 1u32 << number;
    let irq_mask = interrupt_mask(port, number);
    let edge = matches!(trigger, Trigger::FallingEdge | Trigger::RisingEdge);
    let active_high = matches!(trigger, Trigger::HighLevel | Trigger::RisingEdge);

    critical_section::with(|_| unsafe {
        if port == 1 && matches!(number, 8 | 9 | 22 | 23) {
            gpioctl.pin_alternate().modify(|r, w| {
                let bits = if number >= 22 {
                    r.bits() | (1 << 13)
                } else {
                    r.bits() & !(1 << 13)
                };
                w.bits(bits)
            });
        }

        if active_high {
            registers.out().modify(|r, w| w.bits(r.bits() | pin_mask));
        } else {
            registers.clr().write(|w| w.bits(pin_mask));
        }

        if port == 0 {
            gpioctl.pa_int_mode().modify(|r, w| {
                let bits = if edge {
                    r.bits() | irq_mask
                } else {
                    r.bits() & !irq_mask
                };
                w.bits(bits)
            });
            gpioctl.pa_int_if().write(|w| w.bits(irq_mask));
            gpioctl
                .pa_int_en()
                .modify(|r, w| w.bits(r.bits() | irq_mask));
        } else {
            gpioctl.pb_int_mode().modify(|r, w| {
                let bits = if edge {
                    r.bits() | irq_mask
                } else {
                    r.bits() & !irq_mask
                };
                w.bits(bits)
            });
            gpioctl.pb_int_if().write(|w| w.bits(irq_mask));
            gpioctl
                .pb_int_en()
                .modify(|r, w| w.bits(r.bits() | irq_mask));
        }
    });

    let irq = if port == 0 {
        pac::Interrupt::GPIOA as u8
    } else {
        pac::Interrupt::GPIOB as u8
    };
    unsafe {
        qingke::pfic::set_priority(irq, Priority::P14.into());
        qingke::pfic::enable_interrupt(irq);
    }
}

fn disable_interrupt(port: u8, number: u8) {
    let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
    let irq_mask = interrupt_mask(port, number);
    critical_section::with(|_| unsafe {
        if port == 0 {
            gpioctl
                .pa_int_en()
                .modify(|r, w| w.bits(r.bits() & !irq_mask));
        } else {
            gpioctl
                .pb_int_en()
                .modify(|r, w| w.bits(r.bits() & !irq_mask));
        }
    });
}

/// Future completed by the port interrupt handler.
pub struct WaitForInterrupt<'a> {
    port: u8,
    number: u8,
    index: usize,
    _borrow: PhantomData<&'a mut ()>,
}

impl<'a> WaitForInterrupt<'a> {
    fn new(port: u8, number: u8, trigger: Trigger) -> Self {
        let index = event_index(port, number);
        GPIO_EVENTS[index].store(false, Ordering::Release);
        configure_interrupt(port, number, trigger);
        Self {
            port,
            number,
            index,
            _borrow: PhantomData,
        }
    }
}

impl Future for WaitForInterrupt<'_> {
    type Output = ();

    fn poll(self: FuturePin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        if GPIO_EVENTS[this.index].swap(false, Ordering::AcqRel) {
            return Poll::Ready(());
        }
        GPIO_WAKERS[this.index].register(cx.waker());
        if GPIO_EVENTS[this.index].swap(false, Ordering::AcqRel) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for WaitForInterrupt<'_> {
    fn drop(&mut self) {
        disable_interrupt(self.port, self.number);
    }
}

fn signal(index: usize) {
    GPIO_EVENTS[index].store(true, Ordering::Release);
    GPIO_WAKERS[index].wake();
}

#[inline(never)]
fn handle_gpioa_interrupt() {
    let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
    let flags = gpioctl.pa_int_if().read().bits();
    gpioctl.pa_int_if().write(|w| unsafe { w.bits(flags) });
    for bit in 0..16 {
        if flags & (1 << bit) != 0 {
            signal(bit);
        }
    }
}

#[inline(never)]
fn handle_gpiob_interrupt() {
    let gpioctl = unsafe { &*pac::GPIOCTL::PTR };
    let flags = gpioctl.pb_int_if().read().bits();
    gpioctl.pb_int_if().write(|w| unsafe { w.bits(flags) });
    let remapped = gpioctl.pin_alternate().read().intx().bit_is_set();
    for bit in 0..16 {
        if flags & (1 << bit) != 0 {
            let pin = if remapped && matches!(bit, 8 | 9) {
                bit + 14
            } else {
                bit
            };
            signal(16 + pin as usize);
        }
    }
}

// Keep only the vector trampoline in SRAM. Flag scanning and waker dispatch
// execute from Flash.
#[qingke_rt::interrupt]
fn GPIOA() {
    handle_gpioa_interrupt();
}

#[qingke_rt::interrupt]
fn GPIOB() {
    handle_gpiob_interrupt();
}

/// Digital input with interrupt-backed asynchronous wait methods.
pub struct Input<P: GpioPin> {
    inner: crate::hal::gpio::Input<P>,
    port: u8,
    number: u8,
}

impl<P: GpioPin> Input<P> {
    pub fn new(pin: P, pull: Pull) -> Self {
        let port = pin.port();
        let number = pin.number();
        Self {
            inner: crate::hal::gpio::Input::new(pin, pull),
            port,
            number,
        }
    }

    pub fn degrade(self) -> Input<AnyPin>
    where
        P: Into<AnyPin>,
    {
        Input {
            inner: self.inner.degrade(),
            port: self.port,
            number: self.number,
        }
    }

    pub fn wait_for_low(&mut self) -> WaitForInterrupt<'_> {
        WaitForInterrupt::new(self.port, self.number, Trigger::LowLevel)
    }

    pub fn wait_for_high(&mut self) -> WaitForInterrupt<'_> {
        WaitForInterrupt::new(self.port, self.number, Trigger::HighLevel)
    }

    pub fn wait_for_falling_edge(&mut self) -> WaitForInterrupt<'_> {
        WaitForInterrupt::new(self.port, self.number, Trigger::FallingEdge)
    }

    pub fn wait_for_rising_edge(&mut self) -> WaitForInterrupt<'_> {
        WaitForInterrupt::new(self.port, self.number, Trigger::RisingEdge)
    }
}

impl<P: GpioPin> ErrorType for Input<P> {
    type Error = core::convert::Infallible;
}

impl<P: GpioPin> InputPin for Input<P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_high()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.inner.is_low()
    }
}

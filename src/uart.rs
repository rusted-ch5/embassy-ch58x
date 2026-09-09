//! Interrupt-driven UART0..UART3 adapter.
//!
//! Peripheral setup and non-blocking FIFO access live in `ch58x-hal`. This
//! module adds only interrupt binding, task wakeups, and async I/O traits.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::Timer;
use qingke::interrupt::Priority;

use crate::interrupt::typelevel::{Binding, Handler};
use crate::{hal, pac};

pub use hal::uart::{Config, ConfigError, DataBits, Error, Parity, RxPin, StopBits, TxPin};

const RX_INTERRUPT_BITS: u8 = 0b0000_0101;
const TX_INTERRUPT_BIT: u8 = 0b0000_0010;

struct State {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
        }
    }
}

static STATES: [State; 4] = [const { State::new() }; 4];

mod sealed {
    pub trait Sealed {}
}

/// A UART peripheral supported by the async adapter.
pub trait Instance: hal::uart::Instance + sealed::Sealed + Send + 'static {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
    const IRQ: u8;
}

macro_rules! impl_instance {
    ($peripheral:ty, $interrupt:ident) => {
        impl sealed::Sealed for $peripheral {}

        impl Instance for $peripheral {
            type Interrupt = crate::interrupt::typelevel::$interrupt;
            const IRQ: u8 = pac::Interrupt::$interrupt as u8;
        }
    };
}

impl_instance!(pac::UART0, UART0);
impl_instance!(pac::UART1, UART1);
impl_instance!(pac::UART2, UART2);
impl_instance!(pac::UART3, UART3);

fn enable_interrupts<T: Instance>(mask: u8) {
    T::regs()
        .ier()
        .modify(|r, w| unsafe { w.bits(r.bits() | mask) });
}

fn disable_interrupts<T: Instance>(mask: u8) {
    T::regs()
        .ier()
        .modify(|r, w| unsafe { w.bits(r.bits() & !mask) });
}

fn on_interrupt<T: Instance>() {
    let interrupt_id = T::regs().iir().read().int_mask().bits();
    match interrupt_id {
        0x02 => {
            disable_interrupts::<T>(TX_INTERRUPT_BIT);
            STATES[T::INDEX].tx_waker.wake();
        }
        0x04 | 0x06 | 0x0c => {
            disable_interrupts::<T>(RX_INTERRUPT_BITS);
            STATES[T::INDEX].rx_waker.wake();
        }
        _ => {
            // Quiesce an unexpected enabled source before returning from the
            // level-sensitive vector, then let both waiters inspect status.
            disable_interrupts::<T>(RX_INTERRUPT_BITS | TX_INTERRUPT_BIT);
            STATES[T::INDEX].rx_waker.wake();
            STATES[T::INDEX].tx_waker.wake();
        }
    }
}

/// Type-level UART handler used by [`crate::bind_interrupts!`].
pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> Handler<T::Interrupt> for InterruptHandler<T> {
    #[inline(always)]
    unsafe fn on_interrupt() {
        on_interrupt::<T>();
    }
}

struct InterruptGuard<T: Instance> {
    mask: u8,
    _instance: PhantomData<T>,
}

impl<T: Instance> InterruptGuard<T> {
    fn new(mask: u8) -> Self {
        Self {
            mask,
            _instance: PhantomData,
        }
    }
}

impl<T: Instance> Drop for InterruptGuard<T> {
    fn drop(&mut self) {
        disable_interrupts::<T>(self.mask);
    }
}

/// Owned full-duplex UART with interrupt-driven async I/O.
pub struct Uart<T: Instance, TX: TxPin<T>, RX: RxPin<T>> {
    inner: hal::uart::Uart<T, TX, RX>,
}

impl<T, TX, RX> Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    pub fn new(
        peripheral: T,
        tx: TX,
        rx: RX,
        _irq: impl Binding<T::Interrupt, InterruptHandler<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let inner = hal::uart::Uart::new(peripheral, tx, rx, config)?;
        unsafe {
            qingke::pfic::unpend_interrupt(T::IRQ);
            qingke::pfic::set_priority(T::IRQ, Priority::P14.into());
            qingke::pfic::enable_interrupt(T::IRQ);
        }
        Ok(Self { inner })
    }

    /// Actual baud rate produced by the selected integer dividers.
    pub fn actual_baudrate(&self) -> u32 {
        self.inner.actual_baudrate()
    }

    /// Read the bytes currently available, waiting on the UART IRQ if needed.
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Ok(0);
        }

        let _guard = InterruptGuard::<T>::new(RX_INTERRUPT_BITS);
        poll_fn(|cx| {
            match self.inner.try_read(buffer) {
                Ok(count) if count != 0 => return Poll::Ready(Ok(count)),
                Err(error) => return Poll::Ready(Err(error)),
                _ => {}
            }

            STATES[T::INDEX].rx_waker.register(cx.waker());
            enable_interrupts::<T>(RX_INTERRUPT_BITS);

            match self.inner.try_read(buffer) {
                Ok(count) if count != 0 => Poll::Ready(Ok(count)),
                Err(error) => Poll::Ready(Err(error)),
                _ => Poll::Pending,
            }
        })
        .await
    }

    /// Write as many bytes as fit, waiting on the UART IRQ if the FIFO is full.
    pub async fn write(&mut self, buffer: &[u8]) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Ok(0);
        }

        let _guard = InterruptGuard::<T>::new(TX_INTERRUPT_BIT);
        poll_fn(|cx| {
            let count = self.inner.try_write(buffer)?;
            if count != 0 {
                return Poll::Ready(Ok(count));
            }

            STATES[T::INDEX].tx_waker.register(cx.waker());
            enable_interrupts::<T>(TX_INTERRUPT_BIT);

            match self.inner.try_write(buffer) {
                Ok(0) => Poll::Pending,
                result => Poll::Ready(result),
            }
        })
        .await
    }

    async fn wait_for_tx_fifo_empty(&mut self) {
        let _guard = InterruptGuard::<T>::new(TX_INTERRUPT_BIT);
        poll_fn(|cx| {
            if T::regs().lsr().read().tx_fifo_emp().bit_is_set() {
                return Poll::Ready(());
            }

            STATES[T::INDEX].tx_waker.register(cx.waker());
            enable_interrupts::<T>(TX_INTERRUPT_BIT);

            if T::regs().lsr().read().tx_fifo_emp().bit_is_set() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    /// Wait until the FIFO and final shift register are empty.
    pub async fn flush(&mut self) -> Result<(), Error> {
        loop {
            if self.inner.is_tx_idle() {
                return Ok(());
            }

            self.wait_for_tx_fifo_empty().await;
            if self.inner.is_tx_idle() {
                return Ok(());
            }

            // THR_EMPTY signals that the FIFO drained, while TX_ALL_EMP also
            // waits for the final shift register. At most one 12-bit frame is
            // left, so defer the recheck to the Embassy timer instead of
            // repeatedly enabling a level interrupt that is already active.
            let baudrate = u64::from(self.inner.actual_baudrate());
            let frame_micros = 12_000_000u64.div_ceil(baudrate);
            Timer::after_micros(frame_micros).await;
        }
    }
}

impl<T, TX, RX> Drop for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    fn drop(&mut self) {
        disable_interrupts::<T>(RX_INTERRUPT_BITS | TX_INTERRUPT_BIT);
        unsafe {
            qingke::pfic::disable_interrupt(T::IRQ);
            qingke::pfic::unpend_interrupt(T::IRQ);
        }
    }
}

impl<T, TX, RX> embedded_io::ErrorType for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    type Error = Error;
}

impl<T, TX, RX> embedded_io_async::Read for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Uart::read(self, buffer).await
    }
}

impl<T, TX, RX> embedded_io_async::Write for Uart<T, TX, RX>
where
    T: Instance,
    TX: TxPin<T>,
    RX: RxPin<T>,
{
    async fn write(&mut self, buffer: &[u8]) -> Result<usize, Self::Error> {
        Uart::write(self, buffer).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Uart::flush(self).await
    }
}

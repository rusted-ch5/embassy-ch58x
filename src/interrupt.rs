//! CH58x interrupt definitions and binding helpers.

pub use crate::pac::Interrupt;

use crate::pac::Vector;

unsafe extern "C" {
    fn TMR0();
    fn GPIOA();
    fn GPIOB();
    fn SPI0();
    fn USB();
    #[cfg(feature = "ch582")]
    fn USB2();
    fn TMR1();
    fn TMR2();
    fn UART0();
    fn UART1();
    fn RTC();
    fn ADC();
    fn I2C();
    fn PWMX();
    fn TMR3();
    fn UART2();
    fn UART3();
    fn WDOG_BAT();
    fn DefaultHandler();
    #[cfg(feature = "ch585")]
    fn NFC();
    #[cfg(feature = "ch585")]
    fn USB2_DEVICE();
    #[cfg(feature = "ch585")]
    fn USB2_HOST();
    #[cfg(feature = "ch585")]
    fn LED();
}

/// QingKe V4 external interrupt slots 16 through 35.
#[doc(hidden)]
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".vector_table.external_interrupts")]
#[cfg(feature = "ch582")]
pub static __EXTERNAL_INTERRUPTS: [Vector; 20] = [
    Vector { _handler: TMR0 },
    Vector { _handler: GPIOA },
    Vector { _handler: GPIOB },
    Vector { _handler: SPI0 },
    Vector {
        _handler: DefaultHandler,
    },
    Vector {
        _handler: DefaultHandler,
    },
    Vector { _handler: USB },
    Vector { _handler: USB2 },
    Vector { _handler: TMR1 },
    Vector { _handler: TMR2 },
    Vector { _handler: UART0 },
    Vector { _handler: UART1 },
    Vector { _handler: RTC },
    Vector { _handler: ADC },
    Vector { _handler: I2C },
    Vector { _handler: PWMX },
    Vector { _handler: TMR3 },
    Vector { _handler: UART2 },
    Vector { _handler: UART3 },
    Vector { _handler: WDOG_BAT },
];

/// QingKe V3C external interrupt slots 16 through 39 on CH585.
#[doc(hidden)]
#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".vector_table.external_interrupts")]
#[cfg(feature = "ch585")]
pub static __EXTERNAL_INTERRUPTS: [Vector; 24] = [
    Vector { _handler: TMR0 },
    Vector { _handler: GPIOA },
    Vector { _handler: GPIOB },
    Vector { _handler: SPI0 },
    Vector {
        _handler: DefaultHandler,
    },
    Vector {
        _handler: DefaultHandler,
    },
    Vector { _handler: USB },
    Vector {
        _handler: DefaultHandler,
    },
    Vector { _handler: TMR1 },
    Vector { _handler: TMR2 },
    Vector { _handler: UART0 },
    Vector { _handler: UART1 },
    Vector { _handler: RTC },
    Vector { _handler: ADC },
    Vector { _handler: I2C },
    Vector { _handler: PWMX },
    Vector { _handler: TMR3 },
    Vector { _handler: UART2 },
    Vector { _handler: UART3 },
    Vector { _handler: WDOG_BAT },
    Vector { _handler: NFC },
    Vector {
        _handler: USB2_DEVICE,
    },
    Vector {
        _handler: USB2_HOST,
    },
    Vector { _handler: LED },
];

/// Type-level interrupt infrastructure used by Embassy-style driver bindings.
pub mod typelevel {
    use super::Interrupt as PacInterrupt;

    mod sealed {
        pub trait Interrupt {}
    }

    pub trait Interrupt: sealed::Interrupt {
        const IRQ_NUMBER: u8;
    }

    pub trait Handler<I: Interrupt> {
        /// # Safety
        ///
        /// The caller must be the vector entry for exactly `I`.
        unsafe fn on_interrupt();
    }

    /// Compile-time proof that vector `I` invokes handler `H`.
    ///
    /// # Safety
    ///
    /// An implementation promises that every entry of `I` synchronously calls
    /// `H::on_interrupt()` before returning from the interrupt.
    pub unsafe trait Binding<I: Interrupt, H: Handler<I>>: Copy {}

    macro_rules! interrupts {
        ($($name:ident),* $(,)?) => {$ (
            #[allow(non_camel_case_types)]
            pub enum $name {}
            impl sealed::Interrupt for $name {}
            impl Interrupt for $name {
                const IRQ_NUMBER: u8 = PacInterrupt::$name as u8;
            }
        )* };
    }

    interrupts!(
        TMR0, GPIOA, GPIOB, SPI0, USB, USB2, TMR1, TMR2, UART0, UART1, RTC, ADC, I2C, PWMX, TMR3,
        UART2, UART3, WDOG_BAT,
    );

    #[cfg(feature = "ch585")]
    #[allow(non_camel_case_types)]
    pub enum USB2_DEVICE {}
    #[cfg(feature = "ch585")]
    impl sealed::Interrupt for USB2_DEVICE {}
    #[cfg(feature = "ch585")]
    impl Interrupt for USB2_DEVICE {
        const IRQ_NUMBER: u8 = 37;
    }
}

/// Bind CH58x vectors to Embassy-style driver handlers.
#[macro_export]
macro_rules! bind_interrupts {
    ($(#[$attr:meta])* $vis:vis struct $name:ident {
        $(
            $(#[cfg($cond_irq:meta)])?
            $irq:ident => $(
                $(#[cfg($cond_handler:meta)])?
                $handler:ty
            ),*;
        )*
    }) => {
        #[derive(Clone, Copy)]
        $(#[$attr])*
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            $(#[cfg($cond_irq)])?
            #[qingke_rt::interrupt]
            fn $irq() {
                unsafe {
                    $(
                        $(#[cfg($cond_handler)])?
                        <$handler as $crate::interrupt::typelevel::Handler<
                            $crate::interrupt::typelevel::$irq
                        >>::on_interrupt();
                    )*
                }
            }

            $(#[cfg($cond_irq)])?
            $crate::bind_interrupts!(@inner
                $(
                    $(#[cfg($cond_handler)])?
                    unsafe impl $crate::interrupt::typelevel::Binding<
                        $crate::interrupt::typelevel::$irq,
                        $handler
                    > for $name {}
                )*
            );
        )*
    };
    (@inner $($item:item)*) => { $($item)* };
}

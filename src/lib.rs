#![no_std]

#[cfg(all(feature = "ch582", feature = "ch585"))]
compile_error!("embassy-ch58x chip features ch582 and ch585 are mutually exclusive");
#[cfg(not(any(feature = "ch582", feature = "ch585")))]
compile_error!("embassy-ch58x requires exactly one chip feature: ch582 or ch585");

pub use ch58x_hal as hal;
pub use hal::pac;

pub mod executor;
#[cfg(feature = "gpio")]
pub mod gpio;
pub mod interrupt;
#[cfg(feature = "time-driver")]
pub mod time;
#[cfg(feature = "uart")]
pub mod uart;
#[cfg(feature = "usb")]
pub mod usb;

/// Initializes the HAL and Embassy time driver, then returns owned peripherals.
pub fn init(config: hal::sysctl::Config) -> hal::Peripherals {
    let peripherals = hal::init(config);
    #[cfg(feature = "time-driver")]
    time::init();
    peripherals
}

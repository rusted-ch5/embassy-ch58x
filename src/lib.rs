#![no_std]

#[cfg(all(feature = "ch582", feature = "ch585"))]
compile_error!("embassy-ch58x chip features ch582 and ch585 are mutually exclusive");
#[cfg(not(any(feature = "ch582", feature = "ch585")))]
compile_error!("embassy-ch58x requires exactly one chip feature: ch582 or ch585");

pub use ch58x_hal as hal;
pub use hal::pac;

pub mod executor;
pub mod interrupt;

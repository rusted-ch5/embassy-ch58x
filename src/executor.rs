//! CH58x-aware Embassy thread executor.

use core::marker::PhantomData;
use core::ptr;

use embassy_executor::{Spawner, raw};

/// Embassy executor with an interrupt-visible active idle window.
///
/// The generic RISC-V idle path does not match the CH58x PFIC wake-up model.
/// A short active delay keeps global interrupts enabled between task polls.
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    pub fn new() -> Self {
        Self {
            inner: raw::Executor::new(ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            unsafe { self.inner.poll() };
            qingke::riscv::asm::delay(64);
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

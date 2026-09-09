//! CH58x-aware Embassy thread executor.

use core::marker::PhantomData;
use core::ptr::{self, read_volatile, write_volatile};

use embassy_executor::{Spawner, raw};
use portable_atomic::{AtomicBool, Ordering};

const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
const PFIC_SCTLR_SETEVENT: u32 = 1 << 5;
const PFIC_SCTLR_WFITOWFE: u32 = 1 << 3;

// The event latch closes the check-before-sleep race, while this flag avoids
// entering sleep when a task was synchronously queued during `poll()`.
static WORK_PENDING: AtomicBool = AtomicBool::new(false);

#[unsafe(export_name = "__pender")]
fn pender(_context: *mut ()) {
    WORK_PENDING.store(true, Ordering::Release);
    // SETEVENT is a write-one action. Preserve the persistent SCTLR mode bits.
    unsafe {
        let control = read_volatile(PFIC_SCTLR);
        write_volatile(PFIC_SCTLR, control | PFIC_SCTLR_SETEVENT);
    }
}

fn enable_event_sleep() {
    critical_section::with(|_| unsafe {
        let control = read_volatile(PFIC_SCTLR);
        write_volatile(PFIC_SCTLR, control | PFIC_SCTLR_WFITOWFE);
    });
}

/// Embassy executor using the QingKe event latch while idle.
///
/// Task wakeups set both a software flag and the PFIC event latch. Therefore a
/// wakeup racing the final flag check either skips sleep or makes the following
/// WFE-equivalent `wfi` return immediately.
pub struct Executor {
    inner: raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl Executor {
    pub fn new() -> Self {
        enable_event_sleep();
        Self {
            inner: raw::Executor::new(ptr::null_mut()),
            not_send: PhantomData,
        }
    }

    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        loop {
            unsafe { self.inner.poll() };
            if !WORK_PENDING.swap(false, Ordering::AcqRel) {
                // qingke-rt and `enable_event_sleep` set WFITOWFE, so this is
                // event sleep rather than an interrupt-masked RISC-V WFI.
                unsafe { core::arch::asm!("wfi", options(nomem, nostack)) };
            }
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

//! SysTick-backed Embassy time driver.

use core::cell::RefCell;
use core::ptr::{read_volatile, write_volatile};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use qingke::interrupt::Priority;
use qingke_rt::CoreInterrupt;

use crate::pac;

// SysTick is driven from HCLK while Embassy uses one-microsecond ticks. CH582
// runs at exactly 60 MHz; CH585 runs at 62.4 MHz, represented exactly as
// 312/5 counts per tick. Keeping this ratio chip-selected prevents cumulative
// timing drift on CH585.
const SYSTICK_CNT_OFFSET: usize = 0x08;
const SYSTICK_CMP_OFFSET: usize = 0x10;

/// CH582 exposes CNT/CMP as 64-bit registers on an RV32 core. A generated
/// `u64` access becomes two independent word transactions, so a low-word
/// rollover can otherwise produce a mixed counter value.
#[cfg(feature = "ch582")]
fn read_counter_stable() -> u64 {
    let base = pac::SYSTICK::PTR as usize;
    let low = (base + SYSTICK_CNT_OFFSET) as *const u32;
    let high = (base + SYSTICK_CNT_OFFSET + 4) as *const u32;
    loop {
        let high_before = unsafe { read_volatile(high) };
        let low_value = unsafe { read_volatile(low) };
        let high_after = unsafe { read_volatile(high) };
        if high_before == high_after {
            return (u64::from(high_before) << 32) | u64::from(low_value);
        }
    }
}

/// CH585's official `core_riscv.h` defines 32-bit CNT/CMP words with a
/// reserved word after each register. Extend CNT in software. The time queue
/// always arms a rollover compare when its next deadline belongs to a later
/// epoch, so this state is refreshed even when the application has no short
/// timers pending.
#[cfg(feature = "ch585")]
fn read_counter_stable() -> u64 {
    critical_section::with(|cs| {
        let low = unsafe {
            read_volatile((pac::SYSTICK::PTR as usize + SYSTICK_CNT_OFFSET) as *const u32)
        };
        let mut epoch = CH585_COUNTER_EPOCH.borrow(cs).borrow_mut();
        epoch.observe(low)
    })
}

#[cfg(feature = "ch582")]
fn write_counter_register(offset: usize, value: u64) {
    let base = pac::SYSTICK::PTR as usize;
    // Preserve the PAC-generated high-then-low ordering. Callers must first
    // disable the corresponding interrupt/source so the intermediate value
    // cannot become an observable compare match.
    unsafe {
        write_volatile((base + offset + 4) as *mut u32, (value >> 32) as u32);
        write_volatile((base + offset) as *mut u32, value as u32);
    }
}

#[cfg(feature = "ch585")]
fn write_counter_register(offset: usize, value: u64) {
    unsafe {
        write_volatile(
            (pac::SYSTICK::PTR as usize + offset) as *mut u32,
            value as u32,
        );
    }
}

#[cfg(feature = "ch585")]
const fn ch585_alarm_counter(target: u64, now: u64) -> u64 {
    let current_epoch = now >> 32;
    let target_epoch = target >> 32;
    // CMP is only 32 bits on CH585. A deadline in the immediately following
    // epoch can still be armed directly: the low counter must wrap before it
    // can equal the target low word, and the alarm read then advances the
    // software epoch. Avoiding a separate CMP=0 maintenance interrupt also
    // avoids an unnecessary wake-up at rollover.
    //
    // A deadline more than one epoch away would otherwise fire one epoch too
    // early, so retain the rollover maintenance compare for that rare case.
    if target_epoch > current_epoch.saturating_add(1) {
        (current_epoch + 1) << 32
    } else {
        target
    }
}

#[cfg(feature = "ch582")]
const fn counter_to_ticks(counts: u64) -> u64 {
    counts / 60
}

#[cfg(feature = "ch585")]
const fn counter_to_ticks(counts: u64) -> u64 {
    (counts / 312)
        .saturating_mul(5)
        .saturating_add((counts % 312).saturating_mul(5) / 312)
}

#[cfg(feature = "ch582")]
const fn ticks_to_counter_ceil(ticks: u64) -> u64 {
    ticks.saturating_mul(60)
}

#[cfg(feature = "ch585")]
const fn ticks_to_counter_ceil(ticks: u64) -> u64 {
    (ticks / 5)
        .saturating_mul(312)
        .saturating_add((ticks % 5).saturating_mul(312).saturating_add(4) / 5)
}

#[cfg(feature = "ch582")]
const _: () = {
    assert!(counter_to_ticks(60_000_000) == 1_000_000);
    assert!(ticks_to_counter_ceil(1_000_000) == 60_000_000);
};

#[cfg(feature = "ch585")]
const _: () = {
    assert!(counter_to_ticks(62_400_000) == 1_000_000);
    assert!(ticks_to_counter_ceil(1_000_000) == 62_400_000);
    assert!(ticks_to_counter_ceil(30_000) == 1_872_000);
    assert!(ticks_to_counter_ceil(1) == 63);
};

fn clear_systick_pending() {
    let systick = unsafe { &*pac::SYSTICK::PTR };
    // CH58x clears CNTIF by writing zero. Clear both the peripheral source and
    // the already-latched PFIC pending state before installing a new compare.
    systick.sr().write(|w| w.cntif().clear_bit());
    unsafe { qingke::pfic::unpend_interrupt(CoreInterrupt::SysTick as u8) };
}

#[inline(always)]
fn diagnostic_set_alarm(_at: u64, _now: u64) {}

#[inline(always)]
fn diagnostic_alarm_past() {}

#[inline(always)]
fn diagnostic_armed(_at: u64) {}

#[inline(always)]
fn diagnostic_next(_next: u64) {}

#[inline(always)]
fn diagnostic_schedule(_changed: bool) {}

#[inline(always)]
fn diagnostic_irq() {}

struct TimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

#[cfg(feature = "ch585")]
#[derive(Clone, Copy)]
struct Ch585CounterEpoch {
    last_low: u32,
    high: u32,
}

#[cfg(feature = "ch585")]
impl Ch585CounterEpoch {
    const fn observe(&mut self, low: u32) -> u64 {
        if low < self.last_low {
            self.high = self.high.wrapping_add(1);
        }
        self.last_low = low;
        ((self.high as u64) << 32) | low as u64
    }
}

#[cfg(feature = "ch585")]
const _: () = {
    let mut epoch = Ch585CounterEpoch {
        last_low: 0xffff_fffe,
        high: 7,
    };
    assert!(epoch.observe(0xffff_ffff) == 0x0000_0007_ffff_ffff);
    assert!(epoch.observe(0) == 0x0000_0008_0000_0000);
    assert!(epoch.observe(1) == 0x0000_0008_0000_0001);
    assert!(
        ch585_alarm_counter(0x0000_0002_0000_1234, 0x0000_0001_ffff_0000) == 0x0000_0002_0000_1234
    );
    assert!(
        ch585_alarm_counter(0x0000_0003_0000_1234, 0x0000_0001_ffff_0000) == 0x0000_0002_0000_0000
    );
    assert!(
        ch585_alarm_counter(0x0000_0001_ffff_1234, 0x0000_0001_ffff_0000) == 0x0000_0001_ffff_1234
    );
};

#[cfg(feature = "ch585")]
static CH585_COUNTER_EPOCH: Mutex<RefCell<Ch585CounterEpoch>> =
    Mutex::new(RefCell::new(Ch585CounterEpoch {
        last_low: 0,
        high: 0,
    }));

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl TimeDriver {
    fn set_alarm(&self, at: u64) -> bool {
        let systick = unsafe { &*pac::SYSTICK::PTR };
        let now = self.now();
        diagnostic_set_alarm(at, now);
        systick.ctlr().modify(|_, w| w.stie().clear_bit());
        if at <= now {
            diagnostic_alarm_past();
            clear_systick_pending();
            return false;
        }

        // Wake one counter increment after the requested Embassy tick so an
        // integer division boundary cannot make the timer fire early. STIE is
        // disabled across both RV32 word writes so the transient mixed CMP
        // value cannot latch a spurious SysTick.
        let target_compare = ticks_to_counter_ceil(at).saturating_add(1);
        #[cfg(feature = "ch582")]
        let compare = target_compare;
        #[cfg(feature = "ch585")]
        let compare = ch585_alarm_counter(target_compare, read_counter_stable());
        write_counter_register(SYSTICK_CMP_OFFSET, compare);
        clear_systick_pending();
        if at <= self.now() {
            diagnostic_alarm_past();
            return false;
        }
        diagnostic_armed(at);
        systick.ctlr().modify(|_, w| w.stie().set_bit());
        true
    }

    fn dispatch(&self) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let mut next = queue.next_expiration(self.now());
            diagnostic_next(next);
            while !self.set_alarm(next) {
                next = queue.next_expiration(self.now());
                diagnostic_next(next);
            }
        });
    }
}

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        counter_to_ticks(read_counter_stable())
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let changed = queue.schedule_wake(at, waker);
            diagnostic_schedule(changed);
            if changed {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(next) {
                    next = queue.next_expiration(self.now());
                }
            }
        });
    }
}

// qingke-rt's wrapper places the vector entry in `.trap`, saves `ra`, and
// returns with `mret`. A plain C `ret` handles the first interrupt but corrupts
// the machine-interrupt return state, preventing subsequent interrupts.
#[qingke_rt::interrupt]
fn SysTick() {
    diagnostic_irq();
    let systick = unsafe { &*pac::SYSTICK::PTR };
    systick.ctlr().modify(|_, w| w.stie().clear_bit());
    // CH58x clears CNTIF by writing zero.
    systick.sr().write(|w| w.cntif().clear_bit());
    DRIVER.dispatch();
}

pub(crate) fn init() {
    let systick = unsafe { &*pac::SYSTICK::PTR };
    systick
        .ctlr()
        .write(|w| w.stie().clear_bit().ste().clear_bit());
    #[cfg(feature = "ch585")]
    critical_section::with(|cs| {
        *CH585_COUNTER_EPOCH.borrow(cs).borrow_mut() = Ch585CounterEpoch {
            last_low: 0,
            high: 0,
        };
    });
    write_counter_register(SYSTICK_CNT_OFFSET, 0);
    write_counter_register(SYSTICK_CMP_OFFSET, u64::MAX);
    clear_systick_pending();
    systick.ctlr().write(|w| {
        w.stclk()
            .hclk()
            .mode()
            .upcount()
            .stre()
            .clear_bit()
            .init()
            .set_bit()
            .ste()
            .set_bit()
    });

    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15.into());
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}

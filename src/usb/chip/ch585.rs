use core::ptr::{read_volatile, write_volatile};

const USB_BASE: usize = 0x4000_8000;
const PIN_CONFIG: *mut u16 = 0x4000_101a as *mut u16;
const UDP_PULLUP_ENABLE: u16 = 0x0040;
const PIN_USB_ENABLE: u16 = 0x0080;
const SRAM_START: usize = 0x2000_0000;
const SRAM_END: usize = 0x2002_0000;

pub(super) fn write_endpoint_dma(index: usize, address: usize) {
    assert!(index <= 3);
    // CH585 exposes a 32-bit register, but CH585DS1 defines only bits 16:0 as
    // the offset inside its 128 KiB SRAM; bits 31:17 are reserved.
    assert!((SRAM_START..SRAM_END).contains(&address));
    assert_eq!(address & 3, 0);
    let register = (USB_BASE + 0x10 + index * 4) as *mut u32;
    unsafe { write_volatile(register, (address - SRAM_START) as u32) };
}

pub(super) fn enable_usb_pins() {
    // Official CH585 USBFS device setup sets both RB_PIN_USB_EN and
    // RB_UDP_PU_EN in R16_PIN_CONFIG before exposing the port.
    unsafe {
        write_volatile(
            PIN_CONFIG,
            read_volatile(PIN_CONFIG) | UDP_PULLUP_ENABLE | PIN_USB_ENABLE,
        )
    };
}

pub(super) fn disable_usb_pullup() {
    unsafe { write_volatile(PIN_CONFIG, read_volatile(PIN_CONFIG) & !UDP_PULLUP_ENABLE) };
}

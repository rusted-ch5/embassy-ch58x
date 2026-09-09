use core::ptr::{read_volatile, write_volatile};

const USB_BASE: usize = 0x4000_8000;
const PIN_ANALOG_IE: *mut u16 = 0x4000_101a as *mut u16;
const PIN_USB_DP_PULLUP: u16 = 0x0040;
const PIN_USB_ANALOG_ENABLE: u16 = 0x0080;

pub(super) fn write_endpoint_dma(index: usize, address: usize) {
    assert!(index <= 3);
    // CH581/2/3 encode only the offset inside the low 64 KiB SRAM bank.
    assert_eq!(address & 0xffff_0000, 0x2000_0000);
    let register = (USB_BASE + 0x10 + index * 4) as *mut u16;
    unsafe { write_volatile(register, address as u16) };
}

pub(super) fn enable_usb_pins() {
    unsafe {
        write_volatile(
            PIN_ANALOG_IE,
            read_volatile(PIN_ANALOG_IE) | PIN_USB_DP_PULLUP | PIN_USB_ANALOG_ENABLE,
        )
    };
}

pub(super) fn disable_usb_pullup() {
    unsafe {
        write_volatile(
            PIN_ANALOG_IE,
            read_volatile(PIN_ANALOG_IE) & !PIN_USB_DP_PULLUP,
        )
    };
}

//! Chip-selected USBFS register differences.

#[cfg(feature = "ch582")]
mod ch582;
#[cfg(feature = "ch585")]
mod ch585;

pub(super) fn write_endpoint_dma(index: usize, address: usize) {
    #[cfg(feature = "ch582")]
    ch582::write_endpoint_dma(index, address);
    #[cfg(feature = "ch585")]
    ch585::write_endpoint_dma(index, address);
}

pub(super) fn enable_usb_pins() {
    #[cfg(feature = "ch582")]
    ch582::enable_usb_pins();
    #[cfg(feature = "ch585")]
    ch585::enable_usb_pins();
}

pub(super) fn disable_usb_pullup() {
    #[cfg(feature = "ch582")]
    return ch582::disable_usb_pullup();
    #[cfg(feature = "ch585")]
    return ch585::disable_usb_pullup();
}

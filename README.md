# embassy-ch58x

Embassy async runtime support for WCH CH58x microcontrollers.

The crates.io package is `embassy-ch58x-rs`; its Rust library name remains
`embassy_ch58x`.

The crate currently provides:

- CH582 and CH585 interrupt vector tables;
- type-safe interrupt binding for Embassy drivers;
- a thread-mode Embassy executor with race-free event sleep;
- an optional one-microsecond SysTick time driver;
- interrupt-backed asynchronous GPIO input waits;
- interrupt-driven UART with cancellation-safe async I/O;
- an interrupt-driven USBFS device driver with cancellation-safe endpoint
  transfers;
- access to the rusted-ch5 `ch58x-hal-rs` package through
  `embassy_ch58x::hal`.

## Status

The crate is under active development. APIs may change before the first stable
release.

## License

Licensed under either of
[Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at
your option.

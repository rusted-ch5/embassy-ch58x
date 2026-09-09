# embassy-ch58x

Embassy async runtime support for WCH CH58x microcontrollers.

The crate currently provides:

- CH582 and CH585 interrupt vector tables;
- type-safe interrupt binding for Embassy drivers;
- a thread-mode Embassy executor with race-free event sleep;
- an optional one-microsecond SysTick time driver;
- interrupt-backed asynchronous GPIO input waits;
- access to `ch58x-hal` through `embassy_ch58x::hal`.

## Status

The crate is under active development. APIs may change before the first stable
release.

## License

Licensed under either of
[Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at
your option.

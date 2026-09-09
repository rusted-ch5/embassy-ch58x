use std::path::PathBuf;

fn main() {
    let output = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
    std::fs::copy("device.x", output.join("device.x")).unwrap();
    println!("cargo:rustc-link-search={}", output.display());
    println!("cargo:rerun-if-changed=device.x");
}

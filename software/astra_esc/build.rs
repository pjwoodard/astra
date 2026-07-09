use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Pick the linker memory layout for the selected MCU backend so the linker
    // finds it as `memory.x`. Cargo sets CARGO_FEATURE_<FEATURE> for each
    // enabled feature (dashes -> underscores, upper-cased).
    let mem = if env::var_os("CARGO_FEATURE_MCU_H5").is_some() {
        "memory-h5.x"
    } else {
        "memory-g4.x"
    };
    fs::copy(mem, out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-g4.x");
    println!("cargo:rerun-if-changed=memory-h5.x");

    // Linker scripts, emitted here rather than as per-target rustflags in
    // .cargo/config.toml so they apply to whichever target is being built
    // (G4 or H5) without duplicating the two -T flags across per-target
    // sections. (The .cargo/config.toml target sections still carry the
    // per-board probe-rs runner.)
    println!("cargo:rustc-link-arg=-Tlink.x"); // cortex-m-rt
    println!("cargo:rustc-link-arg=-Tdefmt.x"); // defmt-rtt
}

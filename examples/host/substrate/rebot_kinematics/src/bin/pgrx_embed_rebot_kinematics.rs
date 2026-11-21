//! Support binary for `cargo pgrx schema` / `cargo pgrx install` SQL generation.
//!
//! Not meant to be run directly. cargo-pgrx rebuilds this bin with `--cfg
//! pgrx_embed` and `PGRX_EMBED` pointing at a generated source file that defines
//! `main` (it links the extension rlib and emits the CREATE FUNCTION SQL). The
//! bin name must be `pgrx_embed_<package_name>` for cargo-pgrx to find it.
#[cfg(pgrx_embed)]
include!(env!("PGRX_EMBED"));

#[cfg(not(pgrx_embed))]
fn main() {
    panic!("pgrx_embed_rebot_kinematics is only built by `cargo pgrx schema`");
}
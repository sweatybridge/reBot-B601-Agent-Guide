# syntax=docker/dockerfile:1
#
# Cross-build the single edge binary for the board.
#
# Default target is ARMv6 hard-float static musl (Raspberry Pi Zero W, 32-bit
# Raspberry Pi OS). The binary is statically linked, so it has no glibc or
# Python/numpy dependency on the board.
#
#   docker build --target bin --output dist examples/edge-rust
#   ls dist/rebot-edge
#
# Other boards:
#   --build-arg TARGET=aarch64-unknown-linux-musl         (64-bit Pi OS)
#   --build-arg TARGET=armv7-unknown-linux-musleabihf     (32-bit Pi 2/3/4)
#   --build-arg TARGET=x86_64-unknown-linux-musl          (workstation / CI)

ARG RUST_VERSION=1.85
ARG TARGET=arm-unknown-linux-musleabihf

FROM rust:${RUST_VERSION}-slim-bookworm AS build
ARG TARGET
WORKDIR /src
RUN rustup target add "${TARGET}"
COPY Cargo.toml Cargo.lock ./
COPY .cargo ./.cargo
COPY src ./src
COPY testdata ./testdata
# musl targets are self-contained: rustc links its bundled libc with rust-lld,
# so no cross C toolchain is needed in this image.
RUN cargo build --release --target "${TARGET}" --locked

# `docker build --target test .` runs the unit tests on the host architecture.
# The tests include byte-for-byte golden vectors from the Python MitSender.
FROM build AS test
ARG HOST_TARGET=x86_64-unknown-linux-musl
RUN rustup target add "${HOST_TARGET}" \
    && cargo test --release --target "${HOST_TARGET}" --locked

# A runnable image for `--check` on a workstation (x86_64 Linux container):
#   docker run --rm -v "$PWD/examples/trajectories:/data:ro" rebot-edge-host \
#       --check /data/seq_pino_peel.json
FROM build AS hostrun
ARG HOST_TARGET=x86_64-unknown-linux-musl
RUN rustup target add "${HOST_TARGET}" \
    && cargo build --release --target "${HOST_TARGET}" --locked \
    && cp "target/${HOST_TARGET}/release/rebot-edge" /usr/local/bin/rebot-edge
ENTRYPOINT ["/usr/local/bin/rebot-edge"]

# The deliverable: a single static executable, no shell, no libc.
FROM scratch AS bin
ARG TARGET
COPY --from=build /src/target/${TARGET}/release/rebot-edge /rebot-edge

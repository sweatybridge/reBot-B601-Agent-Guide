# syntax=docker/dockerfile:1
# Multi-stage build. pg_durable itself is NOT compiled here — we use Microsoft's
# prebuilt pg18 image (released .deb on official postgres:18). Only the openrr/k
# kinematics wrapper (rebot_kinematics) is built from source, as a pgrx extension.
#
# Build:  docker build -t rebot-substrate:pg18 .
# Run:    docker compose up -d   (see docker-compose.yml)

FROM ghcr.io/microsoft/pg_durable:pg18 AS builder
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      postgresql-server-dev-18 build-essential curl ca-certificates pkg-config \
    && rm -rf /var/lib/apt/lists/*
# Rust toolchain + the cargo-pgrx version pg_durable pins (PG17/18 support).
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
      | sh -s -- -y --profile minimal --default-toolchain stable
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install --locked cargo-pgrx --version 0.16.1
WORKDIR /build/rebot_kinematics
COPY rebot_kinematics/ ./
# Build against the image's own PG18 so the .so matches the runtime exactly.
RUN cargo pgrx init --pg18 /usr/lib/postgresql/18/bin/pg_config
RUN cargo pgrx install --release

FROM ghcr.io/microsoft/pg_durable:pg18
USER root
# pgvector for the similarity index; pg_durable is already preloaded in this image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      postgresql-18-pgvector \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/lib/postgresql/18/lib/rebot_kinematics.so \
     /usr/lib/postgresql/18/lib/
COPY --from=builder /usr/share/postgresql/18/extension/rebot_kinematics* \
     /usr/share/postgresql/18/extension/
USER postgres
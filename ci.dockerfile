FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git curl ca-certificates \
    build-essential pkg-config cmake \
    python3 python3-dev python3-venv \
    libegl1 libgles2 libgl1-mesa-dri libgbm1 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.cargo/bin:${PATH}"

ENV MUJOCO_GL=egl

WORKDIR /workspace

COPY pyproject.toml uv.lock ./
RUN uv sync --all-extras --all-groups --frozen --no-install-project

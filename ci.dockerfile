FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git curl ca-certificates \
    build-essential pkg-config cmake \
    python3 python3-dev python3-venv \
    libegl1 libgles2 libgl1-mesa-dri libgbm1 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

ENV MUJOCO_GL=egl

WORKDIR /workspace

RUN --mount=type=cache,target=/root/.cache/uv \
    uv venv && \
    uv pip install z3-solver==4.15.7.0 genesis-world@git+https://github.com/Genesis-Embodied-AI/Genesis@5a8928a5e1b3d9e668bb2381ded44ef64357afb2

COPY pyproject.toml uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --all-extras --all-groups --frozen --no-install-project

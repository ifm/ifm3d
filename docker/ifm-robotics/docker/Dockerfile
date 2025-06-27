# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# Set the Python version to use
ARG PYTHON_VERSION=3.13

# ---------------------------
# Stage 1: Builder base image
# ---------------------------
FROM ${BASE_IMAGE} AS builder

# Install build dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        gnupg2 \
        wget \
        curl \
        libssl-dev \
        sudo \
        locales \
        ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m -u 30000 -s /bin/bash ifm && \
    echo "ifm ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ifm

USER ifm
WORKDIR /home/ifm

# Install uv and Python via uv
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

ENV PATH="/home/ifm/.local/bin:/home/ifm/venv/bin:$PATH"
ENV VIRTUAL_ENV=/home/ifm/venv

RUN uv python install ${PYTHON_VERSION} && \
    uv venv --python="${PYTHON_VERSION}" --seed $VIRTUAL_ENV && \
    pip install --upgrade pip setuptools wheel

RUN echo 'source $VIRTUAL_ENV/bin/activate' >> ~/.bashrc
# ---------------------------
# Stage 2: Final runtime image
# ---------------------------
FROM ${BASE_IMAGE} AS final

# Install runtime dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        sudo \
        locales \
        dpkg \
        ca-certificates && \
    echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

ENV VIRTUAL_ENV=/home/ifm/venv
ENV PATH="/home/ifm/venv/bin:$PATH"

# Copy home directory (includes Python & virtualenv setup)
COPY --from=builder /home/ifm /home/ifm

# Create the ifm user again in final stage
RUN useradd -m -u 30000 -s /bin/bash ifm && \
    echo "ifm ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/ifm && \
    chmod 0440 /etc/sudoers.d/ifm

# Ensure correct ownership
RUN chown -R ifm:ifm /home/ifm

ARG BASE_IMAGE
ENV BASE_IMAGE=${BASE_IMAGE}

# ---------------------------
# Install .deb packages
# ---------------------------
COPY ./ifm3d*.deb /home/ifm/tmp/
RUN BASE_IMAGE_NAME=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f1) \
    && BASE_IMAGE_VERSION=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f2) \
    && ARCH=$(dpkg --print-architecture) \
    && cd /home/ifm/tmp/ \
    && dpkg -i ifm3d-${BASE_IMAGE_NAME}-${BASE_IMAGE_VERSION}-${ARCH}-*.deb \
    # && dpkg -i ifm3dpy-${BASE_IMAGE_NAME}-${BASE_IMAGE_VERSION}-${ARCH}-*.deb \
    && rm -rf /home/ifm/tmp 

# ---------------------------
# Install .whl packages
# ---------------------------
COPY ./filtered_wheels /filtered_wheels
RUN  /home/ifm/venv/bin/pip install --no-deps /filtered_wheels/*.whl

USER ifm
WORKDIR /home/ifm

ENTRYPOINT ["/bin/bash"]
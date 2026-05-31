FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    cmake \
    make \
    git \
    python3 \
    python3-pip \
    libc6-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /cfs

CMD ["/bin/bash"]

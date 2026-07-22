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

# Short single-line prompt (matches host Git Bash prompt), instead of the
# default root@<container-id>:/path# banner.
RUN echo "PS1='\W \$ '" > /root/.bashrc

WORKDIR /cfs

CMD ["/bin/bash"]

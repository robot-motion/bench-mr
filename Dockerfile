FROM ubuntu:18.04

ENV LANG C.UTF-8

RUN \
  apt-get -y -q update && \
  # Prevents debconf from prompting for user input
  # See https://github.com/phusion/baseimage-docker/issues/58
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    gcc \
    g++ \
    build-essential \
    wget \
    curl \
    unzip \
    git \
    git-lfs \
    python-dev \
    autotools-dev \
    m4 \
    libicu-dev \
    build-essential \
    libbz2-dev \
    libasio-dev \
    libeigen3-dev \
    freeglut3-dev \
    expat \
    libcairo2-dev \
    cmake \
    libboost-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    nlohmann-json-dev \
    python3-pip \
    ffmpeg \
    libhdf5-dev \
    nano \
    htop

# Set up Node.js 10.x repo
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash -
RUN \
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    nodejs

RUN mkdir -p /root/code

# Copy mpb folder
COPY . /root/code/mpb
WORKDIR /root/code/mpb

RUN pip3 install -r plotting/requirements.txt

# Install ipywidgets for Jupyter Lab
RUN jupyter labextension install @jupyter-widgets/jupyterlab-manager

# Build and install libccd 1.4
WORKDIR /root/code
RUN wget https://github.com/danfis/libccd/archive/v1.4.zip && unzip v1.4.zip && cd libccd-1.4/src && echo "#define CCD_FLOAT" | cat - ccd/vec3.h > /tmp/out && mv /tmp/out ccd/vec3.h && make -j4 && make install

# Check out git submodules
WORKDIR /root/code/mpb
RUN git submodule init && git submodule update

# Build and install SBPL
WORKDIR /root/code
RUN git clone https://github.com/sbpl/sbpl.git && cd sbpl && mkdir build && cd build && cmake .. && make -j4 && make install

# Build and install OMPL
WORKDIR /root/code/mpb
RUN cd ompl && mkdir build && cd build && cmake .. && make -j4 && make install

# Creating Build Files
WORKDIR /root/code/mpb
RUN rm -rf build && cmake -H. -Bbuild

# Build mpb
RUN cd build && make

# Run benchmark executable to generate benchmark_template.json
WORKDIR /root/code/mpb/bin
RUN ./benchmark; exit 0

# Set up git lfs
RUN git lfs install

# Setup repo
WORKDIR /root/code/mpb

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

EXPOSE 8888

ENTRYPOINT ["jupyter", "lab", "--ip=0.0.0.0", "--allow-root", "--no-browser", "--NotebookApp.token=''"]
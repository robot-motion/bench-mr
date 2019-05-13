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
    libhdf5-dev

RUN mkdir -p /root/code

RUN python3 -m pip install \
    'ipywidgets==7.4.0' \
    'pandas==0.24.0' \
    'numpy==1.16.0' \
    'numexpr==2.6.0' \
    'matplotlib==3.0.0' \
    'scipy==1.2.0' \
    'seaborn==0.9.0' \
    'scikit-learn==0.20.0' \
    'scikit-image==0.14.0' \
    'sympy==1.3.0' \
    'cython==0.29.0' \
    'patsy==0.5.0' \
    'statsmodels==0.9.0' \
    'cloudpickle==0.8.0' \
    'dill==0.2.0' \
    'dask==1.1.0' \
    'numba==0.42.0' \
    'bokeh==1.0.0' \
    'sqlalchemy==1.3.0' \
    'h5py==2.9.0' \
    'vincent==0.4.0' \
    'beautifulsoup4==4.7.0' \
    'protobuf==3.7.0' \
    'xlrd' \
    'jupyterlab' \
    'click' \
    'bitarray==0.8.3'

# Copy mpb folder
COPY . /root/code/mpb

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

# Install Python requirements for plotting
WORKDIR /root/code/mpb/plotting
RUN pip3 install -r requirements.txt

# Set up git lfs
RUN git lfs install

# Setup repo
WORKDIR /root/code/mpb

EXPOSE 8888

ENTRYPOINT ["jupyter", "lab", "--ip=0.0.0.0", "--allow-root"]
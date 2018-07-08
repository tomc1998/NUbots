ARG UBUNTU_VERSION
FROM ubuntu:${UBUNTU_VERSION}

ARG TOOLCHAIN_VERSION
ARG CMAKE_VERSION
ARG CMAKE_VERSION_SHORT
ARG UBUNTU_NAME

VOLUME /NUbots

WORKDIR /tmp

RUN echo "###############################" \
  ; echo "# Installing APT Repositories #" \
  ; echo "###############################" \
  ; apt update \
  ; apt install -y software-properties-common wget \
  ; apt-add-repository -y ppa:ubuntu-toolchain-r/test \
 && wget -O - "http://apt.llvm.org/llvm-snapshot.gpg.key" | apt-key add - \
 && apt-add-repository -y "deb http://apt.llvm.org/$UBUNTU_NAME/ llvm-toolchain-$UBUNTU_NAME-6.0 main" \
 && apt update

RUN echo "###############################" \
  ; echo "#   Installing APT Packages   #" \
  ; echo "###############################" \
  ; apt install -y clang-format-6.0 \
                   colordiff \
                   unzip \
                   automake \
                   autoconf \
                   libtool \
                   intltool \
                   gtk-doc-tools \
                   texinfo \
                   bison \
                   libpcre3-dev \
                   pkg-config \
                   linux-headers-generic \
                   rsync \
                   git \
                   build-essential \
                   libncurses5-dev \
                   libstdc++6 \
                   gcc-7 \
                   g++-7 \
                   gfortran-7 \
                   ccache \
                   binutils \
                   binutils-dev \
                   ninja-build \
                   nasm \
                   libusb-1.0-0 \
                   libusb-1.0-0-dev \
                   autopoint \
                   gettext \
                   python-pip \
                   python3-pip \
                   zlib1g-dev \
                   libjpeg-turbo8-dev \
                   gcc-arm-none-eabi \
                   libnewlib-arm-none-eabi \
                   libasound2-dev


RUN echo "###############################" \
  ; echo "# Installing Python Libraries #" \
  ; echo "###############################" \
  ; pip3 install pyparsing \
 && pip3 install pydotplus \
 && pip3 install pygments \
 && pip3 install stringcase \
 && pip3 install termcolor \
 && pip3 install protobuf==3.5.0.post1 \
 && pip3 install pillow \
 && pip3 install xxhash

RUN  echo "###############################" \
  ;  echo "#   Setting up GCC Defaults   #" \
  ;  echo "###############################" \
  ;  /usr/bin/update-alternatives --remove-all gcc \
  ;  /usr/bin/update-alternatives --remove-all g++ \
  ;  /usr/bin/update-alternatives --remove-all gfortan \
  ;  /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
  && /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
  && /usr/bin/update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
                                  --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
                                  --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7


# Only download cmake if it either isn't installed already
RUN  echo "###############################" \
  ;  echo "#      Installing CMake       #" \
  ;  echo "###############################" \
  ;  cmake_installed_version=$(cmake --version 2>&1 | head -n1 | cut -d' ' -f3) \
  && if [ "x${cmake_installed_version}" != "x$CMAKE_VERSION" ]; \
     then \
         major_version=$(echo "$CMAKE_VERSION" | cut -d'.' -f1,2) \
         && wget "https://cmake.org/files/v${major_version}/cmake-$CMAKE_VERSION-Linux-x86_64.sh" \
         && /bin/sh "cmake-$CMAKE_VERSION-Linux-x86_64.sh" --prefix=/usr/local --exclude-subdir; \
     fi
COPY "puppet/modules/files/files/FindBoost.cmake" "/usr/local/share/cmake-$CMAKE_VERSION_SHORT/Modules/FindBoost.cmake"

RUN  echo "###############################" \
  ;  echo "#   Installing Intel OpenCL   #" \
  ;  echo "###############################" \
  ;  if [ ! -e "/opt/intel/opencl/libOpenCL.so" ]; \
     then \
         mkdir intel-opencl \
         && cd intel-opencl \
         && wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip \
         && unzip SRB5.0_linux64.zip \
         && mkdir root \
         && for i in *.tar.xz; do tar -C root -xf "${i}"; done \
         && cp -r root/* / \
         && ldconfig; \
     fi

# Only download the toolchain if it either isn't installed already or we want a different version
RUN  echo "###############################" \
  ;  echo "#     Installing Toolchain    #" \
  ;  echo "###############################" \
  ;  toolchain_installed_version=$(dpkg -l | grep '^ii' | grep nubots-toolchain | awk '{print $3}') \
  && if [ "x${toolchain_installed_version}" != "x$TOOLCHAIN_VERSION" ]; \
     then \
         # Remove (and purge) the toolchain if the wrong version is installed
         if [ "x${toolchain_installed_version}" != "x" ]; \
         then \
             dpkg -P nubots-toolchain; \
         fi; \
         # Download and install the correct version
         wget -N "http://nubots.net/debs/nubots-toolchain-$TOOLCHAIN_VERSION.deb" \
         && dpkg -i "nubots-toolchain-$TOOLCHAIN_VERSION.deb"; \
     fi

WORKDIR /NUbots

#! /bin/bash

set -eu

echo "###############################"
echo "# Installing APT Repositories #"
echo "###############################"
sudo apt-add-repository -y ppa:ubuntu-toolchain-r/test
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
sudo apt-add-repository -y "deb http://apt.llvm.org/$(lsb_release -c)/ llvm-toolchain-$(lsb_release -c)-6.0 main"
sudo apt update

echo "###############################"
echo "#   Installing APT Packages   #"
echo "###############################"
sudo apt install -y software-properties-common \
                    clang-format-6.0 \
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


echo "###############################"
echo "# Installing Python Libraries #"
echo "###############################"
sudo pip3 install pyparsing
sudo pip3 install pydotplus
sudo pip3 install pygments
sudo pip3 install stringcase
sudo pip3 install termcolor
sudo pip3 install protobuf==3.5.0.post1
sudo pip3 install pillow
sudo pip3 install xxhash

echo "###############################"
echo "#   Setting up GCC Defaults   #"
echo "###############################"
sudo /usr/bin/update-alternatives --remove-all gcc \
    ;  sudo /usr/bin/update-alternatives --remove-all g++ \
    ;  sudo /usr/bin/update-alternatives --remove-all gfortan \
    ;  sudo /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
    && sudo /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
    && sudo /usr/bin/update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
                                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
                                         --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7

mkdir -p artifacts
cd artifacts

echo "###############################"
echo "#   Installing CMake v3.5.1   #"
echo "###############################"
/usr/bin/wget https://cmake.org/files/v3.5/cmake-3.5.1-Linux-x86_64.sh
/bin/sh cmake-3.5.1-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir

echo "###############################"
echo "#   Installing Intel OpenCL   #"
echo "###############################"
mkdir intel-opencl
cd intel-opencl
wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
unzip SRB5.0_linux64.zip
mkdir root
for i in *.tar.xz; do tar -C root -xf "$i"; done
sudo cp -r root/* /
sudo ldconfig

echo "###############################"
echo "#     Installing Toolchain    #"
echo "###############################"
wget -N http://nubots.net/debs/nubots-toolchain-3.0.1-travis.deb
sudo dpkg -i nubots-toolchain-3.0.1-travis.deb

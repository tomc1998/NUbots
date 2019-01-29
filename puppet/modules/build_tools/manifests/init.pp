class build_tools {

  # Update apt before getting any packages (if we need to)
  exec { "apt-update":
    command => "/usr/bin/apt-key update && /usr/bin/apt-get update",
    onlyif => "/bin/sh -c '[ ! -f /var/cache/apt/pkgcache.bin ] || /usr/bin/find /etc/apt/* -cnewer /var/cache/apt/pkgcache.bin | /bin/grep . > /dev/null'",
  } ->
  # We also need to ensure software-properties is installed for apt-add-repository
  exec { "install-software-properties":
    command => "/usr/bin/apt-get install -y software-properties-common",
    unless => '/usr/bin/dpkg -s software-properties-common',
  } ->
  # Add the ubuntu test toolchain ppa (for modern g++ etc)
  apt::ppa {'ppa:ubuntu-toolchain-r/test': } ~>
  exec { "apt-update-ppa":
    command => "/usr/bin/apt-get update",
    refreshonly => true
  } -> Package <| |>

  $codename = lsb_release()

  # Add the llvm 6.0 source
  apt::source { 'llvm-apt-repo':
    comment  => 'The LLVM 6.0 apt repository',
    location => "http://apt.llvm.org/${codename}",
    release  => "llvm-toolchain-${codename}-6.0",
    repos    => 'main',
    key      => {
      'id'      => '6084F3CF814B57C1CF12EFD515CF4D18AF4F7421',
      'source'  => 'https://apt.llvm.org/llvm-snapshot.gpg.key'
    },
    include  => {
      'src' => true,
      'deb' => true,
    },
  } -> Package <| |>

  # Tools
  package { 'unzip': ensure => latest, }
  package { 'automake': ensure => latest, }
  package { 'autoconf': ensure => latest, }
  package { 'libtool': ensure => latest, }
  package { 'intltool': ensure => latest, }
  package { 'gtk-doc-tools': ensure => latest, }
  package { 'texinfo': ensure => latest, }
  package { 'bison': ensure => latest, }
  package { 'libpcre3-dev': ensure => latest, }
  package { 'pkg-config': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'git': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'libncurses5-dev': ensure => latest, require => [ Package['gcc-7'], Package['g++-7'], ], }
  package { 'libstdc++6': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gcc-7': name => 'gcc-7', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'g++-7': name => 'g++-7', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gfortran-7': name => 'gfortran-7', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'ccache': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils': name => 'binutils', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils-dev': name => 'binutils-dev', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'ninja-build': ensure => latest, }
  package { 'nasm': ensure => latest, }
  package { 'libusb-1.0-0': ensure => latest, }
  package { 'libusb-1.0-0-dev': ensure => latest, }
  package { 'autopoint': ensure => latest, }
  package { 'gettext': ensure => latest, }
  package { 'python-pip': ensure => latest, }
  package { 'python3-pip': ensure => latest, }
  package { 'zlib1g-dev': ensure => latest, }
  package { 'libjpeg-turbo8-dev': ensure => latest, }

  package { 'libgdbm-dev': ensure => latest, }
  package { 'libsqlite3-dev': ensure => latest, }
  package { 'tk-dev': ensure => latest, }
  package { 'libssl-dev': ensure => latest, }
  package { 'openssl': ensure => latest, }
  package { 'libffi-dev': ensure => latest, }
  package { 'libz-dev': ensure => latest, }
  package { 'libreadline-dev': ensure => latest, }
  package { 'libncursesw5-dev': ensure => latest, }
  package { 'libbz2-dev': ensure => latest, }

  # CM730 firmware compilation.
  package { 'gcc-arm-none-eabi': ensure => latest, }
  package { 'libnewlib-arm-none-eabi': ensure => latest, }

  # System libraries
  package { 'libasound2-dev': ensure => latest, }

  # SETUP OUR ALTERNATIVES SO WE USE THE CORRECT COMPILER
  exec {'fix_compiler_environment':
    command => '/usr/bin/update-alternatives --remove-all gcc \
             ;  /usr/bin/update-alternatives --remove-all g++ \
             ;  /usr/bin/update-alternatives --remove-all gfortan \
             ;  /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && /usr/bin/update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
                                             --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
                                             --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7',
    require => [ Package['gcc-7'], Package['g++-7'], Package['gfortran-7'], Package['build-essential'], Package['binutils'], ]
  }

  # Manually install cmake
  exec {'install-cmake':
    creates => '/usr/local/bin/cmake',
    command => '/usr/bin/wget https://github.com/Kitware/CMake/releases/download/v3.13.3/cmake-3.13.3-Linux-x86_64.sh \
             && /bin/sh cmake-3.13.3-Linux-x86_64.sh --prefix=/usr/local --exclude-subdir \
             && rm cmake-3.13.3-Linux-x86_64.sh',
  }

  exec { "Intel_OpenCL_SDK":
    creates     => "/opt/intel/opencl/libOpenCL.so",
    command     => "mkdir intel-opencl &&
                    cd intel-opencl &&
                    wget http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip &&
                    unzip SRB5.0_linux64.zip &&
                    mkdir root &&
                    for i in *.tar.xz; do tar -C root -xf \"\$i\"; done &&
                    cp -r root/* /",
    path        =>  [ '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout     => 0,
    provider    => 'shell',
    require     => [ Package['unzip'], ],
  }
}

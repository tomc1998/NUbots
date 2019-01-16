include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node default {

  # We need build tools to compile
  class {'build_tools': }

  # These user tools make the shell much easier
  class {'user_tools':
    user => 'vagrant',
  }

  # Get and install our toolchain
  $toolchain_version = '3.0.1'
  wget::fetch { 'nubots_deb':
    destination => "/root/nubots-toolchain-${toolchain_version}.deb",
    source      => "http://nubots.net/debs/nubots-toolchain-${toolchain_version}.deb",
    timeout     => 0,
  } ->
  package { 'nubots-toolchain':
    provider => 'dpkg',
    ensure   => 'latest',
    source   => "/root/nubots-toolchain-${toolchain_version}.deb",
  }
}

node nubotsvmbuild {
  $archs = {
    'native'    => {'flags'       => ['', ],
                    'params'      => ['-m64', ],
                    'environment' => {'TARGET' => 'GENERIC', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', 'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig', 'CCAS' => '/usr/bin/gcc', 'AS' => '/usr/bin/gcc', 'CCASFLAGS' => '-m64', },
                   },
    'nuc7i7bnh' => {'flags'       => ['-march=broadwell', '-mtune=broadwell', '-mmmx', '-mno-3dnow', '-msse', '-msse2', '-msse3', '-mssse3', '-mno-sse4a', '-mcx16', '-msahf', '-mmovbe', '-maes', '-mno-sha', '-mpclmul', '-mpopcnt', '-mabm', '-mno-lwp', '-mfma', '-mno-fma4', '-mno-xop', '-mbmi', '-mbmi2', '-mno-tbm', '-mavx', '-mavx2', '-msse4.2', '-msse4.1', '-mlzcnt', '-mno-rtm', '-mno-hle', '-mrdrnd', '-mf16c', '-mfsgsbase', '-mrdseed', '-mprfchw', '-madx', '-mfxsr', '-mxsave', '-mxsaveopt', '-mno-avx512f', '-mno-avx512er', '-mno-avx512cd', '-mno-avx512pf', '-mno-prefetchwt1', '-mclflushopt', '-mxsavec', '-mxsaves', '-mno-avx512dq', '-mno-avx512bw', '-mno-avx512vl', '-mno-avx512ifma', '-mno-avx512vbmi', '-mno-clwb', '-mno-mwaitx', ],
                    'params'      => ['-m64', '--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                    'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', 'PKG_CONFIG_PATH' => '/usr/lib/x86_64-linux-gnu/pkgconfig', 'CCAS' => '/usr/bin/gcc', 'AS' => '/usr/bin/gcc', 'CCASFLAGS' => '-m64', },
                   },
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need build tools to compile and we need it done before the installer
  class {'build_tools': } -> class { 'protobuf': } -> Installer <| |>

  # These user tools make the shell much easier and these also should be done before installing
  class {'user_tools':
    user => 'vagrant',
  } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    # We need to match the protobuf version with the one we install in the python class.
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.5.0/protobuf-cpp-3.5.0.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'nuc7i7bnh' => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'prebuild'    => 'make distclean',
                       'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                       'method'      => 'autotools', },
    'zlib'         => {'url'         => 'http://www.zlib.net/zlib-1.2.11.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake', },
    'bzip2'        => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.1.tar.gz',
                       'creates'     => 'lib/libbz2.so',
                       'method'      => 'make', },
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'nuc7i7bnh' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools', },
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake', },
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.19.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ 'CROSS=1', ], },
                       'method'      => 'make',
                       'creates'     => 'lib/libopenblas.a', },
    'libsvm'       => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                       'creates'     => 'lib/svm.o',
                       'method'      => 'make', },
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-7.950.1.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ], },
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5.93/gperftools-2.5.93.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'nuc7i7bnh' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools', },
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'nuc7i7bnh' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake', },
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.6-pl2.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', ], },
                       'method'      => 'autotools', },
    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.5.1/libjpeg-turbo-1.5.1.tar.gz',
                       'args'        => { 'native'   => [ 'CCASFLAGS="-f elf64"', ],
                                          'nuc7i7bnh' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools', },
    'cppformat'    => {'url'         => 'https://github.com/fmtlib/fmt/archive/3.0.1.tar.gz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libfmt.a', },
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'method'      => 'autotools', },
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake', },
    'boost'        => {'url'         => 'https://dl.bintray.com/boostorg/release/1.65.0/source/boost_1_65_0.tar.gz',
                       'args'        => { 'native'    => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'nuc7i7bnh' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2'], ], },
    'espeak'       => {'url'         => 'https://github.com/Bidski/espeak/archive/v1.48.04.tar.gz',
                       'src_dir'     => 'src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ Installer['portaudio'], ], },
    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/releases/download/1.9.3/fswatch-1.9.3.tar.gz',
                       'method'      => 'autotools', },
    'ffi'          => {'url'         => 'https://github.com/libffi/libffi/archive/v3.2.1.tar.gz',
                       'postbuild'   => 'if [ -e PREFIX/lib32/libffi.a ]; then cp PREFIX/lib32/libffi* PREFIX/lib/; fi',
                       'method'      => 'autotools', },
    'util-linux'   => {'url'         => 'https://www.kernel.org/pub/linux/utils/util-linux/v2.31/util-linux-2.31.tar.xz',
                       'args'        => { 'native'    => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ],
                                          'nuc7i7bnh' => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ], },
                       'creates'     => 'lib/libmount.so',
                        'method'     => 'autotools', },
    'glib'         => {'url'         => 'ftp://ftp.gnome.org/pub/gnome/sources/glib/2.52/glib-2.52.3.tar.xz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/glib.config', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ],
                                          # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/glib.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ], },
                       'postbuild'   => 'cp glib/glibconfig.h PREFIX/include/glibconfig.h',
                       'require'     => [ Installer['ffi'], Installer['util-linux'], ],
                       'creates'     => 'lib/libglib-2.0.so',
                       'method'      => 'autotools', },
    'aravis'       => {'url'         => 'https://github.com/AravisProject/aravis/archive/ARAVIS_0_5_9.tar.gz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/aravis.config', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ],
                                          # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/aravis.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ], },
                       'require'     => [ Installer['xml2'], Installer['zlib'], Installer['glib'], ],
                       'creates'     => 'lib/libaravis-0.6.so',
                       'prebuild'    => 'sed "s/return\s(entry->schema\s>>\s10)\s\&\s0x0000001f;/return ((entry->schema >> 10) \& 0x0000001f) ? ARV_UVCP_SCHEMA_ZIP : ARV_UVCP_SCHEMA_RAW;/" -i src/arvuvcp.h',
                       'postbuild'   => 'cp src/arvconfig.h PREFIX/include/arvconfig.h',
                       'method'      => 'autotools', },
    'gtest'        => {'url'         => 'https://github.com/google/googletest/archive/release-1.8.0.tar.gz',
                       'creates'     => 'lib/libgtest.a',
                       'method' => 'cmake', },
    'leveldb'      => {'url'         => 'https://github.com/google/leveldb/archive/master.tar.gz',
                       'method'      => 'cmake', },
    'gflags'       => {'url'         => 'https://github.com/gflags/gflags/archive/v2.2.1.tar.gz',
                       'method'      => 'cmake', },
    'glog'         => {'url'         => 'https://github.com/google/glog/archive/v0.3.5.tar.gz',
                       'method'      => 'cmake', },
    'snappy'       => {'url'         => 'https://github.com/google/snappy/archive/1.1.7.tar.gz',
                       'args'        => { 'native'    => [ '-DSNAPPY_BUILD_TESTS="OFF"', ],
                                          'nuc7i7bnh' => [ '-DSNAPPY_BUILD_TESTS="OFF"', ], },
                       'method'      => 'cmake', },
    'lmdb'         => {'url'         => 'https://github.com/LMDB/lmdb/archive/LMDB_0.9.22.tar.gz',
                       'prebuild'    => 'cp libraries/liblmdb/* ./',
                       'method'      => 'make', },
    'hdf5'         => {'url'         => 'https://www.hdfgroup.org/package/source-gzip-4/?wpdmdl=13048&refresh=5bdff730711641541404464&ext=.tar.gz',
                       'method'      => 'cmake', },
    'viennacl'     => {'url'         => 'https://github.com/viennacl/viennacl-dev/archive/release-1.7.1.tar.gz',
                       'args'        => { 'native'    => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARY="/opt/intel/opencl/libOpenCL.so"', ],
                                          'nuc7i7bnh' => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARY="/opt/intel/opencl/libOpenCL.so"', ], },
                       'require'     => [ Exec['Intel_OpenCL_SDK'], Installer['boost']],
                       'method'      => 'cmake', },
    'clFFT'        => {'url'         => 'https://github.com/clMathLibraries/clFFT/archive/v2.12.2.tar.gz',
                       'prebuild'    => 'mv src/* ./',
                       'args'        => { 'native'    => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', ],
                                          'nuc7i7bnh' => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', ], },
                       'require'     => [ Exec['Intel_OpenCL_SDK'], ],
                       'method'      => 'cmake', },
    'clRNG'        => {'url'         => 'https://github.com/clMathLibraries/clRNG/archive/v1.0.0-beta.tar.gz',
                       'prebuild'    => 'cp -r src/* ./ && sudo sed -i "s/-Wall -pedantic-errors/-Wall/" CMakeLists.txt',
                       'args'        => { 'native'    => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', ],
                                          'nuc7i7bnh' => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', ], },
                       'require'     => [ Exec['Intel_OpenCL_SDK'], ],
                       'method'      => 'cmake', },
    'isaac'        => {'url'         => 'https://github.com/intel/isaac/archive/master.zip',
                       'prebuild'    => 'mv isaac-master/* ./',
                       'method'      => 'cmake', },
    'greentea'     => {'url'         => 'https://github.com/naibaf7/libdnn/archive/master.zip',
                       'prebuild'    => 'mv libdnn-master/* ./',
                       'args'        => { 'native'    => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', '-DUSE_CUDA="OFF"', '-DUSE_OPENCL=ON', '-DUSE_INDEX_64=OFF',],
                                          'nuc7i7bnh' => [ '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', '-DUSE_CUDA="OFF"', '-DUSE_OPENCL=ON', '-DUSE_INDEX_64=OFF',], },
                       'require'     => [ Exec['Intel_OpenCL_SDK'], Installer['viennacl']],
                       'method'      => 'cmake', },
    'ffmpeg'       => {'url'         => 'http://ffmpeg.org/releases/ffmpeg-2.8.13.tar.gz',
                       'args'        => { 'native'    => [ '--enable-shared' ],
                                          'nuc7i7bnh' => [ '--enable-shared' ], },
                       'method'      => 'autotools', },
    'opencv'       => {'url'         => 'https://github.com/opencv/opencv/archive/3.3.1.tar.gz',
                       'args'        => { 'native'   => [ '-DOPENCV_ENABLE_NONFREE=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_NVCUVID=OFF', '-DWITH_EIGEN=ON', '-DWITH_ARAVIS=ON', '-DWITH_TBB=ON', '-DWITH_OPENMP=ON', '-DWITH_OPENCL=ON', '-DBUILD_opencv_apps=OFFF', '-DBUILD_opencv_js=OFF', '-DBUILD_opencv_python3=ON', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_WITH_DEBUG_INFO=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_TIFF=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', ],
                                          'nuc7i7bnh' => [ '-DOPENCV_ENABLE_NONFREE=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_NVCUVID=OFF', '-DWITH_EIGEN=ON', '-DWITH_ARAVIS=ON', '-DWITH_TBB=ON', '-DWITH_OPENMP=ON', '-DWITH_OPENCL=ON', '-DBUILD_opencv_apps=OFFF', '-DBUILD_opencv_js=OFF', '-DBUILD_opencv_python3=ON', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_WITH_DEBUG_INFO=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_TIFF=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', ], },
                       'require'     => [ Installer['aravis'], Installer['eigen3'], ],
                       'creates'     => 'lib/libopencv_core.so',
                       'method'      => 'cmake', },
    'clcaffe'        => {'url'         => 'https://github.com/01org/caffe/archive/inference-optimize.tar.gz',
                       'prebuild'    => 'export ISAAC_HOME=PREFIX',
                       'args'        => { 'native'   => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_INDEX_64=OFF', '-DUSE_FFT=OFF', '-DBUILD_examples=OFF', '-DBUILD_tools=OFF', '-D'],
                                          'nuc7i7bnh' => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES="/opt/intel/opencl/libOpenCL.so"', '-DOPENCL_INCLUDE_DIRS="/opt/intel/opencl/include/"', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_INDEX_64=OFF', '-DUSE_FFT=OFF', '-DBUILD_examples=OFF', '-DBUILD_tools=OFF', ], },
                       'require'     => [ Exec['Intel_OpenCL_SDK'], Installer['isaac'], Installer['boost'], Installer['hdf5'], Installer['leveldb'], Installer['lmdb'], Installer['glog'], Installer['gflags'], Installer['clFFT'], Installer['greentea'], Installer['opencv'], ],
                       'creates'     => 'lib/libcaffe.so',
                       'method'      => 'cmake', },
  }

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Hash,
                          Optional['require'] => Tuple[Any, 1, default],
                          'method' => String,
                          Optional['src_dir'] => String,
                          Optional['prebuild'] => String,
                          Optional['postbuild'] => String,
                          Optional['environment'] => Hash}] $params| {

        $extension = $params['url'] ? {
          /.*\.zip/       => 'zip',
          /.*\.tgz/       => 'tgz',
          /.*\.tar\.gz/   => 'tar.gz',
          /.*\.txz/       => 'txz',
          /.*\.tar\.xz/   => 'tar.xz',
          /.*\.tbz/       => 'tbz',
          /.*\.tbz2/      => 'tbz2',
          /.*\.tar\.bz2/  => 'tar.bz2',
          /.*\.h/         => 'h',
          /.*\.hpp/       => 'hpp',
          default         => 'UNKNOWN',
        }

        archive { "${archive}":
          url              => $params['url'],
          target           => "/nubots/toolchain/src/${archive}",
          src_target       => "/nubots/toolchain/src",
          purge_target     => true,
          checksum         => false,
          follow_redirects => true,
          timeout          => 0,
          extension        => $extension,
          strip_components => 1,
          root_dir         => '.',
          require          => [ Class['installer::prerequisites'], Class['build_tools'], ],
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['build_tools'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          environment => $params['environment'],
          extension   => $extension,
        }
  }

  # Install quex.
  class { 'quex': }

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/catchorg/Catch2/master/single_include/catch2/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
    method    => 'wget',
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    # Update the armadillo config header file for all archs.
    file { "armadillo_${arch}_config":
      path    => "/nubots/toolchain/${arch}/include/armadillo_bits/config.hpp",
      source  => 'puppet:///modules/files/nubots/toolchain/include/armadillo_bits/config.hpp',
      ensure  => present,
      require => [ Installer['armadillo'], ],
    }
  }

  # After we have installed, create the CMake toolchain files and then build our deb.
  Installer <| |> ~> class { 'toolchain_deb': }

  # Patch some system utilities to make sure they ignore our preset LD_LIBRARY_PATH
  file { "/nubots/toolchain/bin/msgfmt.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/msgfmt \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/msgfmt.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/msgmerge.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/msgmerge \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/msgmerge.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/xgettext.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/xgettext \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/xgettext.sh",
    mode    => "a+x",
  } -> Installer <| |>

  file { "/nubots/toolchain/bin/pkg-config.sh":
    content =>
"
#! /bin/bash
LD_LIBRARY_PATH= /usr/bin/pkg-config \"$@\"
",
    ensure  => present,
    path    => "/nubots/toolchain/bin/pkg-config.sh",
    mode    => "a+x",
  } -> Installer <| |>

  $archs.each |String $arch, Hash $params| {
    $prefix = '/nubots/toolchain'

    # We need to prevent glib from trying to run tests when cross-compiling glib (to avoid SIGILL).
    file { "${arch}_glib.config":
      content =>
"glib_cv_stack_grows=no
glib_cv_uscore=no
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/glib.config",
      mode    => "a-w",
      before  => [ Installer['glib'], ],
    }

    # Force paths to gettext bianries (to avoid SIGILL).
    file { "${arch}_aravis.config":
      content =>
"ac_cv_path_XGETTEXT=${prefix}/bin/xgettext.sh
ac_cv_path_MSGMERGE=${prefix}/bin/msgmerge.sh
ac_cv_path_MSGFMT=${prefix}/bin/msgfmt.sh
ac_cv_path_PKG_CONFIG=${prefix}/bin/pkg-config.sh
",
      ensure  => present,
      path    => "${prefix}/${arch}/src/aravis.config",
      mode    => "a-w",
      before  => [ Installer['aravis'], ],
    }

    # Create CMake toolchain files.
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\")
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\")

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }
  }
}

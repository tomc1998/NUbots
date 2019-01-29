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
  $toolchain_version = '3.0.1.1'
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
  class {'build_tools': } -> class { 'python': } -> class { 'protobuf': } -> Installer <| |>

  # These user tools make the shell much easier and these also should be done before installing
  class {'user_tools':
    user => 'vagrant',
  } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    # We need to match the protobuf version with the one we install in the python class.
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.tar.gz',
                       'args'        => { 'native'   => [ '-Dprotobuf_BUILD_PROTOC_BINARIES=OFF', '-Dprotobuf_BUILD_TESTS=OFF', '-Dprotobuf_WITH_ZLIB=ON', ],
                                          'nuc7i7bnh' => [ '-Dprotobuf_BUILD_PROTOC_BINARIES=OFF', '-Dprotobuf_BUILD_TESTS=OFF', '-Dprotobuf_WITH_ZLIB=ON', ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'src_dir'     => 'cmake',
                       'method'      => 'cmake', },
    'zlib'         => {'url'         => 'http://www.zlib.net/zlib-1.2.11.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake', },
    'bzip2'        => {'url'         => 'https://github.com/Bidski/bzip2/archive/v1.0.6.1.tar.gz',
                       'creates'     => 'lib/libbz2.so',
                       'method'      => 'make', },
    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.9.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'nuc7i7bnh' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools', },
    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake', },
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.3.5.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc7i7bnh' => [ 'CROSS=1', ], },
                       'method'      => 'make',
                       'creates'     => 'lib/libopenblas.a', },
    'libsvm'       => {'url'         => 'https://github.com/Bidski/libsvm/archive/v322.tar.gz',
                       'creates'     => 'lib/svm.o',
                       'method'      => 'make', },
    'armadillo'    => {'url'         => 'https://downloads.sourceforge.net/project/arma/armadillo-9.200.7.tar.xz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libarmadillo.so',
                       'require'     => [ Installer['openblas'], ], },
    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/archive/gperftools-2.7.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'nuc7i7bnh' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools', },
    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'nuc7i7bnh' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake', },
    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.8.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', '--enable-openmp', '--enable-threads', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', '--enable-openmp', '--enable-threads', ], },
                       'method'      => 'autotools', },
    'fftw3f'       => {'url'         => 'http://www.fftw.org/fftw-3.3.8.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', '--enable-float', '--enable-openmp', '--enable-threads', ],
                                          'nuc7i7bnh' => [ '--disable-fortran', '--enable-shared', '--enable-float', '--enable-openmp', '--enable-threads', ], },
                       'method'      => 'autotools',},
    'jpeg'         => {'url'         => 'https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.0.1.tar.gz',
                       'args'        => { 'native'   => [ '-DWITH_JPEG7=ON', '-DWITH_JPEG8=ON', ],
                                          'nuc7i7bnh' => [ '-DWITH_JPEG7=ON', '-DWITH_JPEG8=ON', ], },
                       'method'      => 'cmake', },
    'cppformat'    => {'url'         => 'https://github.com/fmtlib/fmt/archive/5.3.0.tar.gz',
                       'method'      => 'cmake',
                       'creates'     => 'lib/libfmt.a', },
    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'method'      => 'autotools', },
    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',
                       'require'     => [ Installer['fftw3'], Installer['fftw3f'], ], },
    'boost'        => {'url'         => 'https://dl.bintray.com/boostorg/release/1.69.0/source/boost_1_69_0.tar.gz',
                       'args'        => { 'native'   => [ 'address-model=64', 'architecture=x86', 'link=static', ],
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
    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/archive/1.14.0.tar.gz',
                       'method'      => 'autotools', },
    'ffi'          => {'url'         => 'https://github.com/libffi/libffi/archive/v3.2.1.tar.gz',
                       'postbuild'   => 'if [ -e PREFIX/lib32/libffi.a ]; then cp PREFIX/lib32/libffi* PREFIX/lib/; fi',
                       'method'      => 'autotools', },
    'util-linux'   => {'url'         => 'https://mirrors.edge.kernel.org/pub/linux/utils/util-linux/v2.33/util-linux-2.33.tar.xz',
                       'args'        => { 'native'    => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ],
                                          'nuc7i7bnh' => [ '--disable-all-programs', '--enable-libblkid', '--enable-libmount', '--enable-libuuid', '--without-python', '--with-bashcompletiondir=PREFIX/share/bash-completion/completions' ], },
                       'creates'     => 'lib/libmount.so',
                        'method'     => 'autotools', },
    'glib'         => {'url'         => 'ftp://ftp.gnome.org/pub/gnome/sources/glib/2.58/glib-2.58.3.tar.xz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/glib.config', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ],
                                          # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/glib.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--with-threads', '--with-pcre=internal', '--disable-gtk-doc', '--disable-man', ], },
                       'postbuild'   => 'cp glib/glibconfig.h PREFIX/include/glibconfig.h',
                       'require'     => [ Installer['ffi'], Installer['util-linux'], ],
                       'creates'     => 'lib/libglib-2.0.so',
                       'method'      => 'autotools', },
    'aravis'       => {'url'         => 'https://github.com/AravisProject/aravis/archive/ARAVIS_0_6_1.tar.gz',
                       'args'        => { 'native'   => [ '--cache-file=PREFIX/src/aravis.config', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ],
                                          # Technically we are cross compiling for the nuc7i7bnh, even though both the host and build systems are both x86_64-linux-gnu
                                          'nuc7i7bnh' => [ '--cache-file=PREFIX/src/aravis.config', '--host=x86_64-linux-gnu', '--build=x86_64-unknown-linux-gnu', '--disable-viewer', '--disable-gst-plugin', '--disable-gst-0.10-plugin', '--disable-gtk-doc', '--disable-gtk-doc-html', '--disable-gtk-doc-pdf', '--enable-usb', '--disable-zlib-pc', ], },
                       'require'     => [ Installer['xml2'], Installer['zlib'], Installer['glib'], ],
                       'creates'     => 'lib/libaravis-0.6.so',
                       'prebuild'    => 'sed "s/return\s(entry->schema\s>>\s10)\s\&\s0x0000001f;/return ((entry->schema >> 10) \& 0x0000001f) ? ARV_UVCP_SCHEMA_ZIP : ARV_UVCP_SCHEMA_RAW;/" -i src/arvuvcp.h',
                       'postbuild'   => 'cp src/arvconfig.h PREFIX/include/arvconfig.h',
                       'method'      => 'autotools', },

    # Everything below this point is for OpenCL Caffe
    'gflags'       => {'url'         => 'https://github.com/gflags/gflags/archive/v2.2.2.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTING=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTING=OFF', ], },
                       'creates'     => 'lib/libgflags.a',
                       'method'      => 'cmake', },
    'gtest'        => {'url'         => 'https://github.com/google/googletest/archive/release-1.8.1.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_GMOCK=ON', '-DINSTALL_GTEST=ON', ],
                                          'nuc7i7bnh' => [ '-DBUILD_GMOCK=ON', '-DINSTALL_GTEST=ON', ], },
                       'creates'     => 'lib/libgtest.a',
                       'method'      => 'cmake', },
    'snappy'       => {'url'         => 'https://github.com/google/snappy/archive/1.1.7.tar.gz',
                       'args'        => { 'native'   => [ '-DSNAPPY_BUILD_TESTS=OFF', ],
                                          'nuc7i7bnh' => [ '-DSNAPPY_BUILD_TESTS=OFF', ], },
                       'require'     => [ Installer['gtest'], Installer['gflags'], ],
                       'creates'     => 'lib/libsnappy.a',
                       'method'      => 'cmake', },
    'leveldb'      => {'url'         => 'https://github.com/google/leveldb/archive/master.tar.gz',
                       'creates'     => 'lib/leveldb.a',
                       'args'        => { 'native'   => [ '-DLEVELDB_BUILD_TESTS=OFF', '-DLEVELDB_BUILD_BENCHMARKS=OFF', '-DLEVELDB_INSTALL=ON', ],
                                          'nuc7i7bnh' => [ '-DLEVELDB_BUILD_TESTS=OFF', '-DLEVELDB_BUILD_BENCHMARKS=OFF', '-DLEVELDB_INSTALL=ON', ], },
                       'creates'     => 'lib/libleveldb.a',
                       'require'     => [ Installer['snappy'], ],
                       'method'      => 'cmake', },
    'lmdb'         => {'url'         => 'https://github.com/LMDB/lmdb/archive/LMDB_0.9.23.tar.gz',
                       'creates'     => 'lib/liblmdb.so',
                       'args'        => { 'native'   => [ 'prefix=PREFIX', ],
                                          'nuc7i7bnh' => [ 'prefix=PREFIX', ], },
                       'src_dir'     => 'libraries/liblmdb',
                       'method'      => 'make', },
    'hdf5'         => {'url'         => 'https://support.hdfgroup.org/ftp/HDF5/current/src/CMake-hdf5-1.10.4.tar.gz',
                       'args'        => { 'native'   => [ '-DHDF5_BUILD_CPP_LIB=ON', '-DHDF5_ENABLE_Z_LIB_SUPPORT=ON', ],
                                          'nuc7i7bnh' => [ '-DHDF5_BUILD_CPP_LIB=ON', '-DHDF5_ENABLE_Z_LIB_SUPPORT=ON', ], },
                       'src_dir'     => 'CMake-hdf5-1.10.4/hdf5-1.10.4',
                       'creates'     => 'lib/libhdf5.so',
                       'method'      => 'cmake', },
    'glog'         => {'url'         => 'https://github.com/google/glog/archive/v0.3.5.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTING=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTING=OFF', ], },
                       'require'     => [ Installer['gflags'], ],
                       'creates'     => 'lib/libglog.a',
                       'method'      => 'cmake', },
    'opencv'       => {'url'         => 'https://github.com/opencv/opencv/archive/4.0.1.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_CUDA_STUBS=OFF', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_IPP_IW=ON', '-DBUILD_ITT=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_OPENEXR=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', '-DBUILD_TIFF=ON', '-DBUILD_WEBP=ON', '-DBUILD_PROTOBUF=OFF', '-DBUILD_JAVA=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_opencv_apps=OFF', '-DENABLE_FAST_MATH=ON', '-DENABLE_PRECOMPILED_HEADERS=OFF', '-DOpenBLAS_INCLUDE_DIR=PREFIX/include', '-DOpenBLAS_LIB=PREFIX/lib/libopenblas.so', '-DOPENCL_INCLUDE_DIR=/opt/intel/opencl/include', '-DOPENCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', '-DOPENCV_PYTHON3_VERSION=3', '-DOPENCV_ENABLE_NONFREE=ON', '-DOPENCV_EXTRA_MODULES_PATH=PREFIX/src/opencv/opencv_contrib-4.0.1/modules', '-DPROTOBUF_UPDATE_FILES=ON', '-DWITH_ARAVIS=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_EIGEN=ON', '-DWITH_IPP=ON', '-DWITH_ITT=ON', '-DWITH_JASPER=ON', '-DWITH_JPEG=ON', '-DWITH_LAPACK=ON', '-DWITH_OPENCL=ON', '-DWITH_OPENEXR=ON', '-DWITH_OPENMP=ON', '-DWITH_PNG=ON', '-DWITH_PROTOBUF=ON', '-DWITH_TBB=ON', '-DWITH_TIFF=ON', '-DWITH_WEBP=ON', "-DPYTHON_DEFAULT_EXECUTABLE=\"PYTHON_PATH\"", "-DPYTHON3_EXECUTABLE=\"PYTHON_PATH\"", ],
                                          'nuc7i7bnh' => [ '-DBUILD_CUDA_STUBS=OFF', '-DBUILD_DOCS=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_IPP_IW=ON', '-DBUILD_ITT=ON', '-DBUILD_JASPER=ON', '-DBUILD_JPEG=ON', '-DBUILD_OPENEXR=ON', '-DBUILD_PNG=ON', '-DBUILD_TBB=ON', '-DBUILD_TIFF=ON', '-DBUILD_WEBP=ON', '-DBUILD_PROTOBUF=OFF', '-DBUILD_JAVA=OFF', '-DBUILD_PERF_TESTS=OFF', '-DBUILD_TESTS=OFF', '-DBUILD_ZLIB=OFF', '-DBUILD_opencv_apps=OFF', '-DENABLE_FAST_MATH=ON', '-DENABLE_PRECOMPILED_HEADERS=OFF', '-DOpenBLAS_INCLUDE_DIR=PREFIX/include', '-DOpenBLAS_LIB=PREFIX/lib/libopenblas.so', '-DOPENCL_INCLUDE_DIR=/opt/intel/opencl/include', '-DOPENCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', '-DOPENCV_PYTHON3_VERSION=3', '-DOPENCV_ENABLE_NONFREE=ON', '-DOPENCV_EXTRA_MODULES_PATH=PREFIX/src/opencv/opencv_contrib-4.0.1/modules', '-DPROTOBUF_UPDATE_FILES=ON', '-DWITH_ARAVIS=ON', '-DWITH_CUDA=OFF', '-DWITH_CUFFT=OFF', '-DWITH_CUBLAS=OFF', '-DWITH_EIGEN=ON', '-DWITH_IPP=ON', '-DWITH_ITT=ON', '-DWITH_JASPER=ON', '-DWITH_JPEG=ON', '-DWITH_LAPACK=ON', '-DWITH_OPENCL=ON', '-DWITH_OPENEXR=ON', '-DWITH_OPENMP=ON', '-DWITH_PNG=ON', '-DWITH_PROTOBUF=ON', '-DWITH_TBB=ON', '-DWITH_TIFF=ON', '-DWITH_WEBP=ON', "-DPYTHON_DEFAULT_EXECUTABLE=\"PYTHON_PATH\"", "-DPYTHON3_EXECUTABLE=\"PYTHON_PATH\"", ], },
                       'prebuild'    => 'if [ ! -d opencv_contrib-4.0.1 ]; then wget -N https://github.com/opencv/opencv_contrib/archive/4.0.1.tar.gz && tar xf 4.0.1.tar.gz; fi',
                       'require'     => [ Installer['aravis'], Installer['eigen3'], ],
                       'creates'     => 'lib/libopencv_core.so',
                       'method'      => 'cmake', },
    'viennacl'     => {'url'         => 'https://github.com/viennacl/viennacl-dev/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_EXAMPLES=OFF', '-DBUILD_TESTING=OFF', '-DOPENCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DBUILD_EXAMPLES=OFF', '-DBUILD_TESTING=OFF', '-DOPENCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'include/viennacl/version.hpp',
                       'method'      => 'cmake', },
    'clBLAS'       => {'url'         => 'https://github.com/clMathLibraries/clBLAS/archive/master.tar.gz',
                       'args'        => { 'native'   => [ "-DPYTHON_EXECUTABLE=\$(which python)", '-DSUFFIX_LIB=""', '-DNetlib_BLAS_LIBRARY=PREFIX/lib/libopenblas.so', '-DNetlib_INCLUDE_DIRS=PREFIX/include', '-DBUILD_TEST=OFF', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ "-DPYTHON_EXECUTABLE=\$(which python)", '-DSUFFIX_LIB=""', '-DNetlib_BLAS_LIBRARY=PREFIX/lib/libopenblas.so', '-DNetlib_INCLUDE_DIRS=PREFIX/include', '-DBUILD_TEST=OFF', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'lib/libclBLAS.so',
                       'src_dir'     => 'src',
                       'require'     => [ Installer['viennacl'], Installer['fftw3f'], Installer['fftw3'], Installer['boost'], ],
                       'method'      => 'cmake', },
    'clFFT'        => {'url'         => 'https://github.com/clMathLibraries/clFFT/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_CLIENT=OFF', '-DBUILD_TEST=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_CALLBACK_CLIENT=OFF', '-DSUFFIX_LIB=""', '-DOpenCL_INCLUDE_DIR=/opt/intel/opencl/include', '-DOpenCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DBUILD_CLIENT=OFF', '-DBUILD_TEST=OFF', '-DBUILD_EXAMPLES=OFF', '-DBUILD_CALLBACK_CLIENT=OFF', '-DSUFFIX_LIB=""', '-DOpenCL_INCLUDE_DIR=/opt/intel/opencl/include', '-DOpenCL_LIBRARY=/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'lib/libclFFT.so',
                       'src_dir'     => 'src',
                       'require'     => [ Installer['viennacl'], Installer['fftw3f'], Installer['fftw3'], Installer['boost'], ],
                       'method'      => 'cmake', },
    'clRNG'        => {'url'         => 'https://github.com/clMathLibraries/clRNG/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TEST=OFF', '-DSUFFIX_LIB=""', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TEST=OFF', '-DSUFFIX_LIB=""', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ], },
                       'environment' => { 'CFLAGS' => '-Wno-error=expansion-to-defined', 'CXXFLAGS' => '-Wno-error=expansion-to-defined', },
                       'creates'     => 'lib/libclRNG.so',
                       'src_dir'     => 'src',
                       'require'     => [ Installer['viennacl'], Installer['fftw3f'], Installer['fftw3'], Installer['boost'], ],
                       'method'      => 'cmake', },
    'clSPARSE'     => {'url'         => 'https://github.com/clMathLibraries/clSPARSE/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DSUFFIX_LIB=""', '-DBUILD_TESTS=OFF', '-DBUILD_BENCHMARKS=OFF', '-DUSE_SYSTEM_CL2HPP=ON', '-DOPENCL_ROOT=/opt/intel/opencl', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DSUFFIX_LIB=""', '-DBUILD_TESTS=OFF', '-DBUILD_BENCHMARKS=OFF', '-DUSE_SYSTEM_CL2HPP=ON', '-DOPENCL_ROOT=/opt/intel/opencl', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'lib/libclSPARSE.so',
                       'src_dir'     => 'src',
                       'require'     => [ Installer['viennacl'], Installer['fftw3f'], Installer['fftw3'], Installer['boost'], Installer['gtest'], ],
                       'method'      => 'cmake', },
    'isaac'        => {'url'         => 'https://github.com/intel/isaac/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTING=OFF', ],
                                          'nuc7i7bnh' => [ '-DBUILD_TESTING=OFF', ], },
                                        # Because they blindly link against OpenCL without first finding out where OpenCL lives
                       'environment' => { 'CFLAGS' => '-L/opt/intel/opencl',  'CXXFLAGS' => '-L/opt/intel/opencl', },
                       'creates'     => 'lib/libisaac.so',
                       'require'     => [ Installer['viennacl'], Installer['clBLAS'], ],
                       'method'      => 'cmake', },
    'libdnn'       => {'url'         => 'https://github.com/naibaf7/libdnn/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DUSE_CUDA=OFF', '-DUSE_OPENCL=ON', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ],
                                          'nuc7i7bnh' => [ '-DUSE_CUDA=OFF', '-DUSE_OPENCL=ON', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', ], },
                       'creates'     => 'lib/libgreentea_libdnn.so',
                       'require'     => [ Installer['viennacl'], ],
                       'method'      => 'cmake', },
    'caffe'        => {'url'         => 'https://github.com/01org/caffe/archive/inference-optimize.tar.gz',
                       'prebuild'    => 'export ISAAC_HOME=PREFIX',
                       'args'        => { 'native'   => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_FFT=ON', '-DBUILD_examples=OFF', '-DBUILD_tools=ON', '-DBUILD_python_layer=ON', ],
                                          'nuc7i7bnh' => [ '-DUSE_GREENTEA=ON', '-DUSE_CUDA=OFF', '-DUSE_INTEL_SPATIAL=ON', '-DBUILD_docs=OFF', '-DUSE_ISAAC=ON', '-DViennaCL_INCLUDE_DIR=PREFIX/include', '-DBLAS=Open', '-DOPENCL_LIBRARIES=/opt/intel/opencl/libOpenCL.so', '-DOPENCL_INCLUDE_DIRS=/opt/intel/opencl/include', '-Dpython_version=3', '-DUSE_OPENMP=ON', '-DUSE_FFT=ON', '-DBUILD_examples=OFF', '-DBUILD_tools=ON', '-DBUILD_python_layer=ON', ], },
                                        # https://github.com/BVLC/caffe/issues/1761
                       'prebuild'    => "protoc src/caffe/proto/caffe.proto --cpp_out=. &&
                                         mkdir -p include/caffe/proto &&
                                         mv src/caffe/proto/caffe.pb.h include/caffe/proto/ &&
                                         sed 's/CV_CAP_PROP/cv::CAP_PROP/' -i src/caffe/layers/video_data_layer.cpp &&
                                         sed 's/CV_LOAD_IMAGE_COLOR/cv::IMREAD_COLOR/' -i src/caffe/layers/window_data_layer.cpp &&
                                         sed 's/CV_FOURCC/cv::VideoWriter::fourcc/' -i src/caffe/util/bbox_util.cpp &&
                                         sed 's/CV_FILLED/cv::FILLED/' -i src/caffe/util/bbox_util.cpp &&
                                         sed 's/CV_BGR/cv::COLOR_BGR/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_HSV/cv::COLOR_HSV/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_RGB/cv::COLOR_RGB/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_GRAY/cv::COLOR_GRAY/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_Y/cv::COLOR_Y/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_LOAD_IMAGE/cv::IMREAD/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_IMWRITE/cv::IMWRITE/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_THRESH/cv::THRESH/g' -i src/caffe/util/im_transforms.cpp &&
                                         sed 's/CV_LOAD_IMAGE/cv::IMREAD/g' -i src/caffe/util/io.cpp",
                       'require'     => [ Installer['isaac'], Installer['boost'], Installer['hdf5'], Installer['leveldb'], Installer['lmdb'], Installer['glog'], Installer['gflags'], Installer['clFFT'], Installer['clBLAS'], Installer['clRNG'], Installer['clSPARSE'], Installer['libdnn'], Installer['opencv'], ],
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
    url       => 'https://github.com/catchorg/Catch2/releases/download/v2.5.0/catch.hpp',
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

set(OpenCL_INCLUDE_DIR \"/opt/intel/opencl/include\" CACHE STRING \"\")
set(OpenCL_LIBRARY \"/opt/intel/opencl/libOpenCL.so\" CACHE STRING \"\")

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
      before  => Class['toolchain_deb'],
    }
  }
}

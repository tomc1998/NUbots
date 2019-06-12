#ifndef UTILITY_OPENCL_OPENCL_HPP
#define UTILITY_OPENCL_OPENCL_HPP

#include <fmt/format.h>
#include <nuclear>
#include <string>
#include <vector>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif  // !__APPLE__

#include "error_category.hpp"
#include "wrapper.hpp"

namespace utility {
namespace opencl {
    struct OpenCLBuildContext {
        OpenCLBuildContext(context& ctx, const std::string& compile_flags) : ctx(ctx), compile_flags(compile_flags) {}
        context& ctx;
        std::string compile_flags;
    };

    std::tuple<cl_platform_id, cl_device_id, cl_uint> find_best_device(bool use_gpu = true) {
        // Get our platforms
        cl_uint platform_count = 0;
        ::clGetPlatformIDs(0, nullptr, &platform_count);
        std::vector<cl_platform_id> platforms(platform_count);
        ::clGetPlatformIDs(platforms.size(), platforms.data(), nullptr);

        if (platform_count == 0) {
            NUClear::log<NUClear::ERROR>("No OpenCL platforms found. Check OpenCL Installation");
            throw std::runtime_error("No OpenCL platforms found. Check OpenCL Installation");
        }

        // Which device/platform we are going to use
        cl_platform_id best_platform = nullptr;
        cl_device_id best_device     = nullptr;
        cl_uint best_compute_units   = 0;

        // Go through our platforms
        for (const auto& platform : platforms) {
            cl_uint device_count = 0;
            ::clGetDeviceIDs(platform, use_gpu ? CL_DEVICE_TYPE_GPU : CL_DEVICE_TYPE_CPU, 0, nullptr, &device_count);
            std::vector<cl_device_id> devices(device_count);
            ::clGetDeviceIDs(
                platform, use_gpu ? CL_DEVICE_TYPE_GPU : CL_DEVICE_TYPE_CPU, device_count, devices.data(), nullptr);

            // Go through our devices on the platform
            for (const auto& device : devices) {
                // Length of data for strings
                size_t len;
                std::vector<char> data;

                // Print device details
                ::clGetDeviceInfo(device, CL_DEVICE_NAME, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_NAME, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(fmt::format("\tDevice: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DEVICE_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tHardware version: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DRIVER_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DRIVER_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tSoftware version: {}", std::string(data.begin(), data.end())));


                ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, 0, nullptr, &len);
                data.resize(len);
                ::clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, len, data.data(), nullptr);
                NUClear::log<NUClear::INFO>(
                    fmt::format("\tOpenCL C version: {}", std::string(data.begin(), data.end())));


                cl_uint max_compute_units = 0;
                ::clGetDeviceInfo(
                    device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(max_compute_units), &max_compute_units, nullptr);
                NUClear::log<NUClear::INFO>(fmt::format("\tParallel compute units: {}", max_compute_units));

                if (max_compute_units > best_compute_units) {
                    best_compute_units = max_compute_units;
                    best_platform      = platform;
                    best_device        = device;
                }
            }
        }

        if ((best_platform == nullptr) || (best_device == nullptr) || (best_compute_units == 0)) {
            NUClear::log<NUClear::ERROR>("No OpenCL devices found. Check OpenCL Installation");
            throw std::runtime_error("No OpenCL devices found. Check OpenCL Installation");
        }

        // Print information about our selected device
        {
            // Length of data for strings
            size_t len;
            std::vector<char> data;

            // Print device details
            ::clGetDeviceInfo(best_device, CL_DEVICE_NAME, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_NAME, len, data.data(), nullptr);
            NUClear::log(fmt::format("\tDevice: {}", std::string(data.begin(), data.end())));

            ::clGetDeviceInfo(best_device, CL_DEVICE_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_VERSION, len, data.data(), nullptr);
            NUClear::log(fmt::format("\tHardware version: {}", std::string(data.begin(), data.end())));

            ::clGetDeviceInfo(best_device, CL_DRIVER_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DRIVER_VERSION, len, data.data(), nullptr);
            NUClear::log(fmt::format("\tSoftware version: {}", std::string(data.begin(), data.end())));

            ::clGetDeviceInfo(best_device, CL_DEVICE_OPENCL_C_VERSION, 0, nullptr, &len);
            data.resize(len);
            ::clGetDeviceInfo(best_device, CL_DEVICE_OPENCL_C_VERSION, len, data.data(), nullptr);
            NUClear::log(fmt::format("\tOpenCL C version: {}", std::string(data.begin(), data.end())));
        }

        return std::make_tuple(best_platform, best_device, best_compute_units);
    }

    program compile(context& ctx, const std::string& compile_flags, const std::string& source) {
        cl_int error;
        // Create an OpenCL program from the provided context and source file
        const char* cstr = source.c_str();
        size_t csize     = source.size();
        program prog     = program(::clCreateProgramWithSource(ctx, 1, &cstr, &csize, &error), ::clReleaseProgram);

        // Build the program
        error = ::clBuildProgram(prog, 0, nullptr, compile_flags.c_str(), nullptr, nullptr);
        if (error != CL_SUCCESS) {
            // Get devices to build for
            size_t used = 0;
            error       = ::clGetProgramInfo(prog, CL_PROGRAM_NUM_DEVICES, sizeof(used), &used, nullptr);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), fmt::format("Error getting OpenCL program info"));
            }
            std::vector<::cl_device_id> devices(used);
            error = ::clGetProgramInfo(prog, CL_PROGRAM_DEVICES, used, devices.data(), &used);
            if (error != CL_SUCCESS) {
                throw std::system_error(
                    error, opencl_error_category(), fmt::format("Error getting OpenCL program devices"));
            }
            NUClear::log(fmt::format("Number of devices {}", used));

            // Get program build log
            ::clGetProgramBuildInfo(prog, devices.front(), CL_PROGRAM_BUILD_LOG, 0, nullptr, &used);
            std::vector<char> log(used);
            ::clGetProgramBuildInfo(prog, devices.front(), CL_PROGRAM_BUILD_LOG, log.size(), log.data(), &used);

            throw std::system_error(error,
                                    opencl_error_category(),
                                    fmt::format("Error building OpenCL kernel.\nBuild Flags: {}\nCompile Log:\n{}",
                                                compile_flags,
                                                std::string(log.begin(), log.begin() + used)));
        }

        return prog;
    }

    program compile(const std::string& source, OpenCLBuildContext& build_context) {
        return compile(build_context.ctx, build_context.compile_flags, source);
    }

    kernel get_kernel(const program& program, const std::string& kernel_name) {
        cl_int error;
        kernel k = kernel(::clCreateKernel(program, kernel_name.c_str(), &error), ::clReleaseKernel);
        if (error != CL_SUCCESS) {
            throw std::system_error(
                error, opencl_error_category(), fmt::format("Error getting {} kernel", kernel_name));
        }
        return k;
    }

    template <typename T>
    void set_kernel_arg(const kernel& k, const cl_uint& arg_index, const size_t& arg_size, const T& arg_value) {
        cl_int error;
        error = ::clSetKernelArg(k, arg_index, arg_size, &arg_value);
        if (error != CL_SUCCESS) {
            size_t len;
            std::vector<char> data;
            ::clGetKernelInfo(k, CL_KERNEL_FUNCTION_NAME, 0, nullptr, &len);
            data.resize(len);
            ::clGetKernelInfo(k, CL_KERNEL_FUNCTION_NAME, len, data.data(), nullptr);
            throw std::system_error(error,
                                    opencl_error_category(),
                                    fmt::format("Error setting kernel argument {} with size {} for kernel {}",
                                                arg_index,
                                                arg_size,
                                                std::string(data.begin(), data.end())));
        }
    }
}  // namespace opencl
}  // namespace utility

#endif  // UTILITY_OPENCL_OPENCL_HPP

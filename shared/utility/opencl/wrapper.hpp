#ifndef UTILITY_OPENCL_WRAPPER_HPP
#define UTILITY_OPENCL_WRAPPER_HPP

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif  // !__APPLE__

namespace utility {
namespace opencl {
    template <typename T>
    struct opencl_wrapper : public std::shared_ptr<std::remove_reference_t<decltype(*std::declval<T>())>> {
        using std::shared_ptr<std::remove_reference_t<decltype(*std::declval<T>())>>::shared_ptr;

        T* operator&() {
            ptr = this->get();
            return &ptr;
        }

        operator T() const {
            return this->get();
        }

        size_t size() const {
            return sizeof(T);
        }

    private:
        T ptr = nullptr;
    };

    using command_queue = opencl_wrapper<::cl_command_queue>;
    using context       = opencl_wrapper<::cl_context>;
    using event         = opencl_wrapper<::cl_event>;
    using kernel        = opencl_wrapper<::cl_kernel>;
    using mem           = opencl_wrapper<::cl_mem>;
    using program       = opencl_wrapper<::cl_program>;
}  // namespace opencl
}  // namespace utility

#endif  // UTILITY_OPENCL_WRAPPER_HPP

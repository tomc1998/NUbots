/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef EXTENSION_OPENCL_H
#define EXTENSION_OPENCL_H

#include <fmt/format.h>
#include <cstdlib>
#include <nuclear>
#include <sstream>

#if defined(__APPLE__) || defined(__MACOSX)
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif  // !__APPLE__

#include "FileWatch.h"

#include "utility/file/fileutil.h"
#include "utility/opencl/opencl.hpp"
#include "utility/opencl/wrapper.hpp"
#include "utility/strutil/strutil.h"

/**
 * TODO document
 *
 * @author Alex Biddulph
 */
namespace extension {
struct OpenCL {
    OpenCL(utility::opencl::program& program, utility::opencl::context& context) : program(program), context(context) {}
    utility::opencl::program program;
    utility::opencl::context context;
};
}  // namespace extension

// NUClear configuration extension
namespace NUClear {
namespace dsl {
    namespace operation {
        template <>
        struct DSLProxy<::extension::OpenCL> {
            template <typename DSL>
            static inline void bind(const std::shared_ptr<threading::Reaction>& reaction,
                                    const std::string& path,
                                    utility::opencl::context& context,
                                    const std::string& compile_flags = std::string()) {
                auto flags = ::extension::FileWatch::ATTRIBUTE_MODIFIED | ::extension::FileWatch::CREATED
                             | ::extension::FileWatch::UPDATED | ::extension::FileWatch::MOVED_TO;

                // Set paths to the config files.
                std::string kernel_path = fmt::format("opencl/{}", path);

                if (!utility::file::exists(kernel_path)) {
                    throw std::runtime_error(fmt::format("OpenCL file '{}' does not exist.", kernel_path));
                }

                // Bind our default path
                DSLProxy<::extension::FileWatch>::bind<DSL>(
                    reaction,
                    kernel_path,
                    flags,
                    std::static_pointer_cast<void>(std::make_shared<utility::opencl::OpenCLBuildContext>(
                        context, fmt::format("{} -Iopencl", compile_flags))));
            }

            template <typename DSL>
            static inline std::shared_ptr<::extension::OpenCL> get(threading::Reaction& t) {
                // Get the file watch event
                ::extension::FileWatch watch = DSLProxy<::extension::FileWatch>::get<DSL>(t);

                // Check if the watch is valid
                if (watch && utility::strutil::endsWith(watch.path, ".cl")) {
                    // Return our compiled OpenCL program
                    auto ctx = *std::static_pointer_cast<utility::opencl::OpenCLBuildContext>(watch.user_data);
                    auto program = utility::opencl::compile(utility::file::loadFromFile(watch.path), ctx);
                    return std::make_shared<::extension::OpenCL>(program, ctx.ctx);
                }
                else {
                    // Return an empty configuration (which will show up invalid)
                    return std::shared_ptr<::extension::OpenCL>(nullptr);
                }
            }
        };
    }  // namespace operation

    // OpenCL is transient
    namespace trait {
        template <>
        struct is_transient<std::shared_ptr<::extension::OpenCL>> : public std::true_type {};
    }  // namespace trait
}  // namespace dsl
}  // namespace NUClear

#endif  // EXTENSION_OPENCL_H

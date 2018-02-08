/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_SPARSEIMAGEALIGNMENT_H
#define MODULES_VISION_SPARSEIMAGEALIGNMENT_H

#include <nuclear>
#include "message/input/Image.h"
#include "message/input/CameraParameters.h"

namespace module {
namespace vision {

	class SparseImageAlignment {
	public:
		double sparseImageAlignment(const message::input::Image& newImage, const message::input::CameraParameters& cam);


	private:
		
		

	};
	
}  // namespace vision
}  // namespace module


#endif
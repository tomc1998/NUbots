/*
 * This file is part of SegmentedRegion.
 *
 * SegmentedRegion is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SegmentedRegion is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SegmentedRegion.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_SEGMENTEDREGION_H
#define MODULES_VISION_SEGMENTEDREGION_H

#include <nuclear> 
#include <string>
#include <vector>

#include "ColourSegment.h"

namespace modules {
    namespace vision {

        /**
         * Stores the segments for a region.
         *
         * @author Alex Biddulph
         */
        class SegmentedRegion {
//		friend class SegmentFilter;
		
		public:
		    /**
		     * The possible alignment for segments in a region.
		     */		
			enum ScanDirection {
				VERTICAL,
				HORIZONTAL
			};
		
			SegmentedRegion();
			SegmentedRegion(const SegmentedRegion& other);
			SegmentedRegion(const std::vector<std::vector<ColourSegment>>& segmented_scans, ScanDirection direction);
		
			/**
			  * Sets the segments and direction of this region.
			  * @param segmentedScans A 2D vector of segments.
			  * @param direction The alignment of the segments in this region (vertical or horizontal).
			  */
			void set(const std::vector<std::vector<ColourSegment>>& segmentedScans, ScanDirection direction);

			/**
			  * Determine whether this segmented region contains any segments.
			  */
			bool empty() const;
		
			//consider removing later and replacing with iterator
			//! Returns a const reference to the segments.
			const std::vector<std::vector<ColourSegment>>& getSegments() const;

			//! Returns the number of segments in the region.
			size_t getNumberOfScans() const;
			
			//! Returns the alignment of the scans.
			ScanDirection getDirection() const;

		private:
			std::vector<std::vector<ColourSegment>> m_segmentedScans; 		//! @variable The segments in this region.
			ScanDirection m_direction;										//! The alignment of the scans in this region.
        };
    
    }  // vision
}  // modules

#endif  // MODULES_VISION_SEGMENTEDREGION_H

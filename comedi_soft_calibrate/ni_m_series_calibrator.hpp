/*	
	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef _NI_M_SERIES_CALIBRATOR_HPP
#define _NI_M_SERIES_CALIBRATOR_HPP

#include "calibrator.hpp"
#include <string>
#include <vector>

namespace NIMSeries
{
	/* Calibrator for National Instruments M-Series boards. */
	class Calibrator: public ::Calibrator
	{
	public:
		Calibrator();
		CalibrationSet calibrate(const std::string &deviceName);
	private:
	};
};

#endif	// _NI_M_SERIES_CALIBRATOR_HPP

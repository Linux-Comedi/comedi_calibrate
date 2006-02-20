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

/* Calibrator for National Instruments M-Series boards. */
class NIMSeriesCalibrator: public Calibrator
{
public:
	NIMSeriesCalibrator();
	virtual std::string supportedDriverName() const {return "ni_pcimio";}
	virtual std::vector<std::string> supportedDeviceNames() const;
private:
};

#endif	// _NI_M_SERIES_CALIBRATOR_HPP

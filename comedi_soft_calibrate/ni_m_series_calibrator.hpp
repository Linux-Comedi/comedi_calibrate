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
		virtual std::string supportedDriverName() const {return "ni_pcimio";}
		virtual std::vector<std::string> supportedDeviceNames() const;
		virtual CalibrationSet calibrate(comedi_t *dev, const std::string &deviceName);
	private:
		static const int positive_cal_shift = 7;
		static const int negative_cal_shift = 10;
		enum positive_cal_source
		{
			POS_CAL_GROUND = 0 << positive_cal_shift,
			POS_CAL_REF = 2 << positive_cal_shift,
			POS_CAL_PWM_500mV = 3 << positive_cal_shift,
			POS_CAL_PWM_2V = 4 << positive_cal_shift,
			POS_CAL_PWM_10V = 5 << positive_cal_shift,
			POS_CAL_AO = 7 << positive_cal_shift
		};
		enum negative_cal_source
		{
			NEG_CAL_GROUND = 0 << negative_cal_shift,
			NEG_CAL_PWM_10V = 7 << negative_cal_shift,
		};
		void setPWMRef(int high_ns, int low_ns);
		comedi_t *_dev;
	};
};

#endif	// _NI_M_SERIES_CALIBRATOR_HPP

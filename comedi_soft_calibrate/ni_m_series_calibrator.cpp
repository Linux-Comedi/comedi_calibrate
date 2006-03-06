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

#include "ni_m_series_calibrator.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

NIMSeries::Calibrator::Calibrator(): ::Calibrator()
{
}
std::vector<std::string> NIMSeries::Calibrator::supportedDeviceNames() const
{
	std::vector<std::string> supportedDeviceNames;
	supportedDeviceNames.push_back("pci-6220");
	supportedDeviceNames.push_back("pci-6221");
	supportedDeviceNames.push_back("pci-6221_37pin");
	supportedDeviceNames.push_back("pci-6224");
	supportedDeviceNames.push_back("pci-6225");
	supportedDeviceNames.push_back("pci-6229");
	supportedDeviceNames.push_back("pci-6250");
	supportedDeviceNames.push_back("pci-6251");
	supportedDeviceNames.push_back("pci-6254");
	supportedDeviceNames.push_back("pci-6259");
	supportedDeviceNames.push_back("pci-6280");
	supportedDeviceNames.push_back("pci-6281");
	supportedDeviceNames.push_back("pci-6284");
	supportedDeviceNames.push_back("pci-6289");
	return supportedDeviceNames;
}

CalibrationSet NIMSeries::Calibrator::calibrate(comedi_t *dev, const std::string &boardName)
{
	std::cerr << __FUNCTION__ << ": " << boardName << std::endl;
	_dev = dev;
	CalibrationSet calibration;
	return calibration;
}

// Private functions
		
void NIMSeries::Calibrator::setPWMRef(int high_ns, int low_ns)
{
	comedi_insn pwm_insn;
	int pwm_subdev = comedi_find_subdevice_by_type(_dev, COMEDI_SUBD_CALIB, 0);
	if(pwm_subdev < 0)
	{
		throw std::runtime_error("Failed to find PWM subdevice.");
	}
	memset(&pwm_insn, 0, sizeof(pwm_insn));
	pwm_insn.insn = INSN_CONFIG;
	pwm_insn.n = 4;
	pwm_insn.subdev = pwm_subdev;
	std::vector<lsampl_t> config_data(pwm_insn.n);
	config_data.at(0) = INSN_CONFIG_PWM_OUTPUT;
	config_data.at(1) = TRIG_ROUND_NEAREST;
	config_data.at(2) = high_ns;
	config_data.at(3) = low_ns;
	pwm_insn.data = &config_data.at(0);
	int retval = comedi_do_insn(_dev, &pwm_insn);
	if(retval < 0)
	{
		throw std::runtime_error("PWM config insn failed.");
	}
}

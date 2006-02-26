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
#include <string>
#include <vector>

NIMSeries::Calibrator::Calibrator(): ::Calibrator()
{
}

CalibrationSet NIMSeries::Calibrator::calibrate(const std::string &boardName)
{
	CalibrationSet calibration;
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
	return calibration;
}

// Private functions

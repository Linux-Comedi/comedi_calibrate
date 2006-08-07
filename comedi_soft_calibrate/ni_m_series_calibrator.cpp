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

#include <boost/array.hpp>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

NIMSeries::Calibrator::Calibrator(): ::Calibrator(), _dev(0)
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
	NIMSeries::References references(dev);
	references.setReference(NIMSeries::References::POS_CAL_PWM_10V, NIMSeries::References::NEG_CAL_GROUND);
	static const int masterClockPeriodNanoSec = 50;
	static const int pulseWidthIncrement = 0x20;
	static const int incrementsPerPulse = 30;
	static const int settleNanoSec = 1000000;
	int i;
	for(i = 1; i < incrementsPerPulse; ++i)
	{
		references.setPWM(masterClockPeriodNanoSec * pulseWidthIncrement * i,
			masterClockPeriodNanoSec * pulseWidthIncrement * (incrementsPerPulse - i));
		std::vector<double> readings = references.readReference(10, 0, settleNanoSec);
		std::cout << "i = " << i << "\n";
		std::vector<double>::const_iterator it;
		for(it = readings.begin(); it != readings.end(); ++it)
		{
			std::cout << "\t" << *it << "\n";
		}
	}
	CalibrationSet calibration;
	return calibration;
}

// Private functions

// Reference

NIMSeries::References::References(comedi_t *dev): _dev(dev)
{
}

void NIMSeries::References::setPWM(int high_ns, int low_ns)
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

void NIMSeries::References::setReference(enum PositiveCalSource posSource, enum NegativeCalSource negSource)
{
	comedi_insn referenceSourceConfig;
	boost::array<lsampl_t, 2> refData;
	memset(&referenceSourceConfig, 0, sizeof(referenceSourceConfig));
	referenceSourceConfig.insn = INSN_CONFIG;
	referenceSourceConfig.n = refData.size();
	referenceSourceConfig.subdev = ADSubdev();
	refData.at(0) = INSN_CONFIG_ALT_SOURCE;
	refData.at(1) = posSource | negSource;
	referenceSourceConfig.data = &refData.at(0);
	int retval = comedi_do_insn(_dev, &referenceSourceConfig);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_do_insn failed, retval=" << retval << " .";
		throw std::runtime_error(message.str());
	}
}

std::vector<double> NIMSeries::References::readReference(unsigned numSamples, unsigned inputRange, unsigned settleNanoSec) const
{
	if(numSamples <= 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": invalid numSamples=" << numSamples << " .";
		throw std::invalid_argument(message.str());
	}
	if(settleNanoSec >= 1000000000)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": invalid settleNanoSec=" << settleNanoSec << " .";
		throw std::invalid_argument(message.str());
	}
	int ret = comedi_data_read_hint(_dev, ADSubdev(), 0 | CR_ALT_SOURCE, inputRange, AREF_DIFF);
	if(ret < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_data_read_hint() failed, return value=" << ret << " .";
		throw std::runtime_error(message.str());
	}
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = settleNanoSec;
	if(nanosleep(&req, 0))
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": nanosleep() returned error, errno=" << errno << std::endl;
		throw std::runtime_error(message.str());
	}
	std::vector<lsampl_t> rawData(numSamples);
	ret = comedi_data_read_n(_dev, ADSubdev(), 0 | CR_ALT_SOURCE, inputRange, AREF_DIFF, &rawData.at(0), rawData.size());
	if(ret < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_data_read_n() failed, return value=" << ret << " .";
		throw std::runtime_error(message.str());
	}
	std::vector<double> volts;
	std::vector<lsampl_t>::const_iterator it;
	lsampl_t maxData = comedi_get_maxdata(_dev, ADSubdev(), 0);
	if(maxData == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_maxdata() failed.";
		throw std::runtime_error(message.str());
	}
	comedi_range *cRange = comedi_get_range(_dev, ADSubdev(), 0, inputRange);
	if(cRange == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_range() failed.";
		throw std::runtime_error(message.str());
	}
	for(it = rawData.begin(); it != rawData.end(); ++it)
	{
		double value = comedi_to_phys(*it, cRange, maxData);
		volts.push_back(value);
	}
	return volts;
}

int NIMSeries::References::ADSubdev() const
{
	int subdev = comedi_find_subdevice_by_type(_dev, COMEDI_SUBD_AI, 0);
	if(subdev < 0)
	{
		throw std::runtime_error("Failed to find AI subdevice.");
	}
	return subdev;
}

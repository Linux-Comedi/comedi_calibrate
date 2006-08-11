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
#include "calibrator_misc.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
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

CalibrationSet NIMSeries::Calibrator::calibrate(boost::shared_ptr<comedi::Device> dev)
{
	_dev = dev;
	_references.reset(new NIMSeries::References(_dev));
	std::map<unsigned, double> PWMCharacterization = characterizePWM();
	Polynomial nonlinearityCorrection = calibrateAINonlinearity(PWMCharacterization);
	const unsigned numAIRanges = _dev->getNRanges(_dev->findSubdeviceByType(COMEDI_SUBD_AI));
	std::vector<Polynomial> AICalibrations(numAIRanges);
	AICalibrations.at(baseRange) = calibrateAIBaseRange(nonlinearityCorrection);
	Polynomial PWMCalibration = calibratePWM(PWMCharacterization, AICalibrations.at(baseRange));

	CalibrationSet calibration;
	return calibration;
}

// Private functions

Polynomial NIMSeries::Calibrator::calibrateAINonlinearity(const std::map<unsigned, double> &PWMCharacterization)
{
	lsampl_t maxData = _dev->maxData(_dev->findSubdeviceByType(COMEDI_SUBD_AI));
	std::map<unsigned, double>::const_iterator it;
	std::vector<double> nominalCodes;
	std::vector<double> measuredCodes;
	for(it = PWMCharacterization.begin(); it != PWMCharacterization.end() ; ++it)
	{
		const unsigned upTicks = it->first;
		const unsigned downTicks = PWMPeriodTicks - upTicks;
		nominalCodes.push_back((0. * upTicks + static_cast<double>(maxData) * downTicks) / PWMPeriodTicks);
		measuredCodes.push_back(it->second);
	}
	Polynomial fit;
	fit.expansionOrigin = maxData / 2;
	fit.coefficients = fitPolynomial(measuredCodes, nominalCodes, fit.expansionOrigin, 3);
	std::cout << "polynomial fit:\n";
	std::cout << "\torigin = " << fit.expansionOrigin << "\n";
	unsigned j;
	for(j = 0; j < fit.coefficients.size(); ++j)
		std::cout << "\torder "<< j << " = " << fit.coefficients.at(j) << "\n";
	std::cout << std::flush;
	return fit;
}

Polynomial NIMSeries::Calibrator::calibrateAIBaseRange(const Polynomial &nonlinearityCorrection)
{
	_references->setReference(References::POS_CAL_GROUND, References::NEG_CAL_GROUND);
	std::vector<double> readings = _references->readReferenceDouble(numSamples, baseRange, settleNanoSec);
	const double measuredGroundCode = estimateMean(readings);
	const double linearizedGroundCode = nonlinearityCorrection(measuredGroundCode);

	_references->setReference(References::POS_CAL_REF, References::NEG_CAL_GROUND);
	readings = _references->readReferenceDouble(numSamples, baseRange, settleNanoSec);
	const double measuredReferenceCode = estimateMean(readings);
	const double linearizedReferenceCode = nonlinearityCorrection(measuredReferenceCode);

	NIMSeries::EEPROM eeprom(_dev);
	const double referenceVoltage = eeprom.referenceVoltage();
	const double gain = referenceVoltage / (linearizedReferenceCode - linearizedGroundCode);
	Polynomial fullCorrection = nonlinearityCorrection;
	unsigned i;
	for(i = 0; i < fullCorrection.coefficients.size(); ++i)
	{
		fullCorrection.coefficients.at(i) *= gain;
	}
	const double offset = fullCorrection(measuredGroundCode);
	fullCorrection.coefficients.at(0) -= offset;

	std::cout << "eeprom reference=" << referenceVoltage << std::endl;
	std::cout << "measuredGroundCode=" << measuredGroundCode << " linearizedGroundCode=" << linearizedGroundCode << std::endl;
	std::cout << "measuredReferenceCode=" << measuredReferenceCode << " linearizedReferenceCode=" << linearizedReferenceCode << std::endl;
	std::cout << "fullCorrection(measuredGroundCode)=" << fullCorrection(measuredGroundCode) << std::endl;
	std::cout << "fullCorrection(measuredReferenceCode)=" << fullCorrection(measuredReferenceCode) << std::endl;
	return fullCorrection;
}

std::map<unsigned, double> NIMSeries::Calibrator::characterizePWM() const
{
	_references->setReference(NIMSeries::References::POS_CAL_PWM_10V, NIMSeries::References::NEG_CAL_GROUND);
	std::map<unsigned, double> results;
	const unsigned numCalibrationPoints = PWMPeriodTicks / minimumPWMPulseTicks - 1;
	unsigned i;
	for(i = 1; i <= numCalibrationPoints ; ++i)
	{
		/* For 6289, results become nonlinear if upPeriod or downPeriod ever drops below about 1 usec.
			Also, the PWM output is not linear unless you keep (upPeriod + downPeriod) constant. */
		const unsigned upTicks = minimumPWMPulseTicks * i;
		const unsigned upPeriod = upTicks * masterClockPeriodNanoSec;
		const unsigned downPeriod = (PWMPeriodTicks - upTicks) * masterClockPeriodNanoSec;
		unsigned actualUpPeriod, actualDownPeriod;
		_references->setPWM(upPeriod, downPeriod, &actualUpPeriod, &actualDownPeriod);
		assert(upPeriod == actualUpPeriod && downPeriod == actualDownPeriod);
		std::vector<double> readings = _references->readReferenceDouble(numSamples, baseRange, settleNanoSec);
		std::cout << "i = " << i << "\n";
		double mean = estimateMean(readings);
		std::cout << "\testimate of mean = " << mean << "\n";
		std::cout << "\testimate of standard deviation = " << estimateStandardDeviation(readings, mean) << "\n";
		std::cout << "\testimate of standard deviation of mean = " << estimateStandardDeviationOfMean(readings, mean) << "\n";
		results[upTicks] = mean;
	}
	return results;
};

Polynomial NIMSeries::Calibrator::calibratePWM(const std::map<unsigned, double> &PWMCharacterization,
	const Polynomial &baseRangeCalibration)
{
	std::map<unsigned, double>::const_iterator it;
	std::vector<double> upTicks;
	std::vector<double> measuredVoltages;
	for(it = PWMCharacterization.begin(); it != PWMCharacterization.end() ; ++it)
	{
		upTicks.push_back(it->first);
		measuredVoltages.push_back(baseRangeCalibration(it->second));
	}
	//FIXME this only needs to be a 1st order fit
	Polynomial fit;
	fit.expansionOrigin = PWMPeriodTicks / 2;
	fit.coefficients = fitPolynomial(measuredVoltages, upTicks, fit.expansionOrigin, 1);
	return fit;
}

// References

NIMSeries::References::References(boost::shared_ptr<comedi::Device> dev): _dev(dev)
{
}

void NIMSeries::References::setPWM(unsigned high_ns, unsigned low_ns, unsigned *actual_high_ns, unsigned *actual_low_ns)
{
	comedi_insn pwm_insn;
	int pwm_subdev = _dev->findSubdeviceByType(COMEDI_SUBD_CALIB);
	if(pwm_subdev < 0)
	{
		throw std::runtime_error("Failed to find PWM subdevice.");
	}
	memset(&pwm_insn, 0, sizeof(pwm_insn));
	pwm_insn.insn = INSN_CONFIG;
	pwm_insn.n = 5;
	pwm_insn.subdev = pwm_subdev;
	std::vector<lsampl_t> config_data(pwm_insn.n);
	config_data.at(0) = INSN_CONFIG_PWM_OUTPUT;
	config_data.at(1) = TRIG_ROUND_NEAREST;
	config_data.at(2) = high_ns;
	config_data.at(3) = TRIG_ROUND_NEAREST;
	config_data.at(4) = low_ns;
	pwm_insn.data = &config_data.at(0);
	_dev->doInsn(&pwm_insn);
	if(actual_high_ns)
		*actual_high_ns = config_data.at(2);
	if(actual_low_ns)
		*actual_low_ns = config_data.at(4);
}

void NIMSeries::References::setReference(enum PositiveCalSource posSource, enum NegativeCalSource negSource)
{
	comedi_insn referenceSourceConfig;
	boost::array<lsampl_t, 2> refData;
	memset(&referenceSourceConfig, 0, sizeof(referenceSourceConfig));
	referenceSourceConfig.insn = INSN_CONFIG;
	referenceSourceConfig.n = refData.size();
	referenceSourceConfig.subdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	refData.at(0) = INSN_CONFIG_ALT_SOURCE;
	refData.at(1) = posSource | negSource;
	referenceSourceConfig.data = &refData.at(0);
	_dev->doInsn(&referenceSourceConfig);
}

std::vector<lsampl_t> NIMSeries::References::readReference(unsigned numSamples, unsigned inputRange, unsigned settleNanoSec) const
{
	if(settleNanoSec >= 1000000000)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": invalid settleNanoSec=" << settleNanoSec << " .";
		throw std::invalid_argument(message.str());
	}
	unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	_dev->dataReadHint(ADSubdev, 0 | CR_ALT_SOURCE, inputRange, AREF_DIFF);
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = settleNanoSec;
	if(nanosleep(&req, 0))
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": nanosleep() returned error, errno=" << errno << std::endl;
		throw std::runtime_error(message.str());
	}
	return _dev->dataReadN(ADSubdev, 0 | CR_ALT_SOURCE, inputRange, AREF_DIFF, numSamples);
}

std::vector<double> NIMSeries::References::readReferenceDouble(unsigned numSamples, unsigned inputRange, unsigned settleNanoSec) const
{
	std::vector<lsampl_t> rawData = readReference(numSamples, inputRange, settleNanoSec);
	std::vector<double> readings(rawData.size());
	std::copy(rawData.begin(), rawData.end(), readings.begin());
	return readings;
}
// EEPROM

NIMSeries::EEPROM::EEPROM(boost::shared_ptr<comedi::Device> dev): _dev(dev)
{
}

float NIMSeries::EEPROM::referenceVoltage() const
{
	return readFloat(calibrationAreaBaseAddress() + voltageReferenceOffset);
}

// private functions

unsigned NIMSeries::EEPROM::readByte(unsigned address) const
{
	unsigned value = _dev->dataRead(_dev->findSubdeviceByType(COMEDI_SUBD_MEMORY), address, 0, 0);
	assert(value <= 0xff);
	return value;
}

unsigned NIMSeries::EEPROM::readUInt16(unsigned startAddress) const
{
	unsigned value = readByte(startAddress) << 8;
	value |= readByte(startAddress + 1);
	return value;
}

float NIMSeries::EEPROM::readFloat(unsigned startAddress) const
{
	union FloatConverter
	{
		uint32_t integer;
		float floatingPoint;
	};
	union FloatConverter myConverter;
	assert(sizeof(float) == sizeof(uint32_t));
	unsigned address = startAddress;
	myConverter.integer = readByte(address++) << 24;
	myConverter.integer |= readByte(address++) << 16;
	myConverter.integer |= readByte(address++) << 8;
	myConverter.integer |= readByte(address++);
	return myConverter.floatingPoint;
}

unsigned NIMSeries::EEPROM::calibrationAreaBaseAddress() const
{
	return readUInt16(24);
}


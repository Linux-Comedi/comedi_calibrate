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
	std::vector<Polynomial> AICalibrations = calibrateAISubdevice();

	CalibrationSet calibration;
	return calibration;
}

// Private functions

std::vector<Polynomial> NIMSeries::Calibrator::calibrateAISubdevice()
{
	std::map<unsigned, double> PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_10V, baseRange);
	Polynomial nonlinearityCorrection = calibrateAINonlinearity(PWMCharacterization);
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned numAIRanges = _dev->getNRanges(ADSubdev);
	std::vector<Polynomial> AICalibrations(numAIRanges);
	std::vector<bool> calibrated(numAIRanges, false);
	std::cout << "calibrating base range " << baseRange << " ..." << std::endl;
	AICalibrations.at(baseRange) = calibrateAIBaseRange(nonlinearityCorrection);
	calibrated.at(baseRange) = true;
	std::cout << "done." << std::endl;
	Polynomial PWMCalibration = calibratePWM(PWMCharacterization, AICalibrations.at(baseRange));
	// calibrate low-gain ranges
	const double largeRangeThreshold = 1.99;
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_10V, &AICalibrations, &calibrated, largeRangeThreshold);
	// calibrate medium-gain ranges
	unsigned range = smallestCalibratedAIRangeContaining(calibrated, largeRangeThreshold);
	assert(calibrated.at(range) == true);
	PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_2V, range);
	PWMCalibration = calibratePWM(PWMCharacterization, AICalibrations.at(range));
	const double mediumRangeThreshold = 0.499;
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_2V, &AICalibrations, &calibrated, mediumRangeThreshold);
	// calibrate high-gain ranges
	range = smallestCalibratedAIRangeContaining(calibrated, mediumRangeThreshold);
	assert(calibrated.at(range) == true);
	PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_500mV, range);
	PWMCalibration = calibratePWM(PWMCharacterization, AICalibrations.at(range));
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_500mV, &AICalibrations, &calibrated, 0.);
	return AICalibrations;
}

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
	printPolynomial(fit);
	return fit;
}

Polynomial NIMSeries::Calibrator::calibrateAIBaseRange(const Polynomial &nonlinearityCorrection)
{
	NIMSeries::EEPROM eeprom(_dev);
	const double referenceVoltage = eeprom.referenceVoltage();

	Polynomial fullCorrection = calibrateGainAndOffset(nonlinearityCorrection,
		References::POS_CAL_REF, referenceVoltage, baseRange);
	return fullCorrection;
}

Polynomial NIMSeries::Calibrator::calibrateAIRange(const Polynomial &PWMCalibration, const Polynomial &nonlinearityCorrection,
	enum NIMSeries::References::PositiveCalSource posSource, unsigned range)
{
	assert(PWMCalibration.order() == 1);
	Polynomial inversePWMCalibration;
	inversePWMCalibration.expansionOrigin = PWMCalibration.coefficients.at(0);
	inversePWMCalibration.coefficients.resize(2);
	inversePWMCalibration.coefficients.at(0) = PWMCalibration.expansionOrigin;
	inversePWMCalibration.coefficients.at(1) = 1. / PWMCalibration.coefficients.at(1);

	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const comedi_range *cRange = _dev->getRange(ADSubdev, 0, range);
	unsigned upTicks = lrint(inversePWMCalibration(cRange->max * 0.9));
	if(upTicks + minimumPWMPulseTicks > PWMPeriodTicks) upTicks = PWMPeriodTicks - minimumPWMPulseTicks;
	setPWMUpTicks(upTicks);
	const double referenceVoltage = PWMCalibration(upTicks);
	Polynomial fullCorrection = calibrateGainAndOffset(nonlinearityCorrection, posSource, referenceVoltage, range);
	return fullCorrection;
}

std::map<unsigned, double> NIMSeries::Calibrator::characterizePWM(enum NIMSeries::References::PositiveCalSource posReferenceSource, unsigned ADRange)
{
	_references->setReference(posReferenceSource, NIMSeries::References::NEG_CAL_GROUND);
	std::map<unsigned, double> results;
	const unsigned numCalibrationPoints = PWMPeriodTicks / minimumPWMPulseTicks - 1;
	unsigned i;
	for(i = 1; i <= numCalibrationPoints ; ++i)
	{
		/* For 6289, results become nonlinear if upPeriod or downPeriod ever drops below about 1 usec.
			Also, the PWM output is not linear unless you keep (upPeriod + downPeriod) constant. */
		const unsigned upTicks = minimumPWMPulseTicks * i;
		setPWMUpTicks(upTicks);
		std::vector<double> readings = _references->readReferenceDouble(numSamples, ADRange, settleNanoSec);
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
	const Polynomial &ADRangeCalibration)
{
	std::map<unsigned, double>::const_iterator it;
	std::vector<double> upTicks;
	std::vector<double> measuredVoltages;
	for(it = PWMCharacterization.begin(); it != PWMCharacterization.end() ; ++it)
	{
		upTicks.push_back(it->first);
		measuredVoltages.push_back(ADRangeCalibration(it->second));
	}
	Polynomial fit;
	fit.expansionOrigin = PWMPeriodTicks / 2;
	fit.coefficients = fitPolynomial(upTicks, measuredVoltages, fit.expansionOrigin, 1);
	std::cout << "sanity check:\n";
	for(it = PWMCharacterization.begin(); it != PWMCharacterization.end() ; ++it)
	{
		std::cout << "upTicks=" << it->first << " code=" << it->second <<
			" PWMCal=" << fit(it->first) << " ADRangeCal=" << ADRangeCalibration(it->second) << std::endl;
	}
	return fit;
}

void NIMSeries::Calibrator::setPWMUpTicks(unsigned upTicks)
{
	const unsigned upPeriod = upTicks * masterClockPeriodNanoSec;
	const unsigned downPeriod = (PWMPeriodTicks - upTicks) * masterClockPeriodNanoSec;
	unsigned actualUpPeriod, actualDownPeriod;
	_references->setPWM(upPeriod, downPeriod, &actualUpPeriod, &actualDownPeriod);
	assert(upPeriod == actualUpPeriod && downPeriod == actualDownPeriod);
}

Polynomial NIMSeries::Calibrator::calibrateGainAndOffset(const Polynomial &nonlinearityCorrection,
	enum NIMSeries::References::PositiveCalSource posReferenceSource, double referenceVoltage, unsigned range)
{
	_references->setReference(References::POS_CAL_GROUND, References::NEG_CAL_GROUND);
	std::vector<double> readings = _references->readReferenceDouble(numSamples, range, settleNanoSec);
	const double measuredGroundCode = estimateMean(readings);
	const double linearizedGroundCode = nonlinearityCorrection(measuredGroundCode);

	_references->setReference(posReferenceSource, References::NEG_CAL_GROUND);
	readings = _references->readReferenceDouble(numSamples, range, settleNanoSec);
	const double measuredReferenceCode = estimateMean(readings);
	const double linearizedReferenceCode = nonlinearityCorrection(measuredReferenceCode);

	const double gain = referenceVoltage / (linearizedReferenceCode - linearizedGroundCode);
	Polynomial fullCorrection = nonlinearityCorrection;
	unsigned i;
	for(i = 0; i < fullCorrection.coefficients.size(); ++i)
	{
		fullCorrection.coefficients.at(i) *= gain;
	}
	const double offset = fullCorrection(measuredGroundCode);
	fullCorrection.coefficients.at(0) -= offset;
	std::cout << "referenceVoltage=" << referenceVoltage << std::endl;
	std::cout << "measuredGroundCode=" << measuredGroundCode << " linearizedGroundCode=" << linearizedGroundCode << std::endl;
	std::cout << "measuredReferenceCode=" << measuredReferenceCode << " linearizedReferenceCode=" << linearizedReferenceCode << std::endl;
	std::cout << "fullCorrection(measuredGroundCode)=" << fullCorrection(measuredGroundCode) << std::endl;
	std::cout << "fullCorrection(measuredReferenceCode)=" << fullCorrection(measuredReferenceCode) << std::endl;
	printPolynomial(fullCorrection);
	return fullCorrection;
}

unsigned NIMSeries::Calibrator::smallestCalibratedAIRangeContaining(const std::vector<bool> &calibrated, double rangeThreshold)
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned numAIRanges = _dev->getNRanges(ADSubdev);
	unsigned i;
	const comedi_range *smallestCRange = 0;
	unsigned smallestRange;
	for(i = 0; i < numAIRanges; ++i)
	{
		if(calibrated.at(i) == false) continue;
		const comedi_range *cRange = _dev->getRange(ADSubdev, 0, i);
		if(cRange->max > rangeThreshold && (smallestCRange == 0 || cRange->max < smallestCRange->max))
		{
			smallestRange = i;
			smallestCRange = cRange;
		}
	}
	if(smallestCRange == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": no calibrated range with maxium voltage above " << rangeThreshold << "V found.";
		throw std::invalid_argument(message.str());
	}
	return smallestRange;
}

void NIMSeries::Calibrator::calibrateAIRangesAboveThreshold(const Polynomial &PWMCalibration,
	const Polynomial &nonlinearityCorrection, enum NIMSeries::References::PositiveCalSource posReferenceSource,
	std::vector<Polynomial> *AICalibrations, std::vector<bool> *calibrated, double maxRangeThreshold)
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned numAIRanges = _dev->getNRanges(ADSubdev);
	unsigned i;
	for(i = 0; i < numAIRanges; ++i)
	{
		if(calibrated->at(i) == true) continue;
		const comedi_range *cRange = _dev->getRange(ADSubdev, 0, i);
		if(cRange->max < maxRangeThreshold) continue;
		std::cout << "calibrating range " << i << " ..." << std::endl;
		AICalibrations->at(i) = calibrateAIRange(PWMCalibration, nonlinearityCorrection,
			posReferenceSource, i);
		calibrated->at(i) = true;
		std::cout << "done." << std::endl;
	}
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
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	_dev->dataReadHint(ADSubdev, 0 | CR_ALT_SOURCE | CR_DITHER, inputRange, AREF_DIFF);
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = settleNanoSec;
	if(nanosleep(&req, 0))
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": nanosleep() returned error, errno=" << errno << std::endl;
		throw std::runtime_error(message.str());
	}
	return _dev->dataReadN(ADSubdev, 0 | CR_ALT_SOURCE | CR_DITHER, inputRange, AREF_DIFF, numSamples);
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


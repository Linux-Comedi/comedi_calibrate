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
#include <ext/stdio_filebuf.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <unistd.h>
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
	supportedDeviceNames.push_back("pcie-6259");
	supportedDeviceNames.push_back("pci-6280");
	supportedDeviceNames.push_back("pci-6281");
	supportedDeviceNames.push_back("pxi-6281");
	supportedDeviceNames.push_back("pci-6284");
	supportedDeviceNames.push_back("pci-6289");
	return supportedDeviceNames;
}

CalibrationSet NIMSeries::Calibrator::calibrate(boost::shared_ptr<comedi::Device> dev)
{
	_dev = dev;
	_references.reset(new NIMSeries::References(_dev));
	CalibrationSet calibration;
	const unsigned AISubdevice = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	calibration[AISubdevice] = calibrateAISubdevice();
	const unsigned AOSubdevice = _dev->findSubdeviceByType(COMEDI_SUBD_AO);
	calibration[AOSubdevice] = calibrateAOSubdevice(calibration[AISubdevice]);

	return calibration;
}

// Private functions

const SubdeviceCalibration NIMSeries::Calibrator::calibrateAISubdevice()
{
	checkAIBufferSize();
	std::map<unsigned, double> PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_10V, baseRange);
	Polynomial nonlinearityCorrection = calibrateAINonlinearity(PWMCharacterization);
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned numAIRanges = _dev->nRanges(ADSubdev);
	SubdeviceCalibration AICalibration(true);
	std::vector<bool> calibrated(numAIRanges, false);
	std::cout << "calibrating base range " << baseRange << " ..." << std::endl;
	AICalibration.insertPolynomial(calibrateAIBaseRange(nonlinearityCorrection), SubdeviceCalibration::allChannels, baseRange);
	calibrated.at(baseRange) = true;
	std::cout << "done." << std::endl;
	Polynomial PWMCalibration = calibratePWM(PWMCharacterization, AICalibration.polynomial(0, baseRange));
	// calibrate low-gain ranges
	const double largeRangeThreshold = 1.99;
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_10V, &AICalibration, &calibrated, largeRangeThreshold);
	// calibrate medium-gain ranges
	unsigned range = smallestCalibratedAIRangeContaining(calibrated, largeRangeThreshold);
	assert(calibrated.at(range) == true);
	PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_2V, range);
	PWMCalibration = calibratePWM(PWMCharacterization, AICalibration.polynomial(0, range));
	const double mediumRangeThreshold = 0.499;
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_2V, &AICalibration, &calibrated, mediumRangeThreshold);
	// calibrate high-gain ranges
	range = smallestCalibratedAIRangeContaining(calibrated, mediumRangeThreshold);
	assert(calibrated.at(range) == true);
	PWMCharacterization = characterizePWM(NIMSeries::References::POS_CAL_PWM_500mV, range);
	PWMCalibration = calibratePWM(PWMCharacterization, AICalibration.polynomial(0, range));
	calibrateAIRangesAboveThreshold(PWMCalibration, nonlinearityCorrection,
		NIMSeries::References::POS_CAL_PWM_500mV, &AICalibration, &calibrated, 0.);
	return AICalibration;
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
		const unsigned downTicks = PWMPeriodTicks() - upTicks;
		nominalCodes.push_back((0. * upTicks + static_cast<double>(maxData) * downTicks) / PWMPeriodTicks());
		measuredCodes.push_back(it->second);
	}
	Polynomial fit;
	fit.expansionOrigin = maxData / 2;
	fit.coefficients = fitPolynomial(nominalCodes, measuredCodes, fit.expansionOrigin, 3);
	printPolynomial(fit);
	return fit;
}

Polynomial NIMSeries::Calibrator::calibrateAIBaseRange(const Polynomial &nonlinearityCorrection)
{
	NIMSeries::EEPROM eeprom(_dev);
	const double referenceVoltage = eeprom.referenceVoltage();

	Polynomial fullCorrection = calibrateAIGainAndOffset(nonlinearityCorrection,
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
	if(upTicks + minimumPWMPulseTicks > PWMPeriodTicks()) upTicks = PWMPeriodTicks() - minimumPWMPulseTicks;
	setPWMUpTicks(upTicks);
	const double referenceVoltage = PWMCalibration(upTicks);
	Polynomial fullCorrection = calibrateAIGainAndOffset(nonlinearityCorrection, posSource, referenceVoltage, range);
	return fullCorrection;
}

std::map<unsigned, double> NIMSeries::Calibrator::characterizePWM(enum NIMSeries::References::PositiveCalSource posReferenceSource, unsigned ADRange)
{
	_references->setReference(posReferenceSource, NIMSeries::References::NEG_CAL_GROUND);
	std::map<unsigned, double> results;
	const unsigned numCalibrationPoints = TargetPWMPeriodTicks / minimumPWMPulseTicks - 1;
	unsigned i;
	for(i = 1; i <= numCalibrationPoints ; ++i)
	{
		/* For 6289, results become nonlinear if upPeriod or downPeriod ever drops below about 1 usec.
			Also, the PWM output is not linear unless you keep (upPeriod + downPeriod) constant. */
		const unsigned upTicks = minimumPWMPulseTicks * i;
		setPWMUpTicks(upTicks);
		std::vector<double> readings = _references->readReferenceDouble(PWMRoundedNumSamples(numSamples, _references->getMinSamplePeriodNanosec()),
			_references->getMinSamplePeriodNanosec(), ADRange, settleNanosec);
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
	fit.expansionOrigin = PWMPeriodTicks() / 2;
	fit.coefficients = fitPolynomial(upTicks, measuredVoltages, fit.expansionOrigin, 1);
	std::cout << "sanity check:\n";
	const double approxVoltsPerBit = ADRangeCalibration(1) - ADRangeCalibration(0);
	for(it = PWMCharacterization.begin(); it != PWMCharacterization.end() ; ++it)
	{
		const double PWMCal = fit(it->first);
		const double ADRangeCal = ADRangeCalibration(it->second);
		const double LSBError = (ADRangeCal - PWMCal) / approxVoltsPerBit;
		std::cout << "upTicks=" << it->first << " code=" << it->second <<
			" PWMCal=" << PWMCal << " ADRangeCal=" << ADRangeCal << " LSBError=" << LSBError << std::endl;
	}
	return fit;
}

void NIMSeries::Calibrator::setPWMUpTicks(unsigned upTicks)
{
	const unsigned upPeriod = upTicks * masterClockPeriodNanosec;
	const unsigned downPeriod = (PWMPeriodTicks() - upTicks) * masterClockPeriodNanosec;
	unsigned actualUpPeriod, actualDownPeriod;
	_references->setPWM(upPeriod, downPeriod, &actualUpPeriod, &actualDownPeriod);
	assert(upPeriod == actualUpPeriod && downPeriod == actualDownPeriod);
}

Polynomial NIMSeries::Calibrator::calibrateAIGainAndOffset(const Polynomial &nonlinearityCorrection,
	enum NIMSeries::References::PositiveCalSource posReferenceSource, double referenceVoltage, unsigned range)
{
	_references->setReference(References::POS_CAL_GROUND, References::NEG_CAL_GROUND);
	std::vector<double> readings = _references->readReferenceDouble(
		PWMRoundedNumSamples(numSamples, _references->getMinSamplePeriodNanosec()),
		_references->getMinSamplePeriodNanosec(), range, settleNanosec);
	const double measuredGroundCode = estimateMean(readings);
	const double linearizedGroundCode = nonlinearityCorrection(measuredGroundCode);

	_references->setReference(posReferenceSource, References::NEG_CAL_GROUND);
	readings = _references->readReferenceDouble(
		PWMRoundedNumSamples(numSamples, _references->getMinSamplePeriodNanosec()),
		_references->getMinSamplePeriodNanosec(), range, settleNanosec);
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
	const unsigned numAIRanges = _dev->nRanges(ADSubdev);
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
	SubdeviceCalibration *AICalibration, std::vector<bool> *calibrated, double maxRangeThreshold)
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned numAIRanges = _dev->nRanges(ADSubdev);
	unsigned i;
	for(i = 0; i < numAIRanges; ++i)
	{
		if(calibrated->at(i) == true) continue;
		const comedi_range *cRange = _dev->getRange(ADSubdev, 0, i);
		if(cRange->max < maxRangeThreshold) continue;
		std::cout << "calibrating range " << i << " ..." << std::endl;
		AICalibration->insertPolynomial(calibrateAIRange(PWMCalibration, nonlinearityCorrection,
			posReferenceSource, i), SubdeviceCalibration::allChannels, i);
		calibrated->at(i) = true;
		std::cout << "done." << std::endl;
	}
}

unsigned NIMSeries::Calibrator::PWMRoundedNumSamples(unsigned numSamples, unsigned samplePeriodNS) const
{
	unsigned PWMPeriodNS = PWMPeriodTicks() * masterClockPeriodNanosec;
	unsigned totalSamplingPeriod = (((numSamples * samplePeriodNS) + PWMPeriodNS / 2) / PWMPeriodNS) * PWMPeriodNS;
	return totalSamplingPeriod / samplePeriodNS;
}

void NIMSeries::Calibrator::checkAIBufferSize()
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	unsigned bytesPerSample;
	if(_dev->subdeviceFlags(ADSubdev) & SDF_LSAMPL)
	{
		bytesPerSample = sizeof(lsampl_t);
	}else
	{
		bytesPerSample = sizeof(sampl_t);
	}
	const unsigned requiredSize = bytesPerSample * PWMRoundedNumSamples(numSamples, _references->getMinSamplePeriodNanosec());
	if(_dev->maxBufferSize(ADSubdev) < requiredSize)
	{
		std::cerr << "Analog input buffer maximum size is " << _dev->maxBufferSize(ADSubdev) << " bytes, but we want " << requiredSize << " .\n" <<
			"If this fails (it will if you aren't root), you will need to use comedi_config to increase the size of the read buffer." << std::endl;
		_dev->setMaxBufferSize(ADSubdev, requiredSize);
	}
	if(_dev->bufferSize(ADSubdev) < requiredSize)
	{
		_dev->setBufferSize(ADSubdev, requiredSize);
	}
}

/* tweak PWM period to be 1 tick longer than a multiple of the sampling period.
This insures the PWM waveform is not sampled at the same points in its waveform over
and over again, giving a better measure of the average value. */
unsigned NIMSeries::Calibrator::PWMPeriodTicks() const
{
	const unsigned samplePeriodTicks = _references->getMinSamplePeriodNanosec() / masterClockPeriodNanosec;
	// round up to nearest multiple of samplePeriod.
	unsigned ticks = ((TargetPWMPeriodTicks + samplePeriodTicks - 1) / samplePeriodTicks) * samplePeriodTicks;
	return ++ticks;
}

const SubdeviceCalibration NIMSeries::Calibrator::calibrateAOSubdevice(const SubdeviceCalibration &AICalibration)
{
	const unsigned AOSubdevice = _dev->findSubdeviceByType(COMEDI_SUBD_AO);
	const unsigned numAORanges = _dev->nRanges(AOSubdevice);
	const unsigned numAOChannels = _dev->nChannels(AOSubdevice);
	unsigned channel, range;
	SubdeviceCalibration AOCalibrations(false);
	for(channel = 0; channel < numAOChannels; ++channel)
	{
		for(range = 0; range < numAORanges; ++range)
		{
			if(_dev->getRange(AOSubdevice, 0, range)->unit != UNIT_volt) continue;
			const unsigned AIRange = findAIRangeForAO(range);
			Polynomial calibration = calibrateAOChannelAndRange(AICalibration.polynomial(0, AIRange), AIRange, channel, range);
			AOCalibrations.insertPolynomial(calibration, channel, range);
		}
	}
	return AOCalibrations;
}

Polynomial NIMSeries::Calibrator::calibrateAOChannelAndRange(const Polynomial &AICalibration,
	unsigned AIRange, unsigned AOChannel, unsigned AORange)
{
	const unsigned AOSubdevice = _dev->findSubdeviceByType(COMEDI_SUBD_AO);
	_references->setReference(AOChannel);
	std::vector<double> codes;
	std::vector<double> measuredVoltages;

	const lsampl_t lowCode = lrint(_dev->maxData(AOSubdevice) * 0.1);
	codes.push_back(static_cast<double>(lowCode));
	_dev->dataWrite(AOSubdevice, AOChannel, AORange, AREF_GROUND, lowCode);
	std::vector<double> readings = _references->readReferenceDouble(
		numSamples, _references->getMinSamplePeriodNanosec(), AIRange, settleNanosec);
	const double measuredLowCode = estimateMean(readings);
	measuredVoltages.push_back(AICalibration(measuredLowCode));

	_dev->dataWrite(AOSubdevice, AOChannel, AORange, AREF_GROUND, highCode(AIRange, AORange));
	codes.push_back(static_cast<double>(highCode(AIRange, AORange)));
	readings = _references->readReferenceDouble(
		numSamples, _references->getMinSamplePeriodNanosec(), AIRange, settleNanosec);
	const double measuredHighCode = estimateMean(readings);
	measuredVoltages.push_back(AICalibration(measuredHighCode));

	Polynomial fit;
	const comedi_range *AOCRange = _dev->getRange(AOSubdevice, 0, AORange);
	fit.expansionOrigin = 0.;
	fit.coefficients = fitPolynomial(measuredVoltages, codes, fit.expansionOrigin, 1);
	std::cout << "AO calibration for channel " << AOChannel << ", range " << AORange << " .\n";
	unsigned i;
	for(i = 0; i < codes.size(); ++i)
		std::cout << "set ao to " << codes.at(i) << ", measured " << measuredVoltages.at(i) << " .\n";
	printPolynomial(fit);
	return fit;
}

lsampl_t NIMSeries::Calibrator::highCode(unsigned AIRange, unsigned AORange) const
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned DASubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AO);
	const comedi_range *AICRange = _dev->getRange(ADSubdev, 0, AIRange);
	const comedi_range *AOCRange = _dev->getRange(DASubdev, 0, AORange);
	if(AICRange->max >= AOCRange->max) return lrint(_dev->maxData(DASubdev) * 0.9);
	double fractionalCode = (0.9 * AICRange->max - AOCRange->min) / (AOCRange->max - AOCRange->min);
	assert(fractionalCode >= 0. && fractionalCode <= 1.);
	return lrint(_dev->maxData(DASubdev) * fractionalCode);
}

unsigned NIMSeries::Calibrator::findAIRangeForAO(unsigned AORange) const
{
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	const unsigned DASubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AO);
	const unsigned numAIRanges = _dev->nRanges(ADSubdev);
	const double maxAOVoltage = _dev->getRange(DASubdev, 0, AORange)->max;
	unsigned i;
	const comedi_range *AICRange = 0;
	unsigned AIRange;
	for(i = 0; i < numAIRanges; ++i)
	{
		const comedi_range *cRange = _dev->getRange(ADSubdev, 0, i);
		if(AICRange == 0 ||
			(cRange->max >= maxAOVoltage && cRange->max < AICRange->max) ||
			(AICRange->max < maxAOVoltage && cRange->max > AICRange->max))
		{
			AIRange = i;
			AICRange = cRange;
		}
	}
	if(AICRange == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": failed to find AI range appropriate for AO range " << AORange << " .";
		throw std::logic_error(message.str());
	}
	return AIRange;
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
	setReferenceBits(posSource | negSource);
}

void NIMSeries::References::setReference(unsigned AOChannel)
{
	assert((AOChannel & 0xf) == AOChannel);
	setReferenceBits(POS_CAL_AO | NEG_CAL_GROUND | (AOChannel << 15));
}

std::vector<lsampl_t> NIMSeries::References::readReference(unsigned numSamples, unsigned samplePeriodNS,
	unsigned inputRange, unsigned settleNanosec) const
{
	if(settleNanosec >= 1000000000)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": invalid settleNanosec=" << settleNanosec << " .";
		throw std::invalid_argument(message.str());
	}
	if(numSamples < 1)
	{
		return std::vector<lsampl_t>();
	}
	const unsigned ADSubdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	_dev->dataReadHint(ADSubdev, 0 | CR_ALT_SOURCE | CR_ALT_FILTER, inputRange, AREF_DIFF);
	struct timespec req;
	req.tv_sec = 0;
	req.tv_nsec = settleNanosec;
	if(nanosleep(&req, 0))
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": nanosleep() returned error, errno=" << errno << std::endl;
		throw std::runtime_error(message.str());
	}
	comedi_cmd cmd;
	memset(&cmd, 0, sizeof(cmd));
	static const unsigned numChannels = 1;
	cmd.subdev = ADSubdev;
	cmd.start_src = TRIG_NOW;
	cmd.scan_begin_src = TRIG_TIMER;
	cmd.scan_begin_arg = samplePeriodNS;
	cmd.convert_src = TRIG_TIMER;
	cmd.convert_arg = 0;
	cmd.scan_end_src = TRIG_COUNT;
	cmd.scan_end_arg = numChannels;
	cmd.stop_src = TRIG_COUNT;
	cmd.stop_arg = numSamples;
	boost::array<unsigned, numChannels> chanlist;
	chanlist.at(0) = CR_PACK(0 | CR_ALT_SOURCE | CR_ALT_FILTER, inputRange, AREF_DIFF);
	cmd.chanlist = &chanlist.at(0);
	cmd.chanlist_len = chanlist.size();
	unsigned i;
	static const unsigned maxTests = 4;
	int retval = 0;
	for(i = 0; i < maxTests; ++i)
	{
		retval = _dev->commandTest(&cmd);
		if(retval == 0) break;
	}
	if(i == maxTests)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_command_test failed, last retval = " << retval;
		throw std::runtime_error(message.str());
	}
	assert(cmd.scan_begin_arg == samplePeriodNS);
	_dev->command(&cmd);
	std::vector<lsampl_t> longData(numSamples);
	unsigned samplesRead;
	if(_dev->subdeviceFlags(ADSubdev) & SDF_LSAMPL)
	{
		__gnu_cxx::stdio_filebuf<char> comediFile(_dev->fileno(), std::ios::in, false, static_cast<size_t>(BUFSIZ));
		std::streamsize count = comediFile.sgetn(reinterpret_cast<char*>(&longData.at(0)), numSamples * sizeof(lsampl_t));
		samplesRead = count / sizeof(lsampl_t);
	}else
	{
		__gnu_cxx::stdio_filebuf<char> comediFile(_dev->fileno(), std::ios::in, false, static_cast<size_t>(BUFSIZ));
		std::vector<sampl_t> data(numSamples);
		std::streamsize count = comediFile.sgetn(reinterpret_cast<char*>(&data.at(0)), numSamples * sizeof(sampl_t));
		samplesRead = count / sizeof(sampl_t);
		std::copy(data.begin(), data.end(), longData.begin());
	}
	if(samplesRead != numSamples)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": failed to read " << numSamples << " samples from comedi device file, count = " << samplesRead << std::endl;
		throw std::runtime_error(message.str());
	}
	return longData;
}

std::vector<double> NIMSeries::References::readReferenceDouble(unsigned numSamples, unsigned samplePeriodNS,
	unsigned inputRange, unsigned settleNanosec) const
{
	std::vector<lsampl_t> rawData = readReference(numSamples,
		samplePeriodNS, inputRange, settleNanosec);
	std::vector<double> readings(rawData.size());
	std::copy(rawData.begin(), rawData.end(), readings.begin());
	return readings;
}

unsigned NIMSeries::References::getMinSamplePeriodNanosec() const
{
	comedi_cmd cmd;
	static const unsigned numChannels = 1;
	memset(&cmd, 0, sizeof(cmd));
	cmd.subdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	cmd.start_src = TRIG_NOW;
	cmd.scan_begin_src = TRIG_TIMER;
	cmd.scan_begin_arg = 0;
	cmd.convert_src = TRIG_TIMER;
	cmd.convert_arg = 0;
	cmd.scan_end_src = TRIG_COUNT;
	cmd.scan_end_arg = numChannels;
	cmd.stop_src = TRIG_COUNT;
	cmd.stop_arg = 1;
	unsigned chanlist[] = {0};
	cmd.chanlist = chanlist;
	cmd.chanlist_len = numChannels;
	int retval = _dev->commandTest(&cmd);
	assert(retval == 0 || retval >=3);
	return cmd.scan_begin_arg;
}

// private functions

void NIMSeries::References::setReferenceBits(unsigned bits)
{
	comedi_insn referenceSourceConfig;
	boost::array<lsampl_t, 2> refData;
	memset(&referenceSourceConfig, 0, sizeof(referenceSourceConfig));
	referenceSourceConfig.insn = INSN_CONFIG;
	referenceSourceConfig.n = refData.size();
	referenceSourceConfig.subdev = _dev->findSubdeviceByType(COMEDI_SUBD_AI);
	refData.at(0) = INSN_CONFIG_ALT_SOURCE;
	refData.at(1) = bits;
	referenceSourceConfig.data = &refData.at(0);
	_dev->doInsn(&referenceSourceConfig);
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


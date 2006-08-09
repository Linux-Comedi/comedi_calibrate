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

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include "calibrator.hpp"
#include "comedi_wrapper.hpp"
#include <iostream>
#include "ni_m_series_calibrator.hpp"
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class ComediSoftCalibrateApp
{
public:
	ComediSoftCalibrateApp(int argc, char **argv);
	virtual ~ComediSoftCalibrateApp();
	void exec();
private:
	boost::program_options::options_description desc;
	boost::program_options::variables_map vm;
	std::string _deviceFile;
	boost::shared_ptr<comedi::Device> _comediDev;
	std::vector<boost::shared_ptr<Calibrator> > _calibrators;
};

ComediSoftCalibrateApp::ComediSoftCalibrateApp(int argc, char **argv):
	desc("Allowed options")
{
	desc.add_options()
		("help", "produce this help message and exit")
		("file,f", boost::program_options::value<typeof(_deviceFile)>(&_deviceFile)->default_value("/dev/comedi0"), "device file")
	;
	try
	{
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	}
	catch(const std::exception &err)
	{
		std::cerr << "Caught exception: " << err.what() << std::endl;
		std::cerr << desc << std::endl;
		throw;
	}
	boost::program_options::notify(vm);

	_calibrators.push_back(boost::shared_ptr<Calibrator>(new NIMSeries::Calibrator()));
	_comediDev.reset(new comedi::Device(_deviceFile));
}

ComediSoftCalibrateApp::~ComediSoftCalibrateApp()
{}

void ComediSoftCalibrateApp::exec()
{
	if(vm.count("help"))
	{
		std::cout << desc << std::endl;
		return;
	}
	std::vector<boost::shared_ptr<Calibrator> >::iterator it;
	for(it = _calibrators.begin(); it != _calibrators.end(); ++it)
	{
		if((*it)->supportedDriverName() != _comediDev->driverName()) continue;
		std::vector<std::string> devices = (*it)->supportedDeviceNames();
		std::vector<std::string>::iterator dit;
		for(dit = devices.begin(); dit != devices.end(); ++dit)
		{
			if(*dit == _comediDev->boardName()) break;
		}
		if(dit == devices.end()) continue;
		break;
	}
	if(it == _calibrators.end())
	{
		std::ostringstream message;
		message << "Failed to find calibrator for " << _comediDev->driverName() << " driver.";
		std::cerr << message.str() << std::endl;
		throw std::invalid_argument(message.str().c_str());
	}
	CalibrationSet calibration = (*it)->calibrate(_comediDev);
// 	std::cout << "driver name: " << driverName() << std::endl;
// 	std::cout << "board name: " << _boardName << std::endl;
}

int main(int argc, char **argv)
{
	try
	{
		ComediSoftCalibrateApp app(argc, argv);
		app.exec();
	}
	catch(const std::exception &err)
	{
		std::cerr << "Caught exception: " << err.what() << std::endl;
		return 1;
	}
	return 0;
}

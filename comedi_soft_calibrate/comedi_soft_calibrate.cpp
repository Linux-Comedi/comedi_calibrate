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
#include <comedilib.h>
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
	std::string driverName() const
	{
		return comedi_get_driver_name(_comediDev);
	}
	std::string boardName() const
	{
		return comedi_get_board_name(_comediDev);
	}
	comedi_t *_comediDev;
	std::vector<boost::shared_ptr<Calibrator> > _calibrators;
};

ComediSoftCalibrateApp::ComediSoftCalibrateApp(int argc, char **argv):
	desc("Allowed options"), _comediDev(0)
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

	_comediDev = comedi_open(_deviceFile.c_str());
	if(_comediDev == 0)
	{
		std::ostringstream message;
		message << "comedi_open() failed, with device file name: " << _deviceFile;
		std::cerr << message.str() << std::endl;
		comedi_perror("comedi_open");
		throw std::runtime_error(message.str().c_str());
	}

}

ComediSoftCalibrateApp::~ComediSoftCalibrateApp()
{
	if(_comediDev) comedi_close(_comediDev);
}

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
		if((*it)->supportedDriverName() != driverName()) continue;
		std::vector<std::string> devices = (*it)->supportedDeviceNames();
		std::vector<std::string>::iterator dit;
		for(dit = devices.begin(); dit != devices.end(); ++dit)
		{
			if(*dit == boardName()) break;
		}
		if(dit == devices.end()) continue;
		break;
	}
	if(it == _calibrators.end())
	{
		std::ostringstream message;
		message << "Failed to find calibrator for " << driverName() << " driver.";
		std::cerr << message.str() << std::endl;
		throw std::invalid_argument(message.str().c_str());
	}
	CalibrationSet calibration = (*it)->calibrate(_comediDev, boardName());
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

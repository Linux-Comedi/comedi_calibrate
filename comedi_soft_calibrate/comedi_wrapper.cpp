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

#include "comedi_wrapper.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

comedi::Device::Device(const std::string &deviceFile)
{
	_dev = comedi_open(deviceFile.c_str());
	if(_dev == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_open() failed, with device file name: \"" << deviceFile << "\".";
		std::cerr << message.str() << std::endl;
		comedi_perror("comedi_open");
		throw std::runtime_error(message.str().c_str());
	}
}

comedi::Device::~Device()
{
	comedi_close(_dev);
}

unsigned comedi::Device::findSubdeviceByType(int type, unsigned startSubdevice) const
{
	int subdev = comedi_find_subdevice_by_type(_dev, type, startSubdevice);
	if(subdev < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": failed to find subdevice of type " << type << " .";
		throw std::runtime_error(message.str());
	}
	return subdev;
}

std::string comedi::Device::driverName() const
{
	char *name = comedi_get_driver_name(_dev);
	if(name == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_driver_name() failed.";
		throw std::runtime_error(message.str());
	}
	return name;
}

std::string comedi::Device::boardName() const
{
	char *name = comedi_get_board_name(_dev);
	if(name == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_board_name() failed.";
		throw std::runtime_error(message.str());
	}
	return name;
}

unsigned comedi::Device::bufferSize(unsigned subdevice) const
{
	int retval = comedi_get_buffer_size(_dev, subdevice);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_buffer_size() failed.";
		throw std::runtime_error(message.str());
	}
	return retval;
}

void comedi::Device::command(comedi_cmd *cmd)
{
	int retval = comedi_command(_dev, cmd);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_command() failed, return value=" << retval << " .";
		throw std::runtime_error(message.str());
	}
}

int comedi::Device::commandTest(comedi_cmd *cmd)
{
	return comedi_command_test(_dev, cmd);
}

lsampl_t comedi::Device::dataRead(unsigned subdevice, unsigned channel, unsigned range, unsigned aref)
{
	lsampl_t value;
	int retval = comedi_data_read(_dev, subdevice, channel, range, aref, &value);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_data_read() failed, return value=" << retval << " .";
		throw std::runtime_error(message.str());
	}
	return value;
}

std::vector<lsampl_t> comedi::Device::dataReadN(unsigned subdevice, unsigned channel, unsigned range, unsigned aref, unsigned numSamples)
{
	std::vector<lsampl_t> values(numSamples);
	int retval = comedi_data_read_n(_dev, subdevice, channel, range, aref, &values.at(0), values.size());
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_data_read_n() failed, return value=" << retval << " .";
		throw std::runtime_error(message.str());
	}
	return values;
}

void  comedi::Device::dataReadHint(unsigned subdevice, unsigned channel, unsigned range, unsigned aref)
{
	int ret = comedi_data_read_hint(_dev, subdevice, channel, range, aref);
	if(ret < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_data_read_hint() failed, return value = " << ret << " .";
		throw std::runtime_error(message.str());
	}
}

void comedi::Device::doInsn(comedi_insn *instruction)
{
	int retval = comedi_do_insn(_dev, instruction);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_do_insn() failed.";
		throw std::runtime_error(message.str());
	}
}

int comedi::Device::fileno()
{
	int fd = comedi_fileno(_dev);
	if(fd < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_fileno() failed.";
		throw std::runtime_error(message.str());
	}
	return fd;
}

unsigned comedi::Device::nRanges(unsigned subdevice, unsigned channel) const
{
	int retval = comedi_get_n_ranges(_dev, subdevice, channel);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_n_ranges() failed.";
		throw std::runtime_error(message.str());
	}
	return retval;
}

const comedi_range* comedi::Device::getRange(unsigned subdevice, unsigned channel, unsigned range) const
{
	comedi_range *cRange = comedi_get_range(_dev, subdevice, channel, range);
	if(cRange == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_range() failed.";
		throw std::runtime_error(message.str());
	}
	return cRange;
}

unsigned comedi::Device::maxBufferSize(unsigned subdevice) const
{
	int retval = comedi_get_max_buffer_size(_dev, subdevice);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_max_buffer_size() failed.";
		throw std::runtime_error(message.str());
	}
	return retval;
}

lsampl_t comedi::Device::maxData(unsigned subdevice, unsigned channel) const
{
	lsampl_t value = comedi_get_maxdata(_dev, subdevice, 0);
	if(value == 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_maxdata() failed.";
		throw std::runtime_error(message.str());
	}
	return value;
}

void comedi::Device::setBufferSize(unsigned subdevice, unsigned numBytes)
{
	int retval = comedi_set_buffer_size(_dev, subdevice, numBytes);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_set_buffer_size() failed.";
		throw std::runtime_error(message.str());
	}
}

void comedi::Device::setMaxBufferSize(unsigned subdevice, unsigned numBytes)
{
	int retval = comedi_set_max_buffer_size(_dev, subdevice, numBytes);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_set_max_buffer_size() failed.";
		throw std::runtime_error(message.str());
	}
}

unsigned comedi::Device::subdeviceFlags(unsigned subdevice) const
{
	int retval = comedi_get_subdevice_flags(_dev, subdevice);
	if(retval < 0)
	{
		std::ostringstream message;
		message << __FUNCTION__ << ": comedi_get_subdevice_flags() failed.";
		throw std::runtime_error(message.str());
	}
	return retval;
}

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

#ifndef _COMEDI_WRAPPER_HPP
#define _COMEDI_WRAPPER_HPP

#include <boost/noncopyable.hpp>
#include <comedilib.h>
#include <string>
#include <vector>

namespace comedi
{
	class Device: boost::noncopyable
	{
	public:
		Device(const std::string &deviceFile);
		~Device();
		unsigned findSubdeviceByType(int type, unsigned startSubdevice = 0) const;
		std::string driverName() const;
		std::string boardName() const;
		unsigned bufferSize(unsigned subdevice) const;
		int commandTest(comedi_cmd *cmd);
		void command(comedi_cmd *cmd);
		lsampl_t dataRead(unsigned subdevice, unsigned channel, unsigned range, unsigned aref);
		std::vector<lsampl_t> dataReadN(unsigned subdevice, unsigned channel, unsigned range, unsigned aref, unsigned numSamples);
		void dataWrite(unsigned subdevice, unsigned channel, unsigned range, unsigned aref, lsampl_t data);
		void dataReadHint(unsigned subdevice, unsigned channel, unsigned range, unsigned aref);
		std::string defaultCalibrationPath() const;
		void doInsn(comedi_insn *instruction);
		int fileno();
		unsigned maxBufferSize(unsigned subdevice) const;
		unsigned nChannels(unsigned subdevice) const;
		unsigned nRanges(unsigned subdevice, unsigned channel = 0) const;
		const comedi_range* getRange(unsigned subdevice, unsigned channel, unsigned range) const;
		lsampl_t maxData(unsigned subdevice, unsigned channel = 0) const;
		void setBufferSize(unsigned subdevice, unsigned numBytes);
		void setMaxBufferSize(unsigned subdevice, unsigned numBytes);
		unsigned subdeviceFlags(unsigned subdevice) const;
	private:
		comedi_t *_dev;
	};
};

#endif	// _COMEDI_WRAPPER_HPP

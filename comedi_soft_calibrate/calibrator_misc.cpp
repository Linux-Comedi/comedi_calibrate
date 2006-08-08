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

#include "calibrator_misc.hpp"

#include <cmath>
#include <gsl/gsl_statistics_double.h>

double estimateMean(const std::vector<double> &samples)
{
	return gsl_stats_mean(&samples.at(0), 1, samples.size());
}

double estimateStandardDeviationOfMean(const std::vector<double> &samples, double mean)
{
	double value = gsl_stats_variance_m(&samples.at(0), 1, samples.size(), mean);
	return std::sqrt(value / samples.size());
}

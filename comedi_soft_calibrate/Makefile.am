
bin_PROGRAMS = comedi_soft_calibrate

comedi_soft_calibrate_SOURCES = \
	comedi_soft_calibrate.cpp calibrator.hpp ni_m_series_calibrator.cpp ni_m_series_calibrator.hpp
#comedi_soft_calibrate_CXXFLAGS =
comedi_soft_calibrate_LDADD = -lcomedi -lboost_program_options

install-data-local:
	$(INSTALL) -d $(localstatedir)/lib/comedi/calibrations

EXTRA_DIST = results


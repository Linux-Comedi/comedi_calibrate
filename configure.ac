
AC_INIT([comedi_calibrate], [1])
AC_CONFIG_SRCDIR([comedi_calibrate/comedi_calibrate.c])
AM_INIT_AUTOMAKE

AM_MAINTAINER_MODE

AC_PROG_CC
AM_PROG_CC_STDC
AC_ISC_POSIX
AC_PROG_CXX
AC_PROG_RANLIB

AC_HEADER_STDC([])
AC_CHECK_LIB([m],[cos])

AC_ARG_ENABLE([comedilib-checking],
	[AS_HELP_STRING([--disable-comedilib-checking],
		[Disables checking of comedilib for package maintainers])],
	[ENABLE_COMEDILIBCHECK=$enableval],[ENABLE_COMEDILIBCHECK="yes"])
if test "$ENABLE_COMEDILIBCHECK" == "yes"; then
#comedilib
AC_CHECK_LIB([comedi],[comedi_open],,AC_MSG_ERROR([Failed to find libcomedi.]))
fi

#comedilib headers
AC_CHECK_HEADER([comedilib.h],,AC_MSG_ERROR([Failed to find comedilib.h.]))

# GNU Scientific Library
AC_CHECK_HEADER([gsl/gsl_matrix.h],,AC_MSG_ERROR([Failed to find gsl/gsl_matrix.h]))
AC_CHECK_HEADER([gsl/gsl_multifit.h],,AC_MSG_ERROR([Failed to find gsl/gsl_multifit.h]))
AC_CHECK_HEADER([gsl/gsl_statistics.h],,AC_MSG_ERROR([Failed to find gsl/gsl_statistics.h]))
AC_CHECK_LIB([gslcblas],[cblas_dgemm],,AC_MSG_ERROR([Failed to find libgslcblas.]))
AC_CHECK_LIB([gsl],[gsl_blas_dgemm],,AC_MSG_ERROR([Failed to find libgsl.]))

#boost/program_options
AC_LANG_PUSH(C++)
AC_CHECK_HEADER([boost/program_options.hpp],,AC_MSG_ERROR([Failed to find boost/program_options.hpp]))
AC_CHECK_LIB([boost_program_options],[main],,AC_MSG_ERROR([Failed to find libboost_program_options.])) # cheat!
AC_LANG_POP(C++)

AC_CONFIG_FILES([
Makefile
comedi_calibrate/Makefile
comedi_soft_calibrate/Makefile
libcomedi_calibrate/Makefile
man/Makefile
])

AC_OUTPUT



AC_INIT([comedi_calibrate], [0])
AC_CONFIG_SRCDIR([comedi_calibrate/comedi_calibrate.c])
AM_INIT_AUTOMAKE

AM_MAINTAINER_MODE

AC_PROG_CC
AM_PROG_CC_STDC
AC_ISC_POSIX

AC_HEADER_STDC([])

#comedilib
AC_CHECK_HEADER([comedilib.h],,AC_MSG_ERROR([Failed to find comedilib.h.]))
AC_CHECK_LIB([comedi],[comedi_open],,AC_MSG_ERROR([Failed to find libcomedi.]))

AC_CONFIG_FILES(
Makefile
comedi_calibrate/Makefile
)

AC_OUTPUT

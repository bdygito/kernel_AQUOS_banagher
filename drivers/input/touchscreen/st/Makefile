#
# Makefile for the touchscreen drivers.
#

ifeq ($(CONFIG_TOUCHSCREEN_ST),y)
	obj-m += st_fts.o

	st_fts-y	+=	fts.o		\
					fts_proc.o

	st_fts-y	+=	fts_lib/ftsCompensation.o	\
					fts_lib/ftsCore.o			\
					fts_lib/ftsError.o			\
					fts_lib/ftsFrame.o			\
					fts_lib/ftsIO.o				\
					fts_lib/ftsTest.o			\
					fts_lib/ftsTime.o			\
					fts_lib/ftsTool.o			\
					fts_lib/ftsFlash.o			\
					fts_lib/ftsGesture.o
endif

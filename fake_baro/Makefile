include $(APPDIR)/Make.defs

MODULE    = $(CONFIG_ROCKETALT_FAKE_BARO)
PRIORITY  = $(CONFIG_ROCKETALT_FAKE_BARO_PRIORITY)
STACKSIZE = $(CONFIG_ROCKETALT_FAKE_BARO_STACKSIZE)
PROGNAME  = fake_baro

# Source files

ifeq ($(CONFIG_ROCKETALT_FAKE_BARO_USECSV),y)
MAINSRC  = fake_baro_csv_main.c
endif

ifeq ($(CONFIG_ROCKETALT_FAKE_BARO_USEBLE),y)
MAINSRC  = fake_baro_ble_main.c
endif

ifeq ($(CONFIG_ROCKETALT_FAKE_BARO_USECURVE),y)
MAINSRC  = fake_baro_curve_main.c
endif

include $(APPDIR)/Application.mk

include $(APPDIR)/Make.defs

MODULE    = $(CONFIG_ROCKETALT_PROCESSING)
PRIORITY  = $(CONFIG_ROCKETALT_PROCESSING_PRIORITY)
STACKSIZE = $(CONFIG_ROCKETALT_PROCESSING_STACKSIZE)

# Topics to advertise

MAINSRC  += altitude_main.c
PROGNAME += altitude_fusion

MAINSRC  += events_main.c
PROGNAME += events_topic

MAINSRC  += velocity_main.c
PROGNAME += velocity_fusion

ifeq ($(CONFIG_ROCKETALT_PROCESSING_BATMON),y)
MAINSRC  += adcbat_main.c
PROGNAME += adcbat
endif

include $(APPDIR)/Application.mk

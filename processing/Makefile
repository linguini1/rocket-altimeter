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

include $(APPDIR)/Application.mk

include $(APPDIR)/Make.defs

MODULE    = $(CONFIG_ROCKETALT_PROCESSING)
PRIORITY  = $(CONFIG_ROCKETALT_PROCESSING_PRIORITY)
STACKSIZE = $(CONFIG_ROCKETALT_PROCESSING_STACKSIZE)

# Topics to advertise

ifeq ($(CONFIG_ROCKETALT_PROCESSING_ALTFUSION),y)
MAINSRC  += altitude/altitude_main.c
PROGNAME += altitude_fusion
endif

ifeq ($(CONFIG_ROCKETALT_PROCESSING_EVENTS),y)
MAINSRC  += events/events_main.c
PROGNAME += events_topic
endif

ifeq ($(CONFIG_ROCKETALT_PROCESSING_VELFUSION),y)
MAINSRC  += velocity/velocity_main.c
PROGNAME += velocity_fusion
endif

ifeq ($(CONFIG_ROCKETALT_PROCESSING_BATMON),y)
MAINSRC  += batmon/batmon_main.c
PROGNAME += batmon
endif

ifeq ($(CONFIG_ROCKETALT_PROCESSING_ADCUORB),y)
MAINSRC  += adc_uorb/adc_uorb_main.c
PROGNAME += adc_uorb
endif

ifeq ($(CONFIG_ROCKETALT_PROCESSING_CONTMON),y)
MAINSRC  += continuity/continuity_main.c
PROGNAME += continuity
endif

include $(APPDIR)/Application.mk

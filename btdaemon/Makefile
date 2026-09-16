include $(APPDIR)/Make.defs

MODULE    = $(CONFIG_ROCKETALT_BTDAEMON)
PRIORITY  = $(CONFIG_ROCKETALT_BTDAEMON_PRIORITY)
STACKSIZE = $(CONFIG_ROCKETALT_BTDAEMON_STACKSIZE)

MAINSRC  += btdaemon_main.c
PROGNAME += btdaemon

# Include Nimble logic

include $(APPDIR)/wireless/bluetooth/nimble/Makefile.nimble

include $(APPDIR)/Application.mk

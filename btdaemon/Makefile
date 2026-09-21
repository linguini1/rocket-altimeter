include $(APPDIR)/Make.defs

MODULE    = $(CONFIG_ROCKETALT_BTDAEMON)
PRIORITY  = $(CONFIG_ROCKETALT_BTDAEMON_PRIORITY)
STACKSIZE = $(CONFIG_ROCKETALT_BTDAEMON_STACKSIZE)

PROGNAME += btdaemon

MAINSRC  += btdaemon_main.c
CSRCS    += $(wildcard services/**/*.c) # Service definitions

# Include Nimble logic

include $(APPDIR)/wireless/bluetooth/nimble/Makefile.nimble

include $(APPDIR)/Application.mk

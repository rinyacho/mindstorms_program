APPL_COBJS += sonic_task.o
APPL_COBJS += color_task.o
ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/spp_master_test_api/Makefile

endif

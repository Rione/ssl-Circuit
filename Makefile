MAINBOARD_DIR  = 2026/Firmware/MainBoard/MainBoard_V26_1_1
WHEEL_DIR      = 2026/Firmware/WheelUnit/WheelDriver_V26_3
DRIBBLER_DIR   = 2026/Firmware/Dribbler/Dribbler_V26_1_1

.PHONY: build clean

build:
	@$(call build_if_exists,$(MAINBOARD_DIR))
	@$(call build_if_exists,$(WHEEL_DIR))
	@$(call build_if_exists,$(DRIBBLER_DIR))

clean:
	@$(call clean_if_exists,$(MAINBOARD_DIR))
	@$(call clean_if_exists,$(WHEEL_DIR))
	@$(call clean_if_exists,$(DRIBBLER_DIR))

define build_if_exists
	$(if $(wildcard $(1)/Makefile), \
		$(MAKE) -C $(1) build, \
		echo "skip: $(1) (not found)")
endef

define clean_if_exists
	$(if $(wildcard $(1)/Makefile), \
		$(MAKE) -C $(1) clean, \
		echo "skip: $(1) (not found)")
endef

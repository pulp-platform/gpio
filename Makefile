.DEFAULT_GOAL := help

VENVDIR?=$(WORKDIR)/.venv
REQUIREMENTS_TXT?=$(wildcard requirements.txt)
include Makefile.venv

GPIOS ?= 32

## Regenerate the register file and HAL C-header for a different GPIO count. Usage: make reconfigure GPIOS=128
reconfigure: .bender/git/checkouts | venv
	@echo Reconfiguring IP to use $(GPIOS) gpios...
	@sed -i -r 's/default: "[0-9]+"/default: "${GPIOS}"/g' gpio_regs.hjson
ifeq ($(shell expr $(GPIOS) \<= 16), 1)
	@sed -i -r 's|(//)?`define ENABLE_LESS_THAN_16_GPIOS_REG_PKG_WORKAROUND|`define ENABLE_LESS_THAN_16_GPIOS_REG_PKG_WORKAROUND|g' test/tb_gpio.sv
else
	@sed -i -r 's|(//)?`define ENABLE_LESS_THAN_16_GPIOS_REG_PKG_WORKAROUND|//`define ENABLE_LESS_THAN_16_GPIOS_REG_PKG_WORKAROUND|g' test/tb_gpio.sv
endif
ifeq ($(shell expr $(GPIOS) \<= 32), 1)
	@sed -i -r 's|(//)?`define ENABLE_LESS_THAN_32_GPIOS_REG_PKG_WORKAROUND|`define ENABLE_LESS_THAN_32_GPIOS_REG_PKG_WORKAROUND|g' test/tb_gpio.sv
else
	@sed -i -r 's|(//)?`define ENABLE_LESS_THAN_32_GPIOS_REG_PKG_WORKAROUND|//`define ENABLE_LESS_THAN_32_GPIOS_REG_PKG_WORKAROUND|g' test/tb_gpio.sv
endif
	$(VENV)/python $$(./bender path register_interface)/vendor/lowrisc_opentitan/util/regtool.py gpio_regs.hjson -r -t src -p GPIOCount=${GPIOS}
	$(VENV)/python $$(./bender path register_interface)/vendor/lowrisc_opentitan/util/regtool.py gpio_regs.hjson --cdefines -o hal/gpio_hal.h -p GPIOCount=${GPIOS};
	@echo "Done"

.bender/git/checkouts: bender
	bender update
	bender path register_interface

bender:
ifeq (,$(wildcard ./bender))
ifeq (,$(shell which curl))
$(error 'curl' is not installed on your machine. Please make sure to install it so we can download the bender binary.)
else
	curl --proto '=https' --tlsv1.2 -sSf https://fabianschuiki.github.io/bender/init \
		| bash -s -- 0.26.1
	touch bender
endif
endif


.PHONY: help
help: Makefile
	@printf "GPIO Reconfiguration\n"
	@printf "Use this Makefile to regenerate the register file and HAL C-header for a different number GPIOs than the default one.\n\n"
	@printf "Usage: \n"
	@printf "make reconfigure GPIOS=<desired nr of gpios>\n\n"

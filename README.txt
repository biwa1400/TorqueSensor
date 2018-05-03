1. change sdk_config.h
	NRF_SDH_CLOCK_LF_SRC -> <0=> NRF_CLOCK_LF_SRC_RC 
	NRF_SDH_CLOCK_LF_RC_CTIV -> 10
	NRF_SDH_CLOCK_LF_RC_TEMP_CTIV -> 10

2. changeValue2 after print

3. JLink Pin
	JLink           MCU

	VDD             VCC
	VGT             VCC
	SWDIO           SWDIO
	SWDCLK          SWDCLK
	SWO             P18 (needn't)
	RESET           P21 (needn't)
	GNDDECT         GND 
	GND             GND
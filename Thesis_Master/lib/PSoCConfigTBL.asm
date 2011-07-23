; Generated by PSoC Designer 5.0.985.0
;
include "m8c.inc"
;  Personalization tables 
export LoadConfigTBL_transmitter_config_Bank1
export LoadConfigTBL_transmitter_config_Bank0
export LoadConfigTBL_transmitter_config_Ordered
export UnloadConfigTBL_transmitter_config_Bank1
export UnloadConfigTBL_transmitter_config_Bank0
export ReloadConfigTBL_transmitter_config_Bank1
export ReloadConfigTBL_transmitter_config_Bank0
export LoadConfigTBL_pc_listener_Bank1
export LoadConfigTBL_pc_listener_Bank0
export UnloadConfigTBL_pc_listener_Bank1
export UnloadConfigTBL_pc_listener_Bank0
export LoadConfigTBL_receiver_config_Bank1
export LoadConfigTBL_receiver_config_Bank0
export UnloadConfigTBL_receiver_config_Bank1
export UnloadConfigTBL_receiver_config_Bank0
export UnloadConfigTBL_Total_Bank1
export UnloadConfigTBL_Total_Bank0
AREA lit(rom, rel)
LoadConfigTBL_pc_listener_Bank0:
;  Instance name COMP_SERIAL, User Module UART
;       Instance name COMP_SERIAL, Block Name RX(DCC13)
	db		3fh, 00h		;COMP_SERIAL_RX_CONTROL_REG(DCC13CR0)
	db		3dh, 00h		;COMP_SERIAL_(DCC13DR1)
	db		3eh, 00h		;COMP_SERIAL_RX_BUFFER_REG (DCC13DR2)
;       Instance name COMP_SERIAL, Block Name TX(DCC12)
	db		3bh, 00h		;COMP_SERIAL_TX_CONTROL_REG(DCC12CR0)
	db		39h, 00h		;COMP_SERIAL_TX_BUFFER_REG (DCC12DR1)
	db		3ah, 00h		;COMP_SERIAL_(DCC12DR2)
;  Instance name TX_REPEATER, User Module TX8
;       Instance name TX_REPEATER, Block Name TX8(DCC02)
	db		2bh, 00h		;TX_REPEATER_CONTROL_REG  (DCC02CR0)
	db		29h, 00h		;TX_REPEATER_TX_BUFFER_REG(DCC02DR1)
	db		2ah, 00h		;TX_REPEATER_(DCC02DR2)
	db		ffh
LoadConfigTBL_pc_listener_Bank1:
;  Instance name COMP_SERIAL, User Module UART
;       Instance name COMP_SERIAL, Block Name RX(DCC13)
	db		3fh, 00h		;COMP_SERIAL_(DCC13CR1)
	db		3ch, 05h		;COMP_SERIAL_RX_FUNC_REG   (DCC13FN)
	db		3dh, f6h		;COMP_SERIAL_RX_INPUT_REG  (DCC13IN)
	db		3eh, 80h		;COMP_SERIAL_RX_OUTPUT_REG (DCC13OU)
;       Instance name COMP_SERIAL, Block Name TX(DCC12)
	db		3bh, 00h		;COMP_SERIAL_(DCC12CR1)
	db		38h, 1dh		;COMP_SERIAL_TX_FUNC_REG   (DCC12FN)
	db		39h, 06h		;COMP_SERIAL_TX_INPUT_REG  (DCC12IN)
	db		3ah, 85h		;COMP_SERIAL_TX_OUTPUT_REG (DCC12OU)
;  Instance name TX_REPEATER, User Module TX8
;       Instance name TX_REPEATER, Block Name TX8(DCC02)
	db		2bh, 00h		;TX_REPEATER_(DCC02CR1)
	db		28h, 1dh		;TX_REPEATER_FUNC_REG     (DCC02FN)
	db		29h, 01h		;TX_REPEATER_INPUT_REG    (DCC02IN)
	db		2ah, 84h		;TX_REPEATER_OUTPUT_REG   (DCC02OU)
	db		ffh
UnloadConfigTBL_pc_listener_Bank0:
;  Instance name COMP_SERIAL, User Module UART
;       Instance name COMP_SERIAL, Block Name RX(DCC13)
	db		3fh, 00h		;COMP_SERIAL_CONTROL_0 (DCC13CR0)
;       Instance name COMP_SERIAL, Block Name TX(DCC12)
	db		3bh, 00h		;COMP_SERIAL_CONTROL_0 (DCC12CR0)
;  Instance name TX_REPEATER, User Module TX8
;       Instance name TX_REPEATER, Block Name TX8(DCC02)
	db		2bh, 00h		;TX_REPEATER_CONTROL_0 (DCC02CR0)
	db		ffh
UnloadConfigTBL_pc_listener_Bank1:
;  Instance name COMP_SERIAL, User Module UART
;       Instance name COMP_SERIAL, Block Name RX(DCC13)
	db		3fh, 00h		;COMP_SERIAL_CONTROL_1 (DCC13CR1)
	db		3ch, 00h		;COMP_SERIAL_DIG_BasicFunction (DCC13FN)
	db		3dh, 00h		;COMP_SERIAL_DIG_Input (DCC13IN)
	db		3eh, 00h		;COMP_SERIAL_DIG_Output (DCC13OU)
;       Instance name COMP_SERIAL, Block Name TX(DCC12)
	db		3bh, 00h		;COMP_SERIAL_CONTROL_1 (DCC12CR1)
	db		38h, 00h		;COMP_SERIAL_DIG_BasicFunction (DCC12FN)
	db		39h, 00h		;COMP_SERIAL_DIG_Input (DCC12IN)
	db		3ah, 00h		;COMP_SERIAL_DIG_Output (DCC12OU)
;  Instance name TX_REPEATER, User Module TX8
;       Instance name TX_REPEATER, Block Name TX8(DCC02)
	db		2bh, 00h		;TX_REPEATER_CONTROL_1 (DCC02CR1)
	db		28h, 00h		;TX_REPEATER_DIG_BasicFunction (DCC02FN)
	db		29h, 00h		;TX_REPEATER_DIG_Input (DCC02IN)
	db		2ah, 00h		;TX_REPEATER_DIG_Output (DCC02OU)
	db		ffh

;  Instance name COMP_SERIAL, User Module UART
;       Instance name COMP_SERIAL, Block Name RX(DCC13)
;       Instance name COMP_SERIAL, Block Name TX(DCC12)
;  Instance name TX_REPEATER, User Module TX8
;       Instance name TX_REPEATER, Block Name TX8(DCC02)
	db		ffh
LoadConfigTBL_receiver_config_Bank0:
;  Instance name RECEIVE, User Module RX8
;       Instance name RECEIVE, Block Name RX8(DCC02)
	db		2bh, 00h		;RECEIVE_CONTROL_REG  (DCC02CR0)
	db		29h, 00h		;RECEIVE_(DCC02DR1)
	db		2ah, 00h		;RECEIVE_RX_BUFFER_REG(DCC02DR2)
;  Instance name RX_TIMEOUT, User Module Timer16
;       Instance name RX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;RX_TIMEOUT_CONTROL_LSB_REG(DBC00CR0)
	db		21h, f0h		;RX_TIMEOUT_PERIOD_LSB_REG(DBC00DR1)
	db		22h, 00h		;RX_TIMEOUT_COMPARE_LSB_REG(DBC00DR2)
;       Instance name RX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 04h		;RX_TIMEOUT_CONTROL_MSB_REG(DBC01CR0)
	db		25h, 00h		;RX_TIMEOUT_PERIOD_MSB_REG(DBC01DR1)
	db		26h, 00h		;RX_TIMEOUT_COMPARE_MSB_REG(DBC01DR2)
	db		ffh
LoadConfigTBL_receiver_config_Bank1:
;  Instance name RECEIVE, User Module RX8
;       Instance name RECEIVE, Block Name RX8(DCC02)
	db		2bh, 00h		;RECEIVE_(DCC02CR1)
	db		28h, 05h		;RECEIVE_FUNC_REG     (DCC02FN)
	db		29h, c1h		;RECEIVE_INPUT_REG    (DCC02IN)
	db		2ah, 80h		;RECEIVE_OUTPUT_REG   (DCC02OU)
;  Instance name RX_TIMEOUT, User Module Timer16
;       Instance name RX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;RX_TIMEOUT_(DBC00CR1)
	db		20h, 00h		;RX_TIMEOUT_FUNC_LSB_REG(DBC00FN)
	db		21h, 06h		;RX_TIMEOUT_INPUT_LSB_REG(DBC00IN)
	db		22h, 40h		;RX_TIMEOUT_OUTPUT_LSB_REG(DBC00OU)
;       Instance name RX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;RX_TIMEOUT_(DBC01CR1)
	db		24h, 20h		;RX_TIMEOUT_FUNC_MSB_REG(DBC01FN)
	db		25h, 36h		;RX_TIMEOUT_INPUT_MSB_REG(DBC01IN)
	db		26h, 40h		;RX_TIMEOUT_OUTPUT_MSB_REG(DBC01OU)
	db		ffh
UnloadConfigTBL_receiver_config_Bank0:
;  Instance name RECEIVE, User Module RX8
;       Instance name RECEIVE, Block Name RX8(DCC02)
	db		2bh, 00h		;RECEIVE_CONTROL_0 (DCC02CR0)
;  Instance name RX_TIMEOUT, User Module Timer16
;       Instance name RX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;RX_TIMEOUT_CONTROL_0 (DBC00CR0)
;       Instance name RX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;RX_TIMEOUT_CONTROL_0 (DBC01CR0)
	db		ffh
UnloadConfigTBL_receiver_config_Bank1:
;  Instance name RECEIVE, User Module RX8
;       Instance name RECEIVE, Block Name RX8(DCC02)
	db		2bh, 00h		;RECEIVE_CONTROL_1 (DCC02CR1)
	db		28h, 00h		;RECEIVE_DIG_BasicFunction (DCC02FN)
	db		29h, 00h		;RECEIVE_DIG_Input (DCC02IN)
	db		2ah, 00h		;RECEIVE_DIG_Output (DCC02OU)
;  Instance name RX_TIMEOUT, User Module Timer16
;       Instance name RX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;RX_TIMEOUT_CONTROL_1 (DBC00CR1)
	db		20h, 00h		;RX_TIMEOUT_DIG_BasicFunction (DBC00FN)
	db		21h, 00h		;RX_TIMEOUT_DIG_Input (DBC00IN)
	db		22h, 00h		;RX_TIMEOUT_DIG_Output (DBC00OU)
;       Instance name RX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;RX_TIMEOUT_CONTROL_1 (DBC01CR1)
	db		24h, 00h		;RX_TIMEOUT_DIG_BasicFunction (DBC01FN)
	db		25h, 00h		;RX_TIMEOUT_DIG_Input (DBC01IN)
	db		26h, 00h		;RX_TIMEOUT_DIG_Output (DBC01OU)
	db		ffh

;  Instance name RECEIVE, User Module RX8
;       Instance name RECEIVE, Block Name RX8(DCC02)
;  Instance name RX_TIMEOUT, User Module Timer16
;       Instance name RX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
;       Instance name RX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		ffh
LoadConfigTBL_transmitter_config_Bank0:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_CONTROL_REG  (DCC02CR0)
	db		29h, 00h		;TRANSMIT_TX_BUFFER_REG(DCC02DR1)
	db		2ah, 00h		;TRANSMIT_(DCC02DR2)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_CONTROL_LSB_REG(DBC00CR0)
	db		21h, f0h		;TX_TIMEOUT_PERIOD_LSB_REG(DBC00DR1)
	db		22h, 00h		;TX_TIMEOUT_COMPARE_LSB_REG(DBC00DR2)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 04h		;TX_TIMEOUT_CONTROL_MSB_REG(DBC01CR0)
	db		25h, 00h		;TX_TIMEOUT_PERIOD_MSB_REG(DBC01DR1)
	db		26h, 00h		;TX_TIMEOUT_COMPARE_MSB_REG(DBC01DR2)
;  Global Register values Bank 0
	db		6ah, 00h		; ADCDataHigh register (SADC_DH)
	db		6bh, 00h		; ADCDataLow register (SADC_DL)
	db		62h, 00h		; AnalogClockSelect3 register (CLK_CR3)
	db		60h, 09h		; AnalogColumnInputSelect register (AMX_IN)
	db		66h, 00h		; AnalogComparatorControl1 register (CMP_CR1)
	db		61h, 00h		; AnalogMuxBusConfig register (AMUX_CFG)
	db		fch, 00h		; AnalogMuxDACData:0 register (IDACR_D)
	db		fdh, 00h		; AnalogMuxDACData:1 register (IDACL_D)
	db		63h, 55h		; AnalogReferenceControl register (ARF_CR)
	db		65h, 00h		; AnalogSynchronizationControl register (ASY_CR)
	db		e6h, 00h		; DecimatorControl_0 register (DEC_CR0)
	db		e7h, 00h		; DecimatorControl_1 register (DEC_CR1)
	db		a0h, 00h		; DecimatorDataHigh:0 register (DEC0_DH)
	db		a2h, 00h		; DecimatorDataHigh:1 register (DEC1_DH)
	db		a4h, 00h		; DecimatorDataHigh:2 register (DEC2_DH)
	db		a6h, 00h		; DecimatorDataHigh:3 register (DEC3_DH)
	db		a1h, 00h		; DecimatorDataLow:0 register (DEC0_DL)
	db		a3h, 00h		; DecimatorDataLow:1 register (DEC1_DL)
	db		a5h, 00h		; DecimatorDataLow:2 register (DEC2_DL)
	db		a7h, 00h		; DecimatorDataLow:3 register (DEC3_DL)
	db		d6h, 00h		; I2CConfig:0 register (I2C0_CFG)
	db		e8h, 00h		; Multiply0InputX register (MUL0_X)
	db		e9h, 00h		; Multiply0InputY register (MUL0_Y)
	db		a8h, 00h		; Multiply1InputX register (MUL1_X)
	db		a9h, 00h		; Multiply1InputY register (MUL1_Y)
	db		b7h, 00h		; RowDigitalInterconnectInputSelect:0 register (RDI0DSM)
	db		bfh, 00h		; RowDigitalInterconnectInputSelect:1 register (RDI1DSM)
	db		c7h, 00h		; RowDigitalInterconnectInputSelect:2 register (RDI2DSM)
	db		b0h, 00h		; Row_0_InputMux register (RDI0RI)
	db		b1h, 00h		; Row_0_InputSync register (RDI0SYN)
	db		b2h, 00h		; Row_0_LogicInputAMux register (RDI0IS)
	db		b3h, 33h		; Row_0_LogicSelect_0 register (RDI0LT0)
	db		b4h, 33h		; Row_0_LogicSelect_1 register (RDI0LT1)
	db		b5h, 02h		; Row_0_OutputDrive_0 register (RDI0RO0)
	db		b6h, 00h		; Row_0_OutputDrive_1 register (RDI0RO1)
	db		b8h, 55h		; Row_1_InputMux register (RDI1RI)
	db		b9h, 00h		; Row_1_InputSync register (RDI1SYN)
	db		bah, 10h		; Row_1_LogicInputAMux register (RDI1IS)
	db		bbh, 33h		; Row_1_LogicSelect_0 register (RDI1LT0)
	db		bch, 33h		; Row_1_LogicSelect_1 register (RDI1LT1)
	db		bdh, 00h		; Row_1_OutputDrive_0 register (RDI1RO0)
	db		beh, 00h		; Row_1_OutputDrive_1 register (RDI1RO1)
	db		c0h, aah		; Row_2_InputMux register (RDI2RI)
	db		c1h, 00h		; Row_2_InputSync register (RDI2SYN)
	db		c2h, 00h		; Row_2_LogicInputAMux register (RDI2IS)
	db		c3h, 33h		; Row_2_LogicSelect_0 register (RDI2LT0)
	db		c4h, 33h		; Row_2_LogicSelect_1 register (RDI2LT1)
	db		c5h, 00h		; Row_2_OutputDrive_0 register (RDI2RO0)
	db		c6h, 00h		; Row_2_OutputDrive_1 register (RDI2RO1)
	db		ffh
LoadConfigTBL_transmitter_config_Bank1:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_(DCC02CR1)
	db		28h, 1dh		;TRANSMIT_FUNC_REG     (DCC02FN)
	db		29h, 01h		;TRANSMIT_INPUT_REG    (DCC02IN)
	db		2ah, 84h		;TRANSMIT_OUTPUT_REG   (DCC02OU)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_(DBC00CR1)
	db		20h, 00h		;TX_TIMEOUT_FUNC_LSB_REG(DBC00FN)
	db		21h, 06h		;TX_TIMEOUT_INPUT_LSB_REG(DBC00IN)
	db		22h, 40h		;TX_TIMEOUT_OUTPUT_LSB_REG(DBC00OU)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;TX_TIMEOUT_(DBC01CR1)
	db		24h, 20h		;TX_TIMEOUT_FUNC_MSB_REG(DBC01FN)
	db		25h, 36h		;TX_TIMEOUT_INPUT_MSB_REG(DBC01IN)
	db		26h, 40h		;TX_TIMEOUT_OUTPUT_MSB_REG(DBC01OU)
;  Global Register values Bank 1
	db		a8h, 00h		; ADCControl0 register (SADC_CR0)
	db		a9h, 00h		; ADCControl1 register (SADC_CR1)
	db		aah, 00h		; ADCControl2 register (ADC_CR2)
	db		abh, 00h		; ADCControl3 register (ADC_CR3TRIM)
	db		ach, 00h		; ADCControl4 register (ADC_CR4)
	db		61h, 00h		; AnalogClockSelect1 register (CLK_CR1)
	db		69h, 00h		; AnalogClockSelect2 register (CLK_CR2)
	db		8bh, 00h		; AnalogColumnClockDivide register (ACE_CLK_CR3)
	db		60h, 00h		; AnalogColumnClockSelect register (CLK_CR0)
	db		8ah, 00h		; AnalogEClockSelect1 register (ACE_CLK_CR1)
	db		89h, 00h		; AnalogEColumnClockSelect register (ACE_CLK_CR0)
	db		75h, 09h		; AnalogEColumnInputSelect register (ACE_AMX_IN)
	db		76h, 00h		; AnalogEComparatorControl0 register (ACE_CMP_CR0)
	db		77h, 00h		; AnalogEComparatorControl1 register (ACE_CMP_CR1)
	db		7ah, 03h		; AnalogELUTControl0 register (ACE_ALT_CR0)
	db		62h, 00h		; AnalogIOControl_0 register (ABF_CR0)
	db		67h, 33h		; AnalogLUTControl0 register (ALT_CR0)
	db		68h, 00h		; AnalogLUTControl1 register (ALT_CR1)
	db		63h, 00h		; AnalogModulatorControl_0 register (AMD_CR0)
	db		66h, 00h		; AnalogModulatorControl_1 register (AMD_CR1)
	db		6ah, 00h		; AnalogMuxBusConfig1 register (AMUX_CFG1)
	db		afh, 00h		; AnalogMuxClock register (AMUX_CLK)
	db		7bh, 00h		; AnalogOutBufferControl register (ACE_ABF_CR0)
	db		79h, 00h		; ComparatorGlobalInEn register (ACE_CMP_GI_EN)
	db		64h, 00h		; ComparatorGlobalOutEn register (CMP_GO_EN)
	db		fdh, 00h		; DAC_Control_0 register (IDAC_CR0)
	db		dch, 00h		; DAC_Control_1 register (IDAC_CR1)
	db		91h, 00h		; DEC_CR0:0 register (DEC0_CR0)
	db		95h, 00h		; DEC_CR0:1 register (DEC1_CR0)
	db		99h, 00h		; DEC_CR0:2 register (DEC2_CR0)
	db		9dh, 00h		; DEC_CR0:3 register (DEC3_CR0)
	db		9ah, 00h		; DecimatorControl_5 register (DEC_CR5)
	db		92h, 00h		; DecimatorEnable:0 register (DEC_CR3)
	db		96h, 00h		; DecimatorEnable:1 register (DEC_CR4)
	db		d4h, 00h		; Decimator_Control:0 register (DEC0_CR)
	db		d5h, 00h		; Decimator_Control:1 register (DEC1_CR)
	db		d6h, 00h		; Decimator_Control:2 register (DEC2_CR)
	db		d7h, 00h		; Decimator_Control:3 register (DEC3_CR)
	db		d1h, 00h		; GlobalDigitalInterconnect_Drive_Even_Input register (GDI_E_IN)
	db		a1h, 00h		; GlobalDigitalInterconnect_Drive_Even_Input_Control register (GDI_E_IN_CR)
	db		d3h, 00h		; GlobalDigitalInterconnect_Drive_Even_Output register (GDI_E_OU)
	db		a3h, 00h		; GlobalDigitalInterconnect_Drive_Even_Output_Control register (GDI_E_OU_CR)
	db		d0h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Input register (GDI_O_IN)
	db		a0h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Input_Control register (GDI_O_IN_CR)
	db		d2h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Output register (GDI_O_OU)
	db		a2h, 00h		; GlobalDigitalInterconnect_Drive_Odd_Output_Control register (GDI_O_OU_CR)
	db		adh, 00h		; I2CAddress:0 register (I2C0_ADDR)
	db		e7h, 00h		; IDACMode register (IDACMODE)
	db		e1h, 99h		; OscillatorControl_1 register (OSC_CR1)
	db		e2h, 00h		; OscillatorControl_2 register (OSC_CR2)
	db		dfh, 05h		; OscillatorControl_3 register (OSC_CR3)
	db		deh, 03h		; OscillatorControl_4 register (OSC_CR4)
	db		ddh, 00h		; OscillatorGlobalBusEnableControl register (OSC_GO_EN)
	db		85h, 00h		; PWM_Control register (ACE_PWM_CR)
	db		d8h, 00h		; Port_0_MUXBusCtrl register (MUX_CR0)
	db		d9h, 00h		; Port_1_MUXBusCtrl register (MUX_CR1)
	db		dah, 00h		; Port_2_MUXBusCtrl register (MUX_CR2)
	db		dbh, 00h		; Port_3_MUXBusCtrl register (MUX_CR3)
	db		ech, 00h		; Port_4_MUXBusCtrl register (MUX_CR4)
	db		edh, 00h		; Port_5_MUXBusCtrl register (MUX_CR5)
	db		a7h, 00h		; RTClockControl register (RTCCR)
	db		a4h, 00h		; RTCurrentHour register (RTCH)
	db		a5h, 00h		; RTCurrentMinute register (RTCM)
	db		a6h, 00h		; RTCurrentSecond register (RTCS)
	db		82h, 00h		; TSCMPHigh register (SADC_TSCMPH)
	db		81h, 00h		; TSCMPLow register (SADC_TSCMPL)
	db		71h, 00h		; TSource0 register (SADC_TSCR0)
	db		72h, 00h		; TSource1 register (SADC_TSCR1)
	db		ffh
LoadConfigTBL_transmitter_config_Ordered:
;  Ordered Global Register values
	M8C_SetBank1
	mov	reg[00h], 10h		; Port_0_DriveMode_0 register (PRT0DM0)
	mov	reg[01h], efh		; Port_0_DriveMode_1 register (PRT0DM1)
	M8C_SetBank0
	mov	reg[03h], efh		; Port_0_DriveMode_2 register (PRT0DM2)
	mov	reg[02h], 10h		; Port_0_GlobalSelect register (PRT0GS)
	M8C_SetBank1
	mov	reg[02h], 00h		; Port_0_IntCtrl_0 register (PRT0IC0)
	mov	reg[03h], 00h		; Port_0_IntCtrl_1 register (PRT0IC1)
	M8C_SetBank0
	mov	reg[01h], 00h		; Port_0_IntEn register (PRT0IE)
	M8C_SetBank1
	mov	reg[04h], 00h		; Port_1_DriveMode_0 register (PRT1DM0)
	mov	reg[05h], ffh		; Port_1_DriveMode_1 register (PRT1DM1)
	M8C_SetBank0
	mov	reg[07h], ffh		; Port_1_DriveMode_2 register (PRT1DM2)
	mov	reg[06h], 00h		; Port_1_GlobalSelect register (PRT1GS)
	M8C_SetBank1
	mov	reg[06h], 00h		; Port_1_IntCtrl_0 register (PRT1IC0)
	mov	reg[07h], 00h		; Port_1_IntCtrl_1 register (PRT1IC1)
	M8C_SetBank0
	mov	reg[05h], 00h		; Port_1_IntEn register (PRT1IE)
	M8C_SetBank1
	mov	reg[08h], 00h		; Port_2_DriveMode_0 register (PRT2DM0)
	mov	reg[09h], ffh		; Port_2_DriveMode_1 register (PRT2DM1)
	M8C_SetBank0
	mov	reg[0bh], ffh		; Port_2_DriveMode_2 register (PRT2DM2)
	mov	reg[0ah], 00h		; Port_2_GlobalSelect register (PRT2GS)
	M8C_SetBank1
	mov	reg[0ah], 00h		; Port_2_IntCtrl_0 register (PRT2IC0)
	mov	reg[0bh], 00h		; Port_2_IntCtrl_1 register (PRT2IC1)
	M8C_SetBank0
	mov	reg[09h], 00h		; Port_2_IntEn register (PRT2IE)
	M8C_SetBank1
	mov	reg[0ch], 00h		; Port_3_DriveMode_0 register (PRT3DM0)
	mov	reg[0dh], 00h		; Port_3_DriveMode_1 register (PRT3DM1)
	M8C_SetBank0
	mov	reg[0fh], 00h		; Port_3_DriveMode_2 register (PRT3DM2)
	mov	reg[0eh], 00h		; Port_3_GlobalSelect register (PRT3GS)
	M8C_SetBank1
	mov	reg[0eh], 00h		; Port_3_IntCtrl_0 register (PRT3IC0)
	mov	reg[0fh], 00h		; Port_3_IntCtrl_1 register (PRT3IC1)
	M8C_SetBank0
	mov	reg[0dh], 00h		; Port_3_IntEn register (PRT3IE)
	M8C_SetBank1
	mov	reg[10h], 00h		; Port_4_DriveMode_0 register (PRT4DM0)
	mov	reg[11h], 00h		; Port_4_DriveMode_1 register (PRT4DM1)
	M8C_SetBank0
	mov	reg[13h], 00h		; Port_4_DriveMode_2 register (PRT4DM2)
	mov	reg[12h], 00h		; Port_4_GlobalSelect register (PRT4GS)
	M8C_SetBank1
	mov	reg[12h], 00h		; Port_4_IntCtrl_0 register (PRT4IC0)
	mov	reg[13h], 00h		; Port_4_IntCtrl_1 register (PRT4IC1)
	M8C_SetBank0
	mov	reg[11h], 00h		; Port_4_IntEn register (PRT4IE)
	mov	reg[15h], 00h		; Port_5_IntEn register (PRT5IE)
	mov	reg[16h], 00h		; Port_5_GlobalSelect register (PRT5GS)
	mov	reg[17h], 00h		; Port_5_DriveMode_2 register (PRT5DM2)
	M8C_SetBank1
	mov	reg[15h], 00h		; Port_5_DriveMode_1 register (PRT5DM1)
	mov	reg[14h], 00h		; Port_5_DriveMode_0 register (PRT5DM0)
	mov	reg[16h], 00h		; Port_5_IntCtrl_0 register (PRT5IC0)
	mov	reg[17h], 00h		; Port_5_IntCtrl_1 register (PRT5IC1)
	ret
ReloadConfigTBL_transmitter_config_Bank0:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_CONTROL_REG  (DCC02CR0)
	db		29h, 00h		;TRANSMIT_TX_BUFFER_REG(DCC02DR1)
	db		2ah, 00h		;TRANSMIT_(DCC02DR2)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_CONTROL_LSB_REG(DBC00CR0)
	db		21h, f0h		;TX_TIMEOUT_PERIOD_LSB_REG(DBC00DR1)
	db		22h, 00h		;TX_TIMEOUT_COMPARE_LSB_REG(DBC00DR2)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 04h		;TX_TIMEOUT_CONTROL_MSB_REG(DBC01CR0)
	db		25h, 00h		;TX_TIMEOUT_PERIOD_MSB_REG(DBC01DR1)
	db		26h, 00h		;TX_TIMEOUT_COMPARE_MSB_REG(DBC01DR2)
	db		ffh
ReloadConfigTBL_transmitter_config_Bank1:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_(DCC02CR1)
	db		28h, 1dh		;TRANSMIT_FUNC_REG     (DCC02FN)
	db		29h, 01h		;TRANSMIT_INPUT_REG    (DCC02IN)
	db		2ah, 84h		;TRANSMIT_OUTPUT_REG   (DCC02OU)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_(DBC00CR1)
	db		20h, 00h		;TX_TIMEOUT_FUNC_LSB_REG(DBC00FN)
	db		21h, 06h		;TX_TIMEOUT_INPUT_LSB_REG(DBC00IN)
	db		22h, 40h		;TX_TIMEOUT_OUTPUT_LSB_REG(DBC00OU)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;TX_TIMEOUT_(DBC01CR1)
	db		24h, 20h		;TX_TIMEOUT_FUNC_MSB_REG(DBC01FN)
	db		25h, 36h		;TX_TIMEOUT_INPUT_MSB_REG(DBC01IN)
	db		26h, 40h		;TX_TIMEOUT_OUTPUT_MSB_REG(DBC01OU)
	db		ffh
UnloadConfigTBL_transmitter_config_Bank0:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_CONTROL_0 (DCC02CR0)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_CONTROL_0 (DBC00CR0)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;TX_TIMEOUT_CONTROL_0 (DBC01CR0)
	db		ffh
UnloadConfigTBL_transmitter_config_Bank1:
;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
	db		2bh, 00h		;TRANSMIT_CONTROL_1 (DCC02CR1)
	db		28h, 00h		;TRANSMIT_DIG_BasicFunction (DCC02FN)
	db		29h, 00h		;TRANSMIT_DIG_Input (DCC02IN)
	db		2ah, 00h		;TRANSMIT_DIG_Output (DCC02OU)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
	db		23h, 00h		;TX_TIMEOUT_CONTROL_1 (DBC00CR1)
	db		20h, 00h		;TX_TIMEOUT_DIG_BasicFunction (DBC00FN)
	db		21h, 00h		;TX_TIMEOUT_DIG_Input (DBC00IN)
	db		22h, 00h		;TX_TIMEOUT_DIG_Output (DBC00OU)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		27h, 00h		;TX_TIMEOUT_CONTROL_1 (DBC01CR1)
	db		24h, 00h		;TX_TIMEOUT_DIG_BasicFunction (DBC01FN)
	db		25h, 00h		;TX_TIMEOUT_DIG_Input (DBC01IN)
	db		26h, 00h		;TX_TIMEOUT_DIG_Output (DBC01OU)
	db		ffh

;  Instance name TRANSMIT, User Module TX8
;       Instance name TRANSMIT, Block Name TX8(DCC02)
;  Instance name TX_TIMEOUT, User Module Timer16
;       Instance name TX_TIMEOUT, Block Name TIMER16_LSB(DBC00)
;       Instance name TX_TIMEOUT, Block Name TIMER16_MSB(DBC01)
	db		ffh
UnloadConfigTBL_Total_Bank0:
;  Block DBC00
	db		23h, 00h		; CONTROL_0 register (DBC00CR0)
;  Block DBC01
	db		27h, 00h		; CONTROL_0 register (DBC01CR0)
;  Block DCC02
	db		2bh, 00h		; CONTROL_0 register (DCC02CR0)
;  Block DCC03
	db		2fh, 00h		; CONTROL_0 register (DCC03CR0)
;  Block DBC10
	db		33h, 00h		; CONTROL_0 register (DBC10CR0)
;  Block DBC11
	db		37h, 00h		; CONTROL_0 register (DBC11CR0)
;  Block DCC12
	db		3bh, 00h		; CONTROL_0 register (DCC12CR0)
;  Block DCC13
	db		3fh, 00h		; CONTROL_0 register (DCC13CR0)
;  Block DBC20
	db		43h, 00h		; CONTROL_0 register (DBC20CR0)
;  Block DBC21
	db		47h, 00h		; CONTROL_0 register (DBC21CR0)
;  Block DCC22
	db		4bh, 00h		; CONTROL_0 register (DCC22CR0)
;  Block DCC23
	db		4fh, 00h		; CONTROL_0 register (DCC23CR0)
;  Block ACC00
	db		73h, 00h		; CR2 register (ACC00CR2)
;  Block ASC10
;  Block ASD20
	db		93h, 00h		; CR3 register (ASD20CR3)
;  Block 
;  Block ACC01
	db		77h, 00h		; CR2 register (ACC01CR2)
;  Block ASD11
	db		87h, 00h		; CR3 register (ASD11CR3)
;  Block ASC21
;  Block 
;  Block ACE00
;  Block ASE10
;  Block 
;  Block 
;  Block ACE01
;  Block ASE11
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
	db		ffh
UnloadConfigTBL_Total_Bank1:
;  Block DBC00
	db		23h, 00h		; CONTROL_1 register (DBC00CR1)
	db		20h, 00h		; DIG_BasicFunction register (DBC00FN)
	db		21h, 00h		; DIG_Input register (DBC00IN)
	db		22h, 00h		; DIG_Output register (DBC00OU)
;  Block DBC01
	db		27h, 00h		; CONTROL_1 register (DBC01CR1)
	db		24h, 00h		; DIG_BasicFunction register (DBC01FN)
	db		25h, 00h		; DIG_Input register (DBC01IN)
	db		26h, 00h		; DIG_Output register (DBC01OU)
;  Block DCC02
	db		2bh, 00h		; CONTROL_1 register (DCC02CR1)
	db		28h, 00h		; DIG_BasicFunction register (DCC02FN)
	db		29h, 00h		; DIG_Input register (DCC02IN)
	db		2ah, 00h		; DIG_Output register (DCC02OU)
;  Block DCC03
	db		2fh, 00h		; CONTROL_1 register (DCC03CR1)
	db		2ch, 00h		; DIG_BasicFunction register (DCC03FN)
	db		2dh, 00h		; DIG_Input register (DCC03IN)
	db		2eh, 00h		; DIG_Output register (DCC03OU)
;  Block DBC10
	db		33h, 00h		; CONTROL_1 register (DBC10CR1)
	db		30h, 00h		; DIG_BasicFunction register (DBC10FN)
	db		31h, 00h		; DIG_Input register (DBC10IN)
	db		32h, 00h		; DIG_Output register (DBC10OU)
;  Block DBC11
	db		37h, 00h		; CONTROL_1 register (DBC11CR1)
	db		34h, 00h		; DIG_BasicFunction register (DBC11FN)
	db		35h, 00h		; DIG_Input register (DBC11IN)
	db		36h, 00h		; DIG_Output register (DBC11OU)
;  Block DCC12
	db		3bh, 00h		; CONTROL_1 register (DCC12CR1)
	db		38h, 00h		; DIG_BasicFunction register (DCC12FN)
	db		39h, 00h		; DIG_Input register (DCC12IN)
	db		3ah, 00h		; DIG_Output register (DCC12OU)
;  Block DCC13
	db		3fh, 00h		; CONTROL_1 register (DCC13CR1)
	db		3ch, 00h		; DIG_BasicFunction register (DCC13FN)
	db		3dh, 00h		; DIG_Input register (DCC13IN)
	db		3eh, 00h		; DIG_Output register (DCC13OU)
;  Block DBC20
	db		43h, 00h		; CONTROL_1 register (DBC20CR1)
	db		40h, 00h		; DIG_BasicFunction register (DBC20FN)
	db		41h, 00h		; DIG_Input register (DBC20IN)
	db		42h, 00h		; DIG_Output register (DBC20OU)
;  Block DBC21
	db		47h, 00h		; CONTROL_1 register (DBC21CR1)
	db		44h, 00h		; DIG_BasicFunction register (DBC21FN)
	db		45h, 00h		; DIG_Input register (DBC21IN)
	db		46h, 00h		; DIG_Output register (DBC21OU)
;  Block DCC22
	db		4bh, 00h		; CONTROL_1 register (DCC22CR1)
	db		48h, 00h		; DIG_BasicFunction register (DCC22FN)
	db		49h, 00h		; DIG_Input register (DCC22IN)
	db		4ah, 00h		; DIG_Output register (DCC22OU)
;  Block DCC23
	db		4fh, 00h		; CONTROL_1 register (DCC23CR1)
	db		4ch, 00h		; DIG_BasicFunction register (DCC23FN)
	db		4dh, 00h		; DIG_Input register (DCC23IN)
	db		4eh, 00h		; DIG_Output register (DCC23OU)
;  Block ACC00
;  Block ASC10
;  Block ASD20
;  Block 
;  Block ACC01
;  Block ASD11
;  Block ASC21
;  Block 
;  Block ACE00
;  Block ASE10
;  Block 
;  Block 
;  Block ACE01
;  Block ASE11
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
;  Block 
	db		ffh


; PSoC Configuration file trailer PsocConfig.asm
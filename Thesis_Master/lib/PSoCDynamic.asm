; Generated by PSoC Designer 5.0.985.0
;
;
;  thesis_masterDynamic.asm
;
;  Data: 11 June, 2004
;  Copyright Cypress MicroSystems 2001-2004
;
;  This file is generated by the Device Editor on Application Generation.
;  It contains functions that can be used to test the state of the 
;  currently active configuration.
;  
;  DO NOT EDIT THIS FILE MANUALLY, AS IT IS OVERWRITTEN!!!
;  Edits to this file will not be preserved.
;
include "m8c.inc"
include "memory.inc"
include "PSoCDynamic.inc"
export Istransmitter_configLoaded
export _Istransmitter_configLoaded
export Ispc_listenerLoaded
export _Ispc_listenerLoaded
export Isreceiver_configLoaded
export _Isreceiver_configLoaded
Istransmitter_configLoaded:
_Istransmitter_configLoaded:
	RAM_SETPAGE_CUR >ACTIVE_CONFIG_STATUS
	mov		a, 0
	tst		[ACTIVE_CONFIG_STATUS+transmitter_config_ADDR_OFF], transmitter_config_BIT
	jz		transmitter_configIsNotLoaded
	mov		a, 1
transmitter_configIsNotLoaded:
	ret

Ispc_listenerLoaded:
_Ispc_listenerLoaded:
	RAM_SETPAGE_CUR >ACTIVE_CONFIG_STATUS
	mov		a, 0
	tst		[ACTIVE_CONFIG_STATUS+pc_listener_ADDR_OFF], pc_listener_BIT
	jz		pc_listenerIsNotLoaded
	mov		a, 1
pc_listenerIsNotLoaded:
	ret

Isreceiver_configLoaded:
_Isreceiver_configLoaded:
	RAM_SETPAGE_CUR >ACTIVE_CONFIG_STATUS
	mov		a, 0
	tst		[ACTIVE_CONFIG_STATUS+receiver_config_ADDR_OFF], receiver_config_BIT
	jz		receiver_configIsNotLoaded
	mov		a, 1
receiver_configIsNotLoaded:
	ret


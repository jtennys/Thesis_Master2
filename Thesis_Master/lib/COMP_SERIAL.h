//*****************************************************************************
//*****************************************************************************
//  FILENAME:  COMP_SERIAL.h
//  Version: 5.2, Updated on 2009/7/10 at 10:46:57
//  Generated by PSoC Designer 5.0.985.0
//
//  DESCRIPTION:  UART User Module C Language interface file for the
//                22/24/25/26/27xxx PSoC family of devices.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress MicroSystems 2000-2003. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************

// include the global header file
#include <m8c.h>

#define COMP_SERIAL_RXBUF_ENABLE 1

//-------------------------------------------------
// Prototypes of the COMP_SERIAL API.
//-------------------------------------------------

#if ( COMP_SERIAL_RXBUF_ENABLE )
extern char COMP_SERIAL_aRxBuffer[];
extern BYTE COMP_SERIAL_bRxCnt;
extern BYTE COMP_SERIAL_fStatus;
#endif



// Create pragmas to support proper argument and return value passing

#pragma fastcall16  COMP_SERIAL_SetTxIntMode
#pragma fastcall16  COMP_SERIAL_EnableInt
#pragma fastcall16  COMP_SERIAL_DisableInt
#pragma fastcall16  COMP_SERIAL_Start
#pragma fastcall16  COMP_SERIAL_Stop
#pragma fastcall16  COMP_SERIAL_SendData
#pragma fastcall16  COMP_SERIAL_bReadTxStatus
#pragma fastcall16  COMP_SERIAL_bReadRxData
#pragma fastcall16  COMP_SERIAL_bReadRxStatus
#pragma fastcall16  COMP_SERIAL_PutSHexByte
#pragma fastcall16  COMP_SERIAL_PutSHexInt
#pragma fastcall16  COMP_SERIAL_CPutString
#pragma fastcall16  COMP_SERIAL_PutString
#pragma fastcall16  COMP_SERIAL_PutChar
#pragma fastcall16  COMP_SERIAL_Write
#pragma fastcall16  COMP_SERIAL_CWrite

#pragma fastcall16  COMP_SERIAL_cGetChar
#pragma fastcall16  COMP_SERIAL_cReadChar
#pragma fastcall16  COMP_SERIAL_iReadChar
#pragma fastcall16  COMP_SERIAL_IntCntl
#pragma fastcall16  COMP_SERIAL_TxIntMode
#pragma fastcall16  COMP_SERIAL_PutCRLF

#if ( COMP_SERIAL_RXBUF_ENABLE )
#pragma fastcall16  COMP_SERIAL_CmdReset
#pragma fastcall16  COMP_SERIAL_bCmdCheck
#pragma fastcall16  COMP_SERIAL_bErrCheck
#pragma fastcall16  COMP_SERIAL_bCmdLength
#pragma fastcall16  COMP_SERIAL_szGetParam
#pragma fastcall16  COMP_SERIAL_szGetRestOfParams
#endif

//**************************************************
// Prototypes of UART API.
//**************************************************
extern void  COMP_SERIAL_SetTxIntMode(BYTE bTxIntMode);
extern void  COMP_SERIAL_EnableInt(void);
extern void  COMP_SERIAL_DisableInt(void);
extern void  COMP_SERIAL_Start(BYTE bParitySetting);
extern void  COMP_SERIAL_Stop(void);
extern void  COMP_SERIAL_SendData(BYTE bTxData);
extern BYTE  COMP_SERIAL_bReadTxStatus(void);
extern BYTE  COMP_SERIAL_bReadRxData(void);
extern BYTE  COMP_SERIAL_bReadRxStatus(void);

// High level TX functions
extern void   COMP_SERIAL_CPutString(const BYTE * szRomString);
extern void   COMP_SERIAL_PutString(BYTE * szRamString);
extern void   COMP_SERIAL_PutChar(CHAR cData);
extern void   COMP_SERIAL_Write(BYTE * szRamString, BYTE bCount);
extern void   COMP_SERIAL_CWrite(const BYTE * szRomString, INT iCount);
extern void   COMP_SERIAL_PutSHexByte(BYTE bValue);
extern void   COMP_SERIAL_PutSHexInt(INT iValue);
extern void   COMP_SERIAL_PutCRLF(void);
extern void   COMP_SERIAL_TxIntMode(BYTE bMask);

// High level RX functions
extern CHAR   COMP_SERIAL_cGetChar(void);
extern CHAR   COMP_SERIAL_cReadChar(void);
extern INT    COMP_SERIAL_iReadChar(void);
extern void   COMP_SERIAL_IntCntl(BYTE bMask);

#if ( COMP_SERIAL_RXBUF_ENABLE )
extern void   COMP_SERIAL_CmdReset(void);
extern BYTE   COMP_SERIAL_bCmdCheck(void);
extern BYTE   COMP_SERIAL_bErrCheck(void);
extern BYTE   COMP_SERIAL_bCmdLength(void);
extern BYTE * COMP_SERIAL_szGetParam(void);
extern BYTE * COMP_SERIAL_szGetRestOfParams(void);
#endif

//-------------------------------------------------
// Defines for COMP_SERIAL API's.
//-------------------------------------------------



//------------------------------------
//  Parity masks
//------------------------------------
#define  COMP_SERIAL_PARITY_NONE        0x00
#define  COMP_SERIAL_PARITY_EVEN        0x02
#define  COMP_SERIAL_PARITY_ODD         0x06
//------------------------------------
//  Transmitter Status Register masks
//------------------------------------
#define  COMP_SERIAL_TX_COMPLETE        0x20
#define  COMP_SERIAL_TX_BUFFER_EMPTY    0x10

//------------------------------------
//  Receiver Status Register masks
//------------------------------------
#define  COMP_SERIAL_RX_ACTIVE          0x10
#define  COMP_SERIAL_RX_COMPLETE        0x08
#define  COMP_SERIAL_RX_REG_FULL        0x08
#define  COMP_SERIAL_RX_PARITY_ERROR    0x80
#define  COMP_SERIAL_RX_OVERRUN_ERROR   0x40
#define  COMP_SERIAL_RX_FRAMING_ERROR   0x20
#define  COMP_SERIAL_RX_ERROR           0xE0
#define  COMP_SERIAL_RX_NO_ERROR        0xE0          // This symbol is deprecated and will removed in the future

#define  COMP_SERIAL_RX_NO_DATA         0x01

#define  COMP_SERIAL_RX_BUF_ERROR         0xF0  // Mask for any Rx that may occur.
#define  COMP_SERIAL_RX_BUF_OVERRUN       0x10  // This indicates the software buffer has
                                                           // been over run.
#define  COMP_SERIAL_RX_BUF_CMDTERM       0x01  // Command terminator has been received.

// Interrupt control used with  COMP_SERIAL_IntCntl() function
#define COMP_SERIAL_ENABLE_RX_INT  0x01
#define COMP_SERIAL_ENABLE_TX_INT  0x02
#define COMP_SERIAL_DISABLE_RX_INT 0x00
#define COMP_SERIAL_DISABLE_TX_INT 0x00

// Interrupt Modes
#define COMP_SERIAL_INT_MODE_TX_REG_EMPTY 0x00
#define COMP_SERIAL_INT_MODE_TX_COMPLETE  0x01

//-------------------------------------------------
// Register Address Constants for COMP_SERIAL
//-------------------------------------------------

#pragma ioport  COMP_SERIAL_TX_CONTROL_REG: 0x03b           // Control register
BYTE            COMP_SERIAL_TX_CONTROL_REG;
#pragma ioport  COMP_SERIAL_TX_SHIFT_REG:   0x038               // TX Shift Register register
BYTE            COMP_SERIAL_TX_SHIFT_REG;
#pragma ioport  COMP_SERIAL_TX_BUFFER_REG:  0x039               // TX Buffer Register
BYTE            COMP_SERIAL_TX_BUFFER_REG;
#pragma ioport  COMP_SERIAL_TX_FUNC_REG:    0x138           // Function register
BYTE            COMP_SERIAL_TX_FUNC_REG;
#pragma ioport  COMP_SERIAL_TX_INPUT_REG:   0x139           // Input register
BYTE            COMP_SERIAL_TX_INPUT_REG;
#pragma ioport  COMP_SERIAL_TX_OUTPUT_REG:  0x13a           // Output register
BYTE            COMP_SERIAL_TX_OUTPUT_REG;
#pragma ioport  COMP_SERIAL_RX_CONTROL_REG: 0x03f           // Control register
BYTE            COMP_SERIAL_RX_CONTROL_REG;
#pragma ioport  COMP_SERIAL_RX_SHIFT_REG:   0x03c               // RX Shift Register register
BYTE            COMP_SERIAL_RX_SHIFT_REG;
#pragma ioport  COMP_SERIAL_RX_BUFFER_REG:  0x03e               // RX Buffer Register
BYTE            COMP_SERIAL_RX_BUFFER_REG;
#pragma ioport  COMP_SERIAL_RX_FUNC_REG:    0x13c           // Function register
BYTE            COMP_SERIAL_RX_FUNC_REG;
#pragma ioport  COMP_SERIAL_RX_INPUT_REG:   0x13d           // Input register
BYTE            COMP_SERIAL_RX_INPUT_REG;
#pragma ioport  COMP_SERIAL_RX_OUTPUT_REG:  0x13e           // Output register
BYTE            COMP_SERIAL_RX_OUTPUT_REG;

#pragma ioport  COMP_SERIAL_TX_INT_REG:       0x0e1        // TX Interrupt Mask Register
BYTE            COMP_SERIAL_TX_INT_REG;
#pragma ioport  COMP_SERIAL_RX_INT_REG:       0x0e1        // RX Interrupt Mask Register
BYTE            COMP_SERIAL_RX_INT_REG;

// Masks to use with COMP_SERIAL_TX_INT_REG and COMP_SERIAL_RX_INT_REG Registers
#define COMP_SERIAL_TX_INT_MASK         (0x40) // TX Interrupt register Mask
#define COMP_SERIAL_RX_INT_MASK         (0x80) // RX Interrupt register Mask


//-------------------------------------------
//       WARNING WARNING WARNING
// The following defines and function prototypes
// are for backwards compatibility only and
// should not be used for new designs.
//-------------------------------------------
#pragma fastcall16  bCOMP_SERIAL_ReadTxStatus
#pragma fastcall16  bCOMP_SERIAL_ReadRxData
#pragma fastcall16  bCOMP_SERIAL_ReadRxStatus
extern BYTE  bCOMP_SERIAL_ReadTxStatus(void);
extern BYTE  bCOMP_SERIAL_ReadRxData(void);
extern BYTE  bCOMP_SERIAL_ReadRxStatus(void);
//------------------------------------
//  Parity masks
//------------------------------------
#define  UART_PARITY_NONE        0x00
#define  UART_PARITY_EVEN        0x02
#define  UART_PARITY_ODD         0x06

//------------------------------------
//  Transmitter Status Register masks
//------------------------------------
#define  UART_TX_COMPLETE        0x20
#define  UART_TX_BUFFER_EMPTY    0x10

//------------------------------------
//  Receiver Status Register masks
//------------------------------------
#define  UART_RX_ACTIVE          0x10
#define  UART_RX_COMPLETE        0x08
#define  UART_RX_PARITY_ERROR    0x80
#define  UART_RX_OVERRUN_ERROR   0x40
#define  UART_RX_FRAMING_ERROR   0x20
#define  UART_RX_NO_ERROR        0xE0

//-------------------------------------------
//             END WARNING
//-------------------------------------------
// end of file COMP_SERIAL.h

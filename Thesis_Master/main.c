//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        	// part specific constants and macros
#include "PSoCAPI.h"    	// PSoC API definitions for all User Modules
#include "psocdynamic.h"
#include <stdlib.h>
#include <string.h>
#pragma interrupt_handler TX_TIMEOUT_ISR
#pragma interrupt_handler RX_TIMEOUT_ISR

// These defines are used as parameters of the configToggle function.
// Passing one or the other in the function call switches the system between PC, TX, and RX modes.
#define		PC_MODE						(2)
#define		RX_MODE						(1)
#define		TX_MODE						(0)

// These defines are used as comparisons to find what port the newest module is connected to.
#define		PORT_1						('1')
#define		PORT_2						('2')
#define		PORT_3						('3')
#define		PORT_4						('4')

// These defines are used as transmission indicators.
#define		START_TRANSMIT				(252)	// Indicates the beginning of a transmission.
#define		END_TRANSMIT				(253)	// Indicates the end of a transmission.
#define		HELLO_BYTE					(200)	// Indicates master is ready to talk.
#define		ID_ASSIGNMENT				(201)	// Indicates an ID assignment from the master.
#define		ID_ASSIGN_OK				(202)	// Indicates an ID assignment is acknowledged.
#define		PING						(203)	// Indicates that someone is pinging someone else.
#define		CLEAR						(204)	// Indicates that the master is asking for a config clear.
#define		MASTER_ID					(0)		// The master node's ID.
#define		BROADCAST					(254)	// The broadcast ID for talking to all nodes.
#define		BLANK_MODULE_ID				(251)	// This is the ID of an unconfigured module.

// These defines are used to fill in the instruction we are using on the servo.
#define		PING_SERVO					(1)		// This is the instruction number for ping.
#define		READ_SERVO					(2)		// This is the instruction number for a read.
#define		WRITE_SERVO					(3)		// This is the instruction number for a write.
#define		RESET_SERVO					(6)		// This is the instruction to reset the servo EEPROM.

// These defines are used for transmission timing.
#define 	RX_TIMEOUT_DURATION			(3)		// This is receive wait time in 1 ms units.

#define		MAX_TIMEOUTS				(3)		// Number of timeouts allowed before hello mode exit.
#define		NUM_SWEEPS					(5)		// The number of module sweeps to do at init.

// This is the maximum number of allowable modules per branch out from the master
#define		MAX_MODULES					(250)

#define		SERVO_START					(255)

// This function receives a mode identifier as a parameter and toggles the
// system configuration between receive and transmit modes for half duplex UART.
void configToggle(int mode);

// This function pings the index passed to it. Returns 1 on success, 0 on fail.
int pingModule(int module_id);

// This function assigns an ID to a module.
int assignID(int assigned_ID);

int validTransmission(void);

void decodeTransmission(void);


void sayHello(void);

void servoInstruction(char id, char length, char instruction, char address, char value);
void longServoInstruction(char id, char length, char instruction, char address, char value1, char value2);

void clearConfig(void);
// This function checks the current mode and unloads the configuration for that mode.
void unloadAllConfigs(void);
// This function unloads the configuration corresponding to the number passed to it.
void unloadConfig(int config_num);
// Initialization function for the slave module controllers.
void initializeSlaves(void);
int initSweep(void);
// Static wait time of approximately 50 microseconds for use after starting a transmission.
void xmitWait(void);

// This flag is set if there is a timeout.
int TIMEOUT;

int NUM_MODULES;			// Stores the number of modules that have been discovered.
char COMMAND_SOURCE;		// Stores who the current command is from.
char COMMAND_DESTINATION;	// Stores who the current command is for.
char COMMAND_TYPE;			// Stores the type of command that was just read.
char PARAM[10];				// Stores a parameters that accompanies the command (if any).
int STATE;					// Stores the current configuration state of the system.

void main()
{	
	// Initialize the number of modules.
	NUM_MODULES = 0;
	
	// Activate GPIO ISR.
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO);
	
	// Turn on global interrupts for the transmission timeout timer.
	M8C_EnableGInt;
	
	while(1)
	{
		if(!NUM_MODULES)
		{
			initializeSlaves();
		}
		else if(COMP_SERIAL_bCmdCheck())		// If there's a computer command, read it.
		{
			decodeTransmission();
		}
	}
}

int pingModule(int module_id)
{
	int response = 0;
	
	configToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(module_id);
	TRANSMIT_PutChar(PING);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);	// Listen for the response.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!response))
	{
		if(validTransmission())
		{
			if(COMMAND_TYPE == PING)	// This is the response we are looking for.
			{
				// If this is for me, check who it was from.
				if(COMMAND_DESTINATION == MASTER_ID)
				{
					if(COMMAND_SOURCE == module_id)
					{
						response = 1;
					}
				}
			}
		}
	}
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	return response;
}

int assignID(int assigned_ID)
{
	int success = 0;		// Stores 0 on fail, 1 on success.
	
	configToggle(TX_MODE);	// Switch to TX mode.

	// Transmit the assignment.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(ID_ASSIGNMENT);
	TRANSMIT_PutChar(assigned_ID);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);	// Switch back to receive mode.
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	while((TIMEOUT < RX_TIMEOUT_DURATION) && (!success))
	{
		if(validTransmission())
		{
			if(COMMAND_TYPE == ID_ASSIGN_OK)	// This is the response we are looking for.
			{
				// If this is for me, check who it was from.
				if(COMMAND_DESTINATION == MASTER_ID)
				{
					if(COMMAND_SOURCE == assigned_ID)
					{
						success = 1;
					}
				}
			}
		}
	}
	
	RX_TIMEOUT_Stop();
	TIMEOUT = 0;
	RX_TIMEOUT_Start();
	
	return success;
}

void clearConfig(void)
{
	int response = 0;
	
	configToggle(TX_MODE);	// Toggle into TX mode.
			
	// Transmit a clear.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BROADCAST);
	TRANSMIT_PutChar(CLEAR);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
}

// This function transmits a hello message.
void sayHello(void)
{
	configToggle(TX_MODE);				// Toggle into TX mode.
			
	// Transmit a hello.
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(START_TRANSMIT);
	TRANSMIT_PutChar(MASTER_ID);
	TRANSMIT_PutChar(BLANK_MODULE_ID);
	TRANSMIT_PutChar(HELLO_BYTE);
	TRANSMIT_PutChar(END_TRANSMIT);
	TRANSMIT_PutChar(END_TRANSMIT);
	
	// Wait for the transmission to finish.
	while(!( TRANSMIT_bReadTxStatus() & TRANSMIT_TX_COMPLETE));
	
	xmitWait();
	
	configToggle(RX_MODE);				// Listen for the response.
}

// This function returns whether or not a valid transmission has been received.
int validTransmission(void)
{
	int valid_transmit = 0;
	int i = 0;
	char tempByte = 0;
	
	while(TIMEOUT < RX_TIMEOUT_DURATION)
	{
		if(RECEIVE_cReadChar() == START_TRANSMIT)
		{
			while(TIMEOUT < RX_TIMEOUT_DURATION)
			{
				if(RECEIVE_cReadChar() == START_TRANSMIT)
				{
					while(TIMEOUT < RX_TIMEOUT_DURATION)
					{
						if(tempByte = RECEIVE_cReadChar())
						{
							COMMAND_SOURCE = tempByte;
							
							while(TIMEOUT < RX_TIMEOUT_DURATION)
							{
								if(tempByte = RECEIVE_cReadChar())
								{
									if(tempByte >= HELLO_BYTE)
									{
										COMMAND_TYPE = tempByte;
										
										while(TIMEOUT < RX_TIMEOUT_DURATION)
										{
											if(tempByte = RECEIVE_cReadChar())
											{
												if(tempByte != END_TRANSMIT)
												{
													PARAM[i] = tempByte;
													i++;
												}
												else
												{
													valid_transmit = 1;
													TIMEOUT = RX_TIMEOUT_DURATION;
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	
	return valid_transmit;
}

// This function decodes the transmission and takes the correct action.
void decodeTransmission(void)
{
	char* param;
	char ID = 0;
	char tempByte;
	char angle[2];
	char speed[2];
	int total = 0;
	int runningTotal = 0;
	
	if(param = COMP_SERIAL_szGetParam())
	{
		if((param[0] == 'x') || (param[0] == 'X'))
		{
			// Reset
			NUM_MODULES = 0;
		}
		else if((param[0] == 'n') || (param[0] == 'N'))
		{
			itoa(param,NUM_MODULES,10);
			COMP_SERIAL_PutString(param);
			COMP_SERIAL_PutChar('\n');
		}
		else if((param[0] == 'w') || (param[0] == 'W'))
		{
			if(param = COMP_SERIAL_szGetParam())
			{
				ID = atoi(param);
				
				if(param = COMP_SERIAL_szGetParam())
				{
					if((param[0] == 'a') || (param[0] == 'A'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							total = atoi(param);
							angle[0] = total%256;
							angle[1] = total/256;
							longServoInstruction(ID,5,WRITE_SERVO,30,angle[0],angle[1]);
						}
					}
					else if((param[0] == 'p') || (param[0] == 'P'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							servoInstruction(ID,4,WRITE_SERVO,24,atoi(param));
						}
					}
					else if((param[0] == 's') || (param[0] == 'S'))
					{
						if(param = COMP_SERIAL_szGetParam())
						{
							total = atoi(param);
							
							// If no total, do nothing because 0 is no speed control (undesired).
							if(total)
							{
								speed[0] = total%256;
								speed[1] = total/256;
								longServoInstruction(ID,5,WRITE_SERVO,32,speed[0],speed[1]);
							}
						}
					}
				}
			}
		}
		else if((param[0] == 'r') || (param[0] == 'R'))
		{			
			if(param = COMP_SERIAL_szGetParam())
			{
				ID = atoi(param);
				if(param = COMP_SERIAL_szGetParam())
				{
					if((param[0] == 'a') || (param[0] == 'A'))
					{
						angle[0] = 0;
						angle[1] = 0;
						
						servoInstruction(ID,4,READ_SERVO,36,2);
						configToggle(RX_MODE);
							
						// Loop until we read a response or time out.
						while(TIMEOUT < RX_TIMEOUT_DURATION)
						{
							if(RECEIVE_cReadChar() == ID)
							{
								while(TIMEOUT < RX_TIMEOUT_DURATION)
								{
									if(RECEIVE_cReadChar() == 4)
									{
										if(RECEIVE_cGetChar() == 0)
										{
											angle[0] = RECEIVE_cGetChar();
											angle[1] = RECEIVE_cGetChar();
											
											configToggle(PC_MODE);
											
											total = ((angle[1])*256) + angle[0];
											itoa(param,total,10);
											COMP_SERIAL_PutString(param);
											COMP_SERIAL_PutChar('\n');

											TIMEOUT = RX_TIMEOUT_DURATION;
										}
										else
										{
											TIMEOUT = RX_TIMEOUT_DURATION;
										}
									}
								}
							}
						}
					}
					else if ((param[0] == 'p') || (param[0] == 'P'))
					{
						servoInstruction(ID,4,READ_SERVO,24,1);
						configToggle(RX_MODE);
						
						// Loop until we read a response or time out.
						while(TIMEOUT < RX_TIMEOUT_DURATION)
						{
							if(RECEIVE_cReadChar() == ID)
							{
								runningTotal = ID;
								// Loop until we read a response or time out.
								while(TIMEOUT < RX_TIMEOUT_DURATION)
								{
									// Check the length of the packet.
									if(RECEIVE_cReadChar() == 3)
									{
										runningTotal += 3;
										// Loop until we read a response or time out.
										while(TIMEOUT < RX_TIMEOUT_DURATION)
										{
											// Check for the checksum or 1.
											if(tempByte = RECEIVE_cReadChar())
											{
												configToggle(PC_MODE);
												
												if((runningTotal%256) == (255-tempByte))
												{
													// Send a 0 if we hit the checksum.
													COMP_SERIAL_PutChar('0');
													COMP_SERIAL_PutChar('\n');
												}
												else
												{
													// Send a 1.
													COMP_SERIAL_PutChar('1');
													COMP_SERIAL_PutChar('\n');
												}
		
												TIMEOUT = RX_TIMEOUT_DURATION;
											}
										}
									}
								}
							}
						}
					}
					else if ((param[0] == 't') || (param[0] == 'T'))
					{
						if(pingModule(ID))
						{
							configToggle(PC_MODE);
												
							COMP_SERIAL_PutChar(PARAM[0]);
							COMP_SERIAL_PutChar('\n');
						}
					}
					else if ((param[0] == 'c') || (param[0] == 'C'))
					{
						if(pingModule(ID))
						{	
							configToggle(PC_MODE);
							
							COMP_SERIAL_PutChar(PARAM[1]);
							COMP_SERIAL_PutChar('\n');
						}
					}
				}
			}
		}
	}
	
	if(STATE != PC_MODE)
	{
		configToggle(PC_MODE);
	}
	else
	{
		TIMEOUT = 0;
		COMP_SERIAL_CmdReset();
	}
}

// This function receives a destination, command length, instruction type, address, and value.
// With these parameters, the function sends a packet to the communication bus.
void servoInstruction(char id, char length, char instruction, char address, char value)
{
	char checksum;
	int total;
	
	total = id + length + instruction + address + value;
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-(total%256);
	
	// Talk to the servo.
	TX_REPEATER_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_PutChar(id);			// Servo ID
	TX_REPEATER_PutChar(length);		// The instruction length.
	TX_REPEATER_PutChar(instruction);	// The instruction to carry out.
	TX_REPEATER_PutChar(address);		// The address to read/write from/to.
	TX_REPEATER_PutChar(value);			// The value to write or number of bytes to read.
	TX_REPEATER_PutChar(checksum);		// This is the checksum.
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_bReadTxStatus() & TX_REPEATER_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
}

// This function receives a destination, command length, instruction type, address, and two values.
void longServoInstruction(char id, char length, char instruction, char address, char value1, char value2)
{
	char checksum;
	int total;
	
	total = id + length + instruction + address + value1 + value2;
	
	// Calculate the checksum value for our servo communication.
	checksum = 255-(total%256);
	
	// Talk to the servo.
	TX_REPEATER_PutChar(SERVO_START);	// Start byte one
	TX_REPEATER_PutChar(SERVO_START);	// Start byte two
	TX_REPEATER_PutChar(id);			// Servo ID
	TX_REPEATER_PutChar(length);		// The instruction length.
	TX_REPEATER_PutChar(instruction);	// The instruction to carry out.
	TX_REPEATER_PutChar(address);		// The address to read/write from/to.
	TX_REPEATER_PutChar(value1);		// The first value to write.
	TX_REPEATER_PutChar(value2);		// The first value to write.
	TX_REPEATER_PutChar(checksum);		// This is the checksum.
	
	// Wait for the transmission to finish.
	while(!(TX_REPEATER_bReadTxStatus() & TX_REPEATER_TX_COMPLETE));
	
	// Make completely sure we're done.
	xmitWait();
}

// This function allows the program to pass an RX or TX mode flag for switching between modes on the
// half duplex UART serial communication line.
void configToggle(int mode)
{
	// Disconnect from the global bus and leave the pin high.
	PRT0DR |= 0b11111111;
	PRT0GS &= 0b01000000;

	// Unload the configuration of the current state.
	// If there is no state, blindly wipe all configurations.
	if(STATE)
	{
		unloadConfig(STATE);
	}
	else
	{
		unloadAllConfigs();
	}
	
	if(mode == PC_MODE)
	{
		LoadConfig_pc_listener();

		COMP_SERIAL_CmdReset();							// Initialize the buffer.
		COMP_SERIAL_IntCntl(COMP_SERIAL_ENABLE_RX_INT); // Enable RX interrupts  
		COMP_SERIAL_Start(UART_PARITY_NONE);			// Starts the UART.
		
		TX_REPEATER_Start(TX_REPEATER_PARITY_NONE);		// Start the TX repeater.
		
		TIMEOUT = 0;
		STATE = PC_MODE;
	}
	else if(mode == RX_MODE)
	{
		LoadConfig_receiver_config();
		
		// Start the receiver.
		RECEIVE_Start(RECEIVE_PARITY_NONE);
		
		// Start response timeout timer and enable its interrupt routine.
		TIMEOUT = 0;
		RX_TIMEOUT_EnableInt();
		RX_TIMEOUT_Start();
		
		STATE = RX_MODE;
	}
	else if(mode == TX_MODE)
	{
		LoadConfig_transmitter_config();
		// Start the transmitter.
		TRANSMIT_Start(TRANSMIT_PARITY_NONE);
		
		TIMEOUT = 0;
		TX_TIMEOUT_EnableInt();	// Make sure interrupts are enabled.
		TX_TIMEOUT_Start();		// Start the timer.
		
		while(!TIMEOUT)
		{
			// Do nothing while we wait for one timeout period.
			// This is to allow everyone to get in the right configuration.
		}
		
		TX_TIMEOUT_Stop();		// Stop the timer.
		TIMEOUT = 0;			// Reset the timeout flag.
		
		STATE = TX_MODE;
	}
	
	// Reconnect to the global bus.
	PRT0GS |= 0b10111111;
}

// This function blindly unloads all user configurations. This will be called once,
// when the system initially has no known state.
void unloadAllConfigs(void)
{
	UnloadConfig_pc_listener();
	UnloadConfig_receiver_config();
	UnloadConfig_transmitter_config();
}

// This function unloads the configuration corresponding to the config number passed to it.
// We do this instead of unloadAllConfigs to cut down on set up time.
void unloadConfig(int config_num)
{
	if(config_num == PC_MODE)
	{
		UnloadConfig_pc_listener();
	}
	else if(config_num == RX_MODE)
	{
		UnloadConfig_receiver_config();
	}
	else if(config_num == TX_MODE)
	{
		UnloadConfig_transmitter_config();
	}
}

void initializeSlaves(void)
{
	int maxPrev = 0;	// The maximum previous value.
	int currVal;		// The current number of modules found.
	int i = 0;			// An int for looping.
	
	// Do nothing while we find nothing.
	while(!initSweep()) { }
	
	while(!maxPrev)
	{
		// Find the maximum value of modules found, it's our number.
		for(i = 0; i < NUM_SWEEPS; i++)
		{
			currVal = initSweep();
			
			if(currVal > maxPrev)
			{
				maxPrev = currVal;
			}
		}
	}
	
	// Sweep until we get the max number again.
	while(initSweep() != maxPrev) { }
	
	// Store the number of modules.
	NUM_MODULES = maxPrev;
	
	// Switch back to PC mode.
	configToggle(PC_MODE);
}

int initSweep(void)
{
	int i = 0;					// An iterator for looping.
	int num_timeouts = 0;		// The number of consecutive timeouts.
	int ping_tries = 5;			// The number of times to try a ping on an unregistered module.
	int currNumModules = 0;		// The number of modules found in this current sweep.
	
	// Clear the modules.
	clearConfig();
		
	// Send out a probing message.
	sayHello();
	
	// This loop continuously probes and listens at intervals
	// set by the RX_TIMEOUT_DURATION variable.
	while(num_timeouts < MAX_TIMEOUTS)
	{	
		if(validTransmission())
		{
			if(COMMAND_TYPE == HELLO_BYTE)	// Someone else is out there!
			{
				// If this is for me, assign them an ID.
				if(COMMAND_DESTINATION == MASTER_ID)
				{
					currNumModules++;		// Increment the number of modules connected.
					num_timeouts = 0;		// Reset number of timeouts since we found someone.
		
					if(!assignID(currNumModules))
					{
						// If the module did not respond that the ID was assigned,
						// make an effort to ping it in case that transmission was lost
						// before ultimately deciding that the module didn't configure.
						for(i = 0; i < ping_tries; i++)
						{	
							if(pingModule(currNumModules))
							{
								i = ping_tries+1;
							}
						}
						
						// If we landed right at ping_tries, we failed.
						if(i == ping_tries)
						{
							currNumModules--;
						}
					}
				}
			}
		}
		else if(TIMEOUT >= RX_TIMEOUT_DURATION)
		{	
			num_timeouts++;
			
			// If we are not maxed out on modules, look for more.
			if(currNumModules < MAX_MODULES)
			{
				sayHello();
			}
		}
	}
	
	// If we didn't find any new modules, check to see if some already exist.
	if(!currNumModules)
	{
		// Try to ping the next module up from our current number ping_tries times.
		for(i = 0; i < ping_tries; i++)
		{	
			if(pingModule(currNumModules+1))
			{
				currNumModules++;
				i = 0;
			}
		}
	}
	
	return currNumModules;
}

void xmitWait(void)
{
	int i;
	
	for(i = 0; i < 25; i++)
	{
		// Sit here and spin for about 50 microseconds.
	}
}

void TX_TIMEOUT_ISR(void)
{	
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,TX_TIMEOUT_INT_MASK);
}

void RX_TIMEOUT_ISR(void)
{	
	TIMEOUT++;
	
	M8C_ClearIntFlag(INT_CLR0,RX_TIMEOUT_INT_MASK);
}
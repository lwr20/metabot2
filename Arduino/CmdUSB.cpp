#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "HardwareSerial.h"
#include "CmdUSB.h"
#include "modebase.h"

// command line message buffer and pointer
static uint8_t msg[MAX_MSG_SIZE];
static uint8_t msg_index;

// text strings for command prompt
#define CMD_PROMPT ">> "

// External command processing routine
void process_cmd(int arg_cnt, char **args);


/**************************************************************************/
/*!
    Parse the command line. This function tokenizes the command input, then
    searches for the command table entry associated with the commmand. Once found,
    it will jump to the corresponding function.
*/
/**************************************************************************/
void cmd_parse(char *cmd)
{
    uint8_t argc, i = 0;
    char *argv[30];

    // parse the command line statement and break it up into space-delimited
    // strings. the array of strings will be saved in the argv array.
    argv[i] = strtok(cmd, " ");
    do
    {
        argv[++i] = strtok(NULL, " ");
    } while ((i < 30) && (argv[i] != NULL));
    
    // save off the number of arguments for the particular command.
    argc = i;

	process_cmd(argc, argv);

	Serial3.print(CMD_PROMPT); 
}

/**************************************************************************/
/*!
    This function processes the individual characters typed into the command
    prompt. It saves them off into the message buffer unless its a "backspace"
    or "enter" key. 
*/
/**************************************************************************/
void cmd_handler()
{
    char c = Serial3.read();

    switch (c)
    {
    case '\r':
        // terminate the msg and reset the msg ptr. then send
        // it to the handler for processing.
        msg[msg_index] = '\0';
        Serial3.print("\r\n");
        cmd_parse((char *)msg);
        msg_index = 0;
        break;
    
    case '\b':
        // backspace 
        Serial3.print(c);
        if (msg_index > 0)
        {
            msg_index--;
        }
        break;
    
    default:
        // normal character entered. add it to the buffer
        Serial3.print(c);
		msg[msg_index] = c;
		if (msg_index < (MAX_MSG_SIZE - 1))
			msg_index += 1;
        break;
    }
}

/**************************************************************************/
/*!
    This function should be set inside the main loop. It needs to be called
    constantly to check if there is any available input at the command prompt.
*/
/**************************************************************************/
void cmdPoll()
{
    while (Serial3.available())
    {
        cmd_handler();
    }
}

/**************************************************************************/
/*!
    Initialize the command line interface. This sets the terminal speed and
    and initializes things. 
*/
/**************************************************************************/
void cmdInit(uint32_t speed)
{
    // init the msg ptr
    msg_index = 0;

    // set the serial speed
    Serial3.begin(speed);
	Serial3.println("JP Version of CMD");
}

/**************************************************************************/
/*!
    Convert a string to a number. The base must be specified, ie: "32" is a
    different value in base 10 (decimal) and base 16 (hexadecimal).
*/
/**************************************************************************/
int32_t cmdStr2Num(char *str, uint8_t base)
{
    return strtol(str, NULL, base);
}

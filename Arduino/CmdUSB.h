#ifndef CMD_H
#define CMD_H

#define MAX_MSG_SIZE    60
#include <stdint.h>

void cmdInit(uint32_t speed);
void cmdPoll();
int32_t cmdStr2Num(char *str, uint8_t base);

#endif //CMD_H

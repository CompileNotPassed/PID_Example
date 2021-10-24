#include "headfile.h"

void vcan_sendware(void *wareaddr, uint32 waresize)
{
#define VCAN_PORT UART_1
#define CMD_WARE     3

    int16 i, tem;
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};

    for (i = 0; i < waresize; i += 2)
    {
        tem = ((uint8 *) wareaddr)[i];
        ((uint8 *) wareaddr)[i] = ((uint8 *) wareaddr)[i + 1];
        ((uint8 *) wareaddr)[i + 1] = tem;
    }
    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));
    uart_putbuff(VCAN_PORT, (uint8 *) wareaddr, waresize);
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));
}
#ifndef __UARTCMD_H__
#define __UARTCMD_H__

typedef struct _uartCmd_func_S
{
	char *str;
	void (*function)(char *param_str);
} uartCmd_func_t;

void uartCmd_task_init(void);

void uartCmd_Cmd(char *uartcmd);


#endif /*  __UARTCMD_H__ */


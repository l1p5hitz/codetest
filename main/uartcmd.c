#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "uartcmd.h"
#include "uartcmd_list.h"


static uartCmd_func_t* pfuncList = uartCmd_funcList;


#define UARTCMDPROC_BUF_LEN		128
static char uartCmdProc_buf[UARTCMDPROC_BUF_LEN];
static uint8_t uartCmdProc_len = 0;
static char uartCmd_last[UARTCMDPROC_BUF_LEN];

enum KEY_ACTION{
	KEY_NULL = 0,	    /* NULL */
	CTRL_A = 1,         /* Ctrl+a */
	CTRL_B = 2,         /* Ctrl-b */
	CTRL_C = 3,         /* Ctrl-c */
	CTRL_D = 4,         /* Ctrl-d */
	CTRL_E = 5,         /* Ctrl-e */
	CTRL_F = 6,         /* Ctrl-f */
	CTRL_H = 8,         /* Ctrl-h */
	BACKSPACE = 8,      /* Backspace */
	TAB = 9,            /* Tab */
	ENTER = 10, 		/* Enter */
	CTRL_K = 11,        /* Ctrl+k */
	CTRL_L = 12,        /* Ctrl+l */
	CTRL_N = 14,        /* Ctrl-n */
	CTRL_P = 16,        /* Ctrl-p */
	CTRL_T = 20,        /* Ctrl-t */
	CTRL_U = 21,        /* Ctrl+u */
	CTRL_W = 23,        /* Ctrl+w */
	ESC = 27,           /* Escape */
	DELETE =  127       /* Delete */
};

static char* uartCmdProcess_get(void)
{
	char ch;
	int nread;
	nread = fread(&ch, 1, 1, stdin);
	if(nread > 0)
	{
		if(ch == ESC)
		{
			char seq[3];
			if(fread(seq, 1, 2, stdin) < 2)
			{
				return NULL;
			}
			if (seq[0] == '[')
			{
				switch(seq[1])
				{
					case 'A': /* Up */
					{
						//display last cmd
						uint8_t len_last = strlen(uartCmd_last);
						if(len_last > 0)
						{
							uint8_t i;
							for(i=0; i<uartCmdProc_len; i++)
							{
								printf("\x08 \x08"); //backspace & clean with ' '
							}
							printf("%s", uartCmd_last);
							memcpy(uartCmdProc_buf, uartCmd_last, len_last);
							uartCmdProc_len = len_last;
						}
						break;
					}
					case 'B': /* Down */
					{
						//clear
						if(uartCmdProc_len)
						{
							uint8_t i;
							for(i=0; i<uartCmdProc_len; i++)
							{
								printf("\x08 \x08"); //backspace & clean with ' '
							}
							uartCmdProc_len = 0;
						}
						break;
					}
					case 'C': /* Right */
						break;
					case 'D': /* Left */
						break;
					case 'H': /* Home */
						break;
					case 'F': /* End*/
						break;
				}
			}
			return NULL;
		}
		
		uartCmdProc_buf[uartCmdProc_len] = ch;

		//enter
		if(uartCmdProc_buf[uartCmdProc_len] == '\n'
			|| uartCmdProc_buf[uartCmdProc_len] == '\r'
			|| uartCmdProc_len >= UARTCMDPROC_BUF_LEN-1)
		{
			printf("\r\n");
			if(uartCmdProc_len == 0)
			{
				//return NULL;
			}
			uartCmdProc_buf[uartCmdProc_len] = '\0';
			//store it to uartCmd_last
			if(uartCmdProc_len > 0)
			{
				strcpy(uartCmd_last, uartCmdProc_buf);
			}
			uartCmdProc_len = 0;
			return uartCmdProc_buf;
		}

		//backspace, delete
		if ((uartCmdProc_buf[uartCmdProc_len] == BACKSPACE)
			|| (uartCmdProc_buf[uartCmdProc_len] == DELETE))
		{
			if(uartCmdProc_len > 0)
			{
				printf("\x08 \x08");
				uartCmdProc_len--;
				uartCmdProc_buf[uartCmdProc_len] = '\0';
			}
			return NULL;
		}
		printf("%c", uartCmdProc_buf[uartCmdProc_len]);
		uartCmdProc_len++;
	}
	return NULL;
}

static int uartCmdProcess_hdl(char *uartcmd)
{
	uint16_t i;
	char buf[UARTCMDPROC_BUF_LEN];
	if(uartcmd==NULL)
		return -1;
	if(uartcmd[0]=='\0')
		return 0;
	memset(buf, '\0', UARTCMDPROC_BUF_LEN);
	sscanf(uartcmd, "%s", buf);
	if(buf[0]=='\0')
		return 0;

	for(i=0; pfuncList[i].str!=NULL; i++)
	{
		if(!strcmp(buf, pfuncList[i].str))
		{
			uint8_t cmd_len = strlen(pfuncList[i].str);
			uint8_t uartcmd_len = strlen(uartcmd);
			if(uartcmd_len > cmd_len+1)
				memcpy(buf, uartcmd+(cmd_len+1), uartcmd_len+1-(cmd_len+1));
			else
				buf[0]='\0';
			pfuncList[i].function(buf);
			return 0;
		}
	}
	return -1;
}


static void uartCmd_proc(void)
{
	char *uartcmd;
	int ret;

	uartcmd = uartCmdProcess_get();
	if(uartcmd==NULL)
	{
		return;
	}

	if(pfuncList==NULL)
	{
		printf("\r\nno func list\r\n");
		return;
	}
	if(uartcmd[0]!='\0')
	{
		ret = uartCmdProcess_hdl(uartcmd);
		if(ret==0)
		{
			printf("\r\n");
		}
		if(ret<0)
		{
			printf("\r\nunknown cmd [%s]\r\n", uartcmd);
		}
	}
	printf(">>");
}

static void uartCmd_task(void *pvParameters)
{
	memset(uartCmdProc_buf, 0, sizeof(uartCmdProc_buf));
	uartCmdProc_len = 0;
	memset(uartCmd_last, 0, sizeof(uartCmd_last));
	while(1)
	{
		uartCmd_proc();
		vTaskDelay(20/portTICK_RATE_MS); //sleep 20ms
	}
}

void uartCmd_task_init(void)
{
	xTaskCreate(&uartCmd_task, "uartCmd", 3072, NULL, 20, NULL);
}

void uartCmd_Cmd(char *uartcmd)
{
	uartCmdProcess_hdl(uartcmd);
}



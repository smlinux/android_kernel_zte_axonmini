#ifndef LPC5400X_VOICETRIGGER_H_
#define LPC5400X_VOICETRIGGER_H_

int lpc_voicetrigger_init(void **pHandle, void *client, void *pInitCfg);
int lpc_voicetrigger_cleanup(void *handle);
int lpc_voicetrigger_newCommand(void *handle, int index);


#endif	/* LPC5400X_VOICETRIGGER_H_ */


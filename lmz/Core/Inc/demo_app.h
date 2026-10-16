#ifndef __DEMO_APP_H__
#define __DEMO_APP_H__

void DemoApp_Init(void);
void DemoApp_Run(void);
void DemoApp_HandleBluetoothCommand(const char *cmd);
const char *DemoApp_GetPwmStatusText(void);

#endif

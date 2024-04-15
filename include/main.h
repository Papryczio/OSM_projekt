
#ifndef main_CL
#define main_CL


#include "setup.h"
#include "stdlib.h"
#include "R_P_LCD_TMSLAB.h"
#include "R_P_KEYBOARD_TMSLAB.h"
#include "R_P_LEDBAR_TMSLAB.h"
#include "square.h"

#ifdef TMSLAB_C2000
#include "F2837xD_device.h"
#include "systemInit.h"
#endif


void SetUpPeripherials();
void EnableInterrupts();
void InitData();
void ClearScreen();
void DrawPixels(int Key);

void shiftArray(double *array);
void regulate();
void DoubleToChar(unsigned char *text, double dbl);
void addObject(int point_x, int point_y, long object_size_x, long object_size_y, int index);
void drawChart(double *data, const char* text, int textLen, double value, const char* header, int headerLen);

// PID regulator settings
// Quick tune: 0.1, 0.01, 0.01, 0.1
#define setting_Kp 0.1
#define setting_Td 0.01
#define setting_Ti 0.01
#define setting_Tp 0.1

// Ship settings
#define max_speed 15

#endif

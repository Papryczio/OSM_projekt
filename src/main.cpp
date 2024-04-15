#include "main.h"

#ifdef TMSLAB_WIN
	#include "stdio.h"
#endif

#define BUFFERSYNC
#define WIN_PLOT

// Uncomment to have a debug option enabled on certain levels.
//#define DEBUG
#define DEBUG_PID
//#define DEBUG_OBJECT

unsigned long *ekran; // Adres obszaru graficznego LCD [8*128*2]

#ifdef TMSLAB_C2000
	unsigned char *textEkran; // Adres obszaru tekstowego [40*16/2]
#endif

#ifdef TMSLAB_WIN
	unsigned short int *textEkran; 	// Adres obszaru tekstowego [40*16/2]
	long Timer2IsrPeriod=1; 		// okres pracy symulowanego licznika Timer2 podany w przybli�eniu w ms
	extern int (*PartialRefresh)();
	char credits[43]="-               DEMO DISC                -";  // Tekst wyswietlany w naglowku symulatora
#endif

// Timer prescalers
unsigned int preScale = 0;
unsigned int preScale2 = 0;
unsigned int preScale3 = 0;

volatile char EnableRefresh = 0;    //Zezwolenie na odswiezenie zawartosci pamieci graficznej

R_P_LCD_TMSLAB LCD;             // Obiekt obslugujacy LCD
R_P_KEYBOARD_TMSLAB KEYBOARD;   // Obiekt obslugujacy klawiature
R_P_LEDBAR_TMSLAB LEDBAR;       // Obiekt obslugujacy diody LED

// User interface variables.
unsigned char Key;
unsigned int screen = 0;

// Screen settings and variables.
#define MaxObj 300
long size_x[MaxObj] = {0};
long size_y[MaxObj] = {0};

int dx[MaxObj];
int dy[MaxObj];
int s[MaxObj];
int x[MaxObj] = {0};
int y[MaxObj] = {0};

int screenSizeX = 240;
int screenSizeY = 120;
int screenMiddleX = screenSizeX / 2;
int screenMiddleY = screenSizeY / 2;

// Data to create characteristics.
#define MAX_DATA 180
int time = 0;

double positionInTime[MAX_DATA] = {0};
double errorInTime[MAX_DATA] 	= {0};
double controlInTime[MAX_DATA] 	= {0};

class Ship {
	public:
		double x_disruption;
		double x_set;
		double x_current;
		double maxSpeed;

		Ship() {
			this->x_disruption 	= 0;
			this->x_set 		= 0;
			this->x_current 	= 0;
			this->maxSpeed		= max_speed;
		}
};

class PID_Regulator {
	public:
		double Kp;
		double Td;
		double Ti;
		double Tp;
		double error;
		double errorDelayed;
		double lowerPreviousState;
		double control;

		PID_Regulator() {
			this->Kp = setting_Kp;
			this->Td = setting_Td;
			this->Ti = setting_Ti;
			this->Tp = setting_Tp;
			this->error = 0;
			this->errorDelayed = 0;
			this->lowerPreviousState = 0;
			this->control = 0;
		}
};

Ship myShip = Ship();
PID_Regulator PIDreg = PID_Regulator();

unsigned char Bufor[] = "KCode:  ";

int main() {
	SetUpPeripherials();
	#ifdef TMSLAB_C2000
    	LCD.LCD_Init(ekran, textEkran);
	#endif

	#ifdef TMSLAB_WIN
        LCD.LCD_Init(&ekran,&textEkran);
	#endif

    KEYBOARD.InitKB(100);
    LEDBAR.InitLedBar();
//    InitData();
    EnableInterrupts();

    while (1) {
    	EnableRefresh = 1;
    	LCD.Synchronize();
    	EnableRefresh = 0;

    	Key = KEYBOARD.GetKey();

    	// Uncomment to get button key displayed.
		//Bufor[6] = Key / 10 + '0';
		//Bufor[7] = Key % 10 + '0';
		//PrintText(textEkran, Bufor, 8, 16, 7);

		#ifdef TMSLAB_WIN
            if(PartialRefresh()) return 0;
		#endif
    }
}

// [HW] System interrupts
#ifdef TMSLAB_C2000

    interrupt
    void Timer2Isr() {
		#ifdef BUFFERSYNC
        	if (EnableRefresh)
        		LCD.PartialRefresh();
		#else
        	LCD.PartialRefresh();
		#endif

        KEYBOARD.PartialRefresh();

        if (++preScale == 50000) {
            preScale = 0;
            Tim++;
        }

        if (++preScale2 == 5000) {
        	preScale2 = 0;
        	regulate();
        }
    }

    unsigned long ADRFTECHED = 0;
    interrupt

    void NoIsr() {
        ADRFTECHED = PieCtrlRegs.PIECTRL.bit.PIEVECT;
        asm(" ESTOP0");
    }

    void EnableInterrupts() {
        EALLOW;

        //Ustawienie wektorow przerwan
        unsigned long VECTBEG = (unsigned long) &PieVectTable;
        unsigned long VECTLAST = (unsigned long) &PieVectTable
        + sizeof(PieVectTable);
        while (VECTBEG >= VECTLAST)
        *(unsigned long*) VECTBEG++ = (unsigned long) NoIsr;
        PieVectTable.TIMER2_INT = Timer2Isr;

        CpuTimer2Regs.TCR.bit.TIE = 1;
        CpuTimer2Regs.TCR.bit.TRB = 1;

        //Odblokuj przerwania
        IER = IER_MASK;
        asm(" push ier");
        asm(" pop dbgier");

        PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
        PieCtrlRegs.PIEACK.all = 0xffff;
        EDIS;
        EINT;
    }

    void SetUpPeripherials()
    {
        SetupCoreSystem();
        ekran = (unsigned long *) 0x8000;    	//[8*128*2]
        textEkran = (unsigned char*) 0x8a00;	//[40*16/2]
        EALLOW;
        //Okres licznika T2
        CpuTimer2Regs.PRD.all = System_Clk * Timer2ISR_Period;
        EDIS;
    }
    extern "C"
    {
    int _system_pre_init()
    {
        EALLOW;
        WdRegs.WDWCR.all = 0x68;
        EDIS;
        return (1);
    }
    }
#endif

// [SIM] Simulation of system interrupts.
#ifdef TMSLAB_WIN
    void EnableInterrupts() {}
    void SetUpPeripherials() {}

    void Timer2Isr() {
    	// Change those to manipulate time scale of the process
    	unsigned int sampleFrequency = 20;
    	unsigned int regulationFrequency = 100;
    	unsigned int disruptionFrequency = 50;

        if(++preScale == sampleFrequency) {
        	preScale=0;
        	time++;

        	// Save data about position in time
        	if (time < MAX_DATA) {
        		positionInTime[time] = myShip.x_current;
        	} else {
        		shiftArray(positionInTime);
        		positionInTime[MAX_DATA - 1] = myShip.x_current;
        	}

        	// Save data about control error
        	if (time < MAX_DATA) {
        		errorInTime[time] = PIDreg.error;
        	} else {
        		shiftArray(errorInTime);
        		errorInTime[MAX_DATA - 1] = PIDreg.error;
        	}

        	if (time < MAX_DATA) {
        		controlInTime[time] = PIDreg.control;
        	} else {
        		shiftArray(controlInTime);
        		controlInTime[MAX_DATA - 1] = PIDreg.control;
        	}

        	// Update display
        	ClearScreen();
        	DrawPixels(Key);
        }

        // Regulation prescaler.
        if (++preScale2 == regulationFrequency) {
        	preScale2 = 0;

        	regulate();
        }

        if (++preScale3 == disruptionFrequency) {
        	preScale3 = 0;
        	myShip.x_current += myShip.x_disruption;
        }
    }
#endif

/*
* Function used to shift all data in array by one backwards in order to make space for a newcomming data.
*/
void shiftArray(double *array) {
	for (int idx = 0; idx < MAX_DATA - 1; idx++) {
		*(array + idx) = *(array + (idx + 1));
		#ifdef DEBUG
		printf("Array at index %d: %lf", idx, *(array + idx));
		#endif
	}
}

void InitData()
{
	for (int a = 0; a < (128 * 8); a++) {
		ekran[a] = 0;
		for (int a = 0; a < (40 * 16); a++) {
			textEkran[a] = ' ';
		}
	}
}

void ClearScreen()
{
    for (int a = 0; a < (128 * 8); a++) {
    	ekran[a] = 0;
		for (int a = 0; a < (40 * 16); a++) {
			textEkran[a] = ' ';
		}
    }
}

void DrawPixels(int Key) {
	// Evaluate pressed key looking for the corresponding actions.
	// TODO: Adjust key numbers depending on working with hardware or simulation.
	switch(Key) {
		case 1:
			screen = 0;
			break;
		case 2:
			screen = 1;
			break;
		case 3:
			screen = 2;
			break;
		case 4:
			screen = 3;
			break;
		case 5:
			myShip.x_current = 20;
			break;
		case 6:
			myShip.x_current = -20;
			break;
		case 7:
			myShip.x_disruption += 0.5;
			break;
		case 8:
			myShip.x_disruption -= 0.5;
			break;
		default:
			#ifdef DEBUG
				printf("ERROR: Unknown key (%d) pressed!", Key);
			#endif
			break;
    }


	// Clear all objects
	for (int i = 0; i < MaxObj; i++) {
		x[i] = 0;
		y[i] = 0;
		size_x[i] = 0;
		size_y[i] = 0;
	}

	// Based on chosen screen draw boat simulation or characteristics.
	// Screen 1 [SIM] 	- main screen with boat visualization.
	// Screen 2 [CHART] - boat position in time (both set and current).
	// Screen 3 [CHART] - error in time.
	// Screen 4 [CHART] - control in time.
	switch(screen) {
		case 0:
		{
			int shipMiddleX = screenMiddleX + myShip.x_current;

			if (shipMiddleX <= 0) {
				shipMiddleX = 0;
			} else if (shipMiddleX >= 240) {
				shipMiddleX = 240;
			}

			addObject(shipMiddleX, screenMiddleY + 30, 30, 5, 0);
			addObject(shipMiddleX, screenMiddleY + 30, -30, 5, 1);
			addObject(shipMiddleX + 25, screenMiddleY + 30, 5, -15, 2);
			addObject(shipMiddleX - 25, screenMiddleY + 30, -5, -15, 3);

			break;
		}
		case 1:
		{
			drawChart(positionInTime, "Pozycja", 7, myShip.x_current, "Pozycja w czasie", 16);
			break;
		}
		case 2:
		{
			drawChart(errorInTime, "Uchyb", 5, PIDreg.error, "Uchyb w czasie", 14);
			break;
		}
		case 3:
		{
			drawChart(controlInTime, "Sterowanie", 10, PIDreg.control, "Sterowanie w czasie", 19);
			break;
		}
		default:
			#ifdef DEBUG
				printf("ERROR: Screen (%d) not found!", screen);
			#endif
			break;
	}

	// Draw all objects
	for (int c = 0; c < MaxObj; c++) {
		if (size_y[c] > 0) {
			for (int b = y[c]; b < (y[c] + size_y[c]); b++) {
				if (size_x[c] > 0) {
					for (int a = x[c]; a < (x[c] + size_x[c]); a++) {
						SetPixel(ekran, a, b);
					}
				} else {
					for (int a = x[c]; a > (x[c] + size_x[c]); a--) {
						SetPixel(ekran, a, b);
					}
				}
			}
		} else {
			for (int b = y[c]; b > (y[c] + size_y[c]); b--) {
				if (size_x[c] > 0) {
					for (int a = x[c]; a < (x[c] + size_x[c]); a++) {
						SetPixel(ekran, a, b);
					}
				} else {
					for (int a = x[c]; a > (x[c] + size_x[c]); a--) {
						SetPixel(ekran, a, b);
					}
				}
			}
		}
	}
}

/*
 * Function draws a chart with given text, value and header.
 * double *data 		- data set for a chart where index is time,
 * const char* text 	- text, ex. control, error etc,
 * int textLen 			- length of the text string,
 * double value 		- value of the given in text parameter,
 * cont char* header 	- header of a chart,
 * int headerLen 		- length of the header string.
 */
void drawChart(double *data, const char* text, int textLen, double value, const char* header, int headerLen) {
	addObject(0, 59, 240, 1, 0); // X-axis
	addObject(0, 0, 1, 120, 1);  // Y-axis

	unsigned char charValue[] = "";
	DoubleToChar(charValue, value);

 	PrintText(textEkran, text, textLen, 10, 12);
	PrintText(textEkran, charValue, 6, 11 + textLen, 12);
	PrintText(textEkran, header, headerLen, 1, 1);

	for (int idx = 0; idx < MAX_DATA; idx++) {
		int sample = (int)data[idx];

		if (sample + 60 <= 0){
			sample = 0;
		} else if (sample + 60 >= 120) {
			sample = 120;
		}
		addObject(idx + 2, sample + 60, 1, 1, idx + 2);
	}
}

    /*
     *  Function implements functionality of the PID regulator with upper and lower output limit (max velocity that the ship can reach).
     */
    void regulate() {
    	// Calculating error, which is difference between set and current value.
    	PIDreg.error = myShip.x_set - myShip.x_current;

    	// Calculate each branch of PID regulator.
    	double upper 		= PIDreg.error * PIDreg.Kp * (1 + PIDreg.Td / PIDreg.Tp);
    	double middle 		= PIDreg.errorDelayed * PIDreg.Kp * PIDreg.Td / PIDreg.Tp;
    	double lower 		= PIDreg.error * PIDreg.Kp * PIDreg.Tp / PIDreg.Ti + (PIDreg.lowerPreviousState - myShip.x_current);

    	// Setting delayed values.
    	PIDreg.errorDelayed 		= PIDreg.error;
    	PIDreg.lowerPreviousState 	= lower;

    	// Calculate output of the PID system (velocity) previously checking if it fits the set limits.
    	double control = upper - middle + lower;

    	if (control > myShip.maxSpeed) {
    		control = myShip.maxSpeed;
    	} else if (control < -myShip.maxSpeed) {
    		control = -myShip.maxSpeed;
    	}
    	PIDreg.control = control;

    	// Change current position of the ship by calculated control value.
    	myShip.x_current += control;

		#ifdef DEBUG_PID
			printf("upper: 		   %f \n", upper);
			printf("middle: 	   %f \n", middle);
			printf("lower: 		   %f \n", lower);
			printf("error: 		   %f \n", PIDreg.error);
			printf("curr position: %f \n", myShip.x_current);
			printf("set position:  %f \n", myShip.x_set);
			printf("disturb:       %f \n", myShip.x_disruption);
			printf("control:	   %f \n", control);
		#endif
    }

    /*
     * Function used to insert object to the array of objects that is later printed on the screen.
     * int point_x - x-axis location of an object
     * int point_y - y-axis location of an object
     * long object_size_x - size in x orientation (can be both positive and negative value)
     * long object_size_y - size in y orientation (can be both positive and negative value)
     */
    void addObject(int point_x, int point_y, long object_size_x, long object_size_y, int index) {
    	*(x + index) = point_x;
    	*(y + index) = point_y;
    	*(size_x + index) = object_size_x;
    	*(size_y + index) = object_size_y;

    	#ifdef DEBUG_OBJECT
    		printf("x point: %d\n", *(x + index));
    		printf("y point: %d\n", *(y + index));
    		printf("x size:  %ld\n", *(size_x + index));
    		printf("y size:  %ld\n", *(size_y + index));
    	#endif
    }

void DoubleToChar(unsigned char *text, double dbl)
{
    if (dbl >= 10 && dbl < 100) {
        text[2] = '.';
        int value[] = { 0, 0, 0, 0 };
        dbl = dbl * 100;
        value[0] = (int) dbl / 1000;
        dbl = dbl - value[0] * 1000;
        value[1] = (int) dbl / 100;
        dbl = dbl - value[1] * 100;
        value[2] = (int) dbl / 10;
        dbl = dbl - value[2] * 10;
        value[3] = (int) dbl;

        text[0] = value[0] + '0';
        text[1] = value[1] + '0';
        text[3] = value[2] + '0';
        text[4] = value[3] + '0';
        text[5] = ' ';
    } else if (dbl >= 0 && dbl < 100) {
        text[1] = '.';
        int value[] = { 0, 0, 0 };
        dbl = dbl * 100;
        value[0] = (int) dbl / 100;
        dbl = dbl - value[0] * 100;
        value[1] = (int) dbl / 10;
        dbl = dbl - value[1] * 10;
        value[2] = (int) dbl;

        text[0] = value[0] + '0';
        text[2] = value[1] + '0';
        text[3] = value[2] + '0';
        text[4] = ' ';
        text[5] = ' ';
    } else if (dbl <= -10 && dbl > -100) {
    	text[0] = '-';
    	text[3] = '.';
    	int value[] = { 0, 0, 0, 0 };
    	dbl = dbl * 100;
    	value[0] = (unsigned int) dbl / 1000;
    	dbl = dbl - value[0] * 1000;
    	value[1] = (unsigned int) dbl / 100;
    	dbl = dbl - value[1] * 100;
    	value[2] = (unsigned int) dbl / 10;
    	dbl = dbl - value[2] * 10;
    	value[3] = (unsigned int) dbl;

    	text[1] = value[0] + '0';
    	text[2] = value[1] + '0';
    	text[4] = value[2] + '0';
    	text[5] = value[3] + '0';
    } else if (dbl < 0 && dbl > -100) {
    	text[0] = '-';
    	text[2] = '.';
    	int value[] = { 0, 0, 0 };
		dbl = dbl * 100;
		value[0] = (unsigned int) dbl / 100;
		dbl = dbl - value[0] * 100;
		value[1] = (unsigned int) dbl / 10;
		dbl = dbl - value[1] * 10;
		value[2] = (unsigned int) dbl;

		text[1] = value[0] + '0';
		text[3] = value[1] + '0';
		text[4] = value[2] + '0';
		text[5] = ' ';
    }
}



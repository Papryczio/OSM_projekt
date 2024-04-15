
#ifndef setup_CL
#define setup_CL

//Czestotliwosc zegara CPU [MHz]
#define System_Clk 180UL
#define nsTime_to_Cycle(time) (time*System_Clk/1000)
//Przeliczona f zegara CPU [Hz]
#define System_Clk_Hz (System_Clk*1000000UL)
//Czestotliwosc kwarcu [MHz]
#define Kwarc_F 10UL

//Maska wszystkich uzywanych przerwan  M_INT14(TIMER2)
#define IER_MASK (M_INT14)

#define Timer2ISR_Period 10UL

#ifdef SIMULATOR
#define TMSLAB_WIN
#undef TMSLAB_C2000
#else
#define TMSLAB_C2000
#undef TMSLAB_WIN
#endif


#define SetPixel(Pointer,x,y) Pointer[(x>>5)+(y<<3)]|=(unsigned long)0x80000000>>(x&0x1f)
#define PrintText(Pointer,String,Len,x,y) for (int a0=0;a0<Len;a0++) Pointer[a0+x+y*40]=String[a0];

#endif

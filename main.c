/*
 * Robot.c
 *
 * Created: 1/27/2021 4:32:57 PM
 * Author : alecr
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include "njsTWI.h"
#include "MCP23008.h"
#include "MCP23008_LCD.h"
#include "lineSensor.h"

//********VARIABLES**********
char buffer[33];		//this buffer is bigger than 2 digits, no buffer overrun
uint8_t aiRaw;			//store the raw 0-255 A2D value
float aiFloat;			//used to convert the raw A2D values to a float
float B1;
float B2;
float B3;
volatile uint16_t lCount = 0;
volatile uint16_t rCount = 0;

uint16_t average[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t average2[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
uint16_t readCounts;
uint8_t array1[20] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
uint8_t array2[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array3[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array4[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array5[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array6[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array7[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array8[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array9[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array10[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array11[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array12[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array13[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array14[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array15[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array16[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array17[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array18[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array19[20] = {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
uint8_t array20[20] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

uint8_t i, j;

uint16_t rotation = 0;


int squareDrawing[22][22] = {	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0} ,   /*  initializers for row indexed by 0 */	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,   /*  initializers for row indexed by 1 */	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} ,	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0} , 	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0} , /*  initializers for row indexed by 2 */	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} };

int triangleDrawing[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

int halfCircle[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0} ,
	{0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0} ,
	{0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
};

int test[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} 
};
int spiral[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0},
	{0,0,0,0,1,0,0,1,0,0,1,1,0,0,1,0,0,0,0,0,0,0},
	{0,0,0,0,0,1,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

int spiral2[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0},
	{0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0},
	{0,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0},
	{0,0,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0},
	{0,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0},
	{0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,0,0,0},
	{0,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};
int middleEarth[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
};

	
int explored[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
};

int readInput[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ,
};

int manual[22][22] = {
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

int filledSquares[9][2];int originDistance = 42;int checkDistance;int closestPosition[2];int currentPosition[2] = {1,1}; //(x,y)int lastPosition[2] = {2,1};void reset_filledSquares(){;	for ( i = 0; i < 9; i++ ) { //makes filledSquares all 0		for ( j = 0; j < 2; j++ ) {			filledSquares[i][j]=0;		}	}}int q = 0;



//D2 left motor
ISR(INT0_vect)
{
	lCount++;
}

//External interrupt INT1 - D3 on atmega1284
//right motor
ISR(INT1_vect)
{
	rCount++;
}

void calibrate(){
	for(i = 1; i <=20; i++){
		for(j = 0; j < 10; j++){
			_delay_ms(10);
			average[i-1] = average[i -1] + irSensorRead(i);
		}
		average[i-1] = average[i-1]/10;
		sprintf(buffer, "%u", i);
		lcd_clrscr(); lcd_home();
		lcd_puts(buffer);
		lcd_puts(": ");
		sprintf(buffer, "%u", average[i-1]);
		lcd_puts(buffer);
		_delay_ms(50);
	}
}

void testIR(int sensorNumber){
	readCounts = irSensorRead(sensorNumber);
	sprintf(buffer, "%u", readCounts);
	lcd_clrscr(); lcd_home();
	lcd_puts(buffer);
}

void step_forward(uint16_t count, uint8_t pulse, uint8_t little){
	PORTB &= ~(1<<PORTB1);
	PORTB |= (1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(rCount < count && lCount < count){
		OCR0A = pulse;
		OCR0B = pulse;
		/*if(rCount >= 225 && rCount >= 225){
			OCR0A = 0;
			OCR0B = 0;
			break;
		}*/
	}
	OCR0A = 0;
	OCR0B = 0;
	while(lCount < count){ //add 10 turns left
		OCR0A = pulse;
	}
	while(rCount < count+1){ //add 10 turns right
		OCR0B = pulse;
	}

	OCR0A = 0;
	OCR0B = 0;
}

void u_turn(){
	PORTB &= ~(1<< PORTB1);
	PORTB |= (1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(1){
		if(lCount < 915){
			OCR0A = 100;
			OCR0B = 0;
		}
		else if (lCount > 915){
			OCR0A = 0;
			break;
		}
	}
}

void R_Turn_90(){ //in place turn
	PORTB |=(1<<PORTB1);
	PORTB |=(1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(rCount < 200 && lCount < 210){
		OCR0A = 125;
		OCR0B = 125;
		/*if(rCount >= 225 && rCount >= 225){
			OCR0A = 0;
			OCR0B = 0;
			break;
		}*/
	}
	OCR0A = 0;
	OCR0B = 0;
	while(lCount < 210){
		OCR0A = 125;
	}
	while(rCount < 200){
		OCR0B = 125;
	}

	OCR0A = 0;
	OCR0B = 0;
	lcd_clrscr(); lcd_home();
	sprintf(buffer, "%u", lCount);
	lcd_puts(buffer);
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%u", rCount);
	lcd_puts(buffer);
	_delay_ms(1000);
}

void L_Turn_90(){ //in place turn
	PORTB &=~(1<<PORTB1);
	PORTB &=~(1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(rCount < 200 && lCount < 220){
		OCR0A = 125;
		OCR0B = 125;
		/*if(rCount >= 225 && rCount >= 225){
			OCR0A = 0;
			OCR0B = 0;
			break;
		}*/
	}
	OCR0A = 0;
	OCR0B = 0;
	lcd_clrscr(); lcd_home();
	sprintf(buffer, "%u", lCount);
	lcd_puts(buffer);
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%u", rCount);
	lcd_puts(buffer);
	_delay_ms(2000);
	while(rCount < 200){
		OCR0B = 125;
	}
	OCR0B = 0;
	while(lCount < 220){
		OCR0A = 125;
	}
	OCR0A = 0;
	lcd_clrscr(); lcd_home();
	lcd_puts("done");
	lcd_clrscr(); lcd_home();
	sprintf(buffer, "%u", lCount);
	lcd_puts(buffer);
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%u", rCount);
	lcd_puts(buffer);
	
}

void R_Turn_45(){
	PORTB |=(1<<PORTB1);
	PORTB |=(1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(rCount < 86 && lCount < 101){
		OCR0A = 125;
		OCR0B = 125;
		/*if(rCount >= 225 && rCount >= 225){
			OCR0A = 0;
			OCR0B = 0;
			break;
		}*/
	}
	OCR0A = 0;
	OCR0B = 0;
	while(lCount < 101){
		OCR0A = 125;
	}
	while(rCount < 86){
		OCR0B = 125;
	}

	OCR0A = 0;
	OCR0B = 0;
}

void L_Turn_45(){
	PORTB &=~(1<<PORTB1);
	PORTB &=~(1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(rCount < 86 && lCount < 101){
		OCR0A = 125;
		OCR0B = 125;
		/*if(rCount >= 225 && rCount >= 225){
			OCR0A = 0;
			OCR0B = 0;
			break;
		}*/
	}
	OCR0A = 0;
	OCR0B = 0;
	while(lCount < 101){
		OCR0A = 125;
	}
	while(rCount < 86){
		OCR0B = 125;
	}

	OCR0A = 0;
	OCR0B = 0;
}


void right(){ //single wheel turn
	PORTB &=~(1<< PORTB1);
	PORTB |= (1<<PORTB2);//left wheel
	lCount = 0;
	rCount = 0;
	while(1){
		if(lCount < 455){
			OCR0A = 200; //left wheel
			OCR0B = 0;
		}
		else if (lCount > 455){
			OCR0A = 0;
			break;
		}
	}
}

void right_60(){
	PORTB &=~(1<< PORTB1);
	PORTB |= (1<<PORTB2);
	lCount = 0;
	rCount = 0;
	while(1){
		if(lCount < 613){
			OCR0A = 200;
			OCR0B = 0;
		}
		else if (lCount > 613){
			OCR0A = 0;
			break;
		}
	}
}

void left(){
	PORTB |=(1<<PORTB1);
	PORTB &=~(1<<PORTB2);
	rCount = 0;

	while(1){
		if(rCount < 455){
			OCR0B = 200;
			OCR0A = 0;
		}
		else if(rCount> 455){
			OCR0B = 0;
			break;
		}
	}
}

void circle(){
	PORTB &= ~(1<<PORTB1);
	PORTB|=(1<<PORTB2);
	lCount = 0;
	OCR0A = 100;
	OCR0B = 153;
	_delay_ms(20000);
	OCR0A = 0;
	OCR0B = 0;
}

void arc(){
	PORTB &=~(1<<PORTB1);
	PORTB|=(1<<PORTB2);
	lCount = 0;
	while(1){
		if(lCount < 811){
			OCR0A = 100;
			OCR0B = 153;
		}
		else if(lCount >=811){
			OCR0A = 0;
			OCR0B = 0;
		}
	}
}

void find_closest_spot(int drawing[22][22]){
	for (uint8_t i = 1; i <= 21; i++ ) {		//col loop		for (uint8_t j = 1; j <= 21; j++ ) {
			if(drawing[i][j] == 1){
				checkDistance = (i + j) - 2;
				if(checkDistance <= originDistance){
					closestPosition[0] = i;
					closestPosition[1] = j;
					originDistance = checkDistance;
				}
			}
		}
	}
}

void go_to_closest_spot(uint8_t distance){
	R_Turn_90();
	for(i = 1; i <= closestPosition[0]; i++){
		step_forward(distance, 100, 0);
		_delay_ms(500);
	}
	R_Turn_90();
	for(j = 1; j <= closestPosition[1]; j++){
		step_forward(distance, 100, 0);
		_delay_ms(500);
	}
	rotation = 180;
	currentPosition[0] = closestPosition[0];
	currentPosition[1] = closestPosition[1];
}

void find_next_spot(int drawing[22][22]){ //finds the filled spots, except for ones the robot has traversed, and the ones that the robot is on, typically should only get one value
	reset_filledSquares();
	if(currentPosition[0] >= 1 && currentPosition[1] >= 1 && currentPosition[0] <= 21 && currentPosition[1] <= 21) { //if its everywhere in between
		lcd_clrscr();lcd_gotoxy(0,0);
		q = 0;
		for ( i = currentPosition[0]-1; i <= currentPosition[0]+1; i++ ) {			//col loop			for ( j = currentPosition[1]-1; j <= currentPosition[1]+1; j++ ) {				if(drawing[i][j] == 1 && (currentPosition[0] != i || currentPosition[1] != j)){					if(explored[i][j] == 0){						filledSquares[q][0] = 1;						filledSquares[q][1] = 1;					}				}				q++;			}		}
	}
}
void turn_to_spot(){
	if(filledSquares[1][0] == 1 && filledSquares[1][1] == 1){
		//turn to 0 degree, up //good
		switch(rotation){
			case 0:
			rotation = 0;
			//do nothing
			break;
			case 45:
			rotation = 0;
			L_Turn_45();
			//turn 45 to 0, 1 reverse 45 turn
			break;
			case 90:
			rotation = 0;
			//turns 90 to 0, 2 reverse 45 turns
			L_Turn_90();
			break;
			case 135:
			rotation = 0;
			//turns 135 to 0, 3 reverse 45 turns
			L_Turn_90();
			L_Turn_45();
			break;
			case 180:
			rotation = 0;
			R_Turn_90();
			R_Turn_90();
			break;
			case 225:
			rotation = 0;
			//225 to 0, 3 45 turn
			R_Turn_45();
			R_Turn_90();
			break;
			case 270:
			rotation = 0;
			//270 to 0, 2 45 turn
			R_Turn_90();
			break;
			case 315:
			rotation = 0;
			//turn 315 to 0, 1 45 turns
			R_Turn_45();
			break;
		}
	}
	else if(filledSquares[3][0] == 1 && filledSquares[3][1] == 1){
		//turn to 270 degree, left //good
		switch(rotation){
			case 0:
			rotation = 270;
			//turn 0 to 270, 6 45 turns
			L_Turn_90();
			break;
			case 45:
			rotation = 270;
			//turn 45 to 270, 5 45 turns
			L_Turn_90();
			L_Turn_45();
			break;
			case 90:
			rotation = 270;
			//turns 90 to 270, 4 45 turns
			L_Turn_90();
			L_Turn_90();
			break;
			case 135:
			rotation = 270;
			//turns 135 to 270, 3 45 turn
			R_Turn_45();
			R_Turn_90();
			break;
			case 180:
			rotation = 270;
			//180 to 270, 2 45 turn
			R_Turn_90();
			break;
			case 225:
			rotation = 270;
			//225 to 270, 1 45 turn
			R_Turn_45();
			break;
			case 270:
			rotation = 270;
			//do nothing
			break;
			case 315:
			rotation = 270;
			//turn 315 to 270, 1 reverse 45 turns
			L_Turn_45();
			break;
		}
	}
	else if(filledSquares[5][0] == 1 && filledSquares[5][1] == 1){
		//turn to 90 degree, right //good
		switch(rotation){
			case 0:
			rotation = 90;
			//turn 0 to 90, 2 45 turns
			R_Turn_90();
			break;
			case 45:
			rotation = 90;
			//turn 45 to 90, 1 45 turns
			R_Turn_45();
			break;
			case 90:
			rotation = 90;
			//do nothing
			break;
			case 135:
			rotation = 90;
			//turns 135 to 90, 1 reverse 45 turns
			L_Turn_45();
			break;
			case 180:
			rotation = 90;
			//180 to 90, 2 reverse 45 turn
			L_Turn_90();
			break;
			case 225:
			rotation = 90;
			//225 to 90, 3 reverse 45 turn
			L_Turn_90();
			L_Turn_45();
			break;
			case 270:
			rotation = 90;
			//270 to 90, 4 reverse 45 turn
			L_Turn_90();
			L_Turn_90();
			break;
			case 315:
			rotation = 90;
			//turn 315 to 90, 1 reverse 45 turns
			R_Turn_45();
			R_Turn_90();
			break;
		}
	}
	else if(filledSquares[7][0] == 1 && filledSquares[7][1] == 1){
		//turn to 180 degree, down
		switch (rotation){
			case 0:
			rotation = 180;
			//turn 0 to 180, 4 45 turns
			R_Turn_90();
			R_Turn_90();
			break;
			case 45:
			rotation = 180;
			//turn 45 to 180, 3 45 turns
			R_Turn_90();
			R_Turn_45();
			break;
			case 90:
			rotation = 180;
			//turns 90 to 180, 2 45 turns
			R_Turn_90();
			break;
			case 135:
			rotation = 180;
			//turns 135 to 180, 1 45 turns
			R_Turn_45();
			break;
			case 180:
			rotation = 180;
			//do nothing
			break;
			case 225:
			rotation = 180;
			//turn 225 to 180, 1 reverse 45 turn
			L_Turn_45();
			break;
			case 270:
			rotation = 180;
			//turn 270 to 180, 2 reverse 45 turn
			L_Turn_90();
			break;
			case 315:
			rotation = 180;
			//turn 315 to 180, 3 reverse 45 turns
			L_Turn_90();
			L_Turn_45();
			break;
		}
	}
	else if(filledSquares[6][0] == 1 && filledSquares[6][1] == 1){
		//turn to 215 degree, lower left //good
		switch (rotation){
			case 0:
			rotation = 225;
			//turn 0 to 225, 5 45 turns
			R_Turn_90();
			R_Turn_90();
			R_Turn_45();
			break;
			case 45:
			rotation = 225;
			//turn 45 to 225, 4 45 turns
			R_Turn_90();
			R_Turn_90();
			break;
			case 90:
			rotation = 225;
			//turns 90 to 225, 3 45 turns
			R_Turn_90();
			R_Turn_45();
			break;
			case 135:
			rotation = 225;
			//turns 135 to 225, 2 45 turns
			R_Turn_90();
			break;
			case 180:
			rotation = 225;
			//180 to 225, 1 45 turn
			R_Turn_45();
			break;
			case 225:
			rotation = 225;
			break;
			case 270:
			rotation = 225;
			//turn 270 to 225, 1 reverse 45 turn
			L_Turn_45();
			break;
			case 315:
			rotation = 225;
			//turn 315 to 225, 2 reverse 45 turns
			L_Turn_90();
			break;
		}
	}
	else if(filledSquares[8][0] == 1 && filledSquares[8][1] == 1){
		//turn to 135 degree, lower right
		switch (rotation){
			case 0:
			rotation = 135;
			//turn 0 to 135, 3 45 turns
			R_Turn_90();
			R_Turn_45();
			break;
			case 45:
			rotation = 135;
			//turn 45 to 135, 2 45 turns
			R_Turn_90();
			break;
			case 90:
			rotation = 135;
			//turns 90 to 135, 1 45 turns
			R_Turn_45();
			break;
			case 135:
			rotation = 135;
			//do nothing
			break;
			case 180:
			rotation = 135;
			//turn 180 to 135, reverse 45 turn
			L_Turn_45();
			break;
			case 225:
			rotation = 135;
			//turn 225 to 135, 2 reverse 45 turn
			L_Turn_90();
			break;
			case 270:
			rotation = 135;
			//turn 270 to 135, 3 reverse 45 turn
			L_Turn_90();
			L_Turn_45();
			break;
			case 315:
			rotation = 135;
			//turn 315 to 135, 4 reverse 45 turns
			L_Turn_90();
			L_Turn_90();
			break;
		}
	}
	else if(filledSquares[0][0] == 1 && filledSquares[0][1] == 1){
		//turn to 315 degree, upper left //good
		switch(rotation){
			case 0:
			rotation = 315;
			//turn 0 to 315, 1 reverse 45 turns
			L_Turn_45();
			break;
			case 45:
			rotation = 315;
			//turn 45 to 315, 2 reverse 45 turns
			L_Turn_90();
			break;
			case 90:
			rotation = 315;
			//turns 90 to 315, 3 reverse 45 turns
			L_Turn_90();
			L_Turn_45();
			break;
			case 135:
			rotation = 315;
			//turns 135 to 315, 4 reverse 45 turns
			L_Turn_90();
			L_Turn_90();
			break;
			case 180:
			rotation = 315;
			//180 to 315, 2 45 turn
			R_Turn_90();
			R_Turn_45();
			break;
			case 225:
			rotation = 315;
			//225 to 315, 2 45 turn
			R_Turn_90();
			break;
			case 270:
			rotation = 315;
			//270 to 315, 1 45 turn
			R_Turn_45();
			break;
			case 315:
			rotation = 315;
			//do nothing
			break;
		}
	}
	else if(filledSquares[2][0] == 1 && filledSquares[2][1] == 1){
		//turn to 45 degree, upper right //good
		switch(rotation){
			case 0:
			rotation = 45;
			//turn 0 to 45, 1 45 turns
			R_Turn_45();
			break;
			case 45:
			rotation = 45;
			//do nothing
			break;
			case 90:
			rotation = 45;
			//turns 90 to 45, 1 reverse 45 turns
			L_Turn_45();
			break;
			case 135:
			rotation = 45;
			//turns 135 to 45, 2 reverse 45 turns
			L_Turn_90();
			break;
			case 180:
			rotation = 45;
			//180 to 45, 3 reverse 45 turn
			L_Turn_90();
			L_Turn_45();
			break;
			case 225:
			rotation = 45;
			//225 to 45, 4 reverse 45 turn
			L_Turn_90();
			L_Turn_90();
			break;
			case 270:
			rotation = 45;
			//270 to 45, 3 45 turns
			R_Turn_90();
			R_Turn_45();
			break;
			case 315:
			rotation = 45;
			//turn 315 to 45, 2 45 turns
			R_Turn_90();
			break;
		}
	}
}

void move_Forward(uint8_t distance, uint8_t pulse){
	if(filledSquares[0][0] == 1 || filledSquares[0][1] == 1 || filledSquares[1][0] == 1 || filledSquares[1][1] == 1 || filledSquares[2][0] == 1 || filledSquares[2][1] == 1 || filledSquares[3][0] == 1 || filledSquares[3][1] == 1 || filledSquares[5][0] == 1 || filledSquares[5][1] == 1 || filledSquares[6][0] == 1 || filledSquares[6][1] == 1 || filledSquares[7][0] == 1 || filledSquares[7][1] == 1 || filledSquares[8][0] == 1 || filledSquares[8][1] == 1){
		step_forward(distance, pulse, 0);
		explored[currentPosition[0]][currentPosition[1]] = 1;
		if(rotation == 0){
			currentPosition[0] = currentPosition[0] - 1;
		}
		else if(rotation == 45){
			currentPosition[0] = currentPosition[0] - 1;
			currentPosition[1] = currentPosition[1] + 1;
		}
		else if(rotation == 90){
			currentPosition[1] = currentPosition[1] + 1;
		}
		else if(rotation == 135){
			currentPosition[0] = currentPosition[0] + 1;
			currentPosition[1] = currentPosition[1] + 1;
		}
		else if(rotation == 180){
			currentPosition[0] = currentPosition[0] + 1;
		}
		else if(rotation == 225){
			currentPosition[0] = currentPosition[0] + 1;
			currentPosition[1] = currentPosition[1] - 1;
		}
		else if(rotation == 270){
			currentPosition[1] = currentPosition[1] - 1;
		}
		else if(rotation == 315){
			currentPosition[0] = currentPosition[0] - 1;
			currentPosition[1] = currentPosition[1] - 1;
		}
	}
}

void origin(int drawing[22][22], uint8_t distance){
	find_closest_spot(drawing);
	lcd_clrscr(); lcd_home();
	sprintf(buffer, "%d", closestPosition[0]);
	lcd_puts(buffer);
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%d", closestPosition[1]);
	lcd_puts(buffer);
	go_to_closest_spot(distance);
}

void draw(int drawing[22][22], uint8_t distance){
	find_next_spot(drawing);
	lcd_clrscr();lcd_home();
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%d", rotation);
	lcd_puts(buffer);
	
	turn_to_spot();
	lcd_gotoxy(0, 1);
	sprintf(buffer, "%d", rotation);
	lcd_puts(buffer);
	
	move_Forward(distance, 100);
}

void cleanUp(int drawing[22][22]){
	for(i = 0; i < 22; i++){
		drawing[i][0] = 0;
		drawing[0][i] = 0;
		drawing[21][i] = 0;
		drawing[i][21] = 0;
	}
	for(i = 0; i <22; i++){
		for(j = 0; j <22; j++){
			if(drawing[i - 1][j - 1] == 0 && drawing[i - 1][j] == 0 && drawing[i - 1][j + 1] == 0 && drawing[i][j-1] == 0 && drawing[i][j+1] == 0 && drawing[i+1][j-1] == 0 && drawing[i+1][j] == 0 && drawing[i+1][j+1] == 0){
				drawing[i][j] = 0;
			}
			if((drawing[i - 1][j] == 1 && drawing[i+1][j] == 1) || (drawing[i][j-1] == 1 && drawing[i][j+1])){
				drawing[i][j] = 1;
			}
		}
	}
	for(i = 0; i <20; i ++){
		_delay_ms(1000);
		lcd_clrscr(); lcd_home();
		for(j = 0; j <20; j++){
			sprintf(buffer, "%u", readInput[i+1][j+1]);
			if(j<10){
				lcd_gotoxy(j,0);
			}
			else if(j >= 10){
				lcd_gotoxy(j -10,1);
			}
			lcd_puts(buffer);
		}
	}
	
}
void rectangle(){
	step_forward(800, 100, 0);
	R_Turn_90();
	step_forward(800, 100, 0);
	R_Turn_90();
	step_forward(800, 100, 0);
	R_Turn_90();
	step_forward(800, 100, 0);
}

void triangle(){
	step_forward(800, 100, 0);
	right_60();
	step_forward(800, 100, 0);
	right_60();
	step_forward(800, 100, 0);
}

void readGrid(){
	uint8_t forwardCounts = 11;
	for(i = 0; i < 21; i++){
		if(i >= 1){
			if(i%2 == 0){
				forwardCounts = 11;
			}
			else{
				forwardCounts = 12;
			}
			step_forward(forwardCounts, 80, 0);
		}
	//	_delay_ms(100);
		for(j = 0; j < 20; j++){
			for(uint8_t z = 0; z < 10; z ++){
				average2[j] = average2[j] + irSensorRead(j+1);
			}
			average2[j] = average2[j]/10;
			if(average2[j] >= average[j]*.5){
				readInput[i + 1][j + 1] = 1;
			}
			/*lcd_clrscr(); lcd_home();
			sprintf(buffer, "%u", j+1);
			lcd_puts(buffer);
			lcd_puts(": ");
			sprintf(buffer, "%u", irSensorRead(j+1));
			lcd_puts(buffer);
			_delay_ms(1000);*/
		}
		lcd_clrscr(); lcd_home();
		for(j = 0; j <20; j++){
			sprintf(buffer, "%u", readInput[i+1][j+1]);
			if(j<10){
				lcd_gotoxy(j,0);
			}
			else if(j >= 10){
				lcd_gotoxy(j -10,1);
			}
			lcd_puts(buffer);
		}
	}
}

uint8_t analogRead(uint8_t a2dChannel)
{
	//set ADMUX bits for given channel
	switch (a2dChannel)
	{
		case 0:					//A2D Channel 0
		ADMUX &= ~( (1<<MUX0)|(1<<MUX1)|(1<<MUX2) );
		break;
		case 1:					//A2D Channel 1
		ADMUX |= (1<<MUX0);
		ADMUX &= ~( (1<<MUX1)|(1<<MUX2) );
		break;
		case 2:					//A2D Channel 2
		ADMUX |= (1<<MUX1);
		ADMUX &= ~( (1<<MUX0)|(1<<MUX2) );
		break;
		case 3:					//A2D Channel 3
		ADMUX |= (1<<MUX0)|(1<<MUX1);
		ADMUX &= ~(1<<MUX2);
		break;
		case 4:					//A2D Channel 4
		ADMUX |= (1<<MUX2);
		ADMUX &= ~( (1<<MUX0)|(1<<MUX1) );
		break;
		case 5:					//A2D Channel 5
		ADMUX |= (1<<MUX0)|(1<<MUX2);
		ADMUX &= ~(1<<MUX1);
		case 6:					//A2D Channel 6
		ADMUX |= (1<<MUX1)|(1<<MUX2);
		ADMUX &= ~(1<<MUX0);
		break;
		case 7:					//A2D Channel 7
		ADMUX |= (1<<MUX0)|(1<<MUX1)|(1<<MUX2);
		break;
	} //end switch
	ADCSRA |= (1<<ADIF);				//clear hardware "conversion complete" flag
	ADCSRA |= (1<<ADSC);				//start conversion
	while( bit_is_set(ADCSRA, ADSC));	//wait for conversion to be done, patience grasshopper
	return ADCH;
} //end analogRead

void voltageCheck() {
	 //read and print channel 0
	 aiRaw = analogRead(0);
	 aiFloat = aiRaw;
	 aiFloat = (aiFloat/255)*5;
	 B1 = aiFloat;
	 lcd_gotoxy(0,0);
	 sprintf(buffer, "C0 %.02f", B1);
	 lcd_puts(buffer);
	 //read and print channel 1
	 aiRaw = analogRead(1);
	 aiFloat = aiRaw;
	 aiFloat = (aiFloat/255)*5;
	 B2 = (aiFloat * 2) - B1;
	 lcd_gotoxy(8,0);
	 sprintf(buffer, "C1 %.02f", B2);
	 lcd_puts(buffer);
	 //read and print channel 3
	 aiRaw = analogRead(2);
	 aiFloat = aiRaw;
	 aiFloat = (aiFloat/255)*5;
	 B3 = (aiFloat * 3.2) - (B1 + B2);
	 lcd_gotoxy(0,1);
	 sprintf(buffer, "C2 %.02f", B3);
	 lcd_puts(buffer);
	 //delay before repeating
	 if(B1 < 2.5 || B2 < 2.5 || B3 < 2.4){
		 lcd_option();
		 //PORTB &=~(1<<PORTB0);
	}
}

void pwm_init()
{
	// initialize timer0 in PWM mode
	TCCR0A |= (1<<WGM00)|(1<<COM0A1)|(1<<COM0B1)|(1<<WGM01); //set OC0A and OC0B on compare match, clear at bottom, set Fast PWM
	TCCR0B |= (1<<CS02); //set prescalar to 256
	
	//TCCR2A |= (1<<WGM20)|(1<<COM2A1)|(1<<COM2B1)|(1<<WGM21); //set OC0A and OC0B on compare match, clear at bottom, set Fast PWM
	//TCCR2B |= (1<<CS22); //set prescalar to 256
	
	// make sure to make OC0 pin (pin PB3 for atmega32) as output pin
	DDRB |= (1<<DDRB4); // - > OC0B
	DDRB |= (1<<DDRB3); // - > OC0A
	//right motor
	OCR0B = 0;
	//left motor
	OCR0A = 0;
}

uint8_t greenButtonPush (void){
	if((PIND & (1<<PORTD5)) == 0){        	//Verify that the value is the same that what was read
	return 1; }            			//If it is still 0 its because we had a button press
	else{                    			//If the value is different the press is invalid
	return 0; }
}

int main(void)
{
    //initialization
	pwm_init();
    twInit();				//initialize the i2c software/hardware
    lcd_init(LCD_DISP_ON);	//initialize lcd, display on, cursor on
    lcd_clrscr();					//clear screen of lcd
    lcd_home();						//bring cursor to 0,0
	irSensorInit();
    //Setup A2D - Use externa Aref Pin, left adjust (8bit) result, 64 prescaler
    //enable A2D functionality
    ADMUX |= (1<<REFS0)|(1<<ADLAR);
    ADCSRA |= (1<<ADPS1)|(1<<ADPS2)|(1<<ADEN);
    
	lcd_option();
    //DDRB |= (1<<DDRB0);
    //PORTB |= (1<<PORTB0);
	
	//PHASE bits
	//right motor
	DDRB |= (1<<DDRB1);
	//left motor
	DDRB |= (1<<DDRB2);
	
	
	//input pull up, button
	DDRD &= ~(1<<DDRD5);
	PORTD |= (1<<PORTD5);
	
	
	DDRD &= ~(1<<DDRD2);		//Configure PORTD 0 pin  as input
	PORTD |= (1<<PORTD2);		//Activate pull-ups in PORTD pin 2
	DDRD &= ~(1<<DDRD3);		//Configure PORTD 1 pin  as input
	PORTD |= (1<<PORTD3);		//Activate pull-ups in PORTD pin 3
	
	sei(); //enable global interrupts
	EIMSK |= (1<<INT0)|(1<<INT1); //Turn on bits in register for the three different inputs		
	EICRA |= (1<<ISC10)|(1<<ISC11)|(1<<ISC01)|(1<<ISC00);
	
	//voltage check
	voltageCheck();
	_delay_ms(3000);
	lcd_home();
    while (1)
    {	
		//lcd_clrscr();
		//lcd_gotoxy(0, 0);
		//sprintf(buffer, "%u", lCount);
		//lcd_puts(buffer);
		//lcd_gotoxy(0,1);
		//sprintf(buffer, "%u", rCount);
		//lcd_puts(buffer);
		//_delay_ms(10);
		if(greenButtonPush() == 1){
			/*calibrate();
			readGrid();
			cleanUp(readInput);
			*/
			while(1){
				draw(spiral2, 60);
			}
		}
	}
}


/************************************************************************************************************************************************************/
/* Author: Sarika S. Dighe*/
/* Created on 6th May 2018 */
/* Name of Project :- temperature Controllered System */
/* LM35 is connected to ADC0 */
/* This program converts the analog output from an LM35 and convert it to temperature in celsius.*/
/* AIN0 channel is on PE3 pin Temp is sent to UART to be viewed on TeraTerminal */
/* If temperature is below set temperature value controller will turn ON RED LED as a symbol of heater and if temperature is above 
set temperature value controller will turn of BLUE LED as a symbol of cooler.*/
/* Set_temperature value can be increased or decreased with the help of switches connected to PD6 and PF2 points of TM4C123G ARM controller*/
/*********************************************************************************************************************************************************/
#include "TM4C123GH6PM.h"
#include <stdio.h>
#define LCD_DATA GPIOB 
#define LCD_CTRL GPIOA  
#define RS 0x20 /* PORTA BIT5 mask */
#define RW 0x40 /* PORTA BIT6 mask */
#define EN 0x80 /* PORTA BIT7 mask */
#define GPIO_PORTF_CR_R (*((volatile unsigned long *)0x40025524))
void delayUs(int n);
void UART0Tx(char c);
void UART0_init(void);
void UART0_puts(char* s);
void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);

int set_temperature = 23;

int main(void)
{
    uint32_t temperature,value;
	  int ADCval;
    char buffer1[32];
	 /* initialize UART0 and LCD for output */
    UART0_init();
    LCD_init();
    /* enable clocks */
    SYSCTL->RCGCGPIO |= 0x3B;   /* enable clock to GPIOE */
    SYSCTL->RCGCADC |= 1;       /* enable clock to ADC0 */
    GPIOF->LOCK = 0x4C4F434B;   /* unlock commit register */
	  GPIO_PORTF_CR_R = 0x01;    /* make PORTF0 configurable */
    GPIOF->LOCK = 0;            /* lock commit register */
	
   /* initialize PE3 for AIN0 input */
    GPIOE->AFSEL |= 8;          /* enable alternate function */
    GPIOE->DEN &= ~8;           /* disable digital function */
    GPIOE->AMSEL |= 8;          /* enable analog function */
	
    /* initialize ADC0 */
    ADC0->ACTSS &= ~8;        /* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;    /* software trigger conversion */
    ADC0->SSMUX3 = 0;         /* get input from channel 0 */
    ADC0->SSCTL3 |= 6;        /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= 8;         /* enable ADC0 sequencer 3 */
		
/* configure PORTF2 AND PORTD6 for switch input and LED output */
    GPIOD->DIR &= ~0x40;        /* make PORTD6 input for switch */
    GPIOD->DEN |= 0x40;         /* make PORTD6 digital pin */
    GPIOF->DIR &= ~0x11;        /* make PORTF4 input for switch */
    GPIOF->DIR |= 0x0E;         /* make PORTF3, 2, 1 output for LEDs */
    GPIOF->DEN |= 0x1F;         /* make PORTF4-0 digital pins */
    GPIOF->PUR |= 0x11;         /* enable pull up for PORTF4, 0 */
		
    /* configure PORTD6 for falling edge trigger interrupt */
    GPIOD->IS  &= ~0x40;        /* make bit 4, 0 edge sensitive */
    GPIOD->IBE &= ~0x40;        /* trigger is controlled by IEV */
    GPIOD->IEV &= ~0x40;        /* falling edge trigger */
    GPIOD->ICR |= 0x40;         /* clear any prior interrupt */
    GPIOD->IM  |= 0x40;         /* unmask interrupt */
    
	
		 /* configure PORTF4, 0 for falling edge trigger interrupt */
    GPIOF->IS  &= ~0x11;        /* make bit 4, 0 edge sensitive */
    GPIOF->IBE &= ~0x11;        /* trigger is controlled by IEV */
    GPIOF->IEV &= ~0x11;        /* falling edge trigger */
    GPIOF->ICR |= 0x11;         /* clear any prior interrupt */
    GPIOF->IM  |= 0x11;         /* unmask interrupt */
		
		/* enable interrupt in NVIC and set priority to 3 */
    NVIC->IP[30] = 3 << 5;     /* set interrupt priority to 3 */
    NVIC->ISER[0] |= 0x40000000;  /* enable IRQ30 (D30 of ISER[0]) */
		NVIC_EnableIRQ(GPIOD_IRQn);
		NVIC_EnableIRQ(GPIOF_IRQn);
		 __enable_irq(); /* global enable IRQs */
    while(1)
    {
        ADC0->PSSI |= 8;        /* start a conversion sequence 3 */
        while((ADC0->RIS & 8) == 0) ;   /* wait for conversion complete */
        temperature = ADC0->SSFIFO3 * 330 / 4096;
			  value = ((temperature * 9)+160)/5; /* Convert 0C to 0F */
        ADC0->ISC = 8;          /* clear completion flag  */
			  LCD_command(1);       /* clear display */
        LCD_command(0x80);    /* lcd cursor location */
		    sprintf(buffer1, "\r\nADCval = %d Set Point = %dC Temp = %dC Temp = %dF", ADCval, set_temperature, temperature,value);
        UART0_puts(buffer1);
			  if (temperature >= (set_temperature - 2))
				//if (temperature >= 23)
				{
				GPIOF->DATA = 0x04;
			 LCD_data('A');
			 LCD_data('C');
			 LCD_data(' ');
			 LCD_data('O');
			 LCD_data('N');
			}
			else if (temperature <= (set_temperature + 2)){
			//else if (temperature <= 22){
				GPIOF->DATA = 0x02;
				LCD_data('H');
			  LCD_data('E');
				LCD_data('A');
				LCD_data('T');
			  LCD_data(' ');
			 LCD_data('O');
			 LCD_data('N');
			}
			else
			{
					GPIOF->DATA = ~0xFF;
				  LCD_data('O');
			    LCD_data('F');
				  LCD_data('F');
				}
		delayMs(10000);		
    }
	}

void UART0_init(void)
{
    SYSCTL->RCGCUART |= 1;  /* provide clock to UART0 */
    SYSCTL->RCGCGPIO |= 0x01;   /* enable clock to GPIOA */

    /* UART0 initialization */
    UART0->CTL = 0;         /* disable UART0 */
    UART0->IBRD = 104;      /* 16MHz/16=1MHz, 1MHz/104=9600 baud rate */
    UART0->FBRD = 11;       /* fraction part, see Example 4-4 */
    UART0->CC = 0;          /* use system clock */
    UART0->LCRH = 0x60;     /* 8-bit, no parity, 1-stop bit, no FIFO */
    UART0->CTL = 0x301;     /* enable UART0, TXE, RXE */
    
    /* UART0 TX0 and RX0 use PA0 and PA1. Set them up. */
    GPIOA->DEN = 0x03;      /* Make PA0 and PA1 as digital */
    GPIOA->AFSEL = 0x03;    /* Use PA0,PA1 alternate function */
    GPIOA->PCTL = 0x11;     /* configure PA0 and PA1 for UART */
}

void UART0Tx(char c)  
{
    while((UART0->FR & 0x20) != 0); /* wait until Tx buffer not full */
    UART0->DR = c;                  /* before giving it another byte */
}

void UART0_puts(char* s)
{
    while (*s != 0)         /* if not end of string */
        UART0Tx(*s++);      /* send the character through UART0 */
}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int32_t i, j;
    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3180; j++)
            {}  /* do nothing for 1 ms */
}
/* delay n microseconds (16 MHz CPU clock) */
void delayUs(int n)
{
    int i, j;
    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3; j++)
            {}  /* do nothing for 1 us */
}

void GPIOD_Handler(void)
{
    volatile int readback;
    /* increase temp */
    set_temperature = set_temperature + 1;
		delayMs(500);
    GPIOD->ICR |= 0x40; /* clear the interrupt flag before return */
    readback = GPIOD->ICR;     /* a read to force clearing of interrupt flag */
}
void GPIOF_Handler(void)
{
    int i;
    volatile int readback;
    set_temperature = set_temperature - 1;
		delayMs(500);
    
    GPIOF->ICR |= 0x11; /* clear the interrupt flag before return */
    readback = GPIOF->ICR;     /* a read to force clearing of interrupt flag */
}
void LCD_init(void)
{ 
    SYSCTL->RCGCGPIO |= 0x01;  /* enable clock to GPIOA */
    SYSCTL->RCGCGPIO |= 0x02;  /* enable clock to GPIOB */
    
    LCD_CTRL->DIR |= 0xE0;     /* set PORTA pin 7-5 as output for control */
    LCD_CTRL->DEN |= 0xE0;     /* set PORTA pin 7-5 as digital pins */
    LCD_DATA->DIR = 0xFF;      /* set all PORTB pins as output for data */
    LCD_DATA->DEN = 0xFF;      /* set all PORTB pins as digital pins */

    delayMs(20);            /* initialization sequence */
    LCD_command(0x30);
    delayMs(5);
    LCD_command(0x30);
    delayUs(100);
    LCD_command(0x30);
    
    LCD_command(0x38);      /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x06);      /* move cursor right */
    LCD_command(0x01);      /* clear screen, move cursor to home */
    LCD_command(0x0F);      /* turn on display, cursor blinking */
}

void LCD_command(unsigned char command)
{
    LCD_CTRL->DATA = 0;     /* RS = 0, R/W = 0 */
    LCD_DATA->DATA = command;
    LCD_CTRL->DATA = EN;    /* pulse E */
    delayUs(0);
    LCD_CTRL->DATA = 0;
    if (command < 4)
        delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
    else
        delayUs(40);        /* all others 40 us */
}

void LCD_data(unsigned char data)
{
    LCD_CTRL->DATA = RS;    /* RS = 1, R/W = 0 */
    LCD_DATA->DATA = data;
    LCD_CTRL->DATA = EN | RS;   /* pulse E */
    delayUs(0);
    LCD_CTRL->DATA = 0;
    delayUs(40);
}

void SystemInit(void)
{
    /* Grant coprocessor access*/
    /* This is required since TM4C123G has a floating point coprocessor */
    SCB->CPACR |= 0x00f00000;
}
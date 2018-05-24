/*
    File Name:        :  main.c

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/

#include "mcc_generated_files/mcc.h"
#include "xc.h" /* wykrywa rodzaj procka i includuje odpowiedni plik naglowkowy "pic32mm0256gpm048.h"*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h> /*for memset()*/
#include <stdint.h> /*uint8_t etc.*/
#include <sys/attribs.h> /*for Interrupt*/
#include "delay.h"
#include "dogm162.h"

#define LED1_TOG PORTB ^= (1<<_PORTB_RB6_POSITION) /*zmienia stan bitu na przeciwny*/
volatile uint8_t SPI_receive_data ;
volatile uint8_t SPI_receive_flags = 0 ;
volatile uint8_t indeks ;
char   myReadBuffer[15] ; /*bufor odbiorczy*/
char napis[] = "TEST SPI ..";

int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    lcd_Initialize();
    SPI2_Initialize(); /*Slave*/
    TRISBbits.TRISB6 = 0; /*ustawiamy kierunek pinu RB6 jako wyjscie, tu mamy LED jakbysmy potrzebowali*/
    /* When using interrupts, you need to set the Global Interrupt Enable bits
     Use the following macros to:
     Enable the Global Interrupts*/
    INTERRUPT_GlobalEnable();
   
    /* Disable the Global Interrupts
      INTERRUPT_GlobalDisable();*/
      lcd_Locate(1,1);
      lcd_String(napis);

    while (1)
    {
        if(SPI_receive_flags){
            lcd_Locate(2,1);
            lcd_String(myReadBuffer);
         SPI_receive_flags = 0 ;   
        }
    }
}

/*obsluga przerwania odbiorczego SPI2*/
 void __ISR(_SPI2_RX_VECTOR)__SPI2Interrupt(void) {
    /*Read SPI data buffer*/
    SPI_receive_data = SPI2BUF; /*collect the received data*/
    if(SPI_receive_data){ /*if the master has sent something*/
       myReadBuffer[indeks++] = SPI_receive_data ; /*write the data to the buffer in RAM*/  
       SPI_receive_flags = 1; /*set the program flag*/
       /*Uwaga nie ma kontroli przepe?nienia indeksu bufora odbiorczego*/
    } 
    SPI_receive_data = 0; /*clear data*/
    IFS1bits.SPI2RXIF = 0; /*Clear SPI2 Interrupt Flag*/
   }
 



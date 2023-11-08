1] Memory transfer


MOV R0, #30H       ; Move 30H into R0
MOV R1, #40H       ; Move 30H into R0
MOV R3, #05H       ; Set counter as 5
AGAIN: MOV A, @R0  ; Move content of R0 into A
MOV @R1, A
INC R0		   ; Increment R0
INC R1             ; Increment R1
DJNZ R3, AGAIN
END

---------------------------------------------------------------------------------------------------------------------------














































2] LED FLash Program
                     
ORG     0000H
SJMP    0030H		
ORG 0030H 
GO:
MOV P1, #00H
ACALL DELAY
MOV P1, #0FFH
ACALL DELAY
AJMP GO

DELAY:  
MOV     R1,#0F0H      ; delay as per user
MOV     R2,#0F0H
MOV     R3,#10H

D1: 
DJNZ    R1,D1
MOV     R1,#0A0H
DJNZ    R2,D1		 ;decrement and jump if not zero R2
MOV     R2,#0A0H
DJNZ    R3,D1		 ;decrement and jump if not zero R3
RET

END

      



























3] LCD with 8051 
  ; program for lcd initialisation using 8 datalines.
; data lines on port1 & control lines are
; RS-p3.0, R/W- p3.1, EN-p3.2

        
ORG     0000h
LJMP    START           ;Skip Interrupt vector address area

START:
;********** send command to lcd	******************************
MOV    A,#38H	   ; lcd with 2 lines,5x7 matrix      
ACALL  COMMAND	   ; call command subroutine
MOV    A,#0EH	   ; display on, cursor on
ACALL  COMMAND	   ; call command subroutine
MOV    A,#80H  	   ; cursor at line1,pos 0
ACALL  COMMAND	   ;call command subroutine
MOV    A,#01H	   ; clear lcd
ACALL  COMMAND	   ; call aommand subroutine

 ;************ send data to lcd  ******************************                
MOV     A,#'W'			;DISPLY CHAR 
ACALL   DISPLAY			;call display subroutine
MOV     A,#'E'			;DISPLY CHAR   
ACALL   DISPLAY			;call display subroutine
MOV     A,#'L'			;DISPLY CHAR 
ACALL   DISPLAY			;call display subroutine
mov     a,#'C'			;DISPLY CHAR 
ACALL   DISPLAY		    ;call display subroutine
MOV     a,#'O'			;DISPLY CHAR 
ACALL   DISPLAY			;call display subroutine
MOV     a,#'M'			;DISPLY CHAR 
ACALL   DISPLAY			;call display subroutine
MOV     A,#'E'			;DISPLY CHAR 
ACALL   DISPLAY			;call display subroutine
ACALL DELAY	
 ACALL DELAY			 ;call delay
ACALL DELAY	
				
		
sjmp    START

 ;**********command subroutine***************************
command:
ACALL   READY           ;subroutine for command write
MOV     R7,A           ;
MOV     P1,A            ;move command byte to port
CLR     P3.0             ;rs=command
CLR     P3.1             ;rw=write
SETB    P3.2             ;start enable pulse
CLR     P3.2
RET
 ;************* display subroutine*************************			       
display:
ACALL   READY
MOV     P1,A
SETB    P3.0
CLR     P3.1
SETB    P3.2
CLR     P3.2
RET
READY:			       ; ready routine to check
SETB    P1.7	       ; lcd is ready to display the 
CLR     P3.0	       ; next char ; for this using the busy flag of lcd,
SETB    P3.1	       ; i.e. D7 of lcd which is on Po.7 for microcontroller.
WAIT: 
CLR     P3.2
SETB    P3.2
JB      P1.7,WAIT

RET

;************* delay subroutine**************************
DELAY:
 MOV R1,#255
 HERR:MOV R2,#100
HERE: DJNZ R2,HERE
 DJNZ R1,HERR
 RET


END



	
  

































4] Stepper motor
;program for stepper motor thr.port 0 of 89v51RD2,
;lines used p0.0 to p0.3
;motor rotates in forward direction
;to change the direction use "Reverse" look up table

ORG     0000h
SJMP    0030H

ORG     0030H


MOV     P0,#80H    ; initialise port 0
MOV     P2,#80H    ;initialise port 2
ACALL   DELAY        ;delay 

START1:
MOV     DPTR,#REVERSE   ;load forward direction table
MOV     R7,#08H	      ; steps 
NEXT:   CLR     A

MOVC    A,@A+DPTR         ; ini. A for signal.
INC     DPTR
MOV     P2,A  		      ;out stepper signal to p2
MOV     P0,A             ; out stepper signal  to p0
ACALL   DELAY
DEC     R7
CJNE    R7,#00,NEXT         ; keep in step loop
SJMP    START1              ; keep in continous loop

DELAY:
MOV     R1,#0F2H            ; delay as per user
MOV     R2,#0F2H	        ; motor speed is depend on this delay
MOV     R3,#0f8H	        ; if you want slow speed increase delay

D1:
DJNZ    R1,D1
MOV     R1,#010H
DJNZ    R2,D1
MOV     R2,#010H
DJNZ    R3,D1		                                                      
RET

FORWARD: DB 01H,03H,02H,06H,04H,0CH,08H,09H  ; forward direction  table                                                                           
REVERSE: DB 09H,08H,0CH,04H,06H,02H,03H,01H  ;revese direction table
END

   












5] LED, Buzzer, Relay, Pushbuttons with PIC
LED, Pushbutton, Buzzer & Relay Program
/*
Program
*/
#include <p18f4520.h>

#pragma config FOSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF


#define lrbit PORTBbits.RB4   //SW0 interfaced to RB4
#define rlbit PORTBbits.RB5   //SW1
#define buzzer PORTCbits.RC2
#define relay PORTCbits.RC1

void MsDelay (unsigned int time)
{
        unsigned int i, j;
        for (i = 0; i < time; i++)
                for (j = 0; j < 275; j++);/Calibrated for a 1 ms delay in MPLAB/
}

void main()
{
    unsigned char val=0;
    INTCON2bits.RBPU=0;   //To Activate the internal pull on PORTB
        ADCON1 = 0x0F;                //To disable the all analog inputs

        TRISBbits.TRISB4=1;  //To configure RB4 as input
        TRISBbits.TRISB5=1;         //To configure RB5 as input

        TRISCbits.TRISC1=0;   //To configure RC1 (relay)  as output
        TRISCbits.TRISC2=0;          //To cofigure RC2 (buzzer) as output

        TRISD = 0x00;                // To cofigure PORTD (LED) as output

        PORTD = 0x00;                //Initial Value for LED
        buzzer = 0;                        //Initial Value for Buzzer
        relay = 0;                        //Initial Value for Relay

while (1)
        {
        if (!(lrbit)) // if (lrbit == 0)                //To check whether SW0 is pressed
        val = 1;                        // Latch the SWO
        if (!(rlbit))                //To check whether SW1 is pressed
        val = 2;                        // Latch the SW1

        if (val == 1)
        {
                buzzer = 1;
                relay = 1;
                PORTD = PORTD >>1;  //Shift left by 1 bit
                        if (PORTD == 0x00)
                                PORTD = 0x80; // Make the MSB bit eqaul to 1
                MsDelay(250);
        }
        if (val == 2)
        {
                buzzer = 0;
                relay = 0;
                PORTD = PORTD<<1; //Shift right by 1 bit
                         if (PORTD == 0x00)
                                PORTD = 0x01;  // Make the LSB bit eqaul to 1
                MsDelay(250);
        }
        }

}


















































6] Square wave

#include <p18f4550.h>
#include <stdlib.h>

#pragma config FOSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF

void timer_isr(void);


//The program execution comes to this point when a timer interrupt is generated
#pragma code _HIGH_INTERRUPT_VECTOR = 0x0008
void high_ISR (void)
{
	
	_asm 
		goto timer_isr
     _endasm    //The program is relocated to execute the interrupt routine timer_iser

}

// This function is executed as soon as the timer interrupt is generated due to timer overflow
#pragma interrupt timer_isr
void timer_isr(void)
{
	TMR0H = 0X48;                         // Reloading the timer values after overflow
	TMR0L = 0XE5;
	PORTB = ~PORTB;                         //Toggle the PORTB led outputs RB0 - RB3
 	INTCONbits.TMR0IF = 0;	             //Resetting the timer overflow interrupt flag

}

void main()
{	
	ADCON1 = 0x0F;        //Configuring the PORTE pins as digital I/O 
	INTCON2bits.RBPU = 0;
	TRISB = 0;                  //Configruing the LED port pins as outputs
	PORTB = 0xFF;                //Setting the initial value of the LED's after reset	
	T0CON = 0x07;				//Set the timer to 16-bit mode,internal instruction cycle clock,1:256 prescaler
  	TMR0H = 0x48;                // Reset Timer0 to 0x48E5 TO MAKE DELAY OF 1 SECOND
  	TMR0L = 0xE5;
   	INTCONbits.TMR0IF = 0;      // Clear Timer0 overflow flag
	INTCONbits.TMR0IE = 1;		// TMR0 interrupt enabled
 	T0CONbits.TMR0ON = 1;		// Start timer0
	INTCONbits.GIE = 1;			// Global interrupt enabled

	while(1);                      //Program execution stays here untill the timer overflow interrupt is generated
	
}





7] LCD interfacing

#include<p18f4550.h>


#define LCD_DATA PORTD
#define ctrl PORTE
#define rs PORTEbits.RE0
#define rw PORTEbits.RE1
#define en PORTEbits.RE2

void init_LCD(void);
void LCD_command(unsigned char cmd);

void LCD_data(unsigned char data);

void LCD_write_string(static char*str);

void msdelay(unsigned int time);
 
 void main(void)
 {
 
 char var1[]=" Wel-Come";
 char var2="You are in micro";
ADCON1=0x0F;

TRISD=0x00;
TRISE=0x00;

init_LCD();
msdelay(50);

LCD_write_string(var1);
msdelay(15);

LCD_command(0xC0);
LCD_write_string(var2);

while(1);

 }
 
 
 void msdelay(unsigned int time)
 
 {
 unsigned int i,j;
 for(i=0;i<time;i++)
 for(j=0;j<710;j++);
 } 
 
 void init_LCD(void)
 {
 LCD_command(0x38);
 msdelay(15);
 LCD_command(0x01);
 msdelay(15);
 LCD_command(0x0C);
 msdelay(15);
 LCD_command(0x80);
 msdelay(15);
 }
 
 void LCD_command(unsigned char cmd)

 {
 LCD_DATA=cmd;
 rs=0;
 rw=0;
 en=1;
 msdelay(15);
 en=0;
 }
 
 void LCD_data(unsigned char data)
 {
 LCD_DATA=data;
 rs=1;
 rw=0;
 en=1;
 msdelay(15);
 en=0;
 }
 
 void LCD_write_string(static char*str)
 {
 int i=0;
 while(str[i]!=0)
 {
 LCD_data(str[i]);
 msdelay(15);
 i++;
 }
 }

























8] PWM DC motor

#include<p18f4550.h>

#pragma config FOSC = HS
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF

void myMsDelay (unsigned int time)
{
	unsigned int i, j;
	for (i = 0; i < time; i++)
		for (j = 0; j < 275; j++);/Calibrated for a 1 ms delay in MPLAB/
}

void main()
{ 
	TRISCbits.TRISC2 = 0 ;              // Set PORTC, 2 as output
    TRISCbits.TRISC6 = 0 ;
	TRISCbits.TRISC7 = 0 ;
	PR2 = 0x7F;                         // set PWM period to Maximum value 
    CCPR1L = 0x12;                      // Initalise PWM duty cycle to 00 
    CCP1CON = 0x3C;                     // Configure CCP1CON as explained above.
 	T2CON = 0x07;
	PORTCbits.RC6 = 1;
    PORTCbits.RC7 = 0;
  while(1)
	{
		CCPR1L = 0x2F;
		myMsDelay(50);
		CCPR1L = 0x3F;
		myMsDelay(50);
		CCPR1L = 0x4F;
		myMsDelay(50);
		CCPR1L = 0x5F;
		myMsDelay(50);
		CCPR1L = 0x6F;
		myMsDelay(50);
		CCPR1L = 0x7F;
		myMsDelay(50);
 	}   
 
}
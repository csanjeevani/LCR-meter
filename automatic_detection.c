//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <hw_nvic.h>
#include <hw_types.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define NVIC_APINT_R (*((volatile uint32_t *)0xE000ED0C))
#define SYSCTL_RCGCACMP_R (*((volatile uint32_t *)0x400FE63C))



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress()
{
    while(PUSH_BUTTON);
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}



void setTimerMode()
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A)
}

uint8_t cc=0;
uint8_t rr=0;
uint8_t ll=0;

uint64_t taur;
uint64_t tau;
uint64_t taul;
void comparatorIsr()
{

    if(rr==1)
    {taur = WTIMER5_TAV_R;}
    //WTIMER5_TAV_R = 0;
    //taur /= 40;
    if(cc==1)
    {tau = WTIMER5_TAV_R;}

    if(ll==1)
    {taul = WTIMER5_TAV_R;}
    COMP_ACMIS_R = 0x01;//interrupt is handled
}


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A,C,D,E and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    //Comparator Pin PC7
    GPIO_PORTC_DEN_R &= ~0x80;
    GPIO_PORTC_AFSEL_R |= 0x80;
    GPIO_PORTC_AMSEL_R |= 0x80;
    GPIO_PORTC_DIR_R &= ~0x80;
    SYSCTL_RCGCACMP_R |= 0x01;
    COMP_ACREFCTL_R |= COMP_ACREFCTL_VREF_M | COMP_ACREFCTL_EN;

    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_RISE;
    COMP_ACCTL0_R |= COMP_ACCTL0_CINV;
    //COMP_ACCTL0_R       |= (0x02 << 9) | (0x03 << 2);
    COMP_ACRIS_R |= COMP_ACRIS_IN0;
    COMP_ACINTEN_R |0x03;
    NVIC_EN0_R &= ~(1<< INT_COMP0 - 16);

    //Configure the five pins as outputs
    GPIO_PORTD_DIR_R = 0x0C;// MEAS_LR pin PD3 and Integrate pin PD2 set as outputs
    GPIO_PORTE_DIR_R = 0x0E;// MEAS_C pin PE1, HIGHSIDE_R pin PE2 and LOWSIDE_R pin PE3 set as outputs


    //Configure the five pins as Digital pins
    GPIO_PORTD_DEN_R = 0x0C; // MEAS_LR pin PD3 and Integrate pin PD2 set as Data pins
    GPIO_PORTE_DEN_R = 0x0E; // MEAS_C pin PE1, HIGHSIDE_R pin PE2 and LOWSIDE_R pin PE3 set as Data pins

    // set all five pins low
    //GPIO_PORTD_DATA_R &= ~0x0C;
    //GPIO_PORTE_DATA_R &= ~0x0E;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    //Configure AN7(PD0 DUT1) and AN6(PD1 DUT2)
    SYSCTL_RCGCADC_R |= 3;                           // turn on ADC module 0 and 1 clocking
    GPIO_PORTD_AFSEL_R |= 0x03;                      // select alternative functions for AN7 (PD0) and AN6 (PD1)
    GPIO_PORTD_DEN_R &= ~0x03;                       // turn off digital operation on pin PD0 and PD1
    GPIO_PORTD_AMSEL_R |= 0x03;                      // turn on analog operation on pin PD0 and PD1

    //ADC0
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 6;                            // set first sample of ADC0 to AN6
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end on ADC0
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation on ADC0

    //ADC1
    ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 7;                            // set first sample of ADC1 to AN7
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end on ADC1
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation on ADC1

    setTimerMode();
    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module





}

//Functions returning digital volatge values from AN6 and AN7
int16_t readadc0ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}

int16_t readadc1ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
//Blocking function that returns a string
char str[81];
void getstring()
{
    uint8_t max_chars=80;
    uint8_t count;
    char c=0;
    count=0;
    while(c!=13)
    {
        c=getcUart0();
        if(c==8)
        {
            if(count>0)
            {
                count--;
            }
        }
        else if(c==13 || count==max_chars)
        {
            str[count]=0;
            c=13;
        }
        else if(c>=32)
        {
            str[count]=tolower(c);
            count++;
        }
    }

    putsUart0("\r\n");
}
//Function that returns number of arguments, position and type
uint8_t pos[80];
char type[80];
uint8_t argc=0;
void parse_str()
{
    uint8_t count=0;
    uint8_t j=0;
    uint8_t i=1;
    if(((str[0]>=48)&&(str[0]<=57))||((str[0]>=65)&&(str[0]<=90))||((str[0]>=97)&&(str[0]<=122)))
                     {
                          count++;
                          argc=count;
                          pos[j]=0;
                          j++;
                          if((str[0]>=48)&&(str[0]<=57))
                              {
                                  type[j-1]=110; //ascii value for 'n'
                              }
                          else if(((str[0]>=65)&&(str[0]<=90))||((str[0]>=97)&&(str[0]<=122)))
                              {
                                  type[j-1]=97; //ascii value for 'a'

                              }
                      }
    for(i=1; i<=strlen(str);i++)
    {
        if((((str[i]>=48)&&(str[i]<=57))||((str[i]>=65)&&(str[i]<=90))||((str[i]>=97)&&(str[i]<=122))))
        {
            if(!(((str[i-1]>=48)&&(str[i-1]<=57))||((str[i-1]>=65)&&(str[i-1]<=90))||((str[i-1]>=97)&&(str[i-1]<=122))))
                {
                    count++;
                    argc=count;
                    pos[j]=i;
                    j++;
                    if((str[i]>=48)&&(str[i]<=57))
                        {
                            type[j-1]=110; //ascii value for 'n'
                        }
                    else if(((str[i]>=65)&&(str[i]<=90))||((str[i]>=97)&&(str[i]<=122)))
                        {
                            type[j-1]=97; //ascii value for 'a'
                        }
                }

         }


    }

}

//Function that stores values of the two arguments after the set command
uint8_t arg0value[80];
uint8_t arg1value[80];
uint8_t arg2value[80];
void getvalue()
{
    uint8_t value0=pos[0];
    uint8_t value1=pos[1];
    uint8_t value2=pos[2];
    uint8_t p=0;
    uint8_t k=0;
    uint8_t j=0;

    while((((str[value0]>=48)&&(str[value0]<=57))||((str[value0]>=65)&&(str[value0]<=90))||((str[value0]>=97)&&(str[value0]<=122))))
    {
        arg0value[p]=str[value0];
        p++;
        value0++;
    }
    while((((str[value1]>=48)&&(str[value1]<=57))||((str[value1]>=65)&&(str[value1]<=90))||((str[value1]>=97)&&(str[value1]<=122))))
    {
        arg1value[k]=str[value1];
        k++;
        value1++;
    }
    while((((str[value2]>=48)&&(str[value2]<=57))||((str[value2]>=65)&&(str[value2]<=90))||((str[value2]>=97)&&(str[value2]<=122))))
    {
        arg2value[j]=str[value2];
        j++;
        value2++;
    }
}

//Function that returns true if the command is set and there are aleast two arguments after that
bool iscommand(char* verb, int args)
{
    uint8_t i;

    if((args>=2))
    {
    for (i=0; i<sizeof(verb);i++)
        {
            if((verb[i]==arg0value[i]))
                return true;
            else return false;
        }
    }
    return false;
}

//function that sets a particular pin high on receiving the command

void setpins()
{

    uint8_t j;

    char string_pin[80];
    char string_power[80];
    char string_set[80];

    for(j=0;j<sizeof(arg0value);j++)
        {
            string_set[j]= arg0value[j];
        }
    for(j=0;j<sizeof(arg1value);j++)
    {
        string_pin[j]= arg1value[j];
    }

    for(j=0;j<sizeof(arg1value);j++)
    {
        string_power[j]= arg2value[j];
    }

    if(strcmp("set",string_set)==0)
    {
    if(strcmp("measurelr",string_pin)==0)
     {
       if(strcmp("on",string_power)==0)
         {
           GPIO_PORTD_DATA_R |= 0x08;
           GPIO_PORTF_DATA_R |= 0x2;//red led on
          }
       if(strcmp("off",string_power)==0)
          {
           GPIO_PORTD_DATA_R &= ~0x08;
           GPIO_PORTF_DATA_R |= 0x8;//green on
          }
     }

    if(strcmp("measurec",string_pin)==0)
     {
       if(strcmp("on",string_power)==0)
         {
           GPIO_PORTE_DATA_R |= 0x02;
           GPIO_PORTF_DATA_R |= 0x2;//red led on
          }
       if(strcmp("off",string_power)==0)
          {
           GPIO_PORTE_DATA_R &= ~0x02;
           GPIO_PORTF_DATA_R |= 0x8;//green on
          }
     }

    if(strcmp("integrate",string_pin)==0)
     {
       if(strcmp("on",string_power)==0)
         {
           GPIO_PORTD_DATA_R |= 0x04;
           GPIO_PORTF_DATA_R |= 0x2;//red led on
          }
       if(strcmp("off",string_power)==0)
          {
           GPIO_PORTD_DATA_R &= ~0x04;
           GPIO_PORTF_DATA_R |= 0x8;//green on
          }
     }

    if(strcmp("highsider",string_pin)==0)
     {
       if(strcmp("on",string_power)==0)
         {
           GPIO_PORTE_DATA_R |= 0x04;
           GPIO_PORTF_DATA_R |= 0x2;//red led on
          }
       if(strcmp("off",string_power)==0)
          {
           GPIO_PORTE_DATA_R &= ~0x04;
           GPIO_PORTF_DATA_R |= 0x8;//green on
          }
     }

    if(strcmp("lowsider",string_pin)==0)
     {
       if(strcmp("on",string_power)==0)
         {
           GPIO_PORTE_DATA_R |= 0x08;
           GPIO_PORTF_DATA_R |= 0x2;//red led on
          }
       if(strcmp("off",string_power)==0)
          {
           GPIO_PORTE_DATA_R &= ~0x08;
           GPIO_PORTF_DATA_R |= 0x8;//green on
          }
     }
    if(strcmp("integratels",string_pin)==0)
         {
           if(strcmp("on",string_power)==0)
             {
               GPIO_PORTD_DATA_R |= 0x04;
               GPIO_PORTE_DATA_R |= 0x08;
               GPIO_PORTF_DATA_R |= 0x2;//red led on
              }
           if(strcmp("off",string_power)==0)
              {
               GPIO_PORTD_DATA_R &= ~0x04;
               GPIO_PORTE_DATA_R &= ~0x08;
               GPIO_PORTF_DATA_R |= 0x8;//green on
              }
         }
    if(strcmp("integratehs",string_pin)==0)
         {
           if(strcmp("on",string_power)==0)
             {
               GPIO_PORTD_DATA_R |= 0x04;
               GPIO_PORTE_DATA_R |= 0x04;
               GPIO_PORTF_DATA_R |= 0x2;//red led on
              }
           if(strcmp("off",string_power)==0)
              {
               GPIO_PORTD_DATA_R &= ~0x04;
               GPIO_PORTE_DATA_R &= ~0x04;
               GPIO_PORTF_DATA_R |= 0x8;//green on
              }
         }


    }

}

//reset command
int reset()
{

    HWREG(NVIC_APINT)= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ;
    return 0;


}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

char vr[200];
char vc[200];
char vl[200];
float vdut2;
float vdut1;
float vdelta;

float vdiff()
{
    vdut1=readadc1ss3();
    vdut2=readadc0ss3();
    vdelta=((vdut2-vdut1)/4096.0)*3.3;
    return vdelta;
}

float voltager;
double resistor_val;
void resistance()
{

       char r[80];
       //Deintegrate
       GPIO_PORTE_DATA_R &= ~0x02;//measurec off
       GPIO_PORTE_DATA_R |= 0x08;//lowside on
       GPIO_PORTD_DATA_R &= ~0x08;//measurelr off
       GPIO_PORTE_DATA_R &= ~0x02;//measurec off
       GPIO_PORTD_DATA_R |= 0x04;//integrate on
       GPIO_PORTE_DATA_R &= ~0x04;//highsider off
       waitMicrosecond(10000000);
       //Integrate
       WTIMER5_TAV_R = 0;
       //COMP_ACINTEN_R = 0x03;//comparator interrupt on
       GPIO_PORTD_DATA_R |= 0x08;//measure lr on
       //setTimerMode();
       GPIO_PORTE_DATA_R &= ~0x08;//lowsider off
       WTIMER5_TAV_R = 0;
       WTIMER5_CTL_R |= TIMER_CTL_TAEN;// turn-on counter
       COMP_ACINTEN_R = 0x03;//comparator interrupt on
       NVIC_EN0_R |= (1<< INT_COMP0 - 16);
       waitMicrosecond(10000000);
       NVIC_EN0_R &= ~(1<< INT_COMP0 - 16);

       voltager=vdiff();
       sprintf(vr,"%0.7f",voltager);
       putsUart0("vdelta = ");
       putsUart0(vr);
       putsUart0("\r\n");

       putsUart0("res value is: ");

       //comparatorIsr();
       //resistor_val=((taur/71.21)+(3000000/71.21))/2;
       if(taur>100000000)
       {
           resistor_val=((2.0*taur)/187688853.0)+(155840308.0)/187688853.0;
           sprintf(r,"%f",resistor_val);
           putsUart0(r);
           putsUart0(" Mohm");
       }
       if((taur<100000000)&&(taur>5000000))
       {
           resistor_val=((200.0*taur)/10817803.0)+(21529300.0/10817803.0);
           sprintf(r,"%0.7f",resistor_val);
           putsUart0(r);
           putsUart0(" Kohm");
       }
       if((taur<5000000)&&(taur>1000000))
       {
           resistor_val=((5.0*taur)/262976.0)+(154525.0/262976.0);
           sprintf(r,"%fl",resistor_val);
           putsUart0(r);
           putsUart0(" Kohm");
       }
       if((taur<1000000)&&(taur>50000))
       {
           resistor_val=((taur)/51272.0)-(463.0/51272.0);
           sprintf(r,"%f",resistor_val);
           putsUart0(r);
           putsUart0(" Kohm");
       }
       if((taur<50000)&&(taur>500))
       {
           resistor_val=((396.0*taur)/19585.0)-(476852.0/19585.0);
           sprintf(r,"%fl",resistor_val);
           putsUart0(r);
           putsUart0(" ohm");
       }
}

float voltagec;
float cap_val;
void cap()
{

        char c[80];
       //Deintegrate
        GPIO_PORTD_DATA_R &= ~0x08;//measurelr off
       GPIO_PORTE_DATA_R |= 0x08;//lowside on
       GPIO_PORTD_DATA_R &= ~0x08;//measurelr off
       GPIO_PORTE_DATA_R |= 0x02;//measurec on
       GPIO_PORTD_DATA_R &= ~0x04;//integrate off
       GPIO_PORTE_DATA_R &= ~0x04;//highsider off
       waitMicrosecond(10000000);
       //Integrate
       //setTimerMode();
       GPIO_PORTE_DATA_R &= ~0x08;//lowsider off
       GPIO_PORTE_DATA_R |= 0x04;//highsider on
       WTIMER5_TAV_R = 0;
       WTIMER5_CTL_R |= TIMER_CTL_TAEN;// turn-on counter
       COMP_ACINTEN_R = 0x03;//comparator interrupt on
       NVIC_EN0_R |= (1<< INT_COMP0 - 16);
       waitMicrosecond(10000000);
       NVIC_EN0_R &= ~(1<< INT_COMP0 - 16);

       voltagec=vdiff();
       sprintf(vc,"%0.7f",voltagec);
       putsUart0("vdelta = ");
       putsUart0(vc);
       putsUart0("\r\n");
       putsUart0("cap value is: ");

       if(tau>5000000)
       {
           cap_val=((tau*4.0)/22966731.0)+(3576134.0/22966731.0);
           sprintf(c,"%0.7f",cap_val);
           putsUart0(c);
           putsUart0(" uF");
       }
       if((tau<5000000)&&(tau>500000))
       {
           cap_val=(tau/(6000000.0))-(33940.0/6000000.0);
           sprintf(c,"%0.7f",cap_val);
           putsUart0(c);
           putsUart0(" uF");
       }
       if((tau<500000)&&(tau>50000))
       {
           cap_val=(tau/(5000000.0))-(25533.0/5000000.0);
           sprintf(c,"%0.7f",cap_val);
           putsUart0(c);
           putsUart0(" uF");
       }
       if((tau<50000)&&(tau>5000))
       {
           cap_val=(tau/(5405.0))-(6575.0/5405.0);
           sprintf(c,"%0.7f",cap_val);
           putsUart0(c);
           putsUart0(" nF");
       }
}

float vdut1l;
float vdut2l;
float voltagel;
float esr;
float inductance_val;
void inductance()
{

       char rl[80];
       char i[80];
       //Deintegrate
       GPIO_PORTE_DATA_R |= 0x08;//lowside on
       GPIO_PORTD_DATA_R &= ~0x08;//measurelr off
       GPIO_PORTE_DATA_R &= ~0x02;//measurec off
       GPIO_PORTD_DATA_R |= 0x04;//integrate on
       GPIO_PORTE_DATA_R &= ~0x04;//highsider off
       waitMicrosecond(100000);
       GPIO_PORTD_DATA_R &= ~0x04;//integrate off
       GPIO_PORTE_DATA_R |= 0x08;//lowside on
       GPIO_PORTD_DATA_R |= 0x08;//measurelr on
       GPIO_PORTE_DATA_R &= ~0x02;//measurec off
       GPIO_PORTE_DATA_R &= ~0x04;//highsider off

       WTIMER5_TAV_R = 0;
       WTIMER5_CTL_R |= TIMER_CTL_TAEN;// turn-on counter
       COMP_ACINTEN_R = 0x03;//comparator interrupt on
       NVIC_EN0_R |= (1<< INT_COMP0 - 16);
       waitMicrosecond(500000);
       NVIC_EN0_R &= ~(1<< INT_COMP0 - 16);

       voltagel=vdiff();
       vdut1l=vdut1;
       vdut2l=vdut2;
       sprintf(vl,"%0.7f",voltagel);
       putsUart0("vdelta = ");
       putsUart0(vl);
       putsUart0("\r\n");
       putsUart0("inductance value is\r\n ");
       esr=33.0*(abs(vdut2-vdut1))/vdut2;

       if((taul>2000000))
       {
           inductance_val=3/1.0012;
           sprintf(i,"%0.7f",inductance_val);
           putsUart0(i);
           putsUart0(" mH");
           putsUart0("\r\n");
       }
       if((taul<700)&&(taul>150))
       {
           inductance_val=taul/1.137;
           sprintf(i,"%0.7f",inductance_val);
           putsUart0(i);
           putsUart0(" uH");
           putsUart0("\r\n");
       }
       if((taul<150)&&(taul>50))
       {
           inductance_val=taul/0.89;
           sprintf(i,"%0.7f",inductance_val);
           putsUart0(i);
           putsUart0(" uH");
           putsUart0("\r\n");
       }
       if((taul<50))
       {
           inductance_val=taul/0.79;
           sprintf(i,"%0.7f",inductance_val);
           putsUart0(i);
           putsUart0(" uH");
           putsUart0("\r\n");
       }

       putsUart0("\r\n esr is: ");
       sprintf(rl,"%f",esr);
       putsUart0(rl);
       putsUart0("\r\n");
}

void auto_detect()
{
    putsUart0("\r\n testing if test sample is resistance \r\n");
    rr=1;
    resistance();
    rr=0;
    putsUart0("\r\n testing if test sample is capacitance \r\n");
    cc=1;
    cap();
    cc=0;
    putsUart0("\r\n testing if test sample is inductance \r\n");
    ll=1;
    inductance();
    ll=0;
    if((voltager<0.001)&&(voltagec>0.001))
    {
        putsUart0("the test sample is resistance \r\n");
        resistance();
    }
    if((voltager<1.5)&&(voltagec>2.5))
    {
        putsUart0("the test sample is capacitance \r\n");
        cap();
    }
    if(((vdut1l<3500)&&(vdut1l>2800))&&((vdut2l<3500)&&(vdut2l>2800)))
    {
        putsUart0("the test sample is inductance \r\n");
        inductance();
    }



}

//MAIN

int main(void)
{

    // Initialize hardware
    initHw();

    RED_LED = 1;
    waitMicrosecond(500000);
    RED_LED = 0;
    waitMicrosecond(500000);

    char stringin[80];
    uint8_t i=0;
    for(i=0;i<80;i++)
    {
        stringin[i]=0;
    }
    while(1)
    {
        putsUart0("Enter your String:\r\n");
        getstring();
        parse_str();
        putsUart0("you entered: ");
        putsUart0(str);
        putsUart0("\r\n");


      for(i=0;i<strlen(str);i++)
      {
          stringin[i]=str[i];
      }

      if(strcmp("capacitance",stringin)==0)
      {
          cc=1;
          cap();
      }
      putsUart0("\r\n");
      if(strcmp("resistance",stringin)==0)
      {
          rr=1;
          resistance();
      }
      putsUart0("\r\n");
      if(strcmp("inductance",stringin)==0)
      {
          ll=1;
          inductance();
      }
      putsUart0("\r\n");

      putsUart0("\r\n");
      if(strcmp("reset",stringin)==0)
      {
          reset();
      }
      putsUart0("\r\n");
      if(strcmp("auto",stringin)==0)
      {
          auto_detect();
      }
      putsUart0("\r\n");

    }

}

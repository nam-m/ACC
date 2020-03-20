//Motor Control for ACC in RC Car
//Author: Nam Anh Mai

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "UART.h"
#include "tm4c123gh6pm.h"

#define PWM_FREQUENCY 55

void UARTIntHandler(void);
char UART_InChar(void);
void UART_OutChar(char data);
uint32_t UART_InUDec(void);
void UART_OutUDec(uint32_t n);
void UART_Init(void);
void printString(char * string);

int main(void)
{
    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;
    volatile uint32_t ui32Adjust;
    volatile double duty_cycle;
    volatile double pulse_width;

    //Calculation to produce pulse width of 1.5ms
//    ui32Adjust = 83;
    pulse_width = (1/PWM_FREQUENCY)/1000 * ui32Adjust;
    duty_cycle = (1000-ui32Adjust)/10;

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64); //run PWM clock at 40Mhz/64 = 625kHz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Enable PWM output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Enable GPIOD
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Enable GPIOF

    //Enable UART peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //Set Rx/Tx pins as UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX); //PA0 for Tx
    GPIOPinConfigure(GPIO_PA1_U0TX); //PA1 for Rx
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Configure PD0 as PWM output pin for Module 1 PWM Generator 0 (p.1233)
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    //First 3 lines unlock GPIO commit control register of PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    //Configure PF0 and PF4 as input
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    //Configure internal pull-up resistors for PF0 & PF4
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Get PWM clock
    ui32PWMClock = SysCtlClockGet() / 64;
    //Determine count to be loaded into Load register (count down to 0 so minus 1)
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000); //Set pulse width (min resolution = PWM load value/1000)
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    //Get the current config of UART
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART0); //enable UART interrupt
    // select receiver interrupts (RX) and receiver timeout interrupts (RT)
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);


    while(1)
    {
//        UART_Init();
//        PortF_Init();
        // ui32Adjust: 83 -> 1.5ms, 111 -> 2ms, 56 -> 1ms
        printString("Enter motor position:\n\r");
        ui32Adjust = UART_InUDec();
        printString("\n\r");
        if ((ui32Adjust > 111) || (ui32Adjust < 83))
            printString("Input needs to be larger than 83 and smaller than 111\n\r");
        else
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);
        SysCtlDelay(1);
    }
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
    UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
    while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
    {
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
        //echo character
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
        SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
    }
}

char UART_InChar(void)
{
  while((UART0_FR_R&UART_FR_RXFE) != 0);
  return((char)(UART0_DR_R&0xFF));
}

void UART_OutChar(char data)
{
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}

uint32_t UART_InUDec(void)
{
    uint32_t number=0, length=0;
    char character;
    character = UART_InChar();
    while(character != CR){ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
    if((character>='0') && (character<='9'))
    {
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      UART_OutChar(character);
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length)
    {
      number /= 10;
      length--;
      UART_OutChar(character);
    }
    character = UART_InChar();
  }
  return number;
}

void UART_OutUDec(uint32_t n)
{
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
    if(n >= 10){
        UART_OutUDec(n/10);
        n = n%10;
    }
    UART_OutChar(n+'0'); /* n is between 0 and 9 */
}

void UART_Init(void)
{
  SYSCTL_RCGCUART_R |= 0x01;            // activate UART0
  SYSCTL_RCGCGPIO_R |= 0x01;            // activate port A
  while((SYSCTL_PRGPIO_R&0x01) == 0){};
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 104;                    // IBRD = int(50,000,000 / (16 * 115,200)) = int(27.1267)
  UART0_FBRD_R = 11;                     // FBRD = int(0.1267 * 64 + 0.5) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= 0x301;                 // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}

void printString(char * string)
{
    while(*string){
        UART_OutChar(*(string++));
    }
}

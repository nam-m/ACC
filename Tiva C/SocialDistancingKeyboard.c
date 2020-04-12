#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stdlib.h"
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"

#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"

#include "utils/uartstdio.c"
#include "UART.h"
#include "tm4c123gh6pm.h"

//PID***********************
int error_prior = 0;
int integral_prior = 0;
int output_prior = 0;
int output = 100;
int error = 0;
int derivative = 0;
int integral = 0;

//PID Tuning
const float kp = 850;
const float ki = 10;
const float kd = 25;

//RC Car Parameters
const int bias = 100; //0: reverse full speed, 100: 0 speed, 200: forward full speed
int maxspeed = 50;
int min_distance = 30;
volatile uint32_t distance = 0;
//*************************

//Ultrasonic*****************************************************************
void inputInt();
void Captureinit();
void InitConsole(void);

const double temp = 1.0 / 80.0;
volatile uint32_t pulse = 0; //actual distance from ultrasonic
volatile uint8_t echowait = 0; //waiting time for ping
//**************************************************************************

//ServoFunctions************************************************************
volatile uint32_t ServoCount = 0;

uint32_t ServoBase[6];
uint32_t ServoPin[6];
uint32_t ServoPos[6];
uint32_t ServoPosTemp[6];
uint32_t ServoNumber = 0;

char UART_InChar(void);
void UART_OutChar(char data);
uint32_t UART_InUDec(void);

void ServoInterrupt()
{
    uint32_t status = 0;

    status = TimerIntStatus(TIMER5_BASE, true);
    TimerIntClear(TIMER5_BASE, status);
    /*
     *    This increments the counter
     *    The counter works like a timer in PWM mode in count up mode.
     *    The value 4000 sets the wave period.
     *    Then we have the "match" values that set the positive pulse width,
     * those are the values saved in the array ServoPos. The outputs start as HIGH and go to LOW
     * when the match value is reached
     */
    ServoCount++;
    uint32_t i;
    if (ServoCount > 4000)
        ServoCount = 0;
    for (i = 0; i < ServoNumber; i++)
    {
        ServoPos[i] = ServoPosTemp[i];
    }

    for (i = 0; i < ServoNumber; i++)
    {
        if (ServoCount > ServoPos[i])
            GPIOPinWrite(ServoBase[i], ServoPin[i], 0);
        else
            GPIOPinWrite(ServoBase[i], ServoPin[i], ServoPin[i]);
    }
}

int32_t ServoWrite(uint32_t value, uint32_t pos)
{
    if (value > 200)
        return -1;
    value = value + 199;
    ServoPosTemp[pos] = value;
    return 0;
}

int32_t ServoAttach(uint32_t peripheral, uint32_t base, uint32_t pin)
{
    if (ServoNumber < 6)
    {
        SysCtlPeripheralEnable(peripheral);
        SysCtlDelay(3);
        GPIOPinTypeGPIOOutput(base, pin);
        ServoBase[ServoNumber] = base;
        ServoPin[ServoNumber] = pin;
        ServoNumber++;
    }
    else
        return -1;

    return 0;
}
//**********************************************************************************************
/*
 Timer setup
 */
void TimerBegin()
{
    uint32_t Period;
    Period = 400;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    SysCtlDelay(3);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER5_BASE, TIMER_A, Period - 1);
    TimerIntRegister(TIMER5_BASE, TIMER_A, ServoInterrupt);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER5_BASE, TIMER_A);
}

int main()
{
    SysCtlClockSet(
    SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
    InitConsole();
    Captureinit();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTA_BASE, inputInt);
    ServoAttach(SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_0);
    TimerBegin();

//    int i = 0;
//    for (i = 1; i < 30; i++)
//    {
//        UARTprintf("\n\r");
//
//    }

    UARTprintf("Welcome to ACC Controller \n");
    UARTprintf("Set minimum distance:\n\r");
    min_distance = UART_InUDec();
    UARTprintf("\n\r");
    UARTprintf("Set cruise speed:\n\r");
    maxspeed = UART_InUDec();
    UARTprintf("\n\r");

    SysCtlDelay(10000000);

    while (1)
    {
        if (echowait != 1)
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
            SysCtlDelay(266);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
            while (echowait != 0)
            {
                //do nothing
            }
            pulse = (uint32_t) (temp * pulse);
            pulse = pulse / 58; //distance[cm]
        }

        error = (min_distance - pulse) * 1000;
        integral = integral_prior + error * 0.01;
        derivative = (error - error_prior) / 0.01;
        output = (kp * error + ki * integral + kd * derivative) / 1000000;
        error_prior = error;
        integral_prior = integral;

        if (output > maxspeed)
            output = bias - maxspeed;
        else if (output < -maxspeed)
            output = bias + maxspeed;
        else
            output = bias - output;

        UARTprintf("distance = %2dcm ", pulse);
        UARTprintf("error = %d ", error);
        UARTprintf("integral = %d ", integral);
        UARTprintf("derivative = %d ", derivative);
        UARTprintf("output = %d \n", output);

        ServoWrite(output, 0);

        SysCtlDelay(200000);
    }
}

//Ultrasonic functions
void inputInt()
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
    if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2)
    {
        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
        TimerEnable(TIMER2_BASE, TIMER_A);
        echowait = 1;
    }
    else
    {
        pulse = TimerValueGet(TIMER2_BASE, TIMER_A);
        TimerDisable(TIMER2_BASE, TIMER_A);
        echowait = 0;
    }
}

void Captureinit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlDelay(3);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE, TIMER_A);
}

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, 16000000); //baud rate 9600
}

char UART_InChar(void)
{
    while ((UART0_FR_R & UART_FR_RXFE) != 0);
    return ((char) (UART0_DR_R & 0xFF));
}

void UART_OutChar(char data)
{
    while ((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = data;
}

uint32_t UART_InUDec(void)
{
    uint32_t number = 0, length = 0;
    char character;
    character = UART_InChar();
    while (character != CR)
    { // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.
// If the character is not 0-9, it is ignored and not echoed
        if ((character >= '0') && (character <= '9'))
        {
            number = 10 * number + (character - '0'); // this line overflows if above 4294967295
            length++;
            UART_OutChar(character);
        }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
        else if ((character == BS) && length)
        {
            number /= 10;
            length--;
            UART_OutChar(character);
        }
        character = UART_InChar();
    }
    return number;
}

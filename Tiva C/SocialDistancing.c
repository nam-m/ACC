//#include <stdint.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include "stdlib.h"
//#include <string.h>
//
//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_timer.h"
//#include "inc/hw_uart.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_pwm.h"
//#include "inc/hw_types.h"
//
//#include "driverlib/pin_map.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/debug.h"
//#include "driverlib/timer.h"
//#include "driverlib/gpio.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/uart.h"
//#include "driverlib/udma.h"
//#include "driverlib/pwm.h"
//#include "driverlib/ssi.h"
//#include "driverlib/systick.h"
//
//#include "utils/uartstdio.c"
//
////PID***********************
//int error_prior = 0;
//int integral_prior = 0;
//int output = 100;
//const float kp = 850;
//const float ki = 3;
//const float kd = 25;
//const int bias = 100;
//const int maxspeed = 25;
//int min_distance = 30;
////**************************
//
////Ultrasonic*****************************************************************
//void inputInt();
//void Captureinit();
//void InitConsole(void);
//
//const double temp = 1.0 / 80.0;
//volatile uint32_t pulse = 0;
//volatile uint8_t echowait = 0;
////**************************************************************************
//
////ServoFunctions************************************************************
//volatile uint32_t ServoCount = 0;
//uint32_t ServoBase[6];
//uint32_t ServoPin[6];
//uint32_t ServoPos[6];
//uint32_t ServoPosTemp[6];
//uint32_t ServoNumber = 0;
//
//void ServoInterrupt()
//{
//    uint32_t status = 0;
//
//    status = TimerIntStatus(TIMER5_BASE, true);
//    TimerIntClear(TIMER5_BASE, status);
//    ServoCount++;
//    uint32_t i;
//    if (ServoCount > 4000)
//        ServoCount = 0;
//    for (i = 0; i < ServoNumber; i++)
//    {
//        ServoPos[i] = ServoPosTemp[i];
//    }
//
//    for (i = 0; i < ServoNumber; i++)
//    {
//        if (ServoCount > ServoPos[i])
//            GPIOPinWrite(ServoBase[i], ServoPin[i], 0);
//        else
//            GPIOPinWrite(ServoBase[i], ServoPin[i], ServoPin[i]);
//    }
//}
//
//int32_t ServoWrite(uint32_t value, uint32_t pos)
//{
//    if (value > 200)
//        return -1;
//    value = value + 199;
//    ServoPosTemp[pos] = value;
//    return 0;
//}
//
//int32_t ServoAttach(uint32_t peripheral, uint32_t base, uint32_t pin)
//{
//    if (ServoNumber < 6)
//    {
//        SysCtlPeripheralEnable(peripheral);
//        SysCtlDelay(3);
//        GPIOPinTypeGPIOOutput(base, pin);
//        ServoBase[ServoNumber] = base;
//        ServoPin[ServoNumber] = pin;
//        ServoNumber++;
//    }
//    else
//        return -1;
//
//    return 0;
//}
////**********************************************************************************************
///*
// Timer setup
// */
//void TimerBegin()
//{
//    uint32_t Period;
//    Period = 400;
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
//    SysCtlDelay(3);
//    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
//    TimerLoadSet(TIMER5_BASE, TIMER_A, Period - 1);
//    TimerIntRegister(TIMER5_BASE, TIMER_A, ServoInterrupt);
//    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
//    TimerEnable(TIMER5_BASE, TIMER_A);
//}
//
//int main()
//{
//    SysCtlClockSet(
//    SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
//    InitConsole();
//    Captureinit();
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    SysCtlDelay(3);
//    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); //PA3
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    SysCtlDelay(3);
//    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2); //PA2
//    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
//    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
//    GPIOIntRegister(GPIO_PORTA_BASE, inputInt);
//    ServoAttach(SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_0);
//    TimerBegin();
//
//    while (1)
//    {
//        if (echowait != 1)
//        {
//            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
//            SysCtlDelay(266);
//            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
//            while (echowait != 0)
//            {
//                //do nothing
//            }
//            pulse = (uint32_t) (temp * pulse);
//            pulse = pulse / 58;
//        }
//
//        int error = (min_distance - pulse) * 1000;
//        int integral = integral_prior + error * 0.01;
//        int derivative = (error - error_prior) / 0.01;
//        output = (kp * error + ki * integral + kd * derivative) / 1000000;
//        error_prior = error;
//        integral_prior = integral;
//
//        if (output > maxspeed)
//        {
//            output = bias - maxspeed;
//        }
//        else if (output < -maxspeed)
//        {
//            output = bias + maxspeed;
//        }
//        else
//        {
//            output = bias - output;
//        }
//
//        UARTprintf("distance = %2dcm ", pulse);
//        UARTprintf("error = %d ", error);
//        UARTprintf("integral = %d ", integral);
//        UARTprintf("derivative = %d ", derivative);
//        UARTprintf("output = %d \n", output);
//
//        ServoWrite(output, 0);
//
//        SysCtlDelay(200000);
//    }
//}
//
//void inputInt()
//{
//    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
//    if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2)
//    {
//        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
//        TimerEnable(TIMER2_BASE, TIMER_A);
//        echowait = 1;
//    }
//    else
//    {
//        pulse = TimerValueGet(TIMER2_BASE, TIMER_A);
//        TimerDisable(TIMER2_BASE, TIMER_A);
//        echowait = 0;
//    }
//}
//
//void Captureinit()
//{
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
//    SysCtlDelay(3);
//    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
//    TimerEnable(TIMER2_BASE, TIMER_A);
//}
//
//void InitConsole(void)
//{
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    SysCtlDelay(3);
//    GPIOPinConfigure(GPIO_PA0_U0RX);
//    GPIOPinConfigure(GPIO_PA1_U0TX);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
//    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//    UARTStdioConfig(0, 9600, 16000000); //baud rate 9600
//}

#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <math.h>
#include <msp430.h>
/*
 * This project contains some code samples that may be useful.
 */

//set global threshold
unsigned int rear_largest = 22;
unsigned int rear_middle = 15;
unsigned int rear_smallest = 8;
unsigned int front_largest = 20;
unsigned int front_smallest = 10;

unsigned int front_time = 0;
unsigned int rear_time = 0;

int FRONT = 1;
int sensorRead = 0;
#define setupPause 100;
Timer_A_initUpDownModeParam state_toggler; //timer_trigs
Timer_A_initContinuousModeParam  toggler; //timer_params
Timer_A_initCompareModeParam toggler_isr_regs; //initCompParam


Timer_A_initContinuousModeParam echo_timer; //timer_params

void Init_ULTRASONIC(){
    // set echo_in with pull down resistors
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7); //front
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5); //back
}

unsigned int readSensor_front(){
    Timer_A_clear(TIMER_A1_BASE);
    //1. send trigger
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN7); // sending trigger to front sensor
    __delay_cycles(5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7); // sending trigger to front sensor
    //2. wait for echo front to start reading
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7) == 0){};
    //3. start the timer
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    //4. wait for echo to finish
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7)){};
    //5. stop timer
    Timer_A_stop(TIMER_A1_BASE);
    
    unsigned int t = Timer_A_getCounterValue(TIMER_A1_BASE)/58;
    front_time = 0.4*front_time+0.6*t;
    return front_time;
}
unsigned int readSensor_rear(){
    Timer_A_clear(TIMER_A1_BASE);
    //1. send trigger
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN7); // sending trigger to front sensor
    __delay_cycles(5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7); // sending trigger to front sensor
    //2. wait for echo front to start reading
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == 0){};
    //3. start the timer
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    //4. wait for echo to finish
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5)){};
    //5. stop timer
    Timer_A_stop(TIMER_A1_BASE);

    unsigned int t = Timer_A_getCounterValue(TIMER_A1_BASE)/58;
    rear_time = 0.4*rear_time+0.6*t;
    return rear_time;
}


void Init_Timer(void){ // timer for the echo pin reading
    echo_timer.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    echo_timer.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    echo_timer.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    echo_timer.timerClear = TIMER_A_DO_CLEAR;
    echo_timer.startTimer = false;

    Timer_A_initContinuousMode(TIMER_A1_BASE,&echo_timer);
}

void Init_Timer_Trig(){
    state_toggler.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    state_toggler.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    state_toggler.timerPeriod = 50000;
    state_toggler.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    state_toggler.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    state_toggler.timerClear = TIMER_A_DO_CLEAR;
    state_toggler.startTimer = 0;
    Timer_A_initUpDownMode(TIMER_A0_BASE, &state_toggler);

    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0
        );


    toggler_isr_regs.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    toggler_isr_regs.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    toggler_isr_regs.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    toggler_isr_regs.compareValue = 50000;
    Timer_A_initCompareMode(TIMER_A0_BASE, &toggler_isr_regs);

}

void start_timer(){ // timer for state toggler and responsible for sending trigger
    Timer_A_startCounter( TIMER_A0_BASE,
                          TIMER_A_UPDOWN_MODE
                );
}
#pragma vector=TIMER0_A0_VECTOR
__interrupt
void TIMER_A0_ISR(void){
    //enum state logic with high to to that pin...
    if(FRONT){
       FRONT = 0;
    }
    else{
        FRONT = 1;
    }
    sensorRead = 0;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void enterSetUpMode(){

    displayScrollText("SET UP MODE");

     //** NOTE: All thresholds are selected using the front sensor **//

    //set up rear threshold 1
    displayScrollText("SELECT REAR LARGEST");
    //wait for the user to select a threshold
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) != 0){
        if(!FRONT)
            rear_largest = readSensor_rear();
        showInt(rear_largest);
    }
    showInt(rear_largest);
    Timer_A_clear(TIMER_A1_BASE);
    
    //set up rear threshold 2
    displayScrollText("SELECT REAR MID");
    //wait for the user to select a threshold
    rear_time = 0;
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) != 0){
        if(!FRONT)
            rear_middle = readSensor_rear();
        showInt(rear_middle);
    }


    showInt(rear_middle);
    Timer_A_clear(TIMER_A1_BASE);
    //set up rear threshold 3
    displayScrollText("SELECT REAR SMALLEST");
    //wait for the user to select a threshold
    rear_time = 0;
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) != 0){
        if(!FRONT)
            rear_smallest = readSensor_rear();
        showInt(rear_smallest);
    }

    showInt(rear_smallest);
    //set up front threshold 1
    displayScrollText("SELECT FRONT LARGEST");
    //wait for the user to select a threshold
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) != 0){
        if(FRONT)
            front_largest = readSensor_front();
        showInt(front_largest);
    }
    
    showInt(front_largest);
    //set up front threshold 2
    displayScrollText("SELECT FRONT SMALLEST");
    //wait for the user to select a threshold
    front_time = 0;
    rear_time = 0;
    while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) != 0){
        if(FRONT)
            front_smallest = readSensor_front();
        showInt(front_smallest);
    }
    
    showInt(front_smallest);
    front_time = 0;
    displayScrollText("SET UP COMPLETED");
    return;
}

void rearSensorLogic(unsigned int rearSensor){

    //do all the state checks for the rear direction
   //turn on Green LED and turn off all other
    if(rearSensor > rear_largest){
         //turning on green
         GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
         //turning off all other
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
         GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);

       }
       else if(rearSensor >= rear_middle &&  rearSensor < rear_largest){
           //turning on yellow - p1.3
           GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
           //turning off all other
           GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
           GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
           GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
           }
       else if(rearSensor >= rear_smallest && rearSensor < rear_middle){
           //turning on orange - p1.4
           GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
           //turning off all other
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
       }
       else if(rearSensor < rear_smallest){
          //turning on red - p1.5
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
          //turning off all other
          GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
    }

    return;
}


void doubleBeep(){
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    __delay_cycles(10);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    __delay_cycles(1000);
}

void quadBeep(){
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
}

void noBeep(){
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
}
void frontSensorLogic(unsigned int frontSensor ){
    if(frontSensor < front_largest && frontSensor >= front_smallest){
        doubleBeep();
    }
    else if(frontSensor < front_smallest){
        quadBeep();
    }else{
        noBeep();
    }
    return;
}



void main(void)
{

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    //Init_PWM();     //Sets up a TRIGGER output
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_ULTRASONIC();
    Init_Timer_Trig();
    Init_Timer();
     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();
    
    /*Turn on PWM */
    //Timer_A_outputPWM(TIMER_A0_BASE, &param);
    // safe_to_read = 0;
    /* THE ISR */

    start_timer();
    enterSetUpMode();

    while(1){
          if(FRONT){
            if(sensorRead == 0){
                readSensor_front();
                sensorRead = 1;
                showUpper(front_time);
            }
            frontSensorLogic(front_time);
          } else {
            if(sensorRead == 0){
                readSensor_rear();
                showLower(rear_time);
                sensorRead = 1;
            }
            rearSensorLogic(rear_time);
          }
    }

//    while(1)
//    {
//        unsigned long int time = getTime();
//        showInt(time);
//         if(time > 700){
//             // buzz + led off
//           GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);
//           while(time > 0){
//               if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6)){
//                   GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
//               } else {
//                   GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);
//               }
//               time--;
//
//           }
//         } else {
//             GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);
//         }
//    }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}


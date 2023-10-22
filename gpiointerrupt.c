#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*----For the UART driver----*/
#define DISPLAY(x) UART2_write(uart,&output,x,NULL);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART2_Handle uart;

void initUART(void)
{
    UART2_Params uartParams;



    // Configure the driver
    UART2_Params_init(&uartParams);

    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {

    /* UART_open() failed */
    while (1);

    }
}

/*----For the I2C driver----*/

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        DISPLAY(snprintf(output, 64, "Found\n\r"))
        found = true;
        break;
    }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
        }
    }

    int16_t readTemp(void)
    {
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
        temperature |= 0xF000;
        }
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by nplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

/*----For the Timer driver----*/
    // Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char Timer = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)

    {
        Timer = 1;
    }

void initTimer(void)
    {
        Timer_Params params;

        // Init the driver
        Timer_init();

        // Configure the driver
        Timer_Params_init(&params);
        params.period = 1000000;
        params.periodUnits = Timer_PERIOD_US;
        params.timerMode = Timer_CONTINUOUS_CALLBACK;
        params.timerCallback = timerCallback;

        // Open the driver
        timer0 = Timer_open(CONFIG_TIMER_0, &params);

        if (timer0 == NULL) {
            /* Failed to initialized timer */
            while (1) {}
        }

        if (Timer_start(timer0) == Timer_STATUS_ERROR) {
            /* Failed to start timer */
            while (1) {}
        }
}



/* State Machine */

/* setting the ButtonClick */
unsigned char Button0_Click;
unsigned char Button1_Click;

/* Thermostat variables */
int setpoint; /*Temperature setting */
int temperature;
int heat;


/* For temperature */
enum TP_STATES {
    TP_Start,
    TP_UP, /*Looks for if button pressed*/
    TP_Change, /* Adjusts the setpoint */
} TP_STATE;


void Button_Click(){
    switch(TP_STATE) {
        case TP_Start:
            /*setting buttons to 0 aka not clicked */
            Button0_Click = 0;
            Button1_Click = 0;
            TP_STATE = TP_UP;
            break;

        /*Looks for button to be clicked */
        case TP_UP:
            if (Button0_Click || Button1_Click)
            {
                TP_STATE = TP_UP;
            }
            break;

        /*Checks if button has been clicked, if not go to SM_Raise */
        case TP_Change:
            if (!(Button0_Click || Button1_Click))
            {
                TP_STATE = TP_UP;
            }
            break;

        default:
            TP_STATE = TP_Start;
            break;
       }

    /* State actions */
    switch(TP_STATE)
    {
        case TP_Start:
            break;

        case TP_UP:
            break;

     /* adjusts the tempSet */
        case TP_Change:
            if (Button0_Click)
            {
                setpoint -=1;
                Button0_Click = 0;
            }

            if (Button1_Click)
            {
                setpoint +=1;
                Button1_Click = 0;
            }
            break;

        default:
            break;
    }
}

/* For the LEDs */
enum LE_STATES {
    LE_Start,
    LE_On, /* LED on */
    LE_Off, /* LED off */
} LE_STATE;

void LED_tempChange(){
    switch(LE_STATE){
        case LE_Start: /* Sets the starting point */
            heat = 0;
            LE_STATE = LE_Off;
            break;

        case LE_On:
            /*If temperature greater than tempSet point go to LE_Off */
            if (temperature > setpoint){
                LE_STATE  = LE_Off;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                heat = 0;
            }
            break;
         case LE_Off:
            /* If temperature less than tempSet point go to LE_On */
            if (temperature < setpoint){
                LE_STATE = LE_On;
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                heat = 1;
            }
            break;
        default:
            LE_STATE = LE_Start;
            break;
    }
    switch(LE_STATE){
        case LE_Start:
            break;
        case LE_On:
            break;
        case LE_Off:
            break;
        default:
            break;
    }
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    GPIO_toggle(CONFIG_GPIO_LED_0);
    Button1_Click = 1;

}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    initUART();
    initI2C();
    initTimer();
    GPIO_init();

    int seconds = 0;
    const unsigned int timerPeriod = 100000;
    unsigned int buttonTimer = 200000;
    unsigned int tempCheck = 500000;
    unsigned int uartTimer = 100000;


    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        /*GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);*/
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    /*GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);*/

    temperature = readTemp();
    setpoint = readTemp();

    /*Setting the State Machine */
    TP_STATE = TP_Start;
    LE_STATE = LE_Start;

    while(1){
        if (buttonTimer >= 200){
            Button_Click();
            buttonTimer = 0;
        }

        if (tempCheck >= 500){
            temperature = readTemp();
            LED_tempChange();
            tempCheck = 0;
        }

        if(uartTimer >=100){
            DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))
            seconds += 1;
            uartTimer = 0;
        }

        while(!Timer){
        }


        Timer = 0;

        buttonTimer += timerPeriod;
        tempCheck += timerPeriod;
        uartTimer += timerPeriod;

    }

}

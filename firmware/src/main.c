#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_usart.h"

/*============== CONST DEFINES ==============*/
#define COMMAND_UPDATE_BUFFER_SIZE 513
#define DMX_BUFFER_SIZE 513
#define RUN_LED     (1 << 0)
#define USART1_DE   (1 << 10) // PA10
#define UART_PACKET_START (0x7E)
#define UART_PACKET_END   (0xE7)

/*============== MACRO DEFINES ==============*/
#define ToggleRunLed() GPIOA->ODR ^= RUN_LED
#define SetDmxOutputToUartMode() GPIOA->MODER |= (1 << 19)
#define SetDmxOutputToGpioMode() GPIOA->MODER &= ~(3 << 18)
#define Timer14Wait_usec(usec) do { \
    TIM14->SR &= ~TIM_SR_UIF; \
    TIM14->ARR = usec; \
    TIM14->CNT = 0; \
    TIM14->CR1 |= TIM_CR1_CEN; \
    while (!(TIM14->SR & TIM_SR_UIF)); \
    TIM14->CR1 &= ~TIM_CR1_CEN; \
    } while(0)
#define DmxTransmit(data) do { \
    USART1->TDR = data; \
    while(!(USART1->ISR & (USART_ISR_TC))); \
    } while(0)


/**
 * This is used to store an array index and value for the DMX transmit array.
 */
typedef struct
{
    uint_fast16_t index;
    uint_fast8_t value;
} ValuePair;

volatile uint8_t dmx_data[DMX_BUFFER_SIZE];
volatile ValuePair updateBuffer[COMMAND_UPDATE_BUFFER_SIZE];
volatile uint_fast16_t updateBuffer_i = 0;
volatile uint_fast8_t dmxSent;
volatile uint_fast8_t dmxDataUpdateReady;


void initialize();

/**
 * The main program function
 */
int main()
{
    // initialize the hardware
    initialize();

    // initialize DMX512 universe to output 0s, any other default values should be handled here
    for (uint_fast16_t i = 1; i < sizeof(dmx_data); i++)
    {
        dmx_data[i] = 0;
    }

    // application main loop
    while (1)
    {
        // update the DMX buffer if there is data in the RX buffer
        if (dmxDataUpdateReady != 0)
        {
            // update the DMX buffer data
            for (uint_fast16_t i = 0; i < updateBuffer_i; i++)
            {
                if (updateBuffer[i].index < DMX_BUFFER_SIZE && updateBuffer[i].value < 256)
                {
                    dmx_data[updateBuffer[i].index] = updateBuffer[i].value;
                }
            }
            updateBuffer_i = 0;
            dmxDataUpdateReady = 0;
        }

        // DMX send period done
        if (dmxSent)
        {
            dmxSent = 0;

            // toggle run led to denote DMX packet sent
            ToggleRunLed();
        }
    }
    return 0;
}

/*
 * Initilize the peripheral configurations.
 */
void initialize()
{
    // Step 1: Enable the various hardware devices needed for the application.

    // enable GPIOA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    // enable USARTs
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE);

    // enable TIM14
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM14EN, ENABLE);

    // enable TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

    // Set the pin configurations for all required pins.
    // configure USART1 pins: TX in alternate function push pull
    GPIO_InitTypeDef pinCfg;
    pinCfg.GPIO_Mode  = GPIO_Mode_AF;
    pinCfg.GPIO_Pin   = GPIO_Pin_9;
    pinCfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    pinCfg.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &pinCfg);
    GPIOA -> BRR = (1<<9);
    GPIOA -> AFR[1] |= (1<<4);

    // configure LED pin
    pinCfg.GPIO_Mode  = GPIO_Mode_OUT;
    pinCfg.GPIO_OType = GPIO_OType_PP;
    pinCfg.GPIO_Pin   = RUN_LED;
    pinCfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pinCfg);


    // configure PA4 pin for output
    pinCfg.GPIO_Mode  = GPIO_Mode_OUT;
    pinCfg.GPIO_OType = GPIO_OType_PP;
    pinCfg.GPIO_Pin   = GPIO_Pin_4;
    pinCfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pinCfg);

    // configure USART1_DE pin for output
    pinCfg.GPIO_Pin   = USART1_DE;
    pinCfg.GPIO_Mode  = GPIO_Mode_OUT;
    pinCfg.GPIO_OType = GPIO_OType_PP;
    pinCfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pinCfg);

    // configure PA3 pin for input
    pinCfg.GPIO_Pin   = GPIO_Pin_3;
    pinCfg.GPIO_Mode  = GPIO_Mode_AF;
    pinCfg.GPIO_OType = GPIO_OType_PP;
    pinCfg.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    pinCfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pinCfg);

    // Configure USART1 for DMX transmissions
    USART_InitTypeDef usartCfg;
    usartCfg.USART_BaudRate = 250000;
    usartCfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartCfg.USART_Mode = USART_Mode_Tx;
    usartCfg.USART_Parity = USART_Parity_No;
    usartCfg.USART_StopBits = USART_StopBits_2;
    usartCfg.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &usartCfg);
    // enable USART1
    USART_Cmd(USART1, ENABLE);

    // set PA3 to AF1
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);
    // configure USART2
    USART_InitTypeDef usartCfg2;
    usartCfg2.USART_BaudRate = 9600;
    usartCfg2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usartCfg2.USART_Mode = USART_Mode_Rx;
    usartCfg2.USART_Parity = USART_Parity_No;
    usartCfg2.USART_StopBits = USART_StopBits_1;
    usartCfg2.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &usartCfg2);
    // enable USART2
    USART_Cmd(USART2, ENABLE);

    // Configure timer 3 for interrupt at 25 Hz = send DMX frame at 25Hz
    // configure TIM3
    TIM3->PSC  = 7999; // prescaler of 8000
    TIM3->ARR  = 39; // period of 40 for 25Hz
    TIM3->EGR  = 1; // only set update generation
    TIM3->DIER = 1; // only enable update interrupt
    TIM3->CR1  = 1; // only set counter enable

    // set timer 14's prescaler to 7+1=8
    TIM14->PSC = 7;

    // Setup interrrupt configurations
    // turn on the UART2 interrupt
    USART2-> CR1 |= USART_CR1_RE;
    USART2-> CR1 |= USART_CR1_RXNEIE;

    // enable USART2 interrupt and set priority
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 4);

    // enable TIM3 interrupt and set priority
    NVIC_SetPriority(TIM3_IRQn, 2);
    NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * UART 2 Interrupt Service Routine
 * This function receives the UART data from the computer, gathers the data,
 * and sets the dmxDataUpdateReady flag.
 */
void USART2_IRQHandler()
{
    static uint_fast16_t uartPacketSize;
    static uint8_t packetType = 0;
    static uint_fast8_t rxPacketState = 0;
    uint8_t rxData = 0;
    // read the UART2 data into the receive buffer
    // rxBuffer[rxBuffer_i] = USART2-> RDR;
    rxData = USART2-> RDR;

    switch(rxPacketState)
    {
        case 0: // packet header
        {
            // ignore everything but packet start value
            if (rxData == UART_PACKET_START)
            {
                rxPacketState++;
                updateBuffer_i = 0;
            }
            break;
        }
        case 1: // packet type (future use)
        {
            packetType = rxData;
            rxPacketState++;
            break;
        }
        case 2: // get the packet size LSB
        {
            uartPacketSize = rxData;
            rxPacketState++;
            break;
        }
        case 3: // get the packet size MSB
        {
            uartPacketSize += rxData << 8;
            rxPacketState++;
            break;
        }
        default: // handle other data received
        {
            if (rxPacketState == (uartPacketSize-1))
            {
                if (rxData == UART_PACKET_END)
                {
                    // message parsed, reset the receive buffer pointer
                    rxPacketState = 0;
                    // alert the main loop to update the DMX transmit buffer
                    dmxDataUpdateReady = 1;
                }
            }
            else
            {
                // store the packet data into updateBuffer - the ValuePair array
                updateBuffer[updateBuffer_i].index = rxPacketState-3;
                updateBuffer[updateBuffer_i++].value = rxData;

                rxPacketState++;
            }
            break;
        }
    }
}

/**
 * Timer 3 Interrupt Service Routine
 * This function is fired every 40ms and transmits the whole DMX buffer.
 */
void TIM3_IRQHandler()
{
    // clear the interrupt flag
    TIM3->SR &= ~0x0001;

    // bring the TX pin under GPIO control
    // this has the added effect that the pin is low
    SetDmxOutputToGpioMode();

    // wait 92 microseconds for the DMX break
    Timer14Wait_usec(92);

    // bring the TX pin under UART control
    SetDmxOutputToUartMode();

    // wait 12 microseconds for the MAB (Mark After Break)
    Timer14Wait_usec(12);

    // transmit the DMX data
    for (uint_fast16_t i = 0; i < sizeof(dmx_data); i++)
    {
        // transmit the byte with a blocking send
    	// can upgrade to DMA in the future, if desired, though this isn't needed
    	// for a simple application such as this
        DmxTransmit(dmx_data[i]);
    }

    // wait one additional character time before releasing the line
    Timer14Wait_usec(44);

    // flag DMX packet as sent
    dmxSent = 1;
}



// ----------------------------------------------------------------------------

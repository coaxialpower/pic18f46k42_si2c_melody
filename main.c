 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

/*
© [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"
static bool I2C1_Callback(i2c_client_transfer_event_t event)
{
    /* User has to register callback. Refer example code */
    switch (event)
    {
        case I2C_CLIENT_TRANSFER_EVENT_NONE:          /**< I2C Bus Idle state */
        case I2C_CLIENT_TRANSFER_EVENT_ADDR_MATCH:        /**< Address match event */
        case I2C_CLIENT_TRANSFER_EVENT_RX_READY:         /**< Data sent by I2C Host is available */
        case I2C_CLIENT_TRANSFER_EVENT_TX_READY:          /**< I2C client can respond to data read request from I2C Host */
        case I2C_CLIENT_TRANSFER_EVENT_STOP_BIT_RECEIVED: /**< I2C stop bit received */
        case I2C_CLIENT_TRANSFER_EVENT_ERROR:             /**< I2C Bus error occurred */
        default:
            break;
    }
    return true;
}

/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 
    I2C1_CallbackRegister(I2C1_Callback);
    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 


    while(1)
    {
    }    
}
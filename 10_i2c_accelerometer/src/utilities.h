/*
 * utilities.h
 *
 *  Created on: Apr 20, 2015
 *      Author: David
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define LED_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3

#define BUTTON_PORT					gpioPortB
#define SET_BUTTON_PIN				10

#define DEBUG_BREAK		__asm__("BKPT #0");

void delay(uint32_t milliseconds);
int32_t set_timeout_ms(int32_t timeout_ms);
int32_t expired_ms(int32_t timeout_ms);
bool get_button();
void set_led(int number, int level);
void setup_utilities();
void print(const char* format, ...);

#endif /* UTILITIES_H_ */

/*
 * hardware.h
 *
 *  Created on: 6/07/2019
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

typedef enum {
	OFF 	= 0x00,
	ON 		= 0x01
} LEDStatusType;

void initLeds();

ALWAYS_INLINE void setLed1 (LEDStatusType status)
{
	OUTPUT_PIN(GPIOB, 0) = status;
}

ALWAYS_INLINE void setLed2(LEDStatusType status)
{
	OUTPUT_PIN(GPIOB, 7) = status;
}

ALWAYS_INLINE void setLed3(LEDStatusType status)
{
	OUTPUT_PIN(GPIOB, 14) = status;
}

ALWAYS_INLINE void toggleLed1()
{
	OUTPUT_PIN(GPIOB, 0) = !OUTPUT_PIN(GPIOB, 0);
}

ALWAYS_INLINE void toggleLed2()
{
	OUTPUT_PIN(GPIOB, 7) = !OUTPUT_PIN(GPIOB, 7);
}

ALWAYS_INLINE void toggleLed3()
{
	OUTPUT_PIN(GPIOB, 14) = !OUTPUT_PIN(GPIOB, 14);
}

#endif /* HARDWARE_H_ */

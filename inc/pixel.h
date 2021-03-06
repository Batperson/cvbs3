/*
 * pixel.h
 *
 *  Created on: 19/07/2019
 */

#ifndef PIXEL_H_
#define PIXEL_H_

typedef uint8_t PIXEL, *PPIXEL;

#define ARGB(a,r,g,b)		((PIXEL)(a<<6)|(r<<4)|(g<<2)|b)
#define RGB(r,g,b) 			((PIXEL)0x40|(r<<4)|(g<<2)|b)
#define RED					RGB(3,0,0)
#define GREEN				RGB(0,3,0)
#define BLUE				RGB(0,0,3)
#define YELLOW				RGB(3,3,0)
#define MAGENTA				RGB(3,0,3)
#define CYAN				RGB(0,3,3)
#define WHITE				RGB(3,3,3)
#define BLACK				RGB(0,0,0)
#define GRAY				RGB(2,2,2)
#define DKGRAY				RGB(1,1,1)
#define DKRED				RGB(1,0,0)
#define DKGREEN				RGB(0,1,0)
#define DKBLUE				RGB(0,0,1)

#define ORANGE				RGB(3,1,0)
#define PURPLE				RGB(1,0,1)
#define INDIGO 				RGB(2,0,1)
#define VIOLET				RGB(2,0,3)

#define TRANSPARENT			(PIXEL)0

#endif /* PIXEL_H_ */

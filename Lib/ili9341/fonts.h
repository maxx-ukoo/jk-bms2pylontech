/*
 * fonts.h
 *
 *  Created on: Mar 12, 2022
 *      Author: maxx
 */

#ifndef FONTS_H_
#define FONTS_H_


#include <stdint.h>

typedef struct {
	uint8_t start_char;
	uint8_t offset;
    uint8_t width;
    uint8_t height;
    uint8_t bpl;
    const uint8_t *data;
} FontDef;

extern FontDef Font_8x12;
extern FontDef Font_10x13;
extern FontDef Font_34x36;
extern FontDef Font_37x47;
extern FontDef Font_26_B;

#endif /* FONTS_H_ */

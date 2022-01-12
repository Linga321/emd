/*
 * DigitalIoPin.h
 *
 *  Created on: 05 September 2021
 *      Author: Simon Bauer
 *      For Lab 4 Task 1
 */

#ifndef DIGITALIOPIN_H_
#define DIGITALIOPIN_H_

#include "DigitalIoPin.h"
#include "board.h"
#include <cr_section_macros.h>

class DigitalIoPin {
public:
	DigitalIoPin(int port, int pin, bool input = true, bool pullup = true, bool invert = false);
	DigitalIoPin(const DigitalIoPin &) = delete;
	virtual ~DigitalIoPin() {};
	bool read();
	void write(bool value);
private:
	int in_port;
	int in_pin;
	uint32_t modefunc;
};

#endif /* DIGITALIOPIN_H_ */

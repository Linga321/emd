
#include "DigitalIoPin.h"

DigitalIoPin::DigitalIoPin(int port, int pin, bool input, bool pullup, bool invert): in_port(port), in_pin(pin){

	if(pullup == true){
		if(invert == true){
			modefunc = (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN);
		}else{
			modefunc = (IOCON_MODE_PULLUP | IOCON_DIGMODE_EN | IOCON_INV_EN);
		}
	}else{

		if(invert == true){
			modefunc = (IOCON_MODE_INACT | IOCON_DIGMODE_EN );

		}else{
			modefunc = (IOCON_MODE_INACT | IOCON_DIGMODE_EN | IOCON_INV_EN);
		}
	}

	Chip_IOCON_PinMuxSet(LPC_IOCON, in_port, in_pin, modefunc);

	if(input == true){
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, in_port, in_pin);
	}else{
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, in_port, in_pin);
	}
	//Chip_GPIO_SetPinState(LPC_GPIO, in_port, in_pin, invert);

}

bool DigitalIoPin::read() { //reading function
	return Chip_GPIO_GetPinState(LPC_GPIO, in_port, in_pin);
}

void DigitalIoPin::write(bool state) { //writing function
	Chip_GPIO_SetPinState(LPC_GPIO, in_port, in_pin, state);
}


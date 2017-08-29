/*
    This file is additional to Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

    author of this additional File : RAyWB / Robert Ayrenschmalz

*/
#ifndef CustomEventsImpl_H
#define CustomEventsImpl_H



MCP23017 MCP1(0x24);  
MCP23017 MCP2(0x25);  
MCP23017 MCP3(0x20);  

// if you are not shure about the I2C Adresss of your components run Nick Gammon´s I2C Scanner
// see:  http://www.gammon.com.au/i2c


//#########################################################################################
//##### Initialization, Setup of I2C and Components
//#########################################################################################

void Custom_Init_Early()
{ 

#ifndef COMPILE_I2C_DRIVER
HAL::i2cInit(400000);  // usually 100000 Hz  , my setup works on 400000Hz
#endif  

  MCP1.Init();
  MCP2.Init();
  MCP3.Init();
 
}

//#########################################################################################
//##### I2C Portexpander MCP23017
//##### Logic mostly from Github/kasperskaarhoj 
//##### modified to use HAL Routines by Robert Ayrenschmalz alias RAyWB
//#########################################################################################

MCP23017::MCP23017(uint8_t address)
{
  i2c_add=address<<1;
}

void MCP23017::Init(void)
{
	_GPIO = 0x0000;
 	_IODIR = 0xFFFF; //set all to inputs
	_GPPU = 0x0000;
	
   HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(0x05);
   HAL::i2cWrite(0x00);
   HAL::i2cStop(); 
	 writeRegister(MCP23017_IODIR, _IODIR);
	
}

void MCP23017::SetInput(uint8_t pin) {

   _IODIR |= 1 << pin;
	 writeRegister(MCP23017_IODIR, _IODIR);
}


void MCP23017::SetOutput(uint8_t pin) {
	
	 _IODIR &= ~(1 << pin);
	 writeRegister(MCP23017_IODIR, _IODIR);
}


bool MCP23017::Read(uint8_t pin) {
	 _GPIO = readRegister(MCP23017_GPIO);
	 if ( _GPIO & (1 << pin)) return HIGH;
	 else return LOW;
}

void MCP23017::Write(uint8_t pin, bool val) {
	//If this pin is an INPUT pin, a write here will
	//enable the internal pullup
	//otherwise, it will set the OUTPUT voltage
	//as appropriate.
	 bool isOutput = !(_IODIR & 1<<pin);

	  if (isOutput) {
		 //This is an output pin so just write the value
		  if (val) _GPIO |= 1 << pin;
		  else _GPIO &= ~(1 << pin);
		 writeRegister(MCP23017_GPIO, _GPIO);
	   }
	  else {
		 //This is an input pin, so we need to enable the pullup
		  if (val) _GPPU |= 1 << pin;
		  else _GPPU &= ~(1 << pin);
		 writeRegister(MCP23017_GPPU, _GPPU);
	   }
}

uint16_t MCP23017::ReadPort() {
 InterruptProtectedBlock noInts;
 noInts.protect();
	 _GPIO = readRegister(MCP23017_GPIO);
	 return _GPIO;
 noInts.unprotect(); 
}
void MCP23017::WritePort(uint16_t data) {
	 _GPIO = data;
	 writeRegister(MCP23017_GPIO, _GPIO);
}

//PRIVATE
/*
void MCP23017::writeRegister(int regAddress, byte data) {
	
    HAL::i2cStartWait(i2c_add + I2C_WRITE);
    HAL::i2cWrite(regAddress);//IODIRA
    HAL::i2cWrite(data);
    HAL::i2cStop(); 
	
}
*/

void MCP23017::writeRegister(uint16_t regAddress, uint16_t data) {
	
	 HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(regAddress);
   HAL::i2cWrite(data>>8);
   HAL::i2cWrite(data);
   HAL::i2cStop(); 
}

uint16_t MCP23017::readRegister(uint16_t regAddress) {
	
	uint16_t data = 0x00;
	 
	 HAL::i2cStartWait(i2c_add + I2C_WRITE);
   HAL::i2cWrite(regAddress);
	 HAL::i2cStop();
	 HAL::i2cStartWait(i2c_add + I2C_READ);
   data = (HAL::i2cReadAck()<<8);
   data += HAL::i2cReadNak();
   HAL::i2cStop();
   return data ;
} 

// End MCP23017


//#########################################################################################
//#### Read buttons from MCP23017 external I2C device(s)  
//#### as we read the complete bitmask it´s possible to realize two or more buttons pressed
//#### at the same time in order to realize "shift" Functions etc.
//#########################################################################################

int Custom_CheckSlowKeys()
{
  int action=0;
  char buf[20];

  {
	 uint16_t buttonval = 0xFFFF-MCP1.ReadPort();
	 uint16_t buttonval2 = 0xFFFF-MCP2.ReadPort();
	 uint16_t buttonval3 = 0xFFFF-MCP3.ReadPort();
	    
switch (buttonval) {

      
      case 1:       Com::print(" MCP1 Button1\n");
                    break;
            
      case 2:       Com::print(" MCP1 Button2\n");
                    break;
   
// and so on 
   
      default: 
                    Com::print(" MCP1 no Button\n");
                    break;
   }
   
switch (buttonval2) {
  
      case 1:       Com::print(" MCP2 Button1\n");
                    break;
            
      case 2:       Com::print(" MCP2 Button2\n");
                    break;
   
//  and so on 
   
      default:      Com::print(" MCP2 no Button\n");
                    break;
   }

switch (buttonval3) {
  
      case 1:       Com::print(" MCP3 Button1\n");
                    break;
            
      case 2:       Com::print(" MCP3 Button2\n");
                    break;
   
//  and so on 
   
      default:      Com::print(" MCP3 no Button\n");
                    break;
   }

   return(action);
 }


}//SLOWKEYS


#endif

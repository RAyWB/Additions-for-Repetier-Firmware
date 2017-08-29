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
//###  User defined Events
//#########################################################################################

int Custom_Execute(int action,bool allowMoves) 
{
            
  switch(action) {

    
  
   case UI_ACTION_IDENT01:
           Com::printFLN(PSTR("This is 1"));
           break;
   case UI_ACTION_IDENT02:
           Com::printFLN(PSTR("This is 2"));
           break;
   case UI_ACTION_IDENT03:
           Com::printFLN(PSTR("This is 3"));
           break;
   case UI_ACTION_IDENT04:
           Com::printFLN(PSTR("This is 4"));
           break;
   case UI_ACTION_IDENT05:
           Com::printFLN(PSTR("This is 5"));
           break;
   case UI_ACTION_IDENT06:
           Com::printFLN(PSTR("This is 6"));
           break;
   case UI_ACTION_IDENT07:
           Com::printFLN(PSTR("This is 7"));
           break;
   case UI_ACTION_IDENT08:
           Com::printFLN(PSTR("This is 8"));
           break;
   case UI_ACTION_IDENT09:
           Com::printFLN(PSTR("This is 9"));
           break;
   case UI_ACTION_IDENT10:
           Com::printFLN(PSTR("This is 10"));
           break;
   case UI_ACTION_IDENT11:
           Com::printFLN(PSTR("This is 11"));
           break;
   case UI_ACTION_IDENT12:
           Com::printFLN(PSTR("This is 12"));
           break;
   case UI_ACTION_IDENT13:
           Com::printFLN(PSTR("This is 13"));
           break;
   case UI_ACTION_IDENT14:
           Com::printFLN(PSTR("This is 14"));
           break;
   case UI_ACTION_IDENT15:
           Com::printFLN(PSTR("This is 15"));
           break;
   case UI_ACTION_IDENT16:
           Com::printFLN(PSTR("This is 16"));
           break;
   case UI_ACTION_IDENTM01:
           Com::printFLN(PSTR("This is Button 1"));
           break;
   case UI_ACTION_IDENTM02:
           Com::printFLN(PSTR("This is Button 2"));
           break;              
   case UI_ACTION_IDENTM03:
           Com::printFLN(PSTR("This is Button 3"));
           break;
   case UI_ACTION_IDENTM04:
           Com::printFLN(PSTR("This is Button 4"));
           break;      
   case UI_ACTION_IDENTM05:
           Com::printFLN(PSTR("This is Button 5"));
           break;  
   case UI_ACTION_IDENTM06:
           Com::printFLN(PSTR("This is Button 6"));
           break;  
   case UI_ACTION_IDENTM07:
           Com::printFLN(PSTR("This is Button 7"));
           break; 
   case UI_ACTION_IDENTM08:
           Com::printFLN(PSTR("This is Button 8"));
           break;
   case UI_ACTION_IDENTM09:
           Com::printFLN(PSTR("This is Button 9"));
           break;
   case UI_ACTION_IDENTM10:
           Com::printFLN(PSTR("This is Button 10"));
           break;
   case UI_ACTION_IDENTM11:
           Com::printFLN(PSTR("This is Button 11"));
           break;
   case UI_ACTION_IDENTM12:
           Com::printFLN(PSTR("This is Button 12"));
           break;
   case UI_ACTION_IDENTM13:
           Com::printFLN(PSTR("This is Button 13"));
           break;
   case UI_ACTION_IDENTM14:
           Com::printFLN(PSTR("This is Button 14"));
           break;
   case UI_ACTION_IDENTM15:
           Com::printFLN(PSTR("This is Button 15"));
           break;
   case UI_ACTION_IDENTM16:
           Com::printFLN(PSTR("This is Button 16"));
           break;
       
}
return 0 ;
}//Custom_Execute


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
    
switch (buttonval) {

  
      //B0     
      case 1:       action=UI_ACTION_IDENTM01;
                    break;
      //B1
      case 2:       action=UI_ACTION_IDENTM02;
                    break;   
      //B2
      case 4:       action=UI_ACTION_IDENTM03;
                    break;       
      //B3
      case 8:       action=UI_ACTION_IDENTM04;
                    break; 
      //B4            
      case 16:      action=UI_ACTION_IDENTM05;
                    break;       
      //B5
      case 32:      action=UI_ACTION_IDENTM06;
                    break;
      //B6
      case 64:      action=UI_ACTION_IDENTM07;
                    break;
      //B7      
      case 128:     action=UI_ACTION_IDENTM08;
                    break;
     
      case 256:     action=UI_ACTION_IDENTM09;
                    break; 
      //A1          
      case 512:     action=UI_ACTION_IDENTM10;
                    break;
      //A2
      case 1024:    action=UI_ACTION_IDENTM11;
                    break;
      //A3         
      case 2048:    action=UI_ACTION_IDENTM12;
                    break;      
      //A4
      case 4096:    action=UI_ACTION_IDENTM13;
                    break;
      //A5           
      case 8192:    action=UI_ACTION_IDENTM14;
                    break;      
      //A6
      case 16384:   action=UI_ACTION_IDENTM15;
                    break;
      //A7      
      case 32768:   action=UI_ACTION_IDENTM16;
                    break;

      default: 

                    //Com::print(" MCP1 no Button\n");
                    break;
   }
   
switch (buttonval2) {
  
  
      //B0     
      case 1:       action=UI_ACTION_IDENT01;
                    break;
      //B1
      case 2:       action=UI_ACTION_IDENT02;
                    break;   
      //B2
      case 4:       action=UI_ACTION_IDENT03;
                    break;       
      //B3
      case 8:       action=UI_ACTION_IDENT04;
                    break; 
      //B4            
      case 16:      action=UI_ACTION_IDENT05;
                    break;       
      //B5
      case 32:      action=UI_ACTION_IDENT06;
                    break;
      //B6
      case 64:      action=UI_ACTION_IDENT07;
                    break;
      //B7      
      case 128:     action=UI_ACTION_IDENT08;
                    break;
     
      case 256:     action=UI_ACTION_IDENT09;
                    break; 
      //A1          
      case 512:     action=UI_ACTION_IDENT10;
                    break;
      //A2
      case 1024:    action=UI_ACTION_IDENT11;
                    break;
      //A3         
      case 2048:    action=UI_ACTION_IDENT12;
                    break;      
      //A4
      case 4096:    action=UI_ACTION_IDENT13;
                    break;
      //A5           
      case 8192:    action=UI_ACTION_IDENT14;
                    break;      
      //A6
      case 16384:   action=UI_ACTION_IDENT15;
                    break;
      //A7      
      case 32768:   action=UI_ACTION_IDENT16;
                    break;

      default: 

                    //Com::print(" MCP2 no Button\n");
                    break;
   }
   if(action)
   {
    Com::print("action no:"); 
    Com::print(action);
    Com::print("\n");
   }
   return(action);
 }


}//SLOWKEYS


#endif

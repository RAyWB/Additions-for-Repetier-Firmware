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
PCAPWM   EXTPWM1(0x41);
MCP4725  DAC1(0x60);

// if you are not shure about the I2C Adresss of your components run Nick Gammon´s I2C Scanner
// see:  http://www.gammon.com.au/i2c

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42

//#########################################################################################
//### Variables used in M6 command
//#########################################################################################

char  UI_Message[20]; 
float Length = 0; // variable for tool length
bool measured = false ;
bool M6_Message;
int loopcount = 0;

//#########################################################################################
//### GAMMA Variables
//### necessary to correct grayscale , mostly depending on material you want to engrave
//### for my setup these values work
//### i use a PLTB450B 1.6W Laser diode with analog modulation
//#########################################################################################

uint16_t Outval;
bool Gamma_on=true; // preset Gamma on
float GAMMA=1.4;
uint16_t LASER_BASE_OFFSET=1000;
uint16_t LASER_LIMIT=4095;
float newintens;

float SAFE_Z ;//used for flexible Safe Z-Height on SD Pause

//#########################################################################################
//##### Initialization, Setup of I2C and Components
//#########################################################################################

void Custom_Init_Early()
{ 

#ifndef COMPILE_I2C_DRIVER
HAL::i2cInit(400000);  // usually 100000 Hz  , my setup works on 400000Hz
#endif  

#if defined( EPWM_OE) && EPWM_OE > -1  //Output Enable should be used to get defined Power Up Status
  SET_OUTPUT( EPWM_OE );
  WRITE( EPWM_OE, HIGH);
#endif  

#if defined( FEATURE_I2C_MACROS) && FEATURE_I2C_MACROS !=0
  MCP1.Init();
#endif 
 
#if defined( FEATURE_EXT_PWM) && FEATURE_EXT_PWM !=0
  EXTPWM1.Init();
  EXTPWM1.SetFREQ(PWM_FREQU);//PWM frequency PCA9685 referring Datasheet adjustable from 24 to 1526 Hz
                             //never tried over 1200  !!!!! all Pins use same PWM frequency
#endif 

#if defined( FEATURE_SERVO_ELECTRODE) && FEATURE_SERVO_ELECTRODE !=0
   ELECTRODE_OFF; 
 #endif 

#if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
  DAC1.Init();
#endif
 
#if defined( EPWM_OE) && EPWM_OE > -1
  WRITE( EPWM_OE, LOW);// output enable after power up otherwise undefined status
#endif
}

//#########################################################################################
//##### Initialization, Set DAC to defined Status
//##### just to show, of coarse DAC can be written in Custom_Init_Early also
//#########################################################################################

void Custom_Init()
{ 
 #if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
   DAC1.WriteOutput(0);
 #endif 
 
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
//##### I2C DAC MCP4725
//#########################################################################################

#define MCP4725_COMMAND 0x40  // writes only Output,not EEprom

MCP4725::MCP4725(uint8_t address)
{
  i2c_add = address<<1;
}

void MCP4725::Init(void)
{
  HAL::i2cStartWait((i2c_add)+ I2C_WRITE);
  HAL::i2cWrite(MCP4725_COMMAND);  
  HAL::i2cStop();
}

void MCP4725::WriteOutput(uint16_t value)
{
  HAL::i2cStartWait((i2c_add)+ I2C_WRITE);
  HAL::i2cWrite(MCP4725_COMMAND);  
  HAL::i2cWrite(value>>4); // 12 Bit, write Bit11..Bit4
  HAL::i2cWrite(value<<4); // write Bit3..0 , followed by 4*0
  HAL::i2cStop();
}
 
//end MCP4725

//#########################################################################################
//### external PWM  PCA9685   16 channels
//#########################################################################################


PCAPWM::PCAPWM(uint8_t address)
{
  PCA_addr = address<<1;
}

void PCAPWM::Init(void)
{ 
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(0x0);
  HAL::i2cStop();
}

void PCAPWM::SetFREQ(float freq) 
{
  freq *= 0.99; 
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  HAL::i2cStartWait(PCA_addr + I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr + I2C_READ);
  uint8_t oldmode = HAL::i2cReadNak();
  HAL::i2cStop();
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  HAL::i2cStartWait(PCA_addr + I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(newmode);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0xFE);
  HAL::i2cWrite(prescale);
  HAL::i2cStop();
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(oldmode);
  HAL::i2cStop();
  HAL::delayMilliseconds(50);
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x0);
  HAL::i2cWrite(oldmode | 0xa1);
  HAL::i2cStop();
}

void PCAPWM::SetPWM(uint8_t num, uint16_t on, uint16_t off)
{
  HAL::i2cStartWait(PCA_addr+ I2C_WRITE);
  HAL::i2cWrite(0x6+4*num);
  HAL::i2cWrite(on);
  HAL::i2cWrite(on>>8);
  HAL::i2cWrite(off);
  HAL::i2cWrite(off>>8);
  HAL::i2cStop();
}

void PCAPWM::SetPIN(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  if( val >4095) val = 4095;//min(val, 4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      SetPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      SetPWM(num, 0, 4096);
    }
    else {
      SetPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      SetPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      SetPWM(num, 0, 4096);
    }
    else {
      SetPWM(num, 0, val);
    }
  }
}
 
 
void PCAPWM::SetSERVO(uint8_t num, float degree)

{  
  SetFREQ(50);
   HAL::delayMilliseconds(150);
  int degre=(constrain((int)degree*10,-5,1850));
  int pulse=map(degre,-50,1850,110,500);
  SetPWM(num, 0,pulse+5); 
   HAL::delayMilliseconds(150);
} 

//End PCA9685 


//#########################################################################################
//#### Laser Driver replacement
//#########################################################################################

bool SetLaser(uint16_t newIntensity)
 {
  
#if defined( FEATURE_PWM_LASER) && FEATURE_PWM_LASER !=0
  EXTPWM1.SetPIN(EXT_LASER_PIN,newIntensity,LASER_ON_HIGH);
#endif

#if defined( FEATURE_ANALOG_LASER) && FEATURE_ANALOG_LASER !=0
  DAC1.WriteOutput(newIntensity);
#endif  

 
  return false;
 }

 
//#########################################################################################
//#### show Message on Display , gets not overwritten by "idle"
//#########################################################################################

void CustomMessage(){

  char buf[20];
  int i;
  
  if (Printer::mode == PRINTER_MODE_LASER)
  {
   sprintf(buf, "L:%iF:%iG:%i.%iL%i",LaserDriver::intens,LASER_BASE_OFFSET,(int)GAMMA,(int)(GAMMA*10)%10,LASER_LIMIT);
    UI_STATUS_UPD_RAM(buf);  
  }
 
 if (M6_Message)
  { UI_STATUS_UPD_RAM(UI_Message);}

}//CustomMessage

//#########################################################################################
//#### Timer 100ms
//#########################################################################################

void Custom_100MS()
 {
  
 }

//#########################################################################################
//#### Timer 500ms
//#########################################################################################

void Custom_500MS()
 {
   loopcount++;
   if(loopcount=3)
   {
   CustomMessage();     
   loopcount=0;
   }
 }

//#########################################################################################
//#### GCode addition/replacement
//#########################################################################################

 bool Custom_GCode(GCode *com) 
 {
  char buf[35];
  volatile float BackupFeedrate; //backup variable for G1 Feedrate
  uint32_t codenum; //throw away variable

//#########################################################################################
/* modified G0/G1 to use max.possible feedrate for G0 moves
   as most of CAM programs do not add feedrate when generating Gcode
   i use maximum feedrate from fastest axis for G0 moves as for slower axis
   feedrate is limited by Firmware */
//#########################################################################################
  
    switch(com->G) {
        case 0: // G0 -> G1
        case 1: // G1
        
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            {
                // disable laser for G0 moves
                bool laserOn = LaserDriver::laserOn;
                if(Printer::mode == PRINTER_MODE_LASER) {
                    if(com->G == 0) {
                        LaserDriver::laserOn = false;
                        LaserDriver::firstMove = true; //set G1 flag for Laser
                        Com::print("firstmove_true");//for debug
                    } else {
#if LASER_WARMUP_TIME > 0                        
                        uint16_t power = (com->hasX() || com->hasY()) && (LaserDriver::laserOn || com->hasE()) ? LaserDriver::intensity : 0;
                        if(power > 0 && LaserDriver::firstMove) {
                            PrintLine::waitForXFreeLines(1,true);
                            PrintLine::LaserWarmUp(LASER_WARMUP_TIME);
                            LaserDriver::firstMove = false;
                        }
#endif                        
                    }
                }                
#endif // defined Laser

                if(com->hasS()) 
                {
                   if(Printer::mode == PRINTER_MODE_FFF) 
                {
                  Printer::setNoDestinationCheck(com->S != 0);
                }
      
                             
#if defined(SUPPORT_LASER) && SUPPORT_LASER   //implemented S value for Laser to handle GRBL Laser PWM code
                  
                  if(Printer::mode == PRINTER_MODE_LASER) 
                  {
               
#if defined(GAMMA_CORRECTION) && (GAMMA_CORRECTION==true)
                      newintens =(com->S);
                      if (Gamma_on)
                      {
                      Outval=pow(newintens/LASER_PWM_MAX,GAMMA)*LASER_PWM_MAX; //Gamma function
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale gamma function and offset to max 
                      Com::printFLN(PSTR("S orig:"),(int)newintens);
                      }
                      else
                      {
                      Outval=newintens;
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale offset to max 
                      Com::printFLN(PSTR("S Gamma off LV:"),(int)newintens);
                      }
                     
#else
                LaserDriver::intensity = constrain(com->S,0,LASER_PWM_MAX);
#endif                
      
                Com::printFLN(PSTR("LV S:"),(int)LaserDriver::intensity);

              
                 }
#endif // defined SUPPORT_LASER
                }
                
                if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
#if NONLINEAR_SYSTEM
                    if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, true)) {
                        Com::printWarningFLN(PSTR("executeGCode / queueDeltaMove returns error"));
                    }
#else
                    if(com->G == 0) { 
                    BackupFeedrate = Printer::feedrate; //backup feedrate
				           	// select faster axis max feed rate for g0,other axis speed is limited by maxFeedrate
                    if( Printer::maxFeedrate[X_AXIS]>=Printer::maxFeedrate[Y_AXIS])
                    Printer::feedrate = Printer::maxFeedrate[X_AXIS];//use faster axis max Feedrate for G0
                    else Printer::feedrate = Printer::maxFeedrate[Y_AXIS];

                    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
                    Printer::feedrate = BackupFeedrate;//restore Feedrate for G1
                    }//end go
                    else//g1
                    {
                    PrintLine::queueCartesianMove(ALWAYS_CHECK_ENDSTOPS, true);
                    }//end g1
#endif
#if UI_HAS_KEYS
                // ui can only execute motion commands if we are not waiting inside a move for an
                // old move to finish. For normal response times, we always leave one free after
                // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
                // gets filled while waiting, the lost is neglectable.
                PrintLine::waitForXFreeLines(1, true);
#endif // UI_HAS_KEYS
#ifdef DEBUG_QUEUE_MOVE
                {

                    InterruptProtectedBlock noInts;
                    int lc = (int)PrintLine::linesCount;
                    int lp = (int)PrintLine::linesPos;
                    int wp = (int)PrintLine::linesWritePos;
                    int n = (wp - lp);
                    if(n < 0) n += PRINTLINE_CACHE_SIZE;
                    noInts.unprotect();
                    if(n != lc)
                        Com::printFLN(PSTR("Buffer corrupted"));
                }
#endif

#if defined(SUPPORT_LASER) && SUPPORT_LASER
                {
                  LaserDriver::laserOn = laserOn;
                  if(Printer::mode == PRINTER_MODE_LASER) 
                     {
                      Com::printFLN(PSTR("LaserOn:"),(int)LaserDriver::intensity);
                     }
                }
               
            }
#endif // defined
break;

//#########################################################################################
//#### modified G30
//#### G30 C to calibrate tool height
//#### G30 T (prototype for M6 ) adjust height when tool changed
//#### G30 L<value> probes Z and Lifts Z by <value>
///#########################################################################################
 
     case 30: 
      {   // G30 [Pn] [S]
        // G30 (the same as G30 P3) single probe set Z0
        // G30 S1 Z<real_z_pos> - measures probe height (P is ignored) assuming we are at real height Z
 
#if defined(SUPPORT_CNC) && SUPPORT_CNC && defined(M6_ACK_PIN) && (M6_ACK_PIN>=0)
   if(Printer::mode == PRINTER_MODE_CNC)
   {
        if (com->hasC()) // G30 Calibrate
           {
             ELECTRODE_CLOSE;
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,Printer::lastCmdPos[Z_AXIS]+50, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]); //lift z 50mm
             Printer::moveToReal(MEAS_XPOS,MEAS_YPOS,IGNORE_COORDINATE, IGNORE_COORDINATE, 2500); // move to probe point
             Length= Printer::runZProbe(true, true,2, true);  //probe
             measured = true;
             Commands::waitUntilEndOfAllMoves();
             ELECTRODE_OPEN;
             Com::printFLN(PSTR("Tool 1: "),Length,3);
             ELECTRODE_OFF;
             break;
           }
        
       if (com->hasT()) //G30 Toolchange  PROTOTYPE for M6
           {
             Printer::setBlockingReceive(true);
                         
             float OrigOff=Printer::realZPosition()+Printer::coordinateOffset[Z_AXIS];
             Com::printFLN(PSTR("orig_off: "),OrigOff);

            if ((!measured) || (Length==0))
            {
             ELECTRODE_CLOSE;
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,Printer::lastCmdPos[Z_AXIS]+50, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]); //lift z 50mm
             Printer::moveToReal(MEAS_XPOS,MEAS_YPOS,IGNORE_COORDINATE, IGNORE_COORDINATE, 2500); // move to probe point
             Length= Printer::runZProbe(true, true,2, true);  //probe
             measured = true; //set flag
             Commands::waitUntilEndOfAllMoves();
             ELECTRODE_OPEN;
            }
             Com::printFLN(PSTR("Tool 1: "),Length,3); //#### Info 
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,Printer::lastCmdPos[Z_AXIS]+50, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]); //lift z
             Printer::moveToReal(MEAS_XPOS,MEAS_YPOS,IGNORE_COORDINATE, IGNORE_COORDINATE, 2500); // move to probe point
            
           while (!CONTINUE)

        /* wait for acknowledge key pressed !! no release by uiaction , no possibility to release from host
         *  for safety reasons to ensure both hands are off and and no second person can accidentally 
         *  start cycle from host.
         *  for shure it is not dummy proof but  high level safe
         * ####### now we change tool #######
         */
        {
         Printer::defaultLoopActions();
        }

             ELECTRODE_CLOSE;
             float NewLength= Printer::runZProbe(true, true,2, true);//measure new tool
             Commands::waitUntilEndOfAllMoves();
             Com::printFLN(PSTR("Tool 2: "),NewLength,3);//#### Info 
             float diff = NewLength-Length;      // calculate difference       
             Com::printFLN(PSTR("offset: "),diff,3);//#### Info 
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,Printer::lastCmdPos[Z_AXIS]-diff+50, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);// lift z
             Printer::moveToReal(Printer::lastCmdPos[X_AXIS],Printer::lastCmdPos[Y_AXIS],IGNORE_COORDINATE, IGNORE_COORDINATE, 2500);// move back to workpiece
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,Printer::lastCmdPos[Z_AXIS]-diff, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);// move z to corrected origin
             Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords to startheight;
             Printer::coordinateOffset[Z_AXIS] += diff;//add difference to OrigOff
             Com::printFLN(PSTR("Workpiece Z: "), Printer::lastCmdPos[Z_AXIS] + Printer::coordinateOffset[Z_AXIS]);//#### Info 
             Com::printFLN(PSTR("Machine Z: "), Printer::lastCmdPos[Z_AXIS]);//#### Info 
             ELECTRODE_OPEN;       
             ELECTRODE_OFF;
             Length = NewLength; // actual length of flute is now saved for next tool change
             Printer::setBlockingReceive(false);
             break;
           }
   }       
 #endif    
 
#if defined(SUPPORT_LASER) && SUPPORT_LASER

 if(Printer::mode == PRINTER_MODE_LASER)
 {
  if (com->hasL()) // G30 Lift , probes and lifts Z axis by value of L, use to adjust Z relative to workpiece
           { 
             float startheight = float(com->L);
             Printer::coordinateOffset[Z_AXIS] = Printer::runZProbe(true, true,2, true);
             Printer::coordinateOffset[Z_AXIS] -= (Printer::currentPosition[Z_AXIS]);
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, startheight - Printer::coordinateOffset[Z_AXIS], IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
             Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords to startheight;
             Com::printFLN(PSTR("Workpiece Z: "), Printer::lastCmdPos[Z_AXIS]+ Printer::coordinateOffset[Z_AXIS]);
             Com::printFLN(PSTR("Machine Z: "), Printer::lastCmdPos[Z_AXIS]);
             break;
           }
 }
#endif           
        
        if (com->hasS()) 
           {
           Printer::measureZProbeHeight(com->hasZ() ? com->Z : Printer::currentPosition[Z_AXIS]);
           } 
        else 
           {
            uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
             if(Printer::runZProbe(p & 1,p & 2) == ILLEGAL_Z_PROBE) 
                {
                  GCode::fatalError(PSTR("G30 probing failed!"));
                  break;
                }
          Printer::updateCurrentPosition(p & 1);
        
        }
     }
    break;
  
  default:
     return false;
  }
  return true;
}

//#########################################################################################
//#### MCode addition/replacement
//#########################################################################################


bool Custom_MCode(GCode *com)
{
  
  char buf[20];
  char buf2[20]; 
  uint16_t PIN,Speed,angle ;
  
  switch(com->M) {

//#########################################################################################
//#### modified M3   to handle Gamma correction and Limits
//#########################################################################################

              case 3: // Spindle/laser on
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            if(Printer::mode == PRINTER_MODE_LASER) {
                if(com->hasS())
                {
#if defined(GAMMA_CORRECTION) && (GAMMA_CORRECTION==true)
                      newintens =(com->S);
                      if (Gamma_on)
                      {
                      Outval=pow(newintens/LASER_PWM_MAX,GAMMA)*LASER_PWM_MAX; //Gamma function
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale gamma function and offset to max 
                      Com::printFLN(PSTR("orig:"),(int)newintens);
                      }
                      else
                      {
                      Outval=newintens;
                      LaserDriver::intensity = map((int)Outval,0,LASER_PWM_MAX,LASER_BASE_OFFSET,LASER_LIMIT);// scale offset to max 
                      Com::printFLN(PSTR("Gamma off LV:"),(int)newintens);
                      }
                     
#else
                LaserDriver::intensity = constrain(com->S,0,LASER_PWM_MAX);
#endif          
                }  
                LaserDriver::laserOn = true;
                Com::printFLN(PSTR("LaserOn:"),(int)LaserDriver::intensity);
               
            }
#endif // defined
#if defined(SUPPORT_CNC) && SUPPORT_CNC
            if(Printer::mode == PRINTER_MODE_CNC) {
                Commands::waitUntilEndOfAllMoves();
                CNCDriver::spindleOnCW(com->hasS() ? com->S : CNC_RPM_MAX);
            }
#endif // defined
            break;

            
//############## additional command M6 ################################
//######  !!!Tool change including measuring Tool length!!!
//######        Auto adjusts drills/flutes Z-Position 
//#####################################################################
  
  case 6: 
  #if defined(SUPPORT_CNC) && SUPPORT_CNC && defined(M6_ACK_PIN) && (M6_ACK_PIN>=0)
   
      if (Printer::mode == PRINTER_MODE_CNC) 
      { M6_Message=true;
        if (com->hasT()) 
           {
            uint8_t Toolnumber = (com->T);
            sprintf(UI_Message, "Tool T%d push OK", Toolnumber);
            Com::printFLN(PSTR("change Tool : T "),(int)Toolnumber),0;
           }
         else
           {
            sprintf(UI_Message,"Toolchange ");
            Com::printFLN("Toolchange ");
           }
         CNCDriver::spindleOff();
         Printer::setBlockingReceive(true);
         Commands::waitUntilEndOfAllMoves();
             
                         
         float OrigOff=Printer::realZPosition()+Printer::coordinateOffset[Z_AXIS];
         Com::printFLN(PSTR("orig_off: "),OrigOff);
         float ZchangePos=Printer::lastCmdPos[Z_AXIS]+50;

			   ELECTRODE_CLOSE;
           if ((!measured) || (Length==0))
            {
             sprintf(UI_Message,"measuring Tool");
             Com::printFLN(PSTR("measuring Tool"));
             Com::printFLN(PSTR("on Progress"));
                                       
             Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,ZchangePos, IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]); //lift z 
             Printer::moveToReal(MEAS_XPOS,MEAS_YPOS,IGNORE_COORDINATE, IGNORE_COORDINATE, 3500); // move to probe point
             
             Length = Printer::runZProbe(true, true,4, true);  //probe
             measured = true;
             Commands::waitUntilEndOfAllMoves();
             ELECTRODE_OPEN;
            }
         Com::printFLN(PSTR("actual length: "),Length,3);

         Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZchangePos, IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]); //lift z
         Printer::moveToReal(MEAS_XPOS,MEAS_YPOS, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS]); // move to probe point
          
         sprintf(UI_Message,"ch. Tool,OK to cont.");
         Com::printFLN(PSTR("change Tool now"));
         Com::printFLN(PSTR("OK to continue"));
        
        while (!CONTINUE)

        /*  wait for acknowledge button pressed !! no release by uiaction , no possibility to release from host
         *  for safety reasons to ensure both hands are off and and no second person can accidentally 
         *  start cycle from host.
         *  for shure it is not dummy proof but  high level safe
		 *  keeps fingers happy :-)
         */
        {
         Printer::defaultLoopActions();
        }

         ELECTRODE_CLOSE;  
         sprintf(UI_Message,"measuring new Tool");
         Com::printFLN(PSTR("measuring new Tool"));
         Com::printFLN(PSTR("on Progress"));
                         
         float NewLength = Printer::runZProbe(true, true,4, true);//measure new tool
          
         Commands::waitUntilEndOfAllMoves();
         Com::printFLN(PSTR("new length: "),NewLength,3);//#### Info 
         float diff = NewLength-Length;      // calculate difference       
         Com::printFLN(PSTR("tool offset: "),diff,3);//#### Info 
         Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, ZchangePos-diff, IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]);// lift z
         Printer::moveToReal(Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS]);// move back to workpiece
         Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::lastCmdPos[Z_AXIS] - diff, IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]);// move z to corrected origin
         Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords to startheight;
         Printer::coordinateOffset[Z_AXIS] += diff;//OrigOff;//<-----
         Com::printFLN(PSTR("Workpiece Z: "), Printer::lastCmdPos[Z_AXIS] + Printer::coordinateOffset[Z_AXIS]);//#### Info 
         Com::printFLN(PSTR("Machine Z: "), Printer::lastCmdPos[Z_AXIS]);//#### Info 
         ELECTRODE_OPEN;
         ELECTRODE_OFF;
         Length = NewLength; // actual length of flute is now saved for next tool change
         Printer::setBlockingReceive(false);
         Com::printFLN(PSTR("Toolchange done"));
         sprintf(UI_Message,"Toolchange done");
      }
#else
      Com::printFLN(PSTR(("M6 not handled"));
#endif // defined
        M6_Message = false;
      break;

//############## extended M25 ################################

        case 24: //M24 - Start SD print
            sd.startPrint();
            Com::printFLN(PSTR("(re)started from Host"));  
            break;

//############## extended M25 ################################

        case 25: //M25 - Pause SD print
            sd.pausePrint();
            Com::printFLN(PSTR("paused from Host"));  
            break;      
	  
//############## extended M42 ################################
// allows control of external hardware via M42 command
// MCP1 expander Pins start at pin Nr.150
// so pin 0 of this component is addressed M42 P150 S<value>
// Ext.Pwm Pins start at 200 
// So pin 0 of ext.PWM is addressed M42 P200<value>
//############################################################

        case 42: //M42 -Change pin status via gcode
            if (com->hasP()) 
            {
                int pin_number = com->P;
                for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) 
                {
                    if (pgm_read_byte(&sensitive_pins[i]) == pin_number) 
                    {
                      pin_number = -1;
                      break;
                    }
                }
                if (pin_number > -1) 
                {
                    if(com->hasS()) 
                    {
                        if(com->S >= 0 && com->S <= 4095) 
                        {
                          if(pin_number<=149)
                            {
                            pinMode(pin_number, OUTPUT);
                            digitalWrite(pin_number, (com->S)>>4);
                            analogWrite(pin_number, (com->S)>>4);
                            }
                         
                        #if defined( FEATURE_I2C_MACROS) && FEATURE_I2C_MACROS !=0
                          if((pin_number >= 150)&&(pin_number <=165))
                           {                         
						                MCP1.SetOutput(pin_number - 150);
                            MCP1.Write(pin_number-150,(com->S)>>4);
						               }
                        #endif    
                        
    				          	#if defined( FEATURE_EXT_PWM) && FEATURE_EXT_PWM !=0
                          if((pin_number >= 200)&&(pin_number<=215))
                            {
                           	EXTPWM1.SetPIN(pin_number - 200,(com->S));
						              	}
                        #endif

                            Com::printF(Com::tSetOutputSpace, pin_number);
                            Com::printFLN(Com::tSpaceToSpace,(int)com->S);
                            
                        } else
                            Com::printErrorFLN(PSTR("Illegal S value for M42"));
                    } else 
                        {
                        pinMode(pin_number, INPUT_PULLUP);
                        Com::printF(Com::tSpaceToSpace, pin_number);
                        Com::printFLN(Com::tSpaceIsSpace, digitalRead(pin_number));
                        }
                } else 
                    {
                    Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
                    }
            }
            break;
        
        
//############## extended M114 ####################################
//##### returns Machine coordinates and workpiece coordinates
//#################################################################        
        case 114: // M114
            Com::writeToAll = false;
            Com::printFLN(PSTR("Coords"));
            Commands::printCurrentPosition();
            Com::printFLN(PSTR("Machine Coords"));
            Com::printF(PSTR("Xm:"),Printer::realXPosition());
            Com::printF(PSTR(" Ym:"),Printer::realYPosition());
            Com::printFLN(PSTR(" Zm:"),Printer::realZPosition());
         if(com->hasS() && com->S) {
          Com::printF(PSTR("XS:"),Printer::currentPositionSteps[X_AXIS]);
          Com::printF(PSTR(" YS:"),Printer::currentPositionSteps[Y_AXIS]);
          Com::printFLN(PSTR(" ZS:"),Printer::currentPositionSteps[Z_AXIS]);
          }
            break; 

               
//############## extended M401 ###############################
// allows saving  current position and G92 Offsets
// use : M401 : Standard implementation save to Ram 
//       M401 P : save to EEPROM
//############################################################

            case 401: // M401 Memory position
#if EEPROM_MODE !=0
        if(com->hasP()){
             if(HAL::eprGetFloat(EPR_MEM)!=0){
      
          Commands::waitUntilEndOfAllMoves();
          Printer::updateCurrentPosition(false);
          
          HAL::eprSetFloat(EPR_MEM_X, Printer::realXPosition());
          HAL::eprSetFloat(EPR_MEM_Y, Printer::realYPosition());
          HAL::eprSetFloat(EPR_MEM_Z, Printer::realZPosition());

          HAL::eprSetFloat(EPR_OFF_X,Printer::coordinateOffset[X_AXIS]);
          HAL::eprSetFloat(EPR_OFF_Y,Printer::coordinateOffset[Y_AXIS]);
          HAL::eprSetFloat(EPR_OFF_Z,Printer::coordinateOffset[Z_AXIS]);
          HAL::eprSetFloat(EPR_MEM, 1);
               
          Com::printF(PSTR("Saved to EEPROM X:"),HAL::eprGetFloat(EPR_MEM_X));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(EPR_MEM_Y));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(EPR_MEM_Z)); 
          Com::printF(PSTR("Saved to EEPROM OFFSET X:"),HAL::eprGetFloat(EPR_OFF_X));
          Com::printF(PSTR(" OFF  Y:"),HAL::eprGetFloat(EPR_OFF_Y));
          Com::printFLN(PSTR(" OFF  Z:"),HAL::eprGetFloat(EPR_OFF_Z)); 
       }
       
       else
       Com::printFLN(PSTR("No Memory Position available"));
       }
#else  
       Com::printErrorF(Com::tNoEEPROMSupport);
       
#endif
           
            Printer::MemoryPosition();
            break;

//############## extended M402 ###############################
// allows restoring current position and G92 Offsets
// use : M402 : Standard implementation restore from Ram 
//       M402 P : Restore from EEPROM
//############################################################

            
            case 402: // M402 Go to stored position
#if EEPROM_MODE !=0
       if(com->hasP()){
          Com::printF(PSTR("Read from EEPROM X:"),HAL::eprGetFloat(EPR_MEM_X));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(EPR_MEM_Y));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(EPR_MEM_Z)); 
          Com::printF(PSTR("Read from EEPROM OFF X:"),HAL::eprGetFloat(EPR_OFF_X));
          Com::printF(PSTR(" OFF  Y:"),HAL::eprGetFloat(EPR_OFF_Y));
          Com::printFLN(PSTR(" OFF  Z:"),HAL::eprGetFloat(EPR_OFF_Z)); 
          Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(EPR_OFF_X);
          Printer::coordinateOffset[Y_AXIS]= HAL::eprGetFloat(EPR_OFF_Y);
          Printer::coordinateOffset[Z_AXIS]= HAL::eprGetFloat(EPR_OFF_Z);
          Printer::moveToReal(HAL::eprGetFloat(EPR_MEM_X), HAL::eprGetFloat(EPR_MEM_Y), HAL::eprGetFloat(EPR_MEM_Z), IGNORE_COORDINATE, (com->hasF() ? com->F : Printer::feedrate));
          Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];// update gcode coords to startheight;
          Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];// update gcode coords to startheight;
          Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords to startheight;
          Com::printFLN(PSTR("Positions restored")); 
       }
          if(com->hasD()){
          HAL::eprSetFloat(EPR_MEM, 0);
          Com::printFLN(PSTR("Positions deleted"));
          break;
         }
#else Com::printErrorF(Com::tNoEEPROMSupport);
#endif
         else
            Printer::GoToMemoryPosition(com->hasX(),com->hasY(),com->hasZ(),com->hasE(),(com->hasF() ? com->F : Printer::feedrate));
         break;
			
//########################################################################################################
// Standard M452 just switches to Laser mode
// Here added for Gamma correction and base offset to start at where material begins to change colour
// and Limit value .
// that makes a simple grayscale calibration possible.
// 
// Letters used in M452:
//                       C (Correction)<0/1> 0=off 1 =on
//                       K (K-factor) usually between 0.1 and 2.2
//                       P (base Power) depends on Laser Power and material
//                       L (Limit)  depends on Laser Power and material
//
// example for 12 bit PWM (LASER_PWM MAX 4095): Gamma on , Gamma 1.4 , start value 500 Limit 3000 
// Gcode command for this example is  : M452 C1 K1.4 P500 L3000
//########################################################################################################

       case 452:
#if defined(SUPPORT_LASER) && SUPPORT_LASER
            Commands::waitUntilEndOfAllMoves();
            Printer::mode = PRINTER_MODE_LASER;
            
            if(com->hasC())
            {if((com->C)==0) 
             {Gamma_on=false;
              Com::printFLN(PSTR("GAMMA OFF:"));
             }
             else 
             {Gamma_on=true;
              Com::printFLN(PSTR("GAMMA ON"));
             }
            }
            if(com->hasK())
            {
              GAMMA=(float)(com->K);
              Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1);
            }
            if(com->hasP())
            {
              LASER_BASE_OFFSET=(int)(com->P);
              Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
            }

             if(com->hasL())
            {
              LASER_LIMIT=(int)(com->L);
              Com::printFLN(PSTR("LASER_LIMIT:"),(int)LASER_LIMIT); 
            }
      
            
#endif
            Printer::reportPrinterMode();
            break;            
       
//########################################################################################################
// Standard M453 just switches to CNC mode
// Here added Z-Value to have variable Safe Z Height
// Gcode command for this example is  : M453 Z 5.0
//########################################################################################################

        case 453:
#if defined(SUPPORT_CNC) && SUPPORT_CNC
      Commands::waitUntilEndOfAllMoves();
      Printer::mode = PRINTER_MODE_CNC;
#endif
            if(com->hasZ())
             {
              SAFE_Z = com->Z;
             }
            else
             {
              SAFE_Z= CNC_SAFE_Z;
             }
            Com::printFLN(PSTR("Safe Z:"),(float)SAFE_Z,2);  
            Printer::reportPrinterMode();
            break;
                  
//-----------------------------------------------------------------------------------------------
//Playground for self-defined M-code
// !!!just examples!!! comment out for use
// here : defining m-codes for PWM controller
     
  case 6000:
          if (com->hasP()) 
           {PIN = (com ->P);}
          else 
          {
             UI_STATUS_UPD_RAM("unknown Pin");
             break; 
          }
          if (com->hasS()) 
           {
            Speed = (com->S);
            }
          else Speed=4095;
          
           EXTPWM1.SetPIN(PIN,Speed);
          break;

  case 6001:
   
          if (com->hasP()) 
           {PIN = (com ->P);}
          else
          {
             UI_STATUS_UPD_RAM("unknown Pin");
             break; 
          }
             
           EXTPWM1.SetPIN(PIN,0);
           
          break;
 
 case 6002:
   
          if (com->hasP()) 
           {PIN = (com ->P);}
          else 
          {
              UI_STATUS_UPD_RAM("unknown Pin");
              break; 
          }
           EXTPWM1.SetPIN(PIN,4095);
           
          break;

#ifdef FEATURE_SERVO_ELECTRODE && FEATURE_SERVO_ELECTRODE== true
           
case 7000:
            if (com->hasP()) 
           {PIN = (com ->P);}
         
           if (com->hasS()) 
           {
            angle = (com->S);
           }
          else angle=90;

          EXTPWM1.SetSERVO(PIN,angle);
          HAL::delayMilliseconds(SERVO_TIME);        
          break;

 case 7007:

          ELECTRODE_CLOSE;
          ELECTRODE_OPEN;
          ELECTRODE_OFF;
          break;
#endif  

//end playground
                  
  default:
     return false;
  }
  return true;        
  }

//#########################################################################################
//###  User defined Events
//#########################################################################################

int Custom_Execute(int action,bool allowMoves) 
{
            
  switch(action) {
  
        case UI_ACTION_X_UP001:
        case UI_ACTION_X_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP001) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]/100), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
 
        case UI_ACTION_X_UP01:
        case UI_ACTION_X_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP01) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]/10), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;    
      
        case UI_ACTION_X_UP1:
        case UI_ACTION_X_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[X_AXIS], 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
       
        case UI_ACTION_X_UP10:
        case UI_ACTION_X_DOWN10:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(((action == UI_ACTION_X_UP10) ? 1.0 : -1.0) * (Printer::axisStepsPerMM[X_AXIS]*10), 0, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveX manual")); 
            break;
              
        case UI_ACTION_Y_UP001:
        case UI_ACTION_Y_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP001) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]/100, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP01:
        case UI_ACTION_Y_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP01) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]/10, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP1:
        case UI_ACTION_Y_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS], 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
            
        case UI_ACTION_Y_UP10:
        case UI_ACTION_Y_DOWN10:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, ((action == UI_ACTION_Y_UP10) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Y_AXIS]*10, 0, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveY manual")); 
            break;
                     
        case UI_ACTION_Z_UP001:
        case UI_ACTION_Z_DOWN001:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP001) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS]/100, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual")); 
            break;
            
        case UI_ACTION_Z_UP01:
        case UI_ACTION_Z_DOWN01:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP01) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS]/10, 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual"));           
            break;
            
        case UI_ACTION_Z_UP1:
        case UI_ACTION_Z_DOWN1:
            if(!allowMoves) return action;
            PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((action == UI_ACTION_Z_UP1) ? 1.0 : -1.0) * Printer::axisStepsPerMM[Z_AXIS], 0, JOGRATE, false,false);
            GCode::executeFString(PSTR("M117 moveZ manual"));           
            break;
     
        case UI_ACTION_X_ZERO:
            if(!allowMoves) return UI_ACTION_X_ZERO;
            Printer::coordinateOffset[X_AXIS] -= Printer::currentPosition[X_AXIS];
            break;
            
        case UI_ACTION_Y_ZERO:
            if(!allowMoves) return UI_ACTION_Y_ZERO;
            Printer::coordinateOffset[Y_AXIS] -= Printer::currentPosition[Y_AXIS];
            break;
            
        case UI_ACTION_Z_ZERO:
            if(!allowMoves) return UI_ACTION_Z_ZERO;
            Printer::coordinateOffset[Z_AXIS] -= Printer::currentPosition[Z_AXIS];
            break;

       case UI_ACTION_Z_PROBE:
            if(!allowMoves) return UI_ACTION_Z_PROBE;
            UI_STATUS_UPD_F(Com::translatedF(UI_TEXT_Z_PROBE_ID));
            GCode::executeFString(PSTR("G30")); 
            break;
            
       
}
return 0 ;
}//Custom_Execute


//#########################################################################################
//#### Read buttons from MCP1 external I2C device  
//#### as we read the complete bitmask it´s possible to realize two or more buttons pressed
//#### at the same time in order to realize "shift" Functions etc.
//#########################################################################################

int Custom_CheckSlowKeys()
{
  int action=0;
  char buf[20];
  
#if defined( FEATURE_I2C_MACROS) && FEATURE_I2C_MACROS !=0
{
	 uint16_t buttonval = 0xFFFF-MCP1.ReadPort();
	    
switch (buttonval) {
  
      case 1:       RunMacro(HomeAll);
                    break;
            
      case 2:
                    GCode::executeFString(PSTR("M117 moveZ+ manual")); 
                    HAL::delayMilliseconds(100);
                    GCode::executeFString(PSTR("G91\n G1 Z0.001 F20 \n G90\n")); 
                    break;
   
      case 3:       action = UI_ACTION_Z_UP1 ;
                    break;
               
      case 4:       GCode::executeFString(PSTR("M117 set FFF Mode")); 
                    GCode::executeFString(PSTR("M451")); 
                    HAL::delayMilliseconds(200);
                    GCode::executeFString(PSTR("M117")); 
                    break;        
     
      case 5:       action = UI_ACTION_Z_DOWN01 ;
                    break;  
                  
      case 6:       action = UI_ACTION_Z_DOWN1 ;
                    break;        
      
      case 7:       action = UI_ACTION_Y_UP001 ;
                    break;
            
      case 8:       GCode::executeFString(PSTR("M117 set LASER Mode")); 
                    GCode::executeFString(PSTR("M452")); 
                    HAL::delayMilliseconds(200);
                    GCode::executeFString(PSTR("M117")); 
                    break;
           
      case 9:       action = UI_ACTION_Y_UP1 ;
                    break;

      case 10:
                    action = UI_ACTION_Y_UP1 ;
                    break;
     
      
      case 11:      action = UI_ACTION_Y_DOWN001 ;
                    break;        
      
      case 12:      action = UI_ACTION_Y_DOWN01 ;
                    break;  
                  
      case 13:      action = UI_ACTION_Y_DOWN1 ;
                    break;        
    
      case 14:      action = UI_ACTION_Y_DOWN10 ;
                    break;

      case 15:      action = UI_ACTION_Y_DOWN10 ;
                    break;
            
      case 16:      GCode::executeFString(PSTR("M117 set CNC Mode")); 
                    GCode::executeFString(PSTR("M453")); 
                    HAL::delayMilliseconds(200);
                    break;

      case 32:      if(Gamma_on==false)
                    {
                    Gamma_on=true;
                    GCode::executeFString(PSTR("M117 Gamma on")); 
                    Com::print("Gamma on");
                    HAL::delayMilliseconds(300);
                    }
                    else
                    {
                      Gamma_on=false;
                      GCode::executeFString(PSTR("M117 Gamma off")); 
                      Com::print("Gamma off");
                      HAL::delayMilliseconds(300);
                    } 
                    break;

      case 64:      LASER_BASE_OFFSET+=5; 
                    HAL::delayMilliseconds(200);  
                    Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
                    break;                    
                         
      case 128:     LASER_BASE_OFFSET-=5;  
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LASER_BASE_VALUE:"),(int)LASER_BASE_OFFSET); 
                   
                    break;                   
      case 256:     GAMMA+=0.10;
                    HAL::delayMilliseconds(200);   
                    Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1);
                    break;                    
                         
      case 512:     GAMMA-=0.10; 
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("GAMMA_VALUE:"),(float)GAMMA,1); 
                    break;                   
      
      case 1024:    LASER_LIMIT-=50;
                    if(LASER_LIMIT<LASER_PWM_MAX/4) LASER_LIMIT=LASER_PWM_MAX/4;
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LIMIT:"),(int)LASER_LIMIT); 
                    break;

      case 2048:    LASER_LIMIT+=50;
                    if(LASER_LIMIT>LASER_PWM_MAX) LASER_LIMIT=LASER_PWM_MAX;
                    HAL::delayMilliseconds(200); 
                    Com::printFLN(PSTR("LIMIT:"),(int)LASER_LIMIT); 
                    break;
     
      default: 
                    break;
   }

   return(action);
 }
#endif //macros

}//SLOWKEYS


bool Custom_SD_Start_Pause(bool intern)
{ if(intern) 
    {
        Commands::waitUntilEndOfAllBuffers();
        //sdmode = 0; // why ?
        Printer::MemoryPosition();
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                            Printer::memoryE - RETRACT_ON_PAUSE,
                            Printer::maxFeedrate[E_AXIS] / 2);
      
#ifdef CNC_SAFE_Z
      if(Printer::mode == PRINTER_MODE_CNC) {
      Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,  SAFE_Z - Printer::coordinateOffset[Z_AXIS], IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]/2);
      Com::printFLN(PSTR("Safe Z Pause:"),(float)SAFE_Z,2);  
      }
#endif
      if(Printer::mode != PRINTER_MODE_CNC) {
     
#if DRIVE_SYSTEM == DELTA
      Printer::moveToReal(0, 0.9 * EEPROM::deltaMaxRadius(), IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS]);
#else
      Printer::moveToReal(Printer::xMin, Printer::yMin + Printer::yLength, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS]);
#endif
      }
        Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];
        Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];
        Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];
        GCode::executeFString(PSTR(PAUSE_START_COMMANDS));
   }
 else // extern
 /* {    
     
    
      if(Printer::mode == PRINTER_MODE_CNC) {
        Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE,  SAFE_Z - Printer::coordinateOffset[Z_AXIS], IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS]/2);
        Com::printFLN(PSTR("Safe Z Pause from Host:"),(float)SAFE_Z,2);  
      }

  }*/
  return false;

}

bool Custom_SD_End_Pause(bool intern)
{
   if(intern)Com::printFLN(PSTR("done"));  
}

bool Custom_SD_Start_Continue(bool intern)
{
   if(intern)Com::printFLN(PSTR("unpause"));  
  
}

bool Custom_SD_End_Continue(bool intern)
{
   if(intern)Com::printFLN(PSTR("done"));  
}

//#########################################################################################
//##### Feature Joystick, just to move during setup
//##### useful for CNC/LASER to Offset position
//##### uses Analog inputs and Analog Joyticks
//#########################################################################################

//!!!!!!!!!!!!!!!!!!! faulty , does not work with analogRead !!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!! faulty , does not work with analogRead !!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!! will be updated soon , needs modification !!!!!!!!!!!!!!!!!!!!!!!
		    
void RAyWB_CheckJoystick(uint16_t &action) // assigned to slow action
{
//button assignment x-axis

struct {uint16_t min;uint16_t max;uint16_t action;}
    keys_x[] = {
               {   0,   300, UI_ACTION_X_DOWN10       },    // down 10 mm steps
               {   310,   700, UI_ACTION_X_DOWN1     },    // down 1 mm steps
               {   710,   100, UI_ACTION_X_DOWN01   },    // down 0.1 mm steps
               {   1010,  2000, UI_ACTION_X_DOWN001   },    // down 0.01 mm steps
    
               {  2200,  2350, UI_ACTION_X_UP001       },    // up 0.01mm steps
               {  2370,  2500, UI_ACTION_X_UP01        },    // up 0.1 mm steps
               {  2510,  2650, UI_ACTION_X_UP1         },    // up 1mm steps
               {  2660,  2750, UI_ACTION_X_UP10        },    // up 10 mm steps
           
               };
 const uint8_t numOfKeys_x = sizeof(keys_x) / sizeof(keys_x[0]);
 uint16_t adc_x= analogRead(12)<<2;
  
  if ((adc_x<2000)|| ((adc_x>2200)&&(adc_x<3000))) 
    {
    for (int8_t ix = 0; ix < numOfKeys_x; ++ix) 
      {
      if ((adc_x > keys_x[ix].min) && (adc_x< keys_x[ix].max)) 
       {
        action = keys_x[ix].action;
        return ;
        }
     }
   }


//button assignment y-axis

struct {uint16_t min;uint16_t max;uint16_t action;} 
    keys_y[] = {
               {   0,   800, UI_ACTION_Y_UP10        },    // Up 10 mm Steps
               {   810, 1200, UI_ACTION_Y_UP1        },    // Up 1mm Steps
               {   1210,1800, UI_ACTION_Y_UP01       },    // Up 0.1 mm Steps
               {   1810,2000, UI_ACTION_Y_UP001      },    // Up 0.01 mm steps
    
               {  2200,  2350, UI_ACTION_Y_DOWN001   },    // Down 0.01mm Steps
               {  2370,  2500, UI_ACTION_Y_DOWN01    },    // Down 0.1mm Steps
               {  2510,  2650, UI_ACTION_Y_DOWN1     },    // Down 1mm Steps
               {  2660,  3000, UI_ACTION_Y_DOWN10    },    // Down 10 mm Steps
        
               };
                
       const uint8_t numOfKeys_y = sizeof(keys_y) / sizeof(keys_y[0]);
       uint16_t adc_y= analogRead(13)<<2;

  if ((adc_y<2000)|| ((adc_y>2200)&&(adc_y<3000)))
    {
    for (int8_t iy = 0; iy < numOfKeys_y; ++iy)
      {
      if ((adc_y > keys_y[iy].min) && (adc_y < keys_y[iy].max)) 
        {
        action = keys_y[iy].action;
        return ;
        }
      }
    }

// Button assignment Z-Axis
  
struct {uint16_t min;uint16_t max;uint16_t action;} 
    keys_z[] = {
               {   0, 1200, UI_ACTION_Z_UP1        },    // Up 1mm steps
               {   1210,1800, UI_ACTION_Z_UP01       },    // Up 0.1 mm steps
               {   1810,2000, UI_ACTION_Z_UP001      },    // Up 0.01 mm steps
         
               {  2200,  2350, UI_ACTION_Z_DOWN001   },    //Down 0.01 mm steps 
               {  2370,  2500, UI_ACTION_Z_DOWN01    },    //Down 0.1 mm steps
               {  2510,  3000, UI_ACTION_Z_DOWN1     },    // Down 1mm steps
               };
                
          const uint8_t numOfKeys_z = sizeof(keys_z) / sizeof(keys_z[0]);
          uint16_t adc_z= analogRead(14)<<2;

  if ((adc_z<2000)|| ((adc_z>2200)&&(adc_z<3000)))
    {
    for (int8_t iz = 0; iz < numOfKeys_z; ++iz) 
      {
      if ((adc_z > keys_z[iz].min) && (adc_z < keys_z[iz].max)) 
       {
       action = keys_z[iz].action;
       return  ; // no need for that  
       }
     }
    }
 
}// end joystick 

#endif

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
#ifndef CustomEvents_H
#define CustomEvents_H

//special stuff RAyWB

#define FEATURE_I2C_MACROS true
#define FEATURE_EXT_PWM true  

#define FEATURE_PWM_LASER true//false
#define FEATURE_ANALOG_LASER false//true
#define GAMMA_CORRECTION true

#define FEATURE_SERVO_ELECTRODE true//false

#ifdef FEATURE_SERVO_ELECTRODE && FEATURE_SERVO_ELECTRODE== true
#define SERVO_ELECTRODE_PIN 1
#define SERVO_TIME 500
#define ELECTRODE_CLOSE {EXTPWM1.SetSERVO(SERVO_ELECTRODE_PIN,2); HAL::delayMilliseconds(SERVO_TIME);}
#define ELECTRODE_OPEN {EXTPWM1.SetSERVO(SERVO_ELECTRODE_PIN,90); HAL::delayMilliseconds(SERVO_TIME);}
#define ELECTRODE_OFF { HAL::delayMilliseconds(SERVO_TIME);EXTPWM1.SetPWM(SERVO_ELECTRODE_PIN, 0,0);EXTPWM1.SetFREQ(PWM_FREQU);}
#else
#define ELECTRODE_CLOSE {Com::printFLN(PSTR("no Servo Electrode ");}
#define ELECTRODE_OPEN {Com::printFLN(PSTR("no Servo Electrode ");}
#define ELECTRODE_OFF {Com::printFLN(PSTR("no Servo Electrode ");}
#endif



#define EXT_LASER_PIN 0 // PIN Number from PCA9685  
#define EPWM_OE 8  // Output enable for PCA9685, not needed for operation but useful to get defined Startup condition
#define PWM_FREQU 1100


#define FEATURE_JOYSTICK_MOVE false//true  //!!!! activate only if joystick connected otherwise there is permanent moving
#define JOGRATE 500



#define HOME_BUTTON         25  //AD0 printedA7
#define ORIG_M6_ACK_PIN     16
//#define ORIG_SAFETY_PIN     66 

#define M6_ACK_PIN ORIG_M6_ACK_PIN
#define M6_ACK_PIN_PULLUP 1
#define CONTINUE (!READ(M6_ACK_PIN))
#define MEAS_XPOS -10
#define MEAS_YPOS 195

#define EPR_MEM_X 1500
#define EPR_MEM_Y 1504
#define EPR_MEM_Z 1508
#define EPR_OFF_X 1512
#define EPR_OFF_Y 1516
#define EPR_OFF_Z 1520


//############### MCP23017 ########################

#define MCP23017_IODIR 0x00
#define MCP23017_IPOL 0x2
#define MCP23017_GPPU 0x0C
#define MCP23017_GPIO 0x12

class MCP23017
{
	public:
	MCP23017(uint8_t address);
	void Init(void);
	void SetInput(uint8_t pin);
	void SetOutput(uint8_t pin);
	void Write(uint8_t pin, bool val);
    bool Read(uint8_t pin);
    uint16_t ReadPort();
    void WritePort(uint16_t data);
  
  private:

  uint8_t i2c_add;
//    void writeRegister(uint16_t regaddress, byte val);
	void writeRegister(uint16_t regaddress, uint16_t val);
	uint16_t readRegister(uint16_t regaddress);
	uint16_t _GPIO, _IODIR, _GPPU;
	
    };
//end MCP23017


//############### MCP4725 ########################

#define MCP4725_COMMAND 0x40

class MCP4725
{
  public:
  MCP4725(uint8_t address);
  void Init(void);
  void WriteOutput(uint16_t value);
  
  private:
  uint8_t i2c_add;
  
    };
//end MCP4725



//############### PCA9685 ########################

class PCAPWM
{
    public:
    PCAPWM(uint8_t address);
    void Init(void);
    void SetFREQ(float freq);
    void SetPWM(uint8_t num, uint16_t on, uint16_t off);
    void SetPIN(uint8_t num, uint16_t val, bool invert=false);
    void SetSERVO(uint8_t num,float degree);

    private:
    uint8_t PCA_addr;
};

// end PCA9685

extern bool SetLaser(uint16_t newIntensity);

extern void CustomMessage(); //Display Custom Message 

extern void Custom_100MS();
extern void Custom_500MS();
extern void Custom_Init();
extern void Custom_Init_Early();
extern int  Custom_Execute(int action,bool allowMoves);
extern bool Custom_GCode(GCode *com);
extern bool Custom_MCode(GCode *com);
extern bool Custom_SD_Start_Pause(bool intern);
extern bool Custom_SD_End_Pause(bool intern);
extern bool Custom_SD_Start_Continue(bool intern);
extern bool Custom_SD_End_Continue(bool intern);

extern int  Custom_CheckSlowKeys();
extern void RAyWB_CheckJoystick(int &action) ;
extern void RAyWB_CheckJoystick2() ;


/*  under construction
// replace the pins which go to external PWM
// and replace by Single PWM PIN which is NOT used
#undef HEATER_0_PIN
#undef HEATER_1_PIN
#undef HEATER_2_PIN
#undef HEATER_3_PIN
#undef FAN_PIN
#undef FAN2_PIN
//#undef FAN_BOARD_PIN

#define HEATER_0_PIN 9
#define HEATER_1_PIN 9
#define HEATER_2_PIN 9
#define HEATER_3_PIN 9
#define FAN_PIN 9
#define FAN2_PIN 9
//#define FAN_BOARD_PIN 9
*/

// replace original stuff
#undef EVENT_UI_EXECUTE
#define EVENT_UI_EXECUTE(action,allowMoves) Custom_Execute(action,allowMoves)

#undef EVENT_TIMER_100MS
#define EVENT_TIMER_100MS {Custom_100MS();}

#undef EVENT_TIMER_500MS
#define EVENT_TIMER_500MS {Custom_500MS();}

#undef EVENT_INITIALIZE 
#define EVENT_INITIALIZE {Custom_Init();}

#undef EVENT_INITIALIZE_EARLY
#define EVENT_INITIALIZE_EARLY {Custom_Init_Early();}

#undef EVENT_CHECK_SLOW_KEYS(action) {}
#define EVENT_CHECK_SLOW_KEYS(action) {action=Custom_CheckSlowKeys();}

#undef EVENT_CHECK_FAST_KEYS(action) {}
#define EVENT_CHECK_FAST_KEYS(action) {}//RAyWB_CheckJoystick(action);}

//G and M code replacements and /or additional
#undef EVENT_UNHANDLED_G_CODE(c) 
#define EVENT_UNHANDLED_G_CODE(c) Custom_GCode(c) 

#undef EVENT_UNHANDLED_M_CODE(c) 
#define EVENT_UNHANDLED_M_CODE(c) Custom_MCode(c)

#undef  EVENT_SD_PAUSE_START(intern) 
#define EVENT_SD_PAUSE_START(intern) Custom_SD_Start_Pause(intern)

#undef  EVENT_SD_PAUSE_END(intern) {}
#define EVENT_SD_PAUSE_END(intern) {Custom_SD_End_Pause(intern);}

#undef  EVENT_SD_CONTINUE_START(intern) {}
#define EVENT_SD_CONTINUE_START(intern) {Custom_SD_Start_Continue(intern);}

#undef  EVENT_SD_CONTINUE_END(intern) {}
#define EVENT_SD_CONTINUE_END(intern) {Custom_SD_End_Continue(intern);}

#undef  EVENT_SD_STOP_START {}
#define EVENT_SD_STOP_START {}

#undef EVENT_SD_STOP_END {}
#define EVENT_SD_STOP_END {}


// replacement Laser driver

#undef EVENT_INITIALIZE_LASER
#undef EVENT_SET_LASER(newIntensity)
//#undef LASER_PWM_MAX

#define EVENT_INITIALIZE_LASER  false
#define EVENT_SET_LASER(newIntensity)  SetLaser(newIntensity)
//#define LASER_PWM_MAX 4095 // for 12Bit precision on external Hardware
/*

#undef EVENT_INITIALIZE_CNC
#define EVENT_INITIALIZE_CNC true

#undef EVENT_SPINDLE_OFF
#define EVENT_SPINDLE_OFF true

#undef EVENT_SPINDLE_CW(rpm)
#define EVENT_SPINDLE_CW(rpm)true

#undef EVENT_SPINDLE_CCW(rpm)
#define EVENT_SPINDLE_CCW(rpm)true
*/

#define RunMacro(macro) GCode::executeFString(PSTR(macro))

// New menu actions
// repeatable actions 700-999 

#define UI_ACTION_X_UP1                  700
#define UI_ACTION_X_DOWN1                701
#define UI_ACTION_X_UP01                 702
#define UI_ACTION_X_DOWN01               703
#define UI_ACTION_X_UP001                704
#define UI_ACTION_X_DOWN001              705
#define UI_ACTION_X_UP10                 706
#define UI_ACTION_X_DOWN10               707

#define UI_ACTION_Y_UP1                  708
#define UI_ACTION_Y_DOWN1                709
#define UI_ACTION_Y_UP01                 710
#define UI_ACTION_Y_DOWN01               711
#define UI_ACTION_Y_UP001                712
#define UI_ACTION_Y_DOWN001              713
#define UI_ACTION_Y_UP10                 714
#define UI_ACTION_Y_DOWN10               715


#define UI_ACTION_Z_UP1                  716
#define UI_ACTION_Z_DOWN1                717
#define UI_ACTION_Z_UP01                 718
#define UI_ACTION_Z_DOWN01               719
#define UI_ACTION_Z_UP001                720
#define UI_ACTION_Z_DOWN001              721


// single actions 1500-2500

#define UI_ACTION_X_ZERO                 1500
#define UI_ACTION_Y_ZERO                 1501
#define UI_ACTION_Z_ZERO                 1502

#define UI_ACTION_Z_PROBE                1503


//some macros

#define HomeX   "G28 x\nM117 Home X"
#define HomeY   "G28 y\nM117 Home Y " 
#define HomeZ   "G28 z\nM117 Home Z "  
#define HomeAll "M117 Home all\nG28"  
#define ZeroX   "G92 x0\nM117 Reset X coord "
#define ZeroY   "G92 y0\nM117 Reset Y coord "
#define ZeroZ   "G92 z0\nM117 Reset Z coord "
#define ZProbe  "G30 "
#define Clear   "M117                       "



#endif

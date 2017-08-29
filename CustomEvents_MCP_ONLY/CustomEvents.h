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


extern void Custom_Init_Early();
extern int  Custom_Execute(int action,bool allowMoves);
extern int  Custom_CheckSlowKeys();


// replace original stuff
#undef EVENT_UI_EXECUTE
#define EVENT_UI_EXECUTE(action,allowMoves) Custom_Execute(action,allowMoves)

#undef EVENT_INITIALIZE_EARLY
#define EVENT_INITIALIZE_EARLY {Custom_Init_Early();}

#undef EVENT_CHECK_SLOW_KEYS
#define EVENT_CHECK_SLOW_KEYS(action) {action=Custom_CheckSlowKeys();}


#define UI_ACTION_IDENT01              1301
#define UI_ACTION_IDENT02              1302
#define UI_ACTION_IDENT03              1303
#define UI_ACTION_IDENT04              1304
#define UI_ACTION_IDENT05              1305
#define UI_ACTION_IDENT06              1306
#define UI_ACTION_IDENT07              1307
#define UI_ACTION_IDENT08              1308 
#define UI_ACTION_IDENT09              1309
#define UI_ACTION_IDENT10              1310
#define UI_ACTION_IDENT11              1311
#define UI_ACTION_IDENT12              1312
#define UI_ACTION_IDENT13              1313
#define UI_ACTION_IDENT14              1314
#define UI_ACTION_IDENT15              1315
#define UI_ACTION_IDENT16              1316


#define UI_ACTION_IDENTM01              1341
#define UI_ACTION_IDENTM02              1342
#define UI_ACTION_IDENTM03              1343
#define UI_ACTION_IDENTM04              1344
#define UI_ACTION_IDENTM05              1345
#define UI_ACTION_IDENTM06              1346
#define UI_ACTION_IDENTM07              1327
#define UI_ACTION_IDENTM08              1328 
#define UI_ACTION_IDENTM09              1329
#define UI_ACTION_IDENTM10              1330
#define UI_ACTION_IDENTM11              1331
#define UI_ACTION_IDENTM12              1332
#define UI_ACTION_IDENTM13              1333
#define UI_ACTION_IDENTM14              1334
#define UI_ACTION_IDENTM15              1335
#define UI_ACTION_IDENTM16              1336


#endif

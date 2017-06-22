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

    author of this additional File : ASH / Elias Taalab     thanks to RAyWB

*/






#ifndef CustomEventsImpl_H
#define CustomEventsImpl_H

//#########################################################################################
//#### GCode addition/replacement
//#########################################################################################

 bool Custom_GCode(GCode *com)
 {

    switch(com->G) {
        //XXXX is the number you want to generate
//		case XXXX:

//		break;

  default:
     return false;
  }
  return true;
}

//#########################################################################################
//#### MCode addition/replacement
//#########################################################################################



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Custom_MCode(GCode *com)
{

  switch(com->M) {


    case 480:   // M480
      #if EEPROM_MODE !=0
       if(com->hasP()){
        switch(com->P){

		     case 1:
          Commands::waitUntilEndOfAllMoves();
          Printer::updateCurrentPosition(false);

          HAL::eprSetFloat(epr_memoryX1,Printer::realXPosition());                   //store position from ram to eeprom
          HAL::eprSetFloat(epr_memoryY1,Printer::realYPosition());
          HAL::eprSetFloat(epr_memoryZ1,Printer::realZPosition());

          HAL::eprSetFloat(epr_offsetX1,Printer::coordinateOffset[X_AXIS]);                   //store offset values from ram to eeprom
          HAL::eprSetFloat(epr_offsetY1,Printer::coordinateOffset[Y_AXIS]);
          HAL::eprSetFloat(epr_offsetZ1,Printer::coordinateOffset[Z_AXIS]);

          Com::printF(PSTR("EEPROM Saved to position 1: X:"),HAL::eprGetFloat(epr_memoryX1));     //print some messages
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY1));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ1));

          Com::printF(PSTR("Saved Offset 1: X:"),HAL::eprGetFloat(epr_offsetX1));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY1));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ1));

         break;

         case 2:
          Commands::waitUntilEndOfAllMoves();
          Printer::updateCurrentPosition(false);

          HAL::eprSetFloat(epr_memoryX2,Printer::realXPosition());                   //store position from ram to eeprom
          HAL::eprSetFloat(epr_memoryY2,Printer::realYPosition());
          HAL::eprSetFloat(epr_memoryZ2,Printer::realZPosition());

          HAL::eprSetFloat(epr_offsetX2,Printer::coordinateOffset[X_AXIS]);                   //store offset values from ram to eeprom
          HAL::eprSetFloat(epr_offsetY2,Printer::coordinateOffset[Y_AXIS]);
          HAL::eprSetFloat(epr_offsetZ2,Printer::coordinateOffset[Z_AXIS]);


          Com::printF(PSTR("EEPROM Saved to position 2: X:"),HAL::eprGetFloat(epr_memoryX2));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY2));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ2));

          Com::printF(PSTR("Saved Offset 2: X:"),HAL::eprGetFloat(epr_offsetX2));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY2));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ2));

         break;

         case 3:
          Commands::waitUntilEndOfAllMoves();
          Printer::updateCurrentPosition(false);

          HAL::eprSetFloat(epr_memoryX3,Printer::realXPosition());                   //store position from ram to eeprom
          HAL::eprSetFloat(epr_memoryY3,Printer::realYPosition());
          HAL::eprSetFloat(epr_memoryZ3,Printer::realZPosition());

          HAL::eprSetFloat(epr_offsetX3,Printer::coordinateOffset[X_AXIS]);                   //store offset values from ram to eeprom
          HAL::eprSetFloat(epr_offsetY3,Printer::coordinateOffset[Y_AXIS]);
          HAL::eprSetFloat(epr_offsetZ3,Printer::coordinateOffset[Z_AXIS]);


          Com::printF(PSTR("EEPROM Saved to position 3: X:"),HAL::eprGetFloat(epr_memoryX3));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY3));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ3));

          Com::printF(PSTR("Saved Offset 3: X:"),HAL::eprGetFloat(epr_offsetX3));
          Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY3));
          Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ3));

         break;

              }
         }
         #else 
         Com::printErrorF(Com::tNoEEPROMSupport);
         #endif

    break; //end  480

//  ---------------------------------------------------------------------------------------------------------

     case 481:  //M481
      #if EEPROM_MODE !=0
       if(com->hasP()){
        switch(com->P){
         case 1:
            Com::printF(PSTR("Position 1: X:"),HAL::eprGetFloat(epr_memoryX1));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY1));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ1));

            Com::printF(PSTR("Offset 1: X:"),HAL::eprGetFloat(epr_offsetX1));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY1));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ1));

         break;

         case 2:
            Com::printF(PSTR("Position 2: X:"),HAL::eprGetFloat(epr_memoryX2));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY2));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ2));

            Com::printF(PSTR("Offset 2: X:"),HAL::eprGetFloat(epr_offsetX2));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY2));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ2));


         break;

         case 3:
            Com::printF(PSTR("Position 3: X:"),HAL::eprGetFloat(epr_memoryX3));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_memoryY3));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_memoryZ3));

            Com::printF(PSTR("Offset 3: X:"),HAL::eprGetFloat(epr_offsetX3));
            Com::printF(PSTR("  Y:"),HAL::eprGetFloat(epr_offsetY3));
            Com::printFLN(PSTR("  Z:"),HAL::eprGetFloat(epr_offsetZ3));

         break;

            }
         }
         #else 
         Com::printErrorF(Com::tNoEEPROMSupport);
         #endif

    break;  //end 481

//  -----------------------------------------------------------------------------------------------------------------
    case 482:   //M482
    #if EEPROM_MODE !=0
       if(com->hasP()){
        switch(com->P){
         case 1:

            Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_offsetX1);                       //restore offset values
            Printer::coordinateOffset[Y_AXIS]= HAL::eprGetFloat(epr_offsetY1);
            Printer::coordinateOffset[Z_AXIS]= HAL::eprGetFloat(epr_offsetZ1);

            Printer::moveToReal(HAL::eprGetFloat(epr_memoryX1)                                      //move to position stored in eeprom
                                ,HAL::eprGetFloat(epr_memoryY1)
                                ,HAL::eprGetFloat(epr_memoryZ1)
                                ,IGNORE_COORDINATE
                                ,(com->hasF() ? com->F : Printer::feedrate));
            Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];// update gcode coords 
            Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];// update gcode coords 
            Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords 

         break;

         case 2:

            Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_offsetX2);                       //restore offset values
            Printer::coordinateOffset[Y_AXIS]= HAL::eprGetFloat(epr_offsetY2);
            Printer::coordinateOffset[Z_AXIS]= HAL::eprGetFloat(epr_offsetZ2);

            Printer::moveToReal(HAL::eprGetFloat(epr_memoryX2)
                                ,HAL::eprGetFloat(epr_memoryY2)
                                ,HAL::eprGetFloat(epr_memoryZ2)
                                ,IGNORE_COORDINATE
                                ,(com->hasF() ? com->F : Printer::feedrate));
            Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];// update gcode coords 
            Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];// update gcode coords 
            Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords 
                      

         break;

         case 3:

            Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_offsetX3);                       //restore offset values
            Printer::coordinateOffset[Y_AXIS]= HAL::eprGetFloat(epr_offsetY3);
            Printer::coordinateOffset[Z_AXIS]= HAL::eprGetFloat(epr_offsetZ3);

            Printer::moveToReal(HAL::eprGetFloat(epr_memoryX3)
                                ,HAL::eprGetFloat(epr_memoryY3)
                                ,HAL::eprGetFloat(epr_memoryZ3)
                                ,IGNORE_COORDINATE
                                ,(com->hasF() ? com->F : Printer::feedrate));
            Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];// update gcode coords 
            Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];// update gcode coords 
            Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords 
                    

         break;

            }
         }
         #else 
         Com::printErrorF(Com::tNoEEPROMSupport);
         #endif
      break;  //end 482
//-----------------------------------------------------------------------------------------------------------------------




  default:
     return false;
  }


  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Custom_100MS(){
  if(HAL::eprGetByte(epr_EmergencyByte)==0)     //##ray
  {Emergency_PowerOff_loop();}
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Emergency_PowerOff_loop()                               // should run each 100MS
{if (digitalRead(EmergencyOff_PIN)==Emergency_Trigger_on) {  //if power off triggerd (pin set to low)
  if (sd.sdactive) {                                 //if printing from sd
//    Printer::kill(false);                //kill all ("false" mean kill all not only steppers)
  
    HAL::eprSetFloat(epr_BackupFeedrate,Printer::feedrate);                          //backup all variables
    HAL::eprSetInt32(epr_BackupSDposition,sd.sdpos);
    HAL::eprSetFloat(epr_BackupExId,Extruder::current->id);
    HAL::eprSetFloat(epr_BackupTemp,Extruder::current->tempControl.targetTemperatureC);
    HAL::eprSetFloat(epr_BackupTempB,heatedBedController.targetTemperatureC);


    HAL::eprSetFloat(epr_Backup_OffsetX,Printer::coordinateOffset[X_AXIS]);                   //backup offset values to eeprom
    HAL::eprSetFloat(epr_Backup_OffsetY,Printer::coordinateOffset[Y_AXIS]);
    HAL::eprSetFloat(epr_Backup_OffsetZ,Printer::coordinateOffset[Z_AXIS]);

    HAL::eprSetFloat(epr_Backup_MEMX,Printer::lastCmdPos[X_AXIS]);                   //backup offset values to eeprom
    HAL::eprSetFloat(epr_Backup_MEMY,Printer::lastCmdPos[Y_AXIS]);
    HAL::eprSetFloat(epr_Backup_MEMZ,Printer::lastCmdPos[Z_AXIS]);
   
    for(int i=0;i<20;i++)                                   //backup selected sd filename
    {HAL::eprSetByte(epr_BackupSDfilename+i,Printer::printName[i]);}

    HAL::eprSetByte(epr_EmergencyByte,1);     //set emergency byte to 1
    sd.stopPrint(); 
 //   Printer::kill(false);                //kill all ("false" mean kill all not only steppers)

    }
  else HAL::eprSetByte(epr_EmergencyByte,0);           // if the printer is not printing just set emergency byte to 0
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void Custom_INTIALIZE(){
  pinMode(EmergencyOff_PIN,INPUT);
  Emergency_Restore_IfNeeded();

}


//////////////////////////////////////////////////////////////////////////////////////////////////////
void Emergency_Restore_IfNeeded(){                     //restore print if needed //should run on start

  if(HAL::eprGetByte(epr_EmergencyByte)==1){   //if emergency powered off detected

      sd.mount();
    //  sd.initsd();
    
      for(int i=0;i<20;i++)                     //restore sd file name
      {Printer::printName[i]=HAL::eprGetByte(epr_BackupSDfilename+i);}

      sd.setIndex(HAL::eprGetInt32(epr_BackupSDposition));                                      // restore other variables
     
      GCode::executeFString(PSTR("G28"));                                   //first home the printer

      Printer::feedrate=HAL::eprGetFloat(epr_BackupFeedrate);
      Extruder::current->id=HAL::eprGetFloat(epr_BackupExId);
      previousMillisCmd=HAL::timeInMilliseconds();

      Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_Backup_OffsetX);                       //restore offset values
      Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_Backup_OffsetY);
      Printer::coordinateOffset[X_AXIS]= HAL::eprGetFloat(epr_Backup_OffsetZ);


      Extruder::setTemperatureForExtruder(HAL::eprGetFloat(epr_BackupTemp),HAL::eprGetFloat(epr_BackupExId),0,true);
      Extruder::setHeatedBedTemperature(HAL::eprGetFloat(epr_BackupTempB),0);
    
      HAL::eprSetByte(epr_EmergencyByte,0);                            // set emergency byte to 0

      Printer::setMenuMode(MENU_MODE_PAUSED+MENU_MODE_SD_PRINTING,true);    //set the printer as it is printing and paused to enable the continue option in the menu
     
      Printer::moveToReal(HAL::eprGetFloat(epr_Backup_MEMX), HAL::eprGetFloat(epr_Backup_MEMY), HAL::eprGetFloat(epr_Backup_MEMZ), IGNORE_COORDINATE, Printer::feedrate);
      Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];// update gcode coords 
      Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];// update gcode coords 
      Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];// update gcode coords 

  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////




#endif

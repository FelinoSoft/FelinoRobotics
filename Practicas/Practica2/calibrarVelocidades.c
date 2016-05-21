/*--------------------------------------------------------------------------------------------------------*\
|*                                        ROBOT CONFIGURATION                                             *|
|*                                                                                                        *|
|*    MOTORS & SENSORS:                                                                                   *|
|*    [I/O Port]              [Name]              [Type]              [Description]                       *|
|*    Port C                  motorC              NXT                 Right motor                         *|
|*    Port A                  motorA              NXT                 Left motor                          *|
\*---------------------------------------------------------------------------------------------------4246-*/

///////////////////////////////////////////////////////////////////////////////////////////
//
//       Test NXT File I/O Functions to STORE motor speed tests
//
// The NXT has a file system capable of storing up to 64 files in it's flash memory. The
// files are preserved across power off periods on the NXT.
//
// Filenames are limited to 15 characters on the NXT. There is a 3 character file extension.
//
//    MOTORS & SENSORS:
//    [I/O Port]              [Name]              [Type]              [Description]
//    Port C                  motorC              NXT                 Right motor
//    Port A                  motorA              NXT                 Left motor
///////////////////////////////////////////////////////////////////////////////////////////

#pragma platform(NXT)     // This program only works on NXT -- generate error for other platforms

const string sFileName = "prueba.txt";

TFileIOResult nIoResult;
TFileHandle hFileHandle;
int nFileSize 					= 200; //1 byte each char...


{

  string sString, sString2;
  int   motorPower = 0;
  int i = 0;
  int j = 0;

  nxtDisplayTextLine(0, "Empezando ...");     /* Display the main string  */
  wait1Msec(1000);                            // Wait for 1 seconds (look at global variables now!)

  CloseAllHandles(nIoResult);
	wait1Msec(500);
	PlaySoundFile("Woops.rso");
	wait1Msec(3000);


	//
	// Deletes the file if it already exists
	//
	Delete(sFileName, nIoResult);
	hFileHandle = 0;

	OpenWrite(  hFileHandle, nIoResult, sFileName, nFileSize);

   nMotorEncoder[motorC] = 0;  //clear the LEGO encoders in motors A and C
   nMotorEncoder[motorA] = 0;

   int iter = 5;

   //A: rueda izq
   //C: rueda derecha

   for(i = 1; i<= 10; i++)
   {
      motorPower = i*10;
      StringFormat(sString, "%d ", motorPower);
      nxtDisplayTextLine(1, "Potencia %d", motorPower);

      int motorAvel = 0;
      int motorCvel = 0;
      for(j = 1; iter; j++){

	      motor[motorA] = motorPower;    // Motor A is run at a i*10 power level.
	      motor[motorC] = motorPower;    // Motor C is run at a i*10 power level.
	      wait1Msec(1000);       // The program waits 4000 milliseconds before running further code.

	      motorAvel = motorAvel + nMotorEncoder[motorA];
	      motorCvel = motorCvel + nMotorEncoder[motorC];

	      motor[motorA] = 0;
	      motor[motorC] = 0;
	      nMotorEncoder[motorA] = 0;
	      nMotorEncoder[motorC] = 0;
	      wait1Msec(1000);
	  }

      StringFormat(sString2, "%d %d \n", motorCvel/iter, motorAvel/iter);
      strcat(sString, sString2); // concatenate string2 into string1

      WriteText(hFileHandle, nIoResult, sString);
   }
   
  motor[motorA] = 0;    // Motor A is run at a i*10 power level.
  motor[motorC] = 0;    // Motor C is run at a i*10 power level.

  Close(hFileHandle, nIoResult);

  eraseDisplay();
	nxtDisplayBigStringAt(0, 15, "FIN.");
	wait1Msec(3000);

  return;
}

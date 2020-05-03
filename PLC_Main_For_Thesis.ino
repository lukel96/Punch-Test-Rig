/*
  PUNCH TEST RIG MAIN OPERATOINAL PROGRAM

  This program performs all the operations used to control the rig including
  control of the hydraulic valves, control of the HPU motor, monitoring of
  hydraulic system pressure, monitoring of all safety sytems and switches,
  and monitoring of control panel buttons.

  It DOES NOT directly monitor punch force and displacement, this is handled
  by another PLC and program to ensure reliable paralellisation. Similarly, 
  it DOES NOT interface directly with the HMI panel, or any attached equipment 
  such as laptops.

  V0.1
  modified 31 March 2020
  by Luke Lewin
First draft

  V0.2
  modified 05 April 2020
  by Luke Lewin
Inputs changed to Pullup type to simplify wiring, logic reversed in code.

  V0-3
  modified X XXX 2020
  by Luke Lewin
*/

// Defining input and output pin numbers

//Digital Inputs
int EmergencyStop = 32; // wire 81
int GreenButton = 34; // wire 83
int RedButton = 33; // wire 82
int BlackButton = 35; // wire 84
int LeftDoorClosed = 36; // wire 85
int RightDoorClosed = 37; // wire 86
int ClampPlateLocked = 38; // wire 87
int PunchUpperLimit = 39; // wire 88
int PunchLowerLimit = 40; // wire 89
int GuardClosed = 41; // wire 810


//Digital Outputs
int GreenLamp = 28; // output wire 37   relay wire 76
int YellowLamp = 29; // output wire 38   relay wire 75
int RedLamp = 22; // output wire 31   relay wire 74
int HPUValve = 25; // output wire 34   relay wire 71
int CetopValve = 24; // output wire 33   relay wire 72
int DiverterValve = 23; // output wire 32   relay wire 73
int GuardInterlock = 26; // output wire 35   relay wire 78
int HPUMotorStart = 27; // output wire 36   relay wire 77
int Alarm = 10;


//Analogue Inputs
int PressureSensor = A1;  // wire 91
int LoadCell = A2; // wire 92
int LinearEncoder = A3; // wire NOT INSTALLED


//Analogue (PWM) Outputs
int MotorSpeed = 2; // wire 62


//Communication
int HMI_SPI = 1; // wire NOT INSTALLED


//Defining Default Values
int MaxPumpFreq = 50;
int MinPumpFreq = 10;
int StandardPumpFreq = 42; //guessed value
int state = 0;
int PrevState = 0;
int Safe = 0;
int Abort = 0;
float Timer = 0;
float Pressure = 0;
float TargetPressurePrelim = 180;
float TargetPressure = 200;
unsigned long time;
unsigned long StartTimer;



// the setup function runs once when you press reset or power the board
void setup() {
  //Initialise Digital Inputs
  pinMode(EmergencyStop, INPUT_PULLUP);
  pinMode(GreenButton, INPUT_PULLUP);
  pinMode(RedButton, INPUT_PULLUP);
  pinMode(BlackButton, INPUT_PULLUP);
  pinMode(LeftDoorClosed, INPUT_PULLUP);
  pinMode(RightDoorClosed, INPUT_PULLUP);
  pinMode(ClampPlateLocked, INPUT_PULLUP);
  pinMode(PunchUpperLimit, INPUT_PULLUP);
  pinMode(PunchLowerLimit, INPUT_PULLUP);
  pinMode(GuardClosed, INPUT_PULLUP);


  //Initialise Digital Outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GreenLamp, OUTPUT);
  pinMode(YellowLamp, OUTPUT);
  pinMode(RedLamp, OUTPUT);
  pinMode(HPUValve, OUTPUT);
  pinMode(CetopValve, OUTPUT);
  pinMode(DiverterValve, OUTPUT);
  pinMode(GuardInterlock, OUTPUT);
  pinMode(HPUMotorStart, OUTPUT);
  pinMode(Alarm, OUTPUT);
  

  //Initialise Analogue Inputs
  pinMode(PressureSensor, INPUT);
  pinMode(LoadCell, INPUT);
  pinMode(LinearEncoder, INPUT);


  //Initialise Analogue (PWM) Outputs
  pinMode(MotorSpeed, OUTPUT);

}

// the loop function runs over and over again forever
void loop() { // This is the main section of code from which the states
//are controlled.
restart: //restart point if the user exits the run sequence
Abort = 0;
  if(digitalRead(EmergencyStop) == HIGH) {
    //state = 9;
    //StateMachine();
    CheckSafe();
  }
  else if (digitalRead(GreenButton) == LOW) {
    StartTimer = millis();
    time = millis();
    state=1;
    StateMachine();
    while ((time - StartTimer )< 4000) {  // A 4 second safety timer for the start button
      CheckSafe();
      if (Abort == 1) {
        goto restart;
      }
      time = millis();

      if (digitalRead(GreenButton) == HIGH) {
        goto restart; //If the start button is pressed for less than 5 seconds the 
        //program will restart and return to standby state
      }
    }
      CheckSafe();
      if (Abort == 1) {
        goto restart;
      }

      analogWrite(MotorSpeed, 255); //Setting pump motor to full speed
      state = 2; //Starting pump and flushing hydraulics
      StateMachine();
      delay(8000);
      CheckSafe();
      if (Abort == 1) {
        goto restart;
      }
      
      state = 3; //Engaging clamp cylinders. Filling accumulators to
      //target pressure 1
      StateMachine();
      time = 0;
      while (Pressure < TargetPressurePrelim) {
        Pressure = 0;//(analogRead(PressureSensor)/1023)*220;
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      time = time + 1;
      if (time == 2400) {
        break;
      }
      }
      state = 4;
      StateMachine();
      analogWrite(MotorSpeed, 100);  //slow down pump speed for accurate pressure targeting
      delay(3000); //Allowing time for pressure to stabalise and motor to spool down
      state = 3; //Filling accumulators to full pressure
      StateMachine();
      time = 0;
      while (Pressure < TargetPressure) {
        Pressure = (analogRead(PressureSensor)/1023)*220;
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
        time = time + 1;
      if (time == 200) {
        break;
      }
      }
      state = 4;
      StateMachine();
      CheckSafe();
      if (Abort == 1) {
        goto restart;
      }
      delay(3000); //Allowing time for user inspection before punch engages

      CheckSafe();
      if (Abort == 1) {
        goto restart;
      }
      state = 5; //Punch engagement starts
      StateMachine();
      while (digitalRead(RedButton) == LOW and digitalRead(PunchUpperLimit) == LOW) {
        //Punch engagement stops once the red button is pushed, or the punch
        //hits its end stop switch.
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      }
      state = 6; //Holding punch position for user inspection
      StateMachine();
      while (digitalRead(RedButton) == HIGH) {
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      }
      delay(1000); //Timer to avoid an accidental double press of the red button
      
      while (digitalRead(RedButton) == LOW) { //Waiting for user input to
        //allow punch and clamp retract
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      }
      state = 7; //Punch retract starts
      StateMachine();
      while (digitalRead(PunchLowerLimit) == LOW) {
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      }
      state = 8; //Clamp retract starts
      StateMachine();
      while (Pressure > 0.1) {
        Pressure = analogRead(PressureSensor) * 0.2 * 220;
        CheckSafe();
        if (Abort == 1) {
          goto restart;
        }
        delay(25);
      }
      
      state = 0; //Returning to standby condition
      StateMachine();
      Timer = 0;
  }
  
  else {
    state = 0;
    StateMachine();
  }
  
}


void CheckSafe() {   //A function to streamline the checking of safety 
//switches in the main code. Uses the value 0 for safe and 1 for unsafe.
  Safe = 0;
  //Safe = Safe+digitalRead(GuardClosed); NOT YET WIRED
  //Safe = Safe+1-digitalRead(ClampPlateLocked);
  //Safe = Safe+1-digitalRead(LeftDoorClosed);
  //Safe = Safe+1-digitalRead(RightDoorClosed);
  Safe = Safe+digitalRead(EmergencyStop);
  if (Safe > 0) {  //If any sensors read unsafe, the value of 'Safe' will
  //be non-zero and the Emergency Stop state will be initiated. 
    PrevState = state;
    state = 9;
    StateMachine();
    digitalWrite(Alarm, HIGH);
    delay(150);
    digitalWrite(Alarm, LOW);
    delay(150);
    digitalWrite(Alarm, HIGH);
    delay(500);
    digitalWrite(Alarm, LOW);
    while (digitalRead(BlackButton) == HIGH){
    delay(25);
    }
    digitalWrite(YellowLamp, HIGH);
    Abort = 0;
    while (Abort == 0) {
      if (PrevState == 0) {
        break;
      }
      else if (digitalRead(RedButton) == HIGH) {
        Abort = 1;
      }
      else if (digitalRead(GreenButton) == LOW) {
        state = PrevState;
        StateMachine();
        Abort = 2;
      }
      delay(10);
    }
  }
}


void StateMachine() {   //This function stores the various 'states' of the
//program. It's essentially a list of common output configurations that
//can be easily switched between to tidy up the main code.
  switch(state) {
    
    case 0: //Standby (safe)
      digitalWrite(GuardInterlock,LOW);
      digitalWrite(YellowLamp,HIGH);
      digitalWrite(GreenLamp,LOW);
      digitalWrite(RedLamp,LOW);
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(HPUMotorStart,LOW);
    break;

    case 1: //Start Button Safety Timer
      digitalWrite(GuardInterlock,LOW);
      digitalWrite(YellowLamp,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(RedLamp,LOW);
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(HPUMotorStart,LOW);
    break;

    case 2: //Pump Standby
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 3: //Clamp Engage
      digitalWrite(HPUValve,HIGH);
      digitalWrite(CetopValve,HIGH);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 4: //Clamp Hold
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,HIGH);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 5: //Punch Engage
      digitalWrite(HPUValve,HIGH);
      digitalWrite(CetopValve,HIGH);
      digitalWrite(DiverterValve,HIGH);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 6: //Punch Hold
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,HIGH);
      digitalWrite(DiverterValve,HIGH);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,HIGH);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 7: //Punch Retract
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,HIGH);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,LOW);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 8: //Clamp Retract
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(GuardInterlock,HIGH);
      digitalWrite(HPUMotorStart,LOW);
      digitalWrite(GreenLamp,HIGH);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,LOW);
    break;

    case 9: //Emergency Stop
      digitalWrite(HPUValve,LOW);
      digitalWrite(CetopValve,LOW);
      digitalWrite(DiverterValve,LOW);
      digitalWrite(GuardInterlock,LOW);
      digitalWrite(HPUMotorStart,LOW);
      digitalWrite(GreenLamp,LOW);
      digitalWrite(YellowLamp,LOW);
      digitalWrite(RedLamp,HIGH);
    break;

  }
}

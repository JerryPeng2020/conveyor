
#include "Stepper.h"


/* Constructor */

Stepper::Stepper(int _dirPin, int _stpPin, int _limtPin) 
{
    dirPin = _dirPin;
    stpPin = _stpPin;
    limtPin = _limtPin;
}

void Stepper::init(int _dirPin, int _stpPin, int _limtPin) 
{
    dirPin = _dirPin;
    stpPin = _stpPin;
    limtPin = _limtPin;
}



/* Status Setting Function */

// Function: reverse limitation status
void Stepper::reverseLimt(bool reverse)
{
    reverseLimtStatus = reverse;
}

// Function: reverse direction status
void Stepper::reverseDir(bool reverse) 
{
    reverseDirStatus = reverse;
}


/* Basic Function */

// Function: Get limitation status 
// output: [true]-reach limitation
bool Stepper::getLimit() {
    if(!reverseLimtStatus)
        if(digitalRead(limtPin) == LOW) return false;
        else return true;
    else
        if(digitalRead(limtPin) == HIGH) return false;
        else return true;
}

// Function: Set motion direction 
// input: CW/CCW diretion
 void Stepper::setDir(bool dir)
 {  if(!reverseDirStatus)
        if(dir) digitalWrite(dirPin, HIGH);
        else digitalWrite(dirPin, LOW);
    else
        if(!dir) digitalWrite(dirPin, HIGH);
        else digitalWrite(dirPin, LOW);
 }


// Function:  Move one step 
// input: period of step motion pluse
void Stepper::moveOneStep(int period)
{   
    int delayUS = period/2;
    int delayMS = delayUS/1000;
    digitalWrite(stpPin, HIGH);
    if(delayMS>0) delay(delayMS);
    else delayMicroseconds(delayUS);
    //
    digitalWrite(stpPin, LOW);
    if(delayMS>0) delay(delayMS);
    else delayMicroseconds(delayUS);
}

/* Application Function */
// Function: back to home position
void Stepper::homing()
{
    setDir(false);
    while(getLimit()) 
    {
        moveOneStep(homingPeriod);
    }

    setDir(true);
    for(int i=0; i<200; i++) moveOneStep(homingPeriod);
}


// Function: move steps
// input: signed number steps, range:[-2147483647, 2147483647]
void Stepper::moveSteps(int steps)
{
    // set move direction
    if(steps>0) setDir(true);
    else setDir(false);

    /* smooth motion mechanism */
    // parameters
    int stepsABS = abs(steps);
    int addRatio = 10;  // 1/addRatio steps smooth motion section 
    int addMaxPulse = 100;
    int addSection = min(stepsABS/addRatio, addMaxPulse);
    int addTime = 0;  // Unit: us
    int addTimeMax = 1000;  // Unit: us

    for(int i=stepsABS; i>0; i--) 
    {

        // execute moving
        // accelerate
        if(i>stepsABS-addSection)
        {
            addTime = addTimeMax * (1-float((i-stepsABS))/float(addSection));
            moveOneStep(movingPeriod+addTime);
        }
        // decelerate
        else if(i<addSection)
        {
            addTime = addTimeMax * (1-float(i)/float(addSection));
            moveOneStep(movingPeriod+addTime);
        }
        // full speed motion
        else
        {
            moveOneStep(movingPeriod);
        }


    }

}

/****************************************************
 * Copyright (C) 2021 Pinecone.AI - All Rights Reserved
 * 
 * Project: Stepper Motion Class
 * Version: 1.0
 * 
 * Description: Class for stepper control with end limiter
 *  
 *  
 * 
 * Date:   Aug. 22th, 2021
 * Author: Jerry Peng
 * Web:    www.pinecone.ai
 *  
/***************************************************/

#ifndef _STEPPER_H
#define _STEPPER_H

#include <arduino.h>



class Stepper {

 private:
    // GPIO Pin
    int dirPin;
    int stpPin;
    int limtPin;

    // status parameters
    bool reverseLimtStatus = false;  // reverse limitation detection status
    bool reverseDirStatus = false;   // reverse direction status


 public:

    // motion parameters
    int homingPeriod = 800;
    int movingPeriod = 400;

    Stepper(int _dirPin, int _stpPin, int _limtPin);


    void init(int _dirPin, int _stpPin, int _limtPin);

    /* Status Setting Function */
    void reverseLimt(bool reverse);
    void reverseDir(bool reverse);

    /* Basic Function */
    bool getLimit(); // [output] true: reach limitaion
    void setDir(bool dir);        // 
    void moveOneStep(int period); // period: period of one step, Unit: us, range:[0, 2147483647]

    /* Application Function */
    void homing();
    void moveSteps(int steps); // input: signed number steps, range:[-2147483647, 2147483647]

   
    


};


#endif
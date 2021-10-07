
/****************************************************
 * Copyright (C) 2021 Pinecone.AI - All Rights Reserved
 * 
 * Project: Industrial Conveyor
 * Version: 0.1
 * 
 * Description:
 *  
 *  
 * 
 * Date:   Sep. 27th, 2021
 * Author: Jerry Peng
 * Web:    www.pinecone.ai
 *  
 * 
/***************************************************/


#include "Conveyor.h"
Conveyor bot;

  
void setup() 
{
  bot.init();
  
  // debug
  // bot.baffleStepper.moveSteps(2800);
  // bot.setSpeed(128);
} 



//主函数
void loop()
{
  bot.firmwareSystem();

  // debug
  // Serial.print("baffle Status: ");
  // Serial.println(bot.baffleStepper.getLimit());
  // bot.baffleStepper.movingPeriod = 200;
  // bot.baffleStepper.moveSteps(2000);
  // delay(1000);
  // bot.baffleStepper.moveSteps(-2000);
  // delay(1000);
}

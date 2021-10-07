/****************************************************
 * Copyright (C) 2021 Pinecone.AI - All Rights Reserved
 * 
 * Project: Industrial Conveyor
 * Version: 0.1
 * 
 * Description: Prototype  project fo industrial conveyor
 *  
 * 
 * Date:   Sep. 27th, 2021
 * Author: Jerry Peng
 * Web:    www.pinecone.ai
 *  
/***************************************************
 * To be accamplished:
 * 2021-3-6:
 * 
 * 
/***************************************************/

#ifndef _Conveyor_H
#define _Conveyor_H

#include "EEPROM.h"



/* Hardware Pins */
#define MOTOR_DIR  23
#define MOTOR_STP  22
#define DETECTOR 35 // no use


#include "Stepper.h"

// Device ROM   Data设备 ROM 数据
#define DEVICE_TYPE 0  // industrial conveyor
#define VERSION 1     // version 0.1

// EEPROM： 17 for register(currently, 0 used)
#define EEPROM_SIZE 17
#define REG_EEPROM_SIZE 0


// Command Header
#define BEGIN_FLAG_0 0xAA
#define BEGIN_FLAG_1 0x77

/* Register ID */
// ROM
#define DEVICE_TYPE_ID 0
#define VERSION_ID 1
#define MAC_ID 2
// EEPROM
#define EEPROM_ID 11
#define DEVICE_ID 11
#define BAUDRATE_ID 12
#define LEVER_OFFSET_ID 13
// RAM 
#define EEPROM_LOCK_ID 28
#define SPEED_ID 30

// #define BTN_ENABLE_ID 66
#define FB_FREQ_ID 67
// #define EXE_STATUS_ID 68
// #define BIN_STATUS_ID 69



class Conveyor {

  public:
    Stepper motor = Stepper(MOTOR_DIR, MOTOR_STP, DETECTOR);


    uint64_t chipID;

    void alarm(int level, int type);

    Conveyor();
    void init();
    

    // UART
    bool beginFlag = false;
    bool haveInstruction = false;
    int instruction = 0;
    int cnt = 0;
    int dataBuf[120];
    int rxBuf_pre;

    void receiveCommand(); //从串口接收命令
 

    // CRC
    void sendData(uint8_t *data, int len); //发送数据
    uint16_t CRC16_MODBUS(uint8_t *data, int len); //CRC校验
    uint8_t InvertUint8( uint8_t dBuf);
    uint16_t InvertUint16( uint16_t dBuf);

    // command register
    uint8_t comReg[80];

    // LED 
    bool ledStatus = false;

    void firmwareSystem();

    unsigned long time_300ms = 0;   // long button press timer


    /* motion ctrl 运动控制*/
    // void stop();
    bool motionFlag = false;
    void setSpeed(int _speed);

};

#endif

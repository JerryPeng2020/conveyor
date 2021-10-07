#include "Conveyor.h"

/* Constructor */
Conveyor::Conveyor() {}

// initialize
void Conveyor::init()
{

  /* Read ROM data to comReg(command register) */
  comReg[DEVICE_TYPE_ID] = DEVICE_TYPE;//设置存储空间大小，数组作为中间存储  81
  comReg[VERSION_ID] = VERSION;   //11

  // Get chip ID (MAC address), length = 6 bytes
  chipID = ESP.getEfuseMac();  //获取数据存储在chipID变量中作为中转
  for (int i = 0; i < 6; i++)   
    comReg[MAC_ID + i] = chipID >> (40 - i * 8);//将中转变量chipID中的数据分解并存储到数组comReg[]中
                                               //右移一个字节的整数倍，从数组角标为2的位开始存储
  /* EEPROM init and Read EEPROM data to comReg */
  //
  // indivudual EEPROM: will set every uninitialized byte(0x00) to 0xFF.
  //                    If they these bytes have been set to a value(include 0x00),
  //                    the stored data will not change anymore.
  if (!EEPROM.begin(EEPROM_SIZE))//初始化EEPROM存储器的大小
  { /*  Serial.println("failed to initialise EEPROM"); */
  }
  else
  { /* Serial.println("Successfully initialise EEPROM"); */
  }

  // read EEPROM data to comReg
  for (int i = 0; i < REG_EEPROM_SIZE; i++)
  {
    comReg[EEPROM_ID + i] = EEPROM.read(i);//从存储器中读取数据存储到数组comReg[]中
  }

  // Device Serial Port Init: read EEPROM(1) & setup serial port //波特率初始化
  if (comReg[BAUDRATE_ID] == 3)
    Serial.begin(1000000);
  //  else if(buadrateIndex == 7) Serial.begin(9600);
  else
    Serial.begin(115200);

  /* Init comReg RAM data */
  comReg[EEPROM_LOCK_ID] = 1; // eepromLock, 0-disable EEPROM write protect lock(can write),  1-enable it
  // comReg[SORTING_ID] = 0;     // 0-none, 1~4: garbage sorting ID
  // comReg[OFFSET_CLB_ID] = 0;  // 1- calibrate for once and back to 0(idle)
  // comReg[BTN_ENABLE_ID] = 1;  // function button enable
  // //
  // comReg[FB_FREQ_ID] = 0;    // feedback frequency
  // comReg[EXE_STATUS_ID] = 0; // 0-execution done, 1-in execution
  // comReg[BIN_STATUS_ID] = 0; // 0-all none full

  /* Read EEPROM data from comReg to other notations */

  /* Init Components */
  // init Servo Serial Port
//  Serial2.begin(1000000);
  time_300ms = millis();


  /////////////////////////////////////////////////////////////////////////////////////////////////
  /* 电机部分 */

  // 引脚定义
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_STP, OUTPUT);
  pinMode(DETECTOR, INPUT);

  // 初始化
  // 1. 复位检测电平颠倒设置: 函数输入参数为false时（预设），高电平触发； 为ture时，低电平触发。
  motor.reverseLimt(false);

  // 2. 电机旋转方向反向设置：函数输入参数为false时（预设），正向运行； 为ture时，反向运行。
  motor.reverseDir(false);

  // 3. 电机运行时，每一步进周期设置（通过设置步进周期值调节运行速度）；单位：us； 
  motor.homingPeriod = 800;     // 归位速度
  motor.movingPeriod = 400;     // 运行速度


  // 4. 电机初始化归位
  // motor.homing();

  

  /////////////////////////////////////////////////////////////////////////////////////////////////


}

// scan and receive command from USB-UART port 扫描并接收来自 USB-UART 端口的命令
void Conveyor::receiveCommand() //接收命令
{
  /*  recive commands  */
  while (Serial.available() > 0)
  {
    // read data
    int rxBuf = Serial.read();
    if (!beginFlag)
    {
      beginFlag = (rxBuf_pre == BEGIN_FLAG_0 && rxBuf == BEGIN_FLAG_1) ? true : false; // Beginning Flag  开始标志
    }
    else
    {
      if (instruction == 0)
        instruction = rxBuf;
      else
      {
        switch (instruction)
        {

        // Read register 读取寄存器
        case 0x03:
          dataBuf[cnt++] = rxBuf;
          if (cnt >= 4)
          {
            beginFlag = false;
            instruction = 0;
            cnt = 0;
            uint8_t data_read[] = {0xAA, 0x77, 0x03, dataBuf[0], dataBuf[1]};
            uint16_t crc = CRC16_MODBUS(data_read, 5);
            int high = crc / 256;
            int low = crc % 256;
            int addr = dataBuf[0]; // begin address 起始地址
            int num = dataBuf[1];  // reg number 注册号

            // Serial.write(num);   // num debug

            // CRC check pass, send reg data  CRC校验通过，发送reg数据
            if (low == dataBuf[2] && high == dataBuf[3])
            {
              // prepare data 准备数据
              uint8_t data_send[5 + num];
              data_send[0] = 0xAA;
              data_send[1] = 0x77;
              data_send[2] = 0x03;
              data_send[3] = addr;
              data_send[4] = num;

              // update relevant registers 更新相关寄存器
              for (int i = 0; i < num; i++)
              {
                /*
                // read XXX register
                if((addr+i) == XXX_ID )
                  comReg[addr+i] = XXX_value;
                // read XXXX register
                else if((addr+i) == XXXX_ID )
                  comReg[addr+i] = XXXX_value;
                */
              }

              // write data to send 写入要发送的数据  data_send
              for (int i = 0; i < num; i++)
                data_send[5 + i] = comReg[addr + i];

              // send data with CRC inside 发送带有 CRC 的数据
              sendData(data_send, num + 5);//将数据发送出去利用 Serial.write（）
            }
          }
          break;

        // write reg 写寄存器
        case 0x04:
          dataBuf[cnt++] = rxBuf;
          // get data length 获取数据长度
          if (cnt >= 2)
          {
            if (cnt >= dataBuf[1] + 4)
            {
              beginFlag = false;
              instruction = 0;
              cnt = 0;
              int addr = dataBuf[0];
              int num = dataBuf[1];
              // data read 数据读取
              uint8_t data_read[5 + num];
              data_read[0] = 0xAA;
              data_read[1] = 0x77;
              data_read[2] = 0x04;
              data_read[3] = addr;
              data_read[4] = num;

              for (int i = 0; i < num; i++)
              {
                data_read[5 + i] = dataBuf[2 + i];
              }
              // CRC check CRC校验
              uint16_t crc = CRC16_MODBUS(data_read, num + 5);
              int high = crc / 256;
              int low = crc % 256;

              // CRC check pass, write reg CRC校验通过，写入reg
              if (low == dataBuf[num + 2] && high == dataBuf[num + 3])
              {

                // write comReg 写comReg
                for (int i = addr; i < addr + num; i++)
                {
                  // update writeable addr data 更新可写地址数据
                  if (i >= EEPROM_ID && i <= FB_FREQ_ID)  comReg[i] = dataBuf[2 + i - addr];
/**************************************************************************************************************************/
                    /* immediate execute command 立即执行命令 */

                    // 分拣指令
                    if (i == SPEED_ID) 
                    {
                      setSpeed(comReg[i]-127);
                    }
                      
                  

                }

                // if need write EEPROM, check lock frist and then write EEPROM
                /*如果需要写EEPROM，先检查lock然后再写EEPROM*/
                if (comReg[EEPROM_LOCK_ID] == 0)
                {
                  for (int i = EEPROM_ID; i < EEPROM_ID + REG_EEPROM_SIZE; i++)
                    if (i >= addr && i < addr + num)
                      EEPROM.write(i - EEPROM_ID, comReg[i]);//将数据倒置装进EEPROM
                }
                EEPROM.commit(); //保存更改的数据
              }
            }
          }
          break;

        default:
          beginFlag = false;
          instruction = 0;
          cnt = 0;
          break;
        }
      }
    }

    rxBuf_pre = rxBuf;
  }
}



// 7Bot alarm: send alarm massage through UART & buzzer on for a while
//7Bot报警：通过UART及蜂鸣器发出报警
void Conveyor::alarm(int level, int type)
{
  // send alarm massage
  uint8_t data_send[5];
  data_send[0] = 0xAA;
  data_send[1] = 0x77;
  data_send[2] = 0x08;
  data_send[3] = level;
  data_send[4] = type;
  sendData(data_send, 5);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/* CRC & Data Send */

// Be care!!!! sizeof(data) do not work properly, maybe because of
      /* 注意！！！！ sizeof(data) 不能正常工作，可能是因为
          uint8_t 数据在 sizeof() 中没有正确处理。*/
// the uint8_t data is not treat correctly in sizeof().
void Conveyor::sendData(uint8_t *data, int len)  //发送数据
{
  uint16_t crc = CRC16_MODBUS(data, len);
  int high = crc / 256;
  int low = crc % 256;
  for (int i = 0; i < len; i++) Serial.write(data[i]);
  
  Serial.write(low);
  Serial.write(high);
}

uint16_t Conveyor::CRC16_MODBUS(uint8_t *data, int len) //16位CRC校验
{
  uint16_t wCRCin = 0xFFFF;
  uint16_t wCPoly = 0x8005;
  uint8_t wbyte = 0;
  int j = 0;
  while (len > 0)
  {
    len--;
    wbyte = data[j++];
    wbyte = InvertUint8(wbyte);
    wCRCin ^= (wbyte << 8);
    for (int i = 0; i < 8; i++)
    {
      if ((wCRCin & 0x8000) != 0)
        wCRCin = uint16_t((wCRCin << 1) ^ wCPoly);
      else
        wCRCin = uint16_t(wCRCin << 1);
    }
  }
  wCRCin = InvertUint16(wCRCin);
  return (wCRCin);
}

uint8_t Conveyor::InvertUint8(uint8_t dBuf) 
{
  int i;
  uint8_t tmp = 0;
  for (i = 0; i < 8; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (7 - i);
  }
  return tmp;
}

uint16_t Conveyor::InvertUint16(uint16_t dBuf)
{
  int i;
  uint16_t tmp;
  tmp = 0;
  for (i = 0; i < 16; i++)
  {
    if ((dBuf & (1 << i)) != 0)
      tmp |= 1 << (15 - i);
  }
  return tmp;
}




// Function: firmware System  片上资源检测
void Conveyor::firmwareSystem()
{

  // 2. monitor serial port 监控串口
  receiveCommand();
  
  if(motionFlag) motor.moveOneStep(motor.movingPeriod);


}



// Function: set motion speed ( To Be updated later !)
// Input: _speed range [-128, 127]
void Conveyor::setSpeed(int _speed)
{
  
  if(_speed>0) motor.setDir(true);
  else motor.setDir(false);

  // hardware parameters
  int speed = abs(_speed);
  if(speed != 0) motionFlag = true;
  else motionFlag = false;

  int minDelay = 250; // us
  int addDelay = (127 - speed) * 4;
  motor.movingPeriod = minDelay + addDelay;
}

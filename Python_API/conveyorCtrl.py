
# Conveyor Control Example

# Date:    Sep. 27th, 2021
# Author:  Jerry Peng
 
import time 
from lib.Conveyor import Conveyor


# assign serial port to Conveyor. 
# ATTENTION: Change the parameter "/dev/cu.SLAB_USBtoUART" below 
#            according to your own computer OS system. Open whatever port is 7Bot on your computer.
# Usually:  "/dev/cu.SLAB_USBtoUART" on Mac OS
#           "/dev/ttyUSB0" on Linux
#           'COM1' on Windows
bot = Conveyor("/dev/cu.SLAB_USBtoUART") 


# 1.move at full speed (positive direction）
bot.setSpeed(127) # speed valid range: [-127, 127]
time.sleep(10)

# 2.stop
bot.setSpeed(0) # speed valid range: [-127, 127]
time.sleep(1)

# 3.move at full speed (negative direction）
bot.setSpeed(-127) # speed valid range: [-127, 127]
time.sleep(10)

# 4.stop
bot.setSpeed(0) # speed valid range: [-127, 127]


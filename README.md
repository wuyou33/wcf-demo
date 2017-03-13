# WCF

## Overview

Some sample codes for the Wahba Complementary Filter (WCF) introduced in [1](http://ieeexplore.ieee.org/document/7297816/). The same core algorithm was also used in  [UPsat](https://github.com/librespacefoundation/upsat-adcs-software).
Main algorithm (not the adaptive one) is at wahba_rot.c and the SVD is at jacobiTS.c. Intead of the slight speed optimized one used in the paper a more general is provided here.  


[1] P. Marantos, Y. Koveos and K. J. Kyriakopoulos, "UAV State Estimation Using Adaptive Complementary Filters," in IEEE Transactions on Control Systems Technology, vol. 24, no. 4, pp. 1214-1226, July 2016.


## Hardware Dependencies for the demo

* STM32F4xx with 12MHz crystal! If use is on STM32F4DISCOVERY change makefile option COMPILE_OPTS+=-DHSE_VALUE=12000000 to 8000000
* LSM9DS0 is assumed at PB10, PB11
* USB configured is OTG_FS at PA8-12


## Software Dependencies for the demo

* GCC, tested with gcc-arm-none-eabi-6_2-2016q4
* Openocd

## How to use


* Change the 1st line of the makefile  (ARMGCC_PATH = /usr/local/gcc-arm-none-eabi-6_2-2016q4/bin/) to your compiler's path
* In SysTick_Handler in main.c change sprintf output format appropriately, it should give the Euler angles by default
* To use the filter use wahba_StructInit() to initialise the filter, weight, etc.  and wahba_rot to update rotation estimation


```
make -j4
make flash
sudo cat /dev/ttyACM0
```

  

## Disclaimer
Obviously a not well documented and draft code (that is Use at your own risk..) 



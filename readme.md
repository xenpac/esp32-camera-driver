# new esp32-cam I2S camera driver


This is a new driver for the esp32 i2s modul and the OV2640 camera. (OV5640 possible)

esp-idf 4.4 c code project.

It is basicly the same as this one: (look)  
https://github.com/xenpac/ESP32-CAM-Linux-Motion

but implements a new more compact camera I2S driver.
(updated to IDF V4.4, the latest)

The concept is to distinguish between I2S capture driver and camera-driver.

There are actually two different files for them.

In a first step we are using only JPEG, so the camera-modul needs to deliver JPEG-compressed images.  
observed pclk is 5Mhz in all resoltutions when jpeg is captured, so constant.  
All other formats (YUV,RGB...) are not usefull as the datasize would be too high for the esp32 and the network streaming.  

**The I2S driver** is very much interrupt driven.  
It only needs to know a "max framebuffer size".  
No need to set the image-size for jpeg.  
No fixed relationship hsync/vsync or linesize.  

The max framebuffer size shall match the max JPG-imagesize at best quality.  
The driver tells you if the framebuffer is too small.  

Speed-wise test have been performed and the limiting factor is the SPIRAM write speed above 7Mhz pclk.  
However, as pclk is constant 5Mhz in jpeg, this should not be a problem.  

**The Camera Driver** handles all specific camera related procedures.  
It still uses the official camera driver structures, which are a bit complicated.  
It interacts with the cameras driver file and gets the cameras status and function list to the application.  
Also the cameras xclk/input clock is set here and the I2C bus is inited.  
It will probe and init the camera at startup.  
It has been written for the =V2640 modul, but can be adapted to fe. OV5640.  

For **the application** there not much difference in using this new driver, except the function names.  
Added a new Button to download the jpg-image directly from the camera at given resolution.  


**Status Display**
- NetFPS = Frames per second transferred over the network. network is the limiting factor.
- CamFPS = Frames per second the camera delivers. (vertical sync pulses count)
- I2sFPS = Frames per second the I2S engine delivers error free. errorframes are not counted and discarded.
- UpTime = Number of hours the Cam is operating after last reset. to check, if a reset occured.

**DMAerrors** will occur if :
- VSYNC processing takes too long (UXGA,best Quality) and overlaps with Framestart.
- The datacopy from DMAbuffer to Framebuffer takes too long (in_done int) (SPIRAM speed is the problem). (below 7Mhz pclk should work)
**JPGerrors** will occur if:
- sometimes occur when switching resolutions on the fly
- Quality is too high (below 12), especially at high resolutions.
Also:
- interrupt latency may cause trouble on high resolutions when streaming because wifi also uses interrupts. other system resources.
   So interrupts are not garanteed to occur on time.
- the unexpected ? hardware defects....etc.
in essence....errors do occur.

**OV2640 Framerates**
OV2640 datasheet says: UXGA=15fps, VGA=30fps  at xclk=24Mhz.
Framerates at xclk=20Mhz:(default on this software) (pclk is 5Mhz in all resolutions with jpeg)
- above SVGA = 13
- VGA, HVGA = 25
- below HVGA = 50

Framerates at xclk=25Mhz: (Note: 24Mhz is not setable, invalid divider on esp32)
- above SVGA = 16
- VGA, HVGA = 31
- below HVGA = 63


you can **tweek** to higher framerates by changing the xclk input clock and play with the CLK 2x and divider.  
This would typically only work for a specific resoltion. CLK 2X may or may not work.  
Lowering the jpg-quality also helps.  

i got 44 fps at VGA with 18Mhz clock and CLK 2x on.  
100fps at QVGA xclk=40mhz  


have fun, xenpac;)

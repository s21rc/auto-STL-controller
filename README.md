# auto-STL-controller

Watch the video of the loop controller in action with a ICOM IC7000 radio. https://youtu.be/PtbcYaU-QEk

The antenna was built last month and was working perfectly and tuned from 30m to 12m with less than 1.5 VSWR. I decided to make the tune automatic. Though there are some nice tutorial and complete project for such - but I wanted to make it from scratch - to get the full fun of it and also to make it as simple as possible. The control box sits in the shack. 4 pin tuner cable connects to my ICOM radio. 20m of CAT6 UTP cable connects the Stepper motor from the box. There is a small OLED to see status and one Rotary encoder for manual tune (I hate button - too difficult to install in an enclosure).

Parts used:
Arduino Nano (16MHz ATMEGA328P)
DRV8825 motor driver
I2C 128x64 OLED
Rotary encoder with switch module
NEMA17 1.8 degree 1.3A stepper motor
VSWR comparator and bridge (I have used an old board from another commercial antenna, but any VSWR bridge should work)

Operation:
On power on: Load last motor position and last tuned frequency from EEPROM.
TUNE: when tune button on radio is pressed,
#1. the circuit read the frequency, decide bands.
#2. It searches in the EEPROM from base position of that band (this data is entered manually in initialization)
#3. Go to that position very first. and then change one step and measure SWR.
#4. From the 3 point of SWR<2 (falling), SWR<1.5 and again SWR<2 (rising) it calculate the middle position and move the motor there.
#5. If band data is not available it decide direction and move one step and check SWR - and get best position by #4 above.

Schematic and Sketch are attached if anyone wish to built or take idea.

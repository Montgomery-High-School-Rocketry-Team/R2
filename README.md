# R2 of the protoboards

R2 of the protoboards features a buzzer, voltage reg, BNO55, bmp388, 
teensy 4.1, an actual good protoboard, and quaternion estmiation. 

## Files/Folders

### R1 BNO and Data

This folder contains the file/`ino` script that is the thing we are 
putting on the board, it contains quaternion estimation algos and more, it 
uses our custom rocketry utils library

### BNO055 Tests

A folder containing folders of ino projects that test various functions 
such as switching modes, collecting data at a high data rate, and custom 
mode changing that is not in the bno library

It has
- main_testing.ino
- etc...

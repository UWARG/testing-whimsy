This is the test scripts for the WARG Gimbal and Grabber test.

INSTRUCTIONS:
1) Upload the Gimbal_Grabber_Test.ino sketch to the board
2) Connect the Y servo to pin 5 and the X servo to pin 6

Clean Solution:
- Install the keyboard and pyserial lib.
- Change the COM port variable to the appropriate string
- Run the Python Script and use WASD to move the servo

Unclean (but easier) solution:
- Open Serial Monitor and type in either 1, 2, 3, or 4 (and press ENTER) to control the Servo

Note: Use the clean solution if you want smoother movements. Using the unclean one will result in lots of jitter since you need to hit enter to send a command.

# Converters

# Hex to Something Converter (hex_to_something.cpp)

Converter that was made to convert Hex logs from a STM32 ARM chip to more readable datatypes. 

## Instructions
 
1. Compile code using C++ 2017 minumum
2. When running the executable, you will need three additional arguments:
    - Argument 1: Type of conversion (Look at table below)
    - Argument 2: Input file path (relative to executable)
    - Argument 3: Output file path (relative to executable)
    - If you need a scale factor and calibration for the hex to 16 bit signed integer conversion, you need two more arguments
        - Argument 4: Scale factor
        - Argument 5: Calibration value

| Code | Conversion
|- |-
| a | hex to 32 bit float
| b | hex to 16 bit signed integer
| c | hex to 16 bit signed integer, but add scale factor and calibration

 ## Example runs
 
 Example 1:
 ```bash
$ g++ -o a --std=c++17 hex_to_something.cc
 
$ ./a a Logs/rollLog.txt Converted/roll.txt
 ```

Example 2:
 ```bash
$ g++ -o a --std=c++17 hex_to_something.cc
 
$ ./a c Logs/accXLog.txt Converted/accX.txt 4.09 64.5476685
 ```
 
 


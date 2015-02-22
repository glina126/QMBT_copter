EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Custom_components
LIBS:QMBTv1-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MPU6050 IC4
U 1 1 54A7B942
P 5600 3400
F 0 "IC4" H 5400 3900 47  0000 C CNN
F 1 "MPU6050" H 5700 2500 47  0000 C CNN
F 2 "footprints:MPU6050" H 5600 3400 47  0001 C CNN
F 3 "" H 5600 3400 47  0000 C CNN
	1    5600 3400
	1    0    0    -1  
$EndComp
Text GLabel 6300 3050 2    47   Input ~ 0
SDA
Text GLabel 6300 3150 2    47   Input ~ 0
SCL
NoConn ~ 6300 3250
NoConn ~ 6300 3350
NoConn ~ 6300 3550
NoConn ~ 6300 3750
NoConn ~ 6300 3850
NoConn ~ 6300 3950
NoConn ~ 6300 4050
NoConn ~ 5100 3450
NoConn ~ 5100 3350
NoConn ~ 5100 3250
NoConn ~ 5100 3150
NoConn ~ 5100 3550
NoConn ~ 5100 3650
Wire Wire Line
	4300 3850 5100 3850
$Comp
L +3.3V #PWR020
U 1 1 54A7BC44
P 4800 3250
F 0 "#PWR020" H 4800 3210 30  0001 C CNN
F 1 "+3.3V" H 4800 3360 30  0000 C CNN
F 2 "" H 4800 3250 60  0000 C CNN
F 3 "" H 4800 3250 60  0000 C CNN
	1    4800 3250
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 54A7BC5A
P 4600 3300
F 0 "C9" H 4600 3400 40  0000 L CNN
F 1 "10nF" H 4606 3215 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 4638 3150 30  0001 C CNN
F 3 "" H 4600 3300 60  0000 C CNN
	1    4600 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	4800 3750 5100 3750
Wire Wire Line
	4800 3250 4800 3750
Connection ~ 4800 3300
$Comp
L GND #PWR021
U 1 1 54A7BCFB
P 4350 3450
F 0 "#PWR021" H 4350 3450 30  0001 C CNN
F 1 "GND" H 4350 3380 30  0001 C CNN
F 2 "" H 4350 3450 60  0000 C CNN
F 3 "" H 4350 3450 60  0000 C CNN
	1    4350 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3450 4350 3300
Wire Wire Line
	4350 3300 4400 3300
$Comp
L GND #PWR022
U 1 1 54A7BDBC
P 4950 3200
F 0 "#PWR022" H 4950 3200 30  0001 C CNN
F 1 "GND" H 4950 3130 30  0001 C CNN
F 2 "" H 4950 3200 60  0000 C CNN
F 3 "" H 4950 3200 60  0000 C CNN
	1    4950 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3050 4950 3050
Wire Wire Line
	4950 3050 4950 3200
$Comp
L C C10
U 1 1 54A7C020
P 4600 4350
F 0 "C10" H 4600 4450 40  0000 L CNN
F 1 "0.1uF" H 4606 4265 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 4638 4200 30  0001 C CNN
F 3 "" H 4600 4350 60  0000 C CNN
	1    4600 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 54A7C085
P 4600 4700
F 0 "#PWR023" H 4600 4700 30  0001 C CNN
F 1 "GND" H 4600 4630 30  0001 C CNN
F 2 "" H 4600 4700 60  0000 C CNN
F 3 "" H 4600 4700 60  0000 C CNN
	1    4600 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4700 4600 4550
Wire Wire Line
	4600 4150 4600 3950
Wire Wire Line
	4600 3950 5100 3950
$Comp
L C C12
U 1 1 54A7C160
P 7050 3650
F 0 "C12" H 7050 3750 40  0000 L CNN
F 1 "2.2nF" H 7056 3565 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 7088 3500 30  0001 C CNN
F 3 "" H 7050 3650 60  0000 C CNN
	1    7050 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3450 7050 3450
$Comp
L GND #PWR024
U 1 1 54A7C1B8
P 7050 4000
F 0 "#PWR024" H 7050 4000 30  0001 C CNN
F 1 "GND" H 7050 3930 30  0001 C CNN
F 2 "" H 7050 4000 60  0000 C CNN
F 3 "" H 7050 4000 60  0000 C CNN
	1    7050 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4000 7050 3850
$Comp
L GND #PWR025
U 1 1 54A7C250
P 6450 3950
F 0 "#PWR025" H 6450 3950 30  0001 C CNN
F 1 "GND" H 6450 3880 30  0001 C CNN
F 2 "" H 6450 3950 60  0000 C CNN
F 3 "" H 6450 3950 60  0000 C CNN
	1    6450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3950 6450 3650
Wire Wire Line
	6450 3650 6300 3650
$Comp
L C C11
U 1 1 54A7C31F
P 6700 4350
F 0 "C11" H 6700 4450 40  0000 L CNN
F 1 "0.1uF" H 6706 4265 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 6738 4200 30  0001 C CNN
F 3 "" H 6700 4350 60  0000 C CNN
	1    6700 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 4150 6700 4150
$Comp
L GND #PWR026
U 1 1 54A7C383
P 6700 4750
F 0 "#PWR026" H 6700 4750 30  0001 C CNN
F 1 "GND" H 6700 4680 30  0001 C CNN
F 2 "" H 6700 4750 60  0000 C CNN
F 3 "" H 6700 4750 60  0000 C CNN
	1    6700 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4750 6700 4550
Text GLabel 5100 4150 0    47   Input ~ 0
INT
Text GLabel 5100 4050 0    47   Input ~ 0
FSYNC
$Comp
L GND #PWR027
U 1 1 54A83722
P 4300 4050
F 0 "#PWR027" H 4300 4050 30  0001 C CNN
F 1 "GND" H 4300 3980 30  0001 C CNN
F 2 "" H 4300 4050 60  0000 C CNN
F 3 "" H 4300 4050 60  0000 C CNN
	1    4300 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3850 4300 4050
Text Notes 4850 3850 2    47   ~ 0
default address\n
NoConn ~ 5100 4050
$Comp
L +3.3V #PWR028
U 1 1 54A9B9BB
P 6700 4050
F 0 "#PWR028" H 6700 4010 30  0001 C CNN
F 1 "+3.3V" H 6700 4160 30  0000 C CNN
F 2 "" H 6700 4050 60  0000 C CNN
F 3 "" H 6700 4050 60  0000 C CNN
	1    6700 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4150 6700 4050
$EndSCHEMATC

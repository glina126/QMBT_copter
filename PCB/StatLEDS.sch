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
Sheet 7 8
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
L LED D5
U 1 1 54A8DCB7
P 5550 3850
F 0 "D5" H 5550 3950 50  0000 C CNN
F 1 "LED" H 5550 3750 50  0000 C CNN
F 2 "footprints:SMD-0402_LED" H 5550 3850 60  0001 C CNN
F 3 "" H 5550 3850 60  0000 C CNN
	1    5550 3850
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 54A8DCDC
P 6100 3850
F 0 "R17" V 6180 3850 40  0000 C CNN
F 1 "1K" V 6107 3851 40  0000 C CNN
F 2 "Resistors_SMD:R_0402" V 6030 3850 30  0001 C CNN
F 3 "" H 6100 3850 30  0000 C CNN
	1    6100 3850
	0    1    1    0   
$EndComp
$Comp
L GND #PWR041
U 1 1 54A8DD56
P 6400 3950
F 0 "#PWR041" H 6400 3950 30  0001 C CNN
F 1 "GND" H 6400 3880 30  0001 C CNN
F 2 "" H 6400 3950 60  0000 C CNN
F 3 "" H 6400 3950 60  0000 C CNN
	1    6400 3950
	1    0    0    -1  
$EndComp
Text GLabel 5350 3850 0    60   Input ~ 0
D5
Wire Wire Line
	5750 3850 5850 3850
Wire Wire Line
	6350 3850 6400 3850
Wire Wire Line
	6400 3850 6400 3950
$EndSCHEMATC

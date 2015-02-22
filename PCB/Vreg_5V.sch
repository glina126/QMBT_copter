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
Sheet 8 8
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
L MCP1802_(Vreg) IC3
U 1 1 54B41384
P 5600 3650
F 0 "IC3" H 5450 3900 60  0000 C CNN
F 1 "MCP1802_(Vreg)" H 5600 3300 60  0001 C CNN
F 2 "footprints:Sot-23-5" H 5600 3350 60  0000 C CNN
F 3 "" H 5600 3650 60  0000 C CNN
	1    5600 3650
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 54B413B5
P 4850 4050
F 0 "C7" H 4850 4150 40  0000 L CNN
F 1 "C" H 4856 3965 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 4888 3900 30  0001 C CNN
F 3 "" H 4850 4050 60  0000 C CNN
	1    4850 4050
	1    0    0    -1  
$EndComp
Text GLabel 4450 3550 0    60   Input ~ 0
VIN
Wire Wire Line
	4450 3550 5200 3550
Wire Wire Line
	4850 3850 4850 3550
Connection ~ 4850 3550
Wire Wire Line
	5200 3750 4850 3750
Connection ~ 4850 3750
$Comp
L GND #PWR042
U 1 1 54B414CE
P 5100 4050
F 0 "#PWR042" H 5100 4050 30  0001 C CNN
F 1 "GND" H 5100 3980 30  0001 C CNN
F 2 "" H 5100 4050 60  0000 C CNN
F 3 "" H 5100 4050 60  0000 C CNN
	1    5100 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4050 5100 3650
Wire Wire Line
	5100 3650 5200 3650
$Comp
L GND #PWR043
U 1 1 54B414F6
P 4850 4400
F 0 "#PWR043" H 4850 4400 30  0001 C CNN
F 1 "GND" H 4850 4330 30  0001 C CNN
F 2 "" H 4850 4400 60  0000 C CNN
F 3 "" H 4850 4400 60  0000 C CNN
	1    4850 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4400 4850 4250
NoConn ~ 6050 3750
$Comp
L C C8
U 1 1 54B415D6
P 6600 4000
F 0 "C8" H 6600 4100 40  0000 L CNN
F 1 "C" H 6606 3915 40  0000 L CNN
F 2 "SMD_Packages:SMD-0402_c" H 6638 3850 30  0001 C CNN
F 3 "" H 6600 4000 60  0000 C CNN
	1    6600 4000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR044
U 1 1 54B4161B
P 6600 3350
F 0 "#PWR044" H 6600 3440 20  0001 C CNN
F 1 "+5V" H 6600 3440 30  0000 C CNN
F 2 "" H 6600 3350 60  0000 C CNN
F 3 "" H 6600 3350 60  0000 C CNN
	1    6600 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 3550 6600 3550
Wire Wire Line
	6600 3350 6600 3800
Connection ~ 6600 3550
$Comp
L GND #PWR045
U 1 1 54B44D69
P 6600 4300
F 0 "#PWR045" H 6600 4300 30  0001 C CNN
F 1 "GND" H 6600 4230 30  0001 C CNN
F 2 "" H 6600 4300 60  0000 C CNN
F 3 "" H 6600 4300 60  0000 C CNN
	1    6600 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 4200 6600 4300
$EndSCHEMATC

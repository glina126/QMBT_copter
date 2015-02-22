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
Sheet 6 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 5050 4550 0    60   ~ 0
Note:\nThis includes I2C pullups
$Comp
L BSS138DW_array U2
U 1 1 54ADA00A
P 5900 3350
F 0 "U2" H 5550 3800 47  0000 C CNN
F 1 "BSS138DW_array" H 5750 2850 47  0000 C CNN
F 2 "footprints:BSS138DW_array" H 5900 3350 47  0000 C CNN
F 3 "" H 5900 3350 47  0000 C CNN
	1    5900 3350
	1    0    0    -1  
$EndComp
Text GLabel 6350 2850 2    47   Input ~ 0
SDA
$Comp
L R R5
U 1 1 54ADA22E
P 6150 2600
F 0 "R5" V 6230 2600 40  0000 C CNN
F 1 "1K" V 6157 2601 40  0000 C CNN
F 2 "Resistors_SMD:R_0402" V 6080 2600 30  0001 C CNN
F 3 "" H 6150 2600 30  0000 C CNN
	1    6150 2600
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 54ADA253
P 6400 3450
F 0 "R6" V 6480 3450 40  0000 C CNN
F 1 "1K" V 6407 3451 40  0000 C CNN
F 2 "Resistors_SMD:R_0402" V 6330 3450 30  0001 C CNN
F 3 "" H 6400 3450 30  0000 C CNN
	1    6400 3450
	1    0    0    -1  
$EndComp
Text GLabel 6700 3700 2    47   Input ~ 0
H_SDA
Wire Wire Line
	5900 3700 6700 3700
Connection ~ 6400 3700
$Comp
L +5V #PWR037
U 1 1 54ADA534
P 6400 3150
F 0 "#PWR037" H 6400 3240 20  0001 C CNN
F 1 "+5V" H 6400 3240 30  0000 C CNN
F 2 "" H 6400 3150 60  0000 C CNN
F 3 "" H 6400 3150 60  0000 C CNN
	1    6400 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 3200 6400 3150
Wire Wire Line
	5900 3000 5900 2850
Wire Wire Line
	5900 2850 6350 2850
Connection ~ 6150 2850
$Comp
L +3.3V #PWR038
U 1 1 54ADA8C0
P 6150 2250
F 0 "#PWR038" H 6150 2210 30  0001 C CNN
F 1 "+3.3V" H 6150 2360 30  0000 C CNN
F 2 "" H 6150 2250 60  0000 C CNN
F 3 "" H 6150 2250 60  0000 C CNN
	1    6150 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2350 6150 2250
$Comp
L R R3
U 1 1 54ADAE52
P 5100 2750
F 0 "R3" V 5180 2750 40  0000 C CNN
F 1 "1K" V 5107 2751 40  0000 C CNN
F 2 "Resistors_SMD:R_0402" V 5030 2750 30  0001 C CNN
F 3 "" H 5100 2750 30  0000 C CNN
	1    5100 2750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR039
U 1 1 54ADAE58
P 5100 2450
F 0 "#PWR039" H 5100 2540 20  0001 C CNN
F 1 "+5V" H 5100 2540 30  0000 C CNN
F 2 "" H 5100 2450 60  0000 C CNN
F 3 "" H 5100 2450 60  0000 C CNN
	1    5100 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 2500 5100 2450
Wire Wire Line
	4800 3000 5600 3000
Text GLabel 4800 3000 0    47   Input ~ 0
H_SCL
Connection ~ 5100 3000
Wire Wire Line
	5750 3000 5750 2350
Wire Wire Line
	5750 2350 6150 2350
$Comp
L R R4
U 1 1 54ADB202
P 5100 3800
F 0 "R4" V 5180 3800 40  0000 C CNN
F 1 "1K" V 5107 3801 40  0000 C CNN
F 2 "Resistors_SMD:R_0402" V 5030 3800 30  0001 C CNN
F 3 "" H 5100 3800 30  0000 C CNN
	1    5100 3800
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR040
U 1 1 54ADB208
P 5100 3450
F 0 "#PWR040" H 5100 3410 30  0001 C CNN
F 1 "+3.3V" H 5100 3560 30  0000 C CNN
F 2 "" H 5100 3450 60  0000 C CNN
F 3 "" H 5100 3450 60  0000 C CNN
	1    5100 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3550 5100 3450
Wire Wire Line
	4800 4050 5600 4050
Wire Wire Line
	5600 4050 5600 3700
Wire Wire Line
	5750 3700 5750 3950
Wire Wire Line
	5750 3950 5250 3950
Wire Wire Line
	5250 3950 5250 3550
Wire Wire Line
	5250 3550 5100 3550
Text GLabel 4800 4050 0    47   Input ~ 0
SCL
Connection ~ 5100 4050
$EndSCHEMATC

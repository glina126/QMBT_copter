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
Sheet 4 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 950  750  0    60   ~ 0
Broken out:
Text Notes 1550 750  0    60   ~ 0
A0 - A3
Text Notes 1550 850  0    60   ~ 0
MISO, MOSI, SCK
Text Notes 1550 950  0    60   ~ 0
RESET, DTR
Text Notes 1550 1050 0    60   ~ 0
SDA, SCL
Text Notes 1000 800  0    31   ~ 0
(Tiny through hole)\n
Text Notes 2350 850  0    60   ~ 0
- Wireless COMM
Text Notes 2000 750  0    60   ~ 0
- Analog Wireless COMM
Text Notes 2100 950  0    60   ~ 0
- Manual Reset
Text Notes 2000 1050 0    60   ~ 0
- I2C
Text GLabel 4600 4600 0    60   Input ~ 0
A0
Text GLabel 4600 4700 0    60   Input ~ 0
A1
Text GLabel 4600 4800 0    60   Input ~ 0
A2
Text GLabel 4600 4900 0    60   Input ~ 0
A3
Text GLabel 3250 2000 0    60   Input ~ 0
MISO
Text GLabel 3250 2550 0    60   Input ~ 0
MOSI
Text GLabel 3250 3100 0    60   Input ~ 0
SCK
Text GLabel 4600 2000 0    60   Input ~ 0
RESET
Text GLabel 4600 2550 0    60   Input ~ 0
DTR
Text GLabel 6050 2000 0    60   Input ~ 0
SDA
Text GLabel 6050 2550 0    60   Input ~ 0
SCL
$Comp
L CONN_01X01 P9
U 1 1 54A87EE4
P 3750 2000
F 0 "P9" H 3750 2100 50  0000 C CNN
F 1 "MISO" V 3850 2000 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 3750 2000 60  0001 C CNN
F 3 "" H 3750 2000 60  0000 C CNN
	1    3750 2000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P10
U 1 1 54A87EFF
P 3750 2550
F 0 "P10" H 3750 2650 50  0000 C CNN
F 1 "MOSI" V 3850 2550 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 3750 2550 60  0001 C CNN
F 3 "" H 3750 2550 60  0000 C CNN
	1    3750 2550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P11
U 1 1 54A87F38
P 3750 3100
F 0 "P11" H 3750 3200 50  0000 C CNN
F 1 "SCK" V 3850 3100 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 3750 3100 60  0001 C CNN
F 3 "" H 3750 3100 60  0000 C CNN
	1    3750 3100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P12
U 1 1 54A880AD
P 5100 2000
F 0 "P12" H 5100 2100 50  0000 C CNN
F 1 "RST" V 5200 2000 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 5100 2000 60  0001 C CNN
F 3 "" H 5100 2000 60  0000 C CNN
	1    5100 2000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P13
U 1 1 54A880CE
P 5100 2550
F 0 "P13" H 5100 2650 50  0000 C CNN
F 1 "DTR" V 5200 2550 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 5100 2550 60  0001 C CNN
F 3 "" H 5100 2550 60  0000 C CNN
	1    5100 2550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P14
U 1 1 54A8828D
P 6550 2000
F 0 "P14" H 6550 2100 50  0000 C CNN
F 1 "SDA" V 6650 2000 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 6550 2000 60  0001 C CNN
F 3 "" H 6550 2000 60  0000 C CNN
	1    6550 2000
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P15
U 1 1 54A882B2
P 6550 2550
F 0 "P15" H 6550 2650 50  0000 C CNN
F 1 "SCL" V 6650 2550 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 6550 2550 60  0001 C CNN
F 3 "" H 6550 2550 60  0000 C CNN
	1    6550 2550
	1    0    0    -1  
$EndComp
Text Notes 3450 750  0    60   ~ 0
Not Connected/broken out:\n
Text GLabel 3250 3650 0    60   Input ~ 0
D10
$Comp
L CONN_01X01 P16
U 1 1 54A88C13
P 3750 3650
F 0 "P16" H 3750 3750 50  0000 C CNN
F 1 "SS" V 3850 3650 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 3750 3650 60  0001 C CNN
F 3 "" H 3750 3650 60  0000 C CNN
	1    3750 3650
	1    0    0    -1  
$EndComp
Text Notes 2850 3550 0    60   ~ 0
Chip Select
Text Notes 1550 1150 0    60   ~ 0
INT
Text Notes 1700 1150 0    60   ~ 0
- Interrupt pin 2
Text GLabel 7200 2000 0    60   Input ~ 0
D2
Text GLabel 7500 2000 2    60   Input ~ 0
INT
Text GLabel 7250 2650 0    60   Input ~ 0
VIN
$Comp
L CONN_01X02 P17
U 1 1 54A8BF9B
P 7700 2600
F 0 "P17" H 7700 2750 50  0000 C CNN
F 1 "VCC" V 7800 2600 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02" H 7700 2600 60  0001 C CNN
F 3 "" H 7700 2600 60  0000 C CNN
	1    7700 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 54A8C031
P 7400 2850
F 0 "#PWR029" H 7400 2850 30  0001 C CNN
F 1 "GND" H 7400 2780 30  0001 C CNN
F 2 "" H 7400 2850 60  0000 C CNN
F 3 "" H 7400 2850 60  0000 C CNN
	1    7400 2850
	1    0    0    -1  
$EndComp
Text GLabel 2600 4800 0    60   Input ~ 0
D6
Text GLabel 2600 4900 0    60   Input ~ 0
D7
Text GLabel 6750 4500 0    60   Input ~ 0
D8
Text GLabel 6750 4800 0    60   Input ~ 0
D9
$Comp
L +3.3V #PWR030
U 1 1 54AA30AC
P 4500 3100
F 0 "#PWR030" H 4500 3060 30  0001 C CNN
F 1 "+3.3V" H 4500 3210 30  0000 C CNN
F 2 "" H 4500 3100 60  0000 C CNN
F 3 "" H 4500 3100 60  0000 C CNN
	1    4500 3100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P18
U 1 1 54AA3121
P 5000 3200
F 0 "P18" H 5000 3300 50  0000 C CNN
F 1 "3V3" V 5100 3200 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 5000 3200 60  0001 C CNN
F 3 "" H 5000 3200 60  0000 C CNN
	1    5000 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 54AA333A
P 4500 3550
F 0 "#PWR031" H 4500 3550 30  0001 C CNN
F 1 "GND" H 4500 3480 30  0001 C CNN
F 2 "" H 4500 3550 60  0000 C CNN
F 3 "" H 4500 3550 60  0000 C CNN
	1    4500 3550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P19
U 1 1 54AA336A
P 5000 3450
F 0 "P19" H 5000 3550 50  0000 C CNN
F 1 "GND" V 5100 3450 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 5000 3450 60  0001 C CNN
F 3 "" H 5000 3450 60  0000 C CNN
	1    5000 3450
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P20
U 1 1 54AC82F2
P 6150 3200
F 0 "P20" H 6150 3300 50  0000 C CNN
F 1 "3V3" V 6250 3200 50  0000 C CNN
F 2 "footprints:1x0.5mm_Through_hole2" H 6150 3200 60  0001 C CNN
F 3 "" H 6150 3200 60  0000 C CNN
	1    6150 3200
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR032
U 1 1 54AC8332
P 5650 3050
F 0 "#PWR032" H 5650 3140 20  0001 C CNN
F 1 "+5V" H 5650 3140 30  0000 C CNN
F 2 "" H 5650 3050 60  0000 C CNN
F 3 "" H 5650 3050 60  0000 C CNN
	1    5650 3050
	1    0    0    -1  
$EndComp
$Comp
L 6pin_1.27mm C13
U 1 1 54AD8144
P 9050 2400
F 0 "C13" H 9050 2900 60  0000 C CNN
F 1 "6pin_1.27mm" H 9050 2050 60  0000 C CNN
F 2 "footprints:6pin_1.27mm" H 9050 2400 60  0000 C CNN
F 3 "" H 9050 2400 60  0000 C CNN
	1    9050 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 54AD8291
P 8600 2900
F 0 "#PWR033" H 8600 2900 30  0001 C CNN
F 1 "GND" H 8600 2830 30  0001 C CNN
F 2 "" H 8600 2900 60  0000 C CNN
F 3 "" H 8600 2900 60  0000 C CNN
	1    8600 2900
	1    0    0    -1  
$EndComp
Text GLabel 8450 2350 0    47   Input ~ 0
RXI
Text GLabel 8450 2450 0    47   Input ~ 0
TXD
Text GLabel 8450 2550 0    47   Input ~ 0
DTR
$Comp
L +5V #PWR034
U 1 1 54AED4DF
P 8450 2050
F 0 "#PWR034" H 8450 2140 20  0001 C CNN
F 1 "+5V" H 8450 2140 30  0000 C CNN
F 2 "" H 8450 2050 60  0000 C CNN
F 3 "" H 8450 2050 60  0000 C CNN
	1    8450 2050
	1    0    0    -1  
$EndComp
$Comp
L conn_3x4 U3
U 1 1 54B43F48
P 3000 4900
F 0 "U3" H 3000 5350 60  0000 C CNN
F 1 "conn_3x4" H 3350 4450 60  0000 C CNN
F 2 "footprints:conn_3x4" H 3300 4250 60  0000 C CNN
F 3 "" H 3000 4900 60  0000 C CNN
	1    3000 4900
	1    0    0    -1  
$EndComp
Text Notes 3000 4300 0    59   ~ 0
Reciever connector
$Comp
L conn_3x4 U4
U 1 1 54B43F9C
P 4950 4900
F 0 "U4" H 4950 5350 60  0000 C CNN
F 1 "conn_3x4" H 5300 4450 60  0000 C CNN
F 2 "footprints:conn_3x4" H 4950 4900 60  0000 C CNN
F 3 "" H 4950 4900 60  0000 C CNN
	1    4950 4900
	1    0    0    -1  
$EndComp
Text Notes 4950 4300 0    59   ~ 0
Motor Connectors
$Comp
L GND #PWR035
U 1 1 54B440F3
P 2850 5350
F 0 "#PWR035" H 2850 5350 30  0001 C CNN
F 1 "GND" H 2850 5280 30  0001 C CNN
F 2 "" H 2850 5350 60  0000 C CNN
F 3 "" H 2850 5350 60  0000 C CNN
	1    2850 5350
	1    0    0    -1  
$EndComp
Text GLabel 4600 5000 0    59   Input ~ 0
VIN
$Comp
L GND #PWR036
U 1 1 54B446A5
P 4850 5300
F 0 "#PWR036" H 4850 5300 30  0001 C CNN
F 1 "GND" H 4850 5230 30  0001 C CNN
F 2 "" H 4850 5300 60  0000 C CNN
F 3 "" H 4850 5300 60  0000 C CNN
	1    4850 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2000 3550 2000
Wire Wire Line
	3550 2550 3250 2550
Wire Wire Line
	3250 3100 3550 3100
Wire Wire Line
	4600 2000 4900 2000
Wire Wire Line
	4900 2550 4600 2550
Wire Wire Line
	6050 2000 6350 2000
Wire Wire Line
	6350 2550 6050 2550
Wire Wire Line
	3250 3650 3550 3650
Wire Wire Line
	7200 2000 7500 2000
Wire Wire Line
	7400 2550 7400 2850
Wire Wire Line
	7400 2550 7500 2550
Wire Wire Line
	7250 2650 7500 2650
Wire Wire Line
	4800 3200 4500 3200
Wire Wire Line
	4500 3200 4500 3100
Wire Wire Line
	4800 3450 4500 3450
Wire Wire Line
	4500 3450 4500 3550
Wire Wire Line
	5950 3200 5650 3200
Wire Wire Line
	5650 3200 5650 3050
Wire Wire Line
	8850 2550 8450 2550
Wire Wire Line
	8850 2450 8450 2450
Wire Wire Line
	8450 2250 8850 2250
Wire Wire Line
	8450 2350 8850 2350
Wire Wire Line
	8850 2150 8600 2150
Wire Wire Line
	8600 2050 8600 2900
Wire Wire Line
	8850 2050 8600 2050
Connection ~ 8600 2150
Wire Wire Line
	8450 2050 8450 2250
Wire Wire Line
	2600 4800 3000 4800
Wire Wire Line
	3000 4900 2600 4900
Wire Wire Line
	2300 5000 3000 5000
Wire Wire Line
	3000 5100 2850 5100
Wire Wire Line
	2850 5100 2850 5350
Wire Wire Line
	4600 4600 4950 4600
Wire Wire Line
	4950 4700 4600 4700
Wire Wire Line
	4600 4800 4950 4800
Wire Wire Line
	4950 4900 4600 4900
Wire Wire Line
	4950 5000 4600 5000
Wire Wire Line
	4950 5100 4850 5100
Wire Wire Line
	4850 5100 4850 5300
NoConn ~ 6750 4500
NoConn ~ 6750 4800
Text GLabel 2600 4700 0    60   Input ~ 0
D4
Wire Wire Line
	2600 4700 3000 4700
Text GLabel 2600 4600 0    59   Input ~ 0
D3
Wire Wire Line
	2600 4600 3000 4600
$Comp
L +5V #PWR?
U 1 1 54B49B40
P 2300 4850
F 0 "#PWR?" H 2300 4940 20  0001 C CNN
F 1 "+5V" H 2300 4940 30  0000 C CNN
F 2 "" H 2300 4850 60  0000 C CNN
F 3 "" H 2300 4850 60  0000 C CNN
	1    2300 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 5000 2300 4850
$EndSCHEMATC

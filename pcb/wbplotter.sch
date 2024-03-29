EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L esp32:ESP32 U1
U 1 1 5E19F2C6
P 3950 6050
F 0 "U1" V 2885 6100 50  0000 C CNN
F 1 "ESP32" V 2976 6100 50  0000 C CNN
F 2 "esp32:ESP32-Dev-Board" H 3300 6200 50  0001 C CNN
F 3 "" H 3300 6200 50  0001 C CNN
	1    3950 6050
	0    1    1    0   
$EndComp
$Comp
L esp32:DRV8825 U3
U 1 1 5E1A1C57
P 6800 5450
F 0 "U3" H 6775 6065 50  0000 C CNN
F 1 "DRV8825" H 6775 5974 50  0000 C CNN
F 2 "esp32:DRV8825" H 6800 5450 50  0001 C CNN
F 3 "" H 6800 5450 50  0001 C CNN
	1    6800 5450
	1    0    0    -1  
$EndComp
$Comp
L esp32:DRV8825 U2
U 1 1 5E1A2A23
P 6800 4000
F 0 "U2" H 6775 4615 50  0000 C CNN
F 1 "DRV8825" H 6775 4524 50  0000 C CNN
F 2 "esp32:DRV8825" H 6800 4000 50  0001 C CNN
F 3 "" H 6800 4000 50  0001 C CNN
	1    6800 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J5
U 1 1 5E1A7119
P 4600 3050
F 0 "J5" H 4708 3331 50  0000 C CNN
F 1 "M0" H 4708 3240 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 4600 3050 50  0001 C CNN
F 3 "~" H 4600 3050 50  0001 C CNN
	1    4600 3050
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Male J6
U 1 1 5E1B9C01
P 5200 3050
F 0 "J6" H 5308 3331 50  0000 C CNN
F 1 "M1" H 5308 3240 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 5200 3050 50  0001 C CNN
F 3 "~" H 5200 3050 50  0001 C CNN
	1    5200 3050
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x03_Male J7
U 1 1 5E1B9F36
P 5800 3050
F 0 "J7" H 5908 3331 50  0000 C CNN
F 1 "M2" H 5908 3240 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 5800 3050 50  0001 C CNN
F 3 "~" H 5800 3050 50  0001 C CNN
	1    5800 3050
	0    1    1    0   
$EndComp
Text GLabel 3400 5600 0    50   Input ~ 0
GND
Text GLabel 4600 5600 2    50   Input ~ 0
GND
Text GLabel 4700 3250 3    50   Input ~ 0
GND
Text GLabel 5300 3250 3    50   Input ~ 0
GND
Text GLabel 5900 3250 3    50   Input ~ 0
GND
Text GLabel 3400 5500 0    50   Input ~ 0
3.3V
Text GLabel 4500 3250 3    50   Input ~ 0
3.3V
Text GLabel 5100 3250 3    50   Input ~ 0
3.3V
Text GLabel 5700 3250 3    50   Input ~ 0
3.3V
Wire Wire Line
	6300 3750 4600 3750
Wire Wire Line
	4600 3750 4600 3250
Wire Wire Line
	6300 3850 5200 3850
Wire Wire Line
	5200 3850 5200 3250
Wire Wire Line
	6300 3950 5800 3950
Wire Wire Line
	5800 3950 5800 3250
Wire Wire Line
	6300 5200 4600 5200
Wire Wire Line
	4600 5200 4600 3750
Connection ~ 4600 3750
Wire Wire Line
	6300 5300 5200 5300
Wire Wire Line
	5200 5300 5200 3850
Connection ~ 5200 3850
Wire Wire Line
	6300 5400 5800 5400
Wire Wire Line
	5800 5400 5800 3950
Connection ~ 5800 3950
Text GLabel 7250 3750 2    50   Input ~ 0
GND
Text GLabel 7250 4350 2    50   Input ~ 0
GND
Text GLabel 7250 5200 2    50   Input ~ 0
GND
Text GLabel 7250 5800 2    50   Input ~ 0
GND
$Comp
L Connector:Conn_01x04_Male J8
U 1 1 5E1CF712
P 7450 4050
F 0 "J8" H 7422 3932 50  0000 R CNN
F 1 "MotorLeft" H 7422 4023 50  0000 R CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7450 4050 50  0001 C CNN
F 3 "~" H 7450 4050 50  0001 C CNN
	1    7450 4050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male J9
U 1 1 5E1D9CC6
P 7450 5500
F 0 "J9" H 7422 5382 50  0000 R CNN
F 1 "MotorRight" H 7422 5473 50  0000 R CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 7450 5500 50  0001 C CNN
F 3 "~" H 7450 5500 50  0001 C CNN
	1    7450 5500
	-1   0    0    1   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J10
U 1 1 5E1DAA45
P 8450 3650
F 0 "J10" H 8530 3642 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 8530 3551 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 8450 3650 50  0001 C CNN
F 3 "~" H 8450 3650 50  0001 C CNN
	1    8450 3650
	1    0    0    -1  
$EndComp
Text GLabel 8250 3650 0    50   Input ~ 0
GND
Wire Wire Line
	8250 3750 7700 3750
Wire Wire Line
	7700 3750 7700 3650
Wire Wire Line
	7700 3650 7500 3650
Wire Wire Line
	7700 5100 7700 3750
Connection ~ 7700 3750
Wire Wire Line
	3400 6400 3300 6400
Wire Wire Line
	3300 6400 3300 7150
Wire Wire Line
	3300 7150 5900 7150
Wire Wire Line
	5900 7150 5900 4250
Wire Wire Line
	5900 4250 6300 4250
Wire Wire Line
	6300 4350 5950 4350
Wire Wire Line
	5950 4350 5950 7200
Wire Wire Line
	5950 7200 3250 7200
Wire Wire Line
	3250 7200 3250 6300
Wire Wire Line
	3250 6300 3400 6300
Wire Wire Line
	3400 6100 3150 6100
Wire Wire Line
	3150 6100 3150 7250
Wire Wire Line
	3150 7250 6000 7250
Wire Wire Line
	6000 7250 6000 5700
Wire Wire Line
	6000 5700 6300 5700
Wire Wire Line
	6300 5800 6050 5800
Wire Wire Line
	6050 5800 6050 7300
Wire Wire Line
	6050 7300 3100 7300
Wire Wire Line
	3100 7300 3100 6000
Wire Wire Line
	3100 6000 3400 6000
Wire Wire Line
	4600 6400 5850 6400
Wire Wire Line
	5850 6400 5850 5600
Wire Wire Line
	5850 5600 6300 5600
Wire Wire Line
	6300 4150 5850 4150
Wire Wire Line
	5850 4150 5850 5600
Connection ~ 5850 5600
Wire Wire Line
	6300 4050 5700 4050
Wire Wire Line
	5700 4050 5700 5500
Wire Wire Line
	5700 6300 4600 6300
Wire Wire Line
	6300 5500 5700 5500
Connection ~ 5700 5500
Wire Wire Line
	5700 5500 5700 6300
Wire Wire Line
	6300 3650 5600 3650
Wire Wire Line
	5600 3650 5600 5100
Wire Wire Line
	5600 5800 4600 5800
Wire Wire Line
	6300 5100 5600 5100
Connection ~ 5600 5100
Wire Wire Line
	5600 5100 5600 5800
Wire Wire Line
	7250 5700 7500 5700
Wire Wire Line
	7500 5700 7500 5950
Wire Wire Line
	7500 5950 6150 5950
Wire Wire Line
	6150 5950 6150 6700
Wire Wire Line
	6150 6700 4600 6700
Wire Wire Line
	4600 6800 6200 6800
Wire Wire Line
	6200 6800 6200 6000
Wire Wire Line
	6200 6000 8000 6000
Wire Wire Line
	8000 6000 8000 4250
Wire Wire Line
	8000 4250 7250 4250
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5E1ECE4F
P 2200 4250
F 0 "J2" H 2308 4531 50  0000 C CNN
F 1 "5V" H 2308 4440 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 2200 4250 50  0001 C CNN
F 3 "~" H 2200 4250 50  0001 C CNN
	1    2200 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5E1EE1D6
P 2750 4250
F 0 "J3" H 2858 4531 50  0000 C CNN
F 1 "3.3V" H 2858 4440 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 2750 4250 50  0001 C CNN
F 3 "~" H 2750 4250 50  0001 C CNN
	1    2750 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J4
U 1 1 5E1EF4EE
P 3300 4250
F 0 "J4" H 3408 4531 50  0000 C CNN
F 1 "GND" H 3408 4440 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 3300 4250 50  0001 C CNN
F 3 "~" H 3300 4250 50  0001 C CNN
	1    3300 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5500 4800 4650
Wire Wire Line
	4800 4650 2400 4650
Wire Wire Line
	2400 4650 2400 4450
Connection ~ 2400 4250
Wire Wire Line
	2400 4250 2400 4150
Connection ~ 2400 4350
Wire Wire Line
	2400 4350 2400 4250
Connection ~ 2400 4450
Wire Wire Line
	2400 4450 2400 4350
Wire Wire Line
	2950 4150 2950 4250
Connection ~ 2950 4250
Wire Wire Line
	2950 4250 2950 4350
Connection ~ 2950 4350
Wire Wire Line
	2950 4350 2950 4450
Text GLabel 2950 4450 2    50   Input ~ 0
3.3V
Text GLabel 3500 4450 2    50   Input ~ 0
GND
Wire Wire Line
	3500 4150 3500 4250
Connection ~ 3500 4250
Wire Wire Line
	3500 4250 3500 4350
Connection ~ 3500 4350
Wire Wire Line
	3500 4350 3500 4450
$Comp
L Connector:Conn_01x12_Male J1
U 1 1 5E1FB713
P 1900 6200
F 0 "J1" H 2008 6881 50  0000 C CNN
F 1 "Conn_01x12_Male" H 2008 6790 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x12_P2.54mm_Vertical" H 1900 6200 50  0001 C CNN
F 3 "~" H 1900 6200 50  0001 C CNN
	1    1900 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5700 2100 5700
Wire Wire Line
	3400 5800 2100 5800
Wire Wire Line
	3400 5900 2100 5900
Wire Wire Line
	3400 6200 2850 6200
Wire Wire Line
	2850 6200 2850 6000
Wire Wire Line
	2850 6000 2100 6000
Wire Wire Line
	3400 6500 2750 6500
Wire Wire Line
	2750 6500 2750 6100
Wire Wire Line
	2750 6100 2100 6100
Wire Wire Line
	3400 6800 2600 6800
Wire Wire Line
	2600 6800 2600 6200
Wire Wire Line
	2600 6200 2100 6200
Wire Wire Line
	3400 6900 2550 6900
Wire Wire Line
	2550 6900 2550 6300
Wire Wire Line
	2550 6300 2100 6300
Wire Wire Line
	4600 5700 4800 5700
Wire Wire Line
	4800 5700 4800 7350
Wire Wire Line
	4800 7350 2500 7350
Wire Wire Line
	2500 7350 2500 6400
Wire Wire Line
	2500 6400 2100 6400
Wire Wire Line
	4600 5900 4850 5900
Wire Wire Line
	4850 5900 4850 7400
Wire Wire Line
	4850 7400 2450 7400
Wire Wire Line
	2450 7400 2450 6500
Wire Wire Line
	2450 6500 2100 6500
Wire Wire Line
	4600 6000 4900 6000
Wire Wire Line
	4900 6000 4900 7450
Wire Wire Line
	4900 7450 2400 7450
Wire Wire Line
	2400 7450 2400 6600
Wire Wire Line
	2400 6600 2100 6600
Wire Wire Line
	4600 6100 4950 6100
Wire Wire Line
	4950 6100 4950 7500
Wire Wire Line
	4950 7500 2350 7500
Wire Wire Line
	2350 7500 2350 6700
Wire Wire Line
	2350 6700 2100 6700
Wire Wire Line
	4600 6600 5000 6600
Wire Wire Line
	5000 6600 5000 7550
Wire Wire Line
	5000 7550 2300 7550
Wire Wire Line
	2300 7550 2300 6800
Wire Wire Line
	2300 6800 2100 6800
$Comp
L Device:CP1_Small C3
U 1 1 5E228667
P 7500 3550
F 0 "C3" H 7409 3504 50  0000 R CNN
F 1 "Cap" H 7409 3595 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 7500 3550 50  0001 C CNN
F 3 "~" H 7500 3550 50  0001 C CNN
	1    7500 3550
	-1   0    0    1   
$EndComp
$Comp
L Device:CP1_Small C2
U 1 1 5E231C2A
P 7450 5000
F 0 "C2" H 7359 4954 50  0000 R CNN
F 1 "Cap" H 7359 5045 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 7450 5000 50  0001 C CNN
F 3 "~" H 7450 5000 50  0001 C CNN
	1    7450 5000
	-1   0    0    1   
$EndComp
Text GLabel 7500 3450 1    50   Input ~ 0
GND
Text GLabel 7450 4900 1    50   Input ~ 0
GND
$Comp
L Device:CP1_Small C1
U 1 1 5E243BD6
P 2750 5150
F 0 "C1" H 2659 5104 50  0000 R CNN
F 1 "Cap" H 2659 5195 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 2750 5150 50  0001 C CNN
F 3 "~" H 2750 5150 50  0001 C CNN
	1    2750 5150
	-1   0    0    1   
$EndComp
Text GLabel 2750 5050 1    50   Input ~ 0
GND
Text GLabel 2750 5250 3    50   Input ~ 0
3.3V
Wire Wire Line
	7700 5100 7450 5100
Wire Wire Line
	7250 5100 7450 5100
Connection ~ 7450 5100
Wire Wire Line
	4600 5500 4800 5500
NoConn ~ 4600 6200
NoConn ~ 4600 6500
NoConn ~ 4600 6900
NoConn ~ 3400 6700
NoConn ~ 3400 6600
Wire Wire Line
	8250 4050 8250 3750
Connection ~ 8250 3750
Wire Wire Line
	7250 3650 7500 3650
Connection ~ 7500 3650
Text GLabel 8400 5550 0    50   Input ~ 0
GND
Wire Wire Line
	8400 5550 8500 5550
Wire Wire Line
	8500 5550 8500 5650
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5E29FD57
P 8250 4050
F 0 "#FLG01" H 8250 4125 50  0001 C CNN
F 1 "PWR_FLAG" H 8250 4223 50  0000 C CNN
F 2 "" H 8250 4050 50  0001 C CNN
F 3 "~" H 8250 4050 50  0001 C CNN
	1    8250 4050
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E2A7924
P 8500 5650
F 0 "#FLG02" H 8500 5725 50  0001 C CNN
F 1 "PWR_FLAG" H 8500 5823 50  0000 C CNN
F 2 "" H 8500 5650 50  0001 C CNN
F 3 "~" H 8500 5650 50  0001 C CNN
	1    8500 5650
	-1   0    0    1   
$EndComp
$EndSCHEMATC

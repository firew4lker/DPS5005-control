EESchema Schematic File Version 3
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
LIBS:w_analog
LIBS:w_connectors
LIBS:w_device
LIBS:w_logic
LIBS:w_memory
LIBS:w_microcontrollers
LIBS:w_opto
LIBS:w_relay
LIBS:w_rtx
LIBS:w_transistor
LIBS:w_vacuum
LIBS:dps5005control-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "PDS-5005c uart Control Board"
Date "2017-09-03"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY2313-20PU U2
U 1 1 59ABC6E6
P 3875 2350
F 0 "U2" H 4025 3500 50  0000 C CNN
F 1 "ATTINY2313-20PU" H 4325 3400 50  0000 C CNN
F 2 "w_pth_circuits:dil_20-300_socket" H 3875 2350 50  0001 C CIN
F 3 "" H 3875 2350 50  0001 C CNN
	1    3875 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 59ABC7E4
P 3875 3450
F 0 "#PWR01" H 3875 3200 50  0001 C CNN
F 1 "GND" H 3880 3277 50  0000 C CNN
F 2 "" H 3875 3450 50  0001 C CNN
F 3 "" H 3875 3450 50  0001 C CNN
	1    3875 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3875 3450 3875 3350
$Comp
L CONN_4 P2
U 1 1 59ABD3B5
P 2175 2825
F 0 "P2" H 2303 2866 50  0000 L CNN
F 1 "CONN_4" H 2303 2775 50  0000 L CNN
F 2 "w_conn_df13:df13-4p-125dsa" H 2175 2825 60  0001 C CNN
F 3 "" H 2175 2825 60  0001 C CNN
	1    2175 2825
	1    0    0    1   
$EndComp
Wire Wire Line
	5025 2450 5225 2450
Text Label 5225 2550 0    60   ~ 0
Tx
Text Label 5225 2450 0    60   ~ 0
Rx
Wire Wire Line
	5025 2550 5225 2550
$Comp
L CONN_5X2 P3
U 1 1 59ABD882
P 6650 3000
F 0 "P3" H 6650 3443 60  0000 C CNN
F 1 "CONN_5X2" H 6650 3345 50  0000 C CNN
F 2 "w_conn_strip:vasch_strip_5x2" H 6650 3000 60  0001 C CNN
F 3 "" H 6650 3000 60  0001 C CNN
	1    6650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2900 7150 2900
Wire Wire Line
	7150 2900 7150 3300
Wire Wire Line
	7050 3200 7400 3200
Connection ~ 7150 3200
Wire Wire Line
	7050 3100 7150 3100
Connection ~ 7150 3100
Wire Wire Line
	7050 3000 7150 3000
Connection ~ 7150 3000
$Comp
L GND #PWR06
U 1 1 59ABDA21
P 7150 3300
F 0 "#PWR06" H 7150 3300 30  0001 C CNN
F 1 "GND" H 7150 3230 30  0001 C CNN
F 2 "" H 7150 3300 60  0001 C CNN
F 3 "" H 7150 3300 60  0001 C CNN
	1    7150 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2800 6150 2800
Wire Wire Line
	6250 3000 6150 3000
Wire Wire Line
	6250 3100 6150 3100
Wire Wire Line
	6250 3200 6150 3200
NoConn ~ 6250 2900
Wire Wire Line
	7050 2800 7150 2800
Text Label 6150 2800 2    60   ~ 0
MOSI
Text Label 6150 3000 2    60   ~ 0
RST
Text Label 6150 3100 2    60   ~ 0
SCK
Text Label 6150 3200 2    60   ~ 0
MISO
Wire Wire Line
	5025 2150 5225 2150
Text Label 5225 2150 0    60   ~ 0
MISO
Wire Wire Line
	5025 2050 5225 2050
Text Label 5225 2050 0    60   ~ 0
MOSI
Text Label 5225 2250 0    60   ~ 0
SCK
Wire Wire Line
	5025 2250 5225 2250
Wire Wire Line
	2525 1550 2725 1550
Text Label 2525 1550 2    60   ~ 0
RST
$Comp
L R R1
U 1 1 59ABE305
P 3275 1150
F 0 "R1" V 3175 1150 50  0000 L CNN
F 1 "10k" V 3375 1125 50  0000 L CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3205 1150 50  0001 C CNN
F 3 "" H 3275 1150 50  0001 C CNN
	1    3275 1150
	0    1    1    0   
$EndComp
Wire Wire Line
	2675 1150 2675 1550
Connection ~ 2675 1550
Wire Wire Line
	3875 900  3875 1250
Wire Wire Line
	5025 1550 5225 1550
Wire Wire Line
	5025 1650 5225 1650
Wire Wire Line
	5025 1750 5225 1750
Wire Wire Line
	5025 1850 5225 1850
Wire Wire Line
	5025 1950 5225 1950
Wire Wire Line
	5025 3050 5225 3050
Text Label 5225 3050 0    60   ~ 0
ENC1_SW
Text Label 5225 1750 0    60   ~ 0
ENC2_SW
Text Label 5225 1550 0    60   ~ 0
ENC1_A
Text Label 5225 1650 0    60   ~ 0
ENC1_B
Text Label 5225 1850 0    60   ~ 0
ENC2_A
Text Label 5225 1950 0    60   ~ 0
ENC2_B
NoConn ~ 2725 1850
NoConn ~ 2725 2050
Wire Wire Line
	5025 2750 5225 2750
Text Label 5225 2750 0    60   ~ 0
CC_MODE
$Comp
L R R2
U 1 1 59AC02EA
P 1625 1600
F 0 "R2" V 1418 1600 50  0000 C CNN
F 1 "1k" V 1509 1600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1555 1600 50  0001 C CNN
F 3 "" H 1625 1600 50  0001 C CNN
	1    1625 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	1775 1600 1925 1600
Wire Wire Line
	1475 1600 1325 1600
Text Label 1325 1600 2    60   ~ 0
CC_MODE
$Comp
L GND #PWR011
U 1 1 59AC07C6
P 2275 1750
F 0 "#PWR011" H 2275 1750 30  0001 C CNN
F 1 "GND" H 2275 1680 30  0001 C CNN
F 2 "" H 2275 1750 60  0001 C CNN
F 3 "" H 2275 1750 60  0001 C CNN
	1    2275 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 1600 2275 1600
Wire Wire Line
	2275 1600 2275 1750
$Comp
L LED_Small_ALT D1
U 1 1 59AC0B63
P 2025 1600
F 0 "D1" H 2025 1835 50  0000 C CNN
F 1 "LED_Small_ALT" H 2025 1744 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" V 2025 1600 50  0001 C CNN
F 3 "" V 2025 1600 50  0001 C CNN
	1    2025 1600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1825 2675 1725 2675
Wire Wire Line
	1725 2675 1725 2575
Wire Wire Line
	1825 2975 1725 2975
Wire Wire Line
	1725 2975 1725 3075
$Comp
L GND #PWR012
U 1 1 59AC0F5A
P 1725 3075
F 0 "#PWR012" H 1725 2825 50  0001 C CNN
F 1 "GND" H 1730 2902 50  0000 C CNN
F 2 "" H 1725 3075 50  0001 C CNN
F 3 "" H 1725 3075 50  0001 C CNN
	1    1725 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	1825 2775 1725 2775
Wire Wire Line
	1825 2875 1725 2875
Text Label 1725 2875 2    60   ~ 0
Rx
Text Label 1725 2775 2    60   ~ 0
Tx
NoConn ~ 5025 2850
NoConn ~ 5025 2950
NoConn ~ 5025 2650
$Comp
L R R3
U 1 1 59AD66A9
P 1625 2150
F 0 "R3" V 1418 2150 50  0000 C CNN
F 1 "1k" V 1509 2150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 1555 2150 50  0001 C CNN
F 3 "" H 1625 2150 50  0001 C CNN
	1    1625 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	1775 2150 1925 2150
Wire Wire Line
	1025 2150 1475 2150
$Comp
L GND #PWR014
U 1 1 59AD66B3
P 2275 2300
F 0 "#PWR014" H 2275 2300 30  0001 C CNN
F 1 "GND" H 2275 2230 30  0001 C CNN
F 2 "" H 2275 2300 60  0001 C CNN
F 3 "" H 2275 2300 60  0001 C CNN
	1    2275 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 2150 2275 2150
Wire Wire Line
	2275 2150 2275 2300
$Comp
L LED_Small_ALT D2
U 1 1 59AD66BB
P 2025 2150
F 0 "D2" H 2025 2385 50  0000 C CNN
F 1 "LED_Small_ALT" H 2025 2294 50  0000 C CNN
F 2 "LEDs:LED_D5.0mm" V 2025 2150 50  0001 C CNN
F 3 "" V 2025 2150 50  0001 C CNN
	1    2025 2150
	-1   0    0    -1  
$EndComp
$Comp
L CONN_7 P4
U 1 1 59AE78AB
P 6275 1825
F 0 "P4" H 6453 1873 60  0000 L CNN
F 1 "CONN_7" H 6453 1767 60  0000 L CNN
F 2 "w_pin_strip:pin_strip_7" H 6275 1825 60  0001 C CNN
F 3 "" H 6275 1825 60  0001 C CNN
	1    6275 1825
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6625 1625 6825 1625
Wire Wire Line
	6625 1725 6825 1725
Wire Wire Line
	6625 1825 6825 1825
Wire Wire Line
	6625 1925 6825 1925
Wire Wire Line
	6625 2025 6825 2025
Text Label 6825 1825 0    60   ~ 0
ENC2_SW
Text Label 6825 1625 0    60   ~ 0
ENC1_A
Text Label 6825 1725 0    60   ~ 0
ENC1_B
Text Label 6825 1925 0    60   ~ 0
ENC2_A
Text Label 6825 2025 0    60   ~ 0
ENC2_B
Wire Wire Line
	6625 1525 6825 1525
Text Label 6825 1525 0    60   ~ 0
ENC1_SW
$Comp
L GND #PWR015
U 1 1 59AE7D7C
P 6825 2325
F 0 "#PWR015" H 6825 2325 30  0001 C CNN
F 1 "GND" H 6825 2255 30  0001 C CNN
F 2 "" H 6825 2325 60  0001 C CNN
F 3 "" H 6825 2325 60  0001 C CNN
	1    6825 2325
	1    0    0    -1  
$EndComp
Wire Wire Line
	6625 2125 6825 2125
Wire Wire Line
	6825 2125 6825 2325
$Comp
L +3,3V #PWR02
U 1 1 59DA2C26
P 1725 2575
F 0 "#PWR02" H 1725 2535 30  0001 C CNN
F 1 "+3,3V" H 1734 2713 30  0000 C CNN
F 2 "" H 1725 2575 60  0001 C CNN
F 3 "" H 1725 2575 60  0001 C CNN
	1    1725 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	1325 2150 1325 2050
$Comp
L +3,3V #PWR03
U 1 1 59DA2D0C
P 1325 2050
F 0 "#PWR03" H 1325 2010 30  0001 C CNN
F 1 "+3,3V" H 1334 2188 30  0000 C CNN
F 2 "" H 1325 2050 60  0001 C CNN
F 3 "" H 1325 2050 60  0001 C CNN
	1    1325 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 2800 7150 2700
$Comp
L +3,3V #PWR04
U 1 1 59DA2E6B
P 7150 2700
F 0 "#PWR04" H 7150 2660 30  0001 C CNN
F 1 "+3,3V" H 7159 2838 30  0000 C CNN
F 2 "" H 7150 2700 60  0001 C CNN
F 3 "" H 7150 2700 60  0001 C CNN
	1    7150 2700
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR05
U 1 1 59DA3184
P 3875 900
F 0 "#PWR05" H 3875 860 30  0001 C CNN
F 1 "+3,3V" H 3884 1038 30  0000 C CNN
F 2 "" H 3875 900 60  0001 C CNN
F 3 "" H 3875 900 60  0001 C CNN
	1    3875 900 
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG01
U 1 1 59DA330D
P 1025 2125
F 0 "#FLG01" H 1025 2395 30  0001 C CNN
F 1 "PWR_FLAG" H 1025 2383 30  0000 C CNN
F 2 "" H 1025 2125 60  0001 C CNN
F 3 "" H 1025 2125 60  0001 C CNN
	1    1025 2125
	1    0    0    -1  
$EndComp
Wire Wire Line
	1025 2125 1025 2150
Connection ~ 1325 2150
$Comp
L PWR_FLAG #FLG02
U 1 1 59DA35AC
P 7400 3125
F 0 "#FLG02" H 7400 3395 30  0001 C CNN
F 1 "PWR_FLAG" H 7400 3383 30  0000 C CNN
F 2 "" H 7400 3125 60  0001 C CNN
F 3 "" H 7400 3125 60  0001 C CNN
	1    7400 3125
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3200 7400 3125
$Comp
L CP1_Small C1
U 1 1 59DA4007
P 1175 2825
F 0 "C1" H 1266 2871 50  0000 L CNN
F 1 "100u" H 1266 2780 50  0000 L CNN
F 2 "w_capacitors:CP_6.3x11mm" H 1175 2825 50  0001 C CNN
F 3 "" H 1175 2825 50  0001 C CNN
	1    1175 2825
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 59DA420B
P 1175 3075
F 0 "#PWR08" H 1175 2825 50  0001 C CNN
F 1 "GND" H 1180 2902 50  0000 C CNN
F 2 "" H 1175 3075 50  0001 C CNN
F 3 "" H 1175 3075 50  0001 C CNN
	1    1175 3075
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR07
U 1 1 59DA422A
P 1175 2575
F 0 "#PWR07" H 1175 2535 30  0001 C CNN
F 1 "+3,3V" H 1184 2713 30  0000 C CNN
F 2 "" H 1175 2575 60  0001 C CNN
F 3 "" H 1175 2575 60  0001 C CNN
	1    1175 2575
	1    0    0    -1  
$EndComp
Wire Wire Line
	1175 2575 1175 2725
Wire Wire Line
	1175 2925 1175 3075
Wire Wire Line
	3425 1150 3875 1150
Connection ~ 3875 1150
Wire Wire Line
	2675 1150 3125 1150
$EndSCHEMATC

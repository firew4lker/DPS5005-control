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
LIBS:rotaryboard-cache
EELAYER 26 0
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
L Encoder_switch ENC1
U 1 1 59AE86C5
P 2550 2100
F 0 "ENC1" H 2422 2191 40  0000 R CNN
F 1 "Encoder_switch" H 2422 2115 40  0000 R CNN
F 2 "w_misc_comp:encoder_alps-ec12d" H 2422 2024 60  0001 R CNN
F 3 "" H 2550 2100 60  0000 C CNN
	1    2550 2100
	1    0    0    -1  
$EndComp
$Comp
L Encoder_switch ENC2
U 1 1 59AE86CC
P 3800 2100
F 0 "ENC2" H 3672 2191 40  0000 R CNN
F 1 "Encoder_switch" H 3672 2115 40  0000 R CNN
F 2 "w_misc_comp:encoder_alps-ec12d" H 3672 2024 60  0001 R CNN
F 3 "" H 3800 2100 60  0000 C CNN
	1    3800 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2250 2550 2350
Wire Wire Line
	3800 2250 3800 2350
Text Label 2800 2050 0    60   ~ 0
_ENC1_A
Text Label 2800 2150 0    60   ~ 0
_ENC1_B
Wire Wire Line
	2700 2150 2800 2150
Wire Wire Line
	2700 2050 2800 2050
Text Label 4050 2050 0    60   ~ 0
_ENC2_A
Text Label 4050 2150 0    60   ~ 0
_ENC2_B
Wire Wire Line
	3950 2050 4050 2050
Wire Wire Line
	3950 2150 4050 2150
Text Label 2400 1850 2    60   ~ 0
_ENC1_SW
Wire Wire Line
	2400 1850 2500 1850
Wire Wire Line
	2500 1850 2500 1950
Wire Wire Line
	2600 1950 2600 1850
Wire Wire Line
	2600 1850 2700 1850
Wire Wire Line
	3850 1950 3850 1850
Wire Wire Line
	3850 1850 3950 1850
Wire Wire Line
	3750 1950 3750 1850
Wire Wire Line
	3750 1850 3650 1850
Text Label 3950 1850 0    60   ~ 0
_ENC2_SW
Wire Wire Line
	3200 2950 3400 2950
Wire Wire Line
	3200 3050 3400 3050
Wire Wire Line
	3200 3150 3400 3150
Wire Wire Line
	3200 3250 3400 3250
Wire Wire Line
	3200 3350 3400 3350
Text Label 3400 3150 0    60   ~ 0
_ENC2_SW
Text Label 3400 2950 0    60   ~ 0
_ENC1_A
Text Label 3400 3050 0    60   ~ 0
_ENC1_B
Text Label 3400 3250 0    60   ~ 0
_ENC2_A
Text Label 3400 3350 0    60   ~ 0
_ENC2_B
Wire Wire Line
	3200 2850 3400 2850
Text Label 3400 2850 0    60   ~ 0
_ENC1_SW
Wire Wire Line
	3200 3450 3400 3450
$Comp
L CONN_01X07 J1
U 1 1 59AE7E57
P 3000 3150
F 0 "J1" H 2919 2625 50  0000 C CNN
F 1 "CONN_01X07" H 2919 2716 50  0000 C CNN
F 2 "w_pin_strip:pin_strip_7" H 3000 3150 50  0001 C CNN
F 3 "" H 3000 3150 50  0001 C CNN
	1    3000 3150
	-1   0    0    1   
$EndComp
Text Label 3400 3450 0    60   ~ 0
GND
Text Label 2700 1850 0    60   ~ 0
GND
Text Label 3650 1850 2    60   ~ 0
GND
Wire Wire Line
	2550 2350 2650 2350
Wire Wire Line
	3800 2350 3900 2350
Text Label 2650 2350 0    60   ~ 0
GND
Text Label 3900 2350 0    60   ~ 0
GND
$EndSCHEMATC

EESchema Schematic File Version 4
LIBS:balancebot-cache
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "BalanceBot"
Date "2020-05-05"
Rev "0.4.5"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L balancebot-components:DRV8825 A1
U 1 1 5DADF5FC
P 8600 1750
F 0 "A1" H 8600 2300 50  0000 C CNN
F 1 "DRV8825" V 8600 1750 50  0000 C CNB
F 2 "balancebot-footprints:bb_Pololu_Breakout-16_15.2x20.3mm" H 8600 1250 50  0001 C CNN
F 3 "https://www.pololu.com/product/2133" H 8700 1400 50  0001 C CNN
	1    8600 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C1
U 1 1 5DAE052E
P 9400 2100
F 0 "C1" H 9491 2146 50  0000 L CNN
F 1 "50V 100uF" H 9491 2055 50  0000 L CNN
F 2 "balancebot-footprints:bb_C_Rect_L7.0mm_W2.5mm_P5.00mm" H 9400 2100 50  0001 C CNN
F 3 "~" H 9400 2100 50  0001 C CNN
	1    9400 2100
	1    0    0    -1  
$EndComp
$Comp
L balancebot-components:GY-521(MPU6050) U3
U 1 1 5DAEE34C
P 5800 6400
F 0 "U3" H 5750 6950 60  0000 L CNN
F 1 "MPU6050" V 5800 6400 60  0000 C CNB
F 2 "balancebot-footprints:bb_GY-521" H 6128 6294 60  0001 L CNN
F 3 "" H 5750 6550 60  0000 C CNN
	1    5800 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1600 9000 1600
Wire Wire Line
	9500 1700 9000 1700
Wire Wire Line
	9500 1800 9000 1800
Wire Wire Line
	9500 1900 9000 1900
Wire Wire Line
	8200 1800 8200 1900
Wire Wire Line
	8200 1700 7900 1700
$Comp
L power:+3V3 #PWR022
U 1 1 5DB0B375
P 7900 1200
F 0 "#PWR022" H 7900 1050 50  0001 C CNN
F 1 "+3V3" H 7915 1373 50  0000 C CNN
F 2 "" H 7900 1200 50  0001 C CNN
F 3 "" H 7900 1200 50  0001 C CNN
	1    7900 1200
	1    0    0    -1  
$EndComp
Text GLabel 8050 800  0    50   Input ~ 0
MOTOR_ENABLE
Wire Wire Line
	9000 1500 9150 1500
Wire Wire Line
	9150 1500 9150 2100
Wire Wire Line
	9150 2100 9000 2100
Wire Wire Line
	9150 2100 9150 2300
Connection ~ 9150 2100
$Comp
L power:GND #PWR028
U 1 1 5DB0FC39
P 9150 2300
F 0 "#PWR028" H 9150 2050 50  0001 C CNN
F 1 "GND" H 9155 2127 50  0000 C CNN
F 2 "" H 9150 2300 50  0001 C CNN
F 3 "" H 9150 2300 50  0001 C CNN
	1    9150 2300
	1    0    0    -1  
$EndComp
$Comp
L balancebot-components:+VMOT #PWR030
U 1 1 5DB11DAE
P 9400 1200
F 0 "#PWR030" H 9400 1050 50  0001 C CNN
F 1 "+VMOT" H 9400 1373 50  0000 C CNN
F 2 "" H 9400 1200 50  0001 C CNN
F 3 "" H 9400 1200 50  0001 C CNN
	1    9400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 1400 9400 1400
Wire Wire Line
	9400 1400 9400 1200
Wire Wire Line
	9400 1400 9400 2000
Connection ~ 9400 1400
$Comp
L power:GND #PWR031
U 1 1 5DB13EAC
P 9400 2300
F 0 "#PWR031" H 9400 2050 50  0001 C CNN
F 1 "GND" H 9405 2127 50  0000 C CNN
F 2 "" H 9400 2300 50  0001 C CNN
F 3 "" H 9400 2300 50  0001 C CNN
	1    9400 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 2200 9400 2300
Wire Wire Line
	8200 1400 8150 1400
Wire Wire Line
	8150 1400 8150 800 
Wire Wire Line
	8150 800  8050 800 
Text GLabel 8050 2000 0    50   Input ~ 0
LEFT_MOTOR_STEP
Text GLabel 8050 2100 0    50   Input ~ 0
LEFT_MOTOR_DIR
Wire Wire Line
	8200 2100 8050 2100
Wire Wire Line
	8200 2000 8050 2000
Text GLabel 8050 2800 0    50   Input ~ 0
MOTOR_ENABLE
$Comp
L power:+3V3 #PWR023
U 1 1 5DB4BCD8
P 7900 3200
F 0 "#PWR023" H 7900 3050 50  0001 C CNN
F 1 "+3V3" H 7915 3373 50  0000 C CNN
F 2 "" H 7900 3200 50  0001 C CNN
F 3 "" H 7900 3200 50  0001 C CNN
	1    7900 3200
	1    0    0    -1  
$EndComp
$Comp
L balancebot-components:DRV8825 A2
U 1 1 5DB4C0D2
P 8600 3750
F 0 "A2" H 8600 4300 50  0000 C CNN
F 1 "DRV8825" V 8600 3750 50  0000 C CNB
F 2 "balancebot-footprints:bb_Pololu_Breakout-16_15.2x20.3mm" H 8600 3250 50  0001 C CNN
F 3 "https://www.pololu.com/product/2133" H 8700 3400 50  0001 C CNN
	1    8600 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3400 8150 3400
Wire Wire Line
	8150 3400 8150 2800
Wire Wire Line
	8150 2800 8050 2800
Wire Wire Line
	8200 3800 8200 3900
Wire Wire Line
	8200 4000 8050 4000
Wire Wire Line
	8200 4100 8050 4100
$Comp
L balancebot-components:+VMOT #PWR032
U 1 1 5DB53BE0
P 9400 3200
F 0 "#PWR032" H 9400 3050 50  0001 C CNN
F 1 "+VMOT" H 9400 3373 50  0000 C CNN
F 2 "" H 9400 3200 50  0001 C CNN
F 3 "" H 9400 3200 50  0001 C CNN
	1    9400 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C2
U 1 1 5DB53F97
P 9400 4100
F 0 "C2" H 9491 4146 50  0000 L CNN
F 1 "50V 100uF" H 9491 4055 50  0000 L CNN
F 2 "balancebot-footprints:bb_C_Rect_L7.0mm_W2.5mm_P5.00mm" H 9400 4100 50  0001 C CNN
F 3 "~" H 9400 4100 50  0001 C CNN
	1    9400 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 5DB5471A
P 9150 4350
F 0 "#PWR029" H 9150 4100 50  0001 C CNN
F 1 "GND" H 9155 4177 50  0000 C CNN
F 2 "" H 9150 4350 50  0001 C CNN
F 3 "" H 9150 4350 50  0001 C CNN
	1    9150 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 5DB54D4D
P 9400 4350
F 0 "#PWR033" H 9400 4100 50  0001 C CNN
F 1 "GND" H 9405 4177 50  0000 C CNN
F 2 "" H 9400 4350 50  0001 C CNN
F 3 "" H 9400 4350 50  0001 C CNN
	1    9400 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 3600 9500 3600
Wire Wire Line
	9000 3700 9500 3700
Wire Wire Line
	9000 3800 9500 3800
Wire Wire Line
	9000 3900 9500 3900
Wire Wire Line
	9000 3400 9400 3400
Wire Wire Line
	9400 3400 9400 3200
Wire Wire Line
	9400 3400 9400 4000
Connection ~ 9400 3400
Wire Wire Line
	9400 4200 9400 4350
Wire Wire Line
	9000 4100 9150 4100
Wire Wire Line
	9150 4100 9150 4350
Wire Wire Line
	9000 3500 9150 3500
Wire Wire Line
	9150 3500 9150 4100
Connection ~ 9150 4100
Wire Wire Line
	7900 3700 8200 3700
Text GLabel 8050 4000 0    50   Input ~ 0
RIGHT_MOTOR_STEP
Text GLabel 8050 4100 0    50   Input ~ 0
RIGHT_MOTOR_DIR
Text Notes 6950 5950 0    79   ~ 16
Stepper and Servo Control
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	10400 600  6900 600 
$Comp
L power:+3V3 #PWR018
U 1 1 5DBA8C50
P 5250 5850
F 0 "#PWR018" H 5250 5700 50  0001 C CNN
F 1 "+3V3" H 5265 6023 50  0000 C CNN
F 2 "" H 5250 5850 50  0001 C CNN
F 3 "" H 5250 5850 50  0001 C CNN
	1    5250 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6050 5250 6050
Wire Wire Line
	5250 6050 5250 5850
$Comp
L power:GND #PWR019
U 1 1 5DBAD235
P 5250 6850
F 0 "#PWR019" H 5255 6677 50  0001 C CNN
F 1 "GND" H 5255 6677 50  0000 C CNN
F 2 "" H 5250 6850 50  0001 C CNN
F 3 "" H 5250 6850 50  0001 C CNN
	1    5250 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 6150 5250 6150
Text GLabel 5100 6250 0    50   Input ~ 0
MPU_SCL
Text GLabel 5100 6350 0    50   Input ~ 0
MPU_SDA
Wire Wire Line
	5400 6250 5100 6250
Wire Wire Line
	5400 6350 5100 6350
NoConn ~ 5400 6450
NoConn ~ 5400 6550
NoConn ~ 5400 6650
$Comp
L power:GND #PWR03
U 1 1 5DBD6036
P 1050 6800
F 0 "#PWR03" H 1055 6627 50  0001 C CNN
F 1 "GND" H 1055 6627 50  0000 C CNN
F 2 "" H 1050 6800 50  0001 C CNN
F 3 "" H 1050 6800 50  0001 C CNN
	1    1050 6800
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR02
U 1 1 5DBDE325
P 1050 6550
F 0 "#PWR02" H 1050 6400 50  0001 C CNN
F 1 "+BATT" H 1065 6723 50  0000 C CNN
F 2 "" H 1050 6550 50  0001 C CNN
F 3 "" H 1050 6550 50  0001 C CNN
	1    1050 6550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 TB2
U 1 1 5DBEE897
P 1800 6650
F 0 "TB2" H 1880 6642 50  0000 L CNN
F 1 "7V-36V" H 1880 6551 50  0000 L CNN
F 2 "balancebot-footprints:bb_TE_282837-2" H 1800 6650 50  0001 C CNN
F 3 "https://docs.google.com/viewerng/viewer?url=https://s3.amazonaws.com/snapeda/datasheet/282837-2_TE_Connectivity.pdf" H 1800 6650 50  0001 C CNN
	1    1800 6650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 TB1
U 1 1 5DBF2801
P 1800 6050
F 0 "TB1" H 1880 6042 50  0000 L CNN
F 1 "Switch" H 1880 5951 50  0000 L CNN
F 2 "balancebot-footprints:bb_TE_282837-2" H 1800 6050 50  0001 C CNN
F 3 "~" H 1800 6050 50  0001 C CNN
	1    1800 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6650 1050 6650
Wire Wire Line
	1050 6650 1050 6550
Wire Wire Line
	1600 6750 1050 6750
Wire Wire Line
	1050 6750 1050 6800
$Comp
L power:+BATT #PWR04
U 1 1 5DBFE426
P 1400 5900
F 0 "#PWR04" H 1400 5750 50  0001 C CNN
F 1 "+BATT" H 1415 6073 50  0000 C CNN
F 2 "" H 1400 5900 50  0001 C CNN
F 3 "" H 1400 5900 50  0001 C CNN
	1    1400 5900
	1    0    0    -1  
$EndComp
$Comp
L balancebot-components:+VMOT #PWR01
U 1 1 5DC19BB8
P 1050 5900
F 0 "#PWR01" H 1050 5750 50  0001 C CNN
F 1 "+VMOT" H 1050 6073 50  0000 C CNN
F 2 "" H 1050 5900 50  0001 C CNN
F 3 "" H 1050 5900 50  0001 C CNN
	1    1050 5900
	1    0    0    -1  
$EndComp
Text Notes 650  7850 0    79   ~ 16
Power
Text Notes 4800 7300 0    79   ~ 16
Inputs
$Comp
L balancebot-components:ESP-32-NodeMCU-32S U1
U 1 1 5DCC291E
P 2750 1950
F 0 "U1" H 2750 3150 50  0000 C CNN
F 1 "ESP32" V 2750 1950 50  0000 C CNB
F 2 "balancebot-footprints:bb_ESP32-DEVKITC" H 2750 850 50  0001 C CNN
F 3 "https://einstronic.com/wp-content/uploads/2017/06/NodeMCU-32S-Catalogue.pdf" H 2750 650 50  0001 C CNN
	1    2750 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR07
U 1 1 5DCC3DD0
P 2000 950
F 0 "#PWR07" H 2000 800 50  0001 C CNN
F 1 "+3V3" H 2015 1123 50  0000 C CNN
F 2 "" H 2000 950 50  0001 C CNN
F 3 "" H 2000 950 50  0001 C CNN
	1    2000 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1000 2000 1000
Wire Wire Line
	2000 1000 2000 950 
$Comp
L power:+5V #PWR06
U 1 1 5DCD19E1
P 1750 950
F 0 "#PWR06" H 1750 800 50  0001 C CNN
F 1 "+5V" H 1765 1123 50  0000 C CNN
F 2 "" H 1750 950 50  0001 C CNN
F 3 "" H 1750 950 50  0001 C CNN
	1    1750 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 950  1750 2800
Wire Wire Line
	1750 2800 2200 2800
Wire Wire Line
	2200 2300 2000 2300
Wire Wire Line
	2000 2300 2000 2900
$Comp
L power:GND #PWR014
U 1 1 5DCE5161
P 3500 2900
F 0 "#PWR014" H 3500 2650 50  0001 C CNN
F 1 "GND" H 3505 2727 50  0000 C CNN
F 2 "" H 3500 2900 50  0001 C CNN
F 3 "" H 3500 2900 50  0001 C CNN
	1    3500 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1000 3500 1000
Wire Wire Line
	3500 1000 3500 1600
Wire Wire Line
	3300 1600 3500 1600
Text GLabel 1450 1800 0    50   Output ~ 0
MOTOR_ENABLE
Wire Wire Line
	2200 2400 1450 2400
Text GLabel 1450 1600 0    50   Output ~ 0
LEFT_MOTOR_STEP
Text GLabel 1450 1900 0    50   Output ~ 0
RIGHT_MOTOR_STEP
Text GLabel 1450 2000 0    50   Output ~ 0
RIGHT_MOTOR_DIR
Wire Wire Line
	2200 1800 1450 1800
Wire Wire Line
	2200 1900 1450 1900
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 5DD4AE70
P 9700 1700
F 0 "J5" H 9780 1646 50  0000 L CNN
F 1 "Left Motor Interface" H 9780 1601 50  0001 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 9700 1700 50  0001 C CNN
F 3 "~" H 9700 1700 50  0001 C CNN
	1    9700 1700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 5DD4C86B
P 9700 3700
F 0 "J6" H 9780 3646 50  0000 L CNN
F 1 "Right Motor Interface" H 9780 3601 50  0001 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 9700 3700 50  0001 C CNN
F 3 "~" H 9700 3700 50  0001 C CNN
	1    9700 3700
	1    0    0    -1  
$EndComp
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	6900 6000 10400 6000
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	10400 600  10400 6000
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	6900 600  6900 6000
NoConn ~ 3300 2800
NoConn ~ 3300 2700
NoConn ~ 3300 2600
NoConn ~ 3300 2500
NoConn ~ 3300 2400
NoConn ~ 3300 2300
NoConn ~ 3300 2200
NoConn ~ 3300 1900
NoConn ~ 3300 1400
NoConn ~ 3300 1300
NoConn ~ 3300 1100
Text Notes 650  5350 0    79   ~ 16
Modules
$Comp
L power:GND #PWR021
U 1 1 5DE724E0
P 5300 4350
F 0 "#PWR021" H 5300 4100 50  0001 C CNN
F 1 "GND" H 5305 4177 50  0000 C CNN
F 2 "" H 5300 4350 50  0001 C CNN
F 3 "" H 5300 4350 50  0001 C CNN
	1    5300 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4350 5300 3750
$Comp
L power:+5V #PWR017
U 1 1 5DED262C
P 5050 2400
F 0 "#PWR017" H 5050 2250 50  0001 C CNN
F 1 "+5V" H 5065 2573 50  0000 C CNN
F 2 "" H 5050 2400 50  0001 C CNN
F 3 "" H 5050 2400 50  0001 C CNN
	1    5050 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR020
U 1 1 5DED76D1
P 5300 2400
F 0 "#PWR020" H 5300 2250 50  0001 C CNN
F 1 "+3V3" H 5315 2573 50  0000 C CNN
F 2 "" H 5300 2400 50  0001 C CNN
F 3 "" H 5300 2400 50  0001 C CNN
	1    5300 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2400 5300 2450
Text GLabel 1650 1100 0    50   Output ~ 0
EN
Text GLabel 1450 1700 0    50   Output ~ 0
LEFT_MOTOR_DIR
Text GLabel 1650 1400 0    50   Output ~ 0
IO34
Text GLabel 1650 1500 0    50   Output ~ 0
IO35
Text GLabel 1650 2500 0    50   Output ~ 0
IO9
Text GLabel 1650 2600 0    50   Output ~ 0
IO10
Text GLabel 1650 2700 0    50   Output ~ 0
IO11
Wire Wire Line
	1650 1100 2200 1100
Wire Wire Line
	1650 1200 2200 1200
Wire Wire Line
	1650 1300 2200 1300
Wire Wire Line
	1650 1400 2200 1400
Wire Wire Line
	1650 1500 2200 1500
Wire Wire Line
	5050 4250 5050 2400
Text GLabel 4750 3250 0    50   Output ~ 0
MOTOR_ENABLE
Text GLabel 4750 3050 0    50   Output ~ 0
LEFT_MOTOR_STEP
Text GLabel 4750 3350 0    50   Output ~ 0
RIGHT_MOTOR_STEP
Text GLabel 4750 3450 0    50   Output ~ 0
RIGHT_MOTOR_DIR
Text GLabel 1450 2400 0    50   Output ~ 0
ARM_SERVO_PWM
Text GLabel 4900 2550 0    50   Output ~ 0
EN
Text GLabel 4750 3150 0    50   Output ~ 0
LEFT_MOTOR_DIR
Text GLabel 4900 2850 0    50   Output ~ 0
IO34
Text GLabel 4900 2950 0    50   Output ~ 0
IO35
Text GLabel 4900 2750 0    50   Output ~ 0
IO39
Text GLabel 4900 3950 0    50   Output ~ 0
IO9
Text GLabel 4900 4050 0    50   Output ~ 0
IO10
Text GLabel 4900 4150 0    50   Output ~ 0
IO11
Wire Wire Line
	2200 1600 1450 1600
Wire Wire Line
	2200 1700 1450 1700
Wire Wire Line
	2200 2500 1650 2500
Wire Wire Line
	2200 2600 1650 2600
Wire Wire Line
	2200 2700 1650 2700
$Comp
L power:GND #PWR08
U 1 1 5E1EF008
P 2000 2900
F 0 "#PWR08" H 2000 2650 50  0001 C CNN
F 1 "GND" H 2005 2727 50  0000 C CNN
F 2 "" H 2000 2900 50  0001 C CNN
F 3 "" H 2000 2900 50  0001 C CNN
	1    2000 2900
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5E310449
P 9850 6400
F 0 "#FLG02" H 9850 6475 50  0001 C CNN
F 1 "PWR_FLAG" H 9850 6573 50  0000 C CNN
F 2 "" H 9850 6400 50  0001 C CNN
F 3 "~" H 9850 6400 50  0001 C CNN
	1    9850 6400
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5E310D86
P 10150 6400
F 0 "#FLG03" H 10150 6475 50  0001 C CNN
F 1 "PWR_FLAG" H 10150 6573 50  0000 C CNN
F 2 "" H 10150 6400 50  0001 C CNN
F 3 "~" H 10150 6400 50  0001 C CNN
	1    10150 6400
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5E311545
P 9550 6400
F 0 "#FLG01" H 9550 6475 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 6573 50  0000 C CNN
F 2 "" H 9550 6400 50  0001 C CNN
F 3 "~" H 9550 6400 50  0001 C CNN
	1    9550 6400
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR034
U 1 1 5E3136CE
P 9550 6400
F 0 "#PWR034" H 9550 6250 50  0001 C CNN
F 1 "+BATT" H 9565 6573 50  0000 C CNN
F 2 "" H 9550 6400 50  0001 C CNN
F 3 "" H 9550 6400 50  0001 C CNN
	1    9550 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5900 1400 6050
Wire Wire Line
	1400 6050 1600 6050
$Comp
L balancebot-components:+VMOT #PWR035
U 1 1 5E3294E2
P 9850 6400
F 0 "#PWR035" H 9850 6250 50  0001 C CNN
F 1 "+VMOT" H 9850 6573 50  0000 C CNN
F 2 "" H 9850 6400 50  0001 C CNN
F 3 "" H 9850 6400 50  0001 C CNN
	1    9850 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6150 1050 6150
Wire Wire Line
	1050 6150 1050 5900
$Comp
L power:GND #PWR036
U 1 1 5E33EFF9
P 10150 6400
F 0 "#PWR036" H 10150 6150 50  0001 C CNN
F 1 "GND" H 10155 6227 50  0000 C CNN
F 2 "" H 10150 6400 50  0001 C CNN
F 3 "" H 10150 6400 50  0001 C CNN
	1    10150 6400
	-1   0    0    1   
$EndComp
Wire Wire Line
	7900 1200 7900 1500
Wire Wire Line
	7900 3200 7900 3500
Wire Wire Line
	5450 3450 4750 3450
Wire Wire Line
	5300 3750 5450 3750
Wire Wire Line
	5450 3850 4750 3850
Wire Wire Line
	4900 3950 5450 3950
Wire Wire Line
	5450 4050 4900 4050
Wire Wire Line
	5450 4150 4900 4150
Wire Wire Line
	5450 2450 5300 2450
Wire Wire Line
	4900 2550 5450 2550
Wire Wire Line
	4900 2650 5450 2650
Wire Wire Line
	4900 2750 5450 2750
Wire Wire Line
	4900 2850 5450 2850
Wire Wire Line
	4900 2950 5450 2950
Wire Wire Line
	5450 3050 4750 3050
Wire Wire Line
	4750 3150 5450 3150
Wire Wire Line
	5450 3250 4750 3250
Wire Wire Line
	5450 3350 4750 3350
Wire Wire Line
	5450 4250 5050 4250
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	600  5500 4300 5500
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	4300 7900 600  7900
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	4400 5550 4400 7350
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	6200 5550 6200 7350
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	600  5400 6700 5400
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	600  600  6700 600 
Wire Wire Line
	8200 3500 7900 3500
Connection ~ 7900 3500
Wire Wire Line
	7900 3500 7900 3600
Wire Wire Line
	8200 3600 7900 3600
Connection ~ 7900 3600
Wire Wire Line
	7900 3600 7900 3700
Wire Wire Line
	8200 1500 7900 1500
Connection ~ 7900 1500
Wire Wire Line
	7900 1500 7900 1600
Wire Wire Line
	8200 1600 7900 1600
Connection ~ 7900 1600
Wire Wire Line
	7900 1600 7900 1700
Wire Wire Line
	8550 5100 8550 5250
Wire Wire Line
	8650 5100 8550 5100
Wire Wire Line
	8650 4900 8400 4900
Wire Wire Line
	8550 5000 8550 4800
$Comp
L power:GND #PWR025
U 1 1 5DD210B7
P 8550 5250
F 0 "#PWR025" H 8550 5000 50  0001 C CNN
F 1 "GND" H 8555 5077 50  0000 C CNN
F 2 "" H 8550 5250 50  0001 C CNN
F 3 "" H 8550 5250 50  0001 C CNN
	1    8550 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR024
U 1 1 5DD20B47
P 8550 4800
F 0 "#PWR024" H 8550 4650 50  0001 C CNN
F 1 "+5V" H 8565 4973 50  0000 C CNN
F 2 "" H 8550 4800 50  0001 C CNN
F 3 "" H 8550 4800 50  0001 C CNN
	1    8550 4800
	1    0    0    -1  
$EndComp
Text GLabel 8400 4900 0    50   Input ~ 0
ARM_SERVO_PWM
Wire Wire Line
	8650 5000 8550 5000
$Comp
L power:GND #PWR05
U 1 1 5DD39FD1
P 2500 7500
F 0 "#PWR05" H 2505 7327 50  0001 C CNN
F 1 "GND" H 2505 7327 50  0000 C CNN
F 2 "" H 2500 7500 50  0001 C CNN
F 3 "" H 2500 7500 50  0001 C CNN
	1    2500 7500
	1    0    0    -1  
$EndComp
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	600  5500 600  7900
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	4300 5500 4300 7900
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5DD39FCB
P 2500 7300
F 0 "H1" H 2450 7500 50  0000 L CNN
F 1 "MountingHole_Pad" H 2600 7258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2500 7300 50  0001 C CNN
F 3 "~" H 2500 7300 50  0001 C CNN
	1    2500 7300
	1    0    0    -1  
$EndComp
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	600  600  600  5400
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	6700 600  6700 5400
Text GLabel 4900 2650 0    50   Output ~ 0
IO36
Text Notes 5800 3300 0    50   ~ 0
1.0” ESP32 Support\nInterface
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 5DD2056F
P 8850 5000
F 0 "J4" H 9000 5050 50  0000 C CNN
F 1 "Servo Interface" H 9250 4950 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8850 5000 50  0001 C CNN
F 3 "~" H 8850 5000 50  0001 C CNN
	1    8850 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3550 4900 3550
Wire Wire Line
	5450 3650 4900 3650
Text GLabel 4900 3650 0    50   Output ~ 0
IO12
Text GLabel 4900 3550 0    50   Output ~ 0
IO14
Wire Wire Line
	2200 2200 1650 2200
Wire Wire Line
	2200 2100 1650 2100
Text GLabel 1650 2200 0    50   Output ~ 0
IO12
Text GLabel 1650 2100 0    50   Output ~ 0
IO14
Wire Wire Line
	3650 1500 3300 1500
Wire Wire Line
	3650 1200 3300 1200
Text GLabel 3650 1500 2    50   Output ~ 0
MPU_SDA
Text GLabel 3650 1200 2    50   Output ~ 0
MPU_SCL
Text GLabel 1650 1200 0    50   Output ~ 0
IO36
Text GLabel 1650 1300 0    50   Output ~ 0
IO39
Wire Wire Line
	2200 2000 1450 2000
$Comp
L power:+3V3 #PWR026
U 1 1 5E05816F
P 9050 1200
F 0 "#PWR026" H 9050 1050 50  0001 C CNN
F 1 "+3V3" H 9065 1373 50  0000 C CNN
F 2 "" H 9050 1200 50  0001 C CNN
F 3 "" H 9050 1200 50  0001 C CNN
	1    9050 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2000 9050 2000
Wire Wire Line
	9050 2000 9050 1200
$Comp
L power:+3V3 #PWR027
U 1 1 5E05E3AC
P 9050 3200
F 0 "#PWR027" H 9050 3050 50  0001 C CNN
F 1 "+3V3" H 9065 3373 50  0000 C CNN
F 2 "" H 9050 3200 50  0001 C CNN
F 3 "" H 9050 3200 50  0001 C CNN
	1    9050 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4000 9050 4000
Wire Wire Line
	9050 4000 9050 3200
Text GLabel 3650 2100 2    50   Output ~ 0
MPU_INTERRUPT
Text GLabel 5100 6750 0    50   Input ~ 0
MPU_INTERRUPT
Wire Wire Line
	5250 6150 5250 6750
Wire Wire Line
	5250 6750 5250 6850
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	4400 7350 6200 7350
Wire Notes Line width 8 style solid rgb(0, 0, 0)
	4400 5550 6200 5550
Text GLabel 4750 3850 0    50   Output ~ 0
ARM_SERVO_PWM
Wire Wire Line
	3300 2100 3650 2100
Wire Wire Line
	3500 1600 3500 2900
Connection ~ 3500 1600
Wire Wire Line
	5100 6750 5400 6750
$Comp
L Connector_Generic:Conn_01x19 J3
U 1 1 5DE328D8
P 5650 3350
F 0 "J3" H 5650 2300 50  0000 C CNN
F 1 "Passthrough" H 5800 3350 50  0001 L CNN
F 2 "balancebot-footprints:bb_PinSocket_1x19_P2.54mm_Vertical" H 5650 3350 50  0001 C CNN
F 3 "~" H 5650 3350 50  0001 C CNN
	1    5650 3350
	1    0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5DD34684
P 2700 7300
F 0 "H2" H 2650 7500 50  0000 L CNN
F 1 "MountingHole_Pad" H 2800 7258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2700 7300 50  0001 C CNN
F 3 "~" H 2700 7300 50  0001 C CNN
	1    2700 7300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5DD2EFA1
P 2900 7300
F 0 "H3" H 2850 7500 50  0000 L CNN
F 1 "MountingHole_Pad" H 3000 7258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 2900 7300 50  0001 C CNN
F 3 "~" H 2900 7300 50  0001 C CNN
	1    2900 7300
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5DD05082
P 3100 7300
F 0 "H4" H 3050 7500 50  0000 L CNN
F 1 "MountingHole_Pad" H 3200 7258 50  0001 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 3100 7300 50  0001 C CNN
F 3 "~" H 3100 7300 50  0001 C CNN
	1    3100 7300
	1    0    0    -1  
$EndComp
$Comp
L balancebot-components:+VMOT #PWR013
U 1 1 5DD3834E
P 2750 5900
F 0 "#PWR013" H 2750 5750 50  0001 C CNN
F 1 "+VMOT" H 2750 6073 50  0000 C CNN
F 2 "" H 2750 5900 50  0001 C CNN
F 3 "" H 2750 5900 50  0001 C CNN
	1    2750 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5DD39182
P 3300 6350
F 0 "#PWR015" H 3305 6177 50  0001 C CNN
F 1 "GND" H 3305 6177 50  0000 C CNN
F 2 "" H 3300 6350 50  0001 C CNN
F 3 "" H 3300 6350 50  0001 C CNN
	1    3300 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6050 2750 6050
Wire Wire Line
	2750 6050 2750 5900
Wire Wire Line
	3100 7400 2900 7400
Wire Wire Line
	2900 7400 2700 7400
Connection ~ 2900 7400
Wire Wire Line
	2700 7400 2500 7400
Connection ~ 2700 7400
Connection ~ 2500 7400
Wire Wire Line
	2500 7400 2500 7500
NoConn ~ 3300 2000
NoConn ~ 3300 1800
NoConn ~ 3300 1700
$Comp
L Converter_DCDC:OKI-78SR-5_1.5-W36-C U2
U 1 1 5EBE136C
P 3300 6050
F 0 "U2" H 3300 6292 50  0000 C CNN
F 1 "OKI-78SR-5/1.5-W36-C" H 3300 6201 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_muRata_OKI-78SR_Vertical" H 3350 5800 50  0001 L CIN
F 3 "https://power.murata.com/data/power/oki-78sr.pdf" H 3300 6050 50  0001 C CNN
	1    3300 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6050 3900 5900
Wire Wire Line
	3600 6050 3900 6050
$Comp
L power:+5V #PWR016
U 1 1 5DD38B2B
P 3900 5900
F 0 "#PWR016" H 3900 5750 50  0001 C CNN
F 1 "+5V" H 3915 6073 50  0000 C CNN
F 2 "" H 3900 5900 50  0001 C CNN
F 3 "" H 3900 5900 50  0001 C CNN
	1    3900 5900
	1    0    0    -1  
$EndComp
$EndSCHEMATC

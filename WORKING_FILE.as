opt subtitle "HI-TECH Software Omniscient Code Generator (Lite mode) build 9453"

opt pagewidth 120

	opt lm

	processor	16F877A
clrc	macro
	bcf	3,0
	endm
clrz	macro
	bcf	3,2
	endm
setc	macro
	bsf	3,0
	endm
setz	macro
	bsf	3,2
	endm
skipc	macro
	btfss	3,0
	endm
skipz	macro
	btfss	3,2
	endm
skipnc	macro
	btfsc	3,0
	endm
skipnz	macro
	btfsc	3,2
	endm
indf	equ	0
indf0	equ	0
pc	equ	2
pcl	equ	2
status	equ	3
fsr	equ	4
fsr0	equ	4
c	equ	1
z	equ	0
pclath	equ	10
# 21 "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_findWalls
	FNCALL	_main,_turnAround
	FNCALL	_main,_turnLeft90
	FNCALL	_main,_turnRight90
	FNCALL	_main,_lcd_write_data
	FNCALL	_main,_play_iCreate_song
	FNCALL	_main,_writeEEPROMTestData
	FNCALL	_main,_EEPROMToSerial
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_rotateIR
	FNCALL	_main,_wallFollow
	FNCALL	_main,_frontWallCorrect
	FNCALL	_main,_goToNextCell
	FNCALL	_main,_goRight
	FNCALL	_main,_getOrientation
	FNCALL	_main,_goForward
	FNCALL	_main,_goLeft
	FNCALL	_main,_getSuccessfulDrive
	FNCALL	_main,_updateMapData
	FNCALL	_main,_updateLocation
	FNCALL	_main,_updateNode
	FNCALL	_main,_checkIfHome
	FNCALL	_goToNextCell,_getSomethingInTheWay
	FNCALL	_goToNextCell,_goLeft
	FNCALL	_goToNextCell,_goForward
	FNCALL	_goToNextCell,_goRight
	FNCALL	_goToNextCell,_goBackward
	FNCALL	_findWalls,_rotateIR
	FNCALL	_findWalls,_lcd_set_cursor
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_findWalls,_play_iCreate_song
	FNCALL	_goRight,_lcd_set_cursor
	FNCALL	_goRight,_lcd_write_data
	FNCALL	_goRight,_turnRight90
	FNCALL	_goRight,_updateOrientation
	FNCALL	_goRight,_driveForDistance
	FNCALL	_goLeft,_lcd_set_cursor
	FNCALL	_goLeft,_lcd_write_data
	FNCALL	_goLeft,_turnLeft90
	FNCALL	_goLeft,_updateOrientation
	FNCALL	_goLeft,_driveForDistance
	FNCALL	_goForward,_lcd_set_cursor
	FNCALL	_goForward,_lcd_write_data
	FNCALL	_goForward,_getCurrentX
	FNCALL	_goForward,_getCurrentY
	FNCALL	_goForward,_driveForDistance
	FNCALL	_goBackward,_lcd_set_cursor
	FNCALL	_goBackward,_lcd_write_data
	FNCALL	_goBackward,_turnAround
	FNCALL	_goBackward,_updateOrientation
	FNCALL	_goBackward,_driveForDistance
	FNCALL	_wallFollow,_readIR
	FNCALL	_wallFollow,_drive
	FNCALL	_wallFollow,_waitFor
	FNCALL	_findWall,_readIR
	FNCALL	_frontWallCorrect,_readIR
	FNCALL	_frontWallCorrect,_drive
	FNCALL	_frontWallCorrect,_clearSuccessfulDrive
	FNCALL	_driveForDistance,_drive
	FNCALL	_driveForDistance,_ser_putch
	FNCALL	_driveForDistance,_ser_getch
	FNCALL	_driveForDistance,_goReverse
	FNCALL	_driveForDistance,_clearSuccessfulDrive
	FNCALL	_driveForDistance,_turnRight90
	FNCALL	_driveForDistance,_updateOrientation
	FNCALL	_driveForDistance,_turnLeft90
	FNCALL	_driveForDistance,_getCurrentY
	FNCALL	_driveForDistance,_getCurrentX
	FNCALL	_driveForDistance,_findFinalDestination
	FNCALL	_driveForDistance,_setVirtualLocation
	FNCALL	_updateLocation,_lcd_set_cursor
	FNCALL	_updateLocation,_lcd_write_data
	FNCALL	_updateLocation,_getOrientation
	FNCALL	_updateLocation,_lcd_write_1_digit_bcd
	FNCALL	_lookForVictim,_ser_putch
	FNCALL	_lookForVictim,_ser_getch
	FNCALL	_lookForVictim,_play_iCreate_song
	FNCALL	_lookForVictim,_lcd_set_cursor
	FNCALL	_lookForVictim,_lcd_write_data
	FNCALL	_lookForVictim,_getVictimZone
	FNCALL	_lookForVictim,_lcd_write_1_digit_bcd
	FNCALL	_checkForFinalDestination,_getFinalX
	FNCALL	_checkForFinalDestination,_getFinalY
	FNCALL	_checkForFinalDestination,_play_iCreate_song
	FNCALL	_checkForFinalDestination,_lcd_set_cursor
	FNCALL	_checkForFinalDestination,_lcd_write_data
	FNCALL	_init,_init_adc
	FNCALL	_init,_lcd_init
	FNCALL	_init,_ser_init
	FNCALL	_init,_initIRobot
	FNCALL	_init,_initSongs
	FNCALL	_goReverse,_lcd_set_cursor
	FNCALL	_goReverse,_lcd_write_data
	FNCALL	_goReverse,_drive
	FNCALL	_goReverse,_waitFor
	FNCALL	_readIR,_adc_read_channel
	FNCALL	_readIR,_convert
	FNCALL	_findFinalDestination,_lcd_set_cursor
	FNCALL	_findFinalDestination,_lcd_write_1_digit_bcd
	FNCALL	_updateMapData,_addNewData
	FNCALL	_writeEEPROMTestData,_addNewData
	FNCALL	_checkIfHome,_drive
	FNCALL	_checkIfHome,_play_iCreate_song
	FNCALL	_turnAround,_drive
	FNCALL	_turnAround,_waitFor
	FNCALL	_turnLeft90,_drive
	FNCALL	_turnLeft90,_getCurrentX
	FNCALL	_turnLeft90,_getCurrentY
	FNCALL	_turnLeft90,_waitFor
	FNCALL	_turnRight90,_drive
	FNCALL	_turnRight90,_waitFor
	FNCALL	_initSongs,_ser_putArr
	FNCALL	_lcd_init,_lcd_write_control
	FNCALL	_lcd_write_1_digit_bcd,_lcd_write_data
	FNCALL	_lcd_set_cursor,_lcd_write_control
	FNCALL	_EEPROMToSerial,_readEEPROM
	FNCALL	_EEPROMToSerial,_ser_putch
	FNCALL	_addNewData,_writeEEPROM
	FNCALL	_lcd_write_string,_lcd_write_data
	FNCALL	_adc_read_channel,_adc_read
	FNCALL	_initIRobot,_ser_putch
	FNCALL	_waitFor,_ser_putch
	FNCALL	_drive,_ser_putch
	FNCALL	_convert,___wmul
	FNCALL	_convert,___awdiv
	FNCALL	_play_iCreate_song,_ser_putch
	FNCALL	_ser_putArr,_ser_putch
	FNCALL	_ser_getch,_ser_isrx
	FNCALL	_readEEPROM,_initEEPROMMode
	FNCALL	_readEEPROM,_writeSPIByte
	FNCALL	_writeEEPROM,_initEEPROMMode
	FNCALL	_writeEEPROM,_writeSPIByte
	FNCALL	_adc_read,___awdiv
	FNROOT	_main
	FNCALL	intlevel1,_isr1
	global	intlevel1
	FNROOT	intlevel1
	global	_finalX
	global	_finalY
	global	_somethingInTheWay
	global	_xCoord
	global	_xVictim
	global	_xVirtual
	global	_yCoord
	global	_yVictim
	global	_yVirtual
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
	global	_beep
	global	_longbeep
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	7

;initializer for _finalX
	retlw	03h
	line	8

;initializer for _finalY
	retlw	01h
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	15

;initializer for _somethingInTheWay
	retlw	02h
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	51

;initializer for _xCoord
	retlw	01h
	line	53

;initializer for _xVictim
	retlw	-10
	line	55

;initializer for _xVirtual
	retlw	-10
	line	52

;initializer for _yCoord
	retlw	03h
	line	54

;initializer for _yVictim
	retlw	-10
	line	56

;initializer for _yVirtual
	retlw	-10
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	11

;initializer for _lookingForU2
	retlw	08Ch
	retlw	02h
	retlw	0Dh
	retlw	048h
	retlw	01Ch
	retlw	048h
	retlw	01Ch
	retlw	04Fh
	retlw	040h
	retlw	080h
	retlw	010h
	retlw	04Dh
	retlw	01Ch
	retlw	04Ch
	retlw	01Ch
	retlw	048h
	retlw	040h
	retlw	080h
	retlw	010h
	retlw	045h
	retlw	01Ch
	retlw	045h
	retlw	01Ch
	retlw	045h
	retlw	01Ch
	retlw	048h
	retlw	01Ch
	retlw	048h
	retlw	040h
	line	12

;initializer for _finalCountdown
	retlw	08Ch
	retlw	03h
	retlw	0Ch
	retlw	049h
	retlw	08h
	retlw	047h
	retlw	08h
	retlw	049h
	retlw	020h
	retlw	042h
	retlw	040h
	retlw	080h
	retlw	020h
	retlw	04Ah
	retlw	08h
	retlw	049h
	retlw	08h
	retlw	04Ah
	retlw	08h
	retlw	080h
	retlw	0Ch
	retlw	049h
	retlw	06h
	retlw	080h
	retlw	0Ch
	retlw	047h
	retlw	030h
	line	10

;initializer for _superMarioBros
	retlw	08Ch
	retlw	01h
	retlw	0Bh
	retlw	04Ch
	retlw	08h
	retlw	080h
	retlw	04h
	retlw	04Ch
	retlw	010h
	retlw	080h
	retlw	04h
	retlw	04Ch
	retlw	010h
	retlw	080h
	retlw	04h
	retlw	048h
	retlw	08h
	retlw	04Ch
	retlw	010h
	retlw	04Fh
	retlw	020h
	retlw	080h
	retlw	08h
	retlw	043h
	retlw	010h
psect	idataBANK1,class=CODE,space=0,delta=2
global __pidataBANK1
__pidataBANK1:
	line	13

;initializer for _champions
	retlw	08Ch
	retlw	04h
	retlw	09h
	retlw	04Ah
	retlw	040h
	retlw	049h
	retlw	010h
	retlw	04Ah
	retlw	010h
	retlw	049h
	retlw	030h
	retlw	045h
	retlw	030h
	retlw	080h
	retlw	020h
	retlw	042h
	retlw	01Ch
	retlw	047h
	retlw	020h
	retlw	042h
	retlw	030h
	line	14

;initializer for _beep
	retlw	08Ch
	retlw	05h
	retlw	01h
	retlw	048h
	retlw	04h
	line	15

;initializer for _longbeep
	retlw	08Ch
	retlw	06h
	retlw	01h
	retlw	048h
	retlw	010h
	global	_eepromSerial
	global	_start
	global	_RTC_Counter
	global	_closestObject
	global	_addressCount
	global	_addressCurrent
	global	_currentOrientation
	global	_dVirtual
	global	_lastMove
	global	_node
	global	_rxoptr
	global	_ser_tmp
	global	_stepPosition
	global	_stepsToPerpendicular
	global	_txoptr
	global	_vicZone
	global	_victimZone
	global	_wayWent
	global	_rxiptr
	global	_txiptr
	global	_RTC_FLAG_10MS
	global	_RTC_FLAG_1MS
	global	_RTC_FLAG_500MS
	global	_RTC_FLAG_50MS
	global	_frontWall
	global	_goingHome
	global	_home
	global	_leftWall
	global	_moving
	global	_ready
	global	_rightWall
	global	_successfulDrive
	global	_rxfifo
	global	_txfifo
	global	_ADCON0
_ADCON0	set	31
	global	_ADRESH
_ADRESH	set	30
	global	_PORTA
_PORTA	set	5
	global	_PORTC
_PORTC	set	7
	global	_PORTD
_PORTD	set	8
	global	_PORTE
_PORTE	set	9
	global	_RCREG
_RCREG	set	26
	global	_SSPBUF
_SSPBUF	set	19
	global	_SSPCON
_SSPCON	set	20
	global	_TMR0
_TMR0	set	1
	global	_TXREG
_TXREG	set	25
	global	_CARRY
_CARRY	set	24
	global	_CHS0
_CHS0	set	251
	global	_CHS1
_CHS1	set	252
	global	_CHS2
_CHS2	set	253
	global	_CREN
_CREN	set	196
	global	_GIE
_GIE	set	95
	global	_GO
_GO	set	250
	global	_OERR
_OERR	set	193
	global	_PEIE
_PEIE	set	94
	global	_RB0
_RB0	set	48
	global	_RB1
_RB1	set	49
	global	_RCIF
_RCIF	set	101
	global	_RE0
_RE0	set	72
	global	_RE1
_RE1	set	73
	global	_RE2
_RE2	set	74
	global	_RX9
_RX9	set	198
	global	_SPEN
_SPEN	set	199
	global	_SSPIF
_SSPIF	set	99
	global	_TMR0IE
_TMR0IE	set	93
	global	_TMR0IF
_TMR0IF	set	90
	global	_TXIF
_TXIF	set	100
	global	_ADCON1
_ADCON1	set	159
	global	_ADRESL
_ADRESL	set	158
	global	_OPTION_REG
_OPTION_REG	set	129
	global	_SPBRG
_SPBRG	set	153
	global	_SSPSTAT
_SSPSTAT	set	148
	global	_TRISA
_TRISA	set	133
	global	_TRISB
_TRISB	set	134
	global	_TRISC
_TRISC	set	135
	global	_TRISD
_TRISD	set	136
	global	_TRISE
_TRISE	set	137
	global	_BRGH
_BRGH	set	1218
	global	_RCIE
_RCIE	set	1125
	global	_SYNC
_SYNC	set	1220
	global	_TX9
_TX9	set	1222
	global	_TXEN
_TXEN	set	1221
	global	_TXIE
_TXIE	set	1124
	global	_EEADR
_EEADR	set	269
	global	_EEDATA
_EEDATA	set	268
	global	_EECON1
_EECON1	set	396
	global	_EECON2
_EECON2	set	397
	global	_RD
_RD	set	3168
	global	_WR
_WR	set	3169
	global	_WREN
_WREN	set	3170
psect	strings,class=STRING,delta=2
global __pstrings
__pstrings:
;	global	stringdir,stringtab,__stringbase
stringtab:
;	String table - string pointers are 1 byte each
stringcode:stringdir:
movlw high(stringdir)
movwf pclath
movf fsr,w
incf fsr
	addwf pc
__stringbase:
	retlw	0
psect	strings
	
STR_7:	
	retlw	67	;'C'
	retlw	111	;'o'
	retlw	109	;'m'
	retlw	112	;'p'
	retlw	108	;'l'
	retlw	101	;'e'
	retlw	116	;'t'
	retlw	101	;'e'
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	0
psect	strings
	
STR_5:	
	retlw	69	;'E'
	retlw	69	;'E'
	retlw	80	;'P'
	retlw	82	;'R'
	retlw	79	;'O'
	retlw	77	;'M'
	retlw	32	;' '
	retlw	83	;'S'
	retlw	101	;'e'
	retlw	114	;'r'
	retlw	105	;'i'
	retlw	97	;'a'
	retlw	108	;'l'
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	0
psect	strings
	
STR_6:	
	retlw	80	;'P'
	retlw	108	;'l'
	retlw	101	;'e'
	retlw	97	;'a'
	retlw	115	;'s'
	retlw	101	;'e'
	retlw	32	;' '
	retlw	87	;'W'
	retlw	97	;'a'
	retlw	105	;'i'
	retlw	116	;'t'
	retlw	46	;'.'
	retlw	46	;'.'
	retlw	46	;'.'
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	32	;' '
	retlw	0
psect	strings
	
STR_3:	
	retlw	40	;'('
	retlw	45	;'-'
	retlw	44	;','
	retlw	45	;'-'
	retlw	41	;')'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	45	;'-'
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	0
psect	strings
	
STR_4:	
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	32	;' '
	retlw	40	;'('
	retlw	51	;'3'
	retlw	44	;','
	retlw	49	;'1'
	retlw	41	;')'
	retlw	32	;' '
	retlw	71	;'G'
	retlw	82	;'R'
	retlw	69	;'E'
	retlw	71	;'G'
	retlw	0
psect	strings
	
STR_1:	
	retlw	69	;'E'
	retlw	69	;'E'
	retlw	80	;'P'
	retlw	82	;'R'
	retlw	79	;'O'
	retlw	77	;'M'
	retlw	32	;' '
	retlw	83	;'S'
	retlw	117	;'u'
	retlw	99	;'c'
	retlw	99	;'c'
	retlw	101	;'e'
	retlw	115	;'s'
	retlw	115	;'s'
	retlw	0
psect	strings
	
STR_2:	
	retlw	69	;'E'
	retlw	69	;'E'
	retlw	80	;'P'
	retlw	82	;'R'
	retlw	79	;'O'
	retlw	77	;'M'
	retlw	32	;' '
	retlw	70	;'F'
	retlw	97	;'a'
	retlw	105	;'i'
	retlw	108	;'l'
	retlw	101	;'e'
	retlw	100	;'d'
	retlw	0
psect	strings
	file	"WORKING_FILE.as"
	line	#
psect cinit,class=CODE,delta=2
global start_initialization
start_initialization:

psect	bitbssCOMMON,class=COMMON,bit,space=1
global __pbitbssCOMMON
__pbitbssCOMMON:
_RTC_FLAG_10MS:
       ds      1

_RTC_FLAG_1MS:
       ds      1

_RTC_FLAG_500MS:
       ds      1

_RTC_FLAG_50MS:
       ds      1

_frontWall:
       ds      1

_goingHome:
       ds      1

_home:
       ds      1

_leftWall:
       ds      1

_moving:
       ds      1

_ready:
       ds      1

_rightWall:
       ds      1

_successfulDrive:
       ds      1

psect	bssCOMMON,class=COMMON,space=1
global __pbssCOMMON
__pbssCOMMON:
_rxiptr:
       ds      1

_txiptr:
       ds      1

psect	bssBANK0,class=BANK0,space=1
global __pbssBANK0
__pbssBANK0:
_eepromSerial:
       ds      3

_start:
       ds      3

_RTC_Counter:
       ds      2

_closestObject:
       ds      2

_addressCount:
       ds      1

_addressCurrent:
       ds      1

_currentOrientation:
       ds      1

_dVirtual:
       ds      1

_lastMove:
       ds      1

_node:
       ds      1

_rxoptr:
       ds      1

_ser_tmp:
       ds      1

_stepPosition:
       ds      1

_stepsToPerpendicular:
       ds      1

_txoptr:
       ds      1

_vicZone:
       ds      1

_victimZone:
       ds      1

_wayWent:
       ds      1

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	7
_finalX:
       ds      1

psect	dataBANK0
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	8
_finalY:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	15
_somethingInTheWay:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	51
_xCoord:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	53
_xVictim:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	55
_xVirtual:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	52
_yCoord:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	54
_yVictim:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	56
_yVirtual:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_rxfifo:
       ds      16

_txfifo:
       ds      16

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK1
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	14
_beep:
       ds      5

psect	dataBANK1
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	15
_longbeep:
       ds      5

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	12
_finalCountdown:
       ds      27

psect	dataBANK3
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	10
_superMarioBros:
       ds      25

psect clrtext,class=CODE,delta=2
global clear_ram
;	Called with FSR containing the base address, and
;	W with the last address+1
clear_ram:
	clrwdt			;clear the watchdog before getting into this loop
clrloop:
	clrf	indf		;clear RAM location pointed to by FSR
	incf	fsr,f		;increment pointer
	xorwf	fsr,w		;XOR with final address
	btfsc	status,2	;have we reached the end yet?
	retlw	0		;all done for this memory range, return
	xorwf	fsr,w		;XOR again to restore value
	goto	clrloop		;do the next byte

; Clear objects allocated to BITCOMMON
psect cinit,class=CODE,delta=2
	clrf	((__pbitbssCOMMON/8)+0)&07Fh
	clrf	((__pbitbssCOMMON/8)+1)&07Fh
; Clear objects allocated to COMMON
psect cinit,class=CODE,delta=2
	clrf	((__pbssCOMMON)+0)&07Fh
	clrf	((__pbssCOMMON)+1)&07Fh
; Clear objects allocated to BANK0
psect cinit,class=CODE,delta=2
	bcf	status, 7	;select IRP bank0
	movlw	low(__pbssBANK0)
	movwf	fsr
	movlw	low((__pbssBANK0)+018h)
	fcall	clear_ram
; Clear objects allocated to BANK1
psect cinit,class=CODE,delta=2
	movlw	low(__pbssBANK1)
	movwf	fsr
	movlw	low((__pbssBANK1)+020h)
	fcall	clear_ram
global btemp
psect inittext,class=CODE,delta=2
global init_fetch,btemp
;	Called with low address in FSR and high address in W
init_fetch:
	movf btemp,w
	movwf pclath
	movf btemp+1,w
	movwf pc
global init_ram
;Called with:
;	high address of idata address in btemp 
;	low address of idata address in btemp+1 
;	low address of data in FSR
;	high address + 1 of data in btemp-1
init_ram:
	fcall init_fetch
	movwf indf,f
	incf fsr,f
	movf fsr,w
	xorwf btemp-1,w
	btfsc status,2
	retlw 0
	incf btemp+1,f
	btfsc status,2
	incf btemp,f
	goto init_ram
; Initialize objects allocated to BANK3
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK3
	bsf	status, 7	;select IRP bank2
	movlw low(__pdataBANK3+81)
	movwf btemp-1,f
	movlw high(__pidataBANK3)
	movwf btemp,f
	movlw low(__pidataBANK3)
	movwf btemp+1,f
	movlw low(__pdataBANK3)
	movwf fsr,f
	fcall init_ram
; Initialize objects allocated to BANK1
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK1
	bcf	status, 7	;select IRP bank0
	movlw low(__pdataBANK1+31)
	movwf btemp-1,f
	movlw high(__pidataBANK1)
	movwf btemp,f
	movlw low(__pidataBANK1)
	movwf btemp+1,f
	movlw low(__pdataBANK1)
	movwf fsr,f
	fcall init_ram
; Initialize objects allocated to BANK0
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK0
	movlw low(__pdataBANK0+9)
	movwf btemp-1,f
	movlw high(__pidataBANK0)
	movwf btemp,f
	movlw low(__pidataBANK0)
	movwf btemp+1,f
	movlw low(__pdataBANK0)
	movwf fsr,f
	fcall init_ram
psect cinit,class=CODE,delta=2
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
clrf status
ljmp _main	;jump to C main() function
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_lcd_write_string
?_lcd_write_string:	; 0 bytes @ 0x0
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
	global	?_goReverse
?_goReverse:	; 0 bytes @ 0x0
	global	?_clearSuccessfulDrive
?_clearSuccessfulDrive:	; 0 bytes @ 0x0
	global	?_turnRight90
?_turnRight90:	; 0 bytes @ 0x0
	global	?_updateOrientation
?_updateOrientation:	; 0 bytes @ 0x0
	global	?_turnLeft90
?_turnLeft90:	; 0 bytes @ 0x0
	global	?_turnAround
?_turnAround:	; 0 bytes @ 0x0
	global	?_initIRobot
?_initIRobot:	; 0 bytes @ 0x0
	global	?_findWall
?_findWall:	; 1 bit 
	global	?_wallFollow
?_wallFollow:	; 0 bytes @ 0x0
	global	?_init_adc
?_init_adc:	; 0 bytes @ 0x0
	global	?_writeSPIByte
?_writeSPIByte:	; 0 bytes @ 0x0
	global	?_initEEPROMMode
?_initEEPROMMode:	; 0 bytes @ 0x0
	global	?_addNewData
?_addNewData:	; 0 bytes @ 0x0
	global	?_writeEEPROMTestData
?_writeEEPROMTestData:	; 0 bytes @ 0x0
	global	?_EEPROMToSerial
?_EEPROMToSerial:	; 0 bytes @ 0x0
	global	?_lcd_write_control
?_lcd_write_control:	; 0 bytes @ 0x0
	global	?_lcd_write_data
?_lcd_write_data:	; 0 bytes @ 0x0
	global	?_lcd_set_cursor
?_lcd_set_cursor:	; 0 bytes @ 0x0
	global	?_lcd_write_1_digit_bcd
?_lcd_write_1_digit_bcd:	; 0 bytes @ 0x0
	global	?_lcd_init
?_lcd_init:	; 0 bytes @ 0x0
	global	?_ser_isrx
?_ser_isrx:	; 1 bit 
	global	?_ser_init
?_ser_init:	; 0 bytes @ 0x0
	global	?_play_iCreate_song
?_play_iCreate_song:	; 0 bytes @ 0x0
	global	?_initSongs
?_initSongs:	; 0 bytes @ 0x0
	global	?_getSuccessfulDrive
?_getSuccessfulDrive:	; 1 bit 
	global	?_goBackward
?_goBackward:	; 0 bytes @ 0x0
	global	?_goForward
?_goForward:	; 0 bytes @ 0x0
	global	?_goLeft
?_goLeft:	; 0 bytes @ 0x0
	global	?_goRight
?_goRight:	; 0 bytes @ 0x0
	global	?_frontWallCorrect
?_frontWallCorrect:	; 0 bytes @ 0x0
	global	?_isr1
?_isr1:	; 0 bytes @ 0x0
	global	??_isr1
??_isr1:	; 0 bytes @ 0x0
	global	?_init
?_init:	; 0 bytes @ 0x0
	global	?_checkForFinalDestination
?_checkForFinalDestination:	; 0 bytes @ 0x0
	global	?_lookForVictim
?_lookForVictim:	; 0 bytes @ 0x0
	global	?_findWalls
?_findWalls:	; 0 bytes @ 0x0
	global	?_goToNextCell
?_goToNextCell:	; 0 bytes @ 0x0
	global	?_updateLocation
?_updateLocation:	; 0 bytes @ 0x0
	global	?_updateNode
?_updateNode:	; 0 bytes @ 0x0
	global	?_checkIfHome
?_checkIfHome:	; 0 bytes @ 0x0
	global	?_main
?_main:	; 0 bytes @ 0x0
	global	?_getCurrentX
?_getCurrentX:	; 1 bytes @ 0x0
	global	?_getCurrentY
?_getCurrentY:	; 1 bytes @ 0x0
	global	?_getFinalX
?_getFinalX:	; 1 bytes @ 0x0
	global	?_getFinalY
?_getFinalY:	; 1 bytes @ 0x0
	global	?_ser_getch
?_ser_getch:	; 1 bytes @ 0x0
	global	?_getOrientation
?_getOrientation:	; 1 bytes @ 0x0
	global	?_getSomethingInTheWay
?_getSomethingInTheWay:	; 1 bytes @ 0x0
	ds	10
psect	cstackBANK0,class=BANK0,space=1
global __pcstackBANK0
__pcstackBANK0:
	global	??_ser_putch
??_ser_putch:	; 0 bytes @ 0x0
	global	??_clearSuccessfulDrive
??_clearSuccessfulDrive:	; 0 bytes @ 0x0
	global	??_updateOrientation
??_updateOrientation:	; 0 bytes @ 0x0
	global	??_getCurrentX
??_getCurrentX:	; 0 bytes @ 0x0
	global	??_getCurrentY
??_getCurrentY:	; 0 bytes @ 0x0
	global	?_setVirtualLocation
?_setVirtualLocation:	; 0 bytes @ 0x0
	global	??_init_adc
??_init_adc:	; 0 bytes @ 0x0
	global	??_writeSPIByte
??_writeSPIByte:	; 0 bytes @ 0x0
	global	??_initEEPROMMode
??_initEEPROMMode:	; 0 bytes @ 0x0
	global	??_lcd_write_control
??_lcd_write_control:	; 0 bytes @ 0x0
	global	??_lcd_write_data
??_lcd_write_data:	; 0 bytes @ 0x0
	global	??_getFinalX
??_getFinalX:	; 0 bytes @ 0x0
	global	??_getFinalY
??_getFinalY:	; 0 bytes @ 0x0
	global	??_ser_isrx
??_ser_isrx:	; 0 bytes @ 0x0
	global	??_ser_getch
??_ser_getch:	; 0 bytes @ 0x0
	global	??_ser_init
??_ser_init:	; 0 bytes @ 0x0
	global	?_rotateIR
?_rotateIR:	; 0 bytes @ 0x0
	global	??_getOrientation
??_getOrientation:	; 0 bytes @ 0x0
	global	??_getSomethingInTheWay
??_getSomethingInTheWay:	; 0 bytes @ 0x0
	global	??_getSuccessfulDrive
??_getSuccessfulDrive:	; 0 bytes @ 0x0
	global	??_updateNode
??_updateNode:	; 0 bytes @ 0x0
	global	?_getVictimZone
?_getVictimZone:	; 1 bytes @ 0x0
	global	?___wmul
?___wmul:	; 2 bytes @ 0x0
	global	writeSPIByte@data
writeSPIByte@data:	; 1 bytes @ 0x0
	global	getVictimZone@victimY
getVictimZone@victimY:	; 1 bytes @ 0x0
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0x0
	global	setVirtualLocation@yV
setVirtualLocation@yV:	; 1 bytes @ 0x0
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0x0
	ds	1
	global	?_writeEEPROM
?_writeEEPROM:	; 0 bytes @ 0x1
	global	??_getVictimZone
??_getVictimZone:	; 0 bytes @ 0x1
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0x1
	global	?_readEEPROM
?_readEEPROM:	; 1 bytes @ 0x1
	global	writeEEPROM@addressH
writeEEPROM@addressH:	; 1 bytes @ 0x1
	global	readEEPROM@addressL
readEEPROM@addressL:	; 1 bytes @ 0x1
	global	ser_getch@c
ser_getch@c:	; 1 bytes @ 0x1
	global	ser_putch@c
ser_putch@c:	; 1 bytes @ 0x1
	global	updateOrientation@moved
updateOrientation@moved:	; 1 bytes @ 0x1
	global	setVirtualLocation@dV
setVirtualLocation@dV:	; 1 bytes @ 0x1
	ds	1
	global	??_setVirtualLocation
??_setVirtualLocation:	; 0 bytes @ 0x2
	global	?_waitFor
?_waitFor:	; 0 bytes @ 0x2
	global	??_initIRobot
??_initIRobot:	; 0 bytes @ 0x2
	global	??_readEEPROM
??_readEEPROM:	; 0 bytes @ 0x2
	global	?_ser_putArr
?_ser_putArr:	; 0 bytes @ 0x2
	global	??_play_iCreate_song
??_play_iCreate_song:	; 0 bytes @ 0x2
	global	?_drive
?_drive:	; 0 bytes @ 0x2
	global	writeEEPROM@addressL
writeEEPROM@addressL:	; 1 bytes @ 0x2
	global	lcd_write_control@databyte
lcd_write_control@databyte:	; 1 bytes @ 0x2
	global	lcd_write_data@databyte
lcd_write_data@databyte:	; 1 bytes @ 0x2
	global	getVictimZone@victimX
getVictimZone@victimX:	; 1 bytes @ 0x2
	global	play_iCreate_song@song
play_iCreate_song@song:	; 1 bytes @ 0x2
	global	drive@lowByteSpeed
drive@lowByteSpeed:	; 1 bytes @ 0x2
	global	waitFor@highByte
waitFor@highByte:	; 1 bytes @ 0x2
	global	ser_putArr@array
ser_putArr@array:	; 2 bytes @ 0x2
	global	___wmul@multiplicand
___wmul@multiplicand:	; 2 bytes @ 0x2
	ds	1
	global	??_lcd_write_string
??_lcd_write_string:	; 0 bytes @ 0x3
	global	??_writeEEPROM
??_writeEEPROM:	; 0 bytes @ 0x3
	global	??_lcd_set_cursor
??_lcd_set_cursor:	; 0 bytes @ 0x3
	global	??_lcd_write_1_digit_bcd
??_lcd_write_1_digit_bcd:	; 0 bytes @ 0x3
	global	??_lcd_init
??_lcd_init:	; 0 bytes @ 0x3
	global	lcd_set_cursor@address
lcd_set_cursor@address:	; 1 bytes @ 0x3
	global	lcd_write_1_digit_bcd@data
lcd_write_1_digit_bcd@data:	; 1 bytes @ 0x3
	global	drive@highByteRadius
drive@highByteRadius:	; 1 bytes @ 0x3
	global	waitFor@lowByte
waitFor@lowByte:	; 1 bytes @ 0x3
	global	setVirtualLocation@xV
setVirtualLocation@xV:	; 1 bytes @ 0x3
	ds	1
	global	??_waitFor
??_waitFor:	; 0 bytes @ 0x4
	global	?_findFinalDestination
?_findFinalDestination:	; 0 bytes @ 0x4
	global	??_checkForFinalDestination
??_checkForFinalDestination:	; 0 bytes @ 0x4
	global	??_lookForVictim
??_lookForVictim:	; 0 bytes @ 0x4
	global	??_updateLocation
??_updateLocation:	; 0 bytes @ 0x4
	global	??___wmul
??___wmul:	; 0 bytes @ 0x4
	global	lcd_write_string@s
lcd_write_string@s:	; 1 bytes @ 0x4
	global	findFinalDestination@virtualWallY
findFinalDestination@virtualWallY:	; 1 bytes @ 0x4
	global	rotateIR@steps
rotateIR@steps:	; 1 bytes @ 0x4
	global	drive@lowByteRadius
drive@lowByteRadius:	; 1 bytes @ 0x4
	global	ser_putArr@length
ser_putArr@length:	; 2 bytes @ 0x4
	global	___wmul@product
___wmul@product:	; 2 bytes @ 0x4
	ds	1
	global	??_drive
??_drive:	; 0 bytes @ 0x5
	global	readEEPROM@addressH
readEEPROM@addressH:	; 1 bytes @ 0x5
	global	findFinalDestination@robotOrientation
findFinalDestination@robotOrientation:	; 1 bytes @ 0x5
	global	rotateIR@stepNum
rotateIR@stepNum:	; 1 bytes @ 0x5
	ds	1
	global	??_findFinalDestination
??_findFinalDestination:	; 0 bytes @ 0x6
	global	??_ser_putArr
??_ser_putArr:	; 0 bytes @ 0x6
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x6
	global	writeEEPROM@data
writeEEPROM@data:	; 1 bytes @ 0x6
	global	readEEPROM@returnData
readEEPROM@returnData:	; 1 bytes @ 0x6
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x6
	ds	1
	global	??_addNewData
??_addNewData:	; 0 bytes @ 0x7
	global	??_EEPROMToSerial
??_EEPROMToSerial:	; 0 bytes @ 0x7
	global	findFinalDestination@virtualWallX
findFinalDestination@virtualWallX:	; 1 bytes @ 0x7
	global	waitFor@type
waitFor@type:	; 1 bytes @ 0x7
	global	lookForVictim@victim
lookForVictim@victim:	; 1 bytes @ 0x7
	ds	1
	global	addNewData@data
addNewData@data:	; 1 bytes @ 0x8
	global	drive@highByteSpeed
drive@highByteSpeed:	; 1 bytes @ 0x8
	global	___awdiv@dividend
___awdiv@dividend:	; 2 bytes @ 0x8
	ds	1
	global	??_goReverse
??_goReverse:	; 0 bytes @ 0x9
	global	??_turnRight90
??_turnRight90:	; 0 bytes @ 0x9
	global	??_turnLeft90
??_turnLeft90:	; 0 bytes @ 0x9
	global	??_turnAround
??_turnAround:	; 0 bytes @ 0x9
	global	??_writeEEPROMTestData
??_writeEEPROMTestData:	; 0 bytes @ 0x9
	global	?_updateMapData
?_updateMapData:	; 0 bytes @ 0x9
	global	??_checkIfHome
??_checkIfHome:	; 0 bytes @ 0x9
	global	updateMapData@virtualS
updateMapData@virtualS:	; 1 bytes @ 0x9
	global	ser_putArr@i
ser_putArr@i:	; 2 bytes @ 0x9
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0xA
	global	updateMapData@virtualE
updateMapData@virtualE:	; 1 bytes @ 0xA
	global	EEPROMToSerial@transferDone
EEPROMToSerial@transferDone:	; 1 bytes @ 0xA
	ds	1
	global	??_initSongs
??_initSongs:	; 0 bytes @ 0xB
	global	??_init
??_init:	; 0 bytes @ 0xB
	global	updateMapData@virtualN
updateMapData@virtualN:	; 1 bytes @ 0xB
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0xB
	ds	1
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0xC
	global	updateMapData@victim
updateMapData@victim:	; 1 bytes @ 0xC
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0xC
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0xC
	ds	1
	global	updateMapData@move
updateMapData@move:	; 1 bytes @ 0xD
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0xD
	ds	1
	global	??_updateMapData
??_updateMapData:	; 0 bytes @ 0xE
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0xE
	ds	1
	global	?_adc_read
?_adc_read:	; 2 bytes @ 0xF
	ds	1
	global	updateMapData@virtualW
updateMapData@virtualW:	; 1 bytes @ 0x10
	global	driveForDistance@deltaDistance
driveForDistance@deltaDistance:	; 2 bytes @ 0x10
	ds	1
	global	??_adc_read
??_adc_read:	; 0 bytes @ 0x11
	global	updateMapData@completeData
updateMapData@completeData:	; 1 bytes @ 0x11
	ds	1
	global	driveForDistance@distance
driveForDistance@distance:	; 2 bytes @ 0x12
	ds	2
	global	driveForDistance@high
driveForDistance@high:	; 1 bytes @ 0x14
	ds	1
	global	driveForDistance@low
driveForDistance@low:	; 1 bytes @ 0x15
	global	adc_read@adc_value
adc_read@adc_value:	; 2 bytes @ 0x15
	ds	1
	global	driveForDistance@virtualWall
driveForDistance@virtualWall:	; 1 bytes @ 0x16
	ds	1
	global	?_convert
?_convert:	; 2 bytes @ 0x17
	global	driveForDistance@cliff
driveForDistance@cliff:	; 1 bytes @ 0x17
	global	convert@adc_value
convert@adc_value:	; 2 bytes @ 0x17
	ds	1
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x18
	global	??_goForward
??_goForward:	; 0 bytes @ 0x18
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x18
	global	??_goRight
??_goRight:	; 0 bytes @ 0x18
	ds	1
	global	??_convert
??_convert:	; 0 bytes @ 0x19
	global	??_goToNextCell
??_goToNextCell:	; 0 bytes @ 0x19
	ds	2
	global	?_adc_read_channel
?_adc_read_channel:	; 2 bytes @ 0x1B
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x1D
	ds	1
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x1E
	ds	1
	global	?_readIR
?_readIR:	; 2 bytes @ 0x1F
	ds	2
	global	??_readIR
??_readIR:	; 0 bytes @ 0x21
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x21
	ds	2
	global	??_findWall
??_findWall:	; 0 bytes @ 0x23
	global	??_wallFollow
??_wallFollow:	; 0 bytes @ 0x23
	global	??_frontWallCorrect
??_frontWallCorrect:	; 0 bytes @ 0x23
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x23
	ds	2
	global	frontWallCorrect@distToWall
frontWallCorrect@distToWall:	; 2 bytes @ 0x25
	ds	1
	global	wallFollow@distanceToWall
wallFollow@distanceToWall:	; 2 bytes @ 0x26
	ds	2
	global	??_main
??_main:	; 0 bytes @ 0x28
	ds	2
	global	main@victimIndicator
main@victimIndicator:	; 1 bytes @ 0x2A
	ds	1
;;Data sizes: Strings 132, constant 0, data 121, bss 58, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14     10      14
;; BANK0           80     43      76
;; BANK1           80      0      63
;; BANK3           96      0      81
;; BANK2           96      0       0

;;
;; Pointer list with targets:

;; ?_readIR	int  size(1) Largest target is 0
;;
;; ?_convert	int  size(1) Largest target is 2
;;		 -> convert@adc_value(BANK0[2]), 
;;
;; ?___wmul	unsigned int  size(1) Largest target is 0
;;
;; ?_adc_read	int  size(1) Largest target is 0
;;
;; ?___awdiv	int  size(1) Largest target is 0
;;
;; ?_adc_read_channel	int  size(1) Largest target is 0
;;
;; ser_putArr@array	PTR unsigned char  size(2) Largest target is 29
;;		 -> longbeep(BANK1[5]), beep(BANK1[5]), champions(BANK1[21]), lookingForU2(BANK3[29]), 
;;		 -> superMarioBros(BANK3[25]), finalCountdown(BANK3[27]), 
;;
;; lcd_write_string@s	PTR const unsigned char  size(1) Largest target is 23
;;		 -> STR_7(CODE[23]), STR_6(CODE[23]), STR_5(CODE[23]), STR_4(CODE[17]), 
;;		 -> STR_3(CODE[17]), STR_2(CODE[14]), STR_1(CODE[15]), 
;;


;;
;; Critical Paths under _main in COMMON
;;
;;   None.
;;
;; Critical Paths under _isr1 in COMMON
;;
;;   None.
;;
;; Critical Paths under _main in BANK0
;;
;;   _main->_wallFollow
;;   _goToNextCell->_goRight
;;   _goToNextCell->_goBackward
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _wallFollow->_readIR
;;   _findWall->_readIR
;;   _frontWallCorrect->_readIR
;;   _driveForDistance->_goReverse
;;   _driveForDistance->_turnRight90
;;   _driveForDistance->_turnLeft90
;;   _updateLocation->_lcd_set_cursor
;;   _updateLocation->_lcd_write_1_digit_bcd
;;   _lookForVictim->_lcd_set_cursor
;;   _lookForVictim->_lcd_write_1_digit_bcd
;;   _checkForFinalDestination->_lcd_set_cursor
;;   _goReverse->_drive
;;   _readIR->_adc_read_channel
;;   _findFinalDestination->_lcd_set_cursor
;;   _findFinalDestination->_lcd_write_1_digit_bcd
;;   _updateMapData->_addNewData
;;   _writeEEPROMTestData->_addNewData
;;   _checkIfHome->_drive
;;   _turnAround->_drive
;;   _turnLeft90->_drive
;;   _turnRight90->_drive
;;   _initSongs->_ser_putArr
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_set_cursor->_lcd_write_control
;;   _EEPROMToSerial->_readEEPROM
;;   _addNewData->_writeEEPROM
;;   _lcd_write_string->_lcd_write_data
;;   _adc_read_channel->_convert
;;   _initIRobot->_ser_putch
;;   _waitFor->_ser_putch
;;   _drive->_ser_putch
;;   _convert->_adc_read
;;   _play_iCreate_song->_ser_putch
;;   _ser_putArr->_ser_putch
;;   _readEEPROM->_initEEPROMMode
;;   _readEEPROM->_writeSPIByte
;;   _writeEEPROM->_initEEPROMMode
;;   _writeEEPROM->_writeSPIByte
;;   _adc_read->___awdiv
;;   ___awdiv->___wmul
;;
;; Critical Paths under _isr1 in BANK0
;;
;;   None.
;;
;; Critical Paths under _main in BANK1
;;
;;   None.
;;
;; Critical Paths under _isr1 in BANK1
;;
;;   None.
;;
;; Critical Paths under _main in BANK3
;;
;;   None.
;;
;; Critical Paths under _isr1 in BANK3
;;
;;   None.
;;
;; Critical Paths under _main in BANK2
;;
;;   None.
;;
;; Critical Paths under _isr1 in BANK2
;;
;;   None.

;;
;;Main: autosize = 0, tempsize = 2, incstack = 0, save=0
;;

;;
;;Call Graph Tables:
;;
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (0) _main                                                 3     3      0   17862
;;                                             40 BANK0      3     3      0
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;                          _findWalls
;;                         _turnAround
;;                         _turnLeft90
;;                        _turnRight90
;;                     _lcd_write_data
;;                  _play_iCreate_song
;;                _writeEEPROMTestData
;;                     _EEPROMToSerial
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                           _rotateIR
;;                         _wallFollow
;;                   _frontWallCorrect
;;                       _goToNextCell
;;                            _goRight
;;                     _getOrientation
;;                          _goForward
;;                             _goLeft
;;                 _getSuccessfulDrive
;;                      _updateMapData
;;                     _updateLocation
;;                         _updateNode
;;                        _checkIfHome
;; ---------------------------------------------------------------------------------
;; (1) _goToNextCell                                         0     0      0    6704
;;               _getSomethingInTheWay
;;                             _goLeft
;;                          _goForward
;;                            _goRight
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (1) _findWalls                                            1     1      0    1225
;;                                             35 BANK0      1     1      0
;;                           _rotateIR
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                  _play_iCreate_song
;; ---------------------------------------------------------------------------------
;; (1) _goRight                                              1     1      0    1731
;;                                             24 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goLeft                                               0     0      0    1731
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goForward                                            0     0      0    1511
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _getCurrentX
;;                        _getCurrentY
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0    1731
;;                                             24 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnAround
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _wallFollow                                           5     5      0    1311
;;                                             35 BANK0      5     5      0
;;                             _readIR
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1046
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (1) _frontWallCorrect                                     4     4      0    1247
;;                                             35 BANK0      4     4      0
;;                             _readIR
;;                              _drive
;;               _clearSuccessfulDrive
;; ---------------------------------------------------------------------------------
;; (2) _driveForDistance                                    12    10      2    1444
;;                                             12 BANK0     12    10      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;;                          _goReverse
;;               _clearSuccessfulDrive
;;                        _turnRight90
;;                  _updateOrientation
;;                         _turnLeft90
;;                        _getCurrentY
;;                        _getCurrentX
;;               _findFinalDestination
;;                 _setVirtualLocation
;; ---------------------------------------------------------------------------------
;; (1) _updateLocation                                       1     1      0     111
;;                                              4 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                     _getOrientation
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _lookForVictim                                        4     4      0     377
;;                                              4 BANK0      4     4      0
;;                          _ser_putch
;;                          _ser_getch
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                      _getVictimZone
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _checkForFinalDestination                             2     2      0     111
;;                                              4 BANK0      2     2      0
;;                          _getFinalX
;;                          _getFinalY
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (1) _init                                                 0     0      0     156
;;                           _init_adc
;;                           _lcd_init
;;                           _ser_init
;;                         _initIRobot
;;                          _initSongs
;; ---------------------------------------------------------------------------------
;; (3) _goReverse                                            3     3      0     265
;;                                              9 BANK0      3     3      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _readIR                                               4     2      2    1046
;;                                             31 BANK0      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (3) _findFinalDestination                                 4     2      2     287
;;                                              4 BANK0      4     2      2
;;                     _lcd_set_cursor
;;              _lcd_write_1_digit_bcd
;;                        _getCurrentY (ARG)
;;                        _getCurrentX (ARG)
;; ---------------------------------------------------------------------------------
;; (1) _updateMapData                                        9     4      5     272
;;                                              9 BANK0      9     4      5
;;                         _addNewData
;;                     _getOrientation (ARG)
;; ---------------------------------------------------------------------------------
;; (1) _writeEEPROMTestData                                  0     0      0     110
;;                         _addNewData
;; ---------------------------------------------------------------------------------
;; (1) _checkIfHome                                          0     0      0     154
;;                              _drive
;;                  _play_iCreate_song
;; ---------------------------------------------------------------------------------
;; (3) _turnAround                                           3     3      0     198
;;                                              9 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnLeft90                                           3     3      0     198
;;                                              9 BANK0      3     3      0
;;                              _drive
;;                        _getCurrentX
;;                        _getCurrentY
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnRight90                                          3     3      0     198
;;                                              9 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _initSongs                                            0     0      0     112
;;                         _ser_putArr
;; ---------------------------------------------------------------------------------
;; (2) _lcd_init                                             0     0      0      22
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (2) _lcd_write_1_digit_bcd                                1     1      0      44
;;                                              3 BANK0      1     1      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _lcd_set_cursor                                       1     1      0      45
;;                                              3 BANK0      1     1      0
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (1) _EEPROMToSerial                                       4     4      0     136
;;                                              7 BANK0      4     4      0
;;                         _readEEPROM
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _addNewData                                           2     2      0     110
;;                                              7 BANK0      2     2      0
;;                        _writeEEPROM
;; ---------------------------------------------------------------------------------
;; (1) _lcd_write_string                                     2     2      0      67
;;                                              3 BANK0      2     2      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _adc_read_channel                                     4     2      2     345
;;                                             27 BANK0      4     2      2
;;                           _adc_read
;;                            _convert (ARG)
;; ---------------------------------------------------------------------------------
;; (2) _initIRobot                                           3     3      0      22
;;                                              2 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (4) _waitFor                                              6     4      2      88
;;                                              2 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _drive                                                7     4      3     110
;;                                              2 BANK0      7     4      3
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (1) _rotateIR                                             6     5      1      68
;;                                              0 BANK0      6     5      1
;; ---------------------------------------------------------------------------------
;; (3) _convert                                              4     2      2     678
;;                                             23 BANK0      4     2      2
;;                             ___wmul
;;                            ___awdiv
;;                           _adc_read (ARG)
;; ---------------------------------------------------------------------------------
;; (2) _play_iCreate_song                                    1     1      0      44
;;                                              2 BANK0      1     1      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _ser_putArr                                           9     5      4     112
;;                                              2 BANK0      9     5      4
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _ser_getch                                            2     2      0      23
;;                                              0 BANK0      2     2      0
;;                           _ser_isrx
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_data                                       3     3      0      22
;;                                              0 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_control                                    3     3      0      22
;;                                              0 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (2) _readEEPROM                                           6     5      1      90
;;                                              1 BANK0      6     5      1
;;                     _initEEPROMMode
;;                       _writeSPIByte
;; ---------------------------------------------------------------------------------
;; (3) _writeEEPROM                                          6     4      2      88
;;                                              1 BANK0      6     4      2
;;                     _initEEPROMMode
;;                       _writeSPIByte
;; ---------------------------------------------------------------------------------
;; (2) _init_adc                                             1     1      0       0
;;                                              0 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _adc_read                                             8     6      2     323
;;                                             15 BANK0      8     6      2
;;                            ___awdiv
;; ---------------------------------------------------------------------------------
;; (4) ___awdiv                                              9     5      4     300
;;                                              6 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (4) ___wmul                                               6     2      4      92
;;                                              0 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (1) _updateNode                                           1     1      0       0
;;                                              0 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (1) _getSuccessfulDrive                                   0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getSomethingInTheWay                                 0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getOrientation                                       0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _setVirtualLocation                                   4     2      2      66
;;                                              0 BANK0      4     2      2
;;                        _getCurrentY (ARG)
;;                        _getCurrentX (ARG)
;; ---------------------------------------------------------------------------------
;; (3) _getCurrentY                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _getCurrentX                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _updateOrientation                                    2     2      0      22
;;                                              0 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (3) _clearSuccessfulDrive                                 0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _ser_init                                             1     1      0       0
;;                                              0 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _ser_isrx                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getVictimZone                                        3     2      1     132
;;                                              0 BANK0      3     2      1
;; ---------------------------------------------------------------------------------
;; (2) _getFinalY                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getFinalX                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _ser_putch                                            2     2      0      22
;;                                              0 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (4) _initEEPROMMode                                       1     1      0       0
;;                                              0 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _writeSPIByte                                         1     1      0      22
;;                                              0 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 4
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (7) _isr1                                                10    10      0       0
;;                                              0 COMMON    10    10      0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 7
;; ---------------------------------------------------------------------------------

;; Call Graph Graphs:

;; _main (ROOT)
;;   _init
;;     _init_adc
;;     _lcd_init
;;       _lcd_write_control
;;     _ser_init
;;     _initIRobot
;;       _ser_putch
;;     _initSongs
;;       _ser_putArr
;;         _ser_putch
;;   _drive
;;     _ser_putch
;;   _lcd_set_cursor
;;     _lcd_write_control
;;   _lcd_write_string
;;     _lcd_write_data
;;   _findWalls
;;     _rotateIR
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _findWall
;;       _readIR
;;         _adc_read_channel
;;           _adc_read
;;             ___awdiv
;;               ___wmul (ARG)
;;           _convert (ARG)
;;             ___wmul
;;             ___awdiv
;;               ___wmul (ARG)
;;             _adc_read (ARG)
;;               ___awdiv
;;                 ___wmul (ARG)
;;         _convert
;;           ___wmul
;;           ___awdiv
;;             ___wmul (ARG)
;;           _adc_read (ARG)
;;             ___awdiv
;;               ___wmul (ARG)
;;     _lcd_write_data
;;     _play_iCreate_song
;;       _ser_putch
;;   _turnAround
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _turnLeft90
;;     _drive
;;       _ser_putch
;;     _getCurrentX
;;     _getCurrentY
;;     _waitFor
;;       _ser_putch
;;   _turnRight90
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _lcd_write_data
;;   _play_iCreate_song
;;     _ser_putch
;;   _writeEEPROMTestData
;;     _addNewData
;;       _writeEEPROM
;;         _initEEPROMMode
;;         _writeSPIByte
;;   _EEPROMToSerial
;;     _readEEPROM
;;       _initEEPROMMode
;;       _writeSPIByte
;;     _ser_putch
;;   _checkForFinalDestination
;;     _getFinalX
;;     _getFinalY
;;     _play_iCreate_song
;;       _ser_putch
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;   _lookForVictim
;;     _ser_putch
;;     _ser_getch
;;       _ser_isrx
;;     _play_iCreate_song
;;       _ser_putch
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _getVictimZone
;;     _lcd_write_1_digit_bcd
;;       _lcd_write_data
;;   _rotateIR
;;   _wallFollow
;;     _readIR
;;       _adc_read_channel
;;         _adc_read
;;           ___awdiv
;;             ___wmul (ARG)
;;         _convert (ARG)
;;           ___wmul
;;           ___awdiv
;;             ___wmul (ARG)
;;           _adc_read (ARG)
;;             ___awdiv
;;               ___wmul (ARG)
;;       _convert
;;         ___wmul
;;         ___awdiv
;;           ___wmul (ARG)
;;         _adc_read (ARG)
;;           ___awdiv
;;             ___wmul (ARG)
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _frontWallCorrect
;;     _readIR
;;       _adc_read_channel
;;         _adc_read
;;           ___awdiv
;;             ___wmul (ARG)
;;         _convert (ARG)
;;           ___wmul
;;           ___awdiv
;;             ___wmul (ARG)
;;           _adc_read (ARG)
;;             ___awdiv
;;               ___wmul (ARG)
;;       _convert
;;         ___wmul
;;         ___awdiv
;;           ___wmul (ARG)
;;         _adc_read (ARG)
;;           ___awdiv
;;             ___wmul (ARG)
;;     _drive
;;       _ser_putch
;;     _clearSuccessfulDrive
;;   _goToNextCell
;;     _getSomethingInTheWay
;;     _goLeft
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _getCurrentX
;;         _getCurrentY
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _clearSuccessfulDrive
;;         _turnRight90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _updateOrientation
;;         _turnLeft90
;;           _drive
;;             _ser_putch
;;           _getCurrentX
;;           _getCurrentY
;;           _waitFor
;;             _ser_putch
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;         _setVirtualLocation
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;     _goForward
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;       _getCurrentX
;;       _getCurrentY
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _clearSuccessfulDrive
;;         _turnRight90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _updateOrientation
;;         _turnLeft90
;;           _drive
;;             _ser_putch
;;           _getCurrentX
;;           _getCurrentY
;;           _waitFor
;;             _ser_putch
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;         _setVirtualLocation
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;     _goRight
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _clearSuccessfulDrive
;;         _turnRight90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _updateOrientation
;;         _turnLeft90
;;           _drive
;;             _ser_putch
;;           _getCurrentX
;;           _getCurrentY
;;           _waitFor
;;             _ser_putch
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;         _setVirtualLocation
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;     _goBackward
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;       _turnAround
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _clearSuccessfulDrive
;;         _turnRight90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _updateOrientation
;;         _turnLeft90
;;           _drive
;;             _ser_putch
;;           _getCurrentX
;;           _getCurrentY
;;           _waitFor
;;             _ser_putch
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;         _setVirtualLocation
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
;;   _goRight
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _turnRight90
;;       _drive
;;         _ser_putch
;;       _waitFor
;;         _ser_putch
;;     _updateOrientation
;;     _driveForDistance
;;       _drive
;;         _ser_putch
;;       _ser_putch
;;       _ser_getch
;;         _ser_isrx
;;       _goReverse
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _clearSuccessfulDrive
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _getCurrentX
;;         _getCurrentY
;;         _waitFor
;;           _ser_putch
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;       _setVirtualLocation
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;   _getOrientation
;;   _goForward
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _getCurrentX
;;     _getCurrentY
;;     _driveForDistance
;;       _drive
;;         _ser_putch
;;       _ser_putch
;;       _ser_getch
;;         _ser_isrx
;;       _goReverse
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _clearSuccessfulDrive
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _getCurrentX
;;         _getCurrentY
;;         _waitFor
;;           _ser_putch
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;       _setVirtualLocation
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;   _goLeft
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _turnLeft90
;;       _drive
;;         _ser_putch
;;       _getCurrentX
;;       _getCurrentY
;;       _waitFor
;;         _ser_putch
;;     _updateOrientation
;;     _driveForDistance
;;       _drive
;;         _ser_putch
;;       _ser_putch
;;       _ser_getch
;;         _ser_isrx
;;       _goReverse
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _clearSuccessfulDrive
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _updateOrientation
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _getCurrentX
;;         _getCurrentY
;;         _waitFor
;;           _ser_putch
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;       _setVirtualLocation
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
;;   _getSuccessfulDrive
;;   _updateMapData
;;     _addNewData
;;       _writeEEPROM
;;         _initEEPROMMode
;;         _writeSPIByte
;;     _getOrientation (ARG)
;;   _updateLocation
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _getOrientation
;;     _lcd_write_1_digit_bcd
;;       _lcd_write_data
;;   _updateNode
;;   _checkIfHome
;;     _drive
;;       _ser_putch
;;     _play_iCreate_song
;;       _ser_putch
;;
;; _isr1 (ROOT)
;;

;; Address spaces:

;;Name               Size   Autos  Total    Cost      Usage
;;BANK3               60      0      51       9       84.4%
;;BITBANK3            60      0       0       8        0.0%
;;SFR3                 0      0       0       4        0.0%
;;BITSFR3              0      0       0       4        0.0%
;;BANK2               60      0       0      11        0.0%
;;BITBANK2            60      0       0      10        0.0%
;;SFR2                 0      0       0       5        0.0%
;;BITSFR2              0      0       0       5        0.0%
;;SFR1                 0      0       0       2        0.0%
;;BITSFR1              0      0       0       2        0.0%
;;BANK1               50      0      3F       7       78.8%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      F0      12        0.0%
;;ABS                  0      0      EA       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       6       2        0.0%
;;BANK0               50     2B      4C       5       95.0%
;;BITBANK0            50      0       0       4        0.0%
;;SFR0                 0      0       0       1        0.0%
;;BITSFR0              0      0       0       1        0.0%
;;COMMON               E      A       E       1      100.0%
;;BITCOMMON            E      0       2       0       14.3%
;;EEDATA             100      0       0       0        0.0%

	global	_main
psect	maintext,global,class=CODE,delta=2
global __pmaintext
__pmaintext:

;; *************** function _main *****************
;; Defined at:
;;		line 367 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  victimIndica    1   42[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 17F/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_init
;;		_drive
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_findWalls
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_lcd_write_data
;;		_play_iCreate_song
;;		_writeEEPROMTestData
;;		_EEPROMToSerial
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_rotateIR
;;		_wallFollow
;;		_frontWallCorrect
;;		_goToNextCell
;;		_goRight
;;		_getOrientation
;;		_goForward
;;		_goLeft
;;		_getSuccessfulDrive
;;		_updateMapData
;;		_updateLocation
;;		_updateNode
;;		_checkIfHome
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	367
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 1
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	368
	
l11488:	
;main.c: 368: init();
	fcall	_init
	line	369
;main.c: 369: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	371
	
l11490:	
;main.c: 371: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	372
	
l11492:	
;main.c: 372: lcd_write_string("(-,-) - -- --- -");
	movlw	((STR_3-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	373
;main.c: 373: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	374
	
l11494:	
;main.c: 374: lcd_write_string("- - - (3,1) GREG");
	movlw	((STR_4-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	375
	
l11496:	
;main.c: 375: char victimIndicator = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(main@victimIndicator)
	line	376
;main.c: 376: while(!home)
	goto	l11666
	
l6812:	
	line	379
	
l11498:	
;main.c: 377: {
;main.c: 379: if(start.pressed && ready == 0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u4990
	goto	l11526
u4990:
	
l11500:	
	btfsc	(_ready/8),(_ready)&7
	goto	u5001
	goto	u5000
u5001:
	goto	l11526
u5000:
	line	381
	
l11502:	
;main.c: 380: {
;main.c: 381: findWalls();
	fcall	_findWalls
	line	382
	
l11504:	
;main.c: 382: if(leftWall && rightWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u5011
	goto	u5010
u5011:
	goto	l6814
u5010:
	
l11506:	
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u5021
	goto	u5020
u5021:
	goto	l6814
u5020:
	
l11508:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u5031
	goto	u5030
u5031:
	goto	l6814
u5030:
	line	383
	
l11510:	
;main.c: 383: turnAround();
	fcall	_turnAround
	goto	l11520
	line	384
	
l6814:	
;main.c: 384: else if (rightWall && frontWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u5041
	goto	u5040
u5041:
	goto	l6816
u5040:
	
l11512:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u5051
	goto	u5050
u5051:
	goto	l6816
u5050:
	line	385
	
l11514:	
;main.c: 385: turnLeft90();
	fcall	_turnLeft90
	goto	l11520
	line	386
	
l6816:	
;main.c: 386: else if(leftWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u5061
	goto	u5060
u5061:
	goto	l11520
u5060:
	
l11516:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u5071
	goto	u5070
u5071:
	goto	l11520
u5070:
	line	387
	
l11518:	
;main.c: 387: turnRight90();
	fcall	_turnRight90
	goto	l11520
	
l6818:	
	goto	l11520
	line	388
	
l6817:	
	goto	l11520
	
l6815:	
	
l11520:	
;main.c: 388: ready = 1;
	bsf	(_ready/8),(_ready)&7
	line	389
	
l11522:	
;main.c: 389: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	390
	
l11524:	
;main.c: 390: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	391
;main.c: 391: play_iCreate_song(1);
	movlw	(01h)
	fcall	_play_iCreate_song
	goto	l11526
	line	392
	
l6813:	
	line	396
	
l11526:	
;main.c: 392: }
;main.c: 396: if(eepromSerial.pressed && ready == 0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_eepromSerial),w
	skipz
	goto	u5080
	goto	l11544
u5080:
	
l11528:	
	btfsc	(_ready/8),(_ready)&7
	goto	u5091
	goto	u5090
u5091:
	goto	l11544
u5090:
	line	398
	
l11530:	
;main.c: 397: {
;main.c: 398: eepromSerial.pressed = 0;
	clrf	(_eepromSerial)
	line	399
	
l11532:	
;main.c: 399: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	400
	
l11534:	
;main.c: 400: lcd_write_string("EEPROM Serial         ");
	movlw	((STR_5-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	401
	
l11536:	
;main.c: 401: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	402
;main.c: 402: lcd_write_string("Please Wait...        ");
	movlw	((STR_6-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	403
	
l11538:	
;main.c: 403: writeEEPROMTestData();
	fcall	_writeEEPROMTestData
	line	404
	
l11540:	
;main.c: 404: EEPROMToSerial();
	fcall	_EEPROMToSerial
	line	405
;main.c: 405: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	406
	
l11542:	
;main.c: 406: lcd_write_string("Complete              ");
	movlw	((STR_7-__stringbase))&0ffh
	fcall	_lcd_write_string
	goto	l11544
	line	407
	
l6819:	
	line	409
	
l11544:	
;main.c: 407: }
;main.c: 409: if(start.pressed )
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u5100
	goto	l11666
u5100:
	line	411
	
l11546:	
;main.c: 410: {
;main.c: 411: ready = 1;
	bsf	(_ready/8),(_ready)&7
	line	412
	
l11548:	
;main.c: 412: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	414
;main.c: 414: lookForVictim();
	fcall	_lookForVictim
	line	416
	
l11550:	
;main.c: 416: findWalls();
	fcall	_findWalls
	line	417
	
l11552:	
;main.c: 417: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	418
	
l11554:	
;main.c: 418: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u5111
	goto	u5110
u5111:
	goto	l11562
u5110:
	line	420
	
l11556:	
;main.c: 419: {
;main.c: 420: rotateIR(24,0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	421
	
l11558:	
;main.c: 421: wallFollow();
	fcall	_wallFollow
	line	422
	
l11560:	
;main.c: 422: rotateIR(24,0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	goto	l11562
	line	423
	
l6821:	
	line	424
	
l11562:	
;main.c: 423: }
;main.c: 424: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	425
	
l11564:	
;main.c: 425: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u5121
	goto	u5120
u5121:
	goto	l11568
u5120:
	line	426
	
l11566:	
;main.c: 426: frontWallCorrect();
	fcall	_frontWallCorrect
	goto	l11568
	
l6822:	
	line	427
	
l11568:	
;main.c: 427: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	428
;main.c: 428: switch(node)
	goto	l11642
	line	430
;main.c: 429: {
;main.c: 430: case 0:
	
l6824:	
	line	431
	
l11570:	
;main.c: 431: goToNextCell();
	fcall	_goToNextCell
	line	432
;main.c: 432: break;
	goto	l11644
	line	433
;main.c: 433: case 1:
	
l6826:	
	line	434
;main.c: 434: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u5131
	goto	u5130
u5131:
	goto	l11586
u5130:
	line	436
	
l11572:	
;main.c: 435: {
;main.c: 436: if (victimZone == 1)
	movf	(_victimZone),w	;volatile
	xorlw	01h
	skipz
	goto	u5141
	goto	u5140
u5141:
	goto	l11576
u5140:
	line	437
	
l11574:	
;main.c: 437: goRight();
	fcall	_goRight
	goto	l11644
	line	438
	
l6828:	
	
l11576:	
;main.c: 438: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u5151
	goto	u5150
u5151:
	goto	l11580
u5150:
	line	439
	
l11578:	
;main.c: 439: goForward();
	fcall	_goForward
	goto	l11644
	line	440
	
l6830:	
	
l11580:	
;main.c: 440: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u5161
	goto	u5160
u5161:
	goto	l11584
u5160:
	line	441
	
l11582:	
;main.c: 441: goRight();
	fcall	_goRight
	goto	l11644
	line	442
	
l6832:	
	line	443
	
l11584:	
;main.c: 442: else
;main.c: 443: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6833:	
	goto	l11644
	
l6831:	
	goto	l11644
	
l6829:	
	line	444
;main.c: 444: }
	goto	l11644
	line	445
	
l6827:	
	line	446
	
l11586:	
;main.c: 445: else
;main.c: 446: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6834:	
	line	447
;main.c: 447: break;
	goto	l11644
	line	448
;main.c: 448: case 2:
	
l6835:	
	line	449
;main.c: 449: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u5171
	goto	u5170
u5171:
	goto	l11602
u5170:
	line	451
	
l11588:	
;main.c: 450: {
;main.c: 451: if (victimZone == 2)
	movf	(_victimZone),w	;volatile
	xorlw	02h
	skipz
	goto	u5181
	goto	u5180
u5181:
	goto	l11592
u5180:
	line	452
	
l11590:	
;main.c: 452: goForward();
	fcall	_goForward
	goto	l11644
	line	453
	
l6837:	
	
l11592:	
;main.c: 453: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u5191
	goto	u5190
u5191:
	goto	l11596
u5190:
	line	454
	
l11594:	
;main.c: 454: goRight();
	fcall	_goRight
	goto	l11644
	line	455
	
l6839:	
	
l11596:	
;main.c: 455: else if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u5201
	goto	u5200
u5201:
	goto	l11600
u5200:
	line	456
	
l11598:	
;main.c: 456: goLeft();
	fcall	_goLeft
	goto	l11644
	line	457
	
l6841:	
	line	458
	
l11600:	
;main.c: 457: else
;main.c: 458: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6842:	
	goto	l11644
	
l6840:	
	goto	l11644
	
l6838:	
	line	459
;main.c: 459: }
	goto	l11644
	line	460
	
l6836:	
	line	461
	
l11602:	
;main.c: 460: else
;main.c: 461: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6843:	
	line	462
;main.c: 462: break;
	goto	l11644
	line	463
;main.c: 463: case 3:
	
l6844:	
	line	464
;main.c: 464: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u5211
	goto	u5210
u5211:
	goto	l11618
u5210:
	line	466
	
l11604:	
;main.c: 465: {
;main.c: 466: if (victimZone == 3)
	movf	(_victimZone),w	;volatile
	xorlw	03h
	skipz
	goto	u5221
	goto	u5220
u5221:
	goto	l11608
u5220:
	line	467
	
l11606:	
;main.c: 467: goRight();
	fcall	_goRight
	goto	l11644
	line	468
	
l6846:	
	
l11608:	
;main.c: 468: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u5231
	goto	u5230
u5231:
	goto	l11612
u5230:
	line	469
	
l11610:	
;main.c: 469: goForward();
	fcall	_goForward
	goto	l11644
	line	470
	
l6848:	
	
l11612:	
;main.c: 470: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u5241
	goto	u5240
u5241:
	goto	l11616
u5240:
	line	471
	
l11614:	
;main.c: 471: goLeft();
	fcall	_goLeft
	goto	l11644
	line	472
	
l6850:	
	line	473
	
l11616:	
;main.c: 472: else
;main.c: 473: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6851:	
	goto	l11644
	
l6849:	
	goto	l11644
	
l6847:	
	line	474
;main.c: 474: }
	goto	l11644
	line	475
	
l6845:	
	line	476
	
l11618:	
;main.c: 475: else
;main.c: 476: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6852:	
	line	477
;main.c: 477: break;
	goto	l11644
	line	478
;main.c: 478: case 4:
	
l6853:	
	line	479
	
l11620:	
;main.c: 479: if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u5251
	goto	u5250
u5251:
	goto	l11624
u5250:
	line	480
	
l11622:	
;main.c: 480: goRight();
	fcall	_goRight
	goto	l11644
	line	481
	
l6854:	
	line	482
	
l11624:	
;main.c: 481: else
;main.c: 482: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6855:	
	line	483
;main.c: 483: break;
	goto	l11644
	line	484
;main.c: 484: case 5:
	
l6856:	
	line	485
	
l11626:	
;main.c: 485: if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u5261
	goto	u5260
u5261:
	goto	l11630
u5260:
	line	486
	
l11628:	
;main.c: 486: goRight();
	fcall	_goRight
	goto	l11644
	line	487
	
l6857:	
	line	488
	
l11630:	
;main.c: 487: else
;main.c: 488: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6858:	
	line	489
;main.c: 489: break;
	goto	l11644
	line	490
;main.c: 490: case 6:
	
l6859:	
	line	491
	
l11632:	
;main.c: 491: if (getOrientation() == WEST)
	fcall	_getOrientation
	iorlw	0
	skipz
	goto	u5271
	goto	u5270
u5271:
	goto	l11638
u5270:
	line	493
	
l11634:	
;main.c: 492: {
;main.c: 493: play_iCreate_song(6);
	movlw	(06h)
	fcall	_play_iCreate_song
	line	494
	
l11636:	
;main.c: 494: goForward();
	fcall	_goForward
	line	495
;main.c: 495: }
	goto	l11644
	line	496
	
l6860:	
	line	497
	
l11638:	
;main.c: 496: else
;main.c: 497: goToNextCell();
	fcall	_goToNextCell
	goto	l11644
	
l6861:	
	line	498
;main.c: 498: break;
	goto	l11644
	line	499
;main.c: 499: default:
	
l6862:	
	line	500
;main.c: 500: break;
	goto	l11644
	line	501
	
l11640:	
;main.c: 501: }
	goto	l11644
	line	428
	
l6823:	
	
l11642:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_node),w	;volatile
	; Switch size 1, requested type "space"
; Number of cases is 7, Range of values is 0 to 6
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           22    12 (average)
; direct_byte           29     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable            11     6 (fixed)
; spacedrange           20     9 (fixed)
; locatedrange           7     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11570
	xorlw	1^0	; case 1
	skipnz
	goto	l6826
	xorlw	2^1	; case 2
	skipnz
	goto	l6835
	xorlw	3^2	; case 3
	skipnz
	goto	l6844
	xorlw	4^3	; case 4
	skipnz
	goto	l11620
	xorlw	5^4	; case 5
	skipnz
	goto	l11626
	xorlw	6^5	; case 6
	skipnz
	goto	l11632
	goto	l11644
	opt asmopt_on

	line	501
	
l6825:	
	line	502
	
l11644:	
;main.c: 502: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	503
	
l11646:	
;main.c: 503: if(getSuccessfulDrive())
	fcall	_getSuccessfulDrive
	btfss	status,0
	goto	u5281
	goto	u5280
u5281:
	goto	l11666
u5280:
	line	506
	
l11648:	
;main.c: 504: {
;main.c: 506: if(xVictim == xCoord && yVictim == yCoord)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xVictim),w	;volatile
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u5291
	goto	u5290
u5291:
	goto	l11654
u5290:
	
l11650:	
	movf	(_yVictim),w	;volatile
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u5301
	goto	u5300
u5301:
	goto	l11654
u5300:
	line	508
	
l11652:	
;main.c: 507: {
;main.c: 508: victimIndicator = 1;
	clrf	(main@victimIndicator)
	bsf	status,0
	rlf	(main@victimIndicator),f
	goto	l11654
	line	509
	
l6864:	
	line	539
	
l11654:	
;main.c: 509: }
;main.c: 539: updateMapData(0,0,0,0,victimIndicator,getOrientation());
	clrf	(?_updateMapData)
	clrf	0+(?_updateMapData)+01h
	clrf	0+(?_updateMapData)+02h
	movf	(main@victimIndicator),w
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	0+(?_updateMapData)+03h
	fcall	_getOrientation
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+1)+0
	movf	(??_main+1)+0,w
	movwf	0+(?_updateMapData)+04h
	movlw	(0)
	fcall	_updateMapData
	line	541
	
l11656:	
;main.c: 541: victimIndicator = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(main@victimIndicator)
	line	543
	
l11658:	
;main.c: 543: updateLocation();
	fcall	_updateLocation
	line	544
	
l11660:	
;main.c: 544: updateNode();
	fcall	_updateNode
	line	545
	
l11662:	
;main.c: 545: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u5311
	goto	u5310
u5311:
	goto	l11666
u5310:
	line	546
	
l11664:	
;main.c: 546: checkIfHome();
	fcall	_checkIfHome
	goto	l11666
	
l6865:	
	goto	l11666
	line	547
	
l6863:	
	goto	l11666
	line	548
	
l6820:	
	goto	l11666
	line	549
	
l6811:	
	line	376
	
l11666:	
	btfss	(_home/8),(_home)&7
	goto	u5321
	goto	u5320
u5321:
	goto	l11498
u5320:
	goto	l6867
	
l6866:	
	line	551
	
l6867:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text1300,local,class=CODE,delta=2
global __ptext1300
__ptext1300:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 276 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_getSomethingInTheWay
;;		_goLeft
;;		_goForward
;;		_goRight
;;		_goBackward
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1300
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	276
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 1
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	277
	
l11472:	
;main.c: 277: if(!leftWall && (getSomethingInTheWay() != LEFT))
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u4931
	goto	u4930
u4931:
	goto	l6769
u4930:
	
l11474:	
	fcall	_getSomethingInTheWay
	xorlw	01h
	skipnz
	goto	u4941
	goto	u4940
u4941:
	goto	l6769
u4940:
	line	278
	
l11476:	
;main.c: 278: goLeft();
	fcall	_goLeft
	goto	l6775
	line	279
	
l6769:	
;main.c: 279: else if(!frontWall && (getSomethingInTheWay() != FORWARD))
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u4951
	goto	u4950
u4951:
	goto	l6771
u4950:
	
l11478:	
	fcall	_getSomethingInTheWay
	xorlw	0
	skipnz
	goto	u4961
	goto	u4960
u4961:
	goto	l6771
u4960:
	line	280
	
l11480:	
;main.c: 280: goForward();
	fcall	_goForward
	goto	l6775
	line	281
	
l6771:	
;main.c: 281: else if(!rightWall && (getSomethingInTheWay() != RIGHT))
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u4971
	goto	u4970
u4971:
	goto	l11486
u4970:
	
l11482:	
	fcall	_getSomethingInTheWay
	xorlw	03h
	skipnz
	goto	u4981
	goto	u4980
u4981:
	goto	l11486
u4980:
	line	282
	
l11484:	
;main.c: 282: goRight();
	fcall	_goRight
	goto	l6775
	line	283
	
l6773:	
	line	284
	
l11486:	
;main.c: 283: else
;main.c: 284: goBackward();
	fcall	_goBackward
	goto	l6775
	
l6774:	
	goto	l6775
	
l6772:	
	goto	l6775
	
l6770:	
	line	285
	
l6775:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1301,local,class=CODE,delta=2
global __ptext1301
__ptext1301:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 205 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_rotateIR
;;		_lcd_set_cursor
;;		_findWall
;;		_lcd_write_data
;;		_play_iCreate_song
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1301
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	205
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 1
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	206
	
l11444:	
;main.c: 206: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	207
;main.c: 207: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	209
	
l11446:	
;main.c: 209: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4841
	goto	u4840
	
u4841:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u4854
u4840:
	bcf	(_leftWall/8),(_leftWall)&7
u4854:
	line	210
	
l11448:	
;main.c: 210: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4861
	goto	u4860
u4861:
	goto	l11452
u4860:
	line	211
	
l11450:	
;main.c: 211: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	goto	l6761
	line	212
	
l6760:	
	line	213
	
l11452:	
;main.c: 212: else
;main.c: 213: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6761:	
	line	215
;main.c: 215: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	216
	
l11454:	
;main.c: 216: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4871
	goto	u4870
	
u4871:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u4884
u4870:
	bcf	(_frontWall/8),(_frontWall)&7
u4884:
	line	218
	
l11456:	
;main.c: 218: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4891
	goto	u4890
u4891:
	goto	l11460
u4890:
	line	219
	
l11458:	
;main.c: 219: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	goto	l6763
	line	220
	
l6762:	
	line	221
	
l11460:	
;main.c: 220: else
;main.c: 221: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6763:	
	line	223
;main.c: 223: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	224
	
l11462:	
;main.c: 224: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4901
	goto	u4900
	
u4901:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u4914
u4900:
	bcf	(_rightWall/8),(_rightWall)&7
u4914:
	line	226
	
l11464:	
;main.c: 226: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u4921
	goto	u4920
u4921:
	goto	l11470
u4920:
	line	228
	
l11466:	
;main.c: 227: {
;main.c: 228: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	229
	
l11468:	
;main.c: 229: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	230
;main.c: 230: }else
	goto	l6765
	
l6764:	
	line	231
	
l11470:	
;main.c: 231: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6765:	
	line	233
;main.c: 233: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	234
	
l6766:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text1302,local,class=CODE,delta=2
global __ptext1302
__ptext1302:

;; *************** function _goRight *****************
;; Defined at:
;;		line 253 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_turnRight90
;;		_updateOrientation
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1302
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	253
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 2
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	254
	
l11434:	
;drive.c: 254: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	255
;drive.c: 255: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	256
	
l11436:	
;drive.c: 256: turnRight90();
	fcall	_turnRight90
	line	257
	
l11438:	
;drive.c: 257: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	258
	
l11440:	
;drive.c: 258: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	259
	
l11442:	
;drive.c: 259: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	260
	
l5873:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1303,local,class=CODE,delta=2
global __ptext1303
__ptext1303:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 232 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_turnLeft90
;;		_updateOrientation
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1303
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	232
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 2
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	233
	
l11424:	
;drive.c: 233: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	234
;drive.c: 234: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	235
	
l11426:	
;drive.c: 235: turnLeft90();
	fcall	_turnLeft90
	line	236
	
l11428:	
;drive.c: 236: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	237
	
l11430:	
;drive.c: 237: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	238
	
l11432:	
;drive.c: 238: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	239
	
l5867:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1304,local,class=CODE,delta=2
global __ptext1304
__ptext1304:

;; *************** function _goForward *****************
;; Defined at:
;;		line 217 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_getCurrentX
;;		_getCurrentY
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1304
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	217
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 2
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	218
	
l11412:	
;drive.c: 218: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	219
;drive.c: 219: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	220
	
l11414:	
;drive.c: 220: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	221
	
l11416:	
;drive.c: 221: if( (getCurrentX() == 1 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	01h
	skipz
	goto	u4821
	goto	u4820
u4821:
	goto	l11422
u4820:
	
l11418:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u4831
	goto	u4830
u4831:
	goto	l11422
u4830:
	line	223
	
l11420:	
;drive.c: 222: {
;drive.c: 223: driveForDistance(800);
	movlw	low(0320h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(0320h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	224
;drive.c: 224: }else
	goto	l5864
	
l5862:	
	line	226
	
l11422:	
;drive.c: 225: {
;drive.c: 226: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	goto	l5864
	line	227
	
l5863:	
	line	228
	
l5864:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1305,local,class=CODE,delta=2
global __ptext1305
__ptext1305:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 206 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_turnAround
;;		_updateOrientation
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1305
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	206
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 1
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	207
	
l11402:	
;drive.c: 207: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	208
;drive.c: 208: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	209
	
l11404:	
;drive.c: 209: turnAround();
	fcall	_turnAround
	line	210
	
l11406:	
;drive.c: 210: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	211
	
l11408:	
;drive.c: 211: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	212
	
l11410:	
;drive.c: 212: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	213
	
l5859:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_wallFollow
psect	text1306,local,class=CODE,delta=2
global __ptext1306
__ptext1306:

;; *************** function _wallFollow *****************
;; Defined at:
;;		line 582 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  distanceToWa    2   38[BANK0 ] int 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       5       0       0       0
;;Total ram usage:        5 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_readIR
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1306
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	582
	global	__size_of_wallFollow
	__size_of_wallFollow	equ	__end_of_wallFollow-_wallFollow
	
_wallFollow:	
	opt	stack 2
; Regs used in _wallFollow: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	583
	
l11386:	
;main.c: 583: int distanceToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(wallFollow@distanceToWall+1)
	addwf	(wallFollow@distanceToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(wallFollow@distanceToWall)
	addwf	(wallFollow@distanceToWall)

	line	584
	
l11388:	
;main.c: 584: if((distanceToWall > 86) && (distanceToWall < 100))
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(057h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4795
	movlw	low(057h)
	subwf	(wallFollow@distanceToWall),w
u4795:

	skipc
	goto	u4791
	goto	u4790
u4791:
	goto	l11396
u4790:
	
l11390:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4805
	movlw	low(064h)
	subwf	(wallFollow@distanceToWall),w
u4805:

	skipnc
	goto	u4801
	goto	u4800
u4801:
	goto	l11396
u4800:
	line	586
	
l11392:	
;main.c: 585: {
;main.c: 586: drive(0, 50, 0, 1);
	movlw	(032h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_wallFollow+0)+0
	movf	(??_wallFollow+0)+0,w
	movwf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	bsf	status,0
	rlf	0+(?_drive)+02h,f
	movlw	(0)
	fcall	_drive
	line	587
;main.c: 587: waitFor(157,0,8);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(08h)
	movwf	(??_wallFollow+0)+0
	movf	(??_wallFollow+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	588
;main.c: 588: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	589
	
l11394:	
;main.c: 589: _delay((unsigned long)((1000)*(20000000/4000.0)));
	opt asmopt_off
movlw  26
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_wallFollow+0)+0+2),f
movlw	94
movwf	((??_wallFollow+0)+0+1),f
	movlw	134
movwf	((??_wallFollow+0)+0),f
u5337:
	decfsz	((??_wallFollow+0)+0),f
	goto	u5337
	decfsz	((??_wallFollow+0)+0+1),f
	goto	u5337
	decfsz	((??_wallFollow+0)+0+2),f
	goto	u5337
	clrwdt
opt asmopt_on

	line	590
;main.c: 590: }
	goto	l6887
	line	592
	
l6884:	
	
l11396:	
;main.c: 592: else if(distanceToWall < 36)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(024h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4815
	movlw	low(024h)
	subwf	(wallFollow@distanceToWall),w
u4815:

	skipnc
	goto	u4811
	goto	u4810
u4811:
	goto	l6887
u4810:
	line	594
	
l11398:	
;main.c: 593: {
;main.c: 594: drive(0, 50, 255, 255);
	movlw	(032h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_wallFollow+0)+0
	movf	(??_wallFollow+0)+0,w
	movwf	(?_drive)
	movlw	(0FFh)
	movwf	(??_wallFollow+1)+0
	movf	(??_wallFollow+1)+0,w
	movwf	0+(?_drive)+01h
	movlw	(0FFh)
	movwf	(??_wallFollow+2)+0
	movf	(??_wallFollow+2)+0,w
	movwf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	595
;main.c: 595: waitFor(157,255,0b11111000);
	movlw	(0FFh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_wallFollow+0)+0
	movf	(??_wallFollow+0)+0,w
	movwf	(?_waitFor)
	movlw	(0F8h)
	movwf	(??_wallFollow+1)+0
	movf	(??_wallFollow+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	596
;main.c: 596: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	597
	
l11400:	
;main.c: 597: _delay((unsigned long)((1000)*(20000000/4000.0)));
	opt asmopt_off
movlw  26
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_wallFollow+0)+0+2),f
movlw	94
movwf	((??_wallFollow+0)+0+1),f
	movlw	134
movwf	((??_wallFollow+0)+0),f
u5347:
	decfsz	((??_wallFollow+0)+0),f
	goto	u5347
	decfsz	((??_wallFollow+0)+0+1),f
	goto	u5347
	decfsz	((??_wallFollow+0)+0+2),f
	goto	u5347
	clrwdt
opt asmopt_on

	goto	l6887
	line	598
	
l6886:	
	goto	l6887
	line	599
	
l6885:	
	
l6887:	
	return
	opt stack 0
GLOBAL	__end_of_wallFollow
	__end_of_wallFollow:
;; =============== function _wallFollow ends ============

	signat	_wallFollow,88
	global	_findWall
psect	text1307,local,class=CODE,delta=2
global __ptext1307
__ptext1307:

;; *************** function _findWall *****************
;; Defined at:
;;		line 557 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_readIR
;; This function is called by:
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text1307
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	557
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 1
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	558
	
l11374:	
;main.c: 558: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4785
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u4785:

	skipc
	goto	u4781
	goto	u4780
u4781:
	goto	l11382
u4780:
	line	559
	
l11376:	
;main.c: 559: return 0;
	clrc
	
	goto	l6871
	
l11378:	
	goto	l6871
	
l11380:	
	goto	l6871
	line	560
	
l6870:	
	line	561
	
l11382:	
;main.c: 560: else
;main.c: 561: return 1;
	setc
	
	goto	l6871
	
l11384:	
	goto	l6871
	
l6872:	
	line	562
	
l6871:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text1308,local,class=CODE,delta=2
global __ptext1308
__ptext1308:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 326 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  distToWall      2   37[BANK0 ] int 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_readIR
;;		_drive
;;		_clearSuccessfulDrive
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1308
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	326
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 2
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	328
	
l11348:	
;drive.c: 328: int distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	line	329
	
l11350:	
;drive.c: 329: if(distToWall < 45)
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4745
	movlw	low(02Dh)
	subwf	(frontWallCorrect@distToWall),w
u4745:

	skipnc
	goto	u4741
	goto	u4740
u4741:
	goto	l11362
u4740:
	line	331
	
l11352:	
;drive.c: 330: {
;drive.c: 331: drive(255, 125, 128, 0);
	movlw	(07Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_drive)
	movlw	(080h)
	movwf	(??_frontWallCorrect+1)+0
	movf	(??_frontWallCorrect+1)+0,w
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0FFh)
	fcall	_drive
	line	332
;drive.c: 332: while(distToWall < 51)
	goto	l11356
	
l5896:	
	line	333
	
l11354:	
;drive.c: 333: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l11356
	
l5895:	
	line	332
	
l11356:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(033h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4755
	movlw	low(033h)
	subwf	(frontWallCorrect@distToWall),w
u4755:

	skipc
	goto	u4751
	goto	u4750
u4751:
	goto	l11354
u4750:
	goto	l11358
	
l5897:	
	line	334
	
l11358:	
;drive.c: 334: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	335
	
l11360:	
;drive.c: 335: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	336
;drive.c: 336: }
	goto	l5903
	line	337
	
l5894:	
	
l11362:	
;drive.c: 337: else if(distToWall > 55)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4765
	movlw	low(038h)
	subwf	(frontWallCorrect@distToWall),w
u4765:

	skipc
	goto	u4761
	goto	u4760
u4761:
	goto	l5903
u4760:
	line	339
	
l11364:	
;drive.c: 338: {
;drive.c: 339: drive(0, 250, 128, 0);
	movlw	(0FAh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_drive)
	movlw	(080h)
	movwf	(??_frontWallCorrect+1)+0
	movf	(??_frontWallCorrect+1)+0,w
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	340
;drive.c: 340: while(distToWall > 49)
	goto	l11368
	
l5901:	
	line	341
	
l11366:	
;drive.c: 341: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l11368
	
l5900:	
	line	340
	
l11368:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(032h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4775
	movlw	low(032h)
	subwf	(frontWallCorrect@distToWall),w
u4775:

	skipnc
	goto	u4771
	goto	u4770
u4771:
	goto	l11366
u4770:
	goto	l11370
	
l5902:	
	line	342
	
l11370:	
;drive.c: 342: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	343
	
l11372:	
;drive.c: 343: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	goto	l5903
	line	344
	
l5899:	
	goto	l5903
	line	346
	
l5898:	
	
l5903:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_driveForDistance
psect	text1309,local,class=CODE,delta=2
global __ptext1309
__ptext1309:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 32 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;  moveDistance    2   12[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  distance        2   18[BANK0 ] int 
;;  deltaDistanc    2   16[BANK0 ] int 
;;  cliff           1   23[BANK0 ] volatile unsigned char 
;;  virtualWall     1   22[BANK0 ] volatile unsigned char 
;;  low             1   21[BANK0 ] volatile unsigned char 
;;  high            1   20[BANK0 ] volatile unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       8       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0      12       0       0       0
;;Total ram usage:       12 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_drive
;;		_ser_putch
;;		_ser_getch
;;		_goReverse
;;		_clearSuccessfulDrive
;;		_turnRight90
;;		_updateOrientation
;;		_turnLeft90
;;		_getCurrentY
;;		_getCurrentX
;;		_findFinalDestination
;;		_setVirtualLocation
;; This function is called by:
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1309
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	32
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 2
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	35
	
l11258:	
;drive.c: 34: volatile char high, low, cliff, virtualWall;
;drive.c: 35: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	36
;drive.c: 36: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	38
	
l11260:	
;drive.c: 38: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	39
	
l11262:	
;drive.c: 39: drive(0, 250, 128, 0);
	movlw	(0FAh)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(?_drive)
	movlw	(080h)
	movwf	(??_driveForDistance+1)+0
	movf	(??_driveForDistance+1)+0,w
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	40
	
l11264:	
;drive.c: 40: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	42
;drive.c: 42: while(moving)
	goto	l11346
	
l5824:	
	line	44
	
l11266:	
;drive.c: 43: {
;drive.c: 44: if(distance >= 100)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4625
	movlw	low(064h)
	subwf	(driveForDistance@distance),w
u4625:

	skipc
	goto	u4621
	goto	u4620
u4621:
	goto	l11304
u4620:
	line	47
	
l11268:	
;drive.c: 45: {
;drive.c: 47: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	48
;drive.c: 48: ser_putch(10);
	movlw	(0Ah)
	fcall	_ser_putch
	line	49
;drive.c: 49: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	50
	
l11270:	
;drive.c: 50: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u4631
	goto	u4630
u4631:
	goto	l11282
u4630:
	line	52
	
l11272:	
;drive.c: 51: {
;drive.c: 52: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	53
;drive.c: 53: ser_putch(11);
	movlw	(0Bh)
	fcall	_ser_putch
	line	54
;drive.c: 54: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	55
	
l11274:	
;drive.c: 55: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u4641
	goto	u4640
u4641:
	goto	l11282
u4640:
	line	57
	
l11276:	
;drive.c: 56: {
;drive.c: 57: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	58
;drive.c: 58: ser_putch(9);
	movlw	(09h)
	fcall	_ser_putch
	line	59
;drive.c: 59: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	60
	
l11278:	
;drive.c: 60: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u4651
	goto	u4650
u4651:
	goto	l11282
u4650:
	line	62
	
l11280:	
;drive.c: 61: {
;drive.c: 62: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	63
;drive.c: 63: ser_putch(12);
	movlw	(0Ch)
	fcall	_ser_putch
	line	64
;drive.c: 64: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	goto	l11282
	line	65
	
l5828:	
	goto	l11282
	line	66
	
l5827:	
	goto	l11282
	line	67
	
l5826:	
	line	68
	
l11282:	
;drive.c: 65: }
;drive.c: 66: }
;drive.c: 67: }
;drive.c: 68: if(cliff != 0)
	movf	(driveForDistance@cliff),w	;volatile
	skipz
	goto	u4660
	goto	l11304
u4660:
	line	70
	
l11284:	
;drive.c: 69: {
;drive.c: 70: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	71
;drive.c: 71: goReverse();
	fcall	_goReverse
	line	72
	
l11286:	
;drive.c: 72: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	74
	
l11288:	
;drive.c: 74: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u4671
	goto	u4670
u4671:
	goto	l11296
u4670:
	line	76
	
l11290:	
;drive.c: 75: {
;drive.c: 76: somethingInTheWay = LEFT;
	clrf	(_somethingInTheWay)	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay),f	;volatile
	line	77
	
l11292:	
;drive.c: 77: turnRight90();
	fcall	_turnRight90
	line	78
	
l11294:	
;drive.c: 78: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	79
;drive.c: 79: }
	goto	l5831
	line	80
	
l5830:	
	
l11296:	
;drive.c: 80: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u4681
	goto	u4680
u4681:
	goto	l5832
u4680:
	line	82
	
l11298:	
;drive.c: 81: {
;drive.c: 82: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	line	83
	
l11300:	
;drive.c: 83: turnLeft90();
	fcall	_turnLeft90
	line	84
	
l11302:	
;drive.c: 84: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	85
;drive.c: 85: }
	goto	l5831
	line	86
	
l5832:	
	line	87
;drive.c: 86: else
;drive.c: 87: somethingInTheWay = FORWARD;
	clrf	(_somethingInTheWay)	;volatile
	goto	l5831
	
l5833:	
	
l5831:	
	line	88
;drive.c: 88: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11304
	line	89
	
l5829:	
	goto	l11304
	line	90
	
l5825:	
	line	93
	
l11304:	
;drive.c: 89: }
;drive.c: 90: }
;drive.c: 93: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	94
;drive.c: 94: ser_putch(13);
	movlw	(0Dh)
	fcall	_ser_putch
	line	95
;drive.c: 95: virtualWall = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@virtualWall)	;volatile
	line	96
	
l11306:	
;drive.c: 96: if(virtualWall == 1)
	movf	(driveForDistance@virtualWall),w	;volatile
	xorlw	01h
	skipz
	goto	u4691
	goto	u4690
u4691:
	goto	l11330
u4690:
	line	98
	
l11308:	
;drive.c: 97: {
;drive.c: 98: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	99
;drive.c: 99: findFinalDestination(getCurrentX(),getCurrentY(), currentOrientation);
	fcall	_getCurrentY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(?_findFinalDestination)
	movf	(_currentOrientation),w	;volatile
	movwf	(??_driveForDistance+1)+0
	movf	(??_driveForDistance+1)+0,w
	movwf	0+(?_findFinalDestination)+01h
	fcall	_getCurrentX
	fcall	_findFinalDestination
	line	100
	
l11310:	
;drive.c: 100: setVirtualLocation(getCurrentX(), getCurrentY(), currentOrientation);
	fcall	_getCurrentY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(?_setVirtualLocation)
	movf	(_currentOrientation),w	;volatile
	movwf	(??_driveForDistance+1)+0
	movf	(??_driveForDistance+1)+0,w
	movwf	0+(?_setVirtualLocation)+01h
	fcall	_getCurrentX
	fcall	_setVirtualLocation
	line	101
	
l11312:	
;drive.c: 101: goReverse();
	fcall	_goReverse
	line	102
	
l11314:	
;drive.c: 102: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	104
;drive.c: 104: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u4701
	goto	u4700
u4701:
	goto	l11322
u4700:
	line	106
	
l11316:	
;drive.c: 105: {
;drive.c: 106: somethingInTheWay = LEFT;
	clrf	(_somethingInTheWay)	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay),f	;volatile
	line	107
	
l11318:	
;drive.c: 107: turnRight90();
	fcall	_turnRight90
	line	108
	
l11320:	
;drive.c: 108: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	109
;drive.c: 109: }
	goto	l5836
	line	110
	
l5835:	
	
l11322:	
;drive.c: 110: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u4711
	goto	u4710
u4711:
	goto	l5837
u4710:
	line	112
	
l11324:	
;drive.c: 111: {
;drive.c: 112: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	line	113
	
l11326:	
;drive.c: 113: turnLeft90();
	fcall	_turnLeft90
	line	114
	
l11328:	
;drive.c: 114: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	115
;drive.c: 115: }
	goto	l5836
	line	116
	
l5837:	
	line	117
;drive.c: 116: else
;drive.c: 117: somethingInTheWay = FORWARD;
	clrf	(_somethingInTheWay)	;volatile
	goto	l5836
	
l5838:	
	
l5836:	
	line	118
;drive.c: 118: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11330
	line	119
	
l5834:	
	line	163
	
l11330:	
;drive.c: 119: }
;drive.c: 163: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	164
;drive.c: 164: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	165
;drive.c: 165: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	166
;drive.c: 166: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	167
	
l11332:	
;drive.c: 167: deltaDistance = high*256 + low;
	movf	(driveForDistance@high),w	;volatile
	movwf	(??_driveForDistance+0)+0
	clrf	(??_driveForDistance+0)+0+1
	movf	(??_driveForDistance+0)+0,w
	movwf	(??_driveForDistance+0)+1
	clrf	(??_driveForDistance+0)+0
	movf	(driveForDistance@low),w	;volatile
	addwf	0+(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@deltaDistance)
	movlw	0
	skipnc
	movlw	1
	addwf	1+(??_driveForDistance+0)+0,w
	movwf	1+(driveForDistance@deltaDistance)
	line	168
	
l11334:	
;drive.c: 168: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	169
	
l11336:	
;drive.c: 169: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u4725
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u4725:

	skipc
	goto	u4721
	goto	u4720
u4721:
	goto	l11346
u4720:
	line	171
	
l11338:	
;drive.c: 170: {
;drive.c: 171: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	172
	
l11340:	
;drive.c: 172: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	line	173
	
l11342:	
;drive.c: 173: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	174
	
l11344:	
;drive.c: 174: somethingInTheWay = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	goto	l11346
	line	175
	
l5839:	
	goto	l11346
	line	176
	
l5823:	
	line	42
	
l11346:	
	btfsc	(_moving/8),(_moving)&7
	goto	u4731
	goto	u4730
u4731:
	goto	l11266
u4730:
	goto	l5841
	
l5840:	
	line	177
	
l5841:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_updateLocation
psect	text1310,local,class=CODE,delta=2
global __ptext1310
__ptext1310:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 288 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_getOrientation
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1310
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	288
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 4
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	289
	
l11214:	
;main.c: 289: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	290
;main.c: 290: switch(getOrientation())
	goto	l11234
	line	292
;main.c: 291: {
;main.c: 292: case NORTH:
	
l6779:	
	line	293
	
l11216:	
;main.c: 293: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	294
	
l11218:	
;main.c: 294: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	295
;main.c: 295: break;
	goto	l6780
	line	296
;main.c: 296: case SOUTH:
	
l6781:	
	line	297
	
l11220:	
;main.c: 297: --yCoord;
	movlw	(-1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	298
	
l11222:	
;main.c: 298: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	299
;main.c: 299: break;
	goto	l6780
	line	300
;main.c: 300: case EAST:
	
l6782:	
	line	301
	
l11224:	
;main.c: 301: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	302
	
l11226:	
;main.c: 302: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	303
;main.c: 303: break;
	goto	l6780
	line	304
;main.c: 304: case WEST:
	
l6783:	
	line	305
	
l11228:	
;main.c: 305: --xCoord;
	movlw	(-1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	306
	
l11230:	
;main.c: 306: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	307
;main.c: 307: break;
	goto	l6780
	line	308
;main.c: 308: default:
	
l6784:	
	line	309
;main.c: 309: break;
	goto	l6780
	line	310
	
l11232:	
;main.c: 310: }
	goto	l6780
	line	290
	
l6778:	
	
l11234:	
	fcall	_getOrientation
	; Switch size 1, requested type "space"
; Number of cases is 4, Range of values is 0 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           13     7 (average)
; direct_byte           20     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             8     6 (fixed)
; spacedrange           14     9 (fixed)
; locatedrange           4     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11228
	xorlw	1^0	; case 1
	skipnz
	goto	l11220
	xorlw	2^1	; case 2
	skipnz
	goto	l11224
	xorlw	3^2	; case 3
	skipnz
	goto	l11216
	goto	l6780
	opt asmopt_on

	line	310
	
l6780:	
	line	312
;main.c: 312: if(xCoord < 0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(_xCoord),7	;volatile
	goto	u4581
	goto	u4580
u4581:
	goto	l11238
u4580:
	line	314
	
l11236:	
;main.c: 313: {
;main.c: 314: xCoord = 0;
	clrf	(_xCoord)	;volatile
	goto	l11238
	line	315
	
l6785:	
	line	316
	
l11238:	
;main.c: 315: }
;main.c: 316: if(xCoord > 4)
	movf	(_xCoord),w	;volatile
	xorlw	80h
	addlw	-((05h)^80h)
	skipc
	goto	u4591
	goto	u4590
u4591:
	goto	l11242
u4590:
	line	318
	
l11240:	
;main.c: 317: {
;main.c: 318: xCoord = 4;
	movlw	(04h)
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	movwf	(_xCoord)	;volatile
	goto	l11242
	line	319
	
l6786:	
	line	321
	
l11242:	
;main.c: 319: }
;main.c: 321: if(yCoord < 0)
	btfss	(_yCoord),7	;volatile
	goto	u4601
	goto	u4600
u4601:
	goto	l11246
u4600:
	line	323
	
l11244:	
;main.c: 322: {
;main.c: 323: yCoord = 0;
	clrf	(_yCoord)	;volatile
	goto	l11246
	line	324
	
l6787:	
	line	325
	
l11246:	
;main.c: 324: }
;main.c: 325: if(yCoord > 3)
	movf	(_yCoord),w	;volatile
	xorlw	80h
	addlw	-((04h)^80h)
	skipc
	goto	u4611
	goto	u4610
u4611:
	goto	l11250
u4610:
	line	327
	
l11248:	
;main.c: 326: {
;main.c: 327: yCoord = 3;
	movlw	(03h)
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	movwf	(_yCoord)	;volatile
	goto	l11250
	line	328
	
l6788:	
	line	330
	
l11250:	
;main.c: 328: }
;main.c: 330: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	331
	
l11252:	
;main.c: 331: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	332
	
l11254:	
;main.c: 332: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	333
	
l11256:	
;main.c: 333: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	334
	
l6789:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text1311,local,class=CODE,delta=2
global __ptext1311
__ptext1311:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 174 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  victim          1    7[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_putch
;;		_ser_getch
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_getVictimZone
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1311
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	174
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 4
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	175
	
l11182:	
;main.c: 175: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	176
;main.c: 176: ser_putch(17);
	movlw	(011h)
	fcall	_ser_putch
	line	177
;main.c: 177: char victim = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(lookForVictim@victim)
	line	179
	
l11184:	
;main.c: 179: if(victim > 241 && victim != 255)
	movlw	(0F2h)
	subwf	(lookForVictim@victim),w
	skipc
	goto	u4551
	goto	u4550
u4551:
	goto	l6757
u4550:
	
l11186:	
	movf	(lookForVictim@victim),w
	xorlw	0FFh
	skipnz
	goto	u4561
	goto	u4560
u4561:
	goto	l6757
u4560:
	line	181
	
l11188:	
;main.c: 180: {
;main.c: 181: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4571
	goto	u4570
u4571:
	goto	l11206
u4570:
	line	183
	
l11190:	
;main.c: 182: {
;main.c: 183: _delay((unsigned long)((1000)*(20000000/4000.0)));
	opt asmopt_off
movlw  26
movwf	((??_lookForVictim+0)+0+2),f
movlw	94
movwf	((??_lookForVictim+0)+0+1),f
	movlw	134
movwf	((??_lookForVictim+0)+0),f
u5357:
	decfsz	((??_lookForVictim+0)+0),f
	goto	u5357
	decfsz	((??_lookForVictim+0)+0+1),f
	goto	u5357
	decfsz	((??_lookForVictim+0)+0+2),f
	goto	u5357
	clrwdt
opt asmopt_on

	line	184
	
l11192:	
;main.c: 184: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	185
	
l11194:	
;main.c: 185: _delay((unsigned long)((500)*(20000000/4000.0)));
	opt asmopt_off
movlw  13
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_lookForVictim+0)+0+2),f
movlw	175
movwf	((??_lookForVictim+0)+0+1),f
	movlw	193
movwf	((??_lookForVictim+0)+0),f
u5367:
	decfsz	((??_lookForVictim+0)+0),f
	goto	u5367
	decfsz	((??_lookForVictim+0)+0+1),f
	goto	u5367
	decfsz	((??_lookForVictim+0)+0+2),f
	goto	u5367
	clrwdt
opt asmopt_on

	line	186
	
l11196:	
;main.c: 186: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	187
	
l11198:	
;main.c: 187: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	188
	
l11200:	
;main.c: 188: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	189
	
l11202:	
;main.c: 189: xVictim = xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(_xVictim)	;volatile
	line	190
	
l11204:	
;main.c: 190: yVictim = yCoord;
	movf	(_yCoord),w	;volatile
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(_yVictim)	;volatile
	line	191
;main.c: 191: }
	goto	l6757
	line	192
	
l6755:	
	line	194
	
l11206:	
;main.c: 192: else
;main.c: 193: {
;main.c: 194: xVictim = xCoord;
	movf	(_xCoord),w	;volatile
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(_xVictim)	;volatile
	line	195
;main.c: 195: yVictim = yCoord;
	movf	(_yCoord),w	;volatile
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(_yVictim)	;volatile
	line	196
	
l11208:	
;main.c: 196: victimZone = getVictimZone(xCoord, yCoord);
	movf	(_yCoord),w	;volatile
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(?_getVictimZone)
	movf	(_xCoord),w	;volatile
	fcall	_getVictimZone
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lookForVictim+1)+0
	movf	(??_lookForVictim+1)+0,w
	movwf	(_victimZone)	;volatile
	line	197
	
l11210:	
;main.c: 197: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	198
	
l11212:	
;main.c: 198: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l6757
	line	199
	
l6756:	
	goto	l6757
	line	200
	
l6754:	
	line	201
	
l6757:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text1312,local,class=CODE,delta=2
global __ptext1312
__ptext1312:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 163 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_getFinalX
;;		_getFinalY
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1312
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	163
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 4
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	164
	
l11170:	
;main.c: 164: if((xCoord == getFinalX()) && (yCoord == getFinalY()))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	movwf	(??_checkForFinalDestination+0)+0
	clrf	(??_checkForFinalDestination+0)+0+1
	btfsc	(??_checkForFinalDestination+0)+0,7
	decf	(??_checkForFinalDestination+0)+0+1,f
	fcall	_getFinalX
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	0+(??_checkForFinalDestination+0)+0,w
	iorwf	1+(??_checkForFinalDestination+0)+0,w
	skipz
	goto	u4531
	goto	u4530
u4531:
	goto	l6751
u4530:
	
l11172:	
	movf	(_yCoord),w	;volatile
	movwf	(??_checkForFinalDestination+0)+0
	clrf	(??_checkForFinalDestination+0)+0+1
	btfsc	(??_checkForFinalDestination+0)+0,7
	decf	(??_checkForFinalDestination+0)+0+1,f
	fcall	_getFinalY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	0+(??_checkForFinalDestination+0)+0,w
	iorwf	1+(??_checkForFinalDestination+0)+0,w
	skipz
	goto	u4541
	goto	u4540
u4541:
	goto	l6751
u4540:
	line	166
	
l11174:	
;main.c: 165: {
;main.c: 166: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	167
	
l11176:	
;main.c: 167: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	168
	
l11178:	
;main.c: 168: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	169
	
l11180:	
;main.c: 169: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l6751
	line	170
	
l6750:	
	line	171
	
l6751:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text1313,local,class=CODE,delta=2
global __ptext1313
__ptext1313:

;; *************** function _init *****************
;; Defined at:
;;		line 124 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_init_adc
;;		_lcd_init
;;		_ser_init
;;		_initIRobot
;;		_initSongs
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1313
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	124
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 3
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	125
	
l11136:	
;main.c: 125: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	126
	
l11138:	
;main.c: 126: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	127
	
l11140:	
;main.c: 127: eepromSerial.pressed = 0;
	clrf	(_eepromSerial)
	line	128
;main.c: 128: eepromSerial.released = 1;
	clrf	0+(_eepromSerial)+01h
	bsf	status,0
	rlf	0+(_eepromSerial)+01h,f
	line	130
	
l11142:	
;main.c: 130: init_adc();
	fcall	_init_adc
	line	131
	
l11144:	
;main.c: 131: lcd_init();
	fcall	_lcd_init
	line	133
	
l11146:	
;main.c: 133: TRISB = 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	136
	
l11148:	
;main.c: 136: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	138
	
l11150:	
;main.c: 138: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	139
	
l11152:	
;main.c: 139: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	140
	
l11154:	
;main.c: 140: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	141
	
l11156:	
;main.c: 141: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	142
	
l11158:	
;main.c: 142: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	145
	
l11160:	
;main.c: 145: PEIE = 1;
	bsf	(94/8),(94)&7
	line	146
	
l11162:	
;main.c: 146: GIE = 1;
	bsf	(95/8),(95)&7
	line	148
	
l11164:	
;main.c: 148: ser_init();
	fcall	_ser_init
	line	149
	
l11166:	
;main.c: 149: initIRobot();
	fcall	_initIRobot
	line	150
	
l11168:	
;main.c: 150: initSongs();
	fcall	_initSongs
	line	151
	
l6744:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_goReverse
psect	text1314,local,class=CODE,delta=2
global __ptext1314
__ptext1314:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 242 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1314
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	242
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 2
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	243
	
l11126:	
;drive.c: 243: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	244
;drive.c: 244: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	245
	
l11128:	
;drive.c: 245: drive(255, 125, 128, 0);
	movlw	(07Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goReverse+0)+0
	movf	(??_goReverse+0)+0,w
	movwf	(?_drive)
	movlw	(080h)
	movwf	(??_goReverse+1)+0
	movf	(??_goReverse+1)+0,w
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0FFh)
	fcall	_drive
	line	246
	
l11130:	
;drive.c: 246: waitFor(156,254,212);
	movlw	(0FEh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goReverse+0)+0
	movf	(??_goReverse+0)+0,w
	movwf	(?_waitFor)
	movlw	(0D4h)
	movwf	(??_goReverse+1)+0
	movf	(??_goReverse+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Ch)
	fcall	_waitFor
	line	247
	
l11132:	
;drive.c: 247: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	248
	
l11134:	
;drive.c: 248: _delay((unsigned long)((2000)*(20000000/4000.0)));
	opt asmopt_off
movlw  51
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_goReverse+0)+0+2),f
movlw	188
movwf	((??_goReverse+0)+0+1),f
	movlw	16
movwf	((??_goReverse+0)+0),f
u5377:
	decfsz	((??_goReverse+0)+0),f
	goto	u5377
	decfsz	((??_goReverse+0)+0+1),f
	goto	u5377
	decfsz	((??_goReverse+0)+0+2),f
	goto	u5377
opt asmopt_on

	line	249
	
l5870:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_readIR
psect	text1315,local,class=CODE,delta=2
global __ptext1315
__ptext1315:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  cm              2   33[BANK0 ] int 
;; Return value:  Size  Location     Type
;;                  2   31[BANK0 ] int 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_adc_read_channel
;;		_convert
;; This function is called by:
;;		_frontWallCorrect
;;		_findWall
;;		_wallFollow
;;		_leftAngleCorrect
;; This function uses a non-reentrant model
;;
psect	text1315
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 2
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l11120:	
;ir.c: 34: int cm = convert(adc_read_channel(0));
	movlw	(0)
	fcall	_adc_read_channel
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_adc_read_channel)),w
	clrf	(?_convert+1)
	addwf	(?_convert+1)
	movf	(0+(?_adc_read_channel)),w
	clrf	(?_convert)
	addwf	(?_convert)

	fcall	_convert
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_convert)),w
	clrf	(readIR@cm+1)
	addwf	(readIR@cm+1)
	movf	(0+(?_convert)),w
	clrf	(readIR@cm)
	addwf	(readIR@cm)

	line	35
	
l11122:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l5083
	
l11124:	
	line	36
	
l5083:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_findFinalDestination
psect	text1316,local,class=CODE,delta=2
global __ptext1316
__ptext1316:

;; *************** function _findFinalDestination *****************
;; Defined at:
;;		line 12 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
;; Parameters:    Size  Location     Type
;;  virtualWallX    1    wreg     unsigned char 
;;  virtualWallY    1    4[BANK0 ] unsigned char 
;;  robotOrienta    1    5[BANK0 ] enum E1088
;; Auto vars:     Size  Location     Type
;;  virtualWallX    1    7[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1316
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	12
	global	__size_of_findFinalDestination
	__size_of_findFinalDestination	equ	__end_of_findFinalDestination-_findFinalDestination
	
_findFinalDestination:	
	opt	stack 2
; Regs used in _findFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;findFinalDestination@virtualWallX stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(findFinalDestination@virtualWallX)
	line	13
	
l11040:	
;map.c: 13: switch (virtualWallX)
	goto	l11116
	line	15
;map.c: 14: {
;map.c: 15: case 0:
	
l2849:	
	line	16
;map.c: 16: switch (virtualWallY)
	goto	l11050
	line	20
;map.c: 17: {
;map.c: 20: case 1:
	
l2851:	
	line	21
;map.c: 21: finalX = 0;
	clrf	(_finalX)
	line	22
	
l11042:	
;map.c: 22: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	23
;map.c: 23: break;
	goto	l11118
	line	24
;map.c: 24: case 2:
	
l2853:	
	line	25
;map.c: 25: finalX = 0;
	clrf	(_finalX)
	line	26
	
l11044:	
;map.c: 26: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	27
;map.c: 27: break;
	goto	l11118
	line	28
;map.c: 28: case 3:
	
l2854:	
	line	29
;map.c: 29: finalX = 0;
	clrf	(_finalX)
	line	30
	
l11046:	
;map.c: 30: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	31
;map.c: 31: break;
	goto	l11118
	line	32
;map.c: 32: default:
	
l2855:	
	line	33
;map.c: 33: break;
	goto	l11118
	line	34
	
l11048:	
;map.c: 34: }
	goto	l11118
	line	16
	
l2850:	
	
l11050:	
	movf	(findFinalDestination@virtualWallY),w
	; Switch size 1, requested type "space"
; Number of cases is 3, Range of values is 1 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           10     6 (average)
; direct_byte           20    11 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l2851
	xorlw	2^1	; case 2
	skipnz
	goto	l2853
	xorlw	3^2	; case 3
	skipnz
	goto	l2854
	goto	l11118
	opt asmopt_on

	line	34
	
l2852:	
	line	35
;map.c: 35: break;
	goto	l11118
	line	37
;map.c: 37: case 1:
	
l2857:	
	line	38
;map.c: 38: switch (virtualWallY)
	goto	l11068
	line	40
;map.c: 39: {
;map.c: 40: case 0:
	
l2859:	
	line	41
	
l11052:	
;map.c: 41: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	42
	
l11054:	
;map.c: 42: finalY = 0;
	clrf	(_finalY)
	line	43
;map.c: 43: break;
	goto	l11118
	line	44
;map.c: 44: case 1:
	
l2861:	
	line	45
	
l11056:	
;map.c: 45: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	46
;map.c: 46: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	47
;map.c: 47: break;
	goto	l11118
	line	48
;map.c: 48: case 2:
	
l2862:	
	line	49
	
l11058:	
;map.c: 49: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	50
	
l11060:	
;map.c: 50: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	51
;map.c: 51: break;
	goto	l11118
	line	52
;map.c: 52: case 3:
	
l2863:	
	line	53
	
l11062:	
;map.c: 53: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	54
	
l11064:	
;map.c: 54: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	55
;map.c: 55: break;
	goto	l11118
	line	56
;map.c: 56: default:
	
l2864:	
	line	57
;map.c: 57: break;
	goto	l11118
	line	58
	
l11066:	
;map.c: 58: }
	goto	l11118
	line	38
	
l2858:	
	
l11068:	
	movf	(findFinalDestination@virtualWallY),w
	; Switch size 1, requested type "space"
; Number of cases is 4, Range of values is 0 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           13     7 (average)
; direct_byte           20     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             8     6 (fixed)
; spacedrange           14     9 (fixed)
; locatedrange           4     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11052
	xorlw	1^0	; case 1
	skipnz
	goto	l11056
	xorlw	2^1	; case 2
	skipnz
	goto	l11058
	xorlw	3^2	; case 3
	skipnz
	goto	l11062
	goto	l11118
	opt asmopt_on

	line	58
	
l2860:	
	line	59
;map.c: 59: break;
	goto	l11118
	line	61
;map.c: 61: case 2:
	
l2865:	
	line	62
;map.c: 62: switch (virtualWallY)
	goto	l11086
	line	64
;map.c: 63: {
;map.c: 64: case 0:
	
l2867:	
	line	65
	
l11070:	
;map.c: 65: if(robotOrientation == WEST)
	movf	(findFinalDestination@robotOrientation),f
	skipz
	goto	u4501
	goto	u4500
u4501:
	goto	l11118
u4500:
	line	67
	
l11072:	
;map.c: 66: {
;map.c: 67: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	68
	
l11074:	
;map.c: 68: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	goto	l11118
	line	69
	
l2868:	
	line	70
;map.c: 69: }
;map.c: 70: break;
	goto	l11118
	line	71
;map.c: 71: case 1:
	
l2870:	
	line	72
	
l11076:	
;map.c: 72: finalX = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	73
	
l11078:	
;map.c: 73: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	74
;map.c: 74: break;
	goto	l11118
	line	75
;map.c: 75: case 2:
	
l2871:	
	line	76
	
l11080:	
;map.c: 76: if(robotOrientation == EAST)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	02h
	skipz
	goto	u4511
	goto	u4510
u4511:
	goto	l11118
u4510:
	line	78
	
l11082:	
;map.c: 77: {
;map.c: 78: finalX = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	79
;map.c: 79: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	goto	l11118
	line	80
	
l2872:	
	line	81
;map.c: 80: }
;map.c: 81: break;
	goto	l11118
	line	84
;map.c: 84: default:
	
l2873:	
	line	85
;map.c: 85: break;
	goto	l11118
	line	86
	
l11084:	
;map.c: 86: }
	goto	l11118
	line	62
	
l2866:	
	
l11086:	
	movf	(findFinalDestination@virtualWallY),w
	; Switch size 1, requested type "space"
; Number of cases is 3, Range of values is 0 to 2
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           10     6 (average)
; direct_byte           17     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             7     6 (fixed)
; spacedrange           12     9 (fixed)
; locatedrange           3     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11070
	xorlw	1^0	; case 1
	skipnz
	goto	l11076
	xorlw	2^1	; case 2
	skipnz
	goto	l11080
	goto	l11118
	opt asmopt_on

	line	86
	
l2869:	
	line	87
;map.c: 87: break;
	goto	l11118
	line	89
;map.c: 89: case 3:
	
l2874:	
	line	90
;map.c: 90: switch (virtualWallY)
	goto	l11096
	line	92
;map.c: 91: {
;map.c: 92: case 0:
	
l2876:	
	line	93
	
l11088:	
;map.c: 93: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	94
	
l11090:	
;map.c: 94: finalY = 0;
	clrf	(_finalY)
	line	95
;map.c: 95: break;
	goto	l11118
	line	98
;map.c: 98: case 2:
	
l2878:	
	line	99
	
l11092:	
;map.c: 99: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	100
;map.c: 100: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	101
;map.c: 101: break;
	goto	l11118
	line	104
;map.c: 104: default:
	
l2879:	
	line	105
;map.c: 105: break;
	goto	l11118
	line	106
	
l11094:	
;map.c: 106: }
	goto	l11118
	line	90
	
l2875:	
	
l11096:	
	movf	(findFinalDestination@virtualWallY),w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 0 to 2
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           17     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             7     6 (fixed)
; spacedrange           12     9 (fixed)
; locatedrange           3     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11088
	xorlw	2^0	; case 2
	skipnz
	goto	l11092
	goto	l11118
	opt asmopt_on

	line	106
	
l2877:	
	line	107
;map.c: 107: break;
	goto	l11118
	line	109
;map.c: 109: case 4:
	
l2880:	
	line	110
;map.c: 110: switch (virtualWallY)
	goto	l11112
	line	112
;map.c: 111: {
;map.c: 112: case 0:
	
l2882:	
	line	113
	
l11098:	
;map.c: 113: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	114
	
l11100:	
;map.c: 114: finalY = 0;
	clrf	(_finalY)
	line	115
;map.c: 115: break;
	goto	l11118
	line	116
;map.c: 116: case 1:
	
l2884:	
	line	117
	
l11102:	
;map.c: 117: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	118
	
l11104:	
;map.c: 118: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	119
;map.c: 119: break;
	goto	l11118
	line	120
;map.c: 120: case 2:
	
l2885:	
	line	121
	
l11106:	
;map.c: 121: if (robotOrientation == SOUTH)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	01h
	skipz
	goto	u4521
	goto	u4520
u4521:
	goto	l11118
u4520:
	line	123
	
l11108:	
;map.c: 122: {
;map.c: 123: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	124
;map.c: 124: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	goto	l11118
	line	125
	
l2886:	
	line	126
;map.c: 125: }
;map.c: 126: break;
	goto	l11118
	line	127
;map.c: 127: case 3:
	
l2887:	
	line	128
;map.c: 128: finalX = 0;
	clrf	(_finalX)
	line	129
;map.c: 129: finalY = 0;
	clrf	(_finalY)
	line	130
;map.c: 130: break;
	goto	l11118
	line	131
;map.c: 131: default:
	
l2888:	
	line	132
;map.c: 132: break;
	goto	l11118
	line	133
	
l11110:	
;map.c: 133: }
	goto	l11118
	line	110
	
l2881:	
	
l11112:	
	movf	(findFinalDestination@virtualWallY),w
	; Switch size 1, requested type "space"
; Number of cases is 4, Range of values is 0 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           13     7 (average)
; direct_byte           20     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             8     6 (fixed)
; spacedrange           14     9 (fixed)
; locatedrange           4     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11098
	xorlw	1^0	; case 1
	skipnz
	goto	l11102
	xorlw	2^1	; case 2
	skipnz
	goto	l11106
	xorlw	3^2	; case 3
	skipnz
	goto	l2887
	goto	l11118
	opt asmopt_on

	line	133
	
l2883:	
	line	134
;map.c: 134: break;
	goto	l11118
	line	136
;map.c: 136: default:
	
l2889:	
	line	137
;map.c: 137: break;
	goto	l11118
	line	138
	
l11114:	
;map.c: 138: }
	goto	l11118
	line	13
	
l2848:	
	
l11116:	
	movf	(findFinalDestination@virtualWallX),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 0 to 4
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           23     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             9     6 (fixed)
; spacedrange           16     9 (fixed)
; locatedrange           5     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11050
	xorlw	1^0	; case 1
	skipnz
	goto	l11068
	xorlw	2^1	; case 2
	skipnz
	goto	l11086
	xorlw	3^2	; case 3
	skipnz
	goto	l11096
	xorlw	4^3	; case 4
	skipnz
	goto	l11112
	goto	l11118
	opt asmopt_on

	line	138
	
l2856:	
	line	140
	
l11118:	
;map.c: 140: lcd_set_cursor(0x47);
	movlw	(047h)
	fcall	_lcd_set_cursor
	line	141
;map.c: 141: lcd_write_1_digit_bcd(finalX);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	fcall	_lcd_write_1_digit_bcd
	line	142
;map.c: 142: lcd_set_cursor(0x49);
	movlw	(049h)
	fcall	_lcd_set_cursor
	line	143
;map.c: 143: lcd_write_1_digit_bcd(finalY);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	fcall	_lcd_write_1_digit_bcd
	line	144
	
l2890:	
	return
	opt stack 0
GLOBAL	__end_of_findFinalDestination
	__end_of_findFinalDestination:
;; =============== function _findFinalDestination ends ============

	signat	_findFinalDestination,12408
	global	_updateMapData
psect	text1317,local,class=CODE,delta=2
global __ptext1317
__ptext1317:

;; *************** function _updateMapData *****************
;; Defined at:
;;		line 134 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;  virtualW        1    wreg     unsigned char 
;;  virtualS        1    9[BANK0 ] unsigned char 
;;  virtualE        1   10[BANK0 ] unsigned char 
;;  virtualN        1   11[BANK0 ] unsigned char 
;;  victim          1   12[BANK0 ] unsigned char 
;;  move            1   13[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  virtualW        1   16[BANK0 ] unsigned char 
;;  completeData    1   17[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       5       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       9       0       0       0
;;Total ram usage:        9 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_addNewData
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1317
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	134
	global	__size_of_updateMapData
	__size_of_updateMapData	equ	__end_of_updateMapData-_updateMapData
	
_updateMapData:	
	opt	stack 3
; Regs used in _updateMapData: [wreg+status,2+status,0+pclath+cstack]
;updateMapData@virtualW stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateMapData@virtualW)
	line	135
	
l11034:	
;eeprom.c: 135: char completeData = 0;
	clrf	(updateMapData@completeData)
	line	136
	
l11036:	
;eeprom.c: 136: completeData |= virtualW;
	movf	(updateMapData@virtualW),w
	movwf	(??_updateMapData+0)+0
	movf	(??_updateMapData+0)+0,w
	iorwf	(updateMapData@completeData),f
	line	137
;eeprom.c: 137: completeData |= virtualS << 1;
	movf	(updateMapData@virtualS),w
	movwf	(??_updateMapData+0)+0
	addwf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	138
;eeprom.c: 138: completeData |= virtualE << 2;
	movf	(updateMapData@virtualE),w
	movwf	(??_updateMapData+0)+0
	movlw	(02h)-1
u4465:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4465
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	139
;eeprom.c: 139: completeData |= virtualN << 3;
	movf	(updateMapData@virtualN),w
	movwf	(??_updateMapData+0)+0
	movlw	(03h)-1
u4475:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4475
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	140
;eeprom.c: 140: completeData |= victim << 4;
	movf	(updateMapData@victim),w
	movwf	(??_updateMapData+0)+0
	movlw	(04h)-1
u4485:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4485
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	141
;eeprom.c: 141: completeData |= move << 5;
	movf	(updateMapData@move),w
	movwf	(??_updateMapData+0)+0
	movlw	(05h)-1
u4495:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4495
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	142
;eeprom.c: 142: completeData &= 0b01111111;
	movlw	(07Fh)
	movwf	(??_updateMapData+0)+0
	movf	(??_updateMapData+0)+0,w
	andwf	(updateMapData@completeData),f
	line	143
	
l11038:	
;eeprom.c: 143: addNewData(completeData);
	movf	(updateMapData@completeData),w
	fcall	_addNewData
	line	144
	
l1430:	
	return
	opt stack 0
GLOBAL	__end_of_updateMapData
	__end_of_updateMapData:
;; =============== function _updateMapData ends ============

	signat	_updateMapData,24696
	global	_writeEEPROMTestData
psect	text1318,local,class=CODE,delta=2
global __ptext1318
__ptext1318:

;; *************** function _writeEEPROMTestData *****************
;; Defined at:
;;		line 103 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_addNewData
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1318
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	103
	global	__size_of_writeEEPROMTestData
	__size_of_writeEEPROMTestData	equ	__end_of_writeEEPROMTestData-_writeEEPROMTestData
	
_writeEEPROMTestData:	
	opt	stack 3
; Regs used in _writeEEPROMTestData: [wreg+status,2+status,0+pclath+cstack]
	line	104
	
l11032:	
;eeprom.c: 104: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	105
;eeprom.c: 105: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	106
;eeprom.c: 106: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	107
;eeprom.c: 107: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	108
;eeprom.c: 108: addNewData(96);
	movlw	(060h)
	fcall	_addNewData
	line	109
;eeprom.c: 109: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	110
;eeprom.c: 110: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	111
;eeprom.c: 111: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	112
;eeprom.c: 112: addNewData(96);
	movlw	(060h)
	fcall	_addNewData
	line	113
;eeprom.c: 113: addNewData(1);
	movlw	(01h)
	fcall	_addNewData
	line	114
;eeprom.c: 114: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	115
;eeprom.c: 115: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	116
;eeprom.c: 116: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	117
;eeprom.c: 117: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	118
;eeprom.c: 118: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	119
;eeprom.c: 119: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	120
;eeprom.c: 120: addNewData(96);
	movlw	(060h)
	fcall	_addNewData
	line	121
;eeprom.c: 121: addNewData(64);
	movlw	(040h)
	fcall	_addNewData
	line	122
;eeprom.c: 122: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	123
;eeprom.c: 123: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	124
;eeprom.c: 124: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	125
;eeprom.c: 125: addNewData(96);
	movlw	(060h)
	fcall	_addNewData
	line	126
;eeprom.c: 126: addNewData(0);
	movlw	(0)
	fcall	_addNewData
	line	127
;eeprom.c: 127: addNewData(32);
	movlw	(020h)
	fcall	_addNewData
	line	128
;eeprom.c: 128: addNewData(112);
	movlw	(070h)
	fcall	_addNewData
	line	129
	
l1427:	
	return
	opt stack 0
GLOBAL	__end_of_writeEEPROMTestData
	__end_of_writeEEPROMTestData:
;; =============== function _writeEEPROMTestData ends ============

	signat	_writeEEPROMTestData,88
	global	_checkIfHome
psect	text1319,local,class=CODE,delta=2
global __ptext1319
__ptext1319:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 355 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_drive
;;		_play_iCreate_song
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1319
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	355
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 4
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	356
	
l11024:	
;main.c: 356: if((xCoord == 1) && (yCoord == 3))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u4441
	goto	u4440
u4441:
	goto	l6808
u4440:
	
l11026:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4451
	goto	u4450
u4451:
	goto	l6808
u4450:
	line	358
	
l11028:	
;main.c: 357: {
;main.c: 358: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	359
;main.c: 359: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	360
	
l11030:	
;main.c: 360: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l6808
	line	361
	
l6807:	
	line	362
	
l6808:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_turnAround
psect	text1320,local,class=CODE,delta=2
global __ptext1320
__ptext1320:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 263 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_goBackward
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1320
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	263
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 2
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	264
	
l11018:	
;drive.c: 264: drive(0, 50, 0, 1);
	movlw	(032h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	bsf	status,0
	rlf	0+(?_drive)+02h,f
	movlw	(0)
	fcall	_drive
	line	265
;drive.c: 265: waitFor(157,0,180);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0B4h)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	266
;drive.c: 266: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	267
	
l11020:	
;drive.c: 267: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	25
movwf	((??_turnAround+0)+0+1),f
	movlw	154
movwf	((??_turnAround+0)+0),f
u5387:
	decfsz	((??_turnAround+0)+0),f
	goto	u5387
	decfsz	((??_turnAround+0)+0+1),f
	goto	u5387
	decfsz	((??_turnAround+0)+0+2),f
	goto	u5387
	nop2
opt asmopt_on

	line	268
	
l11022:	
;drive.c: 268: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	25
movwf	((??_turnAround+0)+0+1),f
	movlw	154
movwf	((??_turnAround+0)+0),f
u5397:
	decfsz	((??_turnAround+0)+0),f
	goto	u5397
	decfsz	((??_turnAround+0)+0+1),f
	goto	u5397
	decfsz	((??_turnAround+0)+0+2),f
	goto	u5397
	nop2
opt asmopt_on

	line	269
	
l5876:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_turnLeft90
psect	text1321,local,class=CODE,delta=2
global __ptext1321
__ptext1321:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 272 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_drive
;;		_getCurrentX
;;		_getCurrentY
;;		_waitFor
;; This function is called by:
;;		_driveForDistance
;;		_goLeft
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1321
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	272
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 2
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	273
	
l11006:	
;drive.c: 273: drive(0, 50, 0, 1);
	movlw	(032h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	bsf	status,0
	rlf	0+(?_drive)+02h,f
	movlw	(0)
	fcall	_drive
	line	274
	
l11008:	
;drive.c: 274: if( (getCurrentX() == 2 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	02h
	skipz
	goto	u4421
	goto	u4420
u4421:
	goto	l11014
u4420:
	
l11010:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u4431
	goto	u4430
u4431:
	goto	l11014
u4430:
	line	276
	
l11012:	
;drive.c: 275: {
;drive.c: 276: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	277
;drive.c: 277: }else
	goto	l5880
	
l5879:	
	line	279
	
l11014:	
;drive.c: 278: {
;drive.c: 279: waitFor(157,0,90);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(05Ah)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	280
	
l5880:	
	line	281
;drive.c: 280: }
;drive.c: 281: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	282
	
l11016:	
;drive.c: 282: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	25
movwf	((??_turnLeft90+0)+0+1),f
	movlw	154
movwf	((??_turnLeft90+0)+0),f
u5407:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u5407
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u5407
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u5407
	nop2
opt asmopt_on

	line	283
	
l5881:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnRight90
psect	text1322,local,class=CODE,delta=2
global __ptext1322
__ptext1322:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 286 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_driveForDistance
;;		_goRight
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1322
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	286
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 2
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	287
	
l11002:	
;drive.c: 287: drive(0, 50, 255, 255);
	movlw	(032h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_turnRight90+0)+0
	movf	(??_turnRight90+0)+0,w
	movwf	(?_drive)
	movlw	(0FFh)
	movwf	(??_turnRight90+1)+0
	movf	(??_turnRight90+1)+0,w
	movwf	0+(?_drive)+01h
	movlw	(0FFh)
	movwf	(??_turnRight90+2)+0
	movf	(??_turnRight90+2)+0,w
	movwf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	288
;drive.c: 288: waitFor(157,255,174);
	movlw	(0FFh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_turnRight90+0)+0
	movf	(??_turnRight90+0)+0,w
	movwf	(?_waitFor)
	movlw	(0AEh)
	movwf	(??_turnRight90+1)+0
	movf	(??_turnRight90+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	289
;drive.c: 289: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	290
	
l11004:	
;drive.c: 290: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	25
movwf	((??_turnRight90+0)+0+1),f
	movlw	154
movwf	((??_turnRight90+0)+0),f
u5417:
	decfsz	((??_turnRight90+0)+0),f
	goto	u5417
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u5417
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u5417
	nop2
opt asmopt_on

	line	291
	
l5884:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_initSongs
psect	text1323,local,class=CODE,delta=2
global __ptext1323
__ptext1323:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_putArr
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text1323
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	32
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 3
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	33
	
l11000:	
;songs.c: 33: ser_putArr(finalCountdown, 27);
	movlw	(_finalCountdown&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x3/2)
	movwf	(?_ser_putArr+1)
	movlw	low(01Bh)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(01Bh)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	34
;songs.c: 34: ser_putArr(superMarioBros, 25);
	movlw	(_superMarioBros&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x3/2)
	movwf	(?_ser_putArr+1)
	movlw	low(019h)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(019h)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	35
;songs.c: 35: ser_putArr(lookingForU2, 29);
	movlw	(_lookingForU2&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x3/2)
	movwf	(?_ser_putArr+1)
	movlw	low(01Dh)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(01Dh)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	36
;songs.c: 36: ser_putArr(champions, 21);
	movlw	(_champions&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x1/2)
	movwf	(?_ser_putArr+1)
	movlw	low(015h)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(015h)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	37
;songs.c: 37: ser_putArr(beep, 5);
	movlw	(_beep&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x1/2)
	movwf	(?_ser_putArr+1)
	movlw	low(05h)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(05h)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	38
;songs.c: 38: ser_putArr(longbeep, 6);
	movlw	(_longbeep&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x1/2)
	movwf	(?_ser_putArr+1)
	movlw	low(06h)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(06h)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	39
	
l4377:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text1324,local,class=CODE,delta=2
global __ptext1324
__ptext1324:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_lcd_write_control
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text1324
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 4
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l10980:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l10982:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l10984:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l10986:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l10988:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l10990:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l10992:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l10994:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l10996:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l10998:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l2153:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1325,local,class=CODE,delta=2
global __ptext1325
__ptext1325:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1    3[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_lcd_write_data
;; This function is called by:
;;		_findFinalDestination
;;		_lookForVictim
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text1325
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	44
	global	__size_of_lcd_write_1_digit_bcd
	__size_of_lcd_write_1_digit_bcd	equ	__end_of_lcd_write_1_digit_bcd-_lcd_write_1_digit_bcd
	
_lcd_write_1_digit_bcd:	
	opt	stack 4
; Regs used in _lcd_write_1_digit_bcd: [wreg+status,2+status,0+pclath+cstack]
;lcd_write_1_digit_bcd@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_1_digit_bcd@data)
	line	45
	
l10978:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l2141:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_set_cursor
psect	text1326,local,class=CODE,delta=2
global __ptext1326
__ptext1326:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  address         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  address         1    3[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_lcd_write_control
;; This function is called by:
;;		_findFinalDestination
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goReverse
;;		_goRight
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_updateLocation
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1326
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	32
	global	__size_of_lcd_set_cursor
	__size_of_lcd_set_cursor	equ	__end_of_lcd_set_cursor-_lcd_set_cursor
	
_lcd_set_cursor:	
	opt	stack 4
; Regs used in _lcd_set_cursor: [wreg+status,2+status,0+pclath+cstack]
;lcd_set_cursor@address stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_set_cursor@address)
	line	33
	
l10974:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l10976:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l2132:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_EEPROMToSerial
psect	text1327,local,class=CODE,delta=2
global __ptext1327
__ptext1327:

;; *************** function _EEPROMToSerial *****************
;; Defined at:
;;		line 148 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  transferDone    1   10[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_readEEPROM
;;		_ser_putch
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1327
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	148
	global	__size_of_EEPROMToSerial
	__size_of_EEPROMToSerial	equ	__end_of_EEPROMToSerial-_EEPROMToSerial
	
_EEPROMToSerial:	
	opt	stack 4
; Regs used in _EEPROMToSerial: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	149
	
l10948:	
;eeprom.c: 149: char transferDone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(EEPROMToSerial@transferDone)
	line	150
;eeprom.c: 150: addressCurrent = 0;
	clrf	(_addressCurrent)
	line	151
	
l10950:	
;eeprom.c: 151: addressCount = readEEPROM(0,0);
	clrf	(?_readEEPROM)
	movlw	(0)
	fcall	_readEEPROM
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_EEPROMToSerial+0)+0
	movf	(??_EEPROMToSerial+0)+0,w
	movwf	(_addressCount)
	line	152
	
l10952:	
;eeprom.c: 152: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	138
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u5427:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u5427
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u5427
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u5427
	nop2
opt asmopt_on

	line	154
	
l10954:	
;eeprom.c: 154: ser_putch(254);
	movlw	(0FEh)
	fcall	_ser_putch
	line	156
;eeprom.c: 156: while(!transferDone && addressCount > 0)
	goto	l1433
	
l1434:	
	line	158
	
l10956:	
;eeprom.c: 157: {
;eeprom.c: 158: ser_putch(readEEPROM(0,1 + addressCurrent));
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_addressCurrent),w
	addlw	01h
	movwf	(??_EEPROMToSerial+0)+0
	movf	(??_EEPROMToSerial+0)+0,w
	movwf	(?_readEEPROM)
	movlw	(0)
	fcall	_readEEPROM
	fcall	_ser_putch
	line	159
	
l10958:	
;eeprom.c: 159: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	138
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u5437:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u5437
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u5437
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u5437
	nop2
opt asmopt_on

	line	160
	
l10960:	
;eeprom.c: 160: addressCurrent ++;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_EEPROMToSerial+0)+0
	movf	(??_EEPROMToSerial+0)+0,w
	addwf	(_addressCurrent),f
	line	161
	
l10962:	
;eeprom.c: 161: if(addressCurrent >= (addressCount))
	movf	(_addressCount),w
	subwf	(_addressCurrent),w
	skipc
	goto	u4391
	goto	u4390
u4391:
	goto	l1433
u4390:
	line	163
	
l10964:	
;eeprom.c: 162: {
;eeprom.c: 163: transferDone = 1;
	clrf	(EEPROMToSerial@transferDone)
	bsf	status,0
	rlf	(EEPROMToSerial@transferDone),f
	goto	l1433
	line	164
	
l1435:	
	line	165
	
l1433:	
	line	156
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(EEPROMToSerial@transferDone),f
	skipz
	goto	u4401
	goto	u4400
u4401:
	goto	l10968
u4400:
	
l10966:	
	movf	(_addressCount),f
	skipz
	goto	u4411
	goto	u4410
u4411:
	goto	l10956
u4410:
	goto	l10968
	
l1437:	
	goto	l10968
	
l1438:	
	line	167
	
l10968:	
;eeprom.c: 164: }
;eeprom.c: 165: }
;eeprom.c: 167: ser_putch(255);
	movlw	(0FFh)
	fcall	_ser_putch
	line	168
	
l10970:	
;eeprom.c: 168: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	138
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u5447:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u5447
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u5447
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u5447
	nop2
opt asmopt_on

	line	169
	
l10972:	
;eeprom.c: 169: ser_putch(255);
	movlw	(0FFh)
	fcall	_ser_putch
	line	170
	
l1439:	
	return
	opt stack 0
GLOBAL	__end_of_EEPROMToSerial
	__end_of_EEPROMToSerial:
;; =============== function _EEPROMToSerial ends ============

	signat	_EEPROMToSerial,88
	global	_addNewData
psect	text1328,local,class=CODE,delta=2
global __ptext1328
__ptext1328:

;; *************** function _addNewData *****************
;; Defined at:
;;		line 94 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1    8[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_writeEEPROM
;; This function is called by:
;;		_writeEEPROMTestData
;;		_updateMapData
;; This function uses a non-reentrant model
;;
psect	text1328
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	94
	global	__size_of_addNewData
	__size_of_addNewData	equ	__end_of_addNewData-_addNewData
	
_addNewData:	
	opt	stack 3
; Regs used in _addNewData: [wreg+status,2+status,0+pclath+cstack]
;addNewData@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(addNewData@data)
	line	95
	
l10942:	
;eeprom.c: 95: writeEEPROM(data,0,addressCount + 1);
	clrf	(?_writeEEPROM)
	movf	(_addressCount),w
	addlw	01h
	movwf	(??_addNewData+0)+0
	movf	(??_addNewData+0)+0,w
	movwf	0+(?_writeEEPROM)+01h
	movf	(addNewData@data),w
	fcall	_writeEEPROM
	line	96
	
l10944:	
;eeprom.c: 96: addressCount ++;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_addNewData+0)+0
	movf	(??_addNewData+0)+0,w
	addwf	(_addressCount),f
	line	97
	
l10946:	
;eeprom.c: 97: writeEEPROM(addressCount,0,0);
	clrf	(?_writeEEPROM)
	clrf	0+(?_writeEEPROM)+01h
	movf	(_addressCount),w
	fcall	_writeEEPROM
	line	98
	
l1424:	
	return
	opt stack 0
GLOBAL	__end_of_addNewData
	__end_of_addNewData:
;; =============== function _addNewData ends ============

	signat	_addNewData,4216
	global	_lcd_write_string
psect	text1329,local,class=CODE,delta=2
global __ptext1329
__ptext1329:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  s               1    wreg     PTR const unsigned char 
;;		 -> STR_7(23), STR_6(23), STR_5(23), STR_4(17), 
;;		 -> STR_3(17), STR_2(14), STR_1(15), 
;; Auto vars:     Size  Location     Type
;;  s               1    4[BANK0 ] PTR const unsigned char 
;;		 -> STR_7(23), STR_6(23), STR_5(23), STR_4(17), 
;;		 -> STR_3(17), STR_2(14), STR_1(15), 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_lcd_write_data
;; This function is called by:
;;		_main
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text1329
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	38
	global	__size_of_lcd_write_string
	__size_of_lcd_write_string	equ	__end_of_lcd_write_string-_lcd_write_string
	
_lcd_write_string:	
	opt	stack 5
; Regs used in _lcd_write_string: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;lcd_write_string@s stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_string@s)
	line	40
	
l10934:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l10940
	
l2136:	
	
l10936:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l10938:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l10940
	
l2135:	
	
l10940:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u4381
	goto	u4380
u4381:
	goto	l10936
u4380:
	goto	l2138
	
l2137:	
	line	41
	
l2138:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_adc_read_channel
psect	text1330,local,class=CODE,delta=2
global __ptext1330
__ptext1330:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
;; Parameters:    Size  Location     Type
;;  channel         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  channel         1   30[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   27[BANK0 ] int 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_adc_read
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text1330
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
	line	7
	global	__size_of_adc_read_channel
	__size_of_adc_read_channel	equ	__end_of_adc_read_channel-_adc_read_channel
	
_adc_read_channel:	
	opt	stack 2
; Regs used in _adc_read_channel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
;adc_read_channel@channel stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(adc_read_channel@channel)
	line	8
	
l10918:	
;adc.c: 8: switch(channel)
	goto	l10926
	line	10
;adc.c: 9: {
;adc.c: 10: case 0:
	
l690:	
	line	11
;adc.c: 11: CHS0 = 0;
	bcf	(251/8),(251)&7
	line	12
;adc.c: 12: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	13
;adc.c: 13: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	14
;adc.c: 14: break;
	goto	l10928
	line	15
;adc.c: 15: case 1:
	
l692:	
	line	16
;adc.c: 16: CHS0 = 1;
	bsf	(251/8),(251)&7
	line	17
;adc.c: 17: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	18
;adc.c: 18: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	19
;adc.c: 19: break;
	goto	l10928
	line	20
;adc.c: 20: case 2:
	
l693:	
	line	21
;adc.c: 21: CHS0 = 0;
	bcf	(251/8),(251)&7
	line	22
;adc.c: 22: CHS1 = 1;
	bsf	(252/8),(252)&7
	line	23
;adc.c: 23: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	24
;adc.c: 24: break;
	goto	l10928
	line	25
;adc.c: 25: case 3:
	
l694:	
	line	26
;adc.c: 26: CHS0 = 1;
	bsf	(251/8),(251)&7
	line	27
;adc.c: 27: CHS1 = 1;
	bsf	(252/8),(252)&7
	line	28
;adc.c: 28: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	29
;adc.c: 29: break;
	goto	l10928
	line	30
;adc.c: 30: case 4:
	
l695:	
	line	31
;adc.c: 31: CHS0 = 0;
	bcf	(251/8),(251)&7
	line	32
;adc.c: 32: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	33
;adc.c: 33: CHS2 = 1;
	bsf	(253/8),(253)&7
	line	34
;adc.c: 34: break;
	goto	l10928
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l10920:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l10922:	
	goto	l697
	line	39
	
l10924:	
;adc.c: 39: }
	goto	l10928
	line	8
	
l689:	
	
l10926:	
	movf	(adc_read_channel@channel),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 0 to 4
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           23     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             9     6 (fixed)
; spacedrange           16     9 (fixed)
; locatedrange           5     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l690
	xorlw	1^0	; case 1
	skipnz
	goto	l692
	xorlw	2^1	; case 2
	skipnz
	goto	l693
	xorlw	3^2	; case 3
	skipnz
	goto	l694
	xorlw	4^3	; case 4
	skipnz
	goto	l695
	goto	l10920
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l10928:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u5457:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u5457
opt asmopt_on

	line	43
	
l10930:	
;adc.c: 43: return adc_read();
	fcall	_adc_read
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_adc_read)),w
	clrf	(?_adc_read_channel+1)
	addwf	(?_adc_read_channel+1)
	movf	(0+(?_adc_read)),w
	clrf	(?_adc_read_channel)
	addwf	(?_adc_read_channel)

	goto	l697
	
l10932:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_initIRobot
psect	text1331,local,class=CODE,delta=2
global __ptext1331
__ptext1331:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 154 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text1331
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	154
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 4
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	155
	
l10912:	
;main.c: 155: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u5467:
	decfsz	((??_initIRobot+0)+0),f
	goto	u5467
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u5467
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u5467
	nop2
opt asmopt_on

	line	156
	
l10914:	
;main.c: 156: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	157
	
l10916:	
;main.c: 157: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	158
	
l6747:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_waitFor
psect	text1332,local,class=CODE,delta=2
global __ptext1332
__ptext1332:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 301 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;  type            1    wreg     unsigned char 
;;  highByte        1    2[BANK0 ] unsigned char 
;;  lowByte         1    3[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  type            1    7[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       6       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_goReverse
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_wallFollow
;;		_leftAngleCorrect
;; This function uses a non-reentrant model
;;
psect	text1332
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	301
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 2
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	302
	
l10904:	
;drive.c: 302: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u5477:
	decfsz	((??_waitFor+0)+0),f
	goto	u5477
	decfsz	((??_waitFor+0)+0+1),f
	goto	u5477
	decfsz	((??_waitFor+0)+0+2),f
	goto	u5477
	nop2
opt asmopt_on

	line	303
	
l10906:	
;drive.c: 303: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	304
	
l10908:	
;drive.c: 304: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	305
	
l10910:	
;drive.c: 305: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	306
	
l5891:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_drive
psect	text1333,local,class=CODE,delta=2
global __ptext1333
__ptext1333:

;; *************** function _drive *****************
;; Defined at:
;;		line 22 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;  highByteSpee    1    wreg     unsigned char 
;;  lowByteSpeed    1    2[BANK0 ] unsigned char 
;;  highByteRadi    1    3[BANK0 ] unsigned char 
;;  lowByteRadiu    1    4[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  highByteSpee    1    8[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       3       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       7       0       0       0
;;Total ram usage:        7 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_driveForDistance
;;		_goReverse
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_frontWallCorrect
;;		_checkIfHome
;;		_main
;;		_wallFollow
;;		_leftAngleCorrect
;; This function uses a non-reentrant model
;;
psect	text1333
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	22
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 4
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	23
	
l10892:	
;drive.c: 23: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u5487:
	decfsz	((??_drive+0)+0),f
	goto	u5487
	decfsz	((??_drive+0)+0+1),f
	goto	u5487
	decfsz	((??_drive+0)+0+2),f
	goto	u5487
	nop2
opt asmopt_on

	line	24
	
l10894:	
;drive.c: 24: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	25
	
l10896:	
;drive.c: 25: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	26
	
l10898:	
;drive.c: 26: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	27
	
l10900:	
;drive.c: 27: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	28
	
l10902:	
;drive.c: 28: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	29
	
l5820:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_rotateIR
psect	text1334,local,class=CODE,delta=2
global __ptext1334
__ptext1334:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
;; Parameters:    Size  Location     Type
;;  steps           1    wreg     unsigned char 
;;  direction       1    0[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  steps           1    4[BANK0 ] unsigned char 
;;  stepNum         1    5[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       1       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       6       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_findWalls
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1334
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
	line	39
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 6
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	40
	
l10878:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l10880:	
;ir.c: 41: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	44
;ir.c: 44: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l5086
	line	45
	
l5087:	
	line	46
;ir.c: 45: {
;ir.c: 46: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	47
	
l10882:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l10884:	
;ir.c: 48: _delay((unsigned long)((50)*(20000000/4000.0)));
	opt asmopt_off
movlw  2
movwf	((??_rotateIR+0)+0+2),f
movlw	69
movwf	((??_rotateIR+0)+0+1),f
	movlw	169
movwf	((??_rotateIR+0)+0),f
u5497:
	decfsz	((??_rotateIR+0)+0),f
	goto	u5497
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u5497
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u5497
	nop2
opt asmopt_on

	line	44
	
l10886:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l5086:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u4371
	goto	u4370
u4371:
	goto	l5087
u4370:
	goto	l10888
	
l5088:	
	line	51
	
l10888:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l10890:	
;ir.c: 52: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_rotateIR+0)+0+2),f
movlw	138
movwf	((??_rotateIR+0)+0+1),f
	movlw	86
movwf	((??_rotateIR+0)+0),f
u5507:
	decfsz	((??_rotateIR+0)+0),f
	goto	u5507
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u5507
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u5507
	nop2
opt asmopt_on

	line	54
	
l5089:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_convert
psect	text1335,local,class=CODE,delta=2
global __ptext1335
__ptext1335:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
;; Parameters:    Size  Location     Type
;;  adc_value       2   23[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  2   23[BANK0 ] int 
;; Registers used:
;;		wreg, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		___wmul
;;		___awdiv
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text1335
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 3
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l10818:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4305
	movlw	low(052h)
	subwf	(convert@adc_value),w
u4305:

	skipnc
	goto	u4301
	goto	u4300
u4301:
	goto	l10826
u4300:
	line	13
	
l10820:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5067
	
l10822:	
	goto	l5067
	
l10824:	
	goto	l5067
	line	14
	
l5066:	
	
l10826:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4315
	movlw	low(085h)
	subwf	(convert@adc_value),w
u4315:

	skipnc
	goto	u4311
	goto	u4310
u4311:
	goto	l10834
u4310:
	line	15
	
l10828:	
;ir.c: 15: return (100 + (150-100)*(133 - adc_value)/(133 - 82) - 3);
	movlw	low(033h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(033h)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(085h)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(085h)
	movwf	1+(?___wmul)
	movlw	low(032h)
	movwf	0+(?___wmul)+02h
	movlw	high(032h)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(061h)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(061h)
	movwf	1+(?_convert)
	goto	l5067
	
l10830:	
	goto	l5067
	
l10832:	
	goto	l5067
	line	16
	
l5069:	
	
l10834:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4325
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u4325:

	skipnc
	goto	u4321
	goto	u4320
u4321:
	goto	l10842
u4320:
	line	17
	
l10836:	
;ir.c: 17: return (70 + (100-70)*(184 - adc_value)/(184 - 133) - 3);
	movlw	low(033h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(033h)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(0B8h)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(0B8h)
	movwf	1+(?___wmul)
	movlw	low(01Eh)
	movwf	0+(?___wmul)+02h
	movlw	high(01Eh)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(043h)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(043h)
	movwf	1+(?_convert)
	goto	l5067
	
l10838:	
	goto	l5067
	
l10840:	
	goto	l5067
	line	18
	
l5071:	
	
l10842:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4335
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u4335:

	skipnc
	goto	u4331
	goto	u4330
u4331:
	goto	l10850
u4330:
	line	19
	
l10844:	
;ir.c: 19: return (50 + (70-50)*(256 - adc_value)/(256 - 184) - 4);
	movlw	low(048h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(048h)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(0100h)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(0100h)
	movwf	1+(?___wmul)
	movlw	low(014h)
	movwf	0+(?___wmul)+02h
	movlw	high(014h)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(02Eh)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(02Eh)
	movwf	1+(?_convert)
	goto	l5067
	
l10846:	
	goto	l5067
	
l10848:	
	goto	l5067
	line	20
	
l5073:	
	
l10850:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4345
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u4345:

	skipnc
	goto	u4341
	goto	u4340
u4341:
	goto	l10858
u4340:
	line	21
	
l10852:	
;ir.c: 21: return (40 + (50-40)*(317 - adc_value)/(317 - 256) - 3);
	movlw	low(03Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(03Dh)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(013Dh)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(013Dh)
	movwf	1+(?___wmul)
	movlw	low(0Ah)
	movwf	0+(?___wmul)+02h
	movlw	high(0Ah)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(025h)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(025h)
	movwf	1+(?_convert)
	goto	l5067
	
l10854:	
	goto	l5067
	
l10856:	
	goto	l5067
	line	22
	
l5075:	
	
l10858:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4355
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u4355:

	skipnc
	goto	u4351
	goto	u4350
u4351:
	goto	l10866
u4350:
	line	23
	
l10860:	
;ir.c: 23: return (30 + (40-30)*(410 - adc_value)/(410 - 317) - 2);
	movlw	low(05Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(05Dh)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(019Ah)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(019Ah)
	movwf	1+(?___wmul)
	movlw	low(0Ah)
	movwf	0+(?___wmul)+02h
	movlw	high(0Ah)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(01Ch)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(01Ch)
	movwf	1+(?_convert)
	goto	l5067
	
l10862:	
	goto	l5067
	
l10864:	
	goto	l5067
	line	24
	
l5077:	
	
l10866:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4365
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u4365:

	skipnc
	goto	u4361
	goto	u4360
u4361:
	goto	l10874
u4360:
	line	25
	
l10868:	
;ir.c: 25: return (20 + (30-20)*(522 - adc_value)/(522 - 410) - 2);
	movlw	low(070h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___awdiv)
	movlw	high(070h)
	movwf	((?___awdiv))+1
	comf	(convert@adc_value),w
	movwf	(??_convert+0)+0
	comf	(convert@adc_value+1),w
	movwf	((??_convert+0)+0+1)
	incf	(??_convert+0)+0,f
	skipnz
	incf	((??_convert+0)+0+1),f
	movf	0+(??_convert+0)+0,w
	addlw	low(020Ah)
	movwf	(?___wmul)
	movf	1+(??_convert+0)+0,w
	skipnc
	addlw	1
	addlw	high(020Ah)
	movwf	1+(?___wmul)
	movlw	low(0Ah)
	movwf	0+(?___wmul)+02h
	movlw	high(0Ah)
	movwf	(0+(?___wmul)+02h)+1
	fcall	___wmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___wmul)),w
	clrf	1+(?___awdiv)+02h
	addwf	1+(?___awdiv)+02h
	movf	(0+(?___wmul)),w
	clrf	0+(?___awdiv)+02h
	addwf	0+(?___awdiv)+02h

	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___awdiv)),w
	addlw	low(012h)
	movwf	(?_convert)
	movf	(1+(?___awdiv)),w
	skipnc
	addlw	1
	addlw	high(012h)
	movwf	1+(?_convert)
	goto	l5067
	
l10870:	
	goto	l5067
	
l10872:	
	goto	l5067
	line	26
	
l5079:	
	
l10874:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5067
	
l10876:	
	goto	l5067
	
l5080:	
	goto	l5067
	
l5078:	
	goto	l5067
	
l5076:	
	goto	l5067
	
l5074:	
	goto	l5067
	
l5072:	
	goto	l5067
	
l5070:	
	goto	l5067
	
l5068:	
	line	27
	
l5067:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_play_iCreate_song
psect	text1336,local,class=CODE,delta=2
global __ptext1336
__ptext1336:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 26 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
;; Parameters:    Size  Location     Type
;;  song            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  song            1    2[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_checkIfHome
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1336
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\songs.c"
	line	26
	global	__size_of_play_iCreate_song
	__size_of_play_iCreate_song	equ	__end_of_play_iCreate_song-_play_iCreate_song
	
_play_iCreate_song:	
	opt	stack 4
; Regs used in _play_iCreate_song: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;play_iCreate_song@song stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(play_iCreate_song@song)
	line	27
	
l10816:	
;songs.c: 27: ser_putch(141);
	movlw	(08Dh)
	fcall	_ser_putch
	line	28
;songs.c: 28: ser_putch(song);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(play_iCreate_song@song),w
	fcall	_ser_putch
	line	29
	
l4374:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_ser_putArr
psect	text1337,local,class=CODE,delta=2
global __ptext1337
__ptext1337:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
;; Parameters:    Size  Location     Type
;;  array           2    2[BANK0 ] PTR unsigned char 
;;		 -> longbeep(5), beep(5), champions(21), lookingForU2(29), 
;;		 -> superMarioBros(25), finalCountdown(27), 
;;  length          2    4[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  i               2    9[BANK0 ] int 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       4       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       9       0       0       0
;;Total ram usage:        9 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_initSongs
;; This function uses a non-reentrant model
;;
psect	text1337
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 3
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l10808:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l10814
	line	75
	
l3643:	
	line	76
	
l10810:	
;ser.c: 75: {
;ser.c: 76: ser_putch(array[i]);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(ser_putArr@i),w
	addwf	(ser_putArr@array),w
	movwf	(??_ser_putArr+1)+0
	movf	(ser_putArr@array+1),w
	movwf	(??_ser_putArr+0)+0
	skipnc
	incf	(??_ser_putArr+0)+0,f
	btfss	(ser_putArr@i),7
	goto	u4280
	decf	(??_ser_putArr+0)+0,f
u4280:
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(??_ser_putArr+0)+0,w
	movwf	0+((??_ser_putArr+1)+0)+1
	movf	0+(??_ser_putArr+1)+0,w
	movwf	fsr0
	bsf	status,7
	btfss	1+(??_ser_putArr+1)+0,0
	bcf	status,7
	movf	indf,w
	fcall	_ser_putch
	line	74
	
l10812:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l10814
	
l3642:	
	
l10814:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u4295
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u4295:

	skipc
	goto	u4291
	goto	u4290
u4291:
	goto	l10810
u4290:
	goto	l3645
	
l3644:	
	line	78
	
l3645:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putArr
	__end_of_ser_putArr:
;; =============== function _ser_putArr ends ============

	signat	_ser_putArr,8312
	global	_ser_getch
psect	text1338,local,class=CODE,delta=2
global __ptext1338
__ptext1338:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  c               1    1[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_ser_isrx
;; This function is called by:
;;		_driveForDistance
;;		_lookForVictim
;; This function uses a non-reentrant model
;;
psect	text1338
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 3
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l10792:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l10794
	
l3637:	
	line	62
;ser.c: 62: continue;
	goto	l10794
	
l3636:	
	line	61
	
l10794:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u4271
	goto	u4270
u4271:
	goto	l10794
u4270:
	
l3638:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l10796:	
;ser.c: 65: c=rxfifo[rxoptr];
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_rxoptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	movwf	(ser_getch@c)
	line	66
	
l10798:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l10800:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l10802:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l10804:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l3639
	
l10806:	
	line	70
	
l3639:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_lcd_write_data
psect	text1339,local,class=CODE,delta=2
global __ptext1339
__ptext1339:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  databyte        1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  databyte        1    2[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_lcd_write_string
;;		_lcd_write_1_digit_bcd
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goReverse
;;		_goRight
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_updateLocation
;;		_main
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1339
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	20
	global	__size_of_lcd_write_data
	__size_of_lcd_write_data	equ	__end_of_lcd_write_data-_lcd_write_data
	
_lcd_write_data:	
	opt	stack 4
; Regs used in _lcd_write_data: [wreg]
;lcd_write_data@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_data@databyte)
	line	21
	
l10784:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l10786:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l10788:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l10790:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u5517:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u5517
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u5517
opt asmopt_on

	line	28
	
l2129:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_lcd_write_control
psect	text1340,local,class=CODE,delta=2
global __ptext1340
__ptext1340:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  databyte        1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  databyte        1    2[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_lcd_set_cursor
;;		_lcd_init
;; This function uses a non-reentrant model
;;
psect	text1340
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\lcd.c"
	line	8
	global	__size_of_lcd_write_control
	__size_of_lcd_write_control	equ	__end_of_lcd_write_control-_lcd_write_control
	
_lcd_write_control:	
	opt	stack 4
; Regs used in _lcd_write_control: [wreg]
;lcd_write_control@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_control@databyte)
	line	9
	
l10776:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l10778:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l10780:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l10782:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u5527:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u5527
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u5527
	nop2
opt asmopt_on

	line	16
	
l2126:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_readEEPROM
psect	text1341,local,class=CODE,delta=2
global __ptext1341
__ptext1341:

;; *************** function _readEEPROM *****************
;; Defined at:
;;		line 50 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;  addressH        1    wreg     unsigned char 
;;  addressL        1    1[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  addressH        1    5[BANK0 ] unsigned char 
;;  returnData      1    6[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       1       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       6       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_initEEPROMMode
;;		_writeSPIByte
;; This function is called by:
;;		_EEPROMToSerial
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text1341
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	50
	global	__size_of_readEEPROM
	__size_of_readEEPROM	equ	__end_of_readEEPROM-_readEEPROM
	
_readEEPROM:	
	opt	stack 4
; Regs used in _readEEPROM: [wreg+status,2+status,0+pclath+cstack]
;readEEPROM@addressH stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(readEEPROM@addressH)
	line	51
	
l10758:	
;eeprom.c: 51: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_readEEPROM+0)+0+2),f
movlw	138
movwf	((??_readEEPROM+0)+0+1),f
	movlw	86
movwf	((??_readEEPROM+0)+0),f
u5537:
	decfsz	((??_readEEPROM+0)+0),f
	goto	u5537
	decfsz	((??_readEEPROM+0)+0+1),f
	goto	u5537
	decfsz	((??_readEEPROM+0)+0+2),f
	goto	u5537
	nop2
opt asmopt_on

	line	52
	
l10760:	
;eeprom.c: 52: initEEPROMMode();
	fcall	_initEEPROMMode
	line	53
	
l10762:	
;eeprom.c: 53: unsigned char returnData = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(readEEPROM@returnData)
	line	55
	
l10764:	
;eeprom.c: 55: writeSPIByte(3);
	movlw	(03h)
	fcall	_writeSPIByte
	line	57
	
l10766:	
;eeprom.c: 57: writeSPIByte(addressH);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(readEEPROM@addressH),w
	fcall	_writeSPIByte
	line	59
	
l10768:	
;eeprom.c: 59: writeSPIByte(addressL);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(readEEPROM@addressL),w
	fcall	_writeSPIByte
	line	62
	
l10770:	
;eeprom.c: 62: SSPIF = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(99/8),(99)&7
	line	63
;eeprom.c: 63: SSPBUF = 0xFF;
	movlw	(0FFh)
	movwf	(19)	;volatile
	line	64
;eeprom.c: 64: while (!SSPIF);
	goto	l1413
	
l1414:	
	
l1413:	
	btfss	(99/8),(99)&7
	goto	u4261
	goto	u4260
u4261:
	goto	l1413
u4260:
	goto	l10772
	
l1415:	
	line	65
	
l10772:	
;eeprom.c: 65: returnData = SSPBUF;
	movf	(19),w	;volatile
	movwf	(??_readEEPROM+0)+0
	movf	(??_readEEPROM+0)+0,w
	movwf	(readEEPROM@returnData)
	line	67
;eeprom.c: 67: return returnData;
	movf	(readEEPROM@returnData),w
	goto	l1416
	
l10774:	
	line	68
	
l1416:	
	return
	opt stack 0
GLOBAL	__end_of_readEEPROM
	__end_of_readEEPROM:
;; =============== function _readEEPROM ends ============

	signat	_readEEPROM,8313
	global	_writeEEPROM
psect	text1342,local,class=CODE,delta=2
global __ptext1342
__ptext1342:

;; *************** function _writeEEPROM *****************
;; Defined at:
;;		line 27 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;;  addressH        1    1[BANK0 ] unsigned char 
;;  addressL        1    2[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1    6[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       6       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_initEEPROMMode
;;		_writeSPIByte
;; This function is called by:
;;		_addNewData
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text1342
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	27
	global	__size_of_writeEEPROM
	__size_of_writeEEPROM	equ	__end_of_writeEEPROM-_writeEEPROM
	
_writeEEPROM:	
	opt	stack 3
; Regs used in _writeEEPROM: [wreg+status,2+status,0+pclath+cstack]
;writeEEPROM@data stored from wreg
	line	29
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(writeEEPROM@data)
	
l10740:	
;eeprom.c: 29: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_writeEEPROM+0)+0+2),f
movlw	138
movwf	((??_writeEEPROM+0)+0+1),f
	movlw	86
movwf	((??_writeEEPROM+0)+0),f
u5547:
	decfsz	((??_writeEEPROM+0)+0),f
	goto	u5547
	decfsz	((??_writeEEPROM+0)+0+1),f
	goto	u5547
	decfsz	((??_writeEEPROM+0)+0+2),f
	goto	u5547
	nop2
opt asmopt_on

	line	30
	
l10742:	
;eeprom.c: 30: initEEPROMMode();
	fcall	_initEEPROMMode
	line	32
	
l10744:	
;eeprom.c: 32: writeSPIByte(6);
	movlw	(06h)
	fcall	_writeSPIByte
	line	33
	
l10746:	
;eeprom.c: 33: initEEPROMMode();
	fcall	_initEEPROMMode
	line	36
	
l10748:	
;eeprom.c: 36: writeSPIByte(2);
	movlw	(02h)
	fcall	_writeSPIByte
	line	39
	
l10750:	
;eeprom.c: 39: writeSPIByte(addressH);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressH),w
	fcall	_writeSPIByte
	line	42
	
l10752:	
;eeprom.c: 42: writeSPIByte(addressL);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressL),w
	fcall	_writeSPIByte
	line	45
	
l10754:	
;eeprom.c: 45: writeSPIByte(data);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@data),w
	fcall	_writeSPIByte
	line	46
	
l10756:	
;eeprom.c: 46: initEEPROMMode();
	fcall	_initEEPROMMode
	line	47
	
l1410:	
	return
	opt stack 0
GLOBAL	__end_of_writeEEPROM
	__end_of_writeEEPROM:
;; =============== function _writeEEPROM ends ============

	signat	_writeEEPROM,12408
	global	_init_adc
psect	text1343,local,class=CODE,delta=2
global __ptext1343
__ptext1343:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text1343
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 5
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l10730:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l10732:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l10734:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l10736:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l10738:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u5557:
decfsz	(??_init_adc+0)+0,f
	goto	u5557
opt asmopt_on

	line	58
	
l700:	
	return
	opt stack 0
GLOBAL	__end_of_init_adc
	__end_of_init_adc:
;; =============== function _init_adc ends ============

	signat	_init_adc,88
	global	_adc_read
psect	text1344,local,class=CODE,delta=2
global __ptext1344
__ptext1344:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  adc_value       2   21[BANK0 ] volatile int 
;; Return value:  Size  Location     Type
;;                  2   15[BANK0 ] int 
;; Registers used:
;;		wreg, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       4       0       0       0
;;      Totals:         0       8       0       0       0
;;Total ram usage:        8 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		___awdiv
;; This function is called by:
;;		_adc_read_channel
;; This function uses a non-reentrant model
;;
psect	text1344
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 2
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l10720:	
;adc.c: 63: volatile int adc_value;
;adc.c: 65: ADRESH = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(30)	;volatile
	line	66
;adc.c: 66: ADRESL = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(158)^080h	;volatile
	line	68
	
l10722:	
;adc.c: 68: GO = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(250/8),(250)&7
	line	69
;adc.c: 69: while(GO) continue;
	goto	l703
	
l704:	
	
l703:	
	btfsc	(250/8),(250)&7
	goto	u4241
	goto	u4240
u4241:
	goto	l703
u4240:
	goto	l10724
	
l705:	
	line	75
	
l10724:	
;adc.c: 75: adc_value = (ADRESH * 4) + (ADRESL / 64);
	movlw	low(040h)
	movwf	(?___awdiv)
	movlw	high(040h)
	movwf	((?___awdiv))+1
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(158)^080h,w	;volatile
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_adc_read+0)+0
	clrf	(??_adc_read+0)+0+1
	movf	0+(??_adc_read+0)+0,w
	movwf	0+(?___awdiv)+02h
	movf	1+(??_adc_read+0)+0,w
	movwf	1+(?___awdiv)+02h
	fcall	___awdiv
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(30),w	;volatile
	movwf	(??_adc_read+2)+0
	clrf	(??_adc_read+2)+0+1
	movlw	02h
	movwf	btemp+1
u4255:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4255
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l10726:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l10728:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1345,local,class=CODE,delta=2
global __ptext1345
__ptext1345:

;; *************** function ___awdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         2    6[BANK0 ] int 
;;  dividend        2    8[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  quotient        2   13[BANK0 ] int 
;;  sign            1   12[BANK0 ] unsigned char 
;;  counter         1   11[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2    6[BANK0 ] int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       4       0       0       0
;;      Locals:         0       4       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       9       0       0       0
;;Total ram usage:        9 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_adc_read
;;		_convert
;; This function uses a non-reentrant model
;;
psect	text1345
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 3
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l10680:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u4141
	goto	u4140
u4141:
	goto	l10684
u4140:
	line	11
	
l10682:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l10684
	line	13
	
l7710:	
	line	14
	
l10684:	
	btfss	(___awdiv@dividend+1),7
	goto	u4151
	goto	u4150
u4151:
	goto	l10690
u4150:
	line	15
	
l10686:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l10688:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l10690
	line	17
	
l7711:	
	line	18
	
l10690:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l10692:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u4161
	goto	u4160
u4161:
	goto	l10712
u4160:
	line	20
	
l10694:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l10700
	
l7714:	
	line	22
	
l10696:	
	movlw	01h
	
u4175:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u4175
	line	23
	
l10698:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l10700
	line	24
	
l7713:	
	line	21
	
l10700:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u4181
	goto	u4180
u4181:
	goto	l10696
u4180:
	goto	l10702
	
l7715:	
	goto	l10702
	line	25
	
l7716:	
	line	26
	
l10702:	
	movlw	01h
	
u4195:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u4195
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u4205
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u4205:
	skipc
	goto	u4201
	goto	u4200
u4201:
	goto	l10708
u4200:
	line	28
	
l10704:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l10706:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l10708
	line	30
	
l7717:	
	line	31
	
l10708:	
	movlw	01h
	
u4215:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u4215
	line	32
	
l10710:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u4221
	goto	u4220
u4221:
	goto	l10702
u4220:
	goto	l10712
	
l7718:	
	goto	l10712
	line	33
	
l7712:	
	line	34
	
l10712:	
	movf	(___awdiv@sign),w
	skipz
	goto	u4230
	goto	l10716
u4230:
	line	35
	
l10714:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l10716
	
l7719:	
	line	36
	
l10716:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l7720
	
l10718:	
	line	37
	
l7720:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1346,local,class=CODE,delta=2
global __ptext1346
__ptext1346:

;; *************** function ___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
;; Parameters:    Size  Location     Type
;;  multiplier      2    0[BANK0 ] unsigned int 
;;  multiplicand    2    2[BANK0 ] unsigned int 
;; Auto vars:     Size  Location     Type
;;  product         2    4[BANK0 ] unsigned int 
;; Return value:  Size  Location     Type
;;                  2    0[BANK0 ] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       4       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       6       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_convert
;; This function uses a non-reentrant model
;;
psect	text1346
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 3
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l10668:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l10670
	line	6
	
l7570:	
	line	7
	
l10670:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u4101
	goto	u4100
u4101:
	goto	l7571
u4100:
	line	8
	
l10672:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l7571:	
	line	9
	movlw	01h
	
u4115:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u4115
	line	10
	
l10674:	
	movlw	01h
	
u4125:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u4125
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u4131
	goto	u4130
u4131:
	goto	l10670
u4130:
	goto	l10676
	
l7572:	
	line	12
	
l10676:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l7573
	
l10678:	
	line	13
	
l7573:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_updateNode
psect	text1347,local,class=CODE,delta=2
global __ptext1347
__ptext1347:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 337 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1347
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	337
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 6
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	338
	
l10632:	
;main.c: 338: if((xCoord == 2) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3981
	goto	u3980
u3981:
	goto	l10638
u3980:
	
l10634:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3991
	goto	u3990
u3991:
	goto	l10638
u3990:
	line	339
	
l10636:	
;main.c: 339: node = 1;
	clrf	(_node)	;volatile
	bsf	status,0
	rlf	(_node),f	;volatile
	goto	l6804
	line	340
	
l6792:	
	
l10638:	
;main.c: 340: else if((xCoord == 4) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u4001
	goto	u4000
u4001:
	goto	l10644
u4000:
	
l10640:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4011
	goto	u4010
u4011:
	goto	l10644
u4010:
	line	341
	
l10642:	
;main.c: 341: node = 2;
	movlw	(02h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6804
	line	342
	
l6794:	
	
l10644:	
;main.c: 342: else if((xCoord == 2) && (yCoord == 0))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4021
	goto	u4020
u4021:
	goto	l10650
u4020:
	
l10646:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u4031
	goto	u4030
u4031:
	goto	l10650
u4030:
	line	343
	
l10648:	
;main.c: 343: node = 3;
	movlw	(03h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6804
	line	344
	
l6796:	
	
l10650:	
;main.c: 344: else if((xCoord == 4) && (yCoord == 3))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u4041
	goto	u4040
u4041:
	goto	l10656
u4040:
	
l10652:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4051
	goto	u4050
u4051:
	goto	l10656
u4050:
	line	345
	
l10654:	
;main.c: 345: node = 4;
	movlw	(04h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6804
	line	346
	
l6798:	
	
l10656:	
;main.c: 346: else if((xCoord == 2) && (yCoord == 1))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4061
	goto	u4060
u4061:
	goto	l10662
u4060:
	
l10658:	
	movf	(_yCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u4071
	goto	u4070
u4071:
	goto	l10662
u4070:
	line	347
	
l10660:	
;main.c: 347: node = 5;
	movlw	(05h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6804
	line	348
	
l6800:	
	
l10662:	
;main.c: 348: else if((xCoord == 3) && (yCoord == 0))
	movf	(_xCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4081
	goto	u4080
u4081:
	goto	l6802
u4080:
	
l10664:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u4091
	goto	u4090
u4091:
	goto	l6802
u4090:
	line	349
	
l10666:	
;main.c: 349: node = 6;
	movlw	(06h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6804
	line	350
	
l6802:	
	line	351
;main.c: 350: else
;main.c: 351: node = 0;
	clrf	(_node)	;volatile
	goto	l6804
	
l6803:	
	goto	l6804
	
l6801:	
	goto	l6804
	
l6799:	
	goto	l6804
	
l6797:	
	goto	l6804
	
l6795:	
	goto	l6804
	
l6793:	
	line	352
	
l6804:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getSuccessfulDrive
psect	text1348,local,class=CODE,delta=2
global __ptext1348
__ptext1348:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 195 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1348
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	195
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 6
; Regs used in _getSuccessfulDrive: [status]
	line	196
	
l10554:	
;drive.c: 196: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u3841
	goto	u3840
u3841:
	goto	l10558
u3840:
	
l10556:	
	clrc
	
	goto	l5853
	
l10340:	
	
l10558:	
	setc
	
	goto	l5853
	
l10342:	
	goto	l5853
	
l10560:	
	line	197
	
l5853:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_getSomethingInTheWay
psect	text1349,local,class=CODE,delta=2
global __ptext1349
__ptext1349:

;; *************** function _getSomethingInTheWay *****************
;; Defined at:
;;		line 185 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      enum E1111
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1349
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	185
	global	__size_of_getSomethingInTheWay
	__size_of_getSomethingInTheWay	equ	__end_of_getSomethingInTheWay-_getSomethingInTheWay
	
_getSomethingInTheWay:	
	opt	stack 5
; Regs used in _getSomethingInTheWay: [wreg]
	line	186
	
l10550:	
;drive.c: 186: return somethingInTheWay;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_somethingInTheWay),w	;volatile
	goto	l5847
	
l10552:	
	line	187
	
l5847:	
	return
	opt stack 0
GLOBAL	__end_of_getSomethingInTheWay
	__end_of_getSomethingInTheWay:
;; =============== function _getSomethingInTheWay ends ============

	signat	_getSomethingInTheWay,89
	global	_getOrientation
psect	text1350,local,class=CODE,delta=2
global __ptext1350
__ptext1350:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 180 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      enum E1117
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_updateLocation
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1350
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	180
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 5
; Regs used in _getOrientation: [wreg]
	line	181
	
l10546:	
;drive.c: 181: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l5844
	
l10548:	
	line	182
	
l5844:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_setVirtualLocation
psect	text1351,local,class=CODE,delta=2
global __ptext1351
__ptext1351:

;; *************** function _setVirtualLocation *****************
;; Defined at:
;;		line 575 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;  xV              1    wreg     unsigned char 
;;  yV              1    0[BANK0 ] unsigned char 
;;  dV              1    1[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  xV              1    3[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       2       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       4       0       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1351
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	575
	global	__size_of_setVirtualLocation
	__size_of_setVirtualLocation	equ	__end_of_setVirtualLocation-_setVirtualLocation
	
_setVirtualLocation:	
	opt	stack 4
; Regs used in _setVirtualLocation: [wreg]
;setVirtualLocation@xV stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(setVirtualLocation@xV)
	line	576
	
l10544:	
;main.c: 576: xVirtual = xV;
	movf	(setVirtualLocation@xV),w
	movwf	(??_setVirtualLocation+0)+0
	movf	(??_setVirtualLocation+0)+0,w
	movwf	(_xVirtual)
	line	577
;main.c: 577: yVirtual = yV;
	movf	(setVirtualLocation@yV),w
	movwf	(??_setVirtualLocation+0)+0
	movf	(??_setVirtualLocation+0)+0,w
	movwf	(_yVirtual)
	line	578
;main.c: 578: dVirtual = dV;
	movf	(setVirtualLocation@dV),w
	movwf	(??_setVirtualLocation+0)+0
	movf	(??_setVirtualLocation+0)+0,w
	movwf	(_dVirtual)
	line	579
	
l6881:	
	return
	opt stack 0
GLOBAL	__end_of_setVirtualLocation
	__end_of_setVirtualLocation:
;; =============== function _setVirtualLocation ends ============

	signat	_setVirtualLocation,12408
	global	_getCurrentY
psect	text1352,local,class=CODE,delta=2
global __ptext1352
__ptext1352:

;; *************** function _getCurrentY *****************
;; Defined at:
;;		line 570 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goForward
;;		_turnLeft90
;; This function uses a non-reentrant model
;;
psect	text1352
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	570
	global	__size_of_getCurrentY
	__size_of_getCurrentY	equ	__end_of_getCurrentY-_getCurrentY
	
_getCurrentY:	
	opt	stack 4
; Regs used in _getCurrentY: [wreg]
	line	571
	
l10540:	
;main.c: 571: return yCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	goto	l6878
	
l10542:	
	line	572
	
l6878:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentY
	__end_of_getCurrentY:
;; =============== function _getCurrentY ends ============

	signat	_getCurrentY,89
	global	_getCurrentX
psect	text1353,local,class=CODE,delta=2
global __ptext1353
__ptext1353:

;; *************** function _getCurrentX *****************
;; Defined at:
;;		line 565 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goForward
;;		_turnLeft90
;; This function uses a non-reentrant model
;;
psect	text1353
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	565
	global	__size_of_getCurrentX
	__size_of_getCurrentX	equ	__end_of_getCurrentX-_getCurrentX
	
_getCurrentX:	
	opt	stack 4
; Regs used in _getCurrentX: [wreg]
	line	566
	
l10536:	
;main.c: 566: return xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	goto	l6875
	
l10538:	
	line	567
	
l6875:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentX
	__end_of_getCurrentX:
;; =============== function _getCurrentX ends ============

	signat	_getCurrentX,89
	global	_updateOrientation
psect	text1354,local,class=CODE,delta=2
global __ptext1354
__ptext1354:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 294 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;  moved           1    wreg     enum E1111
;; Auto vars:     Size  Location     Type
;;  moved           1    1[BANK0 ] enum E1111
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goBackward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1354
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	294
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 4
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	295
	
l10530:	
;drive.c: 295: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	296
	
l10532:	
;drive.c: 296: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u3831
	goto	u3830
u3831:
	goto	l5888
u3830:
	line	297
	
l10534:	
;drive.c: 297: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l5888
	
l5887:	
	line	298
	
l5888:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_clearSuccessfulDrive
psect	text1355,local,class=CODE,delta=2
global __ptext1355
__ptext1355:

;; *************** function _clearSuccessfulDrive *****************
;; Defined at:
;;		line 190 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		None
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_frontWallCorrect
;; This function uses a non-reentrant model
;;
psect	text1355
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\drive.c"
	line	190
	global	__size_of_clearSuccessfulDrive
	__size_of_clearSuccessfulDrive	equ	__end_of_clearSuccessfulDrive-_clearSuccessfulDrive
	
_clearSuccessfulDrive:	
	opt	stack 4
; Regs used in _clearSuccessfulDrive: []
	line	191
	
l10528:	
;drive.c: 191: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	192
	
l5850:	
	return
	opt stack 0
GLOBAL	__end_of_clearSuccessfulDrive
	__end_of_clearSuccessfulDrive:
;; =============== function _clearSuccessfulDrive ends ============

	signat	_clearSuccessfulDrive,88
	global	_ser_init
psect	text1356,local,class=CODE,delta=2
global __ptext1356
__ptext1356:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text1356
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 5
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l10502:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l10504:	
;ser.c: 126: TRISC &= 0b10111111;
	movlw	(0BFh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_ser_init+0)+0
	movf	(??_ser_init+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	andwf	(135)^080h,f	;volatile
	line	127
	
l10506:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l10508:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l10510:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l10512:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l10514:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l10516:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l10518:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l10520:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l10522:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l10524:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l10526:	
;ser.c: 143: rxiptr=rxoptr=txiptr=txoptr=0;
	movlw	(0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(_txoptr)	;volatile
	movwf	(_txiptr)	;volatile
	movwf	(_rxoptr)	;volatile
	movwf	(??_ser_init+0)+0
	movf	(??_ser_init+0)+0,w
	movwf	(_rxiptr)	;volatile
	line	144
	
l3673:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_ser_isrx
psect	text1357,local,class=CODE,delta=2
global __ptext1357
__ptext1357:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_ser_getch
;; This function uses a non-reentrant model
;;
psect	text1357
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 3
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l10454:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u3761
	goto	u3760
u3761:
	goto	l10462
u3760:
	line	50
	
l10456:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l10458:	
;ser.c: 52: return 0;
	clrc
	
	goto	l3633
	
l10460:	
	goto	l3633
	line	53
	
l3632:	
	line	54
	
l10462:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u3771
	goto	u3770
u3771:
	goto	l10466
u3770:
	
l10464:	
	clrc
	
	goto	l3633
	
l10336:	
	
l10466:	
	setc
	
	goto	l3633
	
l10338:	
	goto	l3633
	
l10468:	
	line	55
	
l3633:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_getVictimZone
psect	text1358,local,class=CODE,delta=2
global __ptext1358
__ptext1358:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
;; Parameters:    Size  Location     Type
;;  victimX         1    wreg     unsigned char 
;;  victimY         1    0[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  victimX         1    2[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       1       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_lookForVictim
;; This function uses a non-reentrant model
;;
psect	text1358
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	157
	global	__size_of_getVictimZone
	__size_of_getVictimZone	equ	__end_of_getVictimZone-_getVictimZone
	
_getVictimZone:	
	opt	stack 5
; Regs used in _getVictimZone: [wreg-fsr0h+status,2+status,0]
;getVictimZone@victimX stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(getVictimZone@victimX)
	line	163
	
l10406:	
;map.c: 163: switch (victimX)
	goto	l10448
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l2900:	
	line	166
;map.c: 166: switch (victimY)
	goto	l10414
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l2902:	
	line	169
	
l10408:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	170
;map.c: 170: break;
	goto	l10450
	line	171
;map.c: 171: case 1:
	
l2904:	
	line	172
	
l10410:	
;map.c: 172: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	173
;map.c: 173: break;
	goto	l10450
	line	178
;map.c: 178: default:
	
l2905:	
	line	179
;map.c: 179: break;
	goto	l10450
	line	180
	
l10412:	
;map.c: 180: }
	goto	l10450
	line	166
	
l2901:	
	
l10414:	
	movf	(getVictimZone@victimY),w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 0 to 1
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           14     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             6     6 (fixed)
; spacedrange           10     9 (fixed)
; locatedrange           2     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l10408
	xorlw	1^0	; case 1
	skipnz
	goto	l10410
	goto	l10450
	opt asmopt_on

	line	180
	
l2903:	
	line	181
;map.c: 181: break;
	goto	l10450
	line	183
;map.c: 183: case 1:
	
l2907:	
	line	184
;map.c: 184: switch (victimY)
	goto	l10422
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l2909:	
	line	187
	
l10416:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	188
;map.c: 188: break;
	goto	l10450
	line	189
;map.c: 189: case 1:
	
l2911:	
	line	190
	
l10418:	
;map.c: 190: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	191
;map.c: 191: break;
	goto	l10450
	line	196
;map.c: 196: default:
	
l2912:	
	line	197
;map.c: 197: break;
	goto	l10450
	line	198
	
l10420:	
;map.c: 198: }
	goto	l10450
	line	184
	
l2908:	
	
l10422:	
	movf	(getVictimZone@victimY),w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 0 to 1
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           14     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             6     6 (fixed)
; spacedrange           10     9 (fixed)
; locatedrange           2     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l10416
	xorlw	1^0	; case 1
	skipnz
	goto	l10418
	goto	l10450
	opt asmopt_on

	line	198
	
l2910:	
	line	199
;map.c: 199: break;
	goto	l10450
	line	201
;map.c: 201: case 2:
	
l2913:	
	line	202
;map.c: 202: switch (victimY)
	goto	l10430
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l2915:	
	line	207
	
l10424:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	208
;map.c: 208: break;
	goto	l10450
	line	211
;map.c: 211: case 3:
	
l2917:	
	line	212
	
l10426:	
;map.c: 212: vicZone = 1;
	clrf	(_vicZone)
	bsf	status,0
	rlf	(_vicZone),f
	line	213
;map.c: 213: break;
	goto	l10450
	line	214
;map.c: 214: default:
	
l2918:	
	line	215
;map.c: 215: break;
	goto	l10450
	line	216
	
l10428:	
;map.c: 216: }
	goto	l10450
	line	202
	
l2914:	
	
l10430:	
	movf	(getVictimZone@victimY),w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 1 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           20    11 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l10424
	xorlw	3^1	; case 3
	skipnz
	goto	l10426
	goto	l10450
	opt asmopt_on

	line	216
	
l2916:	
	line	217
;map.c: 217: break;
	goto	l10450
	line	219
;map.c: 219: case 3:
	
l2919:	
	line	220
;map.c: 220: switch (victimY)
	goto	l10438
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l2921:	
	line	225
	
l10432:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	226
;map.c: 226: break;
	goto	l10450
	line	229
;map.c: 229: case 3:
	
l2923:	
	line	230
	
l10434:	
;map.c: 230: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	231
;map.c: 231: break;
	goto	l10450
	line	232
;map.c: 232: default:
	
l2924:	
	line	233
;map.c: 233: break;
	goto	l10450
	line	234
	
l10436:	
;map.c: 234: }
	goto	l10450
	line	220
	
l2920:	
	
l10438:	
	movf	(getVictimZone@victimY),w
	; Switch size 1, requested type "space"
; Number of cases is 2, Range of values is 1 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            7     4 (average)
; direct_byte           20    11 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	1^0	; case 1
	skipnz
	goto	l10432
	xorlw	3^1	; case 3
	skipnz
	goto	l10434
	goto	l10450
	opt asmopt_on

	line	234
	
l2922:	
	line	235
;map.c: 235: break;
	goto	l10450
	line	237
;map.c: 237: case 4:
	
l2925:	
	line	238
;map.c: 238: switch (victimY)
	goto	l10444
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l2927:	
	line	247
	
l10440:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	248
;map.c: 248: break;
	goto	l10450
	line	249
;map.c: 249: default:
	
l2929:	
	line	250
;map.c: 250: break;
	goto	l10450
	line	251
	
l10442:	
;map.c: 251: }
	goto	l10450
	line	238
	
l2926:	
	
l10444:	
	movf	(getVictimZone@victimY),w
	; Switch size 1, requested type "space"
; Number of cases is 1, Range of values is 3 to 3
; switch strategies available:
; Name         Instructions Cycles
; simple_byte            4     3 (average)
; direct_byte           14    11 (fixed)
; jumptable            263     9 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	3^0	; case 3
	skipnz
	goto	l10440
	goto	l10450
	opt asmopt_on

	line	251
	
l2928:	
	line	252
;map.c: 252: break;
	goto	l10450
	line	254
;map.c: 254: default:
	
l2930:	
	line	255
;map.c: 255: break;
	goto	l10450
	line	256
	
l10446:	
;map.c: 256: }
	goto	l10450
	line	163
	
l2899:	
	
l10448:	
	movf	(getVictimZone@victimX),w
	; Switch size 1, requested type "space"
; Number of cases is 5, Range of values is 0 to 4
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           16     9 (average)
; direct_byte           23     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable             9     6 (fixed)
; spacedrange           16     9 (fixed)
; locatedrange           5     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l10414
	xorlw	1^0	; case 1
	skipnz
	goto	l10422
	xorlw	2^1	; case 2
	skipnz
	goto	l10430
	xorlw	3^2	; case 3
	skipnz
	goto	l10438
	xorlw	4^3	; case 4
	skipnz
	goto	l10444
	goto	l10450
	opt asmopt_on

	line	256
	
l2906:	
	line	258
	
l10450:	
;map.c: 258: return vicZone;
	movf	(_vicZone),w
	goto	l2931
	
l10452:	
	line	259
	
l2931:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text1359,local,class=CODE,delta=2
global __ptext1359
__ptext1359:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_checkForFinalDestination
;; This function uses a non-reentrant model
;;
psect	text1359
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 5
; Regs used in _getFinalY: [wreg]
	line	153
	
l10402:	
;map.c: 153: return finalY;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	goto	l2896
	
l10404:	
	line	154
	
l2896:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text1360,local,class=CODE,delta=2
global __ptext1360
__ptext1360:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      unsigned char 
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_checkForFinalDestination
;; This function uses a non-reentrant model
;;
psect	text1360
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 5
; Regs used in _getFinalX: [wreg]
	line	148
	
l10398:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l2893
	
l10400:	
	line	149
	
l2893:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_putch
psect	text1361,local,class=CODE,delta=2
global __ptext1361
__ptext1361:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
;; Parameters:    Size  Location     Type
;;  c               1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  c               1    1[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       2       0       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_EEPROMToSerial
;;		_ser_putArr
;;		_play_iCreate_song
;;		_drive
;;		_driveForDistance
;;		_waitFor
;;		_initIRobot
;;		_lookForVictim
;;		_ser_puts
;;		_ser_puts2
;;		_ser_puthex
;; This function uses a non-reentrant model
;;
psect	text1361
	file	"C:\Documents and Settings\Administrator\Desktop\30 MAY WORKING FILE\ser.c"
	line	81
	global	__size_of_ser_putch
	__size_of_ser_putch	equ	__end_of_ser_putch-_ser_putch
	
_ser_putch:	
	opt	stack 4
; Regs used in _ser_putch: [wreg-fsr0h+status,2+status,0]
;ser_putch@c stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(ser_putch@c)
	line	82
	
l10370:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l10372
	
l3649:	
	line	83
;ser.c: 83: continue;
	goto	l10372
	
l3648:	
	line	82
	
l10372:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u3731
	goto	u3730
u3731:
	goto	l10372
u3730:
	
l3650:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l10374:	
;ser.c: 85: txfifo[txiptr] = c;
	movf	(ser_putch@c),w
	movwf	(??_ser_putch+0)+0
	movf	(_txiptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	movf	(??_ser_putch+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	line	86
	
l10376:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l10378:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l10380:	
;ser.c: 88: GIE=1;
	bsf	(95/8),(95)&7
	line	89
	
l3651:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_initEEPROMMode
psect	text1362,local,class=CODE,delta=2
global __ptext1362
__ptext1362:

;; *************** function _initEEPROMMode *****************
;; Defined at:
;;		line 21 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_writeEEPROM
;;		_readEEPROM
;; This function uses a non-reentrant model
;;
psect	text1362
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	21
	global	__size_of_initEEPROMMode
	__size_of_initEEPROMMode	equ	__end_of_initEEPROMMode-_initEEPROMMode
	
_initEEPROMMode:	
	opt	stack 3
; Regs used in _initEEPROMMode: [wreg+status,2+status,0]
	line	22
	
l10348:	
;eeprom.c: 22: PORTC &= 0b11111100;
	movlw	(0FCh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_initEEPROMMode+0)+0
	movf	(??_initEEPROMMode+0)+0,w
	andwf	(7),f	;volatile
	line	23
	
l10350:	
;eeprom.c: 23: PORTC |= 0b00000010;
	bsf	(7)+(1/8),(1)&7	;volatile
	line	24
	
l1407:	
	return
	opt stack 0
GLOBAL	__end_of_initEEPROMMode
	__end_of_initEEPROMMode:
;; =============== function _initEEPROMMode ends ============

	signat	_initEEPROMMode,88
	global	_writeSPIByte
psect	text1363,local,class=CODE,delta=2
global __ptext1363
__ptext1363:

;; *************** function _writeSPIByte *****************
;; Defined at:
;;		line 14 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1    0[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_writeEEPROM
;;		_readEEPROM
;; This function uses a non-reentrant model
;;
psect	text1363
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\eeprom.c"
	line	14
	global	__size_of_writeSPIByte
	__size_of_writeSPIByte	equ	__end_of_writeSPIByte-_writeSPIByte
	
_writeSPIByte:	
	opt	stack 3
; Regs used in _writeSPIByte: [wreg]
;writeSPIByte@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(writeSPIByte@data)
	line	15
	
l10344:	
;eeprom.c: 15: SSPIF = 0;
	bcf	(99/8),(99)&7
	line	16
	
l10346:	
;eeprom.c: 16: SSPBUF = data;
	movf	(writeSPIByte@data),w
	movwf	(19)	;volatile
	line	17
;eeprom.c: 17: while(!SSPIF);
	goto	l1401
	
l1402:	
	
l1401:	
	btfss	(99/8),(99)&7
	goto	u3701
	goto	u3700
u3701:
	goto	l1401
u3700:
	goto	l1404
	
l1403:	
	line	18
	
l1404:	
	return
	opt stack 0
GLOBAL	__end_of_writeSPIByte
	__end_of_writeSPIByte:
;; =============== function _writeSPIByte ends ============

	signat	_writeSPIByte,4216
	global	_isr1
psect	text1364,local,class=CODE,delta=2
global __ptext1364
__ptext1364:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 70 in file "C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;		None               void
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:         10       0       0       0       0
;;      Totals:        10       0       0       0       0
;;Total ram usage:       10 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		Interrupt level 1
;; This function uses a non-reentrant model
;;
psect	text1364
	file	"C:\Users\11014065\Desktop\30 MAY WORKING FILE\main.c"
	line	70
	global	__size_of_isr1
	__size_of_isr1	equ	__end_of_isr1-_isr1
	
_isr1:	
	opt	stack 1
; Regs used in _isr1: [wreg-fsr0h+status,2+status,0]
psect	intentry,class=CODE,delta=2
global __pintentry
__pintentry:
global interrupt_function
interrupt_function:
	global saved_w
	saved_w	set	btemp+0
	movwf	saved_w
	swapf	status,w
	movwf	(??_isr1+6)
	movf	fsr0,w
	movwf	(??_isr1+7)
	movf	pclath,w
	movwf	(??_isr1+8)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	btemp+1,w
	movwf	(??_isr1+9)
	ljmp	_isr1
psect	text1364
	line	72
	
i1l10578:	
;main.c: 72: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u388_21
	goto	u388_20
u388_21:
	goto	i1l6741
u388_20:
	line	74
	
i1l10580:	
;main.c: 73: {
;main.c: 74: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	75
	
i1l10582:	
;main.c: 75: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	88
	
i1l10584:	
;main.c: 88: if(!RB0)
	btfsc	(48/8),(48)&7
	goto	u389_21
	goto	u389_20
u389_21:
	goto	i1l6731
u389_20:
	line	90
	
i1l10586:	
;main.c: 89: {
;main.c: 90: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	91
	
i1l10588:	
;main.c: 91: if(start.debounceCount >= 10 & start.released)
	movf	0+(_start)+01h,w
	movwf	(??_isr1+0)+0
	clrf	(??_isr1+0)+0+1
	movlw	(0Ah)
	subwf	0+(_start)+02h,w
	movlw	0
	skipnc
	movlw	1
	movwf	(??_isr1+2)+0
	clrf	(??_isr1+2)+0+1
	movf	0+(??_isr1+0)+0,w
	andwf	0+(??_isr1+2)+0,w
	movwf	(??_isr1+4)+0
	movf	1+(??_isr1+0)+0,w
	andwf	1+(??_isr1+2)+0,w
	movwf	1+(??_isr1+4)+0
	movf	1+(??_isr1+4)+0,w
	iorwf	0+(??_isr1+4)+0,w
	skipnz
	goto	u390_21
	goto	u390_20
u390_21:
	goto	i1l10596
u390_20:
	line	93
	
i1l10590:	
;main.c: 92: {
;main.c: 93: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	94
	
i1l10592:	
;main.c: 94: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l10596
	line	95
	
i1l6732:	
	line	96
;main.c: 95: }
;main.c: 96: }
	goto	i1l10596
	line	97
	
i1l6731:	
	line	99
;main.c: 97: else
;main.c: 98: {
;main.c: 99: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	100
	
i1l10594:	
;main.c: 100: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l10596
	line	101
	
i1l6733:	
	line	103
	
i1l10596:	
;main.c: 101: }
;main.c: 103: if(!RB1)
	btfsc	(49/8),(49)&7
	goto	u391_21
	goto	u391_20
u391_21:
	goto	i1l6734
u391_20:
	line	105
	
i1l10598:	
;main.c: 104: {
;main.c: 105: eepromSerial.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_eepromSerial)+02h,f
	line	106
	
i1l10600:	
;main.c: 106: if(eepromSerial.debounceCount >= 10 & eepromSerial.released)
	movf	0+(_eepromSerial)+01h,w
	movwf	(??_isr1+0)+0
	clrf	(??_isr1+0)+0+1
	movlw	(0Ah)
	subwf	0+(_eepromSerial)+02h,w
	movlw	0
	skipnc
	movlw	1
	movwf	(??_isr1+2)+0
	clrf	(??_isr1+2)+0+1
	movf	0+(??_isr1+0)+0,w
	andwf	0+(??_isr1+2)+0,w
	movwf	(??_isr1+4)+0
	movf	1+(??_isr1+0)+0,w
	andwf	1+(??_isr1+2)+0,w
	movwf	1+(??_isr1+4)+0
	movf	1+(??_isr1+4)+0,w
	iorwf	0+(??_isr1+4)+0,w
	skipnz
	goto	u392_21
	goto	u392_20
u392_21:
	goto	i1l10608
u392_20:
	line	108
	
i1l10602:	
;main.c: 107: {
;main.c: 108: eepromSerial.pressed = 1;
	clrf	(_eepromSerial)
	bsf	status,0
	rlf	(_eepromSerial),f
	line	109
	
i1l10604:	
;main.c: 109: eepromSerial.released = 0;
	clrf	0+(_eepromSerial)+01h
	goto	i1l10608
	line	110
	
i1l6735:	
	line	111
;main.c: 110: }
;main.c: 111: }
	goto	i1l10608
	line	112
	
i1l6734:	
	line	114
;main.c: 112: else
;main.c: 113: {
;main.c: 114: eepromSerial.debounceCount = 0;
	clrf	0+(_eepromSerial)+02h
	line	115
	
i1l10606:	
;main.c: 115: eepromSerial.released = 1;
	clrf	0+(_eepromSerial)+01h
	bsf	status,0
	rlf	0+(_eepromSerial)+01h,f
	goto	i1l10608
	line	116
	
i1l6736:	
	line	117
	
i1l10608:	
;main.c: 116: }
;main.c: 117: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u393_21
	goto	u393_20
u393_21:
	goto	i1l10618
u393_20:
	
i1l10610:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	
i1l10612:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l10614:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u394_21
	goto	u394_20
u394_21:
	goto	i1l10618
u394_20:
	
i1l10616:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l10618
	
i1l6738:	
	goto	i1l10618
	
i1l6737:	
	
i1l10618:	
	btfss	(100/8),(100)&7
	goto	u395_21
	goto	u395_20
u395_21:
	goto	i1l6741
u395_20:
	
i1l10620:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u396_21
	goto	u396_20
u396_21:
	goto	i1l6741
u396_20:
	
i1l10622:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	movwf	(25)	;volatile
	
i1l10624:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l10626:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l10628:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u397_21
	goto	u397_20
u397_21:
	goto	i1l6741
u397_20:
	
i1l10630:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l6741
	
i1l6740:	
	goto	i1l6741
	
i1l6739:	
	goto	i1l6741
	line	118
	
i1l6730:	
	line	119
	
i1l6741:	
	movf	(??_isr1+9),w
	bcf	status, 5	;RP0=0, select bank0
	movwf	btemp+1
	movf	(??_isr1+8),w
	movwf	pclath
	movf	(??_isr1+7),w
	movwf	fsr0
	swapf	(??_isr1+6)^0FFFFFF80h,w
	movwf	status
	swapf	saved_w,f
	swapf	saved_w,w
	retfie
	opt stack 0
GLOBAL	__end_of_isr1
	__end_of_isr1:
;; =============== function _isr1 ends ============

	signat	_isr1,88
psect	text1365,local,class=CODE,delta=2
global __ptext1365
__ptext1365:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

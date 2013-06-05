opt subtitle "HI-TECH Software Omniscient Code Generator (Lite mode) build 7503"

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
# 21 "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_EEPROMToSerial
	FNCALL	_main,_findWalls
	FNCALL	_main,_turnAround
	FNCALL	_main,_turnLeft90
	FNCALL	_main,_turnRight90
	FNCALL	_main,_lcd_write_data
	FNCALL	_main,_play_iCreate_song
	FNCALL	_main,_rotateIR
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_goParallel
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
	FNCALL	_findWalls,_lcd_set_cursor
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_findWalls,_rotateIR
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
	FNCALL	_goParallel,_readIR
	FNCALL	_goParallel,_rotateIR
	FNCALL	_goParallel,___lbtoft
	FNCALL	_goParallel,___ftmul
	FNCALL	_goParallel,___ftadd
	FNCALL	_goParallel,___fttol
	FNCALL	_goParallel,_drive
	FNCALL	_goParallel,_waitFor
	FNCALL	_findWall,_readIR
	FNCALL	_frontWallCorrect,_rotateIR
	FNCALL	_frontWallCorrect,_readIR
	FNCALL	_frontWallCorrect,_drive
	FNCALL	_driveForDistance,_drive
	FNCALL	_driveForDistance,_ser_putch
	FNCALL	_driveForDistance,_ser_getch
	FNCALL	_driveForDistance,_goReverse
	FNCALL	_driveForDistance,_turnRight90
	FNCALL	_driveForDistance,_updateOrientation
	FNCALL	_driveForDistance,_turnLeft90
	FNCALL	_driveForDistance,_getCurrentY
	FNCALL	_driveForDistance,_getCurrentX
	FNCALL	_driveForDistance,_findFinalDestination
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
	FNCALL	___lbtoft,___ftpack
	FNCALL	___ftmul,___ftpack
	FNCALL	___ftadd,___ftpack
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
	FNCALL	_isr1,___lwmod
	FNCALL	intlevel1,_isr1
	global	intlevel1
	FNROOT	intlevel1
	global	_finalX
	global	_xCoord
	global	_yCoord
	global	_finalY
	global	_somethingInTheWay
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
	global	_beep
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	7

;initializer for _finalX
	retlw	03h
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	51

;initializer for _xCoord
	retlw	01h
	line	52

;initializer for _yCoord
	retlw	03h
psect	idataBANK1,class=CODE,space=0,delta=2
global __pidataBANK1
__pidataBANK1:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	8

;initializer for _finalY
	retlw	01h
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	14

;initializer for _somethingInTheWay
	retlw	02h
psect	idataBANK2,class=CODE,space=0,delta=2
global __pidataBANK2
__pidataBANK2:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
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
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
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
psect	idataBANK1
	line	14

;initializer for _beep
	retlw	08Ch
	retlw	05h
	retlw	01h
	retlw	048h
	retlw	04h
	global	_RTC_Counter
	global	_addressCount
	global	_addressCurrent
	global	_currentOrientation
	global	_lastMove
	global	_victimZone
	global	_rxiptr
	global	_rxoptr
	global	_txiptr
	global	_txoptr
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
	global	_eeprom
	global	_start
	global	_closestObject
	global	_node
	global	_ser_tmp
	global	_stepPosition
	global	_stepsToPerpendicular
	global	_vicZone
	global	_wayWent
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
	file	"COMPETITIONv0.7.as"
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

_rxoptr:
       ds      1

_txiptr:
       ds      1

_txoptr:
       ds      1

psect	bssBANK0,class=BANK0,space=1
global __pbssBANK0
__pbssBANK0:
_RTC_Counter:
       ds      2

_addressCount:
       ds      1

_addressCurrent:
       ds      1

_currentOrientation:
       ds      1

_lastMove:
       ds      1

_victimZone:
       ds      1

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	7
_finalX:
       ds      1

psect	dataBANK0
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	51
_xCoord:
       ds      1

psect	dataBANK0
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	52
_yCoord:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_rxfifo:
       ds      16

_txfifo:
       ds      16

_eeprom:
       ds      3

_start:
       ds      3

_closestObject:
       ds      2

_node:
       ds      1

_ser_tmp:
       ds      1

_stepPosition:
       ds      1

_stepsToPerpendicular:
       ds      1

_vicZone:
       ds      1

_wayWent:
       ds      1

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	8
_finalY:
       ds      1

psect	dataBANK1
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	14
_somethingInTheWay:
       ds      1

psect	dataBANK1
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
_beep:
       ds      5

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	12
_finalCountdown:
       ds      27

psect	dataBANK3
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK3
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK2,class=BANK2,space=1
global __pdataBANK2
__pdataBANK2:
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	11
_lookingForU2:
       ds      29

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
	clrf	((__pbssCOMMON)+2)&07Fh
	clrf	((__pbssCOMMON)+3)&07Fh
; Clear objects allocated to BANK0
psect cinit,class=CODE,delta=2
	clrf	((__pbssBANK0)+0)&07Fh
	clrf	((__pbssBANK0)+1)&07Fh
	clrf	((__pbssBANK0)+2)&07Fh
	clrf	((__pbssBANK0)+3)&07Fh
	clrf	((__pbssBANK0)+4)&07Fh
	clrf	((__pbssBANK0)+5)&07Fh
	clrf	((__pbssBANK0)+6)&07Fh
; Clear objects allocated to BANK1
psect cinit,class=CODE,delta=2
	bcf	status, 7	;select IRP bank0
	movlw	low(__pbssBANK1)
	movwf	fsr
	movlw	low((__pbssBANK1)+02Eh)
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
	movlw low(__pdataBANK3+73)
	movwf btemp-1,f
	movlw high(__pidataBANK3)
	movwf btemp,f
	movlw low(__pidataBANK3)
	movwf btemp+1,f
	movlw low(__pdataBANK3)
	movwf fsr,f
	fcall init_ram
; Initialize objects allocated to BANK2
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK2
	movlw low(__pdataBANK2+29)
	movwf btemp-1,f
	movlw high(__pidataBANK2)
	movwf btemp,f
	movlw low(__pidataBANK2)
	movwf btemp+1,f
	movlw low(__pdataBANK2)
	movwf fsr,f
	fcall init_ram
; Initialize objects allocated to BANK1
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK1
	bcf	status, 7	;select IRP bank0
	movlw low(__pdataBANK1+7)
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
	movlw low(__pdataBANK0+3)
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
psect	cstackBANK1,class=BANK1,space=1
global __pcstackBANK1
__pcstackBANK1:
	global	?___ftadd
?___ftadd:	; 3 bytes @ 0x0
	global	___ftadd@f1
___ftadd@f1:	; 3 bytes @ 0x0
	ds	3
	global	___ftadd@f2
___ftadd@f2:	; 3 bytes @ 0x3
	ds	3
	global	___ftadd@sign
___ftadd@sign:	; 1 bytes @ 0x6
	ds	1
	global	___ftadd@exp2
___ftadd@exp2:	; 1 bytes @ 0x7
	ds	1
	global	___ftadd@exp1
___ftadd@exp1:	; 1 bytes @ 0x8
	ds	1
	global	??_goParallel
??_goParallel:	; 0 bytes @ 0x9
	ds	2
	global	goParallel@stepsToWall
goParallel@stepsToWall:	; 1 bytes @ 0xB
	ds	1
	global	goParallel@shortestDistance
goParallel@shortestDistance:	; 2 bytes @ 0xC
	ds	2
	global	goParallel@angleHighByte
goParallel@angleHighByte:	; 1 bytes @ 0xE
	ds	1
	global	goParallel@angleLowByte
goParallel@angleLowByte:	; 1 bytes @ 0xF
	ds	1
	global	goParallel@distance
goParallel@distance:	; 2 bytes @ 0x10
	ds	2
	global	goParallel@step
goParallel@step:	; 2 bytes @ 0x12
	ds	2
	global	goParallel@angleParallelToWall
goParallel@angleParallelToWall:	; 2 bytes @ 0x14
	ds	2
	global	??_main
??_main:	; 0 bytes @ 0x16
	ds	1
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_lcd_write_string
?_lcd_write_string:	; 0 bytes @ 0x0
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
	global	?_goReverse
?_goReverse:	; 0 bytes @ 0x0
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
	global	?_init_adc
?_init_adc:	; 0 bytes @ 0x0
	global	?_writeSPIByte
?_writeSPIByte:	; 0 bytes @ 0x0
	global	?_initEEPROMMode
?_initEEPROMMode:	; 0 bytes @ 0x0
	global	?_addNewData
?_addNewData:	; 0 bytes @ 0x0
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
	global	?_init
?_init:	; 0 bytes @ 0x0
	global	?_checkForFinalDestination
?_checkForFinalDestination:	; 0 bytes @ 0x0
	global	?_lookForVictim
?_lookForVictim:	; 0 bytes @ 0x0
	global	?_findWalls
?_findWalls:	; 0 bytes @ 0x0
	global	?_goParallel
?_goParallel:	; 0 bytes @ 0x0
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
	global	?___lwmod
?___lwmod:	; 2 bytes @ 0x0
	global	___lwmod@divisor
___lwmod@divisor:	; 2 bytes @ 0x0
	ds	2
	global	___lwmod@dividend
___lwmod@dividend:	; 2 bytes @ 0x2
	ds	2
	global	??___lwmod
??___lwmod:	; 0 bytes @ 0x4
	ds	1
	global	___lwmod@counter
___lwmod@counter:	; 1 bytes @ 0x5
	ds	1
psect	cstackBANK0,class=BANK0,space=1
global __pcstackBANK0
__pcstackBANK0:
	global	??_isr1
??_isr1:	; 0 bytes @ 0x0
	ds	10
	global	??_ser_putch
??_ser_putch:	; 0 bytes @ 0xA
	global	??_updateOrientation
??_updateOrientation:	; 0 bytes @ 0xA
	global	??_getCurrentX
??_getCurrentX:	; 0 bytes @ 0xA
	global	??_getCurrentY
??_getCurrentY:	; 0 bytes @ 0xA
	global	??_init_adc
??_init_adc:	; 0 bytes @ 0xA
	global	??_writeSPIByte
??_writeSPIByte:	; 0 bytes @ 0xA
	global	??_initEEPROMMode
??_initEEPROMMode:	; 0 bytes @ 0xA
	global	??_lcd_write_control
??_lcd_write_control:	; 0 bytes @ 0xA
	global	??_lcd_write_data
??_lcd_write_data:	; 0 bytes @ 0xA
	global	??_getFinalX
??_getFinalX:	; 0 bytes @ 0xA
	global	??_getFinalY
??_getFinalY:	; 0 bytes @ 0xA
	global	??_ser_isrx
??_ser_isrx:	; 0 bytes @ 0xA
	global	??_ser_getch
??_ser_getch:	; 0 bytes @ 0xA
	global	??_ser_init
??_ser_init:	; 0 bytes @ 0xA
	global	?_rotateIR
?_rotateIR:	; 0 bytes @ 0xA
	global	??_getOrientation
??_getOrientation:	; 0 bytes @ 0xA
	global	??_getSomethingInTheWay
??_getSomethingInTheWay:	; 0 bytes @ 0xA
	global	??_getSuccessfulDrive
??_getSuccessfulDrive:	; 0 bytes @ 0xA
	global	??_updateNode
??_updateNode:	; 0 bytes @ 0xA
	global	?_getVictimZone
?_getVictimZone:	; 1 bytes @ 0xA
	global	?___wmul
?___wmul:	; 2 bytes @ 0xA
	global	?___ftpack
?___ftpack:	; 3 bytes @ 0xA
	global	writeSPIByte@data
writeSPIByte@data:	; 1 bytes @ 0xA
	global	getVictimZone@victimY
getVictimZone@victimY:	; 1 bytes @ 0xA
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	global	___ftpack@arg
___ftpack@arg:	; 3 bytes @ 0xA
	ds	1
	global	?_writeEEPROM
?_writeEEPROM:	; 0 bytes @ 0xB
	global	??_getVictimZone
??_getVictimZone:	; 0 bytes @ 0xB
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0xB
	global	?_readEEPROM
?_readEEPROM:	; 1 bytes @ 0xB
	global	writeEEPROM@addressH
writeEEPROM@addressH:	; 1 bytes @ 0xB
	global	readEEPROM@addressL
readEEPROM@addressL:	; 1 bytes @ 0xB
	global	ser_getch@c
ser_getch@c:	; 1 bytes @ 0xB
	global	ser_putch@c
ser_putch@c:	; 1 bytes @ 0xB
	global	updateOrientation@moved
updateOrientation@moved:	; 1 bytes @ 0xB
	ds	1
	global	?_waitFor
?_waitFor:	; 0 bytes @ 0xC
	global	??_initIRobot
??_initIRobot:	; 0 bytes @ 0xC
	global	??_readEEPROM
??_readEEPROM:	; 0 bytes @ 0xC
	global	?_ser_putArr
?_ser_putArr:	; 0 bytes @ 0xC
	global	??_play_iCreate_song
??_play_iCreate_song:	; 0 bytes @ 0xC
	global	?_drive
?_drive:	; 0 bytes @ 0xC
	global	writeEEPROM@addressL
writeEEPROM@addressL:	; 1 bytes @ 0xC
	global	lcd_write_control@databyte
lcd_write_control@databyte:	; 1 bytes @ 0xC
	global	lcd_write_data@databyte
lcd_write_data@databyte:	; 1 bytes @ 0xC
	global	getVictimZone@victimX
getVictimZone@victimX:	; 1 bytes @ 0xC
	global	play_iCreate_song@song
play_iCreate_song@song:	; 1 bytes @ 0xC
	global	drive@lowByteSpeed
drive@lowByteSpeed:	; 1 bytes @ 0xC
	global	waitFor@highByte
waitFor@highByte:	; 1 bytes @ 0xC
	global	ser_putArr@array
ser_putArr@array:	; 2 bytes @ 0xC
	global	___wmul@multiplicand
___wmul@multiplicand:	; 2 bytes @ 0xC
	ds	1
	global	??_lcd_write_string
??_lcd_write_string:	; 0 bytes @ 0xD
	global	??_writeEEPROM
??_writeEEPROM:	; 0 bytes @ 0xD
	global	??_lcd_set_cursor
??_lcd_set_cursor:	; 0 bytes @ 0xD
	global	??_lcd_write_1_digit_bcd
??_lcd_write_1_digit_bcd:	; 0 bytes @ 0xD
	global	??_lcd_init
??_lcd_init:	; 0 bytes @ 0xD
	global	lcd_set_cursor@address
lcd_set_cursor@address:	; 1 bytes @ 0xD
	global	lcd_write_1_digit_bcd@data
lcd_write_1_digit_bcd@data:	; 1 bytes @ 0xD
	global	rotateIR@steps
rotateIR@steps:	; 1 bytes @ 0xD
	global	drive@highByteRadius
drive@highByteRadius:	; 1 bytes @ 0xD
	global	waitFor@lowByte
waitFor@lowByte:	; 1 bytes @ 0xD
	global	___ftpack@exp
___ftpack@exp:	; 1 bytes @ 0xD
	ds	1
	global	??_waitFor
??_waitFor:	; 0 bytes @ 0xE
	global	?_findFinalDestination
?_findFinalDestination:	; 0 bytes @ 0xE
	global	??_checkForFinalDestination
??_checkForFinalDestination:	; 0 bytes @ 0xE
	global	??_lookForVictim
??_lookForVictim:	; 0 bytes @ 0xE
	global	??_updateLocation
??_updateLocation:	; 0 bytes @ 0xE
	global	??___wmul
??___wmul:	; 0 bytes @ 0xE
	global	lcd_write_string@s
lcd_write_string@s:	; 1 bytes @ 0xE
	global	findFinalDestination@virtualWallY
findFinalDestination@virtualWallY:	; 1 bytes @ 0xE
	global	rotateIR@stepNum
rotateIR@stepNum:	; 1 bytes @ 0xE
	global	drive@lowByteRadius
drive@lowByteRadius:	; 1 bytes @ 0xE
	global	___ftpack@sign
___ftpack@sign:	; 1 bytes @ 0xE
	global	ser_putArr@length
ser_putArr@length:	; 2 bytes @ 0xE
	global	___wmul@product
___wmul@product:	; 2 bytes @ 0xE
	ds	1
	global	??___ftpack
??___ftpack:	; 0 bytes @ 0xF
	global	??_drive
??_drive:	; 0 bytes @ 0xF
	global	readEEPROM@addressH
readEEPROM@addressH:	; 1 bytes @ 0xF
	global	findFinalDestination@robotOrientation
findFinalDestination@robotOrientation:	; 1 bytes @ 0xF
	ds	1
	global	??_findFinalDestination
??_findFinalDestination:	; 0 bytes @ 0x10
	global	??_ser_putArr
??_ser_putArr:	; 0 bytes @ 0x10
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x10
	global	writeEEPROM@data
writeEEPROM@data:	; 1 bytes @ 0x10
	global	readEEPROM@returnData
readEEPROM@returnData:	; 1 bytes @ 0x10
	global	lookForVictim@victim
lookForVictim@victim:	; 1 bytes @ 0x10
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x10
	ds	1
	global	??_addNewData
??_addNewData:	; 0 bytes @ 0x11
	global	??_EEPROMToSerial
??_EEPROMToSerial:	; 0 bytes @ 0x11
	global	findFinalDestination@virtualWallX
findFinalDestination@virtualWallX:	; 1 bytes @ 0x11
	global	waitFor@type
waitFor@type:	; 1 bytes @ 0x11
	ds	1
	global	?___fttol
?___fttol:	; 4 bytes @ 0x12
	global	addNewData@data
addNewData@data:	; 1 bytes @ 0x12
	global	drive@highByteSpeed
drive@highByteSpeed:	; 1 bytes @ 0x12
	global	___awdiv@dividend
___awdiv@dividend:	; 2 bytes @ 0x12
	global	___fttol@f1
___fttol@f1:	; 3 bytes @ 0x12
	ds	1
	global	??_goReverse
??_goReverse:	; 0 bytes @ 0x13
	global	??_turnRight90
??_turnRight90:	; 0 bytes @ 0x13
	global	??_turnLeft90
??_turnLeft90:	; 0 bytes @ 0x13
	global	??_turnAround
??_turnAround:	; 0 bytes @ 0x13
	global	?_updateMapData
?_updateMapData:	; 0 bytes @ 0x13
	global	??_checkIfHome
??_checkIfHome:	; 0 bytes @ 0x13
	global	updateMapData@virtualS
updateMapData@virtualS:	; 1 bytes @ 0x13
	global	ser_putArr@i
ser_putArr@i:	; 2 bytes @ 0x13
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0x14
	global	updateMapData@virtualE
updateMapData@virtualE:	; 1 bytes @ 0x14
	global	EEPROMToSerial@transferDone
EEPROMToSerial@transferDone:	; 1 bytes @ 0x14
	ds	1
	global	??_initSongs
??_initSongs:	; 0 bytes @ 0x15
	global	??_init
??_init:	; 0 bytes @ 0x15
	global	updateMapData@virtualN
updateMapData@virtualN:	; 1 bytes @ 0x15
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0x15
	ds	1
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0x16
	global	??___fttol
??___fttol:	; 0 bytes @ 0x16
	global	updateMapData@victim
updateMapData@victim:	; 1 bytes @ 0x16
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0x16
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0x16
	ds	1
	global	updateMapData@move
updateMapData@move:	; 1 bytes @ 0x17
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0x17
	ds	1
	global	??_updateMapData
??_updateMapData:	; 0 bytes @ 0x18
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0x18
	ds	1
	global	?_adc_read
?_adc_read:	; 2 bytes @ 0x19
	ds	1
	global	updateMapData@virtualW
updateMapData@virtualW:	; 1 bytes @ 0x1A
	global	___fttol@sign1
___fttol@sign1:	; 1 bytes @ 0x1A
	global	driveForDistance@deltaDistance
driveForDistance@deltaDistance:	; 2 bytes @ 0x1A
	ds	1
	global	??_adc_read
??_adc_read:	; 0 bytes @ 0x1B
	global	updateMapData@completeData
updateMapData@completeData:	; 1 bytes @ 0x1B
	global	___fttol@lval
___fttol@lval:	; 4 bytes @ 0x1B
	ds	1
	global	driveForDistance@distance
driveForDistance@distance:	; 2 bytes @ 0x1C
	ds	2
	global	driveForDistance@high
driveForDistance@high:	; 1 bytes @ 0x1E
	ds	1
	global	driveForDistance@low
driveForDistance@low:	; 1 bytes @ 0x1F
	global	___fttol@exp1
___fttol@exp1:	; 1 bytes @ 0x1F
	global	adc_read@adc_value
adc_read@adc_value:	; 2 bytes @ 0x1F
	ds	1
	global	?___lbtoft
?___lbtoft:	; 3 bytes @ 0x20
	global	driveForDistance@virtualWall
driveForDistance@virtualWall:	; 1 bytes @ 0x20
	ds	1
	global	?_convert
?_convert:	; 2 bytes @ 0x21
	global	driveForDistance@cliff
driveForDistance@cliff:	; 1 bytes @ 0x21
	global	convert@adc_value
convert@adc_value:	; 2 bytes @ 0x21
	ds	1
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x22
	global	??_goForward
??_goForward:	; 0 bytes @ 0x22
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x22
	global	??_goRight
??_goRight:	; 0 bytes @ 0x22
	ds	1
	global	??_convert
??_convert:	; 0 bytes @ 0x23
	global	??_goToNextCell
??_goToNextCell:	; 0 bytes @ 0x23
	global	??___lbtoft
??___lbtoft:	; 0 bytes @ 0x23
	ds	2
	global	?_adc_read_channel
?_adc_read_channel:	; 2 bytes @ 0x25
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x27
	global	___lbtoft@c
___lbtoft@c:	; 1 bytes @ 0x27
	ds	1
	global	?___ftmul
?___ftmul:	; 3 bytes @ 0x28
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x28
	global	___ftmul@f1
___ftmul@f1:	; 3 bytes @ 0x28
	ds	1
	global	?_readIR
?_readIR:	; 2 bytes @ 0x29
	ds	2
	global	??_readIR
??_readIR:	; 0 bytes @ 0x2B
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x2B
	global	___ftmul@f2
___ftmul@f2:	; 3 bytes @ 0x2B
	ds	2
	global	??_findWall
??_findWall:	; 0 bytes @ 0x2D
	global	??_frontWallCorrect
??_frontWallCorrect:	; 0 bytes @ 0x2D
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x2D
	ds	1
	global	??___ftmul
??___ftmul:	; 0 bytes @ 0x2E
	ds	1
	global	frontWallCorrect@distToWall
frontWallCorrect@distToWall:	; 2 bytes @ 0x2F
	ds	3
	global	___ftmul@exp
___ftmul@exp:	; 1 bytes @ 0x32
	ds	1
	global	___ftmul@f3_as_product
___ftmul@f3_as_product:	; 3 bytes @ 0x33
	ds	3
	global	___ftmul@cntr
___ftmul@cntr:	; 1 bytes @ 0x36
	ds	1
	global	___ftmul@sign
___ftmul@sign:	; 1 bytes @ 0x37
	ds	1
	global	??___ftadd
??___ftadd:	; 0 bytes @ 0x38
	ds	4
;;Data sizes: Strings 63, constant 0, data 112, bss 57, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     60      70
;; BANK1           80     23      76
;; BANK3           96      0      73
;; BANK2           96      0      29

;;
;; Pointer list with targets:

;; ?___lbtoft	float  size(1) Largest target is 0
;;
;; ?___ftpack	float  size(1) Largest target is 0
;;
;; ?___fttol	long  size(1) Largest target is 0
;;
;; ?___ftmul	float  size(1) Largest target is 0
;;
;; ?___ftadd	float  size(1) Largest target is 0
;;
;; ?___lwmod	unsigned int  size(1) Largest target is 0
;;
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
;;		 -> beep(BANK1[5]), champions(BANK3[21]), lookingForU2(BANK2[29]), superMarioBros(BANK3[25]), 
;;		 -> finalCountdown(BANK3[27]), 
;;
;; lcd_write_string@s	PTR const unsigned char  size(1) Largest target is 17
;;		 -> STR_4(CODE[17]), STR_3(CODE[17]), STR_2(CODE[14]), STR_1(CODE[15]), 
;;


;;
;; Critical Paths under _main in COMMON
;;
;;   None.
;;
;; Critical Paths under _isr1 in COMMON
;;
;;   _isr1->___lwmod
;;
;; Critical Paths under _main in BANK0
;;
;;   _goToNextCell->_goRight
;;   _goToNextCell->_goBackward
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _goParallel->___ftadd
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
;;   ___lbtoft->___fttol
;;   ___ftmul->___lbtoft
;;   ___ftadd->___ftmul
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
;;   ___fttol->___ftpack
;;
;; Critical Paths under _isr1 in BANK0
;;
;;   None.
;;
;; Critical Paths under _main in BANK1
;;
;;   _main->_goParallel
;;   _goParallel->___ftadd
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
;;Main: autosize = 0, tempsize = 1, incstack = 0, save=0
;;

;;
;;Call Graph Tables:
;;
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (0) _main                                                 1     1      0   27341
;;                                             22 BANK1      1     1      0
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;                     _EEPROMToSerial
;;                          _findWalls
;;                         _turnAround
;;                         _turnLeft90
;;                        _turnRight90
;;                     _lcd_write_data
;;                  _play_iCreate_song
;;                           _rotateIR
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                         _goParallel
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
;; (1) _goToNextCell                                         0     0      0    9170
;;               _getSomethingInTheWay
;;                             _goLeft
;;                          _goForward
;;                            _goRight
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (1) _findWalls                                            1     1      0    1475
;;                                             45 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                           _rotateIR
;;                  _play_iCreate_song
;; ---------------------------------------------------------------------------------
;; (1) _goRight                                              1     1      0    2370
;;                                             34 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goLeft                                               0     0      0    2370
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goForward                                            0     0      0    2060
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _getCurrentX
;;                        _getCurrentY
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0    2370
;;                                             34 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnAround
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goParallel                                          13    13      0    5060
;;                                              9 BANK1     13    13      0
;;                             _readIR
;;                           _rotateIR
;;                           ___lbtoft
;;                            ___ftmul
;;                            ___ftadd
;;                            ___fttol
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1218
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (1) _frontWallCorrect                                     4     4      0    1605
;;                                             45 BANK0      4     4      0
;;                           _rotateIR
;;                             _readIR
;;                              _drive
;; ---------------------------------------------------------------------------------
;; (2) _driveForDistance                                    12    10      2    1964
;;                                             22 BANK0     12    10      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;;                          _goReverse
;;                        _turnRight90
;;                  _updateOrientation
;;                         _turnLeft90
;;                        _getCurrentY
;;                        _getCurrentX
;;               _findFinalDestination
;; ---------------------------------------------------------------------------------
;; (1) _updateLocation                                       1     1      0     158
;;                                             14 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                     _getOrientation
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _lookForVictim                                        3     3      0     536
;;                                             14 BANK0      3     3      0
;;                          _ser_putch
;;                          _ser_getch
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                      _getVictimZone
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _checkForFinalDestination                             0     0      0     158
;;                          _getFinalX
;;                          _getFinalY
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (1) _init                                                 0     0      0     223
;;                           _init_adc
;;                           _lcd_init
;;                           _ser_init
;;                         _initIRobot
;;                          _initSongs
;; ---------------------------------------------------------------------------------
;; (3) _goReverse                                            3     3      0     375
;;                                             19 BANK0      3     3      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _readIR                                               4     2      2    1218
;;                                             41 BANK0      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (3) _findFinalDestination                                 4     2      2     406
;;                                             14 BANK0      4     2      2
;;                     _lcd_set_cursor
;;              _lcd_write_1_digit_bcd
;;                        _getCurrentY (ARG)
;;                        _getCurrentX (ARG)
;; ---------------------------------------------------------------------------------
;; (1) _updateMapData                                        9     4      5     396
;;                                             19 BANK0      9     4      5
;;                         _addNewData
;;                     _getOrientation (ARG)
;; ---------------------------------------------------------------------------------
;; (1) _checkIfHome                                          0     0      0     217
;;                              _drive
;;                  _play_iCreate_song
;; ---------------------------------------------------------------------------------
;; (3) _turnAround                                           3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnLeft90                                           3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                        _getCurrentX
;;                        _getCurrentY
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnRight90                                          3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _initSongs                                            0     0      0     161
;;                         _ser_putArr
;; ---------------------------------------------------------------------------------
;; (2) _lcd_init                                             0     0      0      31
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (2) _lcd_write_1_digit_bcd                                1     1      0      62
;;                                             13 BANK0      1     1      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _lcd_set_cursor                                       1     1      0      65
;;                                             13 BANK0      1     1      0
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (1) _EEPROMToSerial                                       4     4      0     198
;;                                             17 BANK0      4     4      0
;;                         _readEEPROM
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _addNewData                                           2     2      0     155
;;                                             17 BANK0      2     2      0
;;                        _writeEEPROM
;; ---------------------------------------------------------------------------------
;; (1) _lcd_write_string                                     2     2      0      96
;;                                             13 BANK0      2     2      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _adc_read_channel                                     4     2      2     345
;;                                             37 BANK0      4     2      2
;;                           _adc_read
;;                            _convert (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___lbtoft                                             8     5      3     343
;;                                             32 BANK0      8     5      3
;;                           ___ftpack
;;                            ___fttol (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___ftmul                                             16    10      6     800
;;                                             40 BANK0     16    10      6
;;                           ___ftpack
;;                           ___lbtoft (ARG)
;;                            ___fttol (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___ftadd                                             13     7      6    1537
;;                                             56 BANK0      4     4      0
;;                                              0 BANK1      9     3      6
;;                           ___ftpack
;;                            ___ftmul (ARG)
;;                           ___lbtoft (ARG)
;;                            ___fttol (ARG)
;; ---------------------------------------------------------------------------------
;; (2) _initIRobot                                           3     3      0      31
;;                                             12 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (4) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _drive                                                7     4      3     155
;;                                             12 BANK0      7     4      3
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _rotateIR                                             5     4      1      99
;;                                             10 BANK0      5     4      1
;; ---------------------------------------------------------------------------------
;; (3) _convert                                              4     2      2     839
;;                                             33 BANK0      4     2      2
;;                             ___wmul
;;                            ___awdiv
;;                           _adc_read (ARG)
;; ---------------------------------------------------------------------------------
;; (2) _play_iCreate_song                                    1     1      0      62
;;                                             12 BANK0      1     1      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _ser_putArr                                           9     5      4     161
;;                                             12 BANK0      9     5      4
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _ser_getch                                            2     2      0      34
;;                                             10 BANK0      2     2      0
;;                           _ser_isrx
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_data                                       3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_control                                    3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (2) _readEEPROM                                           6     5      1     130
;;                                             11 BANK0      6     5      1
;;                     _initEEPROMMode
;;                       _writeSPIByte
;; ---------------------------------------------------------------------------------
;; (3) _writeEEPROM                                          6     4      2     124
;;                                             11 BANK0      6     4      2
;;                     _initEEPROMMode
;;                       _writeSPIByte
;; ---------------------------------------------------------------------------------
;; (2) _init_adc                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _adc_read                                             8     6      2     323
;;                                             25 BANK0      8     6      2
;;                            ___awdiv
;; ---------------------------------------------------------------------------------
;; (4) ___awdiv                                              9     5      4     300
;;                                             16 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___fttol                                             14    10      4     252
;;                                             18 BANK0     14    10      4
;;                           ___ftpack (ARG)
;; ---------------------------------------------------------------------------------
;; (3) ___ftpack                                             8     3      5     312
;;                                             10 BANK0      8     3      5
;; ---------------------------------------------------------------------------------
;; (4) ___wmul                                               6     2      4     136
;;                                             10 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (1) _updateNode                                           1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (1) _getSuccessfulDrive                                   0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getSomethingInTheWay                                 0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getOrientation                                       0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _getCurrentY                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _getCurrentX                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _updateOrientation                                    2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (2) _ser_init                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _ser_isrx                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getVictimZone                                        3     2      1     186
;;                                             10 BANK0      3     2      1
;; ---------------------------------------------------------------------------------
;; (2) _getFinalY                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getFinalX                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _ser_putch                                            2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (4) _initEEPROMMode                                       1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _writeSPIByte                                         1     1      0      31
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 4
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (7) _isr1                                                10    10      0     159
;;                                              0 BANK0     10    10      0
;;                            ___lwmod
;; ---------------------------------------------------------------------------------
;; (8) ___lwmod                                              6     2      4     159
;;                                              0 COMMON     6     2      4
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 8
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
;;   _EEPROMToSerial
;;     _readEEPROM
;;       _initEEPROMMode
;;       _writeSPIByte
;;     _ser_putch
;;   _findWalls
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
;;     _rotateIR
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
;;   _rotateIR
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
;;   _goParallel
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
;;     _rotateIR
;;     ___lbtoft
;;       ___ftpack
;;       ___fttol (ARG)
;;         ___ftpack (ARG)
;;     ___ftmul
;;       ___ftpack
;;       ___lbtoft (ARG)
;;         ___ftpack
;;         ___fttol (ARG)
;;           ___ftpack (ARG)
;;       ___fttol (ARG)
;;         ___ftpack (ARG)
;;     ___ftadd
;;       ___ftpack
;;       ___ftmul (ARG)
;;         ___ftpack
;;         ___lbtoft (ARG)
;;           ___ftpack
;;           ___fttol (ARG)
;;             ___ftpack (ARG)
;;         ___fttol (ARG)
;;           ___ftpack (ARG)
;;       ___lbtoft (ARG)
;;         ___ftpack
;;         ___fttol (ARG)
;;           ___ftpack (ARG)
;;       ___fttol (ARG)
;;         ___ftpack (ARG)
;;     ___fttol
;;       ___ftpack (ARG)
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _frontWallCorrect
;;     _rotateIR
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
;;   ___lwmod
;;

;; Address spaces:

;;Name               Size   Autos  Total    Cost      Usage
;;BANK3               60      0      49       9       76.0%
;;BITBANK3            60      0       0       8        0.0%
;;SFR3                 0      0       0       4        0.0%
;;BITSFR3              0      0       0       4        0.0%
;;BANK2               60      0      1D      11       30.2%
;;BITBANK2            60      0       0      10        0.0%
;;SFR2                 0      0       0       5        0.0%
;;BITSFR2              0      0       0       5        0.0%
;;SFR1                 0      0       0       2        0.0%
;;BITSFR1              0      0       0       2        0.0%
;;BANK1               50     17      4C       7       95.0%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0     10E      12        0.0%
;;ABS                  0      0     104       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       A       2        0.0%
;;BANK0               50     3C      46       5       87.5%
;;BITBANK0            50      0       0       4        0.0%
;;SFR0                 0      0       0       1        0.0%
;;BITSFR0              0      0       0       1        0.0%
;;COMMON               E      6       C       1       85.7%
;;BITCOMMON            E      0       2       0       14.3%
;;EEDATA             100      0       0       0        0.0%

	global	_main
psect	maintext,global,class=CODE,delta=2
global __pmaintext
__pmaintext:

;; *************** function _main *****************
;; Defined at:
;;		line 333 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
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
;;      Locals:         0       0       0       0       0
;;      Temps:          0       0       1       0       0
;;      Totals:         0       0       1       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_init
;;		_drive
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_EEPROMToSerial
;;		_findWalls
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_lcd_write_data
;;		_play_iCreate_song
;;		_rotateIR
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_goParallel
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
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	333
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	334
	
l12445:	
;main.c: 334: init();
	fcall	_init
	line	335
;main.c: 335: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	337
	
l12447:	
;main.c: 337: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	338
	
l12449:	
;main.c: 338: lcd_write_string("(-,-) - -- --- -");
	movlw	((STR_3-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	339
;main.c: 339: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	340
	
l12451:	
;main.c: 340: lcd_write_string("- - - (3,1) GREG");
	movlw	((STR_4-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	342
;main.c: 342: while(!home)
	goto	l12577
	
l6788:	
	line	345
	
l12453:	
;main.c: 343: {
;main.c: 345: if(eeprom.pressed && ready == 0)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_eeprom)^080h,w
	skipz
	goto	u7530
	goto	l12461
u7530:
	
l12455:	
	btfsc	(_ready/8),(_ready)&7
	goto	u7541
	goto	u7540
u7541:
	goto	l12461
u7540:
	line	347
	
l12457:	
;main.c: 346: {
;main.c: 347: EEPROMToSerial();
	fcall	_EEPROMToSerial
	line	348
	
l12459:	
;main.c: 348: eeprom.pressed = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_eeprom)^080h
	goto	l12461
	line	349
	
l6789:	
	line	351
	
l12461:	
;main.c: 349: }
;main.c: 351: if(start.pressed && ready == 0)
	movf	(_start)^080h,w
	skipz
	goto	u7550
	goto	l12491
u7550:
	
l12463:	
	btfsc	(_ready/8),(_ready)&7
	goto	u7561
	goto	u7560
u7561:
	goto	l12491
u7560:
	line	353
	
l12465:	
;main.c: 352: {
;main.c: 353: findWalls();
	fcall	_findWalls
	line	354
	
l12467:	
;main.c: 354: if(leftWall && rightWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7571
	goto	u7570
u7571:
	goto	l6791
u7570:
	
l12469:	
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7581
	goto	u7580
u7581:
	goto	l6791
u7580:
	
l12471:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7591
	goto	u7590
u7591:
	goto	l6791
u7590:
	line	355
	
l12473:	
;main.c: 355: turnAround();
	fcall	_turnAround
	goto	l12483
	line	356
	
l6791:	
;main.c: 356: else if (rightWall && frontWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7601
	goto	u7600
u7601:
	goto	l6793
u7600:
	
l12475:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7611
	goto	u7610
u7611:
	goto	l6793
u7610:
	line	357
	
l12477:	
;main.c: 357: turnLeft90();
	fcall	_turnLeft90
	goto	l12483
	line	358
	
l6793:	
;main.c: 358: else if(leftWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7621
	goto	u7620
u7621:
	goto	l12483
u7620:
	
l12479:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7631
	goto	u7630
u7631:
	goto	l12483
u7630:
	line	359
	
l12481:	
;main.c: 359: turnRight90();
	fcall	_turnRight90
	goto	l12483
	
l6795:	
	goto	l12483
	line	360
	
l6794:	
	goto	l12483
	
l6792:	
	
l12483:	
;main.c: 360: ready = 1;
	bsf	(_ready/8),(_ready)&7
	line	361
	
l12485:	
;main.c: 361: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	362
	
l12487:	
;main.c: 362: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	363
;main.c: 363: play_iCreate_song(1);
	movlw	(01h)
	fcall	_play_iCreate_song
	line	364
	
l12489:	
;main.c: 364: rotateIR(12, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_main+0)^080h+0
	movf	(??_main+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(0Ch)
	fcall	_rotateIR
	goto	l12491
	line	365
	
l6790:	
	line	367
	
l12491:	
;main.c: 365: }
;main.c: 367: if(start.pressed && ready == 1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_start)^080h,w
	skipz
	goto	u7640
	goto	l12577
u7640:
	
l12493:	
	btfss	(_ready/8),(_ready)&7
	goto	u7651
	goto	u7650
u7651:
	goto	l12577
u7650:
	line	369
	
l12495:	
;main.c: 368: {
;main.c: 369: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	370
;main.c: 370: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	371
;main.c: 371: lookForVictim();
	fcall	_lookForVictim
	line	372
;main.c: 372: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	373
	
l12497:	
;main.c: 373: findWalls();
	fcall	_findWalls
	line	374
	
l12499:	
;main.c: 374: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	375
	
l12501:	
;main.c: 375: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7661
	goto	u7660
u7661:
	goto	l12505
u7660:
	line	376
	
l12503:	
;main.c: 376: goParallel();
	fcall	_goParallel
	goto	l12507
	line	377
	
l6797:	
	line	378
	
l12505:	
;main.c: 377: else
;main.c: 378: rotateIR(12, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_main+0)^080h+0
	movf	(??_main+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(0Ch)
	fcall	_rotateIR
	goto	l12507
	
l6798:	
	line	379
	
l12507:	
;main.c: 379: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	380
	
l12509:	
;main.c: 380: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7671
	goto	u7670
u7671:
	goto	l12513
u7670:
	line	381
	
l12511:	
;main.c: 381: frontWallCorrect();
	fcall	_frontWallCorrect
	goto	l12513
	
l6799:	
	line	382
	
l12513:	
;main.c: 382: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	383
;main.c: 383: switch(node)
	goto	l12561
	line	385
;main.c: 384: {
;main.c: 385: case 0:
	
l6801:	
	line	386
	
l12515:	
;main.c: 386: goToNextCell();
	fcall	_goToNextCell
	line	387
;main.c: 387: break;
	goto	l12563
	line	388
;main.c: 388: case 1:
	
l6803:	
	line	389
;main.c: 389: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7681
	goto	u7680
u7681:
	goto	l12529
u7680:
	line	391
	
l12517:	
;main.c: 390: {
;main.c: 391: if (victimZone == 1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	01h
	skipz
	goto	u7691
	goto	u7690
u7691:
	goto	l12521
u7690:
	line	392
	
l12519:	
;main.c: 392: goRight();
	fcall	_goRight
	goto	l12563
	line	393
	
l6805:	
	
l12521:	
;main.c: 393: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u7701
	goto	u7700
u7701:
	goto	l12525
u7700:
	line	394
	
l12523:	
;main.c: 394: goForward();
	fcall	_goForward
	goto	l12563
	line	395
	
l6807:	
	
l12525:	
;main.c: 395: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7711
	goto	u7710
u7711:
	goto	l12563
u7710:
	line	396
	
l12527:	
;main.c: 396: goRight();
	fcall	_goRight
	goto	l12563
	
l6809:	
	goto	l12563
	line	397
	
l6808:	
	goto	l12563
	
l6806:	
;main.c: 397: }
	goto	l12563
	line	398
	
l6804:	
	line	399
	
l12529:	
;main.c: 398: else
;main.c: 399: goToNextCell();
	fcall	_goToNextCell
	goto	l12563
	
l6810:	
	line	400
;main.c: 400: break;
	goto	l12563
	line	401
;main.c: 401: case 2:
	
l6811:	
	line	402
;main.c: 402: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7721
	goto	u7720
u7721:
	goto	l12543
u7720:
	line	404
	
l12531:	
;main.c: 403: {
;main.c: 404: if (victimZone == 2)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	02h
	skipz
	goto	u7731
	goto	u7730
u7731:
	goto	l12535
u7730:
	line	405
	
l12533:	
;main.c: 405: goForward();
	fcall	_goForward
	goto	l12563
	line	406
	
l6813:	
	
l12535:	
;main.c: 406: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7741
	goto	u7740
u7741:
	goto	l12539
u7740:
	line	407
	
l12537:	
;main.c: 407: goRight();
	fcall	_goRight
	goto	l12563
	line	408
	
l6815:	
	
l12539:	
;main.c: 408: else if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u7751
	goto	u7750
u7751:
	goto	l12563
u7750:
	line	409
	
l12541:	
;main.c: 409: goLeft();
	fcall	_goLeft
	goto	l12563
	
l6817:	
	goto	l12563
	line	410
	
l6816:	
	goto	l12563
	
l6814:	
;main.c: 410: }
	goto	l12563
	line	411
	
l6812:	
	line	412
	
l12543:	
;main.c: 411: else
;main.c: 412: goToNextCell();
	fcall	_goToNextCell
	goto	l12563
	
l6818:	
	line	413
;main.c: 413: break;
	goto	l12563
	line	414
;main.c: 414: case 3:
	
l6819:	
	line	415
;main.c: 415: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7761
	goto	u7760
u7761:
	goto	l12557
u7760:
	line	417
	
l12545:	
;main.c: 416: {
;main.c: 417: if (victimZone == 3)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	03h
	skipz
	goto	u7771
	goto	u7770
u7771:
	goto	l12549
u7770:
	line	418
	
l12547:	
;main.c: 418: goRight();
	fcall	_goRight
	goto	l12563
	line	419
	
l6821:	
	
l12549:	
;main.c: 419: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u7781
	goto	u7780
u7781:
	goto	l12553
u7780:
	line	420
	
l12551:	
;main.c: 420: goForward();
	fcall	_goForward
	goto	l12563
	line	421
	
l6823:	
	
l12553:	
;main.c: 421: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7791
	goto	u7790
u7791:
	goto	l12563
u7790:
	line	422
	
l12555:	
;main.c: 422: goLeft();
	fcall	_goLeft
	goto	l12563
	
l6825:	
	goto	l12563
	line	423
	
l6824:	
	goto	l12563
	
l6822:	
;main.c: 423: }
	goto	l12563
	line	424
	
l6820:	
	line	425
	
l12557:	
;main.c: 424: else
;main.c: 425: goToNextCell();
	fcall	_goToNextCell
	goto	l12563
	
l6826:	
	line	426
;main.c: 426: break;
	goto	l12563
	line	427
;main.c: 427: default:
	
l6827:	
	line	428
;main.c: 428: break;
	goto	l12563
	line	429
	
l12559:	
;main.c: 429: }
	goto	l12563
	line	383
	
l6800:	
	
l12561:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_node)^080h,w	;volatile
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
	goto	l12515
	xorlw	1^0	; case 1
	skipnz
	goto	l6803
	xorlw	2^1	; case 2
	skipnz
	goto	l6811
	xorlw	3^2	; case 3
	skipnz
	goto	l6819
	goto	l12563
	opt asmopt_on

	line	429
	
l6802:	
	line	430
	
l12563:	
;main.c: 430: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	433
	
l12565:	
;main.c: 433: if(getSuccessfulDrive())
	fcall	_getSuccessfulDrive
	btfss	status,0
	goto	u7801
	goto	u7800
u7801:
	goto	l12577
u7800:
	line	436
	
l12567:	
;main.c: 434: {
;main.c: 436: updateMapData(0,0,0,0,0,getOrientation());
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_updateMapData)
	clrf	0+(?_updateMapData)+01h
	clrf	0+(?_updateMapData)+02h
	clrf	0+(?_updateMapData)+03h
	fcall	_getOrientation
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_main+0)^080h+0
	movf	(??_main+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_updateMapData)+04h
	movlw	(0)
	fcall	_updateMapData
	line	437
	
l12569:	
;main.c: 437: updateLocation();
	fcall	_updateLocation
	line	438
	
l12571:	
;main.c: 438: updateNode();
	fcall	_updateNode
	line	439
	
l12573:	
;main.c: 439: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7811
	goto	u7810
u7811:
	goto	l6829
u7810:
	line	440
	
l12575:	
;main.c: 440: checkIfHome();
	fcall	_checkIfHome
	
l6829:	
	line	441
;main.c: 441: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	goto	l12577
	line	442
	
l6828:	
	goto	l12577
	line	443
	
l6796:	
	goto	l12577
	line	444
	
l6787:	
	line	342
	
l12577:	
	btfss	(_home/8),(_home)&7
	goto	u7821
	goto	u7820
u7821:
	goto	l12453
u7820:
	goto	l6831
	
l6830:	
	line	446
	
l6831:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text2197,local,class=CODE,delta=2
global __ptext2197
__ptext2197:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 266 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    7
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
psect	text2197
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	266
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 0
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	267
	
l12429:	
;main.c: 267: if(!leftWall && (getSomethingInTheWay() != LEFT))
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u7471
	goto	u7470
u7471:
	goto	l6755
u7470:
	
l12431:	
	fcall	_getSomethingInTheWay
	xorlw	01h
	skipnz
	goto	u7481
	goto	u7480
u7481:
	goto	l6755
u7480:
	line	268
	
l12433:	
;main.c: 268: goLeft();
	fcall	_goLeft
	goto	l6761
	line	269
	
l6755:	
;main.c: 269: else if(!frontWall && (getSomethingInTheWay() != FORWARD))
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u7491
	goto	u7490
u7491:
	goto	l6757
u7490:
	
l12435:	
	fcall	_getSomethingInTheWay
	xorlw	0
	skipnz
	goto	u7501
	goto	u7500
u7501:
	goto	l6757
u7500:
	line	270
	
l12437:	
;main.c: 270: goForward();
	fcall	_goForward
	goto	l6761
	line	271
	
l6757:	
;main.c: 271: else if(!rightWall && (getSomethingInTheWay() != RIGHT))
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u7511
	goto	u7510
u7511:
	goto	l12443
u7510:
	
l12439:	
	fcall	_getSomethingInTheWay
	xorlw	03h
	skipnz
	goto	u7521
	goto	u7520
u7521:
	goto	l12443
u7520:
	line	272
	
l12441:	
;main.c: 272: goRight();
	fcall	_goRight
	goto	l6761
	line	273
	
l6759:	
	line	274
	
l12443:	
;main.c: 273: else
;main.c: 274: goBackward();
	fcall	_goBackward
	goto	l6761
	
l6760:	
	goto	l6761
	
l6758:	
	goto	l6761
	
l6756:	
	line	275
	
l6761:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text2198,local,class=CODE,delta=2
global __ptext2198
__ptext2198:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 192 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_lcd_set_cursor
;;		_findWall
;;		_lcd_write_data
;;		_rotateIR
;;		_play_iCreate_song
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2198
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	192
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	193
	
l12397:	
;main.c: 193: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	195
	
l12399:	
;main.c: 195: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7361
	goto	u7360
	
u7361:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u7374
u7360:
	bcf	(_leftWall/8),(_leftWall)&7
u7374:
	line	196
	
l12401:	
;main.c: 196: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7381
	goto	u7380
u7381:
	goto	l12405
u7380:
	line	197
	
l12403:	
;main.c: 197: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	goto	l6737
	line	198
	
l6736:	
	line	199
	
l12405:	
;main.c: 198: else
;main.c: 199: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6737:	
	line	201
;main.c: 201: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	202
	
l12407:	
;main.c: 202: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7391
	goto	u7390
	
u7391:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u7404
u7390:
	bcf	(_frontWall/8),(_frontWall)&7
u7404:
	line	204
	
l12409:	
;main.c: 204: if(xCoord == 2 && yCoord == 1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u7411
	goto	u7410
u7411:
	goto	l6738
u7410:
	
l12411:	
	movf	(_yCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u7421
	goto	u7420
u7421:
	goto	l6738
u7420:
	line	206
	
l12413:	
;main.c: 205: {
;main.c: 206: frontWall = 1;
	bsf	(_frontWall/8),(_frontWall)&7
	line	207
	
l6738:	
	line	208
;main.c: 207: }
;main.c: 208: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7431
	goto	u7430
u7431:
	goto	l12417
u7430:
	line	209
	
l12415:	
;main.c: 209: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	goto	l6740
	line	210
	
l6739:	
	line	211
	
l12417:	
;main.c: 210: else
;main.c: 211: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6740:	
	line	213
;main.c: 213: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	214
	
l12419:	
;main.c: 214: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7441
	goto	u7440
	
u7441:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u7454
u7440:
	bcf	(_rightWall/8),(_rightWall)&7
u7454:
	line	216
	
l12421:	
;main.c: 216: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7461
	goto	u7460
u7461:
	goto	l12427
u7460:
	line	218
	
l12423:	
;main.c: 217: {
;main.c: 218: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	219
	
l12425:	
;main.c: 219: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	220
;main.c: 220: }else
	goto	l6742
	
l6741:	
	line	221
	
l12427:	
;main.c: 221: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6742:	
	line	223
;main.c: 223: rotateIR(36, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(024h)
	fcall	_rotateIR
	line	224
	
l6743:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text2199,local,class=CODE,delta=2
global __ptext2199
__ptext2199:

;; *************** function _goRight *****************
;; Defined at:
;;		line 204 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
psect	text2199
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	204
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 1
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	205
	
l12387:	
;drive.c: 205: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	206
;drive.c: 206: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	207
	
l12389:	
;drive.c: 207: turnRight90();
	fcall	_turnRight90
	line	208
	
l12391:	
;drive.c: 208: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	209
	
l12393:	
;drive.c: 209: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	210
	
l12395:	
;drive.c: 210: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	211
	
l5866:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text2200,local,class=CODE,delta=2
global __ptext2200
__ptext2200:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 183 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
psect	text2200
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	183
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 1
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	184
	
l12377:	
;drive.c: 184: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	185
;drive.c: 185: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	186
	
l12379:	
;drive.c: 186: turnLeft90();
	fcall	_turnLeft90
	line	187
	
l12381:	
;drive.c: 187: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	188
	
l12383:	
;drive.c: 188: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	189
	
l12385:	
;drive.c: 189: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	190
	
l5860:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text2201,local,class=CODE,delta=2
global __ptext2201
__ptext2201:

;; *************** function _goForward *****************
;; Defined at:
;;		line 168 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
psect	text2201
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	168
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 1
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	169
	
l12365:	
;drive.c: 169: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	170
;drive.c: 170: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	171
	
l12367:	
;drive.c: 171: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	172
	
l12369:	
;drive.c: 172: if( (getCurrentX() == 1 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	01h
	skipz
	goto	u7341
	goto	u7340
u7341:
	goto	l12375
u7340:
	
l12371:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u7351
	goto	u7350
u7351:
	goto	l12375
u7350:
	line	174
	
l12373:	
;drive.c: 173: {
;drive.c: 174: driveForDistance(800);
	movlw	low(0320h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(0320h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	175
;drive.c: 175: }else
	goto	l5857
	
l5855:	
	line	177
	
l12375:	
;drive.c: 176: {
;drive.c: 177: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	goto	l5857
	line	178
	
l5856:	
	line	179
	
l5857:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text2202,local,class=CODE,delta=2
global __ptext2202
__ptext2202:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_turnAround
;;		_updateOrientation
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text2202
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	157
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 0
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	158
	
l12355:	
;drive.c: 158: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	159
;drive.c: 159: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	160
	
l12357:	
;drive.c: 160: turnAround();
	fcall	_turnAround
	line	161
	
l12359:	
;drive.c: 161: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	162
	
l12361:	
;drive.c: 162: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	163
	
l12363:	
;drive.c: 163: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	164
	
l5852:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_goParallel
psect	text2203,local,class=CODE,delta=2
global __ptext2203
__ptext2203:

;; *************** function _goParallel *****************
;; Defined at:
;;		line 227 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  step            2   18[BANK1 ] int 
;;  angleParalle    2   20[BANK1 ] int 
;;  distance        2   16[BANK1 ] int 
;;  shortestDist    2   12[BANK1 ] int 
;;  angleLowByte    1   15[BANK1 ] unsigned char 
;;  angleHighByt    1   14[BANK1 ] unsigned char 
;;  stepsToWall     1   11[BANK1 ] unsigned char 
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
;;      Locals:         0       0      11       0       0
;;      Temps:          0       0       2       0       0
;;      Totals:         0       0      13       0       0
;;Total ram usage:       13 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_readIR
;;		_rotateIR
;;		___lbtoft
;;		___ftmul
;;		___ftadd
;;		___fttol
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2203
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	227
	global	__size_of_goParallel
	__size_of_goParallel	equ	__end_of_goParallel-_goParallel
	
_goParallel:	
	opt	stack 1
; Regs used in _goParallel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	228
	
l12309:	
;main.c: 228: PORTC |= 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	iorwf	(7),f	;volatile
	line	230
	
l12311:	
;main.c: 230: int distance, shortestDistance = 999;
	movlw	low(03E7h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@shortestDistance)^080h
	movlw	high(03E7h)
	movwf	((goParallel@shortestDistance)^080h)+1
	line	233
	
l12313:	
;main.c: 231: char stepsToWall;
;main.c: 233: for (int step = -12; step <= 12; step++)
	movlw	low(-12)
	movwf	(goParallel@step)^080h
	movlw	high(-12)
	movwf	((goParallel@step)^080h)+1
	
l12315:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7275
	movlw	low(0Dh)
	subwf	(goParallel@step)^080h,w
u7275:

	skipc
	goto	u7271
	goto	u7270
u7271:
	goto	l12319
u7270:
	goto	l12333
	
l12317:	
	goto	l12333
	line	234
	
l6746:	
	line	235
	
l12319:	
;main.c: 234: {
;main.c: 235: distance = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@distance+1)^080h
	addwf	(goParallel@distance+1)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?_readIR)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@distance)^080h
	addwf	(goParallel@distance)^080h

	line	236
	
l12321:	
;main.c: 236: if(distance < shortestDistance)
	movf	(goParallel@distance+1)^080h,w
	xorlw	80h
	movwf	(??_goParallel+0)^080h+0
	movf	(goParallel@shortestDistance+1)^080h,w
	xorlw	80h
	subwf	(??_goParallel+0)^080h+0,w
	skipz
	goto	u7285
	movf	(goParallel@shortestDistance)^080h,w
	subwf	(goParallel@distance)^080h,w
u7285:

	skipnc
	goto	u7281
	goto	u7280
u7281:
	goto	l12327
u7280:
	line	238
	
l12323:	
;main.c: 237: {
;main.c: 238: stepsToWall = step;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@step)^080h,w
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@stepsToWall)^080h
	line	239
	
l12325:	
;main.c: 239: shortestDistance = distance;
	movf	(goParallel@distance+1)^080h,w
	clrf	(goParallel@shortestDistance+1)^080h
	addwf	(goParallel@shortestDistance+1)^080h
	movf	(goParallel@distance)^080h,w
	clrf	(goParallel@shortestDistance)^080h
	addwf	(goParallel@shortestDistance)^080h

	goto	l12327
	line	240
	
l6748:	
	line	241
	
l12327:	
;main.c: 240: }
;main.c: 241: rotateIR(1, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(01h)
	fcall	_rotateIR
	line	233
	
l12329:	
	movlw	low(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(goParallel@step)^080h,f
	skipnc
	incf	(goParallel@step+1)^080h,f
	movlw	high(01h)
	addwf	(goParallel@step+1)^080h,f
	
l12331:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7295
	movlw	low(0Dh)
	subwf	(goParallel@step)^080h,w
u7295:

	skipc
	goto	u7291
	goto	u7290
u7291:
	goto	l12319
u7290:
	goto	l12333
	
l6747:	
	line	243
	
l12333:	
;main.c: 242: }
;main.c: 243: rotateIR(12, 0b00001111);
	movlw	(0Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(0Ch)
	fcall	_rotateIR
	line	245
;main.c: 245: int angleParallelToWall = (int)((stepsToWall*3.75)-6);
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@stepsToWall)^080h,w
	fcall	___lbtoft
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___lbtoft)),w
	movwf	0+(?___ftmul)+03h
	movf	(1+(?___lbtoft)),w
	movwf	1+(?___ftmul)+03h
	movf	(2+(?___lbtoft)),w
	movwf	2+(?___ftmul)+03h
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x70
	movwf	(?___ftmul+1)
	movlw	0x40
	movwf	(?___ftmul+2)
	fcall	___ftmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___ftmul)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	0+(?___ftadd)^080h+03h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___ftmul)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	1+(?___ftadd)^080h+03h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(2+(?___ftmul)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	2+(?___ftadd)^080h+03h
	movlw	0x0
	movwf	(?___ftadd)^080h
	movlw	0xc0
	movwf	(?___ftadd+1)^080h
	movlw	0xc0
	movwf	(?___ftadd+2)^080h
	fcall	___ftadd
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(0+(?___ftadd))^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___fttol)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?___ftadd))^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___fttol+1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(2+(?___ftadd))^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___fttol+2)
	fcall	___fttol
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	1+(((0+(?___fttol)))),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@angleParallelToWall+1)^080h
	addwf	(goParallel@angleParallelToWall+1)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	0+(((0+(?___fttol)))),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@angleParallelToWall)^080h
	addwf	(goParallel@angleParallelToWall)^080h

	line	246
	
l12335:	
;main.c: 246: char angleHighByte = 0;
	clrf	(goParallel@angleHighByte)^080h
	line	247
	
l12337:	
;main.c: 247: char angleLowByte = (char) angleParallelToWall;
	movf	(goParallel@angleParallelToWall)^080h,w
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@angleLowByte)^080h
	line	249
	
l12339:	
;main.c: 249: if(angleParallelToWall < 0)
	btfss	(goParallel@angleParallelToWall+1)^080h,7
	goto	u7301
	goto	u7300
u7301:
	goto	l12343
u7300:
	line	250
	
l12341:	
;main.c: 250: angleParallelToWall = 360 + angleParallelToWall;
	movf	(goParallel@angleParallelToWall)^080h,w
	addlw	low(0168h)
	movwf	(goParallel@angleParallelToWall)^080h
	movf	(goParallel@angleParallelToWall+1)^080h,w
	skipnc
	addlw	1
	addlw	high(0168h)
	movwf	1+(goParallel@angleParallelToWall)^080h
	goto	l12343
	
l6749:	
	line	252
	
l12343:	
;main.c: 252: if(angleParallelToWall > 255)
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7315
	movlw	low(0100h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7315:

	skipc
	goto	u7311
	goto	u7310
u7311:
	goto	l12349
u7310:
	line	254
	
l12345:	
;main.c: 253: {
;main.c: 254: angleHighByte = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@angleHighByte)^080h
	bsf	status,0
	rlf	(goParallel@angleHighByte)^080h,f
	line	255
	
l12347:	
;main.c: 255: angleLowByte = (char)(angleParallelToWall - 255);
	movf	(goParallel@angleParallelToWall)^080h,w
	addlw	01h
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@angleLowByte)^080h
	goto	l12349
	line	256
	
l6750:	
	line	257
	
l12349:	
;main.c: 256: }
;main.c: 257: if((angleParallelToWall > 8) && (angleParallelToWall < 352))
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(09h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7325
	movlw	low(09h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7325:

	skipc
	goto	u7321
	goto	u7320
u7321:
	goto	l6752
u7320:
	
l12351:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0160h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7335
	movlw	low(0160h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7335:

	skipnc
	goto	u7331
	goto	u7330
u7331:
	goto	l6752
u7330:
	line	259
	
l12353:	
;main.c: 258: {
;main.c: 259: drive(0, 50, 0, 1);
	movlw	(032h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	bsf	status,0
	rlf	0+(?_drive)+02h,f
	movlw	(0)
	fcall	_drive
	line	260
;main.c: 260: waitFor(157,angleHighByte,angleLowByte);
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleHighByte)^080h,w
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_waitFor)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleLowByte)^080h,w
	movwf	(??_goParallel+1)^080h+0
	movf	(??_goParallel+1)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	261
;main.c: 261: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l6752
	line	262
	
l6751:	
	line	263
	
l6752:	
	return
	opt stack 0
GLOBAL	__end_of_goParallel
	__end_of_goParallel:
;; =============== function _goParallel ends ============

	signat	_goParallel,88
	global	_findWall
psect	text2204,local,class=CODE,delta=2
global __ptext2204
__ptext2204:

;; *************** function _findWall *****************
;; Defined at:
;;		line 452 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;;		_readIR
;; This function is called by:
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text2204
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	452
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	453
	
l12297:	
;main.c: 453: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7265
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u7265:

	skipc
	goto	u7261
	goto	u7260
u7261:
	goto	l12305
u7260:
	line	454
	
l12299:	
;main.c: 454: return 0;
	clrc
	
	goto	l6835
	
l12301:	
	goto	l6835
	
l12303:	
	goto	l6835
	line	455
	
l6834:	
	line	456
	
l12305:	
;main.c: 455: else
;main.c: 456: return 1;
	setc
	
	goto	l6835
	
l12307:	
	goto	l6835
	
l6836:	
	line	457
	
l6835:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text2205,local,class=CODE,delta=2
global __ptext2205
__ptext2205:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 277 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  distToWall      2   47[BANK0 ] int 
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
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_rotateIR
;;		_readIR
;;		_drive
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2205
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	277
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 1
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	278
	
l12271:	
;drive.c: 278: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	279
	
l12273:	
;drive.c: 279: int distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	line	280
	
l12275:	
;drive.c: 280: if(distToWall < 45)
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7225
	movlw	low(02Dh)
	subwf	(frontWallCorrect@distToWall),w
u7225:

	skipnc
	goto	u7221
	goto	u7220
u7221:
	goto	l12285
u7220:
	line	282
	
l12277:	
;drive.c: 281: {
;drive.c: 282: drive(255, 125, 128, 0);
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
	line	283
;drive.c: 283: while(distToWall < 51)
	goto	l12281
	
l5889:	
	line	284
	
l12279:	
;drive.c: 284: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l12281
	
l5888:	
	line	283
	
l12281:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(033h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7235
	movlw	low(033h)
	subwf	(frontWallCorrect@distToWall),w
u7235:

	skipc
	goto	u7231
	goto	u7230
u7231:
	goto	l12279
u7230:
	goto	l12283
	
l5890:	
	line	285
	
l12283:	
;drive.c: 285: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	286
;drive.c: 286: }
	goto	l12295
	line	287
	
l5887:	
	
l12285:	
;drive.c: 287: else if(distToWall > 55)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7245
	movlw	low(038h)
	subwf	(frontWallCorrect@distToWall),w
u7245:

	skipc
	goto	u7241
	goto	u7240
u7241:
	goto	l12295
u7240:
	line	289
	
l12287:	
;drive.c: 288: {
;drive.c: 289: drive(0, 250, 128, 0);
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
	line	290
;drive.c: 290: while(distToWall > 49)
	goto	l12291
	
l5894:	
	line	291
	
l12289:	
;drive.c: 291: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l12291
	
l5893:	
	line	290
	
l12291:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(032h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7255
	movlw	low(032h)
	subwf	(frontWallCorrect@distToWall),w
u7255:

	skipnc
	goto	u7251
	goto	u7250
u7251:
	goto	l12289
u7250:
	goto	l12293
	
l5895:	
	line	292
	
l12293:	
;drive.c: 292: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l12295
	line	293
	
l5892:	
	goto	l12295
	line	294
	
l5891:	
	
l12295:	
;drive.c: 293: }
;drive.c: 294: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	295
	
l5896:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_driveForDistance
psect	text2206,local,class=CODE,delta=2
global __ptext2206
__ptext2206:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 31 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;  moveDistance    2   22[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  distance        2   28[BANK0 ] int 
;;  deltaDistanc    2   26[BANK0 ] int 
;;  cliff           1   33[BANK0 ] volatile unsigned char 
;;  virtualWall     1   32[BANK0 ] volatile unsigned char 
;;  low             1   31[BANK0 ] volatile unsigned char 
;;  high            1   30[BANK0 ] volatile unsigned char 
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
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_drive
;;		_ser_putch
;;		_ser_getch
;;		_goReverse
;;		_turnRight90
;;		_updateOrientation
;;		_turnLeft90
;;		_getCurrentY
;;		_getCurrentX
;;		_findFinalDestination
;; This function is called by:
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text2206
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	31
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 1
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l12187:	
;drive.c: 33: volatile char high, low, cliff, virtualWall;
;drive.c: 34: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	35
;drive.c: 35: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	37
	
l12189:	
;drive.c: 37: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	38
	
l12191:	
;drive.c: 38: drive(0, 250, 128, 0);
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
	line	39
	
l12193:	
;drive.c: 39: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	41
;drive.c: 41: while(moving)
	goto	l12269
	
l5820:	
	line	43
	
l12195:	
;drive.c: 42: {
;drive.c: 43: if(distance >= 100)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7105
	movlw	low(064h)
	subwf	(driveForDistance@distance),w
u7105:

	skipc
	goto	u7101
	goto	u7100
u7101:
	goto	l12231
u7100:
	line	46
	
l12197:	
;drive.c: 44: {
;drive.c: 46: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	47
;drive.c: 47: ser_putch(10);
	movlw	(0Ah)
	fcall	_ser_putch
	line	48
;drive.c: 48: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	49
	
l12199:	
;drive.c: 49: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u7111
	goto	u7110
u7111:
	goto	l12211
u7110:
	line	51
	
l12201:	
;drive.c: 50: {
;drive.c: 51: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	52
;drive.c: 52: ser_putch(11);
	movlw	(0Bh)
	fcall	_ser_putch
	line	53
;drive.c: 53: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	54
	
l12203:	
;drive.c: 54: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u7121
	goto	u7120
u7121:
	goto	l12211
u7120:
	line	56
	
l12205:	
;drive.c: 55: {
;drive.c: 56: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	57
;drive.c: 57: ser_putch(9);
	movlw	(09h)
	fcall	_ser_putch
	line	58
;drive.c: 58: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	line	59
	
l12207:	
;drive.c: 59: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u7131
	goto	u7130
u7131:
	goto	l12211
u7130:
	line	61
	
l12209:	
;drive.c: 60: {
;drive.c: 61: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	62
;drive.c: 62: ser_putch(12);
	movlw	(0Ch)
	fcall	_ser_putch
	line	63
;drive.c: 63: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	goto	l12211
	line	64
	
l5824:	
	goto	l12211
	line	65
	
l5823:	
	goto	l12211
	line	66
	
l5822:	
	line	67
	
l12211:	
;drive.c: 64: }
;drive.c: 65: }
;drive.c: 66: }
;drive.c: 67: if(cliff == 1)
	movf	(driveForDistance@cliff),w	;volatile
	xorlw	01h
	skipz
	goto	u7141
	goto	u7140
u7141:
	goto	l12231
u7140:
	line	69
	
l12213:	
;drive.c: 68: {
;drive.c: 69: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	70
;drive.c: 70: goReverse();
	fcall	_goReverse
	line	72
	
l12215:	
;drive.c: 72: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u7151
	goto	u7150
u7151:
	goto	l12223
u7150:
	line	74
	
l12217:	
;drive.c: 73: {
;drive.c: 74: somethingInTheWay = LEFT;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay)^080h,f	;volatile
	line	75
	
l12219:	
;drive.c: 75: turnRight90();
	fcall	_turnRight90
	line	76
	
l12221:	
;drive.c: 76: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	77
;drive.c: 77: }
	goto	l5827
	line	78
	
l5826:	
	
l12223:	
;drive.c: 78: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u7161
	goto	u7160
u7161:
	goto	l5828
u7160:
	line	80
	
l12225:	
;drive.c: 79: {
;drive.c: 80: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	line	81
	
l12227:	
;drive.c: 81: turnLeft90();
	fcall	_turnLeft90
	line	82
	
l12229:	
;drive.c: 82: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	83
;drive.c: 83: }
	goto	l5827
	line	84
	
l5828:	
	line	85
;drive.c: 84: else
;drive.c: 85: somethingInTheWay = FORWARD;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	goto	l5827
	
l5829:	
	
l5827:	
	line	86
;drive.c: 86: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l12231
	line	87
	
l5825:	
	goto	l12231
	line	88
	
l5821:	
	line	91
	
l12231:	
;drive.c: 87: }
;drive.c: 88: }
;drive.c: 91: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	92
;drive.c: 92: ser_putch(13);
	movlw	(0Dh)
	fcall	_ser_putch
	line	93
;drive.c: 93: virtualWall = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@virtualWall)	;volatile
	line	94
	
l12233:	
;drive.c: 94: if(virtualWall == 1)
	movf	(driveForDistance@virtualWall),w	;volatile
	xorlw	01h
	skipz
	goto	u7171
	goto	u7170
u7171:
	goto	l12253
u7170:
	line	96
	
l12235:	
;drive.c: 95: {
;drive.c: 96: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	97
;drive.c: 97: findFinalDestination(getCurrentX(),getCurrentY(), currentOrientation);
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
	line	98
;drive.c: 98: goReverse();
	fcall	_goReverse
	line	100
	
l12237:	
;drive.c: 100: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u7181
	goto	u7180
u7181:
	goto	l12245
u7180:
	line	102
	
l12239:	
;drive.c: 101: {
;drive.c: 102: somethingInTheWay = LEFT;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay)^080h,f	;volatile
	line	103
	
l12241:	
;drive.c: 103: turnRight90();
	fcall	_turnRight90
	line	104
	
l12243:	
;drive.c: 104: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	105
;drive.c: 105: }
	goto	l5832
	line	106
	
l5831:	
	
l12245:	
;drive.c: 106: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u7191
	goto	u7190
u7191:
	goto	l5833
u7190:
	line	108
	
l12247:	
;drive.c: 107: {
;drive.c: 108: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	line	109
	
l12249:	
;drive.c: 109: turnLeft90();
	fcall	_turnLeft90
	line	110
	
l12251:	
;drive.c: 110: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	111
;drive.c: 111: }
	goto	l5832
	line	112
	
l5833:	
	line	113
;drive.c: 112: else
;drive.c: 113: somethingInTheWay = FORWARD;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	goto	l5832
	
l5834:	
	
l5832:	
	line	114
;drive.c: 114: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l12253
	line	115
	
l5830:	
	line	118
	
l12253:	
;drive.c: 115: }
;drive.c: 118: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	119
;drive.c: 119: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	120
;drive.c: 120: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	121
;drive.c: 121: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	122
	
l12255:	
;drive.c: 122: deltaDistance = high*256 + low;
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
	line	123
	
l12257:	
;drive.c: 123: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	124
	
l12259:	
;drive.c: 124: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u7205
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u7205:

	skipc
	goto	u7201
	goto	u7200
u7201:
	goto	l12269
u7200:
	line	126
	
l12261:	
;drive.c: 125: {
;drive.c: 126: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	127
	
l12263:	
;drive.c: 127: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	line	128
	
l12265:	
;drive.c: 128: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	129
	
l12267:	
;drive.c: 129: somethingInTheWay = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	goto	l12269
	line	130
	
l5835:	
	goto	l12269
	line	131
	
l5819:	
	line	41
	
l12269:	
	btfsc	(_moving/8),(_moving)&7
	goto	u7211
	goto	u7210
u7211:
	goto	l12195
u7210:
	goto	l5837
	
l5836:	
	line	132
	
l5837:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_updateLocation
psect	text2207,local,class=CODE,delta=2
global __ptext2207
__ptext2207:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 278 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_getOrientation
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2207
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	278
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	279
	
l12163:	
;main.c: 279: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	280
;main.c: 280: switch(getOrientation())
	goto	l12183
	line	282
;main.c: 281: {
;main.c: 282: case NORTH:
	
l6765:	
	line	283
	
l12165:	
;main.c: 283: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	284
	
l12167:	
;main.c: 284: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	285
;main.c: 285: break;
	goto	l12185
	line	286
;main.c: 286: case SOUTH:
	
l6767:	
	line	287
	
l12169:	
;main.c: 287: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	288
	
l12171:	
;main.c: 288: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	289
;main.c: 289: break;
	goto	l12185
	line	290
;main.c: 290: case EAST:
	
l6768:	
	line	291
	
l12173:	
;main.c: 291: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	292
	
l12175:	
;main.c: 292: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	293
;main.c: 293: break;
	goto	l12185
	line	294
;main.c: 294: case WEST:
	
l6769:	
	line	295
	
l12177:	
;main.c: 295: --xCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_xCoord),f	;volatile
	line	296
	
l12179:	
;main.c: 296: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	297
;main.c: 297: break;
	goto	l12185
	line	298
;main.c: 298: default:
	
l6770:	
	line	299
;main.c: 299: break;
	goto	l12185
	line	300
	
l12181:	
;main.c: 300: }
	goto	l12185
	line	280
	
l6764:	
	
l12183:	
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
	goto	l12177
	xorlw	1^0	; case 1
	skipnz
	goto	l12169
	xorlw	2^1	; case 2
	skipnz
	goto	l12173
	xorlw	3^2	; case 3
	skipnz
	goto	l12165
	goto	l12185
	opt asmopt_on

	line	300
	
l6766:	
	line	302
	
l12185:	
;main.c: 302: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	303
;main.c: 303: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	304
;main.c: 304: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	305
;main.c: 305: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	306
	
l6771:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text2208,local,class=CODE,delta=2
global __ptext2208
__ptext2208:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 167 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  victim          1   16[BANK0 ] unsigned char 
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
;;      Temps:          0       2       0       0       0
;;      Totals:         0       3       0       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
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
psect	text2208
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	167
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 3
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	168
	
l12141:	
;main.c: 168: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	169
;main.c: 169: ser_putch(17);
	movlw	(011h)
	fcall	_ser_putch
	line	170
;main.c: 170: char victim = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(lookForVictim@victim)
	line	172
	
l12143:	
;main.c: 172: if(victim > 241 && victim != 255)
	movlw	(0F2h)
	subwf	(lookForVictim@victim),w
	skipc
	goto	u7071
	goto	u7070
u7071:
	goto	l6733
u7070:
	
l12145:	
	movf	(lookForVictim@victim),w
	xorlw	0FFh
	skipnz
	goto	u7081
	goto	u7080
u7081:
	goto	l6733
u7080:
	line	174
	
l12147:	
;main.c: 173: {
;main.c: 174: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7091
	goto	u7090
u7091:
	goto	l12157
u7090:
	line	176
	
l12149:	
;main.c: 175: {
;main.c: 176: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	177
	
l12151:	
;main.c: 177: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	178
	
l12153:	
;main.c: 178: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	179
	
l12155:	
;main.c: 179: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	180
;main.c: 180: }
	goto	l6733
	line	181
	
l6731:	
	line	183
	
l12157:	
;main.c: 181: else
;main.c: 182: {
;main.c: 183: victimZone = getVictimZone(xCoord, yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
	line	184
	
l12159:	
;main.c: 184: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	185
	
l12161:	
;main.c: 185: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l6733
	line	186
	
l6732:	
	goto	l6733
	line	187
	
l6730:	
	line	188
	
l6733:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text2209,local,class=CODE,delta=2
global __ptext2209
__ptext2209:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 156 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;;		_getFinalX
;;		_getFinalY
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2209
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	156
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 3
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	157
	
l12129:	
;main.c: 157: if((xCoord == getFinalX()) && (yCoord == getFinalY()))
	fcall	_getFinalX
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u7051
	goto	u7050
u7051:
	goto	l6727
u7050:
	
l12131:	
	fcall	_getFinalY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u7061
	goto	u7060
u7061:
	goto	l6727
u7060:
	line	159
	
l12133:	
;main.c: 158: {
;main.c: 159: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	160
	
l12135:	
;main.c: 160: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	161
	
l12137:	
;main.c: 161: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	162
	
l12139:	
;main.c: 162: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l6727
	line	163
	
l6726:	
	line	164
	
l6727:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text2210,local,class=CODE,delta=2
global __ptext2210
__ptext2210:

;; *************** function _init *****************
;; Defined at:
;;		line 117 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    5
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
psect	text2210
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	117
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	118
	
l12095:	
;main.c: 118: start.pressed = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	line	119
	
l12097:	
;main.c: 119: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	line	120
	
l12099:	
;main.c: 120: eeprom.pressed = 0;
	clrf	(_eeprom)^080h
	line	121
;main.c: 121: eeprom.released = 1;
	clrf	0+(_eeprom)^080h+01h
	bsf	status,0
	rlf	0+(_eeprom)^080h+01h,f
	line	123
	
l12101:	
;main.c: 123: init_adc();
	fcall	_init_adc
	line	124
	
l12103:	
;main.c: 124: lcd_init();
	fcall	_lcd_init
	line	126
	
l12105:	
;main.c: 126: TRISB = 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	129
	
l12107:	
;main.c: 129: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	131
	
l12109:	
;main.c: 131: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	132
	
l12111:	
;main.c: 132: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	133
	
l12113:	
;main.c: 133: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	134
	
l12115:	
;main.c: 134: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	135
	
l12117:	
;main.c: 135: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	138
	
l12119:	
;main.c: 138: PEIE = 1;
	bsf	(94/8),(94)&7
	line	139
	
l12121:	
;main.c: 139: GIE = 1;
	bsf	(95/8),(95)&7
	line	141
	
l12123:	
;main.c: 141: ser_init();
	fcall	_ser_init
	line	142
	
l12125:	
;main.c: 142: initIRobot();
	fcall	_initIRobot
	line	143
	
l12127:	
;main.c: 143: initSongs();
	fcall	_initSongs
	line	144
	
l6720:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_goReverse
psect	text2211,local,class=CODE,delta=2
global __ptext2211
__ptext2211:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 193 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text2211
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	193
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 1
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	194
	
l12085:	
;drive.c: 194: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	195
;drive.c: 195: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	196
	
l12087:	
;drive.c: 196: drive(255, 125, 128, 0);
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
	line	197
	
l12089:	
;drive.c: 197: waitFor(156,254,12);
	movlw	(0FEh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goReverse+0)+0
	movf	(??_goReverse+0)+0,w
	movwf	(?_waitFor)
	movlw	(0Ch)
	movwf	(??_goReverse+1)+0
	movf	(??_goReverse+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Ch)
	fcall	_waitFor
	line	198
	
l12091:	
;drive.c: 198: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	199
	
l12093:	
;drive.c: 199: _delay((unsigned long)((2000)*(20000000/4000.0)));
	opt asmopt_off
movlw  51
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_goReverse+0)+0+2),f
movlw	137
movwf	((??_goReverse+0)+0+1),f
	movlw	256
movwf	((??_goReverse+0)+0),f
u7837:
	decfsz	((??_goReverse+0)+0),f
	goto	u7837
	decfsz	((??_goReverse+0)+0+1),f
	goto	u7837
	decfsz	((??_goReverse+0)+0+2),f
	goto	u7837
opt asmopt_on

	line	200
	
l5863:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_readIR
psect	text2212,local,class=CODE,delta=2
global __ptext2212
__ptext2212:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  cm              2   43[BANK0 ] int 
;; Return value:  Size  Location     Type
;;                  2   41[BANK0 ] int 
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
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_adc_read_channel
;;		_convert
;; This function is called by:
;;		_frontWallCorrect
;;		_goParallel
;;		_findWall
;; This function uses a non-reentrant model
;;
psect	text2212
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 1
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l12079:	
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
	
l12081:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l5081
	
l12083:	
	line	36
	
l5081:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_findFinalDestination
psect	text2213,local,class=CODE,delta=2
global __ptext2213
__ptext2213:

;; *************** function _findFinalDestination *****************
;; Defined at:
;;		line 12 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
;; Parameters:    Size  Location     Type
;;  virtualWallX    1    wreg     unsigned char 
;;  virtualWallY    1   14[BANK0 ] unsigned char 
;;  robotOrienta    1   15[BANK0 ] enum E1088
;; Auto vars:     Size  Location     Type
;;  virtualWallX    1   17[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text2213
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	12
	global	__size_of_findFinalDestination
	__size_of_findFinalDestination	equ	__end_of_findFinalDestination-_findFinalDestination
	
_findFinalDestination:	
	opt	stack 1
; Regs used in _findFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;findFinalDestination@virtualWallX stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(findFinalDestination@virtualWallX)
	line	13
	
l11999:	
;map.c: 13: switch (virtualWallX)
	goto	l12075
	line	15
;map.c: 14: {
;map.c: 15: case 0:
	
l2849:	
	line	16
;map.c: 16: switch (virtualWallY)
	goto	l12009
	line	20
;map.c: 17: {
;map.c: 20: case 1:
	
l2851:	
	line	21
;map.c: 21: finalX = 0;
	clrf	(_finalX)
	line	22
	
l12001:	
;map.c: 22: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	23
;map.c: 23: break;
	goto	l12077
	line	24
;map.c: 24: case 2:
	
l2853:	
	line	25
;map.c: 25: finalX = 0;
	clrf	(_finalX)
	line	26
	
l12003:	
;map.c: 26: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	27
;map.c: 27: break;
	goto	l12077
	line	28
;map.c: 28: case 3:
	
l2854:	
	line	29
;map.c: 29: finalX = 0;
	clrf	(_finalX)
	line	30
	
l12005:	
;map.c: 30: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	31
;map.c: 31: break;
	goto	l12077
	line	32
;map.c: 32: default:
	
l2855:	
	line	33
;map.c: 33: break;
	goto	l12077
	line	34
	
l12007:	
;map.c: 34: }
	goto	l12077
	line	16
	
l2850:	
	
l12009:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l12077
	opt asmopt_on

	line	34
	
l2852:	
	line	35
;map.c: 35: break;
	goto	l12077
	line	37
;map.c: 37: case 1:
	
l2857:	
	line	38
;map.c: 38: switch (virtualWallY)
	goto	l12027
	line	40
;map.c: 39: {
;map.c: 40: case 0:
	
l2859:	
	line	41
	
l12011:	
;map.c: 41: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	42
	
l12013:	
;map.c: 42: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	43
;map.c: 43: break;
	goto	l12077
	line	44
;map.c: 44: case 1:
	
l2861:	
	line	45
	
l12015:	
;map.c: 45: finalX = 1;
	bcf	status, 5	;RP0=0, select bank0
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	46
;map.c: 46: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	47
;map.c: 47: break;
	goto	l12077
	line	48
;map.c: 48: case 2:
	
l2862:	
	line	49
	
l12017:	
;map.c: 49: finalX = 1;
	bcf	status, 5	;RP0=0, select bank0
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	50
	
l12019:	
;map.c: 50: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	51
;map.c: 51: break;
	goto	l12077
	line	52
;map.c: 52: case 3:
	
l2863:	
	line	53
	
l12021:	
;map.c: 53: finalX = 1;
	bcf	status, 5	;RP0=0, select bank0
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	54
	
l12023:	
;map.c: 54: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	55
;map.c: 55: break;
	goto	l12077
	line	56
;map.c: 56: default:
	
l2864:	
	line	57
;map.c: 57: break;
	goto	l12077
	line	58
	
l12025:	
;map.c: 58: }
	goto	l12077
	line	38
	
l2858:	
	
l12027:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l12011
	xorlw	1^0	; case 1
	skipnz
	goto	l12015
	xorlw	2^1	; case 2
	skipnz
	goto	l12017
	xorlw	3^2	; case 3
	skipnz
	goto	l12021
	goto	l12077
	opt asmopt_on

	line	58
	
l2860:	
	line	59
;map.c: 59: break;
	goto	l12077
	line	61
;map.c: 61: case 2:
	
l2865:	
	line	62
;map.c: 62: switch (virtualWallY)
	goto	l12045
	line	64
;map.c: 63: {
;map.c: 64: case 0:
	
l2867:	
	line	65
	
l12029:	
;map.c: 65: if(robotOrientation == WEST)
	movf	(findFinalDestination@robotOrientation),f
	skipz
	goto	u7021
	goto	u7020
u7021:
	goto	l12077
u7020:
	line	67
	
l12031:	
;map.c: 66: {
;map.c: 67: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	68
	
l12033:	
;map.c: 68: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	goto	l12077
	line	69
	
l2868:	
	line	70
;map.c: 69: }
;map.c: 70: break;
	goto	l12077
	line	71
;map.c: 71: case 1:
	
l2870:	
	line	72
	
l12035:	
;map.c: 72: finalX = 2;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	73
	
l12037:	
;map.c: 73: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	74
;map.c: 74: break;
	goto	l12077
	line	75
;map.c: 75: case 2:
	
l2871:	
	line	76
	
l12039:	
;map.c: 76: if(robotOrientation == EAST)
	bcf	status, 5	;RP0=0, select bank0
	movf	(findFinalDestination@robotOrientation),w
	xorlw	02h
	skipz
	goto	u7031
	goto	u7030
u7031:
	goto	l12077
u7030:
	line	78
	
l12041:	
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
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	goto	l12077
	line	80
	
l2872:	
	line	81
;map.c: 80: }
;map.c: 81: break;
	goto	l12077
	line	84
;map.c: 84: default:
	
l2873:	
	line	85
;map.c: 85: break;
	goto	l12077
	line	86
	
l12043:	
;map.c: 86: }
	goto	l12077
	line	62
	
l2866:	
	
l12045:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l12029
	xorlw	1^0	; case 1
	skipnz
	goto	l12035
	xorlw	2^1	; case 2
	skipnz
	goto	l12039
	goto	l12077
	opt asmopt_on

	line	86
	
l2869:	
	line	87
;map.c: 87: break;
	goto	l12077
	line	89
;map.c: 89: case 3:
	
l2874:	
	line	90
;map.c: 90: switch (virtualWallY)
	goto	l12055
	line	92
;map.c: 91: {
;map.c: 92: case 0:
	
l2876:	
	line	93
	
l12047:	
;map.c: 93: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	94
	
l12049:	
;map.c: 94: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	95
;map.c: 95: break;
	goto	l12077
	line	98
;map.c: 98: case 2:
	
l2878:	
	line	99
	
l12051:	
;map.c: 99: finalX = 3;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	100
;map.c: 100: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	101
;map.c: 101: break;
	goto	l12077
	line	104
;map.c: 104: default:
	
l2879:	
	line	105
;map.c: 105: break;
	goto	l12077
	line	106
	
l12053:	
;map.c: 106: }
	goto	l12077
	line	90
	
l2875:	
	
l12055:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l12047
	xorlw	2^0	; case 2
	skipnz
	goto	l12051
	goto	l12077
	opt asmopt_on

	line	106
	
l2877:	
	line	107
;map.c: 107: break;
	goto	l12077
	line	109
;map.c: 109: case 4:
	
l2880:	
	line	110
;map.c: 110: switch (virtualWallY)
	goto	l12071
	line	112
;map.c: 111: {
;map.c: 112: case 0:
	
l2882:	
	line	113
	
l12057:	
;map.c: 113: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	114
	
l12059:	
;map.c: 114: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	115
;map.c: 115: break;
	goto	l12077
	line	116
;map.c: 116: case 1:
	
l2884:	
	line	117
	
l12061:	
;map.c: 117: finalX = 4;
	movlw	(04h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	118
	
l12063:	
;map.c: 118: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	119
;map.c: 119: break;
	goto	l12077
	line	120
;map.c: 120: case 2:
	
l2885:	
	line	121
	
l12065:	
;map.c: 121: if (robotOrientation == SOUTH)
	bcf	status, 5	;RP0=0, select bank0
	movf	(findFinalDestination@robotOrientation),w
	xorlw	01h
	skipz
	goto	u7041
	goto	u7040
u7041:
	goto	l12077
u7040:
	line	123
	
l12067:	
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
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	goto	l12077
	line	125
	
l2886:	
	line	126
;map.c: 125: }
;map.c: 126: break;
	goto	l12077
	line	127
;map.c: 127: case 3:
	
l2887:	
	line	128
;map.c: 128: finalX = 0;
	clrf	(_finalX)
	line	129
;map.c: 129: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	130
;map.c: 130: break;
	goto	l12077
	line	131
;map.c: 131: default:
	
l2888:	
	line	132
;map.c: 132: break;
	goto	l12077
	line	133
	
l12069:	
;map.c: 133: }
	goto	l12077
	line	110
	
l2881:	
	
l12071:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l12057
	xorlw	1^0	; case 1
	skipnz
	goto	l12061
	xorlw	2^1	; case 2
	skipnz
	goto	l12065
	xorlw	3^2	; case 3
	skipnz
	goto	l2887
	goto	l12077
	opt asmopt_on

	line	133
	
l2883:	
	line	134
;map.c: 134: break;
	goto	l12077
	line	136
;map.c: 136: default:
	
l2889:	
	line	137
;map.c: 137: break;
	goto	l12077
	line	138
	
l12073:	
;map.c: 138: }
	goto	l12077
	line	13
	
l2848:	
	
l12075:	
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
	goto	l12009
	xorlw	1^0	; case 1
	skipnz
	goto	l12027
	xorlw	2^1	; case 2
	skipnz
	goto	l12045
	xorlw	3^2	; case 3
	skipnz
	goto	l12055
	xorlw	4^3	; case 4
	skipnz
	goto	l12071
	goto	l12077
	opt asmopt_on

	line	138
	
l2856:	
	line	140
	
l12077:	
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
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_finalY)^080h,w
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
psect	text2214,local,class=CODE,delta=2
global __ptext2214
__ptext2214:

;; *************** function _updateMapData *****************
;; Defined at:
;;		line 135 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;  virtualW        1    wreg     unsigned char 
;;  virtualS        1   19[BANK0 ] unsigned char 
;;  virtualE        1   20[BANK0 ] unsigned char 
;;  virtualN        1   21[BANK0 ] unsigned char 
;;  victim          1   22[BANK0 ] unsigned char 
;;  move            1   23[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  virtualW        1   26[BANK0 ] unsigned char 
;;  completeData    1   27[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_addNewData
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2214
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	135
	global	__size_of_updateMapData
	__size_of_updateMapData	equ	__end_of_updateMapData-_updateMapData
	
_updateMapData:	
	opt	stack 2
; Regs used in _updateMapData: [wreg+status,2+status,0+pclath+cstack]
;updateMapData@virtualW stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateMapData@virtualW)
	line	136
	
l11993:	
;eeprom.c: 136: char completeData = 0;
	clrf	(updateMapData@completeData)
	line	137
	
l11995:	
;eeprom.c: 137: completeData |= virtualW;
	movf	(updateMapData@virtualW),w
	movwf	(??_updateMapData+0)+0
	movf	(??_updateMapData+0)+0,w
	iorwf	(updateMapData@completeData),f
	line	138
;eeprom.c: 138: completeData |= virtualS << 1;
	movf	(updateMapData@virtualS),w
	movwf	(??_updateMapData+0)+0
	addwf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	139
;eeprom.c: 139: completeData |= virtualE << 2;
	movf	(updateMapData@virtualE),w
	movwf	(??_updateMapData+0)+0
	movlw	(02h)-1
u6985:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u6985
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	140
;eeprom.c: 140: completeData |= virtualN << 3;
	movf	(updateMapData@virtualN),w
	movwf	(??_updateMapData+0)+0
	movlw	(03h)-1
u6995:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u6995
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	141
;eeprom.c: 141: completeData |= victim << 4;
	movf	(updateMapData@victim),w
	movwf	(??_updateMapData+0)+0
	movlw	(04h)-1
u7005:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u7005
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	142
;eeprom.c: 142: completeData |= move << 5;
	movf	(updateMapData@move),w
	movwf	(??_updateMapData+0)+0
	movlw	(05h)-1
u7015:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u7015
	clrc
	rlf	(??_updateMapData+0)+0,w
	movwf	(??_updateMapData+1)+0
	movf	(??_updateMapData+1)+0,w
	iorwf	(updateMapData@completeData),f
	line	143
;eeprom.c: 143: completeData &= 0b01111111;
	movlw	(07Fh)
	movwf	(??_updateMapData+0)+0
	movf	(??_updateMapData+0)+0,w
	andwf	(updateMapData@completeData),f
	line	144
	
l11997:	
;eeprom.c: 144: addNewData(completeData);
	movf	(updateMapData@completeData),w
	fcall	_addNewData
	line	145
	
l1430:	
	return
	opt stack 0
GLOBAL	__end_of_updateMapData
	__end_of_updateMapData:
;; =============== function _updateMapData ends ============

	signat	_updateMapData,24696
	global	_checkIfHome
psect	text2215,local,class=CODE,delta=2
global __ptext2215
__ptext2215:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 321 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;;		_drive
;;		_play_iCreate_song
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2215
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	321
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 3
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	322
	
l11985:	
;main.c: 322: if((xCoord == 1) && (yCoord == 3))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u6961
	goto	u6960
u6961:
	goto	l6784
u6960:
	
l11987:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u6971
	goto	u6970
u6971:
	goto	l6784
u6970:
	line	324
	
l11989:	
;main.c: 323: {
;main.c: 324: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	325
;main.c: 325: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	326
	
l11991:	
;main.c: 326: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l6784
	line	327
	
l6783:	
	line	328
	
l6784:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_turnAround
psect	text2216,local,class=CODE,delta=2
global __ptext2216
__ptext2216:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 214 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_goBackward
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2216
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	214
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	215
	
l11979:	
;drive.c: 215: drive(0, 50, 0, 1);
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
	line	216
;drive.c: 216: waitFor(157,0,180);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0B4h)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	217
;drive.c: 217: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	218
	
l11981:	
;drive.c: 218: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  76
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	206
movwf	((??_turnAround+0)+0+1),f
	movlw	129
movwf	((??_turnAround+0)+0),f
u7847:
	decfsz	((??_turnAround+0)+0),f
	goto	u7847
	decfsz	((??_turnAround+0)+0+1),f
	goto	u7847
	decfsz	((??_turnAround+0)+0+2),f
	goto	u7847
	clrwdt
opt asmopt_on

	line	219
	
l11983:	
;drive.c: 219: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  76
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	206
movwf	((??_turnAround+0)+0+1),f
	movlw	129
movwf	((??_turnAround+0)+0),f
u7857:
	decfsz	((??_turnAround+0)+0),f
	goto	u7857
	decfsz	((??_turnAround+0)+0+1),f
	goto	u7857
	decfsz	((??_turnAround+0)+0+2),f
	goto	u7857
	clrwdt
opt asmopt_on

	line	220
	
l5869:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_turnLeft90
psect	text2217,local,class=CODE,delta=2
global __ptext2217
__ptext2217:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 223 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;; Hardware stack levels required when called:    4
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
psect	text2217
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	223
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 1
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	224
	
l11967:	
;drive.c: 224: drive(0, 50, 0, 1);
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
	line	225
	
l11969:	
;drive.c: 225: if( (getCurrentX() == 2 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	02h
	skipz
	goto	u6941
	goto	u6940
u6941:
	goto	l11975
u6940:
	
l11971:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u6951
	goto	u6950
u6951:
	goto	l11975
u6950:
	line	227
	
l11973:	
;drive.c: 226: {
;drive.c: 227: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	228
;drive.c: 228: }else
	goto	l5873
	
l5872:	
	line	230
	
l11975:	
;drive.c: 229: {
;drive.c: 230: waitFor(157,0,90);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(05Ah)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	231
	
l5873:	
	line	232
;drive.c: 231: }
;drive.c: 232: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	233
	
l11977:	
;drive.c: 233: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  76
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	206
movwf	((??_turnLeft90+0)+0+1),f
	movlw	129
movwf	((??_turnLeft90+0)+0),f
u7867:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u7867
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u7867
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u7867
	clrwdt
opt asmopt_on

	line	234
	
l5874:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnRight90
psect	text2218,local,class=CODE,delta=2
global __ptext2218
__ptext2218:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 237 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_driveForDistance
;;		_goRight
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2218
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	237
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 1
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	238
	
l11963:	
;drive.c: 238: drive(0, 50, 255, 255);
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
	line	239
;drive.c: 239: waitFor(157,255,174);
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
	line	240
;drive.c: 240: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	241
	
l11965:	
;drive.c: 241: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  76
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	206
movwf	((??_turnRight90+0)+0+1),f
	movlw	129
movwf	((??_turnRight90+0)+0),f
u7877:
	decfsz	((??_turnRight90+0)+0),f
	goto	u7877
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u7877
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u7877
	clrwdt
opt asmopt_on

	line	242
	
l5877:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_initSongs
psect	text2219,local,class=CODE,delta=2
global __ptext2219
__ptext2219:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 31 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
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
;;		_ser_putArr
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text2219
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	31
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	32
	
l11961:	
;songs.c: 32: ser_putArr(finalCountdown, 27);
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
	line	33
;songs.c: 33: ser_putArr(superMarioBros, 25);
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
	line	34
;songs.c: 34: ser_putArr(lookingForU2, 29);
	movlw	(_lookingForU2&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x2/2)
	movwf	(?_ser_putArr+1)
	movlw	low(01Dh)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(01Dh)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	35
;songs.c: 35: ser_putArr(champions, 21);
	movlw	(_champions&0ffh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_ser_putArr)
	movlw	(0x3/2)
	movwf	(?_ser_putArr+1)
	movlw	low(015h)
	movwf	0+(?_ser_putArr)+02h
	movlw	high(015h)
	movwf	(0+(?_ser_putArr)+02h)+1
	fcall	_ser_putArr
	line	36
;songs.c: 36: ser_putArr(beep, 5);
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
	line	37
	
l4375:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text2220,local,class=CODE,delta=2
global __ptext2220
__ptext2220:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_write_control
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text2220
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l11941:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l11943:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l11945:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l11947:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l11949:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l11951:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l11953:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l11955:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l11957:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l11959:	
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
psect	text2221,local,class=CODE,delta=2
global __ptext2221
__ptext2221:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1   13[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_write_data
;; This function is called by:
;;		_findFinalDestination
;;		_lookForVictim
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text2221
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	44
	global	__size_of_lcd_write_1_digit_bcd
	__size_of_lcd_write_1_digit_bcd	equ	__end_of_lcd_write_1_digit_bcd-_lcd_write_1_digit_bcd
	
_lcd_write_1_digit_bcd:	
	opt	stack 3
; Regs used in _lcd_write_1_digit_bcd: [wreg+status,2+status,0+pclath+cstack]
;lcd_write_1_digit_bcd@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_1_digit_bcd@data)
	line	45
	
l11939:	
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
psect	text2222,local,class=CODE,delta=2
global __ptext2222
__ptext2222:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
;; Parameters:    Size  Location     Type
;;  address         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  address         1   13[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
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
psect	text2222
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	32
	global	__size_of_lcd_set_cursor
	__size_of_lcd_set_cursor	equ	__end_of_lcd_set_cursor-_lcd_set_cursor
	
_lcd_set_cursor:	
	opt	stack 3
; Regs used in _lcd_set_cursor: [wreg+status,2+status,0+pclath+cstack]
;lcd_set_cursor@address stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_set_cursor@address)
	line	33
	
l11935:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l11937:	
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
psect	text2223,local,class=CODE,delta=2
global __ptext2223
__ptext2223:

;; *************** function _EEPROMToSerial *****************
;; Defined at:
;;		line 149 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  transferDone    1   20[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_readEEPROM
;;		_ser_putch
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2223
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	149
	global	__size_of_EEPROMToSerial
	__size_of_EEPROMToSerial	equ	__end_of_EEPROMToSerial-_EEPROMToSerial
	
_EEPROMToSerial:	
	opt	stack 3
; Regs used in _EEPROMToSerial: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	150
	
l11909:	
;eeprom.c: 150: char transferDone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(EEPROMToSerial@transferDone)
	line	151
;eeprom.c: 151: addressCurrent = 0;
	clrf	(_addressCurrent)
	line	152
	
l11911:	
;eeprom.c: 152: addressCount = readEEPROM(0,0);
	clrf	(?_readEEPROM)
	movlw	(0)
	fcall	_readEEPROM
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_EEPROMToSerial+0)+0
	movf	(??_EEPROMToSerial+0)+0,w
	movwf	(_addressCount)
	line	153
	
l11913:	
;eeprom.c: 153: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	136
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u7887:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u7887
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u7887
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u7887
opt asmopt_on

	line	155
	
l11915:	
;eeprom.c: 155: ser_putch(254);
	movlw	(0FEh)
	fcall	_ser_putch
	line	157
;eeprom.c: 157: while(!transferDone && addressCount > 0)
	goto	l1433
	
l1434:	
	line	159
	
l11917:	
;eeprom.c: 158: {
;eeprom.c: 159: ser_putch(readEEPROM(0,1 + addressCurrent));
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
	line	160
	
l11919:	
;eeprom.c: 160: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	136
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u7897:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u7897
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u7897
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u7897
opt asmopt_on

	line	161
	
l11921:	
;eeprom.c: 161: addressCurrent ++;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_EEPROMToSerial+0)+0
	movf	(??_EEPROMToSerial+0)+0,w
	addwf	(_addressCurrent),f
	line	162
	
l11923:	
;eeprom.c: 162: if(addressCurrent >= (addressCount))
	movf	(_addressCount),w
	subwf	(_addressCurrent),w
	skipc
	goto	u6911
	goto	u6910
u6911:
	goto	l1433
u6910:
	line	164
	
l11925:	
;eeprom.c: 163: {
;eeprom.c: 164: transferDone = 1;
	clrf	(EEPROMToSerial@transferDone)
	bsf	status,0
	rlf	(EEPROMToSerial@transferDone),f
	goto	l1433
	line	165
	
l1435:	
	line	166
	
l1433:	
	line	157
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(EEPROMToSerial@transferDone),f
	skipz
	goto	u6921
	goto	u6920
u6921:
	goto	l11929
u6920:
	
l11927:	
	movf	(_addressCount),f
	skipz
	goto	u6931
	goto	u6930
u6931:
	goto	l11917
u6930:
	goto	l11929
	
l1437:	
	goto	l11929
	
l1438:	
	line	168
	
l11929:	
;eeprom.c: 165: }
;eeprom.c: 166: }
;eeprom.c: 168: ser_putch(255);
	movlw	(0FFh)
	fcall	_ser_putch
	line	169
	
l11931:	
;eeprom.c: 169: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_EEPROMToSerial+0)+0+2),f
movlw	136
movwf	((??_EEPROMToSerial+0)+0+1),f
	movlw	86
movwf	((??_EEPROMToSerial+0)+0),f
u7907:
	decfsz	((??_EEPROMToSerial+0)+0),f
	goto	u7907
	decfsz	((??_EEPROMToSerial+0)+0+1),f
	goto	u7907
	decfsz	((??_EEPROMToSerial+0)+0+2),f
	goto	u7907
opt asmopt_on

	line	170
	
l11933:	
;eeprom.c: 170: ser_putch(255);
	movlw	(0FFh)
	fcall	_ser_putch
	line	171
	
l1439:	
	return
	opt stack 0
GLOBAL	__end_of_EEPROMToSerial
	__end_of_EEPROMToSerial:
;; =============== function _EEPROMToSerial ends ============

	signat	_EEPROMToSerial,88
	global	_addNewData
psect	text2224,local,class=CODE,delta=2
global __ptext2224
__ptext2224:

;; *************** function _addNewData *****************
;; Defined at:
;;		line 94 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1   18[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_writeEEPROM
;; This function is called by:
;;		_updateMapData
;;		_writeEEPROMTestData
;; This function uses a non-reentrant model
;;
psect	text2224
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	94
	global	__size_of_addNewData
	__size_of_addNewData	equ	__end_of_addNewData-_addNewData
	
_addNewData:	
	opt	stack 2
; Regs used in _addNewData: [wreg+status,2+status,0+pclath+cstack]
;addNewData@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(addNewData@data)
	line	95
	
l11903:	
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
	
l11905:	
;eeprom.c: 96: addressCount ++;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_addNewData+0)+0
	movf	(??_addNewData+0)+0,w
	addwf	(_addressCount),f
	line	97
	
l11907:	
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
psect	text2225,local,class=CODE,delta=2
global __ptext2225
__ptext2225:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
;; Parameters:    Size  Location     Type
;;  s               1    wreg     PTR const unsigned char 
;;		 -> STR_4(17), STR_3(17), STR_2(14), STR_1(15), 
;; Auto vars:     Size  Location     Type
;;  s               1   14[BANK0 ] PTR const unsigned char 
;;		 -> STR_4(17), STR_3(17), STR_2(14), STR_1(15), 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_lcd_write_data
;; This function is called by:
;;		_main
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text2225
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	38
	global	__size_of_lcd_write_string
	__size_of_lcd_write_string	equ	__end_of_lcd_write_string-_lcd_write_string
	
_lcd_write_string:	
	opt	stack 4
; Regs used in _lcd_write_string: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;lcd_write_string@s stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_string@s)
	line	40
	
l11895:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l11901
	
l2136:	
	
l11897:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l11899:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l11901
	
l2135:	
	
l11901:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u6901
	goto	u6900
u6901:
	goto	l11897
u6900:
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
psect	text2226,local,class=CODE,delta=2
global __ptext2226
__ptext2226:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
;; Parameters:    Size  Location     Type
;;  channel         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  channel         1   40[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   37[BANK0 ] int 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_adc_read
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text2226
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
	line	7
	global	__size_of_adc_read_channel
	__size_of_adc_read_channel	equ	__end_of_adc_read_channel-_adc_read_channel
	
_adc_read_channel:	
	opt	stack 1
; Regs used in _adc_read_channel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
;adc_read_channel@channel stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(adc_read_channel@channel)
	line	8
	
l10461:	
;adc.c: 8: switch(channel)
	goto	l10469
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
	goto	l10471
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
	goto	l10471
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
	goto	l10471
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
	goto	l10471
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
	goto	l10471
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l10463:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l10465:	
	goto	l697
	line	39
	
l10467:	
;adc.c: 39: }
	goto	l10471
	line	8
	
l689:	
	
l10469:	
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
	goto	l10463
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l10471:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u7917:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u7917
opt asmopt_on

	line	43
	
l10473:	
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
	
l10475:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	___lbtoft
psect	text2227,local,class=CODE,delta=2
global __ptext2227
__ptext2227:

;; *************** function ___lbtoft *****************
;; Defined at:
;;		line 28 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\lbtoft.c"
;; Parameters:    Size  Location     Type
;;  c               1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  c               1   39[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  3   32[BANK0 ] float 
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       3       0       0       0
;;      Locals:         0       1       0       0       0
;;      Temps:          0       4       0       0       0
;;      Totals:         0       8       0       0       0
;;Total ram usage:        8 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		___ftpack
;; This function is called by:
;;		_goParallel
;; This function uses a non-reentrant model
;;
psect	text2227
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\lbtoft.c"
	line	28
	global	__size_of___lbtoft
	__size_of___lbtoft	equ	__end_of___lbtoft-___lbtoft
	
___lbtoft:	
	opt	stack 3
; Regs used in ___lbtoft: [wreg+status,2+status,0+pclath+cstack]
;___lbtoft@c stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(___lbtoft@c)
	line	29
	
l11891:	
	movf	(___lbtoft@c),w
	movwf	((??___lbtoft+0)+0)
	clrf	((??___lbtoft+0)+0+1)
	clrf	((??___lbtoft+0)+0+2)
	movf	0+(??___lbtoft+0)+0,w
	movwf	(?___ftpack)
	movf	1+(??___lbtoft+0)+0,w
	movwf	(?___ftpack+1)
	movf	2+(??___lbtoft+0)+0,w
	movwf	(?___ftpack+2)
	movlw	(08Eh)
	movwf	(??___lbtoft+3)+0
	movf	(??___lbtoft+3)+0,w
	movwf	0+(?___ftpack)+03h
	clrf	0+(?___ftpack)+04h
	fcall	___ftpack
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___ftpack)),w
	movwf	(?___lbtoft)
	movf	(1+(?___ftpack)),w
	movwf	(?___lbtoft+1)
	movf	(2+(?___ftpack)),w
	movwf	(?___lbtoft+2)
	goto	l6976
	
l11893:	
	line	30
	
l6976:	
	return
	opt stack 0
GLOBAL	__end_of___lbtoft
	__end_of___lbtoft:
;; =============== function ___lbtoft ends ============

	signat	___lbtoft,4219
	global	___ftmul
psect	text2228,local,class=CODE,delta=2
global __ptext2228
__ptext2228:

;; *************** function ___ftmul *****************
;; Defined at:
;;		line 52 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\ftmul.c"
;; Parameters:    Size  Location     Type
;;  f1              3   40[BANK0 ] float 
;;  f2              3   43[BANK0 ] float 
;; Auto vars:     Size  Location     Type
;;  f3_as_produc    3   51[BANK0 ] unsigned um
;;  sign            1   55[BANK0 ] unsigned char 
;;  cntr            1   54[BANK0 ] unsigned char 
;;  exp             1   50[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  3   40[BANK0 ] float 
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       6       0       0       0
;;      Locals:         0       6       0       0       0
;;      Temps:          0       4       0       0       0
;;      Totals:         0      16       0       0       0
;;Total ram usage:       16 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		___ftpack
;; This function is called by:
;;		_goParallel
;; This function uses a non-reentrant model
;;
psect	text2228
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\ftmul.c"
	line	52
	global	__size_of___ftmul
	__size_of___ftmul	equ	__end_of___ftmul-___ftmul
	
___ftmul:	
	opt	stack 3
; Regs used in ___ftmul: [wreg+status,2+status,0+pclath+cstack]
	line	56
	
l11841:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(___ftmul@f1),w
	movwf	((??___ftmul+0)+0)
	movf	(___ftmul@f1+1),w
	movwf	((??___ftmul+0)+0+1)
	movf	(___ftmul@f1+2),w
	movwf	((??___ftmul+0)+0+2)
	clrc
	rlf	(??___ftmul+0)+1,w
	rlf	(??___ftmul+0)+2,w
	movwf	(??___ftmul+3)+0
	movf	(??___ftmul+3)+0,w
	movwf	(___ftmul@exp)
	movf	((___ftmul@exp)),f
	skipz
	goto	u6761
	goto	u6760
u6761:
	goto	l11847
u6760:
	line	57
	
l11843:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l6950
	
l11845:	
	goto	l6950
	
l6949:	
	line	58
	
l11847:	
	movf	(___ftmul@f2),w
	movwf	((??___ftmul+0)+0)
	movf	(___ftmul@f2+1),w
	movwf	((??___ftmul+0)+0+1)
	movf	(___ftmul@f2+2),w
	movwf	((??___ftmul+0)+0+2)
	clrc
	rlf	(??___ftmul+0)+1,w
	rlf	(??___ftmul+0)+2,w
	movwf	(??___ftmul+3)+0
	movf	(??___ftmul+3)+0,w
	movwf	(___ftmul@sign)
	movf	((___ftmul@sign)),f
	skipz
	goto	u6771
	goto	u6770
u6771:
	goto	l11853
u6770:
	line	59
	
l11849:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l6950
	
l11851:	
	goto	l6950
	
l6951:	
	line	60
	
l11853:	
	movf	(___ftmul@sign),w
	addlw	07Bh
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	addwf	(___ftmul@exp),f
	line	61
	movf	(___ftmul@f1),w
	movwf	((??___ftmul+0)+0)
	movf	(___ftmul@f1+1),w
	movwf	((??___ftmul+0)+0+1)
	movf	(___ftmul@f1+2),w
	movwf	((??___ftmul+0)+0+2)
	movlw	010h
u6785:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6780:
	addlw	-1
	skipz
	goto	u6785
	movf	0+(??___ftmul+0)+0,w
	movwf	(??___ftmul+3)+0
	movf	(??___ftmul+3)+0,w
	movwf	(___ftmul@sign)
	line	62
	movf	(___ftmul@f2),w
	movwf	((??___ftmul+0)+0)
	movf	(___ftmul@f2+1),w
	movwf	((??___ftmul+0)+0+1)
	movf	(___ftmul@f2+2),w
	movwf	((??___ftmul+0)+0+2)
	movlw	010h
u6795:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6790:
	addlw	-1
	skipz
	goto	u6795
	movf	0+(??___ftmul+0)+0,w
	movwf	(??___ftmul+3)+0
	movf	(??___ftmul+3)+0,w
	xorwf	(___ftmul@sign),f
	line	63
	movlw	(080h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	andwf	(___ftmul@sign),f
	line	64
	
l11855:	
	bsf	(___ftmul@f1)+(15/8),(15)&7
	line	66
	
l11857:	
	bsf	(___ftmul@f2)+(15/8),(15)&7
	line	67
	
l11859:	
	movlw	0FFh
	andwf	(___ftmul@f2),f
	movlw	0FFh
	andwf	(___ftmul@f2+1),f
	movlw	0
	andwf	(___ftmul@f2+2),f
	line	68
	
l11861:	
	movlw	0
	movwf	(___ftmul@f3_as_product)
	movlw	0
	movwf	(___ftmul@f3_as_product+1)
	movlw	0
	movwf	(___ftmul@f3_as_product+2)
	line	69
	
l11863:	
	movlw	(07h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l11865
	line	70
	
l6952:	
	line	71
	
l11865:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6801
	goto	u6800
u6801:
	goto	l11869
u6800:
	line	72
	
l11867:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6811
	addwf	(___ftmul@f3_as_product+1),f
u6811:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6812
	addwf	(___ftmul@f3_as_product+2),f
u6812:

	goto	l11869
	
l6953:	
	line	73
	
l11869:	
	movlw	01h
u6825:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6825

	line	74
	
l11871:	
	movlw	01h
u6835:
	clrc
	rlf	(___ftmul@f2),f
	rlf	(___ftmul@f2+1),f
	rlf	(___ftmul@f2+2),f
	addlw	-1
	skipz
	goto	u6835
	line	75
	
l11873:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6841
	goto	u6840
u6841:
	goto	l11865
u6840:
	goto	l11875
	
l6954:	
	line	76
	
l11875:	
	movlw	(09h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l11877
	line	77
	
l6955:	
	line	78
	
l11877:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6851
	goto	u6850
u6851:
	goto	l11881
u6850:
	line	79
	
l11879:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6861
	addwf	(___ftmul@f3_as_product+1),f
u6861:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6862
	addwf	(___ftmul@f3_as_product+2),f
u6862:

	goto	l11881
	
l6956:	
	line	80
	
l11881:	
	movlw	01h
u6875:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6875

	line	81
	
l11883:	
	movlw	01h
u6885:
	clrc
	rrf	(___ftmul@f3_as_product+2),f
	rrf	(___ftmul@f3_as_product+1),f
	rrf	(___ftmul@f3_as_product),f
	addlw	-1
	skipz
	goto	u6885

	line	82
	
l11885:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6891
	goto	u6890
u6891:
	goto	l11877
u6890:
	goto	l11887
	
l6957:	
	line	83
	
l11887:	
	movf	(___ftmul@f3_as_product),w
	movwf	(?___ftpack)
	movf	(___ftmul@f3_as_product+1),w
	movwf	(?___ftpack+1)
	movf	(___ftmul@f3_as_product+2),w
	movwf	(?___ftpack+2)
	movf	(___ftmul@exp),w
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	0+(?___ftpack)+03h
	movf	(___ftmul@sign),w
	movwf	(??___ftmul+1)+0
	movf	(??___ftmul+1)+0,w
	movwf	0+(?___ftpack)+04h
	fcall	___ftpack
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___ftpack)),w
	movwf	(?___ftmul)
	movf	(1+(?___ftpack)),w
	movwf	(?___ftmul+1)
	movf	(2+(?___ftpack)),w
	movwf	(?___ftmul+2)
	goto	l6950
	
l11889:	
	line	84
	
l6950:	
	return
	opt stack 0
GLOBAL	__end_of___ftmul
	__end_of___ftmul:
;; =============== function ___ftmul ends ============

	signat	___ftmul,8315
	global	___ftadd
psect	text2229,local,class=CODE,delta=2
global __ptext2229
__ptext2229:

;; *************** function ___ftadd *****************
;; Defined at:
;;		line 87 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\ftadd.c"
;; Parameters:    Size  Location     Type
;;  f1              3    0[BANK1 ] float 
;;  f2              3    3[BANK1 ] float 
;; Auto vars:     Size  Location     Type
;;  exp1            1    8[BANK1 ] unsigned char 
;;  exp2            1    7[BANK1 ] unsigned char 
;;  sign            1    6[BANK1 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  3    0[BANK1 ] float 
;; Registers used:
;;		wreg, status,2, status,0, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       6       0       0
;;      Locals:         0       0       3       0       0
;;      Temps:          0       4       0       0       0
;;      Totals:         0       4       9       0       0
;;Total ram usage:       13 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    3
;; This function calls:
;;		___ftpack
;; This function is called by:
;;		_goParallel
;;		___ftsub
;; This function uses a non-reentrant model
;;
psect	text2229
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\ftadd.c"
	line	87
	global	__size_of___ftadd
	__size_of___ftadd	equ	__end_of___ftadd-___ftadd
	
___ftadd:	
	opt	stack 3
; Regs used in ___ftadd: [wreg+status,2+status,0+pclath+cstack]
	line	90
	
l11771:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f1+1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0+1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f1+2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0+2)
	clrc
	rlf	(??___ftadd+0)+1,w
	rlf	(??___ftadd+0)+2,w
	movwf	(??___ftadd+3)+0
	movf	(??___ftadd+3)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(___ftadd@exp1)^080h
	line	91
	movf	(___ftadd@f2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2+1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0+1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2+2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??___ftadd+0)+0+2)
	clrc
	rlf	(??___ftadd+0)+1,w
	rlf	(??___ftadd+0)+2,w
	movwf	(??___ftadd+3)+0
	movf	(??___ftadd+3)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(___ftadd@exp2)^080h
	line	92
	
l11773:	
	movf	(___ftadd@exp1)^080h,w
	skipz
	goto	u6520
	goto	l11779
u6520:
	
l11775:	
	movf	(___ftadd@exp2)^080h,w
	subwf	(___ftadd@exp1)^080h,w
	skipnc
	goto	u6531
	goto	u6530
u6531:
	goto	l11783
u6530:
	
l11777:	
	decf	(___ftadd@exp1)^080h,w
	xorlw	0ffh
	addwf	(___ftadd@exp2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movlw	(019h)
	subwf	0+(??___ftadd+0)+0,w
	skipc
	goto	u6541
	goto	u6540
u6541:
	goto	l11783
u6540:
	goto	l11779
	
l6897:	
	line	93
	
l11779:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2)^080h,w
	movwf	(?___ftadd)^080h
	movf	(___ftadd@f2+1)^080h,w
	movwf	(?___ftadd+1)^080h
	movf	(___ftadd@f2+2)^080h,w
	movwf	(?___ftadd+2)^080h
	goto	l6898
	
l11781:	
	goto	l6898
	
l6895:	
	line	94
	
l11783:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6550
	goto	l6901
u6550:
	
l11785:	
	movf	(___ftadd@exp1)^080h,w
	subwf	(___ftadd@exp2)^080h,w
	skipnc
	goto	u6561
	goto	u6560
u6561:
	goto	l11789
u6560:
	
l11787:	
	decf	(___ftadd@exp2)^080h,w
	xorlw	0ffh
	addwf	(___ftadd@exp1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movlw	(019h)
	subwf	0+(??___ftadd+0)+0,w
	skipc
	goto	u6571
	goto	u6570
u6571:
	goto	l11789
u6570:
	
l6901:	
	line	95
	goto	l6898
	
l6899:	
	line	96
	
l11789:	
	movlw	(06h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(___ftadd@sign)^080h
	line	97
	
l11791:	
	btfss	(___ftadd@f1+2)^080h,(23)&7
	goto	u6581
	goto	u6580
u6581:
	goto	l6902
u6580:
	line	98
	
l11793:	
	bsf	(___ftadd@sign)^080h+(7/8),(7)&7
	
l6902:	
	line	99
	btfss	(___ftadd@f2+2)^080h,(23)&7
	goto	u6591
	goto	u6590
u6591:
	goto	l6903
u6590:
	line	100
	
l11795:	
	bsf	(___ftadd@sign)^080h+(6/8),(6)&7
	
l6903:	
	line	101
	bsf	(___ftadd@f1)^080h+(15/8),(15)&7
	line	102
	
l11797:	
	movlw	0FFh
	andwf	(___ftadd@f1)^080h,f
	movlw	0FFh
	andwf	(___ftadd@f1+1)^080h,f
	movlw	0
	andwf	(___ftadd@f1+2)^080h,f
	line	103
	
l11799:	
	bsf	(___ftadd@f2)^080h+(15/8),(15)&7
	line	104
	movlw	0FFh
	andwf	(___ftadd@f2)^080h,f
	movlw	0FFh
	andwf	(___ftadd@f2+1)^080h,f
	movlw	0
	andwf	(___ftadd@f2+2)^080h,f
	line	106
	movf	(___ftadd@exp2)^080h,w
	subwf	(___ftadd@exp1)^080h,w
	skipnc
	goto	u6601
	goto	u6600
u6601:
	goto	l11811
u6600:
	goto	l11801
	line	109
	
l6905:	
	line	110
	
l11801:	
	movlw	01h
u6615:
	clrc
	rlf	(___ftadd@f2)^080h,f
	rlf	(___ftadd@f2+1)^080h,f
	rlf	(___ftadd@f2+2)^080h,f
	addlw	-1
	skipz
	goto	u6615
	line	111
	movlw	low(01h)
	subwf	(___ftadd@exp2)^080h,f
	line	112
	
l11803:	
	movf	(___ftadd@exp2)^080h,w
	xorwf	(___ftadd@exp1)^080h,w
	skipnz
	goto	u6621
	goto	u6620
u6621:
	goto	l11809
u6620:
	
l11805:	
	movlw	low(01h)
	subwf	(___ftadd@sign)^080h,f
	movf	((___ftadd@sign)^080h),w
	andlw	07h
	btfss	status,2
	goto	u6631
	goto	u6630
u6631:
	goto	l11801
u6630:
	goto	l11809
	
l6907:	
	goto	l11809
	
l6908:	
	line	113
	goto	l11809
	
l6910:	
	line	114
	
l11807:	
	movlw	01h
u6645:
	clrc
	rrf	(___ftadd@f1+2)^080h,f
	rrf	(___ftadd@f1+1)^080h,f
	rrf	(___ftadd@f1)^080h,f
	addlw	-1
	skipz
	goto	u6645

	line	115
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(___ftadd@exp1)^080h,f
	goto	l11809
	line	116
	
l6909:	
	line	113
	
l11809:	
	movf	(___ftadd@exp1)^080h,w
	xorwf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6651
	goto	u6650
u6651:
	goto	l11807
u6650:
	goto	l6912
	
l6911:	
	line	117
	goto	l6912
	
l6904:	
	
l11811:	
	movf	(___ftadd@exp1)^080h,w
	subwf	(___ftadd@exp2)^080h,w
	skipnc
	goto	u6661
	goto	u6660
u6661:
	goto	l6912
u6660:
	goto	l11813
	line	120
	
l6914:	
	line	121
	
l11813:	
	movlw	01h
u6675:
	clrc
	rlf	(___ftadd@f1)^080h,f
	rlf	(___ftadd@f1+1)^080h,f
	rlf	(___ftadd@f1+2)^080h,f
	addlw	-1
	skipz
	goto	u6675
	line	122
	movlw	low(01h)
	subwf	(___ftadd@exp1)^080h,f
	line	123
	
l11815:	
	movf	(___ftadd@exp2)^080h,w
	xorwf	(___ftadd@exp1)^080h,w
	skipnz
	goto	u6681
	goto	u6680
u6681:
	goto	l11821
u6680:
	
l11817:	
	movlw	low(01h)
	subwf	(___ftadd@sign)^080h,f
	movf	((___ftadd@sign)^080h),w
	andlw	07h
	btfss	status,2
	goto	u6691
	goto	u6690
u6691:
	goto	l11813
u6690:
	goto	l11821
	
l6916:	
	goto	l11821
	
l6917:	
	line	124
	goto	l11821
	
l6919:	
	line	125
	
l11819:	
	movlw	01h
u6705:
	clrc
	rrf	(___ftadd@f2+2)^080h,f
	rrf	(___ftadd@f2+1)^080h,f
	rrf	(___ftadd@f2)^080h,f
	addlw	-1
	skipz
	goto	u6705

	line	126
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(___ftadd@exp2)^080h,f
	goto	l11821
	line	127
	
l6918:	
	line	124
	
l11821:	
	movf	(___ftadd@exp1)^080h,w
	xorwf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6711
	goto	u6710
u6711:
	goto	l11819
u6710:
	goto	l6912
	
l6920:	
	goto	l6912
	line	128
	
l6913:	
	line	129
	
l6912:	
	btfss	(___ftadd@sign)^080h,(7)&7
	goto	u6721
	goto	u6720
u6721:
	goto	l11825
u6720:
	line	131
	
l11823:	
	movlw	0FFh
	xorwf	(___ftadd@f1)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f1+1)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f1+2)^080h,f
	line	132
	movlw	01h
	addwf	(___ftadd@f1)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f1+1)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f1+2)^080h,f
	goto	l11825
	line	133
	
l6921:	
	line	134
	
l11825:	
	btfss	(___ftadd@sign)^080h,(6)&7
	goto	u6731
	goto	u6730
u6731:
	goto	l11829
u6730:
	line	136
	
l11827:	
	movlw	0FFh
	xorwf	(___ftadd@f2)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f2+1)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f2+2)^080h,f
	line	137
	movlw	01h
	addwf	(___ftadd@f2)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f2+1)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f2+2)^080h,f
	goto	l11829
	line	138
	
l6922:	
	line	139
	
l11829:	
	clrf	(___ftadd@sign)^080h
	line	140
	movf	(___ftadd@f1)^080h,w
	addwf	(___ftadd@f2)^080h,f
	movf	(___ftadd@f1+1)^080h,w
	clrz
	skipnc
	incf	(___ftadd@f1+1)^080h,w
	skipnz
	goto	u6741
	addwf	(___ftadd@f2+1)^080h,f
u6741:
	movf	(___ftadd@f1+2)^080h,w
	clrz
	skipnc
	incf	(___ftadd@f1+2)^080h,w
	skipnz
	goto	u6742
	addwf	(___ftadd@f2+2)^080h,f
u6742:

	line	141
	
l11831:	
	btfss	(___ftadd@f2+2)^080h,(23)&7
	goto	u6751
	goto	u6750
u6751:
	goto	l11837
u6750:
	line	142
	
l11833:	
	movlw	0FFh
	xorwf	(___ftadd@f2)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f2+1)^080h,f
	movlw	0FFh
	xorwf	(___ftadd@f2+2)^080h,f
	line	143
	movlw	01h
	addwf	(___ftadd@f2)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f2+1)^080h,f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftadd@f2+2)^080h,f
	line	144
	
l11835:	
	clrf	(___ftadd@sign)^080h
	bsf	status,0
	rlf	(___ftadd@sign)^080h,f
	goto	l11837
	line	145
	
l6923:	
	line	146
	
l11837:	
	movf	(___ftadd@f2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___ftpack)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2+1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___ftpack+1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2+2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?___ftpack+2)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@exp1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	movwf	0+(?___ftpack)+03h
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@sign)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+1)+0
	movf	(??___ftadd+1)+0,w
	movwf	0+(?___ftpack)+04h
	fcall	___ftpack
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___ftpack)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(?___ftadd)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?___ftpack)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(?___ftadd+1)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(2+(?___ftpack)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(?___ftadd+2)^080h
	goto	l6898
	
l11839:	
	line	148
	
l6898:	
	return
	opt stack 0
GLOBAL	__end_of___ftadd
	__end_of___ftadd:
;; =============== function ___ftadd ends ============

	signat	___ftadd,8315
	global	_initIRobot
psect	text2230,local,class=CODE,delta=2
global __ptext2230
__ptext2230:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;;		_ser_putch
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text2230
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	147
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	148
	
l11765:	
;main.c: 148: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	136
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u7927:
	decfsz	((??_initIRobot+0)+0),f
	goto	u7927
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u7927
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u7927
opt asmopt_on

	line	149
	
l11767:	
;main.c: 149: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	150
	
l11769:	
;main.c: 150: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	151
	
l6723:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_waitFor
psect	text2231,local,class=CODE,delta=2
global __ptext2231
__ptext2231:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 252 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;  type            1    wreg     unsigned char 
;;  highByte        1   12[BANK0 ] unsigned char 
;;  lowByte         1   13[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  type            1   17[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_goReverse
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_goParallel
;; This function uses a non-reentrant model
;;
psect	text2231
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	252
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 1
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	253
	
l11757:	
;drive.c: 253: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	136
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u7937:
	decfsz	((??_waitFor+0)+0),f
	goto	u7937
	decfsz	((??_waitFor+0)+0+1),f
	goto	u7937
	decfsz	((??_waitFor+0)+0+2),f
	goto	u7937
opt asmopt_on

	line	254
	
l11759:	
;drive.c: 254: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	255
	
l11761:	
;drive.c: 255: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	256
	
l11763:	
;drive.c: 256: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	257
	
l5884:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_drive
psect	text2232,local,class=CODE,delta=2
global __ptext2232
__ptext2232:

;; *************** function _drive *****************
;; Defined at:
;;		line 21 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;  highByteSpee    1    wreg     unsigned char 
;;  lowByteSpeed    1   12[BANK0 ] unsigned char 
;;  highByteRadi    1   13[BANK0 ] unsigned char 
;;  lowByteRadiu    1   14[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  highByteSpee    1   18[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_driveForDistance
;;		_goReverse
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_frontWallCorrect
;;		_goParallel
;;		_checkIfHome
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2232
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	21
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 3
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	22
	
l11745:	
;drive.c: 22: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	136
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u7947:
	decfsz	((??_drive+0)+0),f
	goto	u7947
	decfsz	((??_drive+0)+0+1),f
	goto	u7947
	decfsz	((??_drive+0)+0+2),f
	goto	u7947
opt asmopt_on

	line	23
	
l11747:	
;drive.c: 23: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	24
	
l11749:	
;drive.c: 24: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	25
	
l11751:	
;drive.c: 25: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	26
	
l11753:	
;drive.c: 26: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	27
	
l11755:	
;drive.c: 27: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	28
	
l5816:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_rotateIR
psect	text2233,local,class=CODE,delta=2
global __ptext2233
__ptext2233:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
;; Parameters:    Size  Location     Type
;;  steps           1    wreg     unsigned char 
;;  direction       1   10[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  steps           1   13[BANK0 ] unsigned char 
;;  stepNum         1   14[BANK0 ] unsigned char 
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
;;      Temps:          0       2       0       0       0
;;      Totals:         0       5       0       0       0
;;Total ram usage:        5 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_frontWallCorrect
;;		_findWalls
;;		_goParallel
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2233
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
	line	39
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 4
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	40
	
l11731:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l11733:	
;ir.c: 41: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	44
;ir.c: 44: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l5084
	line	45
	
l5085:	
	line	46
;ir.c: 45: {
;ir.c: 46: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	47
	
l11735:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l11737:	
;ir.c: 48: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u7957:
	decfsz	((??_rotateIR+0)+0),f
	goto	u7957
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u7957
	nop2
opt asmopt_on

	line	44
	
l11739:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l5084:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u6511
	goto	u6510
u6511:
	goto	l5085
u6510:
	goto	l11741
	
l5086:	
	line	51
	
l11741:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l11743:	
;ir.c: 52: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u7967:
	decfsz	((??_rotateIR+0)+0),f
	goto	u7967
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u7967
	nop2
opt asmopt_on

	line	54
	
l5087:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_convert
psect	text2234,local,class=CODE,delta=2
global __ptext2234
__ptext2234:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
;; Parameters:    Size  Location     Type
;;  adc_value       2   33[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  2   33[BANK0 ] int 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		___wmul
;;		___awdiv
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text2234
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 2
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l11671:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6445
	movlw	low(052h)
	subwf	(convert@adc_value),w
u6445:

	skipnc
	goto	u6441
	goto	u6440
u6441:
	goto	l11679
u6440:
	line	13
	
l11673:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5065
	
l11675:	
	goto	l5065
	
l11677:	
	goto	l5065
	line	14
	
l5064:	
	
l11679:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6455
	movlw	low(085h)
	subwf	(convert@adc_value),w
u6455:

	skipnc
	goto	u6451
	goto	u6450
u6451:
	goto	l11687
u6450:
	line	15
	
l11681:	
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
	goto	l5065
	
l11683:	
	goto	l5065
	
l11685:	
	goto	l5065
	line	16
	
l5067:	
	
l11687:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6465
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u6465:

	skipnc
	goto	u6461
	goto	u6460
u6461:
	goto	l11695
u6460:
	line	17
	
l11689:	
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
	goto	l5065
	
l11691:	
	goto	l5065
	
l11693:	
	goto	l5065
	line	18
	
l5069:	
	
l11695:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6475
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u6475:

	skipnc
	goto	u6471
	goto	u6470
u6471:
	goto	l11703
u6470:
	line	19
	
l11697:	
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
	goto	l5065
	
l11699:	
	goto	l5065
	
l11701:	
	goto	l5065
	line	20
	
l5071:	
	
l11703:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u6485
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u6485:

	skipnc
	goto	u6481
	goto	u6480
u6481:
	goto	l11711
u6480:
	line	21
	
l11705:	
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
	goto	l5065
	
l11707:	
	goto	l5065
	
l11709:	
	goto	l5065
	line	22
	
l5073:	
	
l11711:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6495
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u6495:

	skipnc
	goto	u6491
	goto	u6490
u6491:
	goto	l11719
u6490:
	line	23
	
l11713:	
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
	goto	l5065
	
l11715:	
	goto	l5065
	
l11717:	
	goto	l5065
	line	24
	
l5075:	
	
l11719:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6505
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u6505:

	skipnc
	goto	u6501
	goto	u6500
u6501:
	goto	l11727
u6500:
	line	25
	
l11721:	
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
	goto	l5065
	
l11723:	
	goto	l5065
	
l11725:	
	goto	l5065
	line	26
	
l5077:	
	
l11727:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5065
	
l11729:	
	goto	l5065
	
l5078:	
	goto	l5065
	
l5076:	
	goto	l5065
	
l5074:	
	goto	l5065
	
l5072:	
	goto	l5065
	
l5070:	
	goto	l5065
	
l5068:	
	goto	l5065
	
l5066:	
	line	27
	
l5065:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_play_iCreate_song
psect	text2235,local,class=CODE,delta=2
global __ptext2235
__ptext2235:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 25 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
;; Parameters:    Size  Location     Type
;;  song            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  song            1   12[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
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
psect	text2235
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\songs.c"
	line	25
	global	__size_of_play_iCreate_song
	__size_of_play_iCreate_song	equ	__end_of_play_iCreate_song-_play_iCreate_song
	
_play_iCreate_song:	
	opt	stack 3
; Regs used in _play_iCreate_song: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;play_iCreate_song@song stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(play_iCreate_song@song)
	line	26
	
l11669:	
;songs.c: 26: ser_putch(141);
	movlw	(08Dh)
	fcall	_ser_putch
	line	27
;songs.c: 27: ser_putch(song);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(play_iCreate_song@song),w
	fcall	_ser_putch
	line	28
	
l4372:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_ser_putArr
psect	text2236,local,class=CODE,delta=2
global __ptext2236
__ptext2236:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
;; Parameters:    Size  Location     Type
;;  array           2   12[BANK0 ] PTR unsigned char 
;;		 -> beep(5), champions(21), lookingForU2(29), superMarioBros(25), 
;;		 -> finalCountdown(27), 
;;  length          2   14[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  i               2   19[BANK0 ] int 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_putch
;; This function is called by:
;;		_initSongs
;; This function uses a non-reentrant model
;;
psect	text2236
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l11661:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l11667
	line	75
	
l3643:	
	line	76
	
l11663:	
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
	goto	u6420
	decf	(??_ser_putArr+0)+0,f
u6420:
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
	
l11665:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l11667
	
l3642:	
	
l11667:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u6435
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u6435:

	skipc
	goto	u6431
	goto	u6430
u6431:
	goto	l11663
u6430:
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
psect	text2237,local,class=CODE,delta=2
global __ptext2237
__ptext2237:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  c               1   11[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_ser_isrx
;; This function is called by:
;;		_driveForDistance
;;		_lookForVictim
;; This function uses a non-reentrant model
;;
psect	text2237
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 2
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l11645:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l11647
	
l3637:	
	line	62
;ser.c: 62: continue;
	goto	l11647
	
l3636:	
	line	61
	
l11647:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u6411
	goto	u6410
u6411:
	goto	l11647
u6410:
	
l3638:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l11649:	
;ser.c: 65: c=rxfifo[rxoptr];
	movf	(_rxoptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	movwf	(ser_getch@c)
	line	66
	
l11651:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l11653:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l11655:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l11657:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l3639
	
l11659:	
	line	70
	
l3639:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_lcd_write_data
psect	text2238,local,class=CODE,delta=2
global __ptext2238
__ptext2238:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
;; Parameters:    Size  Location     Type
;;  databyte        1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  databyte        1   12[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
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
psect	text2238
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	20
	global	__size_of_lcd_write_data
	__size_of_lcd_write_data	equ	__end_of_lcd_write_data-_lcd_write_data
	
_lcd_write_data:	
	opt	stack 3
; Regs used in _lcd_write_data: [wreg]
;lcd_write_data@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_data@databyte)
	line	21
	
l11637:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l11639:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l11641:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l11643:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u7977:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u7977
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u7977
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
psect	text2239,local,class=CODE,delta=2
global __ptext2239
__ptext2239:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
;; Parameters:    Size  Location     Type
;;  databyte        1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  databyte        1   12[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_lcd_set_cursor
;;		_lcd_init
;; This function uses a non-reentrant model
;;
psect	text2239
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\lcd.c"
	line	8
	global	__size_of_lcd_write_control
	__size_of_lcd_write_control	equ	__end_of_lcd_write_control-_lcd_write_control
	
_lcd_write_control:	
	opt	stack 3
; Regs used in _lcd_write_control: [wreg]
;lcd_write_control@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_control@databyte)
	line	9
	
l11629:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l11631:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l11633:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l11635:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u7987:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u7987
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u7987
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
psect	text2240,local,class=CODE,delta=2
global __ptext2240
__ptext2240:

;; *************** function _readEEPROM *****************
;; Defined at:
;;		line 50 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;  addressH        1    wreg     unsigned char 
;;  addressL        1   11[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  addressH        1   15[BANK0 ] unsigned char 
;;  returnData      1   16[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_initEEPROMMode
;;		_writeSPIByte
;; This function is called by:
;;		_EEPROMToSerial
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text2240
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	50
	global	__size_of_readEEPROM
	__size_of_readEEPROM	equ	__end_of_readEEPROM-_readEEPROM
	
_readEEPROM:	
	opt	stack 3
; Regs used in _readEEPROM: [wreg+status,2+status,0+pclath+cstack]
;readEEPROM@addressH stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(readEEPROM@addressH)
	line	51
	
l11611:	
;eeprom.c: 51: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_readEEPROM+0)+0+2),f
movlw	136
movwf	((??_readEEPROM+0)+0+1),f
	movlw	86
movwf	((??_readEEPROM+0)+0),f
u7997:
	decfsz	((??_readEEPROM+0)+0),f
	goto	u7997
	decfsz	((??_readEEPROM+0)+0+1),f
	goto	u7997
	decfsz	((??_readEEPROM+0)+0+2),f
	goto	u7997
opt asmopt_on

	line	52
	
l11613:	
;eeprom.c: 52: initEEPROMMode();
	fcall	_initEEPROMMode
	line	53
	
l11615:	
;eeprom.c: 53: unsigned char returnData = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(readEEPROM@returnData)
	line	55
	
l11617:	
;eeprom.c: 55: writeSPIByte(3);
	movlw	(03h)
	fcall	_writeSPIByte
	line	57
	
l11619:	
;eeprom.c: 57: writeSPIByte(addressH);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(readEEPROM@addressH),w
	fcall	_writeSPIByte
	line	59
	
l11621:	
;eeprom.c: 59: writeSPIByte(addressL);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(readEEPROM@addressL),w
	fcall	_writeSPIByte
	line	62
	
l11623:	
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
	goto	u6401
	goto	u6400
u6401:
	goto	l1413
u6400:
	goto	l11625
	
l1415:	
	line	65
	
l11625:	
;eeprom.c: 65: returnData = SSPBUF;
	movf	(19),w	;volatile
	movwf	(??_readEEPROM+0)+0
	movf	(??_readEEPROM+0)+0,w
	movwf	(readEEPROM@returnData)
	line	67
;eeprom.c: 67: return returnData;
	movf	(readEEPROM@returnData),w
	goto	l1416
	
l11627:	
	line	68
	
l1416:	
	return
	opt stack 0
GLOBAL	__end_of_readEEPROM
	__end_of_readEEPROM:
;; =============== function _readEEPROM ends ============

	signat	_readEEPROM,8313
	global	_writeEEPROM
psect	text2241,local,class=CODE,delta=2
global __ptext2241
__ptext2241:

;; *************** function _writeEEPROM *****************
;; Defined at:
;;		line 27 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;;  addressH        1   11[BANK0 ] unsigned char 
;;  addressL        1   12[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1   16[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		_initEEPROMMode
;;		_writeSPIByte
;; This function is called by:
;;		_addNewData
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text2241
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	27
	global	__size_of_writeEEPROM
	__size_of_writeEEPROM	equ	__end_of_writeEEPROM-_writeEEPROM
	
_writeEEPROM:	
	opt	stack 2
; Regs used in _writeEEPROM: [wreg+status,2+status,0+pclath+cstack]
;writeEEPROM@data stored from wreg
	line	29
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(writeEEPROM@data)
	
l11593:	
;eeprom.c: 29: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_writeEEPROM+0)+0+2),f
movlw	136
movwf	((??_writeEEPROM+0)+0+1),f
	movlw	86
movwf	((??_writeEEPROM+0)+0),f
u8007:
	decfsz	((??_writeEEPROM+0)+0),f
	goto	u8007
	decfsz	((??_writeEEPROM+0)+0+1),f
	goto	u8007
	decfsz	((??_writeEEPROM+0)+0+2),f
	goto	u8007
opt asmopt_on

	line	30
	
l11595:	
;eeprom.c: 30: initEEPROMMode();
	fcall	_initEEPROMMode
	line	32
	
l11597:	
;eeprom.c: 32: writeSPIByte(6);
	movlw	(06h)
	fcall	_writeSPIByte
	line	33
	
l11599:	
;eeprom.c: 33: initEEPROMMode();
	fcall	_initEEPROMMode
	line	36
	
l11601:	
;eeprom.c: 36: writeSPIByte(2);
	movlw	(02h)
	fcall	_writeSPIByte
	line	39
	
l11603:	
;eeprom.c: 39: writeSPIByte(addressH);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressH),w
	fcall	_writeSPIByte
	line	42
	
l11605:	
;eeprom.c: 42: writeSPIByte(addressL);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressL),w
	fcall	_writeSPIByte
	line	45
	
l11607:	
;eeprom.c: 45: writeSPIByte(data);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@data),w
	fcall	_writeSPIByte
	line	46
	
l11609:	
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
psect	text2242,local,class=CODE,delta=2
global __ptext2242
__ptext2242:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text2242
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l11583:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l11585:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l11587:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l11589:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l11591:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u8017:
decfsz	(??_init_adc+0)+0,f
	goto	u8017
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
psect	text2243,local,class=CODE,delta=2
global __ptext2243
__ptext2243:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  adc_value       2   31[BANK0 ] volatile int 
;; Return value:  Size  Location     Type
;;                  2   25[BANK0 ] int 
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
;; Hardware stack levels required when called:    3
;; This function calls:
;;		___awdiv
;; This function is called by:
;;		_adc_read_channel
;; This function uses a non-reentrant model
;;
psect	text2243
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 1
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l10139:	
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
	
l10141:	
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
	goto	u4431
	goto	u4430
u4431:
	goto	l703
u4430:
	goto	l10143
	
l705:	
	line	75
	
l10143:	
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
u4445:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4445
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l10145:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l10147:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text2244,local,class=CODE,delta=2
global __ptext2244
__ptext2244:

;; *************** function ___awdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\awdiv.c"
;; Parameters:    Size  Location     Type
;;  divisor         2   16[BANK0 ] int 
;;  dividend        2   18[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  quotient        2   23[BANK0 ] int 
;;  sign            1   22[BANK0 ] unsigned char 
;;  counter         1   21[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2   16[BANK0 ] int 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_adc_read
;;		_convert
;; This function uses a non-reentrant model
;;
psect	text2244
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 2
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l10053:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u4271
	goto	u4270
u4271:
	goto	l10057
u4270:
	line	11
	
l10055:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l10057
	line	13
	
l6991:	
	line	14
	
l10057:	
	btfss	(___awdiv@dividend+1),7
	goto	u4281
	goto	u4280
u4281:
	goto	l10063
u4280:
	line	15
	
l10059:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l10061:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l10063
	line	17
	
l6992:	
	line	18
	
l10063:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l10065:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u4291
	goto	u4290
u4291:
	goto	l10085
u4290:
	line	20
	
l10067:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l10073
	
l6995:	
	line	22
	
l10069:	
	movlw	01h
	
u4305:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u4305
	line	23
	
l10071:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l10073
	line	24
	
l6994:	
	line	21
	
l10073:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u4311
	goto	u4310
u4311:
	goto	l10069
u4310:
	goto	l10075
	
l6996:	
	goto	l10075
	line	25
	
l6997:	
	line	26
	
l10075:	
	movlw	01h
	
u4325:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u4325
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u4335
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u4335:
	skipc
	goto	u4331
	goto	u4330
u4331:
	goto	l10081
u4330:
	line	28
	
l10077:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l10079:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l10081
	line	30
	
l6998:	
	line	31
	
l10081:	
	movlw	01h
	
u4345:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u4345
	line	32
	
l10083:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u4351
	goto	u4350
u4351:
	goto	l10075
u4350:
	goto	l10085
	
l6999:	
	goto	l10085
	line	33
	
l6993:	
	line	34
	
l10085:	
	movf	(___awdiv@sign),w
	skipz
	goto	u4360
	goto	l10089
u4360:
	line	35
	
l10087:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l10089
	
l7000:	
	line	36
	
l10089:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l7001
	
l10091:	
	line	37
	
l7001:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___fttol
psect	text2245,local,class=CODE,delta=2
global __ptext2245
__ptext2245:

;; *************** function ___fttol *****************
;; Defined at:
;;		line 45 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\fttol.c"
;; Parameters:    Size  Location     Type
;;  f1              3   18[BANK0 ] float 
;; Auto vars:     Size  Location     Type
;;  lval            4   27[BANK0 ] unsigned long 
;;  exp1            1   31[BANK0 ] unsigned char 
;;  sign1           1   26[BANK0 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  4   18[BANK0 ] long 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       4       0       0       0
;;      Locals:         0       6       0       0       0
;;      Temps:          0       4       0       0       0
;;      Totals:         0      14       0       0       0
;;Total ram usage:       14 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_goParallel
;; This function uses a non-reentrant model
;;
psect	text2245
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\fttol.c"
	line	45
	global	__size_of___fttol
	__size_of___fttol	equ	__end_of___fttol-___fttol
	
___fttol:	
	opt	stack 4
; Regs used in ___fttol: [wreg+status,2+status,0]
	line	49
	
l9993:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(___fttol@f1),w
	movwf	((??___fttol+0)+0)
	movf	(___fttol@f1+1),w
	movwf	((??___fttol+0)+0+1)
	movf	(___fttol@f1+2),w
	movwf	((??___fttol+0)+0+2)
	clrc
	rlf	(??___fttol+0)+1,w
	rlf	(??___fttol+0)+2,w
	movwf	(??___fttol+3)+0
	movf	(??___fttol+3)+0,w
	movwf	(___fttol@exp1)
	movf	((___fttol@exp1)),f
	skipz
	goto	u4151
	goto	u4150
u4151:
	goto	l9999
u4150:
	line	50
	
l9995:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6961
	
l9997:	
	goto	l6961
	
l6960:	
	line	51
	
l9999:	
	movf	(___fttol@f1),w
	movwf	((??___fttol+0)+0)
	movf	(___fttol@f1+1),w
	movwf	((??___fttol+0)+0+1)
	movf	(___fttol@f1+2),w
	movwf	((??___fttol+0)+0+2)
	movlw	017h
u4165:
	clrc
	rrf	(??___fttol+0)+2,f
	rrf	(??___fttol+0)+1,f
	rrf	(??___fttol+0)+0,f
u4160:
	addlw	-1
	skipz
	goto	u4165
	movf	0+(??___fttol+0)+0,w
	movwf	(??___fttol+3)+0
	movf	(??___fttol+3)+0,w
	movwf	(___fttol@sign1)
	line	52
	
l10001:	
	bsf	(___fttol@f1)+(15/8),(15)&7
	line	53
	
l10003:	
	movlw	0FFh
	andwf	(___fttol@f1),f
	movlw	0FFh
	andwf	(___fttol@f1+1),f
	movlw	0
	andwf	(___fttol@f1+2),f
	line	54
	
l10005:	
	movf	(___fttol@f1),w
	movwf	(___fttol@lval)
	movf	(___fttol@f1+1),w
	movwf	((___fttol@lval))+1
	movf	(___fttol@f1+2),w
	movwf	((___fttol@lval))+2
	clrf	((___fttol@lval))+3
	line	55
	
l10007:	
	movlw	low(08Eh)
	subwf	(___fttol@exp1),f
	line	56
	
l10009:	
	btfss	(___fttol@exp1),7
	goto	u4171
	goto	u4170
u4171:
	goto	l10019
u4170:
	line	57
	
l10011:	
	movf	(___fttol@exp1),w
	xorlw	80h
	addlw	-((-15)^80h)
	skipnc
	goto	u4181
	goto	u4180
u4181:
	goto	l10017
u4180:
	line	58
	
l10013:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6961
	
l10015:	
	goto	l6961
	
l6963:	
	goto	l10017
	line	59
	
l6964:	
	line	60
	
l10017:	
	movlw	01h
u4195:
	clrc
	rrf	(___fttol@lval+3),f
	rrf	(___fttol@lval+2),f
	rrf	(___fttol@lval+1),f
	rrf	(___fttol@lval),f
	addlw	-1
	skipz
	goto	u4195

	line	61
	movlw	(01h)
	movwf	(??___fttol+0)+0
	movf	(??___fttol+0)+0,w
	addwf	(___fttol@exp1),f
	btfss	status,2
	goto	u4201
	goto	u4200
u4201:
	goto	l10017
u4200:
	goto	l10029
	
l6965:	
	line	62
	goto	l10029
	
l6962:	
	line	63
	
l10019:	
	movlw	(018h)
	subwf	(___fttol@exp1),w
	skipc
	goto	u4211
	goto	u4210
u4211:
	goto	l10027
u4210:
	line	64
	
l10021:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6961
	
l10023:	
	goto	l6961
	
l6967:	
	line	65
	goto	l10027
	
l6969:	
	line	66
	
l10025:	
	movlw	01h
	movwf	(??___fttol+0)+0
u4225:
	clrc
	rlf	(___fttol@lval),f
	rlf	(___fttol@lval+1),f
	rlf	(___fttol@lval+2),f
	rlf	(___fttol@lval+3),f
	decfsz	(??___fttol+0)+0
	goto	u4225
	line	67
	movlw	low(01h)
	subwf	(___fttol@exp1),f
	goto	l10027
	line	68
	
l6968:	
	line	65
	
l10027:	
	movf	(___fttol@exp1),f
	skipz
	goto	u4231
	goto	u4230
u4231:
	goto	l10025
u4230:
	goto	l10029
	
l6970:	
	goto	l10029
	line	69
	
l6966:	
	line	70
	
l10029:	
	movf	(___fttol@sign1),w
	skipz
	goto	u4240
	goto	l10033
u4240:
	line	71
	
l10031:	
	comf	(___fttol@lval),f
	comf	(___fttol@lval+1),f
	comf	(___fttol@lval+2),f
	comf	(___fttol@lval+3),f
	incf	(___fttol@lval),f
	skipnz
	incf	(___fttol@lval+1),f
	skipnz
	incf	(___fttol@lval+2),f
	skipnz
	incf	(___fttol@lval+3),f
	goto	l10033
	
l6971:	
	line	72
	
l10033:	
	movf	(___fttol@lval+3),w
	movwf	(?___fttol+3)
	movf	(___fttol@lval+2),w
	movwf	(?___fttol+2)
	movf	(___fttol@lval+1),w
	movwf	(?___fttol+1)
	movf	(___fttol@lval),w
	movwf	(?___fttol)

	goto	l6961
	
l10035:	
	line	73
	
l6961:	
	return
	opt stack 0
GLOBAL	__end_of___fttol
	__end_of___fttol:
;; =============== function ___fttol ends ============

	signat	___fttol,4220
	global	___ftpack
psect	text2246,local,class=CODE,delta=2
global __ptext2246
__ptext2246:

;; *************** function ___ftpack *****************
;; Defined at:
;;		line 63 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\float.c"
;; Parameters:    Size  Location     Type
;;  arg             3   10[BANK0 ] unsigned um
;;  exp             1   13[BANK0 ] unsigned char 
;;  sign            1   14[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  3   10[BANK0 ] float 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       5       0       0       0
;;      Locals:         0       0       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       8       0       0       0
;;Total ram usage:        8 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		___ftadd
;;		___ftmul
;;		___lbtoft
;;		___ftdiv
;;		___abtoft
;;		___awtoft
;;		___lwtoft
;;		___altoft
;;		___lltoft
;;		___attoft
;;		___lttoft
;; This function uses a non-reentrant model
;;
psect	text2246
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\float.c"
	line	63
	global	__size_of___ftpack
	__size_of___ftpack	equ	__end_of___ftpack-___ftpack
	
___ftpack:	
	opt	stack 3
; Regs used in ___ftpack: [wreg+status,2+status,0]
	line	64
	
l11447:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(___ftpack@exp),w
	skipz
	goto	u6130
	goto	l11451
u6130:
	
l11449:	
	movf	(___ftpack@arg+2),w
	iorwf	(___ftpack@arg+1),w
	iorwf	(___ftpack@arg),w
	skipz
	goto	u6141
	goto	u6140
u6141:
	goto	l11457
u6140:
	goto	l11451
	
l7185:	
	line	65
	
l11451:	
	movlw	0x0
	movwf	(?___ftpack)
	movlw	0x0
	movwf	(?___ftpack+1)
	movlw	0x0
	movwf	(?___ftpack+2)
	goto	l7186
	
l11453:	
	goto	l7186
	
l7183:	
	line	66
	goto	l11457
	
l7188:	
	line	67
	
l11455:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	68
	movlw	01h
u6155:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u6155

	goto	l11457
	line	69
	
l7187:	
	line	66
	
l11457:	
	movlw	low highword(0FE0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u6161
	goto	u6160
u6161:
	goto	l11455
u6160:
	goto	l7190
	
l7189:	
	line	70
	goto	l7190
	
l7191:	
	line	71
	
l11459:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	72
	
l11461:	
	movlw	01h
	addwf	(___ftpack@arg),f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftpack@arg+1),f
	movlw	0
	skipnc
movlw 1
	addwf	(___ftpack@arg+2),f
	line	73
	
l11463:	
	movlw	01h
u6175:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u6175

	line	74
	
l7190:	
	line	70
	movlw	low highword(0FF0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u6181
	goto	u6180
u6181:
	goto	l11459
u6180:
	goto	l11467
	
l7192:	
	line	75
	goto	l11467
	
l7194:	
	line	76
	
l11465:	
	movlw	low(01h)
	subwf	(___ftpack@exp),f
	line	77
	movlw	01h
u6195:
	clrc
	rlf	(___ftpack@arg),f
	rlf	(___ftpack@arg+1),f
	rlf	(___ftpack@arg+2),f
	addlw	-1
	skipz
	goto	u6195
	goto	l11467
	line	78
	
l7193:	
	line	75
	
l11467:	
	btfss	(___ftpack@arg+1),(15)&7
	goto	u6201
	goto	u6200
u6201:
	goto	l11465
u6200:
	
l7195:	
	line	79
	btfsc	(___ftpack@exp),(0)&7
	goto	u6211
	goto	u6210
u6211:
	goto	l7196
u6210:
	line	80
	
l11469:	
	movlw	0FFh
	andwf	(___ftpack@arg),f
	movlw	07Fh
	andwf	(___ftpack@arg+1),f
	movlw	0FFh
	andwf	(___ftpack@arg+2),f
	
l7196:	
	line	81
	clrc
	rrf	(___ftpack@exp),f

	line	82
	
l11471:	
	movf	(___ftpack@exp),w
	movwf	((??___ftpack+0)+0)
	clrf	((??___ftpack+0)+0+1)
	clrf	((??___ftpack+0)+0+2)
	movlw	010h
u6225:
	clrc
	rlf	(??___ftpack+0)+0,f
	rlf	(??___ftpack+0)+1,f
	rlf	(??___ftpack+0)+2,f
u6220:
	addlw	-1
	skipz
	goto	u6225
	movf	0+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg),f
	movf	1+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+1),f
	movf	2+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+2),f
	line	83
	
l11473:	
	movf	(___ftpack@sign),w
	skipz
	goto	u6230
	goto	l7197
u6230:
	line	84
	
l11475:	
	bsf	(___ftpack@arg)+(23/8),(23)&7
	
l7197:	
	line	85
	line	86
	
l7186:	
	return
	opt stack 0
GLOBAL	__end_of___ftpack
	__end_of___ftpack:
;; =============== function ___ftpack ends ============

	signat	___ftpack,12411
	global	___wmul
psect	text2247,local,class=CODE,delta=2
global __ptext2247
__ptext2247:

;; *************** function ___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
;; Parameters:    Size  Location     Type
;;  multiplier      2   10[BANK0 ] unsigned int 
;;  multiplicand    2   12[BANK0 ] unsigned int 
;; Auto vars:     Size  Location     Type
;;  product         2   14[BANK0 ] unsigned int 
;; Return value:  Size  Location     Type
;;                  2   10[BANK0 ] unsigned int 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_convert
;; This function uses a non-reentrant model
;;
psect	text2247
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 2
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l11435:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l11437
	line	6
	
l6851:	
	line	7
	
l11437:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u6091
	goto	u6090
u6091:
	goto	l6852
u6090:
	line	8
	
l11439:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l6852:	
	line	9
	movlw	01h
	
u6105:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u6105
	line	10
	
l11441:	
	movlw	01h
	
u6115:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u6115
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u6121
	goto	u6120
u6121:
	goto	l11437
u6120:
	goto	l11443
	
l6853:	
	line	12
	
l11443:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l6854
	
l11445:	
	line	13
	
l6854:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_updateNode
psect	text2248,local,class=CODE,delta=2
global __ptext2248
__ptext2248:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 309 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2248
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	309
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 5
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	310
	
l11417:	
;main.c: 310: if((xCoord == 2) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u6031
	goto	u6030
u6031:
	goto	l11423
u6030:
	
l11419:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u6041
	goto	u6040
u6041:
	goto	l11423
u6040:
	line	311
	
l11421:	
;main.c: 311: node = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	bsf	status,0
	rlf	(_node)^080h,f	;volatile
	goto	l6780
	line	312
	
l6774:	
	
l11423:	
;main.c: 312: else if((xCoord == 4) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u6051
	goto	u6050
u6051:
	goto	l11429
u6050:
	
l11425:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u6061
	goto	u6060
u6061:
	goto	l11429
u6060:
	line	313
	
l11427:	
;main.c: 313: node = 2;
	movlw	(02h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6780
	line	314
	
l6776:	
	
l11429:	
;main.c: 314: else if((xCoord == 2) && (yCoord == 0))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u6071
	goto	u6070
u6071:
	goto	l6778
u6070:
	
l11431:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u6081
	goto	u6080
u6081:
	goto	l6778
u6080:
	line	315
	
l11433:	
;main.c: 315: node = 3;
	movlw	(03h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6780
	line	316
	
l6778:	
	line	317
;main.c: 316: else
;main.c: 317: node = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	goto	l6780
	
l6779:	
	goto	l6780
	
l6777:	
	goto	l6780
	
l6775:	
	line	318
	
l6780:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getSuccessfulDrive
psect	text2249,local,class=CODE,delta=2
global __ptext2249
__ptext2249:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 146 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2249
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	146
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 5
; Regs used in _getSuccessfulDrive: [status]
	line	147
	
l11409:	
;drive.c: 147: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u6021
	goto	u6020
u6021:
	goto	l11413
u6020:
	
l11411:	
	clrc
	
	goto	l5846
	
l11173:	
	
l11413:	
	setc
	
	goto	l5846
	
l11175:	
	goto	l5846
	
l11415:	
	line	148
	
l5846:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_getSomethingInTheWay
psect	text2250,local,class=CODE,delta=2
global __ptext2250
__ptext2250:

;; *************** function _getSomethingInTheWay *****************
;; Defined at:
;;		line 140 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      enum E1103
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text2250
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	140
	global	__size_of_getSomethingInTheWay
	__size_of_getSomethingInTheWay	equ	__end_of_getSomethingInTheWay-_getSomethingInTheWay
	
_getSomethingInTheWay:	
	opt	stack 4
; Regs used in _getSomethingInTheWay: [wreg]
	line	141
	
l11405:	
;drive.c: 141: return somethingInTheWay;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_somethingInTheWay)^080h,w	;volatile
	goto	l5843
	
l11407:	
	line	142
	
l5843:	
	return
	opt stack 0
GLOBAL	__end_of_getSomethingInTheWay
	__end_of_getSomethingInTheWay:
;; =============== function _getSomethingInTheWay ends ============

	signat	_getSomethingInTheWay,89
	global	_getOrientation
psect	text2251,local,class=CODE,delta=2
global __ptext2251
__ptext2251:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 135 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;		None
;; Return value:  Size  Location     Type
;;                  1    wreg      enum E1109
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_updateLocation
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2251
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	135
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 4
; Regs used in _getOrientation: [wreg]
	line	136
	
l11401:	
;drive.c: 136: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l5840
	
l11403:	
	line	137
	
l5840:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_getCurrentY
psect	text2252,local,class=CODE,delta=2
global __ptext2252
__ptext2252:

;; *************** function _getCurrentY *****************
;; Defined at:
;;		line 465 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goForward
;;		_turnLeft90
;; This function uses a non-reentrant model
;;
psect	text2252
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	465
	global	__size_of_getCurrentY
	__size_of_getCurrentY	equ	__end_of_getCurrentY-_getCurrentY
	
_getCurrentY:	
	opt	stack 3
; Regs used in _getCurrentY: [wreg]
	line	466
	
l11397:	
;main.c: 466: return yCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	goto	l6842
	
l11399:	
	line	467
	
l6842:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentY
	__end_of_getCurrentY:
;; =============== function _getCurrentY ends ============

	signat	_getCurrentY,89
	global	_getCurrentX
psect	text2253,local,class=CODE,delta=2
global __ptext2253
__ptext2253:

;; *************** function _getCurrentX *****************
;; Defined at:
;;		line 460 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goForward
;;		_turnLeft90
;; This function uses a non-reentrant model
;;
psect	text2253
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	460
	global	__size_of_getCurrentX
	__size_of_getCurrentX	equ	__end_of_getCurrentX-_getCurrentX
	
_getCurrentX:	
	opt	stack 3
; Regs used in _getCurrentX: [wreg]
	line	461
	
l11393:	
;main.c: 461: return xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	goto	l6839
	
l11395:	
	line	462
	
l6839:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentX
	__end_of_getCurrentX:
;; =============== function _getCurrentX ends ============

	signat	_getCurrentX,89
	global	_updateOrientation
psect	text2254,local,class=CODE,delta=2
global __ptext2254
__ptext2254:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 245 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
;; Parameters:    Size  Location     Type
;;  moved           1    wreg     enum E1103
;; Auto vars:     Size  Location     Type
;;  moved           1   11[BANK0 ] enum E1103
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_driveForDistance
;;		_goBackward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text2254
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\drive.c"
	line	245
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 3
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	246
	
l11387:	
;drive.c: 246: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	247
	
l11389:	
;drive.c: 247: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u6011
	goto	u6010
u6011:
	goto	l5881
u6010:
	line	248
	
l11391:	
;drive.c: 248: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l5881
	
l5880:	
	line	249
	
l5881:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_ser_init
psect	text2255,local,class=CODE,delta=2
global __ptext2255
__ptext2255:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_init
;; This function uses a non-reentrant model
;;
psect	text2255
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l11361:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l11363:	
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
	
l11365:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l11367:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l11369:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l11371:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l11373:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l11375:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l11377:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l11379:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l11381:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l11383:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l11385:	
;ser.c: 143: rxiptr=rxoptr=txiptr=txoptr=0;
	movlw	(0)
	movwf	(_txoptr)	;volatile
	movwf	(_txiptr)	;volatile
	movwf	(_rxoptr)	;volatile
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
psect	text2256,local,class=CODE,delta=2
global __ptext2256
__ptext2256:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_ser_getch
;; This function uses a non-reentrant model
;;
psect	text2256
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 2
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l11313:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u5941
	goto	u5940
u5941:
	goto	l11321
u5940:
	line	50
	
l11315:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l11317:	
;ser.c: 52: return 0;
	clrc
	
	goto	l3633
	
l11319:	
	goto	l3633
	line	53
	
l3632:	
	line	54
	
l11321:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u5951
	goto	u5950
u5951:
	goto	l11325
u5950:
	
l11323:	
	clrc
	
	goto	l3633
	
l11169:	
	
l11325:	
	setc
	
	goto	l3633
	
l11171:	
	goto	l3633
	
l11327:	
	line	55
	
l3633:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_getVictimZone
psect	text2257,local,class=CODE,delta=2
global __ptext2257
__ptext2257:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
;; Parameters:    Size  Location     Type
;;  victimX         1    wreg     unsigned char 
;;  victimY         1   10[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  victimX         1   12[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_lookForVictim
;; This function uses a non-reentrant model
;;
psect	text2257
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	157
	global	__size_of_getVictimZone
	__size_of_getVictimZone	equ	__end_of_getVictimZone-_getVictimZone
	
_getVictimZone:	
	opt	stack 4
; Regs used in _getVictimZone: [wreg-fsr0h+status,2+status,0]
;getVictimZone@victimX stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(getVictimZone@victimX)
	line	163
	
l11265:	
;map.c: 163: switch (victimX)
	goto	l11307
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l2900:	
	line	166
;map.c: 166: switch (victimY)
	goto	l11273
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l2902:	
	line	169
	
l11267:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	170
;map.c: 170: break;
	goto	l11309
	line	171
;map.c: 171: case 1:
	
l2904:	
	line	172
	
l11269:	
;map.c: 172: vicZone = 4;
	movlw	(04h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	173
;map.c: 173: break;
	goto	l11309
	line	178
;map.c: 178: default:
	
l2905:	
	line	179
;map.c: 179: break;
	goto	l11309
	line	180
	
l11271:	
;map.c: 180: }
	goto	l11309
	line	166
	
l2901:	
	
l11273:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l11267
	xorlw	1^0	; case 1
	skipnz
	goto	l11269
	goto	l11309
	opt asmopt_on

	line	180
	
l2903:	
	line	181
;map.c: 181: break;
	goto	l11309
	line	183
;map.c: 183: case 1:
	
l2907:	
	line	184
;map.c: 184: switch (victimY)
	goto	l11281
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l2909:	
	line	187
	
l11275:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	188
;map.c: 188: break;
	goto	l11309
	line	189
;map.c: 189: case 1:
	
l2911:	
	line	190
	
l11277:	
;map.c: 190: vicZone = 4;
	movlw	(04h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	191
;map.c: 191: break;
	goto	l11309
	line	196
;map.c: 196: default:
	
l2912:	
	line	197
;map.c: 197: break;
	goto	l11309
	line	198
	
l11279:	
;map.c: 198: }
	goto	l11309
	line	184
	
l2908:	
	
l11281:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l11275
	xorlw	1^0	; case 1
	skipnz
	goto	l11277
	goto	l11309
	opt asmopt_on

	line	198
	
l2910:	
	line	199
;map.c: 199: break;
	goto	l11309
	line	201
;map.c: 201: case 2:
	
l2913:	
	line	202
;map.c: 202: switch (victimY)
	goto	l11289
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l2915:	
	line	207
	
l11283:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	208
;map.c: 208: break;
	goto	l11309
	line	211
;map.c: 211: case 3:
	
l2917:	
	line	212
	
l11285:	
;map.c: 212: vicZone = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_vicZone)^080h
	bsf	status,0
	rlf	(_vicZone)^080h,f
	line	213
;map.c: 213: break;
	goto	l11309
	line	214
;map.c: 214: default:
	
l2918:	
	line	215
;map.c: 215: break;
	goto	l11309
	line	216
	
l11287:	
;map.c: 216: }
	goto	l11309
	line	202
	
l2914:	
	
l11289:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l11283
	xorlw	3^1	; case 3
	skipnz
	goto	l11285
	goto	l11309
	opt asmopt_on

	line	216
	
l2916:	
	line	217
;map.c: 217: break;
	goto	l11309
	line	219
;map.c: 219: case 3:
	
l2919:	
	line	220
;map.c: 220: switch (victimY)
	goto	l11297
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l2921:	
	line	225
	
l11291:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	226
;map.c: 226: break;
	goto	l11309
	line	229
;map.c: 229: case 3:
	
l2923:	
	line	230
	
l11293:	
;map.c: 230: vicZone = 2;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	231
;map.c: 231: break;
	goto	l11309
	line	232
;map.c: 232: default:
	
l2924:	
	line	233
;map.c: 233: break;
	goto	l11309
	line	234
	
l11295:	
;map.c: 234: }
	goto	l11309
	line	220
	
l2920:	
	
l11297:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l11291
	xorlw	3^1	; case 3
	skipnz
	goto	l11293
	goto	l11309
	opt asmopt_on

	line	234
	
l2922:	
	line	235
;map.c: 235: break;
	goto	l11309
	line	237
;map.c: 237: case 4:
	
l2925:	
	line	238
;map.c: 238: switch (victimY)
	goto	l11303
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l2927:	
	line	247
	
l11299:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	248
;map.c: 248: break;
	goto	l11309
	line	249
;map.c: 249: default:
	
l2929:	
	line	250
;map.c: 250: break;
	goto	l11309
	line	251
	
l11301:	
;map.c: 251: }
	goto	l11309
	line	238
	
l2926:	
	
l11303:	
	bcf	status, 5	;RP0=0, select bank0
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
	goto	l11299
	goto	l11309
	opt asmopt_on

	line	251
	
l2928:	
	line	252
;map.c: 252: break;
	goto	l11309
	line	254
;map.c: 254: default:
	
l2930:	
	line	255
;map.c: 255: break;
	goto	l11309
	line	256
	
l11305:	
;map.c: 256: }
	goto	l11309
	line	163
	
l2899:	
	
l11307:	
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
	goto	l11273
	xorlw	1^0	; case 1
	skipnz
	goto	l11281
	xorlw	2^1	; case 2
	skipnz
	goto	l11289
	xorlw	3^2	; case 3
	skipnz
	goto	l11297
	xorlw	4^3	; case 4
	skipnz
	goto	l11303
	goto	l11309
	opt asmopt_on

	line	256
	
l2906:	
	line	258
	
l11309:	
;map.c: 258: return vicZone;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_vicZone)^080h,w
	goto	l2931
	
l11311:	
	line	259
	
l2931:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text2258,local,class=CODE,delta=2
global __ptext2258
__ptext2258:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_checkForFinalDestination
;; This function uses a non-reentrant model
;;
psect	text2258
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 4
; Regs used in _getFinalY: [wreg]
	line	153
	
l11261:	
;map.c: 153: return finalY;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_finalY)^080h,w
	goto	l2896
	
l11263:	
	line	154
	
l2896:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text2259,local,class=CODE,delta=2
global __ptext2259
__ptext2259:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_checkForFinalDestination
;; This function uses a non-reentrant model
;;
psect	text2259
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 4
; Regs used in _getFinalX: [wreg]
	line	148
	
l11257:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l2893
	
l11259:	
	line	149
	
l2893:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_putch
psect	text2260,local,class=CODE,delta=2
global __ptext2260
__ptext2260:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
;; Parameters:    Size  Location     Type
;;  c               1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  c               1   11[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
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
psect	text2260
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\ser.c"
	line	81
	global	__size_of_ser_putch
	__size_of_ser_putch	equ	__end_of_ser_putch-_ser_putch
	
_ser_putch:	
	opt	stack 3
; Regs used in _ser_putch: [wreg-fsr0h+status,2+status,0]
;ser_putch@c stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(ser_putch@c)
	line	82
	
l11229:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l11231
	
l3649:	
	line	83
;ser.c: 83: continue;
	goto	l11231
	
l3648:	
	line	82
	
l11231:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u5911
	goto	u5910
u5911:
	goto	l11231
u5910:
	
l3650:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l11233:	
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
	
l11235:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l11237:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l11239:	
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
psect	text2261,local,class=CODE,delta=2
global __ptext2261
__ptext2261:

;; *************** function _initEEPROMMode *****************
;; Defined at:
;;		line 21 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_writeEEPROM
;;		_readEEPROM
;; This function uses a non-reentrant model
;;
psect	text2261
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	21
	global	__size_of_initEEPROMMode
	__size_of_initEEPROMMode	equ	__end_of_initEEPROMMode-_initEEPROMMode
	
_initEEPROMMode:	
	opt	stack 2
; Regs used in _initEEPROMMode: [wreg+status,2+status,0]
	line	22
	
l11205:	
;eeprom.c: 22: PORTC &= 0b11111100;
	movlw	(0FCh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_initEEPROMMode+0)+0
	movf	(??_initEEPROMMode+0)+0,w
	andwf	(7),f	;volatile
	line	23
	
l11207:	
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
psect	text2262,local,class=CODE,delta=2
global __ptext2262
__ptext2262:

;; *************** function _writeSPIByte *****************
;; Defined at:
;;		line 14 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
;; Parameters:    Size  Location     Type
;;  data            1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  data            1   10[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_writeEEPROM
;;		_readEEPROM
;; This function uses a non-reentrant model
;;
psect	text2262
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\eeprom.c"
	line	14
	global	__size_of_writeSPIByte
	__size_of_writeSPIByte	equ	__end_of_writeSPIByte-_writeSPIByte
	
_writeSPIByte:	
	opt	stack 2
; Regs used in _writeSPIByte: [wreg]
;writeSPIByte@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(writeSPIByte@data)
	line	15
	
l11201:	
;eeprom.c: 15: SSPIF = 0;
	bcf	(99/8),(99)&7
	line	16
	
l11203:	
;eeprom.c: 16: SSPBUF = data;
	movf	(writeSPIByte@data),w
	movwf	(19)	;volatile
	line	17
;eeprom.c: 17: while(!SSPIF);
	goto	l1401
	
l1402:	
	
l1401:	
	btfss	(99/8),(99)&7
	goto	u5881
	goto	u5880
u5881:
	goto	l1401
u5880:
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
psect	text2263,local,class=CODE,delta=2
global __ptext2263
__ptext2263:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
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
;;      Temps:          0      10       0       0       0
;;      Totals:         0      10       0       0       0
;;Total ram usage:       10 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    1
;; This function calls:
;;		___lwmod
;; This function is called by:
;;		Interrupt level 1
;; This function uses a non-reentrant model
;;
psect	text2263
	file	"C:\Documents and Settings\Administrator\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.8\COMPETITIONv0.8\main.c"
	line	62
	global	__size_of_isr1
	__size_of_isr1	equ	__end_of_isr1-_isr1
	
_isr1:	
	opt	stack 0
; Regs used in _isr1: [wreg-fsr0h+status,2+status,0+pclath+cstack]
psect	intentry,class=CODE,delta=2
global __pintentry
__pintentry:
global interrupt_function
interrupt_function:
	global saved_w
	saved_w	set	btemp+0
	movwf	saved_w
	swapf	status,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_isr1+6)
	movf	fsr0,w
	movwf	(??_isr1+7)
	movf	pclath,w
	movwf	(??_isr1+8)
	movf	btemp+1,w
	movwf	(??_isr1+9)
	ljmp	_isr1
psect	text2263
	line	64
	
i1l9803:	
;main.c: 64: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u368_21
	goto	u368_20
u368_21:
	goto	i1l6717
u368_20:
	line	66
	
i1l9805:	
;main.c: 65: {
;main.c: 66: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	67
	
i1l9807:	
;main.c: 67: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	69
;main.c: 69: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	71
	
i1l9809:	
;main.c: 71: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	73
	
i1l9811:	
;main.c: 73: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
	movlw	low(0Ah)
	movwf	(?___lwmod)
	movlw	high(0Ah)
	movwf	((?___lwmod))+1
	movf	(_RTC_Counter+1),w	;volatile
	clrf	1+(?___lwmod)+02h
	addwf	1+(?___lwmod)+02h
	movf	(_RTC_Counter),w	;volatile
	clrf	0+(?___lwmod)+02h
	addwf	0+(?___lwmod)+02h

	fcall	___lwmod
	movf	((1+(?___lwmod))),w
	iorwf	((0+(?___lwmod))),w
	skipz
	goto	u369_21
	goto	u369_20
u369_21:
	goto	i1l9815
u369_20:
	
i1l9813:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l9815
	
i1l6704:	
	line	74
	
i1l9815:	
;main.c: 74: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
	movlw	low(032h)
	movwf	(?___lwmod)
	movlw	high(032h)
	movwf	((?___lwmod))+1
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_RTC_Counter+1),w	;volatile
	clrf	1+(?___lwmod)+02h
	addwf	1+(?___lwmod)+02h
	movf	(_RTC_Counter),w	;volatile
	clrf	0+(?___lwmod)+02h
	addwf	0+(?___lwmod)+02h

	fcall	___lwmod
	movf	((1+(?___lwmod))),w
	iorwf	((0+(?___lwmod))),w
	skipz
	goto	u370_21
	goto	u370_20
u370_21:
	goto	i1l9819
u370_20:
	
i1l9817:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l9819
	
i1l6705:	
	line	75
	
i1l9819:	
;main.c: 75: if(RTC_Counter % 500 == 0)
	movlw	low(01F4h)
	movwf	(?___lwmod)
	movlw	high(01F4h)
	movwf	((?___lwmod))+1
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_RTC_Counter+1),w	;volatile
	clrf	1+(?___lwmod)+02h
	addwf	1+(?___lwmod)+02h
	movf	(_RTC_Counter),w	;volatile
	clrf	0+(?___lwmod)+02h
	addwf	0+(?___lwmod)+02h

	fcall	___lwmod
	line	78
;main.c: 76: {
	
i1l6706:	
	line	80
;main.c: 78: }
;main.c: 80: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u371_21
	goto	u371_20
u371_21:
	goto	i1l6707
u371_20:
	line	82
	
i1l9821:	
;main.c: 81: {
;main.c: 82: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	0+(_start)^080h+02h,f
	line	83
	
i1l9823:	
;main.c: 83: if(start.debounceCount >= 10 & start.released)
	movf	0+(_start)^080h+01h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_isr1+0)+0
	clrf	(??_isr1+0)+0+1
	movlw	(0Ah)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	subwf	0+(_start)^080h+02h,w
	movlw	0
	skipnc
	movlw	1
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
	goto	u372_21
	goto	u372_20
u372_21:
	goto	i1l9831
u372_20:
	line	85
	
i1l9825:	
;main.c: 84: {
;main.c: 85: start.pressed = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	bsf	status,0
	rlf	(_start)^080h,f
	line	86
	
i1l9827:	
;main.c: 86: start.released = 0;
	clrf	0+(_start)^080h+01h
	goto	i1l9831
	line	87
	
i1l6708:	
	line	88
;main.c: 87: }
;main.c: 88: }
	goto	i1l9831
	line	89
	
i1l6707:	
	line	91
;main.c: 89: else
;main.c: 90: {
;main.c: 91: start.debounceCount = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	0+(_start)^080h+02h
	line	92
	
i1l9829:	
;main.c: 92: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	goto	i1l9831
	line	93
	
i1l6709:	
	line	95
	
i1l9831:	
;main.c: 93: }
;main.c: 95: if(!RB1)
	bcf	status, 5	;RP0=0, select bank0
	btfsc	(49/8),(49)&7
	goto	u373_21
	goto	u373_20
u373_21:
	goto	i1l6710
u373_20:
	line	97
	
i1l9833:	
;main.c: 96: {
;main.c: 97: eeprom.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	0+(_eeprom)^080h+02h,f
	line	98
	
i1l9835:	
;main.c: 98: if(eeprom.debounceCount >= 10 & eeprom.released)
	movf	0+(_eeprom)^080h+01h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_isr1+0)+0
	clrf	(??_isr1+0)+0+1
	movlw	(0Ah)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	subwf	0+(_eeprom)^080h+02h,w
	movlw	0
	skipnc
	movlw	1
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
	goto	u374_21
	goto	u374_20
u374_21:
	goto	i1l9843
u374_20:
	line	100
	
i1l9837:	
;main.c: 99: {
;main.c: 100: eeprom.pressed = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_eeprom)^080h
	bsf	status,0
	rlf	(_eeprom)^080h,f
	line	101
	
i1l9839:	
;main.c: 101: eeprom.released = 0;
	clrf	0+(_eeprom)^080h+01h
	goto	i1l9843
	line	102
	
i1l6711:	
	line	103
;main.c: 102: }
;main.c: 103: }
	goto	i1l9843
	line	104
	
i1l6710:	
	line	106
;main.c: 104: else
;main.c: 105: {
;main.c: 106: eeprom.debounceCount = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	0+(_eeprom)^080h+02h
	line	107
	
i1l9841:	
;main.c: 107: eeprom.released = 1;
	clrf	0+(_eeprom)^080h+01h
	bsf	status,0
	rlf	0+(_eeprom)^080h+01h,f
	goto	i1l9843
	line	108
	
i1l6712:	
	line	110
	
i1l9843:	
;main.c: 108: }
;main.c: 110: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	bcf	status, 5	;RP0=0, select bank0
	btfss	(101/8),(101)&7
	goto	u375_21
	goto	u375_20
u375_21:
	goto	i1l9853
u375_20:
	
i1l9845:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	
i1l9847:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_ser_tmp)^080h
	
i1l9849:	
	movf	(_ser_tmp)^080h,w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u376_21
	goto	u376_20
u376_21:
	goto	i1l9853
u376_20:
	
i1l9851:	
	movf	(_ser_tmp)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l9853
	
i1l6714:	
	goto	i1l9853
	
i1l6713:	
	
i1l9853:	
	bcf	status, 5	;RP0=0, select bank0
	btfss	(100/8),(100)&7
	goto	u377_21
	goto	u377_20
u377_21:
	goto	i1l6717
u377_20:
	
i1l9855:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u378_21
	goto	u378_20
u378_21:
	goto	i1l6717
u378_20:
	
i1l9857:	
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(25)	;volatile
	
i1l9859:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l9861:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l9863:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u379_21
	goto	u379_20
u379_21:
	goto	i1l6717
u379_20:
	
i1l9865:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l6717
	
i1l6716:	
	goto	i1l6717
	
i1l6715:	
	goto	i1l6717
	line	111
	
i1l6703:	
	line	112
	
i1l6717:	
	bcf	status, 5	;RP0=0, select bank0
	movf	(??_isr1+9),w
	movwf	btemp+1
	movf	(??_isr1+8),w
	movwf	pclath
	movf	(??_isr1+7),w
	movwf	fsr0
	swapf	(??_isr1+6)^00h,w
	movwf	status
	swapf	saved_w,f
	swapf	saved_w,w
	retfie
	opt stack 0
GLOBAL	__end_of_isr1
	__end_of_isr1:
;; =============== function _isr1 ends ============

	signat	_isr1,88
	global	___lwmod
psect	text2264,local,class=CODE,delta=2
global __ptext2264
__ptext2264:

;; *************** function ___lwmod *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwmod.c"
;; Parameters:    Size  Location     Type
;;  divisor         2    0[COMMON] unsigned int 
;;  dividend        2    2[COMMON] unsigned int 
;; Auto vars:     Size  Location     Type
;;  counter         1    5[COMMON] unsigned char 
;; Return value:  Size  Location     Type
;;                  2    0[COMMON] unsigned int 
;; Registers used:
;;		wreg, status,2, status,0
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         4       0       0       0       0
;;      Locals:         1       0       0       0       0
;;      Temps:          1       0       0       0       0
;;      Totals:         6       0       0       0       0
;;Total ram usage:        6 bytes
;; Hardware stack levels used:    1
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_isr1
;; This function uses a non-reentrant model
;;
psect	text2264
	file	"C:\Program Files\HI-TECH Software\PICC\9.81\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l9897:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u390_21
	goto	u390_20
u390_21:
	goto	i1l9915
u390_20:
	line	9
	
i1l9899:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l9905
	
i1l6869:	
	line	11
	
i1l9901:	
	movlw	01h
	
u391_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u391_25
	line	12
	
i1l9903:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l9905
	line	13
	
i1l6868:	
	line	10
	
i1l9905:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u392_21
	goto	u392_20
u392_21:
	goto	i1l9901
u392_20:
	goto	i1l9907
	
i1l6870:	
	goto	i1l9907
	line	14
	
i1l6871:	
	line	15
	
i1l9907:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u393_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u393_25:
	skipc
	goto	u393_21
	goto	u393_20
u393_21:
	goto	i1l9911
u393_20:
	line	16
	
i1l9909:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l9911
	
i1l6872:	
	line	17
	
i1l9911:	
	movlw	01h
	
u394_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u394_25
	line	18
	
i1l9913:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u395_21
	goto	u395_20
u395_21:
	goto	i1l9907
u395_20:
	goto	i1l9915
	
i1l6873:	
	goto	i1l9915
	line	19
	
i1l6867:	
	line	20
	
i1l9915:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l6874
	
i1l9917:	
	line	21
	
i1l6874:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text2265,local,class=CODE,delta=2
global __ptext2265
__ptext2265:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

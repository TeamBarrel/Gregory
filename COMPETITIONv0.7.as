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
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
	FNCALL	_checkIfHome,_drive
	FNCALL	_checkIfHome,_play_iCreate_song
	FNCALL	_turnAround,_drive
	FNCALL	_turnAround,_waitFor
	FNCALL	_turnLeft90,_drive
	FNCALL	_turnLeft90,_waitFor
	FNCALL	_turnRight90,_drive
	FNCALL	_turnRight90,_waitFor
	FNCALL	_initSongs,_ser_putArr
	FNCALL	_lcd_init,_lcd_write_control
	FNCALL	_lcd_write_1_digit_bcd,_lcd_write_data
	FNCALL	_lcd_set_cursor,_lcd_write_control
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
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	7

;initializer for _finalX
	retlw	03h
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	51

;initializer for _xCoord
	retlw	01h
	line	52

;initializer for _yCoord
	retlw	03h
psect	idataBANK1,class=CODE,space=0,delta=2
global __pidataBANK1
__pidataBANK1:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	8

;initializer for _finalY
	retlw	01h
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	14

;initializer for _somethingInTheWay
	retlw	02h
psect	idataBANK2,class=CODE,space=0,delta=2
global __pidataBANK2
__pidataBANK2:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
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
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	7
_finalX:
       ds      1

psect	dataBANK0
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	51
_xCoord:
       ds      1

psect	dataBANK0
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	8
_finalY:
       ds      1

psect	dataBANK1
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	14
_somethingInTheWay:
       ds      1

psect	dataBANK1
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
_beep:
       ds      5

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
	line	12
_finalCountdown:
       ds      27

psect	dataBANK3
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK3
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK2,class=BANK2,space=1
global __pdataBANK2
__pdataBANK2:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
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
	movlw	low((__pbssBANK1)+02Bh)
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
	global	getVictimZone@victimY
getVictimZone@victimY:	; 1 bytes @ 0xA
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	global	___ftpack@arg
___ftpack@arg:	; 3 bytes @ 0xA
	ds	1
	global	??_getVictimZone
??_getVictimZone:	; 0 bytes @ 0xB
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0xB
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
	global	?_ser_putArr
?_ser_putArr:	; 0 bytes @ 0xC
	global	??_play_iCreate_song
??_play_iCreate_song:	; 0 bytes @ 0xC
	global	?_drive
?_drive:	; 0 bytes @ 0xC
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
	global	findFinalDestination@robotOrientation
findFinalDestination@robotOrientation:	; 1 bytes @ 0xF
	ds	1
	global	??_findFinalDestination
??_findFinalDestination:	; 0 bytes @ 0x10
	global	??_ser_putArr
??_ser_putArr:	; 0 bytes @ 0x10
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x10
	global	lookForVictim@victim
lookForVictim@victim:	; 1 bytes @ 0x10
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x10
	ds	1
	global	findFinalDestination@virtualWallX
findFinalDestination@virtualWallX:	; 1 bytes @ 0x11
	global	waitFor@type
waitFor@type:	; 1 bytes @ 0x11
	ds	1
	global	?___fttol
?___fttol:	; 4 bytes @ 0x12
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
	global	??_checkIfHome
??_checkIfHome:	; 0 bytes @ 0x13
	global	ser_putArr@i
ser_putArr@i:	; 2 bytes @ 0x13
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0x14
	ds	1
	global	??_initSongs
??_initSongs:	; 0 bytes @ 0x15
	global	??_init
??_init:	; 0 bytes @ 0x15
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0x15
	ds	1
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0x16
	global	??___fttol
??___fttol:	; 0 bytes @ 0x16
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0x16
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0x16
	ds	1
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0x17
	ds	1
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0x18
	ds	1
	global	?_adc_read
?_adc_read:	; 2 bytes @ 0x19
	ds	1
	global	___fttol@sign1
___fttol@sign1:	; 1 bytes @ 0x1A
	global	driveForDistance@deltaDistance
driveForDistance@deltaDistance:	; 2 bytes @ 0x1A
	ds	1
	global	??_adc_read
??_adc_read:	; 0 bytes @ 0x1B
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
;;Data sizes: Strings 63, constant 0, data 112, bss 54, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     60      70
;; BANK1           80     23      73
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
;;   _checkIfHome->_drive
;;   _turnAround->_drive
;;   _turnLeft90->_drive
;;   _turnRight90->_drive
;;   _initSongs->_ser_putArr
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_set_cursor->_lcd_write_control
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
;; (0) _main                                                 1     1      0   26617
;;                                             22 BANK1      1     1      0
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
;; (1) _findWalls                                            1     1      0    1413
;;                                             45 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                           _rotateIR
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
;; (1) _frontWallCorrect                                     4     4      0    1537
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
;;   _turnAround
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _turnLeft90
;;     _drive
;;       _ser_putch
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
;;BANK1               50     17      49       7       91.3%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0     10B      12        0.0%
;;ABS                  0      0     101       3        0.0%
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
;;		line 308 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
;;		_updateLocation
;;		_updateNode
;;		_checkIfHome
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	308
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	309
	
l13000:	
;main.c: 309: init();
	fcall	_init
	line	310
;main.c: 310: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	312
	
l13002:	
;main.c: 312: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	313
	
l13004:	
;main.c: 313: lcd_write_string("(-,-) - -- --- -");
	movlw	((STR_3-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	314
;main.c: 314: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	315
	
l13006:	
;main.c: 315: lcd_write_string("- - - (3,1) GREG");
	movlw	((STR_4-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	317
;main.c: 317: while(!home)
	goto	l13122
	
l6774:	
	line	319
	
l13008:	
;main.c: 318: {
;main.c: 319: if(start.pressed && ready == 0)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_start)^080h,w
	skipz
	goto	u7290
	goto	l13038
u7290:
	
l13010:	
	btfsc	(_ready/8),(_ready)&7
	goto	u7301
	goto	u7300
u7301:
	goto	l13038
u7300:
	line	321
	
l13012:	
;main.c: 320: {
;main.c: 321: findWalls();
	fcall	_findWalls
	line	322
	
l13014:	
;main.c: 322: if(leftWall && rightWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7311
	goto	u7310
u7311:
	goto	l6776
u7310:
	
l13016:	
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7321
	goto	u7320
u7321:
	goto	l6776
u7320:
	
l13018:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7331
	goto	u7330
u7331:
	goto	l6776
u7330:
	line	323
	
l13020:	
;main.c: 323: turnAround();
	fcall	_turnAround
	goto	l13030
	line	324
	
l6776:	
;main.c: 324: else if (rightWall && frontWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7341
	goto	u7340
u7341:
	goto	l6778
u7340:
	
l13022:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7351
	goto	u7350
u7351:
	goto	l6778
u7350:
	line	325
	
l13024:	
;main.c: 325: turnLeft90();
	fcall	_turnLeft90
	goto	l13030
	line	326
	
l6778:	
;main.c: 326: else if(leftWall && frontWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7361
	goto	u7360
u7361:
	goto	l13030
u7360:
	
l13026:	
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7371
	goto	u7370
u7371:
	goto	l13030
u7370:
	line	327
	
l13028:	
;main.c: 327: turnRight90();
	fcall	_turnRight90
	goto	l13030
	
l6780:	
	goto	l13030
	line	328
	
l6779:	
	goto	l13030
	
l6777:	
	
l13030:	
;main.c: 328: ready = 1;
	bsf	(_ready/8),(_ready)&7
	line	329
	
l13032:	
;main.c: 329: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	330
	
l13034:	
;main.c: 330: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	331
;main.c: 331: play_iCreate_song(1);
	movlw	(01h)
	fcall	_play_iCreate_song
	line	332
	
l13036:	
;main.c: 332: rotateIR(12, 0b00001101);
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
	goto	l13038
	line	333
	
l6775:	
	line	335
	
l13038:	
;main.c: 333: }
;main.c: 335: if(start.pressed && ready == 1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_start)^080h,w
	skipz
	goto	u7380
	goto	l13122
u7380:
	
l13040:	
	btfss	(_ready/8),(_ready)&7
	goto	u7391
	goto	u7390
u7391:
	goto	l13122
u7390:
	line	337
	
l13042:	
;main.c: 336: {
;main.c: 337: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	338
;main.c: 338: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	339
;main.c: 339: lookForVictim();
	fcall	_lookForVictim
	line	340
;main.c: 340: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	341
	
l13044:	
;main.c: 341: findWalls();
	fcall	_findWalls
	line	342
	
l13046:	
;main.c: 342: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	343
	
l13048:	
;main.c: 343: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7401
	goto	u7400
u7401:
	goto	l13052
u7400:
	line	344
	
l13050:	
;main.c: 344: goParallel();
	fcall	_goParallel
	goto	l13054
	line	345
	
l6782:	
	line	346
	
l13052:	
;main.c: 345: else
;main.c: 346: rotateIR(12, 0b00001101);
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
	goto	l13054
	
l6783:	
	line	347
	
l13054:	
;main.c: 347: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	348
	
l13056:	
;main.c: 348: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7411
	goto	u7410
u7411:
	goto	l13060
u7410:
	line	349
	
l13058:	
;main.c: 349: frontWallCorrect();
	fcall	_frontWallCorrect
	goto	l13060
	
l6784:	
	line	350
	
l13060:	
;main.c: 350: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	351
;main.c: 351: switch(node)
	goto	l13108
	line	353
;main.c: 352: {
;main.c: 353: case 0:
	
l6786:	
	line	354
	
l13062:	
;main.c: 354: goToNextCell();
	fcall	_goToNextCell
	line	355
;main.c: 355: break;
	goto	l13110
	line	356
;main.c: 356: case 1:
	
l6788:	
	line	357
;main.c: 357: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7421
	goto	u7420
u7421:
	goto	l13076
u7420:
	line	359
	
l13064:	
;main.c: 358: {
;main.c: 359: if (victimZone == 1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	01h
	skipz
	goto	u7431
	goto	u7430
u7431:
	goto	l13068
u7430:
	line	360
	
l13066:	
;main.c: 360: goRight();
	fcall	_goRight
	goto	l13110
	line	361
	
l6790:	
	
l13068:	
;main.c: 361: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u7441
	goto	u7440
u7441:
	goto	l13072
u7440:
	line	362
	
l13070:	
;main.c: 362: goForward();
	fcall	_goForward
	goto	l13110
	line	363
	
l6792:	
	
l13072:	
;main.c: 363: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7451
	goto	u7450
u7451:
	goto	l13110
u7450:
	line	364
	
l13074:	
;main.c: 364: goRight();
	fcall	_goRight
	goto	l13110
	
l6794:	
	goto	l13110
	line	365
	
l6793:	
	goto	l13110
	
l6791:	
;main.c: 365: }
	goto	l13110
	line	366
	
l6789:	
	line	367
	
l13076:	
;main.c: 366: else
;main.c: 367: goToNextCell();
	fcall	_goToNextCell
	goto	l13110
	
l6795:	
	line	368
;main.c: 368: break;
	goto	l13110
	line	369
;main.c: 369: case 2:
	
l6796:	
	line	370
;main.c: 370: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7461
	goto	u7460
u7461:
	goto	l13090
u7460:
	line	372
	
l13078:	
;main.c: 371: {
;main.c: 372: if (victimZone == 2)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	02h
	skipz
	goto	u7471
	goto	u7470
u7471:
	goto	l13082
u7470:
	line	373
	
l13080:	
;main.c: 373: goForward();
	fcall	_goForward
	goto	l13110
	line	374
	
l6798:	
	
l13082:	
;main.c: 374: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7481
	goto	u7480
u7481:
	goto	l13086
u7480:
	line	375
	
l13084:	
;main.c: 375: goRight();
	fcall	_goRight
	goto	l13110
	line	376
	
l6800:	
	
l13086:	
;main.c: 376: else if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u7491
	goto	u7490
u7491:
	goto	l13110
u7490:
	line	377
	
l13088:	
;main.c: 377: goLeft();
	fcall	_goLeft
	goto	l13110
	
l6802:	
	goto	l13110
	line	378
	
l6801:	
	goto	l13110
	
l6799:	
;main.c: 378: }
	goto	l13110
	line	379
	
l6797:	
	line	380
	
l13090:	
;main.c: 379: else
;main.c: 380: goToNextCell();
	fcall	_goToNextCell
	goto	l13110
	
l6803:	
	line	381
;main.c: 381: break;
	goto	l13110
	line	382
;main.c: 382: case 3:
	
l6804:	
	line	383
;main.c: 383: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7501
	goto	u7500
u7501:
	goto	l13104
u7500:
	line	385
	
l13092:	
;main.c: 384: {
;main.c: 385: if (victimZone == 3)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	xorlw	03h
	skipz
	goto	u7511
	goto	u7510
u7511:
	goto	l13096
u7510:
	line	386
	
l13094:	
;main.c: 386: goRight();
	fcall	_goRight
	goto	l13110
	line	387
	
l6806:	
	
l13096:	
;main.c: 387: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u7521
	goto	u7520
u7521:
	goto	l13100
u7520:
	line	388
	
l13098:	
;main.c: 388: goForward();
	fcall	_goForward
	goto	l13110
	line	389
	
l6808:	
	
l13100:	
;main.c: 389: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u7531
	goto	u7530
u7531:
	goto	l13110
u7530:
	line	390
	
l13102:	
;main.c: 390: goLeft();
	fcall	_goLeft
	goto	l13110
	
l6810:	
	goto	l13110
	line	391
	
l6809:	
	goto	l13110
	
l6807:	
;main.c: 391: }
	goto	l13110
	line	392
	
l6805:	
	line	393
	
l13104:	
;main.c: 392: else
;main.c: 393: goToNextCell();
	fcall	_goToNextCell
	goto	l13110
	
l6811:	
	line	394
;main.c: 394: break;
	goto	l13110
	line	395
;main.c: 395: default:
	
l6812:	
	line	396
;main.c: 396: break;
	goto	l13110
	line	397
	
l13106:	
;main.c: 397: }
	goto	l13110
	line	351
	
l6785:	
	
l13108:	
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
	goto	l13062
	xorlw	1^0	; case 1
	skipnz
	goto	l6788
	xorlw	2^1	; case 2
	skipnz
	goto	l6796
	xorlw	3^2	; case 3
	skipnz
	goto	l6804
	goto	l13110
	opt asmopt_on

	line	397
	
l6787:	
	line	398
	
l13110:	
;main.c: 398: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	401
	
l13112:	
;main.c: 401: if(getSuccessfulDrive())
	fcall	_getSuccessfulDrive
	btfss	status,0
	goto	u7541
	goto	u7540
u7541:
	goto	l13122
u7540:
	line	403
	
l13114:	
;main.c: 402: {
;main.c: 403: updateLocation();
	fcall	_updateLocation
	line	404
	
l13116:	
;main.c: 404: updateNode();
	fcall	_updateNode
	line	405
	
l13118:	
;main.c: 405: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u7551
	goto	u7550
u7551:
	goto	l6814
u7550:
	line	406
	
l13120:	
;main.c: 406: checkIfHome();
	fcall	_checkIfHome
	
l6814:	
	line	407
;main.c: 407: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	goto	l13122
	line	408
	
l6813:	
	goto	l13122
	line	409
	
l6781:	
	goto	l13122
	line	410
	
l6773:	
	line	317
	
l13122:	
	btfss	(_home/8),(_home)&7
	goto	u7561
	goto	u7560
u7561:
	goto	l13008
u7560:
	goto	l6816
	
l6815:	
	line	412
	
l6816:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text1990,local,class=CODE,delta=2
global __ptext1990
__ptext1990:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 241 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text1990
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	241
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 0
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	242
	
l12984:	
;main.c: 242: if(!leftWall && (getSomethingInTheWay() != LEFT))
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u7231
	goto	u7230
u7231:
	goto	l6741
u7230:
	
l12986:	
	fcall	_getSomethingInTheWay
	xorlw	01h
	skipnz
	goto	u7241
	goto	u7240
u7241:
	goto	l6741
u7240:
	line	243
	
l12988:	
;main.c: 243: goLeft();
	fcall	_goLeft
	goto	l6747
	line	244
	
l6741:	
;main.c: 244: else if(!frontWall && (getSomethingInTheWay() != FORWARD))
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u7251
	goto	u7250
u7251:
	goto	l6743
u7250:
	
l12990:	
	fcall	_getSomethingInTheWay
	xorlw	0
	skipnz
	goto	u7261
	goto	u7260
u7261:
	goto	l6743
u7260:
	line	245
	
l12992:	
;main.c: 245: goForward();
	fcall	_goForward
	goto	l6747
	line	246
	
l6743:	
;main.c: 246: else if(!rightWall && (getSomethingInTheWay() != RIGHT))
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u7271
	goto	u7270
u7271:
	goto	l12998
u7270:
	
l12994:	
	fcall	_getSomethingInTheWay
	xorlw	03h
	skipnz
	goto	u7281
	goto	u7280
u7281:
	goto	l12998
u7280:
	line	247
	
l12996:	
;main.c: 247: goRight();
	fcall	_goRight
	goto	l6747
	line	248
	
l6745:	
	line	249
	
l12998:	
;main.c: 248: else
;main.c: 249: goBackward();
	fcall	_goBackward
	goto	l6747
	
l6746:	
	goto	l6747
	
l6744:	
	goto	l6747
	
l6742:	
	line	250
	
l6747:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1991,local,class=CODE,delta=2
global __ptext1991
__ptext1991:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 175 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1991
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	175
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	176
	
l12958:	
;main.c: 176: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	178
	
l12960:	
;main.c: 178: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7141
	goto	u7140
	
u7141:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u7154
u7140:
	bcf	(_leftWall/8),(_leftWall)&7
u7154:
	line	179
	
l12962:	
;main.c: 179: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u7161
	goto	u7160
u7161:
	goto	l12966
u7160:
	line	180
	
l12964:	
;main.c: 180: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	goto	l6724
	line	181
	
l6723:	
	line	182
	
l12966:	
;main.c: 181: else
;main.c: 182: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6724:	
	line	184
;main.c: 184: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	185
	
l12968:	
;main.c: 185: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7171
	goto	u7170
	
u7171:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u7184
u7170:
	bcf	(_frontWall/8),(_frontWall)&7
u7184:
	line	186
	
l12970:	
;main.c: 186: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u7191
	goto	u7190
u7191:
	goto	l12974
u7190:
	line	187
	
l12972:	
;main.c: 187: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	goto	l6726
	line	188
	
l6725:	
	line	189
	
l12974:	
;main.c: 188: else
;main.c: 189: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6726:	
	line	191
;main.c: 191: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	192
	
l12976:	
;main.c: 192: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u7201
	goto	u7200
	
u7201:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u7214
u7200:
	bcf	(_rightWall/8),(_rightWall)&7
u7214:
	line	193
	
l12978:	
;main.c: 193: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u7221
	goto	u7220
u7221:
	goto	l12982
u7220:
	line	194
	
l12980:	
;main.c: 194: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l6728
	line	195
	
l6727:	
	line	196
	
l12982:	
;main.c: 195: else
;main.c: 196: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6728:	
	line	198
;main.c: 198: rotateIR(36, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(024h)
	fcall	_rotateIR
	line	199
	
l6729:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text1992,local,class=CODE,delta=2
global __ptext1992
__ptext1992:

;; *************** function _goRight *****************
;; Defined at:
;;		line 198 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text1992
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	198
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 1
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	199
	
l12948:	
;drive.c: 199: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	200
;drive.c: 200: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	201
	
l12950:	
;drive.c: 201: turnRight90();
	fcall	_turnRight90
	line	202
	
l12952:	
;drive.c: 202: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	203
	
l12954:	
;drive.c: 203: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	204
	
l12956:	
;drive.c: 204: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	205
	
l5864:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1993,local,class=CODE,delta=2
global __ptext1993
__ptext1993:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 177 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text1993
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	177
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 1
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	178
	
l12938:	
;drive.c: 178: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	179
;drive.c: 179: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	180
	
l12940:	
;drive.c: 180: turnLeft90();
	fcall	_turnLeft90
	line	181
	
l12942:	
;drive.c: 181: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	182
	
l12944:	
;drive.c: 182: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	183
	
l12946:	
;drive.c: 183: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	184
	
l5858:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1994,local,class=CODE,delta=2
global __ptext1994
__ptext1994:

;; *************** function _goForward *****************
;; Defined at:
;;		line 168 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1994
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	168
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 1
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	169
	
l12932:	
;drive.c: 169: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	170
;drive.c: 170: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	171
	
l12934:	
;drive.c: 171: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	172
	
l12936:	
;drive.c: 172: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	173
	
l5855:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1995,local,class=CODE,delta=2
global __ptext1995
__ptext1995:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text1995
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	157
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 0
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	158
	
l12922:	
;drive.c: 158: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	159
;drive.c: 159: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	160
	
l12924:	
;drive.c: 160: turnAround();
	fcall	_turnAround
	line	161
	
l12926:	
;drive.c: 161: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	162
	
l12928:	
;drive.c: 162: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	163
	
l12930:	
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
psect	text1996,local,class=CODE,delta=2
global __ptext1996
__ptext1996:

;; *************** function _goParallel *****************
;; Defined at:
;;		line 202 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text1996
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	202
	global	__size_of_goParallel
	__size_of_goParallel	equ	__end_of_goParallel-_goParallel
	
_goParallel:	
	opt	stack 1
; Regs used in _goParallel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	203
	
l12876:	
;main.c: 203: PORTC |= 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	iorwf	(7),f	;volatile
	line	205
	
l12878:	
;main.c: 205: int distance, shortestDistance = 999;
	movlw	low(03E7h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@shortestDistance)^080h
	movlw	high(03E7h)
	movwf	((goParallel@shortestDistance)^080h)+1
	line	208
	
l12880:	
;main.c: 206: char stepsToWall;
;main.c: 208: for (int step = -12; step <= 12; step++)
	movlw	low(-12)
	movwf	(goParallel@step)^080h
	movlw	high(-12)
	movwf	((goParallel@step)^080h)+1
	
l12882:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7075
	movlw	low(0Dh)
	subwf	(goParallel@step)^080h,w
u7075:

	skipc
	goto	u7071
	goto	u7070
u7071:
	goto	l12886
u7070:
	goto	l12900
	
l12884:	
	goto	l12900
	line	209
	
l6732:	
	line	210
	
l12886:	
;main.c: 209: {
;main.c: 210: distance = readIR();
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

	line	211
	
l12888:	
;main.c: 211: if(distance < shortestDistance)
	movf	(goParallel@distance+1)^080h,w
	xorlw	80h
	movwf	(??_goParallel+0)^080h+0
	movf	(goParallel@shortestDistance+1)^080h,w
	xorlw	80h
	subwf	(??_goParallel+0)^080h+0,w
	skipz
	goto	u7085
	movf	(goParallel@shortestDistance)^080h,w
	subwf	(goParallel@distance)^080h,w
u7085:

	skipnc
	goto	u7081
	goto	u7080
u7081:
	goto	l12894
u7080:
	line	213
	
l12890:	
;main.c: 212: {
;main.c: 213: stepsToWall = step;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@step)^080h,w
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@stepsToWall)^080h
	line	214
	
l12892:	
;main.c: 214: shortestDistance = distance;
	movf	(goParallel@distance+1)^080h,w
	clrf	(goParallel@shortestDistance+1)^080h
	addwf	(goParallel@shortestDistance+1)^080h
	movf	(goParallel@distance)^080h,w
	clrf	(goParallel@shortestDistance)^080h
	addwf	(goParallel@shortestDistance)^080h

	goto	l12894
	line	215
	
l6734:	
	line	216
	
l12894:	
;main.c: 215: }
;main.c: 216: rotateIR(1, 0b00001101);
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
	line	208
	
l12896:	
	movlw	low(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(goParallel@step)^080h,f
	skipnc
	incf	(goParallel@step+1)^080h,f
	movlw	high(01h)
	addwf	(goParallel@step+1)^080h,f
	
l12898:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7095
	movlw	low(0Dh)
	subwf	(goParallel@step)^080h,w
u7095:

	skipc
	goto	u7091
	goto	u7090
u7091:
	goto	l12886
u7090:
	goto	l12900
	
l6733:	
	line	218
	
l12900:	
;main.c: 217: }
;main.c: 218: rotateIR(12, 0b00001111);
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
	line	220
;main.c: 220: int angleParallelToWall = (int)((stepsToWall*3.75)-6);
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

	line	221
	
l12902:	
;main.c: 221: char angleHighByte = 0;
	clrf	(goParallel@angleHighByte)^080h
	line	222
	
l12904:	
;main.c: 222: char angleLowByte = (char) angleParallelToWall;
	movf	(goParallel@angleParallelToWall)^080h,w
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@angleLowByte)^080h
	line	224
	
l12906:	
;main.c: 224: if(angleParallelToWall < 0)
	btfss	(goParallel@angleParallelToWall+1)^080h,7
	goto	u7101
	goto	u7100
u7101:
	goto	l12910
u7100:
	line	225
	
l12908:	
;main.c: 225: angleParallelToWall = 360 + angleParallelToWall;
	movf	(goParallel@angleParallelToWall)^080h,w
	addlw	low(0168h)
	movwf	(goParallel@angleParallelToWall)^080h
	movf	(goParallel@angleParallelToWall+1)^080h,w
	skipnc
	addlw	1
	addlw	high(0168h)
	movwf	1+(goParallel@angleParallelToWall)^080h
	goto	l12910
	
l6735:	
	line	227
	
l12910:	
;main.c: 227: if(angleParallelToWall > 255)
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7115
	movlw	low(0100h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7115:

	skipc
	goto	u7111
	goto	u7110
u7111:
	goto	l12916
u7110:
	line	229
	
l12912:	
;main.c: 228: {
;main.c: 229: angleHighByte = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@angleHighByte)^080h
	bsf	status,0
	rlf	(goParallel@angleHighByte)^080h,f
	line	230
	
l12914:	
;main.c: 230: angleLowByte = (char)(angleParallelToWall - 255);
	movf	(goParallel@angleParallelToWall)^080h,w
	addlw	01h
	movwf	(??_goParallel+0)^080h+0
	movf	(??_goParallel+0)^080h+0,w
	movwf	(goParallel@angleLowByte)^080h
	goto	l12916
	line	231
	
l6736:	
	line	232
	
l12916:	
;main.c: 231: }
;main.c: 232: if((angleParallelToWall > 8) && (angleParallelToWall < 352))
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(09h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7125
	movlw	low(09h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7125:

	skipc
	goto	u7121
	goto	u7120
u7121:
	goto	l6738
u7120:
	
l12918:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0160h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7135
	movlw	low(0160h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u7135:

	skipnc
	goto	u7131
	goto	u7130
u7131:
	goto	l6738
u7130:
	line	234
	
l12920:	
;main.c: 233: {
;main.c: 234: drive(0, 50, 0, 1);
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
	line	235
;main.c: 235: waitFor(157,angleHighByte,angleLowByte);
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
	line	236
;main.c: 236: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l6738
	line	237
	
l6737:	
	line	238
	
l6738:	
	return
	opt stack 0
GLOBAL	__end_of_goParallel
	__end_of_goParallel:
;; =============== function _goParallel ends ============

	signat	_goParallel,88
	global	_findWall
psect	text1997,local,class=CODE,delta=2
global __ptext1997
__ptext1997:

;; *************** function _findWall *****************
;; Defined at:
;;		line 418 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text1997
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	418
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	419
	
l12864:	
;main.c: 419: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7065
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u7065:

	skipc
	goto	u7061
	goto	u7060
u7061:
	goto	l12872
u7060:
	line	420
	
l12866:	
;main.c: 420: return 0;
	clrc
	
	goto	l6820
	
l12868:	
	goto	l6820
	
l12870:	
	goto	l6820
	line	421
	
l6819:	
	line	422
	
l12872:	
;main.c: 421: else
;main.c: 422: return 1;
	setc
	
	goto	l6820
	
l12874:	
	goto	l6820
	
l6821:	
	line	423
	
l6820:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text1998,local,class=CODE,delta=2
global __ptext1998
__ptext1998:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 265 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text1998
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	265
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 1
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	266
	
l12842:	
;drive.c: 266: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	267
	
l12844:	
;drive.c: 267: int distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	line	268
	
l12846:	
;drive.c: 268: if(distToWall < 45)
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u7025
	movlw	low(02Dh)
	subwf	(frontWallCorrect@distToWall),w
u7025:

	skipnc
	goto	u7021
	goto	u7020
u7021:
	goto	l12854
u7020:
	line	270
	
l12848:	
;drive.c: 269: {
;drive.c: 270: drive(255, 125, 128, 0);
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
	line	271
;drive.c: 271: while(readIR() < 51){}
	goto	l12850
	
l5885:	
	goto	l12850
	
l5884:	
	
l12850:	
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(033h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7035
	movlw	low(033h)
	subwf	(0+(?_readIR)),w
u7035:

	skipc
	goto	u7031
	goto	u7030
u7031:
	goto	l12850
u7030:
	goto	l12852
	
l5886:	
	line	272
	
l12852:	
;drive.c: 272: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	273
;drive.c: 273: }
	goto	l12862
	line	274
	
l5883:	
	
l12854:	
;drive.c: 274: else if(distToWall > 55)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7045
	movlw	low(038h)
	subwf	(frontWallCorrect@distToWall),w
u7045:

	skipc
	goto	u7041
	goto	u7040
u7041:
	goto	l12862
u7040:
	line	276
	
l12856:	
;drive.c: 275: {
;drive.c: 276: drive(0, 250, 128, 0);
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
	line	277
;drive.c: 277: while(readIR() > 51){}
	goto	l12858
	
l5890:	
	goto	l12858
	
l5889:	
	
l12858:	
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(034h))^80h
	subwf	btemp+1,w
	skipz
	goto	u7055
	movlw	low(034h)
	subwf	(0+(?_readIR)),w
u7055:

	skipnc
	goto	u7051
	goto	u7050
u7051:
	goto	l12858
u7050:
	goto	l12860
	
l5891:	
	line	278
	
l12860:	
;drive.c: 278: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l12862
	line	279
	
l5888:	
	goto	l12862
	line	280
	
l5887:	
	
l12862:	
;drive.c: 279: }
;drive.c: 280: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_frontWallCorrect+0)+0
	movf	(??_frontWallCorrect+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	281
	
l5892:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_driveForDistance
psect	text1999,local,class=CODE,delta=2
global __ptext1999
__ptext1999:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 31 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text1999
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	31
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 1
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l12758:	
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
	
l12760:	
;drive.c: 37: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	38
	
l12762:	
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
	
l12764:	
;drive.c: 39: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	41
;drive.c: 41: while(moving)
	goto	l12840
	
l5820:	
	line	43
	
l12766:	
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
	goto	u6905
	movlw	low(064h)
	subwf	(driveForDistance@distance),w
u6905:

	skipc
	goto	u6901
	goto	u6900
u6901:
	goto	l12802
u6900:
	line	46
	
l12768:	
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
	
l12770:	
;drive.c: 49: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6911
	goto	u6910
u6911:
	goto	l12782
u6910:
	line	51
	
l12772:	
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
	
l12774:	
;drive.c: 54: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6921
	goto	u6920
u6921:
	goto	l12782
u6920:
	line	56
	
l12776:	
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
	
l12778:	
;drive.c: 59: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6931
	goto	u6930
u6931:
	goto	l12782
u6930:
	line	61
	
l12780:	
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
	goto	l12782
	line	64
	
l5824:	
	goto	l12782
	line	65
	
l5823:	
	goto	l12782
	line	66
	
l5822:	
	line	67
	
l12782:	
;drive.c: 64: }
;drive.c: 65: }
;drive.c: 66: }
;drive.c: 67: if(cliff == 1)
	movf	(driveForDistance@cliff),w	;volatile
	xorlw	01h
	skipz
	goto	u6941
	goto	u6940
u6941:
	goto	l12802
u6940:
	line	69
	
l12784:	
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
	
l12786:	
;drive.c: 72: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u6951
	goto	u6950
u6951:
	goto	l12794
u6950:
	line	74
	
l12788:	
;drive.c: 73: {
;drive.c: 74: somethingInTheWay = LEFT;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay)^080h,f	;volatile
	line	75
	
l12790:	
;drive.c: 75: turnRight90();
	fcall	_turnRight90
	line	76
	
l12792:	
;drive.c: 76: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	77
;drive.c: 77: }
	goto	l5827
	line	78
	
l5826:	
	
l12794:	
;drive.c: 78: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u6961
	goto	u6960
u6961:
	goto	l5828
u6960:
	line	80
	
l12796:	
;drive.c: 79: {
;drive.c: 80: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	line	81
	
l12798:	
;drive.c: 81: turnLeft90();
	fcall	_turnLeft90
	line	82
	
l12800:	
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
	goto	l12802
	line	87
	
l5825:	
	goto	l12802
	line	88
	
l5821:	
	line	91
	
l12802:	
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
	
l12804:	
;drive.c: 94: if(virtualWall == 1)
	movf	(driveForDistance@virtualWall),w	;volatile
	xorlw	01h
	skipz
	goto	u6971
	goto	u6970
u6971:
	goto	l12824
u6970:
	line	96
	
l12806:	
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
	
l12808:	
;drive.c: 100: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u6981
	goto	u6980
u6981:
	goto	l12816
u6980:
	line	102
	
l12810:	
;drive.c: 101: {
;drive.c: 102: somethingInTheWay = LEFT;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_somethingInTheWay)^080h	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay)^080h,f	;volatile
	line	103
	
l12812:	
;drive.c: 103: turnRight90();
	fcall	_turnRight90
	line	104
	
l12814:	
;drive.c: 104: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	105
;drive.c: 105: }
	goto	l5832
	line	106
	
l5831:	
	
l12816:	
;drive.c: 106: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u6991
	goto	u6990
u6991:
	goto	l5833
u6990:
	line	108
	
l12818:	
;drive.c: 107: {
;drive.c: 108: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	line	109
	
l12820:	
;drive.c: 109: turnLeft90();
	fcall	_turnLeft90
	line	110
	
l12822:	
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
	goto	l12824
	line	115
	
l5830:	
	line	118
	
l12824:	
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
	
l12826:	
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
	
l12828:	
;drive.c: 123: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	124
	
l12830:	
;drive.c: 124: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u7005
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u7005:

	skipc
	goto	u7001
	goto	u7000
u7001:
	goto	l12840
u7000:
	line	126
	
l12832:	
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
	
l12834:	
;drive.c: 127: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	line	128
	
l12836:	
;drive.c: 128: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	129
	
l12838:	
;drive.c: 129: somethingInTheWay = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_somethingInTheWay)^080h	;volatile
	goto	l12840
	line	130
	
l5835:	
	goto	l12840
	line	131
	
l5819:	
	line	41
	
l12840:	
	btfsc	(_moving/8),(_moving)&7
	goto	u7011
	goto	u7010
u7011:
	goto	l12766
u7010:
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
psect	text2000,local,class=CODE,delta=2
global __ptext2000
__ptext2000:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 253 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2000
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	253
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	254
	
l12734:	
;main.c: 254: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	255
;main.c: 255: switch(getOrientation())
	goto	l12754
	line	257
;main.c: 256: {
;main.c: 257: case NORTH:
	
l6751:	
	line	258
	
l12736:	
;main.c: 258: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	259
	
l12738:	
;main.c: 259: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	260
;main.c: 260: break;
	goto	l12756
	line	261
;main.c: 261: case SOUTH:
	
l6753:	
	line	262
	
l12740:	
;main.c: 262: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	263
	
l12742:	
;main.c: 263: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	264
;main.c: 264: break;
	goto	l12756
	line	265
;main.c: 265: case EAST:
	
l6754:	
	line	266
	
l12744:	
;main.c: 266: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	267
	
l12746:	
;main.c: 267: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	268
;main.c: 268: break;
	goto	l12756
	line	269
;main.c: 269: case WEST:
	
l6755:	
	line	270
	
l12748:	
;main.c: 270: --xCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_xCoord),f	;volatile
	line	271
	
l12750:	
;main.c: 271: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	272
;main.c: 272: break;
	goto	l12756
	line	273
;main.c: 273: default:
	
l6756:	
	line	274
;main.c: 274: break;
	goto	l12756
	line	275
	
l12752:	
;main.c: 275: }
	goto	l12756
	line	255
	
l6750:	
	
l12754:	
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
	goto	l12748
	xorlw	1^0	; case 1
	skipnz
	goto	l12740
	xorlw	2^1	; case 2
	skipnz
	goto	l12744
	xorlw	3^2	; case 3
	skipnz
	goto	l12736
	goto	l12756
	opt asmopt_on

	line	275
	
l6752:	
	line	277
	
l12756:	
;main.c: 277: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	278
;main.c: 278: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	279
;main.c: 279: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	280
;main.c: 280: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	281
	
l6757:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text2001,local,class=CODE,delta=2
global __ptext2001
__ptext2001:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 150 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2001
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	150
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 3
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	151
	
l12712:	
;main.c: 151: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	152
;main.c: 152: ser_putch(17);
	movlw	(011h)
	fcall	_ser_putch
	line	153
;main.c: 153: char victim = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(lookForVictim@victim)
	line	155
	
l12714:	
;main.c: 155: if(victim > 241 && victim != 255)
	movlw	(0F2h)
	subwf	(lookForVictim@victim),w
	skipc
	goto	u6871
	goto	u6870
u6871:
	goto	l6720
u6870:
	
l12716:	
	movf	(lookForVictim@victim),w
	xorlw	0FFh
	skipnz
	goto	u6881
	goto	u6880
u6881:
	goto	l6720
u6880:
	line	157
	
l12718:	
;main.c: 156: {
;main.c: 157: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u6891
	goto	u6890
u6891:
	goto	l12728
u6890:
	line	159
	
l12720:	
;main.c: 158: {
;main.c: 159: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	160
	
l12722:	
;main.c: 160: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	161
	
l12724:	
;main.c: 161: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	162
	
l12726:	
;main.c: 162: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	163
;main.c: 163: }
	goto	l6720
	line	164
	
l6718:	
	line	166
	
l12728:	
;main.c: 164: else
;main.c: 165: {
;main.c: 166: victimZone = getVictimZone(xCoord, yCoord);
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
	line	167
	
l12730:	
;main.c: 167: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	168
	
l12732:	
;main.c: 168: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l6720
	line	169
	
l6719:	
	goto	l6720
	line	170
	
l6717:	
	line	171
	
l6720:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text2002,local,class=CODE,delta=2
global __ptext2002
__ptext2002:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 139 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2002
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	139
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 3
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	140
	
l12700:	
;main.c: 140: if((xCoord == getFinalX()) && (yCoord == getFinalY()))
	fcall	_getFinalX
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u6851
	goto	u6850
u6851:
	goto	l6714
u6850:
	
l12702:	
	fcall	_getFinalY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u6861
	goto	u6860
u6861:
	goto	l6714
u6860:
	line	142
	
l12704:	
;main.c: 141: {
;main.c: 142: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	143
	
l12706:	
;main.c: 143: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	144
	
l12708:	
;main.c: 144: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	145
	
l12710:	
;main.c: 145: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l6714
	line	146
	
l6713:	
	line	147
	
l6714:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text2003,local,class=CODE,delta=2
global __ptext2003
__ptext2003:

;; *************** function _init *****************
;; Defined at:
;;		line 102 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2003
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	102
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	103
	
l12668:	
;main.c: 103: start.pressed = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	line	104
	
l12670:	
;main.c: 104: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	line	106
	
l12672:	
;main.c: 106: init_adc();
	fcall	_init_adc
	line	107
	
l12674:	
;main.c: 107: lcd_init();
	fcall	_lcd_init
	line	109
	
l12676:	
;main.c: 109: TRISB = 0b00000001;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	112
	
l12678:	
;main.c: 112: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	114
	
l12680:	
;main.c: 114: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	115
	
l12682:	
;main.c: 115: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	116
	
l12684:	
;main.c: 116: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	117
	
l12686:	
;main.c: 117: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	118
	
l12688:	
;main.c: 118: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	121
	
l12690:	
;main.c: 121: PEIE = 1;
	bsf	(94/8),(94)&7
	line	122
	
l12692:	
;main.c: 122: GIE = 1;
	bsf	(95/8),(95)&7
	line	124
	
l12694:	
;main.c: 124: ser_init();
	fcall	_ser_init
	line	125
	
l12696:	
;main.c: 125: initIRobot();
	fcall	_initIRobot
	line	126
	
l12698:	
;main.c: 126: initSongs();
	fcall	_initSongs
	line	127
	
l6707:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_goReverse
psect	text2004,local,class=CODE,delta=2
global __ptext2004
__ptext2004:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 187 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2004
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	187
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 1
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	188
	
l12658:	
;drive.c: 188: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	189
;drive.c: 189: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	190
	
l12660:	
;drive.c: 190: drive(255, 125, 128, 0);
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
	line	191
	
l12662:	
;drive.c: 191: waitFor(156,254,12);
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
	line	192
	
l12664:	
;drive.c: 192: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	193
	
l12666:	
;drive.c: 193: _delay((unsigned long)((2000)*(20000000/4000.0)));
	opt asmopt_off
movlw  51
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_goReverse+0)+0+2),f
movlw	188
movwf	((??_goReverse+0)+0+1),f
	movlw	16
movwf	((??_goReverse+0)+0),f
u7577:
	decfsz	((??_goReverse+0)+0),f
	goto	u7577
	decfsz	((??_goReverse+0)+0+1),f
	goto	u7577
	decfsz	((??_goReverse+0)+0+2),f
	goto	u7577
opt asmopt_on

	line	194
	
l5861:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_readIR
psect	text2005,local,class=CODE,delta=2
global __ptext2005
__ptext2005:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
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
psect	text2005
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 1
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l12652:	
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
	
l12654:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l5081
	
l12656:	
	line	36
	
l5081:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_findFinalDestination
psect	text2006,local,class=CODE,delta=2
global __ptext2006
__ptext2006:

;; *************** function _findFinalDestination *****************
;; Defined at:
;;		line 12 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
;; Parameters:    Size  Location     Type
;;  virtualWallX    1    wreg     unsigned char 
;;  virtualWallY    1   14[BANK0 ] unsigned char 
;;  robotOrienta    1   15[BANK0 ] enum E1096
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
psect	text2006
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
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
	
l12572:	
;map.c: 13: switch (virtualWallX)
	goto	l12648
	line	15
;map.c: 14: {
;map.c: 15: case 0:
	
l2849:	
	line	16
;map.c: 16: switch (virtualWallY)
	goto	l12582
	line	20
;map.c: 17: {
;map.c: 20: case 1:
	
l2851:	
	line	21
;map.c: 21: finalX = 0;
	clrf	(_finalX)
	line	22
	
l12574:	
;map.c: 22: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	23
;map.c: 23: break;
	goto	l12650
	line	24
;map.c: 24: case 2:
	
l2853:	
	line	25
;map.c: 25: finalX = 0;
	clrf	(_finalX)
	line	26
	
l12576:	
;map.c: 26: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	27
;map.c: 27: break;
	goto	l12650
	line	28
;map.c: 28: case 3:
	
l2854:	
	line	29
;map.c: 29: finalX = 0;
	clrf	(_finalX)
	line	30
	
l12578:	
;map.c: 30: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	31
;map.c: 31: break;
	goto	l12650
	line	32
;map.c: 32: default:
	
l2855:	
	line	33
;map.c: 33: break;
	goto	l12650
	line	34
	
l12580:	
;map.c: 34: }
	goto	l12650
	line	16
	
l2850:	
	
l12582:	
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
	goto	l12650
	opt asmopt_on

	line	34
	
l2852:	
	line	35
;map.c: 35: break;
	goto	l12650
	line	37
;map.c: 37: case 1:
	
l2857:	
	line	38
;map.c: 38: switch (virtualWallY)
	goto	l12600
	line	40
;map.c: 39: {
;map.c: 40: case 0:
	
l2859:	
	line	41
	
l12584:	
;map.c: 41: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	42
	
l12586:	
;map.c: 42: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	43
;map.c: 43: break;
	goto	l12650
	line	44
;map.c: 44: case 1:
	
l2861:	
	line	45
	
l12588:	
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
	goto	l12650
	line	48
;map.c: 48: case 2:
	
l2862:	
	line	49
	
l12590:	
;map.c: 49: finalX = 1;
	bcf	status, 5	;RP0=0, select bank0
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	50
	
l12592:	
;map.c: 50: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	51
;map.c: 51: break;
	goto	l12650
	line	52
;map.c: 52: case 3:
	
l2863:	
	line	53
	
l12594:	
;map.c: 53: finalX = 1;
	bcf	status, 5	;RP0=0, select bank0
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	54
	
l12596:	
;map.c: 54: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_finalY)^080h
	line	55
;map.c: 55: break;
	goto	l12650
	line	56
;map.c: 56: default:
	
l2864:	
	line	57
;map.c: 57: break;
	goto	l12650
	line	58
	
l12598:	
;map.c: 58: }
	goto	l12650
	line	38
	
l2858:	
	
l12600:	
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
	goto	l12584
	xorlw	1^0	; case 1
	skipnz
	goto	l12588
	xorlw	2^1	; case 2
	skipnz
	goto	l12590
	xorlw	3^2	; case 3
	skipnz
	goto	l12594
	goto	l12650
	opt asmopt_on

	line	58
	
l2860:	
	line	59
;map.c: 59: break;
	goto	l12650
	line	61
;map.c: 61: case 2:
	
l2865:	
	line	62
;map.c: 62: switch (virtualWallY)
	goto	l12618
	line	64
;map.c: 63: {
;map.c: 64: case 0:
	
l2867:	
	line	65
	
l12602:	
;map.c: 65: if(robotOrientation == WEST)
	movf	(findFinalDestination@robotOrientation),f
	skipz
	goto	u6821
	goto	u6820
u6821:
	goto	l12650
u6820:
	line	67
	
l12604:	
;map.c: 66: {
;map.c: 67: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	68
	
l12606:	
;map.c: 68: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	goto	l12650
	line	69
	
l2868:	
	line	70
;map.c: 69: }
;map.c: 70: break;
	goto	l12650
	line	71
;map.c: 71: case 1:
	
l2870:	
	line	72
	
l12608:	
;map.c: 72: finalX = 2;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	73
	
l12610:	
;map.c: 73: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	74
;map.c: 74: break;
	goto	l12650
	line	75
;map.c: 75: case 2:
	
l2871:	
	line	76
	
l12612:	
;map.c: 76: if(robotOrientation == EAST)
	bcf	status, 5	;RP0=0, select bank0
	movf	(findFinalDestination@robotOrientation),w
	xorlw	02h
	skipz
	goto	u6831
	goto	u6830
u6831:
	goto	l12650
u6830:
	line	78
	
l12614:	
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
	goto	l12650
	line	80
	
l2872:	
	line	81
;map.c: 80: }
;map.c: 81: break;
	goto	l12650
	line	84
;map.c: 84: default:
	
l2873:	
	line	85
;map.c: 85: break;
	goto	l12650
	line	86
	
l12616:	
;map.c: 86: }
	goto	l12650
	line	62
	
l2866:	
	
l12618:	
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
	goto	l12602
	xorlw	1^0	; case 1
	skipnz
	goto	l12608
	xorlw	2^1	; case 2
	skipnz
	goto	l12612
	goto	l12650
	opt asmopt_on

	line	86
	
l2869:	
	line	87
;map.c: 87: break;
	goto	l12650
	line	89
;map.c: 89: case 3:
	
l2874:	
	line	90
;map.c: 90: switch (virtualWallY)
	goto	l12628
	line	92
;map.c: 91: {
;map.c: 92: case 0:
	
l2876:	
	line	93
	
l12620:	
;map.c: 93: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	94
	
l12622:	
;map.c: 94: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	95
;map.c: 95: break;
	goto	l12650
	line	98
;map.c: 98: case 2:
	
l2878:	
	line	99
	
l12624:	
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
	goto	l12650
	line	104
;map.c: 104: default:
	
l2879:	
	line	105
;map.c: 105: break;
	goto	l12650
	line	106
	
l12626:	
;map.c: 106: }
	goto	l12650
	line	90
	
l2875:	
	
l12628:	
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
	goto	l12620
	xorlw	2^0	; case 2
	skipnz
	goto	l12624
	goto	l12650
	opt asmopt_on

	line	106
	
l2877:	
	line	107
;map.c: 107: break;
	goto	l12650
	line	109
;map.c: 109: case 4:
	
l2880:	
	line	110
;map.c: 110: switch (virtualWallY)
	goto	l12644
	line	112
;map.c: 111: {
;map.c: 112: case 0:
	
l2882:	
	line	113
	
l12630:	
;map.c: 113: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	114
	
l12632:	
;map.c: 114: finalY = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	line	115
;map.c: 115: break;
	goto	l12650
	line	116
;map.c: 116: case 1:
	
l2884:	
	line	117
	
l12634:	
;map.c: 117: finalX = 4;
	movlw	(04h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	118
	
l12636:	
;map.c: 118: finalY = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_finalY)^080h
	bsf	status,0
	rlf	(_finalY)^080h,f
	line	119
;map.c: 119: break;
	goto	l12650
	line	120
;map.c: 120: case 2:
	
l2885:	
	line	121
	
l12638:	
;map.c: 121: if (robotOrientation == SOUTH)
	bcf	status, 5	;RP0=0, select bank0
	movf	(findFinalDestination@robotOrientation),w
	xorlw	01h
	skipz
	goto	u6841
	goto	u6840
u6841:
	goto	l12650
u6840:
	line	123
	
l12640:	
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
	goto	l12650
	line	125
	
l2886:	
	line	126
;map.c: 125: }
;map.c: 126: break;
	goto	l12650
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
	goto	l12650
	line	131
;map.c: 131: default:
	
l2888:	
	line	132
;map.c: 132: break;
	goto	l12650
	line	133
	
l12642:	
;map.c: 133: }
	goto	l12650
	line	110
	
l2881:	
	
l12644:	
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
	goto	l12630
	xorlw	1^0	; case 1
	skipnz
	goto	l12634
	xorlw	2^1	; case 2
	skipnz
	goto	l12638
	xorlw	3^2	; case 3
	skipnz
	goto	l2887
	goto	l12650
	opt asmopt_on

	line	133
	
l2883:	
	line	134
;map.c: 134: break;
	goto	l12650
	line	136
;map.c: 136: default:
	
l2889:	
	line	137
;map.c: 137: break;
	goto	l12650
	line	138
	
l12646:	
;map.c: 138: }
	goto	l12650
	line	13
	
l2848:	
	
l12648:	
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
	goto	l12582
	xorlw	1^0	; case 1
	skipnz
	goto	l12600
	xorlw	2^1	; case 2
	skipnz
	goto	l12618
	xorlw	3^2	; case 3
	skipnz
	goto	l12628
	xorlw	4^3	; case 4
	skipnz
	goto	l12644
	goto	l12650
	opt asmopt_on

	line	138
	
l2856:	
	line	140
	
l12650:	
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
	global	_checkIfHome
psect	text2007,local,class=CODE,delta=2
global __ptext2007
__ptext2007:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 296 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2007
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	296
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 3
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	297
	
l12564:	
;main.c: 297: if((xCoord == 1) && (yCoord == 3))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u6801
	goto	u6800
u6801:
	goto	l6770
u6800:
	
l12566:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u6811
	goto	u6810
u6811:
	goto	l6770
u6810:
	line	299
	
l12568:	
;main.c: 298: {
;main.c: 299: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	300
;main.c: 300: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	301
	
l12570:	
;main.c: 301: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l6770
	line	302
	
l6769:	
	line	303
	
l6770:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_turnAround
psect	text2008,local,class=CODE,delta=2
global __ptext2008
__ptext2008:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 208 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2008
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	208
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	209
	
l12558:	
;drive.c: 209: drive(0, 50, 0, 1);
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
	line	210
;drive.c: 210: waitFor(157,0,170);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0AAh)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	211
;drive.c: 211: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	212
	
l12560:	
;drive.c: 212: _delay((unsigned long)((6000)*(20000000/4000.0)));
	opt asmopt_off
movlw  153
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	50
movwf	((??_turnAround+0)+0+1),f
	movlw	56
movwf	((??_turnAround+0)+0),f
u7587:
	decfsz	((??_turnAround+0)+0),f
	goto	u7587
	decfsz	((??_turnAround+0)+0+1),f
	goto	u7587
	decfsz	((??_turnAround+0)+0+2),f
	goto	u7587
	nop2
opt asmopt_on

	line	213
	
l12562:	
;drive.c: 213: _delay((unsigned long)((6000)*(20000000/4000.0)));
	opt asmopt_off
movlw  153
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	50
movwf	((??_turnAround+0)+0+1),f
	movlw	56
movwf	((??_turnAround+0)+0),f
u7597:
	decfsz	((??_turnAround+0)+0),f
	goto	u7597
	decfsz	((??_turnAround+0)+0+1),f
	goto	u7597
	decfsz	((??_turnAround+0)+0+2),f
	goto	u7597
	nop2
opt asmopt_on

	line	214
	
l5867:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_turnLeft90
psect	text2009,local,class=CODE,delta=2
global __ptext2009
__ptext2009:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 217 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
;;		_goLeft
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2009
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	217
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 1
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	218
	
l12554:	
;drive.c: 218: drive(0, 50, 0, 1);
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
	line	219
;drive.c: 219: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	220
;drive.c: 220: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	221
	
l12556:	
;drive.c: 221: _delay((unsigned long)((6000)*(20000000/4000.0)));
	opt asmopt_off
movlw  153
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	50
movwf	((??_turnLeft90+0)+0+1),f
	movlw	56
movwf	((??_turnLeft90+0)+0),f
u7607:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u7607
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u7607
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u7607
	nop2
opt asmopt_on

	line	222
	
l5870:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnRight90
psect	text2010,local,class=CODE,delta=2
global __ptext2010
__ptext2010:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 225 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2010
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	225
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 1
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	226
	
l12550:	
;drive.c: 226: drive(0, 50, 255, 255);
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
	line	227
;drive.c: 227: waitFor(157,255,169);
	movlw	(0FFh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_turnRight90+0)+0
	movf	(??_turnRight90+0)+0,w
	movwf	(?_waitFor)
	movlw	(0A9h)
	movwf	(??_turnRight90+1)+0
	movf	(??_turnRight90+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	228
;drive.c: 228: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	229
	
l12552:	
;drive.c: 229: _delay((unsigned long)((6000)*(20000000/4000.0)));
	opt asmopt_off
movlw  153
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	50
movwf	((??_turnRight90+0)+0+1),f
	movlw	56
movwf	((??_turnRight90+0)+0),f
u7617:
	decfsz	((??_turnRight90+0)+0),f
	goto	u7617
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u7617
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u7617
	nop2
opt asmopt_on

	line	230
	
l5873:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_initSongs
psect	text2011,local,class=CODE,delta=2
global __ptext2011
__ptext2011:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 31 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
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
psect	text2011
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
	line	31
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	32
	
l12548:	
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
psect	text2012,local,class=CODE,delta=2
global __ptext2012
__ptext2012:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2012
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l12528:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l12530:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l12532:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l12534:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l12536:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l12538:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l12540:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l12542:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l12544:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l12546:	
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
psect	text2013,local,class=CODE,delta=2
global __ptext2013
__ptext2013:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2013
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
	
l12526:	
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
psect	text2014,local,class=CODE,delta=2
global __ptext2014
__ptext2014:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2014
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
	
l12522:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l12524:	
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
	global	_lcd_write_string
psect	text2015,local,class=CODE,delta=2
global __ptext2015
__ptext2015:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2015
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
	
l12514:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l12520
	
l2136:	
	
l12516:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l12518:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l12520
	
l2135:	
	
l12520:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u6791
	goto	u6790
u6791:
	goto	l12516
u6790:
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
psect	text2016,local,class=CODE,delta=2
global __ptext2016
__ptext2016:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
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
psect	text2016
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
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
	
l11178:	
;adc.c: 8: switch(channel)
	goto	l11186
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
	goto	l11188
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
	goto	l11188
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
	goto	l11188
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
	goto	l11188
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
	goto	l11188
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l11180:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l11182:	
	goto	l697
	line	39
	
l11184:	
;adc.c: 39: }
	goto	l11188
	line	8
	
l689:	
	
l11186:	
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
	goto	l11180
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l11188:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u7627:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u7627
opt asmopt_on

	line	43
	
l11190:	
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
	
l11192:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	___lbtoft
psect	text2017,local,class=CODE,delta=2
global __ptext2017
__ptext2017:

;; *************** function ___lbtoft *****************
;; Defined at:
;;		line 28 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\lbtoft.c"
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
psect	text2017
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\lbtoft.c"
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
	
l12510:	
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
	goto	l7635
	
l12512:	
	line	30
	
l7635:	
	return
	opt stack 0
GLOBAL	__end_of___lbtoft
	__end_of___lbtoft:
;; =============== function ___lbtoft ends ============

	signat	___lbtoft,4219
	global	___ftmul
psect	text2018,local,class=CODE,delta=2
global __ptext2018
__ptext2018:

;; *************** function ___ftmul *****************
;; Defined at:
;;		line 52 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\ftmul.c"
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
psect	text2018
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\ftmul.c"
	line	52
	global	__size_of___ftmul
	__size_of___ftmul	equ	__end_of___ftmul-___ftmul
	
___ftmul:	
	opt	stack 3
; Regs used in ___ftmul: [wreg+status,2+status,0+pclath+cstack]
	line	56
	
l12460:	
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
	goto	u6651
	goto	u6650
u6651:
	goto	l12466
u6650:
	line	57
	
l12462:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l7609
	
l12464:	
	goto	l7609
	
l7608:	
	line	58
	
l12466:	
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
	goto	u6661
	goto	u6660
u6661:
	goto	l12472
u6660:
	line	59
	
l12468:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l7609
	
l12470:	
	goto	l7609
	
l7610:	
	line	60
	
l12472:	
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
u6675:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6670:
	addlw	-1
	skipz
	goto	u6675
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
u6685:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6680:
	addlw	-1
	skipz
	goto	u6685
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
	
l12474:	
	bsf	(___ftmul@f1)+(15/8),(15)&7
	line	66
	
l12476:	
	bsf	(___ftmul@f2)+(15/8),(15)&7
	line	67
	
l12478:	
	movlw	0FFh
	andwf	(___ftmul@f2),f
	movlw	0FFh
	andwf	(___ftmul@f2+1),f
	movlw	0
	andwf	(___ftmul@f2+2),f
	line	68
	
l12480:	
	movlw	0
	movwf	(___ftmul@f3_as_product)
	movlw	0
	movwf	(___ftmul@f3_as_product+1)
	movlw	0
	movwf	(___ftmul@f3_as_product+2)
	line	69
	
l12482:	
	movlw	(07h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l12484
	line	70
	
l7611:	
	line	71
	
l12484:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6691
	goto	u6690
u6691:
	goto	l12488
u6690:
	line	72
	
l12486:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6701
	addwf	(___ftmul@f3_as_product+1),f
u6701:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6702
	addwf	(___ftmul@f3_as_product+2),f
u6702:

	goto	l12488
	
l7612:	
	line	73
	
l12488:	
	movlw	01h
u6715:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6715

	line	74
	
l12490:	
	movlw	01h
u6725:
	clrc
	rlf	(___ftmul@f2),f
	rlf	(___ftmul@f2+1),f
	rlf	(___ftmul@f2+2),f
	addlw	-1
	skipz
	goto	u6725
	line	75
	
l12492:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6731
	goto	u6730
u6731:
	goto	l12484
u6730:
	goto	l12494
	
l7613:	
	line	76
	
l12494:	
	movlw	(09h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l12496
	line	77
	
l7614:	
	line	78
	
l12496:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6741
	goto	u6740
u6741:
	goto	l12500
u6740:
	line	79
	
l12498:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6751
	addwf	(___ftmul@f3_as_product+1),f
u6751:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6752
	addwf	(___ftmul@f3_as_product+2),f
u6752:

	goto	l12500
	
l7615:	
	line	80
	
l12500:	
	movlw	01h
u6765:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6765

	line	81
	
l12502:	
	movlw	01h
u6775:
	clrc
	rrf	(___ftmul@f3_as_product+2),f
	rrf	(___ftmul@f3_as_product+1),f
	rrf	(___ftmul@f3_as_product),f
	addlw	-1
	skipz
	goto	u6775

	line	82
	
l12504:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6781
	goto	u6780
u6781:
	goto	l12496
u6780:
	goto	l12506
	
l7616:	
	line	83
	
l12506:	
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
	goto	l7609
	
l12508:	
	line	84
	
l7609:	
	return
	opt stack 0
GLOBAL	__end_of___ftmul
	__end_of___ftmul:
;; =============== function ___ftmul ends ============

	signat	___ftmul,8315
	global	___ftadd
psect	text2019,local,class=CODE,delta=2
global __ptext2019
__ptext2019:

;; *************** function ___ftadd *****************
;; Defined at:
;;		line 87 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\ftadd.c"
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
psect	text2019
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\ftadd.c"
	line	87
	global	__size_of___ftadd
	__size_of___ftadd	equ	__end_of___ftadd-___ftadd
	
___ftadd:	
	opt	stack 3
; Regs used in ___ftadd: [wreg+status,2+status,0+pclath+cstack]
	line	90
	
l12390:	
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
	
l12392:	
	movf	(___ftadd@exp1)^080h,w
	skipz
	goto	u6410
	goto	l12398
u6410:
	
l12394:	
	movf	(___ftadd@exp2)^080h,w
	subwf	(___ftadd@exp1)^080h,w
	skipnc
	goto	u6421
	goto	u6420
u6421:
	goto	l12402
u6420:
	
l12396:	
	decf	(___ftadd@exp1)^080h,w
	xorlw	0ffh
	addwf	(___ftadd@exp2)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movlw	(019h)
	subwf	0+(??___ftadd+0)+0,w
	skipc
	goto	u6431
	goto	u6430
u6431:
	goto	l12402
u6430:
	goto	l12398
	
l7556:	
	line	93
	
l12398:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@f2)^080h,w
	movwf	(?___ftadd)^080h
	movf	(___ftadd@f2+1)^080h,w
	movwf	(?___ftadd+1)^080h
	movf	(___ftadd@f2+2)^080h,w
	movwf	(?___ftadd+2)^080h
	goto	l7557
	
l12400:	
	goto	l7557
	
l7554:	
	line	94
	
l12402:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6440
	goto	l7560
u6440:
	
l12404:	
	movf	(___ftadd@exp1)^080h,w
	subwf	(___ftadd@exp2)^080h,w
	skipnc
	goto	u6451
	goto	u6450
u6451:
	goto	l12408
u6450:
	
l12406:	
	decf	(___ftadd@exp2)^080h,w
	xorlw	0ffh
	addwf	(___ftadd@exp1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movlw	(019h)
	subwf	0+(??___ftadd+0)+0,w
	skipc
	goto	u6461
	goto	u6460
u6461:
	goto	l12408
u6460:
	
l7560:	
	line	95
	goto	l7557
	
l7558:	
	line	96
	
l12408:	
	movlw	(06h)
	bcf	status, 5	;RP0=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(___ftadd@sign)^080h
	line	97
	
l12410:	
	btfss	(___ftadd@f1+2)^080h,(23)&7
	goto	u6471
	goto	u6470
u6471:
	goto	l7561
u6470:
	line	98
	
l12412:	
	bsf	(___ftadd@sign)^080h+(7/8),(7)&7
	
l7561:	
	line	99
	btfss	(___ftadd@f2+2)^080h,(23)&7
	goto	u6481
	goto	u6480
u6481:
	goto	l7562
u6480:
	line	100
	
l12414:	
	bsf	(___ftadd@sign)^080h+(6/8),(6)&7
	
l7562:	
	line	101
	bsf	(___ftadd@f1)^080h+(15/8),(15)&7
	line	102
	
l12416:	
	movlw	0FFh
	andwf	(___ftadd@f1)^080h,f
	movlw	0FFh
	andwf	(___ftadd@f1+1)^080h,f
	movlw	0
	andwf	(___ftadd@f1+2)^080h,f
	line	103
	
l12418:	
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
	goto	u6491
	goto	u6490
u6491:
	goto	l12430
u6490:
	goto	l12420
	line	109
	
l7564:	
	line	110
	
l12420:	
	movlw	01h
u6505:
	clrc
	rlf	(___ftadd@f2)^080h,f
	rlf	(___ftadd@f2+1)^080h,f
	rlf	(___ftadd@f2+2)^080h,f
	addlw	-1
	skipz
	goto	u6505
	line	111
	movlw	low(01h)
	subwf	(___ftadd@exp2)^080h,f
	line	112
	
l12422:	
	movf	(___ftadd@exp2)^080h,w
	xorwf	(___ftadd@exp1)^080h,w
	skipnz
	goto	u6511
	goto	u6510
u6511:
	goto	l12428
u6510:
	
l12424:	
	movlw	low(01h)
	subwf	(___ftadd@sign)^080h,f
	movf	((___ftadd@sign)^080h),w
	andlw	07h
	btfss	status,2
	goto	u6521
	goto	u6520
u6521:
	goto	l12420
u6520:
	goto	l12428
	
l7566:	
	goto	l12428
	
l7567:	
	line	113
	goto	l12428
	
l7569:	
	line	114
	
l12426:	
	movlw	01h
u6535:
	clrc
	rrf	(___ftadd@f1+2)^080h,f
	rrf	(___ftadd@f1+1)^080h,f
	rrf	(___ftadd@f1)^080h,f
	addlw	-1
	skipz
	goto	u6535

	line	115
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(___ftadd@exp1)^080h,f
	goto	l12428
	line	116
	
l7568:	
	line	113
	
l12428:	
	movf	(___ftadd@exp1)^080h,w
	xorwf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6541
	goto	u6540
u6541:
	goto	l12426
u6540:
	goto	l7571
	
l7570:	
	line	117
	goto	l7571
	
l7563:	
	
l12430:	
	movf	(___ftadd@exp1)^080h,w
	subwf	(___ftadd@exp2)^080h,w
	skipnc
	goto	u6551
	goto	u6550
u6551:
	goto	l7571
u6550:
	goto	l12432
	line	120
	
l7573:	
	line	121
	
l12432:	
	movlw	01h
u6565:
	clrc
	rlf	(___ftadd@f1)^080h,f
	rlf	(___ftadd@f1+1)^080h,f
	rlf	(___ftadd@f1+2)^080h,f
	addlw	-1
	skipz
	goto	u6565
	line	122
	movlw	low(01h)
	subwf	(___ftadd@exp1)^080h,f
	line	123
	
l12434:	
	movf	(___ftadd@exp2)^080h,w
	xorwf	(___ftadd@exp1)^080h,w
	skipnz
	goto	u6571
	goto	u6570
u6571:
	goto	l12440
u6570:
	
l12436:	
	movlw	low(01h)
	subwf	(___ftadd@sign)^080h,f
	movf	((___ftadd@sign)^080h),w
	andlw	07h
	btfss	status,2
	goto	u6581
	goto	u6580
u6581:
	goto	l12432
u6580:
	goto	l12440
	
l7575:	
	goto	l12440
	
l7576:	
	line	124
	goto	l12440
	
l7578:	
	line	125
	
l12438:	
	movlw	01h
u6595:
	clrc
	rrf	(___ftadd@f2+2)^080h,f
	rrf	(___ftadd@f2+1)^080h,f
	rrf	(___ftadd@f2)^080h,f
	addlw	-1
	skipz
	goto	u6595

	line	126
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??___ftadd+0)+0
	movf	(??___ftadd+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(___ftadd@exp2)^080h,f
	goto	l12440
	line	127
	
l7577:	
	line	124
	
l12440:	
	movf	(___ftadd@exp1)^080h,w
	xorwf	(___ftadd@exp2)^080h,w
	skipz
	goto	u6601
	goto	u6600
u6601:
	goto	l12438
u6600:
	goto	l7571
	
l7579:	
	goto	l7571
	line	128
	
l7572:	
	line	129
	
l7571:	
	btfss	(___ftadd@sign)^080h,(7)&7
	goto	u6611
	goto	u6610
u6611:
	goto	l12444
u6610:
	line	131
	
l12442:	
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
	goto	l12444
	line	133
	
l7580:	
	line	134
	
l12444:	
	btfss	(___ftadd@sign)^080h,(6)&7
	goto	u6621
	goto	u6620
u6621:
	goto	l12448
u6620:
	line	136
	
l12446:	
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
	goto	l12448
	line	138
	
l7581:	
	line	139
	
l12448:	
	clrf	(___ftadd@sign)^080h
	line	140
	movf	(___ftadd@f1)^080h,w
	addwf	(___ftadd@f2)^080h,f
	movf	(___ftadd@f1+1)^080h,w
	clrz
	skipnc
	incf	(___ftadd@f1+1)^080h,w
	skipnz
	goto	u6631
	addwf	(___ftadd@f2+1)^080h,f
u6631:
	movf	(___ftadd@f1+2)^080h,w
	clrz
	skipnc
	incf	(___ftadd@f1+2)^080h,w
	skipnz
	goto	u6632
	addwf	(___ftadd@f2+2)^080h,f
u6632:

	line	141
	
l12450:	
	btfss	(___ftadd@f2+2)^080h,(23)&7
	goto	u6641
	goto	u6640
u6641:
	goto	l12456
u6640:
	line	142
	
l12452:	
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
	
l12454:	
	clrf	(___ftadd@sign)^080h
	bsf	status,0
	rlf	(___ftadd@sign)^080h,f
	goto	l12456
	line	145
	
l7582:	
	line	146
	
l12456:	
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
	goto	l7557
	
l12458:	
	line	148
	
l7557:	
	return
	opt stack 0
GLOBAL	__end_of___ftadd
	__end_of___ftadd:
;; =============== function ___ftadd ends ============

	signat	___ftadd,8315
	global	_initIRobot
psect	text2020,local,class=CODE,delta=2
global __ptext2020
__ptext2020:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 130 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2020
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	130
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	131
	
l12384:	
;main.c: 131: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u7637:
	decfsz	((??_initIRobot+0)+0),f
	goto	u7637
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u7637
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u7637
	nop2
opt asmopt_on

	line	132
	
l12386:	
;main.c: 132: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	133
	
l12388:	
;main.c: 133: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	134
	
l6710:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_waitFor
psect	text2021,local,class=CODE,delta=2
global __ptext2021
__ptext2021:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 240 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2021
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	240
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 1
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	241
	
l12376:	
;drive.c: 241: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u7647:
	decfsz	((??_waitFor+0)+0),f
	goto	u7647
	decfsz	((??_waitFor+0)+0+1),f
	goto	u7647
	decfsz	((??_waitFor+0)+0+2),f
	goto	u7647
	nop2
opt asmopt_on

	line	242
	
l12378:	
;drive.c: 242: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	243
	
l12380:	
;drive.c: 243: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	244
	
l12382:	
;drive.c: 244: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	245
	
l5880:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_drive
psect	text2022,local,class=CODE,delta=2
global __ptext2022
__ptext2022:

;; *************** function _drive *****************
;; Defined at:
;;		line 21 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2022
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
	
l12364:	
;drive.c: 22: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u7657:
	decfsz	((??_drive+0)+0),f
	goto	u7657
	decfsz	((??_drive+0)+0+1),f
	goto	u7657
	decfsz	((??_drive+0)+0+2),f
	goto	u7657
	nop2
opt asmopt_on

	line	23
	
l12366:	
;drive.c: 23: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	24
	
l12368:	
;drive.c: 24: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	25
	
l12370:	
;drive.c: 25: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	26
	
l12372:	
;drive.c: 26: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	27
	
l12374:	
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
psect	text2023,local,class=CODE,delta=2
global __ptext2023
__ptext2023:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
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
psect	text2023
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
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
	
l12350:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l12352:	
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
	
l12354:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l12356:	
;ir.c: 48: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u7667:
	decfsz	((??_rotateIR+0)+0),f
	goto	u7667
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u7667
	nop2
opt asmopt_on

	line	44
	
l12358:	
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
	goto	u6401
	goto	u6400
u6401:
	goto	l5085
u6400:
	goto	l12360
	
l5086:	
	line	51
	
l12360:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l12362:	
;ir.c: 52: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u7677:
	decfsz	((??_rotateIR+0)+0),f
	goto	u7677
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u7677
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
psect	text2024,local,class=CODE,delta=2
global __ptext2024
__ptext2024:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
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
psect	text2024
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 2
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l12290:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6335
	movlw	low(052h)
	subwf	(convert@adc_value),w
u6335:

	skipnc
	goto	u6331
	goto	u6330
u6331:
	goto	l12298
u6330:
	line	13
	
l12292:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5065
	
l12294:	
	goto	l5065
	
l12296:	
	goto	l5065
	line	14
	
l5064:	
	
l12298:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6345
	movlw	low(085h)
	subwf	(convert@adc_value),w
u6345:

	skipnc
	goto	u6341
	goto	u6340
u6341:
	goto	l12306
u6340:
	line	15
	
l12300:	
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
	
l12302:	
	goto	l5065
	
l12304:	
	goto	l5065
	line	16
	
l5067:	
	
l12306:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6355
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u6355:

	skipnc
	goto	u6351
	goto	u6350
u6351:
	goto	l12314
u6350:
	line	17
	
l12308:	
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
	
l12310:	
	goto	l5065
	
l12312:	
	goto	l5065
	line	18
	
l5069:	
	
l12314:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6365
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u6365:

	skipnc
	goto	u6361
	goto	u6360
u6361:
	goto	l12322
u6360:
	line	19
	
l12316:	
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
	
l12318:	
	goto	l5065
	
l12320:	
	goto	l5065
	line	20
	
l5071:	
	
l12322:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u6375
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u6375:

	skipnc
	goto	u6371
	goto	u6370
u6371:
	goto	l12330
u6370:
	line	21
	
l12324:	
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
	
l12326:	
	goto	l5065
	
l12328:	
	goto	l5065
	line	22
	
l5073:	
	
l12330:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6385
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u6385:

	skipnc
	goto	u6381
	goto	u6380
u6381:
	goto	l12338
u6380:
	line	23
	
l12332:	
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
	
l12334:	
	goto	l5065
	
l12336:	
	goto	l5065
	line	24
	
l5075:	
	
l12338:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6395
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u6395:

	skipnc
	goto	u6391
	goto	u6390
u6391:
	goto	l12346
u6390:
	line	25
	
l12340:	
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
	
l12342:	
	goto	l5065
	
l12344:	
	goto	l5065
	line	26
	
l5077:	
	
l12346:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5065
	
l12348:	
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
psect	text2025,local,class=CODE,delta=2
global __ptext2025
__ptext2025:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 25 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
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
;;		_checkIfHome
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2025
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\songs.c"
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
	
l12288:	
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
psect	text2026,local,class=CODE,delta=2
global __ptext2026
__ptext2026:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
psect	text2026
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l12280:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l12286
	line	75
	
l3643:	
	line	76
	
l12282:	
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
	goto	u6310
	decf	(??_ser_putArr+0)+0,f
u6310:
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
	
l12284:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l12286
	
l3642:	
	
l12286:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u6325
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u6325:

	skipc
	goto	u6321
	goto	u6320
u6321:
	goto	l12282
u6320:
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
psect	text2027,local,class=CODE,delta=2
global __ptext2027
__ptext2027:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
psect	text2027
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 2
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l12264:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l12266
	
l3637:	
	line	62
;ser.c: 62: continue;
	goto	l12266
	
l3636:	
	line	61
	
l12266:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u6301
	goto	u6300
u6301:
	goto	l12266
u6300:
	
l3638:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l12268:	
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
	
l12270:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l12272:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l12274:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l12276:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l3639
	
l12278:	
	line	70
	
l3639:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_lcd_write_data
psect	text2028,local,class=CODE,delta=2
global __ptext2028
__ptext2028:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2028
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
	
l12256:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l12258:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l12260:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l12262:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u7687:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u7687
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u7687
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
psect	text2029,local,class=CODE,delta=2
global __ptext2029
__ptext2029:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
psect	text2029
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\lcd.c"
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
	
l12248:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l12250:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l12252:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l12254:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u7697:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u7697
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u7697
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
	global	_init_adc
psect	text2030,local,class=CODE,delta=2
global __ptext2030
__ptext2030:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
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
psect	text2030
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l12238:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l12240:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l12242:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l12244:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l12246:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u7707:
decfsz	(??_init_adc+0)+0,f
	goto	u7707
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
psect	text2031,local,class=CODE,delta=2
global __ptext2031
__ptext2031:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
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
psect	text2031
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 1
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l10892:	
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
	
l10894:	
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
	goto	u4471
	goto	u4470
u4471:
	goto	l703
u4470:
	goto	l10896
	
l705:	
	line	75
	
l10896:	
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
u4485:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4485
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l10898:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l10900:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text2032,local,class=CODE,delta=2
global __ptext2032
__ptext2032:

;; *************** function ___awdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\awdiv.c"
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
psect	text2032
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 2
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l10806:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u4311
	goto	u4310
u4311:
	goto	l10810
u4310:
	line	11
	
l10808:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l10810
	line	13
	
l7650:	
	line	14
	
l10810:	
	btfss	(___awdiv@dividend+1),7
	goto	u4321
	goto	u4320
u4321:
	goto	l10816
u4320:
	line	15
	
l10812:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l10814:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l10816
	line	17
	
l7651:	
	line	18
	
l10816:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l10818:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u4331
	goto	u4330
u4331:
	goto	l10838
u4330:
	line	20
	
l10820:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l10826
	
l7654:	
	line	22
	
l10822:	
	movlw	01h
	
u4345:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u4345
	line	23
	
l10824:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l10826
	line	24
	
l7653:	
	line	21
	
l10826:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u4351
	goto	u4350
u4351:
	goto	l10822
u4350:
	goto	l10828
	
l7655:	
	goto	l10828
	line	25
	
l7656:	
	line	26
	
l10828:	
	movlw	01h
	
u4365:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u4365
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u4375
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u4375:
	skipc
	goto	u4371
	goto	u4370
u4371:
	goto	l10834
u4370:
	line	28
	
l10830:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l10832:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l10834
	line	30
	
l7657:	
	line	31
	
l10834:	
	movlw	01h
	
u4385:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u4385
	line	32
	
l10836:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u4391
	goto	u4390
u4391:
	goto	l10828
u4390:
	goto	l10838
	
l7658:	
	goto	l10838
	line	33
	
l7652:	
	line	34
	
l10838:	
	movf	(___awdiv@sign),w
	skipz
	goto	u4400
	goto	l10842
u4400:
	line	35
	
l10840:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l10842
	
l7659:	
	line	36
	
l10842:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l7660
	
l10844:	
	line	37
	
l7660:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___fttol
psect	text2033,local,class=CODE,delta=2
global __ptext2033
__ptext2033:

;; *************** function ___fttol *****************
;; Defined at:
;;		line 45 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\fttol.c"
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
psect	text2033
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\fttol.c"
	line	45
	global	__size_of___fttol
	__size_of___fttol	equ	__end_of___fttol-___fttol
	
___fttol:	
	opt	stack 4
; Regs used in ___fttol: [wreg+status,2+status,0]
	line	49
	
l10746:	
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
	goto	u4191
	goto	u4190
u4191:
	goto	l10752
u4190:
	line	50
	
l10748:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l7620
	
l10750:	
	goto	l7620
	
l7619:	
	line	51
	
l10752:	
	movf	(___fttol@f1),w
	movwf	((??___fttol+0)+0)
	movf	(___fttol@f1+1),w
	movwf	((??___fttol+0)+0+1)
	movf	(___fttol@f1+2),w
	movwf	((??___fttol+0)+0+2)
	movlw	017h
u4205:
	clrc
	rrf	(??___fttol+0)+2,f
	rrf	(??___fttol+0)+1,f
	rrf	(??___fttol+0)+0,f
u4200:
	addlw	-1
	skipz
	goto	u4205
	movf	0+(??___fttol+0)+0,w
	movwf	(??___fttol+3)+0
	movf	(??___fttol+3)+0,w
	movwf	(___fttol@sign1)
	line	52
	
l10754:	
	bsf	(___fttol@f1)+(15/8),(15)&7
	line	53
	
l10756:	
	movlw	0FFh
	andwf	(___fttol@f1),f
	movlw	0FFh
	andwf	(___fttol@f1+1),f
	movlw	0
	andwf	(___fttol@f1+2),f
	line	54
	
l10758:	
	movf	(___fttol@f1),w
	movwf	(___fttol@lval)
	movf	(___fttol@f1+1),w
	movwf	((___fttol@lval))+1
	movf	(___fttol@f1+2),w
	movwf	((___fttol@lval))+2
	clrf	((___fttol@lval))+3
	line	55
	
l10760:	
	movlw	low(08Eh)
	subwf	(___fttol@exp1),f
	line	56
	
l10762:	
	btfss	(___fttol@exp1),7
	goto	u4211
	goto	u4210
u4211:
	goto	l10772
u4210:
	line	57
	
l10764:	
	movf	(___fttol@exp1),w
	xorlw	80h
	addlw	-((-15)^80h)
	skipnc
	goto	u4221
	goto	u4220
u4221:
	goto	l10770
u4220:
	line	58
	
l10766:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l7620
	
l10768:	
	goto	l7620
	
l7622:	
	goto	l10770
	line	59
	
l7623:	
	line	60
	
l10770:	
	movlw	01h
u4235:
	clrc
	rrf	(___fttol@lval+3),f
	rrf	(___fttol@lval+2),f
	rrf	(___fttol@lval+1),f
	rrf	(___fttol@lval),f
	addlw	-1
	skipz
	goto	u4235

	line	61
	movlw	(01h)
	movwf	(??___fttol+0)+0
	movf	(??___fttol+0)+0,w
	addwf	(___fttol@exp1),f
	btfss	status,2
	goto	u4241
	goto	u4240
u4241:
	goto	l10770
u4240:
	goto	l10782
	
l7624:	
	line	62
	goto	l10782
	
l7621:	
	line	63
	
l10772:	
	movlw	(018h)
	subwf	(___fttol@exp1),w
	skipc
	goto	u4251
	goto	u4250
u4251:
	goto	l10780
u4250:
	line	64
	
l10774:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l7620
	
l10776:	
	goto	l7620
	
l7626:	
	line	65
	goto	l10780
	
l7628:	
	line	66
	
l10778:	
	movlw	01h
	movwf	(??___fttol+0)+0
u4265:
	clrc
	rlf	(___fttol@lval),f
	rlf	(___fttol@lval+1),f
	rlf	(___fttol@lval+2),f
	rlf	(___fttol@lval+3),f
	decfsz	(??___fttol+0)+0
	goto	u4265
	line	67
	movlw	low(01h)
	subwf	(___fttol@exp1),f
	goto	l10780
	line	68
	
l7627:	
	line	65
	
l10780:	
	movf	(___fttol@exp1),f
	skipz
	goto	u4271
	goto	u4270
u4271:
	goto	l10778
u4270:
	goto	l10782
	
l7629:	
	goto	l10782
	line	69
	
l7625:	
	line	70
	
l10782:	
	movf	(___fttol@sign1),w
	skipz
	goto	u4280
	goto	l10786
u4280:
	line	71
	
l10784:	
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
	goto	l10786
	
l7630:	
	line	72
	
l10786:	
	movf	(___fttol@lval+3),w
	movwf	(?___fttol+3)
	movf	(___fttol@lval+2),w
	movwf	(?___fttol+2)
	movf	(___fttol@lval+1),w
	movwf	(?___fttol+1)
	movf	(___fttol@lval),w
	movwf	(?___fttol)

	goto	l7620
	
l10788:	
	line	73
	
l7620:	
	return
	opt stack 0
GLOBAL	__end_of___fttol
	__end_of___fttol:
;; =============== function ___fttol ends ============

	signat	___fttol,4220
	global	___ftpack
psect	text2034,local,class=CODE,delta=2
global __ptext2034
__ptext2034:

;; *************** function ___ftpack *****************
;; Defined at:
;;		line 63 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\float.c"
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
psect	text2034
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\float.c"
	line	63
	global	__size_of___ftpack
	__size_of___ftpack	equ	__end_of___ftpack-___ftpack
	
___ftpack:	
	opt	stack 3
; Regs used in ___ftpack: [wreg+status,2+status,0]
	line	64
	
l12102:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(___ftpack@exp),w
	skipz
	goto	u6030
	goto	l12106
u6030:
	
l12104:	
	movf	(___ftpack@arg+2),w
	iorwf	(___ftpack@arg+1),w
	iorwf	(___ftpack@arg),w
	skipz
	goto	u6041
	goto	u6040
u6041:
	goto	l12112
u6040:
	goto	l12106
	
l7844:	
	line	65
	
l12106:	
	movlw	0x0
	movwf	(?___ftpack)
	movlw	0x0
	movwf	(?___ftpack+1)
	movlw	0x0
	movwf	(?___ftpack+2)
	goto	l7845
	
l12108:	
	goto	l7845
	
l7842:	
	line	66
	goto	l12112
	
l7847:	
	line	67
	
l12110:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	68
	movlw	01h
u6055:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u6055

	goto	l12112
	line	69
	
l7846:	
	line	66
	
l12112:	
	movlw	low highword(0FE0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u6061
	goto	u6060
u6061:
	goto	l12110
u6060:
	goto	l7849
	
l7848:	
	line	70
	goto	l7849
	
l7850:	
	line	71
	
l12114:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	72
	
l12116:	
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
	
l12118:	
	movlw	01h
u6075:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u6075

	line	74
	
l7849:	
	line	70
	movlw	low highword(0FF0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u6081
	goto	u6080
u6081:
	goto	l12114
u6080:
	goto	l12122
	
l7851:	
	line	75
	goto	l12122
	
l7853:	
	line	76
	
l12120:	
	movlw	low(01h)
	subwf	(___ftpack@exp),f
	line	77
	movlw	01h
u6095:
	clrc
	rlf	(___ftpack@arg),f
	rlf	(___ftpack@arg+1),f
	rlf	(___ftpack@arg+2),f
	addlw	-1
	skipz
	goto	u6095
	goto	l12122
	line	78
	
l7852:	
	line	75
	
l12122:	
	btfss	(___ftpack@arg+1),(15)&7
	goto	u6101
	goto	u6100
u6101:
	goto	l12120
u6100:
	
l7854:	
	line	79
	btfsc	(___ftpack@exp),(0)&7
	goto	u6111
	goto	u6110
u6111:
	goto	l7855
u6110:
	line	80
	
l12124:	
	movlw	0FFh
	andwf	(___ftpack@arg),f
	movlw	07Fh
	andwf	(___ftpack@arg+1),f
	movlw	0FFh
	andwf	(___ftpack@arg+2),f
	
l7855:	
	line	81
	clrc
	rrf	(___ftpack@exp),f

	line	82
	
l12126:	
	movf	(___ftpack@exp),w
	movwf	((??___ftpack+0)+0)
	clrf	((??___ftpack+0)+0+1)
	clrf	((??___ftpack+0)+0+2)
	movlw	010h
u6125:
	clrc
	rlf	(??___ftpack+0)+0,f
	rlf	(??___ftpack+0)+1,f
	rlf	(??___ftpack+0)+2,f
u6120:
	addlw	-1
	skipz
	goto	u6125
	movf	0+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg),f
	movf	1+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+1),f
	movf	2+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+2),f
	line	83
	
l12128:	
	movf	(___ftpack@sign),w
	skipz
	goto	u6130
	goto	l7856
u6130:
	line	84
	
l12130:	
	bsf	(___ftpack@arg)+(23/8),(23)&7
	
l7856:	
	line	85
	line	86
	
l7845:	
	return
	opt stack 0
GLOBAL	__end_of___ftpack
	__end_of___ftpack:
;; =============== function ___ftpack ends ============

	signat	___ftpack,12411
	global	___wmul
psect	text2035,local,class=CODE,delta=2
global __ptext2035
__ptext2035:

;; *************** function ___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\wmul.c"
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
psect	text2035
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 2
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l12090:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l12092
	line	6
	
l7510:	
	line	7
	
l12092:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u5991
	goto	u5990
u5991:
	goto	l7511
u5990:
	line	8
	
l12094:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l7511:	
	line	9
	movlw	01h
	
u6005:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u6005
	line	10
	
l12096:	
	movlw	01h
	
u6015:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u6015
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u6021
	goto	u6020
u6021:
	goto	l12092
u6020:
	goto	l12098
	
l7512:	
	line	12
	
l12098:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l7513
	
l12100:	
	line	13
	
l7513:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_updateNode
psect	text2036,local,class=CODE,delta=2
global __ptext2036
__ptext2036:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 284 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2036
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	284
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 5
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	285
	
l12072:	
;main.c: 285: if((xCoord == 2) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5931
	goto	u5930
u5931:
	goto	l12078
u5930:
	
l12074:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5941
	goto	u5940
u5941:
	goto	l12078
u5940:
	line	286
	
l12076:	
;main.c: 286: node = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	bsf	status,0
	rlf	(_node)^080h,f	;volatile
	goto	l6766
	line	287
	
l6760:	
	
l12078:	
;main.c: 287: else if((xCoord == 4) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u5951
	goto	u5950
u5951:
	goto	l12084
u5950:
	
l12080:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5961
	goto	u5960
u5961:
	goto	l12084
u5960:
	line	288
	
l12082:	
;main.c: 288: node = 2;
	movlw	(02h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6766
	line	289
	
l6762:	
	
l12084:	
;main.c: 289: else if((xCoord == 2) && (yCoord == 0))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5971
	goto	u5970
u5971:
	goto	l6764
u5970:
	
l12086:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u5981
	goto	u5980
u5981:
	goto	l6764
u5980:
	line	290
	
l12088:	
;main.c: 290: node = 3;
	movlw	(03h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6766
	line	291
	
l6764:	
	line	292
;main.c: 291: else
;main.c: 292: node = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	goto	l6766
	
l6765:	
	goto	l6766
	
l6763:	
	goto	l6766
	
l6761:	
	line	293
	
l6766:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getSuccessfulDrive
psect	text2037,local,class=CODE,delta=2
global __ptext2037
__ptext2037:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 146 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
psect	text2037
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	146
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 5
; Regs used in _getSuccessfulDrive: [status]
	line	147
	
l12064:	
;drive.c: 147: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u5921
	goto	u5920
u5921:
	goto	l12068
u5920:
	
l12066:	
	clrc
	
	goto	l5846
	
l11816:	
	
l12068:	
	setc
	
	goto	l5846
	
l11818:	
	goto	l5846
	
l12070:	
	line	148
	
l5846:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_getSomethingInTheWay
psect	text2038,local,class=CODE,delta=2
global __ptext2038
__ptext2038:

;; *************** function _getSomethingInTheWay *****************
;; Defined at:
;;		line 140 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text2038
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	140
	global	__size_of_getSomethingInTheWay
	__size_of_getSomethingInTheWay	equ	__end_of_getSomethingInTheWay-_getSomethingInTheWay
	
_getSomethingInTheWay:	
	opt	stack 4
; Regs used in _getSomethingInTheWay: [wreg]
	line	141
	
l12060:	
;drive.c: 141: return somethingInTheWay;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_somethingInTheWay)^080h,w	;volatile
	goto	l5843
	
l12062:	
	line	142
	
l5843:	
	return
	opt stack 0
GLOBAL	__end_of_getSomethingInTheWay
	__end_of_getSomethingInTheWay:
;; =============== function _getSomethingInTheWay ends ============

	signat	_getSomethingInTheWay,89
	global	_getOrientation
psect	text2039,local,class=CODE,delta=2
global __ptext2039
__ptext2039:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 135 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_updateLocation
;;		_main
;; This function uses a non-reentrant model
;;
psect	text2039
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	135
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 4
; Regs used in _getOrientation: [wreg]
	line	136
	
l12056:	
;drive.c: 136: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l5840
	
l12058:	
	line	137
	
l5840:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_getCurrentY
psect	text2040,local,class=CODE,delta=2
global __ptext2040
__ptext2040:

;; *************** function _getCurrentY *****************
;; Defined at:
;;		line 431 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
;; This function uses a non-reentrant model
;;
psect	text2040
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	431
	global	__size_of_getCurrentY
	__size_of_getCurrentY	equ	__end_of_getCurrentY-_getCurrentY
	
_getCurrentY:	
	opt	stack 3
; Regs used in _getCurrentY: [wreg]
	line	432
	
l12052:	
;main.c: 432: return yCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	goto	l6827
	
l12054:	
	line	433
	
l6827:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentY
	__end_of_getCurrentY:
;; =============== function _getCurrentY ends ============

	signat	_getCurrentY,89
	global	_getCurrentX
psect	text2041,local,class=CODE,delta=2
global __ptext2041
__ptext2041:

;; *************** function _getCurrentX *****************
;; Defined at:
;;		line 426 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
;; This function uses a non-reentrant model
;;
psect	text2041
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
	line	426
	global	__size_of_getCurrentX
	__size_of_getCurrentX	equ	__end_of_getCurrentX-_getCurrentX
	
_getCurrentX:	
	opt	stack 3
; Regs used in _getCurrentX: [wreg]
	line	427
	
l12048:	
;main.c: 427: return xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	goto	l6824
	
l12050:	
	line	428
	
l6824:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentX
	__end_of_getCurrentX:
;; =============== function _getCurrentX ends ============

	signat	_getCurrentX,89
	global	_updateOrientation
psect	text2042,local,class=CODE,delta=2
global __ptext2042
__ptext2042:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 233 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
;; Parameters:    Size  Location     Type
;;  moved           1    wreg     enum E1111
;; Auto vars:     Size  Location     Type
;;  moved           1   11[BANK0 ] enum E1111
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
psect	text2042
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\drive.c"
	line	233
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 3
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	234
	
l12042:	
;drive.c: 234: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	235
	
l12044:	
;drive.c: 235: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u5911
	goto	u5910
u5911:
	goto	l5877
u5910:
	line	236
	
l12046:	
;drive.c: 236: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l5877
	
l5876:	
	line	237
	
l5877:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_ser_init
psect	text2043,local,class=CODE,delta=2
global __ptext2043
__ptext2043:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
psect	text2043
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l12016:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l12018:	
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
	
l12020:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l12022:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l12024:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l12026:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l12028:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l12030:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l12032:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l12034:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l12036:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l12038:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l12040:	
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
psect	text2044,local,class=CODE,delta=2
global __ptext2044
__ptext2044:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
psect	text2044
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 2
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l11968:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u5841
	goto	u5840
u5841:
	goto	l11976
u5840:
	line	50
	
l11970:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l11972:	
;ser.c: 52: return 0;
	clrc
	
	goto	l3633
	
l11974:	
	goto	l3633
	line	53
	
l3632:	
	line	54
	
l11976:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u5851
	goto	u5850
u5851:
	goto	l11980
u5850:
	
l11978:	
	clrc
	
	goto	l3633
	
l11812:	
	
l11980:	
	setc
	
	goto	l3633
	
l11814:	
	goto	l3633
	
l11982:	
	line	55
	
l3633:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_getVictimZone
psect	text2045,local,class=CODE,delta=2
global __ptext2045
__ptext2045:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
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
psect	text2045
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
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
	
l11920:	
;map.c: 163: switch (victimX)
	goto	l11962
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l2900:	
	line	166
;map.c: 166: switch (victimY)
	goto	l11928
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l2902:	
	line	169
	
l11922:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	170
;map.c: 170: break;
	goto	l11964
	line	171
;map.c: 171: case 1:
	
l2904:	
	line	172
	
l11924:	
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
	goto	l11964
	line	178
;map.c: 178: default:
	
l2905:	
	line	179
;map.c: 179: break;
	goto	l11964
	line	180
	
l11926:	
;map.c: 180: }
	goto	l11964
	line	166
	
l2901:	
	
l11928:	
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
	goto	l11922
	xorlw	1^0	; case 1
	skipnz
	goto	l11924
	goto	l11964
	opt asmopt_on

	line	180
	
l2903:	
	line	181
;map.c: 181: break;
	goto	l11964
	line	183
;map.c: 183: case 1:
	
l2907:	
	line	184
;map.c: 184: switch (victimY)
	goto	l11936
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l2909:	
	line	187
	
l11930:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	188
;map.c: 188: break;
	goto	l11964
	line	189
;map.c: 189: case 1:
	
l2911:	
	line	190
	
l11932:	
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
	goto	l11964
	line	196
;map.c: 196: default:
	
l2912:	
	line	197
;map.c: 197: break;
	goto	l11964
	line	198
	
l11934:	
;map.c: 198: }
	goto	l11964
	line	184
	
l2908:	
	
l11936:	
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
	goto	l11930
	xorlw	1^0	; case 1
	skipnz
	goto	l11932
	goto	l11964
	opt asmopt_on

	line	198
	
l2910:	
	line	199
;map.c: 199: break;
	goto	l11964
	line	201
;map.c: 201: case 2:
	
l2913:	
	line	202
;map.c: 202: switch (victimY)
	goto	l11944
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l2915:	
	line	207
	
l11938:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	208
;map.c: 208: break;
	goto	l11964
	line	211
;map.c: 211: case 3:
	
l2917:	
	line	212
	
l11940:	
;map.c: 212: vicZone = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_vicZone)^080h
	bsf	status,0
	rlf	(_vicZone)^080h,f
	line	213
;map.c: 213: break;
	goto	l11964
	line	214
;map.c: 214: default:
	
l2918:	
	line	215
;map.c: 215: break;
	goto	l11964
	line	216
	
l11942:	
;map.c: 216: }
	goto	l11964
	line	202
	
l2914:	
	
l11944:	
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
	goto	l11938
	xorlw	3^1	; case 3
	skipnz
	goto	l11940
	goto	l11964
	opt asmopt_on

	line	216
	
l2916:	
	line	217
;map.c: 217: break;
	goto	l11964
	line	219
;map.c: 219: case 3:
	
l2919:	
	line	220
;map.c: 220: switch (victimY)
	goto	l11952
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l2921:	
	line	225
	
l11946:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	226
;map.c: 226: break;
	goto	l11964
	line	229
;map.c: 229: case 3:
	
l2923:	
	line	230
	
l11948:	
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
	goto	l11964
	line	232
;map.c: 232: default:
	
l2924:	
	line	233
;map.c: 233: break;
	goto	l11964
	line	234
	
l11950:	
;map.c: 234: }
	goto	l11964
	line	220
	
l2920:	
	
l11952:	
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
	goto	l11946
	xorlw	3^1	; case 3
	skipnz
	goto	l11948
	goto	l11964
	opt asmopt_on

	line	234
	
l2922:	
	line	235
;map.c: 235: break;
	goto	l11964
	line	237
;map.c: 237: case 4:
	
l2925:	
	line	238
;map.c: 238: switch (victimY)
	goto	l11958
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l2927:	
	line	247
	
l11954:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_vicZone)^080h
	line	248
;map.c: 248: break;
	goto	l11964
	line	249
;map.c: 249: default:
	
l2929:	
	line	250
;map.c: 250: break;
	goto	l11964
	line	251
	
l11956:	
;map.c: 251: }
	goto	l11964
	line	238
	
l2926:	
	
l11958:	
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
	goto	l11954
	goto	l11964
	opt asmopt_on

	line	251
	
l2928:	
	line	252
;map.c: 252: break;
	goto	l11964
	line	254
;map.c: 254: default:
	
l2930:	
	line	255
;map.c: 255: break;
	goto	l11964
	line	256
	
l11960:	
;map.c: 256: }
	goto	l11964
	line	163
	
l2899:	
	
l11962:	
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
	goto	l11928
	xorlw	1^0	; case 1
	skipnz
	goto	l11936
	xorlw	2^1	; case 2
	skipnz
	goto	l11944
	xorlw	3^2	; case 3
	skipnz
	goto	l11952
	xorlw	4^3	; case 4
	skipnz
	goto	l11958
	goto	l11964
	opt asmopt_on

	line	256
	
l2906:	
	line	258
	
l11964:	
;map.c: 258: return vicZone;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_vicZone)^080h,w
	goto	l2931
	
l11966:	
	line	259
	
l2931:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text2046,local,class=CODE,delta=2
global __ptext2046
__ptext2046:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
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
psect	text2046
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 4
; Regs used in _getFinalY: [wreg]
	line	153
	
l11916:	
;map.c: 153: return finalY;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_finalY)^080h,w
	goto	l2896
	
l11918:	
	line	154
	
l2896:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text2047,local,class=CODE,delta=2
global __ptext2047
__ptext2047:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
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
psect	text2047
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 4
; Regs used in _getFinalX: [wreg]
	line	148
	
l11912:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l2893
	
l11914:	
	line	149
	
l2893:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_putch
psect	text2048,local,class=CODE,delta=2
global __ptext2048
__ptext2048:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
;;		_ser_putArr
;;		_play_iCreate_song
;;		_drive
;;		_driveForDistance
;;		_waitFor
;;		_initIRobot
;;		_lookForVictim
;;		_EEPROMToSerial
;;		_ser_puts
;;		_ser_puts2
;;		_ser_puthex
;; This function uses a non-reentrant model
;;
psect	text2048
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\ser.c"
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
	
l11884:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l11886
	
l3649:	
	line	83
;ser.c: 83: continue;
	goto	l11886
	
l3648:	
	line	82
	
l11886:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u5811
	goto	u5810
u5811:
	goto	l11886
u5810:
	
l3650:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l11888:	
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
	
l11890:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l11892:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l11894:	
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
	global	_isr1
psect	text2049,local,class=CODE,delta=2
global __ptext2049
__ptext2049:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2049
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.7\main.c"
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
psect	text2049
	line	64
	
i1l10568:	
;main.c: 64: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u374_21
	goto	u374_20
u374_21:
	goto	i1l6704
u374_20:
	line	66
	
i1l10570:	
;main.c: 65: {
;main.c: 66: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	67
	
i1l10572:	
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
	
i1l10574:	
;main.c: 71: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	73
	
i1l10576:	
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
	goto	u375_21
	goto	u375_20
u375_21:
	goto	i1l10580
u375_20:
	
i1l10578:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l10580
	
i1l6694:	
	line	74
	
i1l10580:	
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
	goto	u376_21
	goto	u376_20
u376_21:
	goto	i1l10584
u376_20:
	
i1l10582:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l10584
	
i1l6695:	
	line	75
	
i1l10584:	
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
	
i1l6696:	
	line	80
;main.c: 78: }
;main.c: 80: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u377_21
	goto	u377_20
u377_21:
	goto	i1l6697
u377_20:
	line	82
	
i1l10586:	
;main.c: 81: {
;main.c: 82: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	0+(_start)^080h+02h,f
	line	83
	
i1l10588:	
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
	goto	u378_21
	goto	u378_20
u378_21:
	goto	i1l10596
u378_20:
	line	85
	
i1l10590:	
;main.c: 84: {
;main.c: 85: start.pressed = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	bsf	status,0
	rlf	(_start)^080h,f
	line	86
	
i1l10592:	
;main.c: 86: start.released = 0;
	clrf	0+(_start)^080h+01h
	goto	i1l10596
	line	87
	
i1l6698:	
	line	88
;main.c: 87: }
;main.c: 88: }
	goto	i1l10596
	line	89
	
i1l6697:	
	line	91
;main.c: 89: else
;main.c: 90: {
;main.c: 91: start.debounceCount = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	0+(_start)^080h+02h
	line	92
	
i1l10594:	
;main.c: 92: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	goto	i1l10596
	line	93
	
i1l6699:	
	line	95
	
i1l10596:	
;main.c: 93: }
;main.c: 95: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	bcf	status, 5	;RP0=0, select bank0
	btfss	(101/8),(101)&7
	goto	u379_21
	goto	u379_20
u379_21:
	goto	i1l10606
u379_20:
	
i1l10598:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	
i1l10600:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_ser_tmp)^080h
	
i1l10602:	
	movf	(_ser_tmp)^080h,w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u380_21
	goto	u380_20
u380_21:
	goto	i1l10606
u380_20:
	
i1l10604:	
	movf	(_ser_tmp)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l10606
	
i1l6701:	
	goto	i1l10606
	
i1l6700:	
	
i1l10606:	
	bcf	status, 5	;RP0=0, select bank0
	btfss	(100/8),(100)&7
	goto	u381_21
	goto	u381_20
u381_21:
	goto	i1l6704
u381_20:
	
i1l10608:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u382_21
	goto	u382_20
u382_21:
	goto	i1l6704
u382_20:
	
i1l10610:	
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(25)	;volatile
	
i1l10612:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l10614:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l10616:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u383_21
	goto	u383_20
u383_21:
	goto	i1l6704
u383_20:
	
i1l10618:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l6704
	
i1l6703:	
	goto	i1l6704
	
i1l6702:	
	goto	i1l6704
	line	96
	
i1l6693:	
	line	97
	
i1l6704:	
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
psect	text2050,local,class=CODE,delta=2
global __ptext2050
__ptext2050:

;; *************** function ___lwmod *****************
;; Defined at:
;;		line 5 in file "C:\Program Files\HI-TECH Software\PICC\9.82\sources\lwmod.c"
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
psect	text2050
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l10650:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u394_21
	goto	u394_20
u394_21:
	goto	i1l10668
u394_20:
	line	9
	
i1l10652:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l10658
	
i1l7528:	
	line	11
	
i1l10654:	
	movlw	01h
	
u395_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u395_25
	line	12
	
i1l10656:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l10658
	line	13
	
i1l7527:	
	line	10
	
i1l10658:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u396_21
	goto	u396_20
u396_21:
	goto	i1l10654
u396_20:
	goto	i1l10660
	
i1l7529:	
	goto	i1l10660
	line	14
	
i1l7530:	
	line	15
	
i1l10660:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u397_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u397_25:
	skipc
	goto	u397_21
	goto	u397_20
u397_21:
	goto	i1l10664
u397_20:
	line	16
	
i1l10662:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l10664
	
i1l7531:	
	line	17
	
i1l10664:	
	movlw	01h
	
u398_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u398_25
	line	18
	
i1l10666:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u399_21
	goto	u399_20
u399_21:
	goto	i1l10660
u399_20:
	goto	i1l10668
	
i1l7532:	
	goto	i1l10668
	line	19
	
i1l7526:	
	line	20
	
i1l10668:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l7533
	
i1l10670:	
	line	21
	
i1l7533:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text2051,local,class=CODE,delta=2
global __ptext2051
__ptext2051:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

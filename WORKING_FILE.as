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
# 21 "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_findWalls
	FNCALL	_main,_play_iCreate_song
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
	FNCALL	_driveForDistance,_getCurrentY
	FNCALL	_driveForDistance,_getCurrentX
	FNCALL	_driveForDistance,_findFinalDestination
	FNCALL	_driveForDistance,_turnRight90
	FNCALL	_driveForDistance,_updateOrientation
	FNCALL	_driveForDistance,_turnLeft90
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
	global	_yCoord
	global	_beep
	global	_longbeep
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
	line	7

;initializer for _finalX
	retlw	03h
	line	8

;initializer for _finalY
	retlw	01h
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	15

;initializer for _somethingInTheWay
	retlw	02h
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	51

;initializer for _xCoord
	retlw	01h
	line	52

;initializer for _yCoord
	retlw	03h
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
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
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
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
psect	idataBANK1,class=CODE,space=0,delta=2
global __pidataBANK1
__pidataBANK1:
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
	global	_start
	global	_RTC_Counter
	global	_closestObject
	global	_addressCount
	global	_addressCurrent
	global	_currentOrientation
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
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
	line	7
_finalX:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
	line	8
_finalY:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	15
_somethingInTheWay:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	51
_xCoord:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	52
_yCoord:
       ds      1

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	14
_beep:
       ds      5

psect	dataBANK0
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	15
_longbeep:
       ds      5

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
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK1
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	12
_finalCountdown:
       ds      27

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
	movlw	low((__pbssBANK0)+014h)
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
	movlw low(__pdataBANK3+56)
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
	movlw low(__pdataBANK1+46)
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
	movlw low(__pdataBANK0+15)
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
	global	??_getCurrentX
??_getCurrentX:	; 0 bytes @ 0x0
	global	??_getCurrentY
??_getCurrentY:	; 0 bytes @ 0x0
	global	??_updateOrientation
??_updateOrientation:	; 0 bytes @ 0x0
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
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0x0
	ds	1
	global	?_writeEEPROM
?_writeEEPROM:	; 0 bytes @ 0x1
	global	??_getVictimZone
??_getVictimZone:	; 0 bytes @ 0x1
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0x1
	global	writeEEPROM@addressH
writeEEPROM@addressH:	; 1 bytes @ 0x1
	global	ser_getch@c
ser_getch@c:	; 1 bytes @ 0x1
	global	ser_putch@c
ser_putch@c:	; 1 bytes @ 0x1
	global	updateOrientation@moved
updateOrientation@moved:	; 1 bytes @ 0x1
	ds	1
	global	?_waitFor
?_waitFor:	; 0 bytes @ 0x2
	global	??_initIRobot
??_initIRobot:	; 0 bytes @ 0x2
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
	global	lookForVictim@victim
lookForVictim@victim:	; 1 bytes @ 0x6
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x6
	ds	1
	global	??_addNewData
??_addNewData:	; 0 bytes @ 0x7
	global	findFinalDestination@virtualWallX
findFinalDestination@virtualWallX:	; 1 bytes @ 0x7
	global	waitFor@type
waitFor@type:	; 1 bytes @ 0x7
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
	ds	1
;;Data sizes: Strings 63, constant 0, data 117, bss 54, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14     10      14
;; BANK0           80     41      76
;; BANK1           80      0      78
;; BANK3           96      0      56
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
;;		 -> longbeep(BANK0[5]), beep(BANK0[5]), champions(BANK1[21]), lookingForU2(BANK3[29]), 
;;		 -> superMarioBros(BANK1[25]), finalCountdown(BANK3[27]), 
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
;;   _checkIfHome->_drive
;;   _turnAround->_drive
;;   _turnLeft90->_drive
;;   _turnRight90->_drive
;;   _initSongs->_ser_putArr
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_set_cursor->_lcd_write_control
;;   _addNewData->_writeEEPROM
;;   _lcd_write_string->_lcd_write_data
;;   _adc_read_channel->_convert
;;   _initIRobot->_ser_putch
;;   _waitFor->_ser_putch
;;   _drive->_ser_putch
;;   _convert->_adc_read
;;   _play_iCreate_song->_ser_putch
;;   _ser_putArr->_ser_putch
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
;;Main: autosize = 0, tempsize = 1, incstack = 0, save=0
;;

;;
;;Call Graph Tables:
;;
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (0) _main                                                 1     1      0   16191
;;                                             40 BANK0      1     1      0
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                          _findWalls
;;                  _play_iCreate_song
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
;; (1) _goToNextCell                                         0     0      0    6256
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
;; (1) _goRight                                              1     1      0    1619
;;                                             24 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goLeft                                               0     0      0    1619
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (1) _goForward                                            0     0      0    1399
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _getCurrentX
;;                        _getCurrentY
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0    1619
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
;; (2) _driveForDistance                                    12    10      2    1332
;;                                             12 BANK0     12    10      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;;                          _goReverse
;;               _clearSuccessfulDrive
;;                        _getCurrentY
;;                        _getCurrentX
;;               _findFinalDestination
;;                        _turnRight90
;;                  _updateOrientation
;;                         _turnLeft90
;; ---------------------------------------------------------------------------------
;; (1) _updateLocation                                       1     1      0     111
;;                                              4 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                     _getOrientation
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _lookForVictim                                        3     3      0     377
;;                                              4 BANK0      3     3      0
;;                          _ser_putch
;;                          _ser_getch
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                      _getVictimZone
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _checkForFinalDestination                             0     0      0     111
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
;; (3) _updateOrientation                                    2     2      0      22
;;                                              0 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (4) _getCurrentY                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (4) _getCurrentX                                          0     0      0       0
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
;;   _play_iCreate_song
;;     _ser_putch
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
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
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
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
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
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
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
;;         _getCurrentY
;;         _getCurrentX
;;         _findFinalDestination
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_1_digit_bcd
;;             _lcd_write_data
;;           _getCurrentY (ARG)
;;           _getCurrentX (ARG)
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
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
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
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
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
;;       _getCurrentY
;;       _getCurrentX
;;       _findFinalDestination
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_1_digit_bcd
;;           _lcd_write_data
;;         _getCurrentY (ARG)
;;         _getCurrentX (ARG)
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
;;BANK3               60      0      38       9       58.3%
;;BITBANK3            60      0       0       8        0.0%
;;SFR3                 0      0       0       4        0.0%
;;BITSFR3              0      0       0       4        0.0%
;;BANK2               60      0       0      11        0.0%
;;BITBANK2            60      0       0      10        0.0%
;;SFR2                 0      0       0       5        0.0%
;;BITSFR2              0      0       0       5        0.0%
;;SFR1                 0      0       0       2        0.0%
;;BITSFR1              0      0       0       2        0.0%
;;BANK1               50      0      4E       7       97.5%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      E6      12        0.0%
;;ABS                  0      0      E0       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       6       2        0.0%
;;BANK0               50     29      4C       5       95.0%
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
;;		line 317 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       0       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_init
;;		_drive
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_play_iCreate_song
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
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	317
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 1
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	318
	
l11227:	
;main.c: 318: init();
	fcall	_init
	line	319
;main.c: 319: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	321
	
l11229:	
;main.c: 321: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	322
	
l11231:	
;main.c: 322: lcd_write_string("(-,-) - -- --- -");
	movlw	((STR_3-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	323
;main.c: 323: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	324
	
l11233:	
;main.c: 324: lcd_write_string("- - - (3,1) GREG");
	movlw	((STR_4-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	326
;main.c: 326: while(!home)
	goto	l11341
	
l6773:	
	line	344
;main.c: 327: {
;main.c: 344: ready = 1;
	bsf	(_ready/8),(_ready)&7
	line	346
	
l11235:	
;main.c: 346: if(start.pressed && ready == 1)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u4620
	goto	l11341
u4620:
	
l11237:	
	btfss	(_ready/8),(_ready)&7
	goto	u4631
	goto	u4630
u4631:
	goto	l11341
u4630:
	line	348
	
l11239:	
;main.c: 347: {
;main.c: 348: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	350
;main.c: 350: lookForVictim();
	fcall	_lookForVictim
	line	352
	
l11241:	
;main.c: 352: findWalls();
	fcall	_findWalls
	line	353
	
l11243:	
;main.c: 353: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	354
	
l11245:	
;main.c: 354: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4641
	goto	u4640
u4641:
	goto	l11253
u4640:
	line	356
	
l11247:	
;main.c: 355: {
;main.c: 356: rotateIR(24,0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	357
	
l11249:	
;main.c: 357: wallFollow();
	fcall	_wallFollow
	line	358
	
l11251:	
;main.c: 358: rotateIR(24,0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	goto	l11253
	line	359
	
l6775:	
	line	360
	
l11253:	
;main.c: 359: }
;main.c: 360: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	361
	
l11255:	
;main.c: 361: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4651
	goto	u4650
u4651:
	goto	l11259
u4650:
	line	362
	
l11257:	
;main.c: 362: frontWallCorrect();
	fcall	_frontWallCorrect
	goto	l11259
	
l6776:	
	line	363
	
l11259:	
;main.c: 363: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	364
;main.c: 364: switch(node)
	goto	l11325
	line	366
;main.c: 365: {
;main.c: 366: case 0:
	
l6778:	
	line	367
	
l11261:	
;main.c: 367: goToNextCell();
	fcall	_goToNextCell
	line	368
;main.c: 368: break;
	goto	l11327
	line	369
;main.c: 369: case 1:
	
l6780:	
	line	370
;main.c: 370: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4661
	goto	u4660
u4661:
	goto	l11277
u4660:
	line	372
	
l11263:	
;main.c: 371: {
;main.c: 372: if (victimZone == 1)
	movf	(_victimZone),w	;volatile
	xorlw	01h
	skipz
	goto	u4671
	goto	u4670
u4671:
	goto	l11267
u4670:
	line	373
	
l11265:	
;main.c: 373: goRight();
	fcall	_goRight
	goto	l11327
	line	374
	
l6782:	
	
l11267:	
;main.c: 374: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u4681
	goto	u4680
u4681:
	goto	l11271
u4680:
	line	375
	
l11269:	
;main.c: 375: goForward();
	fcall	_goForward
	goto	l11327
	line	376
	
l6784:	
	
l11271:	
;main.c: 376: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u4691
	goto	u4690
u4691:
	goto	l11275
u4690:
	line	377
	
l11273:	
;main.c: 377: goRight();
	fcall	_goRight
	goto	l11327
	line	378
	
l6786:	
	line	379
	
l11275:	
;main.c: 378: else
;main.c: 379: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6787:	
	goto	l11327
	
l6785:	
	goto	l11327
	
l6783:	
	line	380
;main.c: 380: }
	goto	l11327
	line	381
	
l6781:	
	line	382
	
l11277:	
;main.c: 381: else
;main.c: 382: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6788:	
	line	383
;main.c: 383: break;
	goto	l11327
	line	384
;main.c: 384: case 2:
	
l6789:	
	line	385
;main.c: 385: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4701
	goto	u4700
u4701:
	goto	l11293
u4700:
	line	387
	
l11279:	
;main.c: 386: {
;main.c: 387: if (victimZone == 2)
	movf	(_victimZone),w	;volatile
	xorlw	02h
	skipz
	goto	u4711
	goto	u4710
u4711:
	goto	l11283
u4710:
	line	388
	
l11281:	
;main.c: 388: goForward();
	fcall	_goForward
	goto	l11327
	line	389
	
l6791:	
	
l11283:	
;main.c: 389: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u4721
	goto	u4720
u4721:
	goto	l11287
u4720:
	line	390
	
l11285:	
;main.c: 390: goRight();
	fcall	_goRight
	goto	l11327
	line	391
	
l6793:	
	
l11287:	
;main.c: 391: else if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u4731
	goto	u4730
u4731:
	goto	l11291
u4730:
	line	392
	
l11289:	
;main.c: 392: goLeft();
	fcall	_goLeft
	goto	l11327
	line	393
	
l6795:	
	line	394
	
l11291:	
;main.c: 393: else
;main.c: 394: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6796:	
	goto	l11327
	
l6794:	
	goto	l11327
	
l6792:	
	line	395
;main.c: 395: }
	goto	l11327
	line	396
	
l6790:	
	line	397
	
l11293:	
;main.c: 396: else
;main.c: 397: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6797:	
	line	398
;main.c: 398: break;
	goto	l11327
	line	399
;main.c: 399: case 3:
	
l6798:	
	line	400
;main.c: 400: if (goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4741
	goto	u4740
u4741:
	goto	l11309
u4740:
	line	402
	
l11295:	
;main.c: 401: {
;main.c: 402: if (victimZone == 3)
	movf	(_victimZone),w	;volatile
	xorlw	03h
	skipz
	goto	u4751
	goto	u4750
u4751:
	goto	l11299
u4750:
	line	403
	
l11297:	
;main.c: 403: goRight();
	fcall	_goRight
	goto	l11327
	line	404
	
l6800:	
	
l11299:	
;main.c: 404: else if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u4761
	goto	u4760
u4761:
	goto	l11303
u4760:
	line	405
	
l11301:	
;main.c: 405: goForward();
	fcall	_goForward
	goto	l11327
	line	406
	
l6802:	
	
l11303:	
;main.c: 406: else if (getOrientation() == SOUTH)
	fcall	_getOrientation
	xorlw	01h
	skipz
	goto	u4771
	goto	u4770
u4771:
	goto	l11307
u4770:
	line	407
	
l11305:	
;main.c: 407: goLeft();
	fcall	_goLeft
	goto	l11327
	line	408
	
l6804:	
	line	409
	
l11307:	
;main.c: 408: else
;main.c: 409: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6805:	
	goto	l11327
	
l6803:	
	goto	l11327
	
l6801:	
	line	410
;main.c: 410: }
	goto	l11327
	line	411
	
l6799:	
	line	412
	
l11309:	
;main.c: 411: else
;main.c: 412: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6806:	
	line	413
;main.c: 413: break;
	goto	l11327
	line	414
;main.c: 414: case 4:
	
l6807:	
	line	415
	
l11311:	
;main.c: 415: if (getOrientation() == EAST)
	fcall	_getOrientation
	xorlw	02h
	skipz
	goto	u4781
	goto	u4780
u4781:
	goto	l11315
u4780:
	line	416
	
l11313:	
;main.c: 416: goRight();
	fcall	_goRight
	goto	l11327
	line	417
	
l6808:	
	line	418
	
l11315:	
;main.c: 417: else
;main.c: 418: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6809:	
	line	419
;main.c: 419: break;
	goto	l11327
	line	420
;main.c: 420: case 5:
	
l6810:	
	line	421
	
l11317:	
;main.c: 421: if (getOrientation() == NORTH)
	fcall	_getOrientation
	xorlw	03h
	skipz
	goto	u4791
	goto	u4790
u4791:
	goto	l11321
u4790:
	line	422
	
l11319:	
;main.c: 422: goRight();
	fcall	_goRight
	goto	l11327
	line	423
	
l6811:	
	line	424
	
l11321:	
;main.c: 423: else
;main.c: 424: goToNextCell();
	fcall	_goToNextCell
	goto	l11327
	
l6812:	
	line	425
;main.c: 425: break;
	goto	l11327
	line	426
;main.c: 426: default:
	
l6813:	
	line	427
;main.c: 427: break;
	goto	l11327
	line	428
	
l11323:	
;main.c: 428: }
	goto	l11327
	line	364
	
l6777:	
	
l11325:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_node),w	;volatile
	; Switch size 1, requested type "space"
; Number of cases is 6, Range of values is 0 to 5
; switch strategies available:
; Name         Instructions Cycles
; simple_byte           19    10 (average)
; direct_byte           26     8 (fixed)
; jumptable            260     6 (fixed)
; rangetable            10     6 (fixed)
; spacedrange           18     9 (fixed)
; locatedrange           6     3 (fixed)
;	Chosen strategy is simple_byte

	opt asmopt_off
	xorlw	0^0	; case 0
	skipnz
	goto	l11261
	xorlw	1^0	; case 1
	skipnz
	goto	l6780
	xorlw	2^1	; case 2
	skipnz
	goto	l6789
	xorlw	3^2	; case 3
	skipnz
	goto	l6798
	xorlw	4^3	; case 4
	skipnz
	goto	l11311
	xorlw	5^4	; case 5
	skipnz
	goto	l11317
	goto	l11327
	opt asmopt_on

	line	428
	
l6779:	
	line	429
	
l11327:	
;main.c: 429: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	430
	
l11329:	
;main.c: 430: if(getSuccessfulDrive())
	fcall	_getSuccessfulDrive
	btfss	status,0
	goto	u4801
	goto	u4800
u4801:
	goto	l11341
u4800:
	line	433
	
l11331:	
;main.c: 431: {
;main.c: 433: updateMapData(0,0,0,0,0,getOrientation());
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_updateMapData)
	clrf	0+(?_updateMapData)+01h
	clrf	0+(?_updateMapData)+02h
	clrf	0+(?_updateMapData)+03h
	fcall	_getOrientation
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	0+(?_updateMapData)+04h
	movlw	(0)
	fcall	_updateMapData
	line	434
	
l11333:	
;main.c: 434: updateLocation();
	fcall	_updateLocation
	line	435
	
l11335:	
;main.c: 435: updateNode();
	fcall	_updateNode
	line	436
	
l11337:	
;main.c: 436: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4811
	goto	u4810
u4811:
	goto	l11341
u4810:
	line	437
	
l11339:	
;main.c: 437: checkIfHome();
	fcall	_checkIfHome
	goto	l11341
	
l6815:	
	goto	l11341
	line	439
	
l6814:	
	goto	l11341
	line	440
	
l6774:	
	goto	l11341
	line	441
	
l6772:	
	line	326
	
l11341:	
	btfss	(_home/8),(_home)&7
	goto	u4821
	goto	u4820
u4821:
	goto	l6773
u4820:
	goto	l6817
	
l6816:	
	line	443
	
l6817:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text1237,local,class=CODE,delta=2
global __ptext1237
__ptext1237:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 246 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1237
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	246
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 1
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	247
	
l11211:	
;main.c: 247: if(!leftWall && (getSomethingInTheWay() != LEFT))
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u4561
	goto	u4560
u4561:
	goto	l6736
u4560:
	
l11213:	
	fcall	_getSomethingInTheWay
	xorlw	01h
	skipnz
	goto	u4571
	goto	u4570
u4571:
	goto	l6736
u4570:
	line	248
	
l11215:	
;main.c: 248: goLeft();
	fcall	_goLeft
	goto	l6742
	line	249
	
l6736:	
;main.c: 249: else if(!frontWall && (getSomethingInTheWay() != FORWARD))
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u4581
	goto	u4580
u4581:
	goto	l6738
u4580:
	
l11217:	
	fcall	_getSomethingInTheWay
	xorlw	0
	skipnz
	goto	u4591
	goto	u4590
u4591:
	goto	l6738
u4590:
	line	250
	
l11219:	
;main.c: 250: goForward();
	fcall	_goForward
	goto	l6742
	line	251
	
l6738:	
;main.c: 251: else if(!rightWall && (getSomethingInTheWay() != RIGHT))
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u4601
	goto	u4600
u4601:
	goto	l11225
u4600:
	
l11221:	
	fcall	_getSomethingInTheWay
	xorlw	03h
	skipnz
	goto	u4611
	goto	u4610
u4611:
	goto	l11225
u4610:
	line	252
	
l11223:	
;main.c: 252: goRight();
	fcall	_goRight
	goto	l6742
	line	253
	
l6740:	
	line	254
	
l11225:	
;main.c: 253: else
;main.c: 254: goBackward();
	fcall	_goBackward
	goto	l6742
	
l6741:	
	goto	l6742
	
l6739:	
	goto	l6742
	
l6737:	
	line	255
	
l6742:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1238,local,class=CODE,delta=2
global __ptext1238
__ptext1238:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 175 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1238
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	175
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 1
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	176
	
l11183:	
;main.c: 176: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	177
;main.c: 177: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	179
	
l11185:	
;main.c: 179: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4471
	goto	u4470
	
u4471:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u4484
u4470:
	bcf	(_leftWall/8),(_leftWall)&7
u4484:
	line	180
	
l11187:	
;main.c: 180: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4491
	goto	u4490
u4491:
	goto	l11191
u4490:
	line	181
	
l11189:	
;main.c: 181: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	goto	l6728
	line	182
	
l6727:	
	line	183
	
l11191:	
;main.c: 182: else
;main.c: 183: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6728:	
	line	185
;main.c: 185: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	186
	
l11193:	
;main.c: 186: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4501
	goto	u4500
	
u4501:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u4514
u4500:
	bcf	(_frontWall/8),(_frontWall)&7
u4514:
	line	188
	
l11195:	
;main.c: 188: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4521
	goto	u4520
u4521:
	goto	l11199
u4520:
	line	189
	
l11197:	
;main.c: 189: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	goto	l6730
	line	190
	
l6729:	
	line	191
	
l11199:	
;main.c: 190: else
;main.c: 191: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6730:	
	line	193
;main.c: 193: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	194
	
l11201:	
;main.c: 194: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4531
	goto	u4530
	
u4531:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u4544
u4530:
	bcf	(_rightWall/8),(_rightWall)&7
u4544:
	line	196
	
l11203:	
;main.c: 196: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u4551
	goto	u4550
u4551:
	goto	l11209
u4550:
	line	198
	
l11205:	
;main.c: 197: {
;main.c: 198: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	199
	
l11207:	
;main.c: 199: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	200
;main.c: 200: }else
	goto	l6732
	
l6731:	
	line	201
	
l11209:	
;main.c: 201: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l6732:	
	line	203
;main.c: 203: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	204
	
l6733:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text1239,local,class=CODE,delta=2
global __ptext1239
__ptext1239:

;; *************** function _goRight *****************
;; Defined at:
;;		line 251 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1239
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	251
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 2
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	252
	
l11173:	
;drive.c: 252: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	253
;drive.c: 253: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	254
	
l11175:	
;drive.c: 254: turnRight90();
	fcall	_turnRight90
	line	255
	
l11177:	
;drive.c: 255: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	256
	
l11179:	
;drive.c: 256: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	257
	
l11181:	
;drive.c: 257: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	258
	
l5865:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1240,local,class=CODE,delta=2
global __ptext1240
__ptext1240:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 230 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1240
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	230
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 2
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	231
	
l11163:	
;drive.c: 231: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	232
;drive.c: 232: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	233
	
l11165:	
;drive.c: 233: turnLeft90();
	fcall	_turnLeft90
	line	234
	
l11167:	
;drive.c: 234: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	235
	
l11169:	
;drive.c: 235: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	236
	
l11171:	
;drive.c: 236: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	237
	
l5859:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1241,local,class=CODE,delta=2
global __ptext1241
__ptext1241:

;; *************** function _goForward *****************
;; Defined at:
;;		line 215 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1241
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	215
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 2
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	216
	
l11151:	
;drive.c: 216: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	217
;drive.c: 217: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	218
	
l11153:	
;drive.c: 218: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	219
	
l11155:	
;drive.c: 219: if( (getCurrentX() == 1 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	01h
	skipz
	goto	u4451
	goto	u4450
u4451:
	goto	l11161
u4450:
	
l11157:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u4461
	goto	u4460
u4461:
	goto	l11161
u4460:
	line	221
	
l11159:	
;drive.c: 220: {
;drive.c: 221: driveForDistance(800);
	movlw	low(0320h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(0320h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	222
;drive.c: 222: }else
	goto	l5856
	
l5854:	
	line	224
	
l11161:	
;drive.c: 223: {
;drive.c: 224: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	goto	l5856
	line	225
	
l5855:	
	line	226
	
l5856:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1242,local,class=CODE,delta=2
global __ptext1242
__ptext1242:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 204 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1242
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	204
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 1
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	205
	
l11141:	
;drive.c: 205: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	206
;drive.c: 206: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	207
	
l11143:	
;drive.c: 207: turnAround();
	fcall	_turnAround
	line	208
	
l11145:	
;drive.c: 208: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	209
	
l11147:	
;drive.c: 209: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	210
	
l11149:	
;drive.c: 210: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	211
	
l5851:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_wallFollow
psect	text1243,local,class=CODE,delta=2
global __ptext1243
__ptext1243:

;; *************** function _wallFollow *****************
;; Defined at:
;;		line 467 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1243
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	467
	global	__size_of_wallFollow
	__size_of_wallFollow	equ	__end_of_wallFollow-_wallFollow
	
_wallFollow:	
	opt	stack 2
; Regs used in _wallFollow: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	468
	
l11125:	
;main.c: 468: int distanceToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(wallFollow@distanceToWall+1)
	addwf	(wallFollow@distanceToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(wallFollow@distanceToWall)
	addwf	(wallFollow@distanceToWall)

	line	469
	
l11127:	
;main.c: 469: if((distanceToWall > 86) && (distanceToWall < 100))
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(057h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4425
	movlw	low(057h)
	subwf	(wallFollow@distanceToWall),w
u4425:

	skipc
	goto	u4421
	goto	u4420
u4421:
	goto	l11135
u4420:
	
l11129:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4435
	movlw	low(064h)
	subwf	(wallFollow@distanceToWall),w
u4435:

	skipnc
	goto	u4431
	goto	u4430
u4431:
	goto	l11135
u4430:
	line	471
	
l11131:	
;main.c: 470: {
;main.c: 471: drive(0, 50, 0, 1);
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
	line	472
;main.c: 472: waitFor(157,0,8);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(08h)
	movwf	(??_wallFollow+0)+0
	movf	(??_wallFollow+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	473
;main.c: 473: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	474
	
l11133:	
;main.c: 474: _delay((unsigned long)((1000)*(20000000/4000.0)));
	opt asmopt_off
movlw  26
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_wallFollow+0)+0+2),f
movlw	94
movwf	((??_wallFollow+0)+0+1),f
	movlw	134
movwf	((??_wallFollow+0)+0),f
u4837:
	decfsz	((??_wallFollow+0)+0),f
	goto	u4837
	decfsz	((??_wallFollow+0)+0+1),f
	goto	u4837
	decfsz	((??_wallFollow+0)+0+2),f
	goto	u4837
	clrwdt
opt asmopt_on

	line	475
;main.c: 475: }
	goto	l6834
	line	476
	
l6831:	
	
l11135:	
;main.c: 476: else if(distanceToWall < 36)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(wallFollow@distanceToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(024h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4445
	movlw	low(024h)
	subwf	(wallFollow@distanceToWall),w
u4445:

	skipnc
	goto	u4441
	goto	u4440
u4441:
	goto	l6834
u4440:
	line	479
	
l11137:	
;main.c: 477: {
;main.c: 479: drive(0, 50, 255, 255);
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
	line	480
;main.c: 480: waitFor(157,255,0b11111000);
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
	line	481
;main.c: 481: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	482
	
l11139:	
;main.c: 482: _delay((unsigned long)((1000)*(20000000/4000.0)));
	opt asmopt_off
movlw  26
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_wallFollow+0)+0+2),f
movlw	94
movwf	((??_wallFollow+0)+0+1),f
	movlw	134
movwf	((??_wallFollow+0)+0),f
u4847:
	decfsz	((??_wallFollow+0)+0),f
	goto	u4847
	decfsz	((??_wallFollow+0)+0+1),f
	goto	u4847
	decfsz	((??_wallFollow+0)+0+2),f
	goto	u4847
	clrwdt
opt asmopt_on

	goto	l6834
	line	483
	
l6833:	
	goto	l6834
	line	484
	
l6832:	
	
l6834:	
	return
	opt stack 0
GLOBAL	__end_of_wallFollow
	__end_of_wallFollow:
;; =============== function _wallFollow ends ============

	signat	_wallFollow,88
	global	_findWall
psect	text1244,local,class=CODE,delta=2
global __ptext1244
__ptext1244:

;; *************** function _findWall *****************
;; Defined at:
;;		line 449 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1244
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	449
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 1
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	450
	
l11113:	
;main.c: 450: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4415
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u4415:

	skipc
	goto	u4411
	goto	u4410
u4411:
	goto	l11121
u4410:
	line	451
	
l11115:	
;main.c: 451: return 0;
	clrc
	
	goto	l6821
	
l11117:	
	goto	l6821
	
l11119:	
	goto	l6821
	line	452
	
l6820:	
	line	453
	
l11121:	
;main.c: 452: else
;main.c: 453: return 1;
	setc
	
	goto	l6821
	
l11123:	
	goto	l6821
	
l6822:	
	line	454
	
l6821:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text1245,local,class=CODE,delta=2
global __ptext1245
__ptext1245:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 324 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1245
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	324
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 2
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	326
	
l11087:	
;drive.c: 326: int distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	line	327
	
l11089:	
;drive.c: 327: if(distToWall < 45)
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4375
	movlw	low(02Dh)
	subwf	(frontWallCorrect@distToWall),w
u4375:

	skipnc
	goto	u4371
	goto	u4370
u4371:
	goto	l11101
u4370:
	line	329
	
l11091:	
;drive.c: 328: {
;drive.c: 329: drive(255, 125, 128, 0);
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
	line	330
;drive.c: 330: while(distToWall < 51)
	goto	l11095
	
l5888:	
	line	331
	
l11093:	
;drive.c: 331: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l11095
	
l5887:	
	line	330
	
l11095:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(033h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4385
	movlw	low(033h)
	subwf	(frontWallCorrect@distToWall),w
u4385:

	skipc
	goto	u4381
	goto	u4380
u4381:
	goto	l11093
u4380:
	goto	l11097
	
l5889:	
	line	332
	
l11097:	
;drive.c: 332: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	333
	
l11099:	
;drive.c: 333: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	334
;drive.c: 334: }
	goto	l5895
	line	335
	
l5886:	
	
l11101:	
;drive.c: 335: else if(distToWall > 55)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4395
	movlw	low(038h)
	subwf	(frontWallCorrect@distToWall),w
u4395:

	skipc
	goto	u4391
	goto	u4390
u4391:
	goto	l5895
u4390:
	line	337
	
l11103:	
;drive.c: 336: {
;drive.c: 337: drive(0, 250, 128, 0);
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
	line	338
;drive.c: 338: while(distToWall > 49)
	goto	l11107
	
l5893:	
	line	339
	
l11105:	
;drive.c: 339: distToWall = readIR();
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall+1)
	addwf	(frontWallCorrect@distToWall+1)
	movf	(0+(?_readIR)),w
	clrf	(frontWallCorrect@distToWall)
	addwf	(frontWallCorrect@distToWall)

	goto	l11107
	
l5892:	
	line	338
	
l11107:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(frontWallCorrect@distToWall+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(032h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4405
	movlw	low(032h)
	subwf	(frontWallCorrect@distToWall),w
u4405:

	skipnc
	goto	u4401
	goto	u4400
u4401:
	goto	l11105
u4400:
	goto	l11109
	
l5894:	
	line	340
	
l11109:	
;drive.c: 340: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	341
	
l11111:	
;drive.c: 341: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	goto	l5895
	line	342
	
l5891:	
	goto	l5895
	line	344
	
l5890:	
	
l5895:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_driveForDistance
psect	text1246,local,class=CODE,delta=2
global __ptext1246
__ptext1246:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 32 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
;;		_getCurrentY
;;		_getCurrentX
;;		_findFinalDestination
;;		_turnRight90
;;		_updateOrientation
;;		_turnLeft90
;; This function is called by:
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1246
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	32
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 2
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	35
	
l11017:	
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
	
l11019:	
;drive.c: 38: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	39
	
l11021:	
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
	
l11023:	
;drive.c: 40: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	42
;drive.c: 42: while(moving)
	goto	l11085
	
l5822:	
	line	44
	
l11025:	
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
	goto	u4295
	movlw	low(064h)
	subwf	(driveForDistance@distance),w
u4295:

	skipc
	goto	u4291
	goto	u4290
u4291:
	goto	l11041
u4290:
	line	47
	
l11027:	
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
	
l11029:	
;drive.c: 50: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u4301
	goto	u4300
u4301:
	goto	l11033
u4300:
	line	52
	
l11031:	
;drive.c: 51: {
;drive.c: 52: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	53
;drive.c: 53: ser_putch(7);
	movlw	(07h)
	fcall	_ser_putch
	line	54
;drive.c: 54: cliff = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@cliff)	;volatile
	goto	l11033
	line	67
	
l5824:	
	line	68
	
l11033:	
;drive.c: 67: }
;drive.c: 68: if(cliff != 0)
	movf	(driveForDistance@cliff),w	;volatile
	skipz
	goto	u4310
	goto	l11041
u4310:
	line	70
	
l11035:	
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
	
l11037:	
;drive.c: 72: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	88
	
l11039:	
;drive.c: 88: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11041
	line	89
	
l5825:	
	goto	l11041
	line	90
	
l5823:	
	line	93
	
l11041:	
;drive.c: 89: }
;drive.c: 90: }
;drive.c: 93: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	94
	
l11043:	
;drive.c: 94: ser_putch(13);
	movlw	(0Dh)
	fcall	_ser_putch
	line	95
	
l11045:	
;drive.c: 95: virtualWall = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@virtualWall)	;volatile
	line	96
	
l11047:	
;drive.c: 96: if(virtualWall == 1)
	movf	(driveForDistance@virtualWall),w	;volatile
	xorlw	01h
	skipz
	goto	u4321
	goto	u4320
u4321:
	goto	l11069
u4320:
	line	98
	
l11049:	
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
;drive.c: 100: goReverse();
	fcall	_goReverse
	line	101
	
l11051:	
;drive.c: 101: clearSuccessfulDrive();
	fcall	_clearSuccessfulDrive
	line	102
	
l11053:	
;drive.c: 102: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u4331
	goto	u4330
u4331:
	goto	l11061
u4330:
	line	104
	
l11055:	
;drive.c: 103: {
;drive.c: 104: somethingInTheWay = LEFT;
	clrf	(_somethingInTheWay)	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay),f	;volatile
	line	105
	
l11057:	
;drive.c: 105: turnRight90();
	fcall	_turnRight90
	line	106
	
l11059:	
;drive.c: 106: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	107
;drive.c: 107: }
	goto	l5828
	line	108
	
l5827:	
	
l11061:	
;drive.c: 108: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u4341
	goto	u4340
u4341:
	goto	l5829
u4340:
	line	110
	
l11063:	
;drive.c: 109: {
;drive.c: 110: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	line	111
	
l11065:	
;drive.c: 111: turnLeft90();
	fcall	_turnLeft90
	line	112
	
l11067:	
;drive.c: 112: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	113
;drive.c: 113: }
	goto	l5828
	line	114
	
l5829:	
	line	115
;drive.c: 114: else
;drive.c: 115: somethingInTheWay = FORWARD;
	clrf	(_somethingInTheWay)	;volatile
	goto	l5828
	
l5830:	
	
l5828:	
	line	116
;drive.c: 116: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11069
	line	117
	
l5826:	
	line	161
	
l11069:	
;drive.c: 117: }
;drive.c: 161: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	162
;drive.c: 162: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	163
;drive.c: 163: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	164
;drive.c: 164: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	165
	
l11071:	
;drive.c: 165: deltaDistance = high*256 + low;
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
	line	166
	
l11073:	
;drive.c: 166: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	167
	
l11075:	
;drive.c: 167: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u4355
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u4355:

	skipc
	goto	u4351
	goto	u4350
u4351:
	goto	l11085
u4350:
	line	169
	
l11077:	
;drive.c: 168: {
;drive.c: 169: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	170
	
l11079:	
;drive.c: 170: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	line	171
	
l11081:	
;drive.c: 171: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	172
	
l11083:	
;drive.c: 172: somethingInTheWay = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	goto	l11085
	line	173
	
l5831:	
	goto	l11085
	line	174
	
l5821:	
	line	42
	
l11085:	
	btfsc	(_moving/8),(_moving)&7
	goto	u4361
	goto	u4360
u4361:
	goto	l11025
u4360:
	goto	l5833
	
l5832:	
	line	175
	
l5833:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_updateLocation
psect	text1247,local,class=CODE,delta=2
global __ptext1247
__ptext1247:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 258 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1247
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	258
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 4
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	259
	
l10993:	
;main.c: 259: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	260
;main.c: 260: switch(getOrientation())
	goto	l11013
	line	262
;main.c: 261: {
;main.c: 262: case NORTH:
	
l6746:	
	line	263
	
l10995:	
;main.c: 263: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	264
	
l10997:	
;main.c: 264: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	265
;main.c: 265: break;
	goto	l11015
	line	266
;main.c: 266: case SOUTH:
	
l6748:	
	line	267
	
l10999:	
;main.c: 267: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	268
	
l11001:	
;main.c: 268: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	269
;main.c: 269: break;
	goto	l11015
	line	270
;main.c: 270: case EAST:
	
l6749:	
	line	271
	
l11003:	
;main.c: 271: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	272
	
l11005:	
;main.c: 272: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	273
;main.c: 273: break;
	goto	l11015
	line	274
;main.c: 274: case WEST:
	
l6750:	
	line	275
	
l11007:	
;main.c: 275: --xCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_xCoord),f	;volatile
	line	276
	
l11009:	
;main.c: 276: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	277
;main.c: 277: break;
	goto	l11015
	line	278
;main.c: 278: default:
	
l6751:	
	line	279
;main.c: 279: break;
	goto	l11015
	line	280
	
l11011:	
;main.c: 280: }
	goto	l11015
	line	260
	
l6745:	
	
l11013:	
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
	goto	l11007
	xorlw	1^0	; case 1
	skipnz
	goto	l10999
	xorlw	2^1	; case 2
	skipnz
	goto	l11003
	xorlw	3^2	; case 3
	skipnz
	goto	l10995
	goto	l11015
	opt asmopt_on

	line	280
	
l6747:	
	line	282
	
l11015:	
;main.c: 282: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	283
;main.c: 283: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	284
;main.c: 284: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	285
;main.c: 285: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	286
	
l6752:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text1248,local,class=CODE,delta=2
global __ptext1248
__ptext1248:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 150 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  victim          1    6[BANK0 ] unsigned char 
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
psect	text1248
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	150
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 4
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	151
	
l10971:	
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
	
l10973:	
;main.c: 155: if(victim > 241 && victim != 255)
	movlw	(0F2h)
	subwf	(lookForVictim@victim),w
	skipc
	goto	u4261
	goto	u4260
u4261:
	goto	l6724
u4260:
	
l10975:	
	movf	(lookForVictim@victim),w
	xorlw	0FFh
	skipnz
	goto	u4271
	goto	u4270
u4271:
	goto	l6724
u4270:
	line	157
	
l10977:	
;main.c: 156: {
;main.c: 157: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4281
	goto	u4280
u4281:
	goto	l10987
u4280:
	line	159
	
l10979:	
;main.c: 158: {
;main.c: 159: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	160
	
l10981:	
;main.c: 160: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	161
	
l10983:	
;main.c: 161: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	162
	
l10985:	
;main.c: 162: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	163
;main.c: 163: }
	goto	l6724
	line	164
	
l6722:	
	line	166
	
l10987:	
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
	
l10989:	
;main.c: 167: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	168
	
l10991:	
;main.c: 168: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l6724
	line	169
	
l6723:	
	goto	l6724
	line	170
	
l6721:	
	line	171
	
l6724:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text1249,local,class=CODE,delta=2
global __ptext1249
__ptext1249:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 139 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
;;		_getFinalX
;;		_getFinalY
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1249
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	139
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 4
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	140
	
l10959:	
;main.c: 140: if((xCoord == getFinalX()) && (yCoord == getFinalY()))
	fcall	_getFinalX
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u4241
	goto	u4240
u4241:
	goto	l6718
u4240:
	
l10961:	
	fcall	_getFinalY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u4251
	goto	u4250
u4251:
	goto	l6718
u4250:
	line	142
	
l10963:	
;main.c: 141: {
;main.c: 142: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	143
	
l10965:	
;main.c: 143: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	144
	
l10967:	
;main.c: 144: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	145
	
l10969:	
;main.c: 145: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l6718
	line	146
	
l6717:	
	line	147
	
l6718:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text1250,local,class=CODE,delta=2
global __ptext1250
__ptext1250:

;; *************** function _init *****************
;; Defined at:
;;		line 102 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1250
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	102
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 3
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	103
	
l10927:	
;main.c: 103: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	104
	
l10929:	
;main.c: 104: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	106
	
l10931:	
;main.c: 106: init_adc();
	fcall	_init_adc
	line	107
	
l10933:	
;main.c: 107: lcd_init();
	fcall	_lcd_init
	line	109
	
l10935:	
;main.c: 109: TRISB = 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	112
	
l10937:	
;main.c: 112: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	114
	
l10939:	
;main.c: 114: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	115
	
l10941:	
;main.c: 115: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	116
	
l10943:	
;main.c: 116: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	117
	
l10945:	
;main.c: 117: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	118
	
l10947:	
;main.c: 118: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	121
	
l10949:	
;main.c: 121: PEIE = 1;
	bsf	(94/8),(94)&7
	line	122
	
l10951:	
;main.c: 122: GIE = 1;
	bsf	(95/8),(95)&7
	line	124
	
l10953:	
;main.c: 124: ser_init();
	fcall	_ser_init
	line	125
	
l10955:	
;main.c: 125: initIRobot();
	fcall	_initIRobot
	line	126
	
l10957:	
;main.c: 126: initSongs();
	fcall	_initSongs
	line	127
	
l6711:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_goReverse
psect	text1251,local,class=CODE,delta=2
global __ptext1251
__ptext1251:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 240 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1251
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	240
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 2
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	241
	
l10917:	
;drive.c: 241: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	242
;drive.c: 242: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	243
	
l10919:	
;drive.c: 243: drive(255, 125, 128, 0);
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
	line	244
	
l10921:	
;drive.c: 244: waitFor(156,254,12);
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
	line	245
	
l10923:	
;drive.c: 245: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	246
	
l10925:	
;drive.c: 246: _delay((unsigned long)((2000)*(20000000/4000.0)));
	opt asmopt_off
movlw  51
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_goReverse+0)+0+2),f
movlw	188
movwf	((??_goReverse+0)+0+1),f
	movlw	16
movwf	((??_goReverse+0)+0),f
u4857:
	decfsz	((??_goReverse+0)+0),f
	goto	u4857
	decfsz	((??_goReverse+0)+0+1),f
	goto	u4857
	decfsz	((??_goReverse+0)+0+2),f
	goto	u4857
opt asmopt_on

	line	247
	
l5862:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_readIR
psect	text1252,local,class=CODE,delta=2
global __ptext1252
__ptext1252:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
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
psect	text1252
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 2
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l10911:	
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
	
l10913:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l5083
	
l10915:	
	line	36
	
l5083:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_findFinalDestination
psect	text1253,local,class=CODE,delta=2
global __ptext1253
__ptext1253:

;; *************** function _findFinalDestination *****************
;; Defined at:
;;		line 12 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
;; Parameters:    Size  Location     Type
;;  virtualWallX    1    wreg     unsigned char 
;;  virtualWallY    1    4[BANK0 ] unsigned char 
;;  robotOrienta    1    5[BANK0 ] enum E1096
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
psect	text1253
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
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
	
l10831:	
;map.c: 13: switch (virtualWallX)
	goto	l10907
	line	15
;map.c: 14: {
;map.c: 15: case 0:
	
l2849:	
	line	16
;map.c: 16: switch (virtualWallY)
	goto	l10841
	line	20
;map.c: 17: {
;map.c: 20: case 1:
	
l2851:	
	line	21
;map.c: 21: finalX = 0;
	clrf	(_finalX)
	line	22
	
l10833:	
;map.c: 22: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	23
;map.c: 23: break;
	goto	l10909
	line	24
;map.c: 24: case 2:
	
l2853:	
	line	25
;map.c: 25: finalX = 0;
	clrf	(_finalX)
	line	26
	
l10835:	
;map.c: 26: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	27
;map.c: 27: break;
	goto	l10909
	line	28
;map.c: 28: case 3:
	
l2854:	
	line	29
;map.c: 29: finalX = 0;
	clrf	(_finalX)
	line	30
	
l10837:	
;map.c: 30: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	31
;map.c: 31: break;
	goto	l10909
	line	32
;map.c: 32: default:
	
l2855:	
	line	33
;map.c: 33: break;
	goto	l10909
	line	34
	
l10839:	
;map.c: 34: }
	goto	l10909
	line	16
	
l2850:	
	
l10841:	
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
	goto	l10909
	opt asmopt_on

	line	34
	
l2852:	
	line	35
;map.c: 35: break;
	goto	l10909
	line	37
;map.c: 37: case 1:
	
l2857:	
	line	38
;map.c: 38: switch (virtualWallY)
	goto	l10859
	line	40
;map.c: 39: {
;map.c: 40: case 0:
	
l2859:	
	line	41
	
l10843:	
;map.c: 41: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	42
	
l10845:	
;map.c: 42: finalY = 0;
	clrf	(_finalY)
	line	43
;map.c: 43: break;
	goto	l10909
	line	44
;map.c: 44: case 1:
	
l2861:	
	line	45
	
l10847:	
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
	goto	l10909
	line	48
;map.c: 48: case 2:
	
l2862:	
	line	49
	
l10849:	
;map.c: 49: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	50
	
l10851:	
;map.c: 50: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	51
;map.c: 51: break;
	goto	l10909
	line	52
;map.c: 52: case 3:
	
l2863:	
	line	53
	
l10853:	
;map.c: 53: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	54
	
l10855:	
;map.c: 54: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	55
;map.c: 55: break;
	goto	l10909
	line	56
;map.c: 56: default:
	
l2864:	
	line	57
;map.c: 57: break;
	goto	l10909
	line	58
	
l10857:	
;map.c: 58: }
	goto	l10909
	line	38
	
l2858:	
	
l10859:	
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
	goto	l10843
	xorlw	1^0	; case 1
	skipnz
	goto	l10847
	xorlw	2^1	; case 2
	skipnz
	goto	l10849
	xorlw	3^2	; case 3
	skipnz
	goto	l10853
	goto	l10909
	opt asmopt_on

	line	58
	
l2860:	
	line	59
;map.c: 59: break;
	goto	l10909
	line	61
;map.c: 61: case 2:
	
l2865:	
	line	62
;map.c: 62: switch (virtualWallY)
	goto	l10877
	line	64
;map.c: 63: {
;map.c: 64: case 0:
	
l2867:	
	line	65
	
l10861:	
;map.c: 65: if(robotOrientation == WEST)
	movf	(findFinalDestination@robotOrientation),f
	skipz
	goto	u4211
	goto	u4210
u4211:
	goto	l10909
u4210:
	line	67
	
l10863:	
;map.c: 66: {
;map.c: 67: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	68
	
l10865:	
;map.c: 68: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	goto	l10909
	line	69
	
l2868:	
	line	70
;map.c: 69: }
;map.c: 70: break;
	goto	l10909
	line	71
;map.c: 71: case 1:
	
l2870:	
	line	72
	
l10867:	
;map.c: 72: finalX = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	73
	
l10869:	
;map.c: 73: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	74
;map.c: 74: break;
	goto	l10909
	line	75
;map.c: 75: case 2:
	
l2871:	
	line	76
	
l10871:	
;map.c: 76: if(robotOrientation == EAST)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	02h
	skipz
	goto	u4221
	goto	u4220
u4221:
	goto	l10909
u4220:
	line	78
	
l10873:	
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
	goto	l10909
	line	80
	
l2872:	
	line	81
;map.c: 80: }
;map.c: 81: break;
	goto	l10909
	line	84
;map.c: 84: default:
	
l2873:	
	line	85
;map.c: 85: break;
	goto	l10909
	line	86
	
l10875:	
;map.c: 86: }
	goto	l10909
	line	62
	
l2866:	
	
l10877:	
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
	goto	l10861
	xorlw	1^0	; case 1
	skipnz
	goto	l10867
	xorlw	2^1	; case 2
	skipnz
	goto	l10871
	goto	l10909
	opt asmopt_on

	line	86
	
l2869:	
	line	87
;map.c: 87: break;
	goto	l10909
	line	89
;map.c: 89: case 3:
	
l2874:	
	line	90
;map.c: 90: switch (virtualWallY)
	goto	l10887
	line	92
;map.c: 91: {
;map.c: 92: case 0:
	
l2876:	
	line	93
	
l10879:	
;map.c: 93: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	94
	
l10881:	
;map.c: 94: finalY = 0;
	clrf	(_finalY)
	line	95
;map.c: 95: break;
	goto	l10909
	line	98
;map.c: 98: case 2:
	
l2878:	
	line	99
	
l10883:	
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
	goto	l10909
	line	104
;map.c: 104: default:
	
l2879:	
	line	105
;map.c: 105: break;
	goto	l10909
	line	106
	
l10885:	
;map.c: 106: }
	goto	l10909
	line	90
	
l2875:	
	
l10887:	
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
	goto	l10879
	xorlw	2^0	; case 2
	skipnz
	goto	l10883
	goto	l10909
	opt asmopt_on

	line	106
	
l2877:	
	line	107
;map.c: 107: break;
	goto	l10909
	line	109
;map.c: 109: case 4:
	
l2880:	
	line	110
;map.c: 110: switch (virtualWallY)
	goto	l10903
	line	112
;map.c: 111: {
;map.c: 112: case 0:
	
l2882:	
	line	113
	
l10889:	
;map.c: 113: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	114
	
l10891:	
;map.c: 114: finalY = 0;
	clrf	(_finalY)
	line	115
;map.c: 115: break;
	goto	l10909
	line	116
;map.c: 116: case 1:
	
l2884:	
	line	117
	
l10893:	
;map.c: 117: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	118
	
l10895:	
;map.c: 118: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	119
;map.c: 119: break;
	goto	l10909
	line	120
;map.c: 120: case 2:
	
l2885:	
	line	121
	
l10897:	
;map.c: 121: if (robotOrientation == SOUTH)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	01h
	skipz
	goto	u4231
	goto	u4230
u4231:
	goto	l10909
u4230:
	line	123
	
l10899:	
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
	goto	l10909
	line	125
	
l2886:	
	line	126
;map.c: 125: }
;map.c: 126: break;
	goto	l10909
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
	goto	l10909
	line	131
;map.c: 131: default:
	
l2888:	
	line	132
;map.c: 132: break;
	goto	l10909
	line	133
	
l10901:	
;map.c: 133: }
	goto	l10909
	line	110
	
l2881:	
	
l10903:	
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
	goto	l10889
	xorlw	1^0	; case 1
	skipnz
	goto	l10893
	xorlw	2^1	; case 2
	skipnz
	goto	l10897
	xorlw	3^2	; case 3
	skipnz
	goto	l2887
	goto	l10909
	opt asmopt_on

	line	133
	
l2883:	
	line	134
;map.c: 134: break;
	goto	l10909
	line	136
;map.c: 136: default:
	
l2889:	
	line	137
;map.c: 137: break;
	goto	l10909
	line	138
	
l10905:	
;map.c: 138: }
	goto	l10909
	line	13
	
l2848:	
	
l10907:	
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
	goto	l10841
	xorlw	1^0	; case 1
	skipnz
	goto	l10859
	xorlw	2^1	; case 2
	skipnz
	goto	l10877
	xorlw	3^2	; case 3
	skipnz
	goto	l10887
	xorlw	4^3	; case 4
	skipnz
	goto	l10903
	goto	l10909
	opt asmopt_on

	line	138
	
l2856:	
	line	140
	
l10909:	
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
psect	text1254,local,class=CODE,delta=2
global __ptext1254
__ptext1254:

;; *************** function _updateMapData *****************
;; Defined at:
;;		line 135 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
psect	text1254
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
	line	135
	global	__size_of_updateMapData
	__size_of_updateMapData	equ	__end_of_updateMapData-_updateMapData
	
_updateMapData:	
	opt	stack 3
; Regs used in _updateMapData: [wreg+status,2+status,0+pclath+cstack]
;updateMapData@virtualW stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateMapData@virtualW)
	line	136
	
l10825:	
;eeprom.c: 136: char completeData = 0;
	clrf	(updateMapData@completeData)
	line	137
	
l10827:	
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
u4175:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4175
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
u4185:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4185
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
u4195:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4195
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
u4205:
	clrc
	rlf	(??_updateMapData+0)+0,f
	addlw	-1
	skipz
	goto	u4205
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
	
l10829:	
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
psect	text1255,local,class=CODE,delta=2
global __ptext1255
__ptext1255:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 305 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1255
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	305
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 4
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	306
	
l10817:	
;main.c: 306: if((xCoord == 1) && (yCoord == 3))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u4151
	goto	u4150
u4151:
	goto	l6769
u4150:
	
l10819:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4161
	goto	u4160
u4161:
	goto	l6769
u4160:
	line	308
	
l10821:	
;main.c: 307: {
;main.c: 308: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	309
;main.c: 309: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	310
	
l10823:	
;main.c: 310: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l6769
	line	311
	
l6768:	
	line	312
	
l6769:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_turnAround
psect	text1256,local,class=CODE,delta=2
global __ptext1256
__ptext1256:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 261 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1256
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	261
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 2
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	262
	
l10811:	
;drive.c: 262: drive(0, 50, 0, 1);
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
	line	263
;drive.c: 263: waitFor(157,0,180);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0B4h)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	264
;drive.c: 264: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	265
	
l10813:	
;drive.c: 265: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	25
movwf	((??_turnAround+0)+0+1),f
	movlw	154
movwf	((??_turnAround+0)+0),f
u4867:
	decfsz	((??_turnAround+0)+0),f
	goto	u4867
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4867
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4867
	nop2
opt asmopt_on

	line	266
	
l10815:	
;drive.c: 266: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	25
movwf	((??_turnAround+0)+0+1),f
	movlw	154
movwf	((??_turnAround+0)+0),f
u4877:
	decfsz	((??_turnAround+0)+0),f
	goto	u4877
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4877
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4877
	nop2
opt asmopt_on

	line	267
	
l5868:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_turnLeft90
psect	text1257,local,class=CODE,delta=2
global __ptext1257
__ptext1257:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 270 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1257
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	270
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 2
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	271
	
l10799:	
;drive.c: 271: drive(0, 50, 0, 1);
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
	line	272
	
l10801:	
;drive.c: 272: if( (getCurrentX() == 2 && getCurrentY() == 2))
	fcall	_getCurrentX
	xorlw	02h
	skipz
	goto	u4131
	goto	u4130
u4131:
	goto	l10807
u4130:
	
l10803:	
	fcall	_getCurrentY
	xorlw	02h
	skipz
	goto	u4141
	goto	u4140
u4141:
	goto	l10807
u4140:
	line	274
	
l10805:	
;drive.c: 273: {
;drive.c: 274: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	275
;drive.c: 275: }else
	goto	l5872
	
l5871:	
	line	277
	
l10807:	
;drive.c: 276: {
;drive.c: 277: waitFor(157,0,90);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(05Ah)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	278
	
l5872:	
	line	279
;drive.c: 278: }
;drive.c: 279: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	280
	
l10809:	
;drive.c: 280: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	25
movwf	((??_turnLeft90+0)+0+1),f
	movlw	154
movwf	((??_turnLeft90+0)+0),f
u4887:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u4887
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u4887
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u4887
	nop2
opt asmopt_on

	line	281
	
l5873:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnRight90
psect	text1258,local,class=CODE,delta=2
global __ptext1258
__ptext1258:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 284 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1258
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	284
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 2
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	285
	
l10795:	
;drive.c: 285: drive(0, 50, 255, 255);
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
	line	286
;drive.c: 286: waitFor(157,255,174);
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
	line	287
;drive.c: 287: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	288
	
l10797:	
;drive.c: 288: _delay((unsigned long)((3000)*(20000000/4000.0)));
	opt asmopt_off
movlw  77
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	25
movwf	((??_turnRight90+0)+0+1),f
	movlw	154
movwf	((??_turnRight90+0)+0),f
u4897:
	decfsz	((??_turnRight90+0)+0),f
	goto	u4897
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u4897
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u4897
	nop2
opt asmopt_on

	line	289
	
l5876:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_initSongs
psect	text1259,local,class=CODE,delta=2
global __ptext1259
__ptext1259:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 32 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
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
psect	text1259
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
	line	32
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 3
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	33
	
l10793:	
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
	movlw	(0x1/2)
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
	movlw	(0x0/2)
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
	movlw	(0x0/2)
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
psect	text1260,local,class=CODE,delta=2
global __ptext1260
__ptext1260:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
psect	text1260
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 4
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l10773:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l10775:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l10777:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l10779:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l10781:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l10783:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l10785:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l10787:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l10789:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l10791:	
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
psect	text1261,local,class=CODE,delta=2
global __ptext1261
__ptext1261:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
psect	text1261
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
	
l10771:	
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
psect	text1262,local,class=CODE,delta=2
global __ptext1262
__ptext1262:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
psect	text1262
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
	
l10767:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l10769:	
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
	global	_addNewData
psect	text1263,local,class=CODE,delta=2
global __ptext1263
__ptext1263:

;; *************** function _addNewData *****************
;; Defined at:
;;		line 94 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
;;		_updateMapData
;;		_writeEEPROMTestData
;; This function uses a non-reentrant model
;;
psect	text1263
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
	
l10761:	
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
	
l10763:	
;eeprom.c: 96: addressCount ++;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_addNewData+0)+0
	movf	(??_addNewData+0)+0,w
	addwf	(_addressCount),f
	line	97
	
l10765:	
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
psect	text1264,local,class=CODE,delta=2
global __ptext1264
__ptext1264:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
;; Parameters:    Size  Location     Type
;;  s               1    wreg     PTR const unsigned char 
;;		 -> STR_4(17), STR_3(17), STR_2(14), STR_1(15), 
;; Auto vars:     Size  Location     Type
;;  s               1    4[BANK0 ] PTR const unsigned char 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		_lcd_write_data
;; This function is called by:
;;		_main
;;		_testEEPROM
;; This function uses a non-reentrant model
;;
psect	text1264
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
	
l10753:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l10759
	
l2136:	
	
l10755:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l10757:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l10759
	
l2135:	
	
l10759:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u4121
	goto	u4120
u4121:
	goto	l10755
u4120:
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
psect	text1265,local,class=CODE,delta=2
global __ptext1265
__ptext1265:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
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
psect	text1265
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
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
	
l10737:	
;adc.c: 8: switch(channel)
	goto	l10745
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
	goto	l10747
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
	goto	l10747
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
	goto	l10747
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
	goto	l10747
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
	goto	l10747
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l10739:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l10741:	
	goto	l697
	line	39
	
l10743:	
;adc.c: 39: }
	goto	l10747
	line	8
	
l689:	
	
l10745:	
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
	goto	l10739
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l10747:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u4907:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u4907
opt asmopt_on

	line	43
	
l10749:	
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
	
l10751:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_initIRobot
psect	text1266,local,class=CODE,delta=2
global __ptext1266
__ptext1266:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 130 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1266
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	130
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 4
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	131
	
l10731:	
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
u4917:
	decfsz	((??_initIRobot+0)+0),f
	goto	u4917
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u4917
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u4917
	nop2
opt asmopt_on

	line	132
	
l10733:	
;main.c: 132: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	133
	
l10735:	
;main.c: 133: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	134
	
l6714:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_waitFor
psect	text1267,local,class=CODE,delta=2
global __ptext1267
__ptext1267:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 299 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1267
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	299
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 2
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	300
	
l10723:	
;drive.c: 300: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u4927:
	decfsz	((??_waitFor+0)+0),f
	goto	u4927
	decfsz	((??_waitFor+0)+0+1),f
	goto	u4927
	decfsz	((??_waitFor+0)+0+2),f
	goto	u4927
	nop2
opt asmopt_on

	line	301
	
l10725:	
;drive.c: 301: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	302
	
l10727:	
;drive.c: 302: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	303
	
l10729:	
;drive.c: 303: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	304
	
l5883:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_drive
psect	text1268,local,class=CODE,delta=2
global __ptext1268
__ptext1268:

;; *************** function _drive *****************
;; Defined at:
;;		line 22 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1268
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
	
l10711:	
;drive.c: 23: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u4937:
	decfsz	((??_drive+0)+0),f
	goto	u4937
	decfsz	((??_drive+0)+0+1),f
	goto	u4937
	decfsz	((??_drive+0)+0+2),f
	goto	u4937
	nop2
opt asmopt_on

	line	24
	
l10713:	
;drive.c: 24: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	25
	
l10715:	
;drive.c: 25: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	26
	
l10717:	
;drive.c: 26: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	27
	
l10719:	
;drive.c: 27: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	28
	
l10721:	
;drive.c: 28: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	29
	
l5818:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_rotateIR
psect	text1269,local,class=CODE,delta=2
global __ptext1269
__ptext1269:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
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
psect	text1269
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
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
	
l10697:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l10699:	
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
	
l10701:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l10703:	
;ir.c: 48: _delay((unsigned long)((50)*(20000000/4000.0)));
	opt asmopt_off
movlw  2
movwf	((??_rotateIR+0)+0+2),f
movlw	69
movwf	((??_rotateIR+0)+0+1),f
	movlw	169
movwf	((??_rotateIR+0)+0),f
u4947:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4947
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4947
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4947
	nop2
opt asmopt_on

	line	44
	
l10705:	
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
	goto	u4111
	goto	u4110
u4111:
	goto	l5087
u4110:
	goto	l10707
	
l5088:	
	line	51
	
l10707:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l10709:	
;ir.c: 52: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_rotateIR+0)+0+2),f
movlw	138
movwf	((??_rotateIR+0)+0+1),f
	movlw	86
movwf	((??_rotateIR+0)+0),f
u4957:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4957
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4957
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4957
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
psect	text1270,local,class=CODE,delta=2
global __ptext1270
__ptext1270:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
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
psect	text1270
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 3
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l10637:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4045
	movlw	low(052h)
	subwf	(convert@adc_value),w
u4045:

	skipnc
	goto	u4041
	goto	u4040
u4041:
	goto	l10645
u4040:
	line	13
	
l10639:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5067
	
l10641:	
	goto	l5067
	
l10643:	
	goto	l5067
	line	14
	
l5066:	
	
l10645:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4055
	movlw	low(085h)
	subwf	(convert@adc_value),w
u4055:

	skipnc
	goto	u4051
	goto	u4050
u4051:
	goto	l10653
u4050:
	line	15
	
l10647:	
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
	
l10649:	
	goto	l5067
	
l10651:	
	goto	l5067
	line	16
	
l5069:	
	
l10653:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4065
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u4065:

	skipnc
	goto	u4061
	goto	u4060
u4061:
	goto	l10661
u4060:
	line	17
	
l10655:	
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
	
l10657:	
	goto	l5067
	
l10659:	
	goto	l5067
	line	18
	
l5071:	
	
l10661:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4075
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u4075:

	skipnc
	goto	u4071
	goto	u4070
u4071:
	goto	l10669
u4070:
	line	19
	
l10663:	
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
	
l10665:	
	goto	l5067
	
l10667:	
	goto	l5067
	line	20
	
l5073:	
	
l10669:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4085
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u4085:

	skipnc
	goto	u4081
	goto	u4080
u4081:
	goto	l10677
u4080:
	line	21
	
l10671:	
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
	
l10673:	
	goto	l5067
	
l10675:	
	goto	l5067
	line	22
	
l5075:	
	
l10677:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4095
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u4095:

	skipnc
	goto	u4091
	goto	u4090
u4091:
	goto	l10685
u4090:
	line	23
	
l10679:	
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
	
l10681:	
	goto	l5067
	
l10683:	
	goto	l5067
	line	24
	
l5077:	
	
l10685:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4105
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u4105:

	skipnc
	goto	u4101
	goto	u4100
u4101:
	goto	l10693
u4100:
	line	25
	
l10687:	
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
	
l10689:	
	goto	l5067
	
l10691:	
	goto	l5067
	line	26
	
l5079:	
	
l10693:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5067
	
l10695:	
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
psect	text1271,local,class=CODE,delta=2
global __ptext1271
__ptext1271:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 26 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
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
psect	text1271
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\songs.c"
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
	
l10635:	
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
psect	text1272,local,class=CODE,delta=2
global __ptext1272
__ptext1272:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
psect	text1272
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 3
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l10627:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l10633
	line	75
	
l3643:	
	line	76
	
l10629:	
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
	goto	u4020
	decf	(??_ser_putArr+0)+0,f
u4020:
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
	
l10631:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l10633
	
l3642:	
	
l10633:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u4035
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u4035:

	skipc
	goto	u4031
	goto	u4030
u4031:
	goto	l10629
u4030:
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
psect	text1273,local,class=CODE,delta=2
global __ptext1273
__ptext1273:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
psect	text1273
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 3
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l10611:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l10613
	
l3637:	
	line	62
;ser.c: 62: continue;
	goto	l10613
	
l3636:	
	line	61
	
l10613:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u4011
	goto	u4010
u4011:
	goto	l10613
u4010:
	
l3638:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l10615:	
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
	
l10617:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l10619:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l10621:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l10623:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l3639
	
l10625:	
	line	70
	
l3639:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_lcd_write_data
psect	text1274,local,class=CODE,delta=2
global __ptext1274
__ptext1274:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1274
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
	
l10603:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l10605:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l10607:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l10609:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u4967:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u4967
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u4967
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
psect	text1275,local,class=CODE,delta=2
global __ptext1275
__ptext1275:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
psect	text1275
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\lcd.c"
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
	
l10595:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l10597:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l10599:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l10601:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u4977:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u4977
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u4977
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
	global	_writeEEPROM
psect	text1276,local,class=CODE,delta=2
global __ptext1276
__ptext1276:

;; *************** function _writeEEPROM *****************
;; Defined at:
;;		line 27 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
psect	text1276
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
	
l10577:	
;eeprom.c: 29: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_writeEEPROM+0)+0+2),f
movlw	138
movwf	((??_writeEEPROM+0)+0+1),f
	movlw	86
movwf	((??_writeEEPROM+0)+0),f
u4987:
	decfsz	((??_writeEEPROM+0)+0),f
	goto	u4987
	decfsz	((??_writeEEPROM+0)+0+1),f
	goto	u4987
	decfsz	((??_writeEEPROM+0)+0+2),f
	goto	u4987
	nop2
opt asmopt_on

	line	30
	
l10579:	
;eeprom.c: 30: initEEPROMMode();
	fcall	_initEEPROMMode
	line	32
	
l10581:	
;eeprom.c: 32: writeSPIByte(6);
	movlw	(06h)
	fcall	_writeSPIByte
	line	33
	
l10583:	
;eeprom.c: 33: initEEPROMMode();
	fcall	_initEEPROMMode
	line	36
	
l10585:	
;eeprom.c: 36: writeSPIByte(2);
	movlw	(02h)
	fcall	_writeSPIByte
	line	39
	
l10587:	
;eeprom.c: 39: writeSPIByte(addressH);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressH),w
	fcall	_writeSPIByte
	line	42
	
l10589:	
;eeprom.c: 42: writeSPIByte(addressL);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@addressL),w
	fcall	_writeSPIByte
	line	45
	
l10591:	
;eeprom.c: 45: writeSPIByte(data);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(writeEEPROM@data),w
	fcall	_writeSPIByte
	line	46
	
l10593:	
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
psect	text1277,local,class=CODE,delta=2
global __ptext1277
__ptext1277:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
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
psect	text1277
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 5
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l10567:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l10569:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l10571:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l10573:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l10575:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u4997:
decfsz	(??_init_adc+0)+0,f
	goto	u4997
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
psect	text1278,local,class=CODE,delta=2
global __ptext1278
__ptext1278:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
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
psect	text1278
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 2
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l10557:	
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
	
l10559:	
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
	goto	u3991
	goto	u3990
u3991:
	goto	l703
u3990:
	goto	l10561
	
l705:	
	line	75
	
l10561:	
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
u4005:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4005
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l10563:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l10565:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1279,local,class=CODE,delta=2
global __ptext1279
__ptext1279:

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
psect	text1279
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 3
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l10517:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u3891
	goto	u3890
u3891:
	goto	l10521
u3890:
	line	11
	
l10519:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l10521
	line	13
	
l7657:	
	line	14
	
l10521:	
	btfss	(___awdiv@dividend+1),7
	goto	u3901
	goto	u3900
u3901:
	goto	l10527
u3900:
	line	15
	
l10523:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l10525:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l10527
	line	17
	
l7658:	
	line	18
	
l10527:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l10529:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u3911
	goto	u3910
u3911:
	goto	l10549
u3910:
	line	20
	
l10531:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l10537
	
l7661:	
	line	22
	
l10533:	
	movlw	01h
	
u3925:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u3925
	line	23
	
l10535:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l10537
	line	24
	
l7660:	
	line	21
	
l10537:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u3931
	goto	u3930
u3931:
	goto	l10533
u3930:
	goto	l10539
	
l7662:	
	goto	l10539
	line	25
	
l7663:	
	line	26
	
l10539:	
	movlw	01h
	
u3945:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u3945
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u3955
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u3955:
	skipc
	goto	u3951
	goto	u3950
u3951:
	goto	l10545
u3950:
	line	28
	
l10541:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l10543:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l10545
	line	30
	
l7664:	
	line	31
	
l10545:	
	movlw	01h
	
u3965:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u3965
	line	32
	
l10547:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u3971
	goto	u3970
u3971:
	goto	l10539
u3970:
	goto	l10549
	
l7665:	
	goto	l10549
	line	33
	
l7659:	
	line	34
	
l10549:	
	movf	(___awdiv@sign),w
	skipz
	goto	u3980
	goto	l10553
u3980:
	line	35
	
l10551:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l10553
	
l7666:	
	line	36
	
l10553:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l7667
	
l10555:	
	line	37
	
l7667:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1280,local,class=CODE,delta=2
global __ptext1280
__ptext1280:

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
psect	text1280
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 3
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l10505:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l10507
	line	6
	
l7517:	
	line	7
	
l10507:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u3851
	goto	u3850
u3851:
	goto	l7518
u3850:
	line	8
	
l10509:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l7518:	
	line	9
	movlw	01h
	
u3865:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u3865
	line	10
	
l10511:	
	movlw	01h
	
u3875:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u3875
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u3881
	goto	u3880
u3881:
	goto	l10507
u3880:
	goto	l10513
	
l7519:	
	line	12
	
l10513:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l7520
	
l10515:	
	line	13
	
l7520:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_updateNode
psect	text1281,local,class=CODE,delta=2
global __ptext1281
__ptext1281:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 289 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1281
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	289
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 6
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	290
	
l10475:	
;main.c: 290: if((xCoord == 2) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3751
	goto	u3750
u3751:
	goto	l10481
u3750:
	
l10477:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3761
	goto	u3760
u3761:
	goto	l10481
u3760:
	line	291
	
l10479:	
;main.c: 291: node = 1;
	clrf	(_node)	;volatile
	bsf	status,0
	rlf	(_node),f	;volatile
	goto	l6765
	line	292
	
l6755:	
	
l10481:	
;main.c: 292: else if((xCoord == 4) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u3771
	goto	u3770
u3771:
	goto	l10487
u3770:
	
l10483:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3781
	goto	u3780
u3781:
	goto	l10487
u3780:
	line	293
	
l10485:	
;main.c: 293: node = 2;
	movlw	(02h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6765
	line	294
	
l6757:	
	
l10487:	
;main.c: 294: else if((xCoord == 2) && (yCoord == 0))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3791
	goto	u3790
u3791:
	goto	l10493
u3790:
	
l10489:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u3801
	goto	u3800
u3801:
	goto	l10493
u3800:
	line	295
	
l10491:	
;main.c: 295: node = 3;
	movlw	(03h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6765
	line	296
	
l6759:	
	
l10493:	
;main.c: 296: else if((xCoord == 4) && (yCoord == 3))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u3811
	goto	u3810
u3811:
	goto	l10499
u3810:
	
l10495:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u3821
	goto	u3820
u3821:
	goto	l10499
u3820:
	line	297
	
l10497:	
;main.c: 297: node = 4;
	movlw	(04h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6765
	line	298
	
l6761:	
	
l10499:	
;main.c: 298: else if((xCoord == 2) && (yCoord == 1))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3831
	goto	u3830
u3831:
	goto	l6763
u3830:
	
l10501:	
	movf	(_yCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u3841
	goto	u3840
u3841:
	goto	l6763
u3840:
	line	299
	
l10503:	
;main.c: 299: node = 5;
	movlw	(05h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l6765
	line	300
	
l6763:	
	line	301
;main.c: 300: else
;main.c: 301: node = 0;
	clrf	(_node)	;volatile
	goto	l6765
	
l6764:	
	goto	l6765
	
l6762:	
	goto	l6765
	
l6760:	
	goto	l6765
	
l6758:	
	goto	l6765
	
l6756:	
	line	302
	
l6765:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getSuccessfulDrive
psect	text1282,local,class=CODE,delta=2
global __ptext1282
__ptext1282:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 193 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1282
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	193
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 6
; Regs used in _getSuccessfulDrive: [status]
	line	194
	
l10409:	
;drive.c: 194: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u3631
	goto	u3630
u3631:
	goto	l10413
u3630:
	
l10411:	
	clrc
	
	goto	l5845
	
l10151:	
	
l10413:	
	setc
	
	goto	l5845
	
l10153:	
	goto	l5845
	
l10415:	
	line	195
	
l5845:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_getSomethingInTheWay
psect	text1283,local,class=CODE,delta=2
global __ptext1283
__ptext1283:

;; *************** function _getSomethingInTheWay *****************
;; Defined at:
;;		line 183 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1283
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	183
	global	__size_of_getSomethingInTheWay
	__size_of_getSomethingInTheWay	equ	__end_of_getSomethingInTheWay-_getSomethingInTheWay
	
_getSomethingInTheWay:	
	opt	stack 5
; Regs used in _getSomethingInTheWay: [wreg]
	line	184
	
l10405:	
;drive.c: 184: return somethingInTheWay;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_somethingInTheWay),w	;volatile
	goto	l5839
	
l10407:	
	line	185
	
l5839:	
	return
	opt stack 0
GLOBAL	__end_of_getSomethingInTheWay
	__end_of_getSomethingInTheWay:
;; =============== function _getSomethingInTheWay ends ============

	signat	_getSomethingInTheWay,89
	global	_getOrientation
psect	text1284,local,class=CODE,delta=2
global __ptext1284
__ptext1284:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 178 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1284
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	178
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 5
; Regs used in _getOrientation: [wreg]
	line	179
	
l10401:	
;drive.c: 179: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l5836
	
l10403:	
	line	180
	
l5836:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_updateOrientation
psect	text1285,local,class=CODE,delta=2
global __ptext1285
__ptext1285:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 292 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1285
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	292
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 4
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	293
	
l10395:	
;drive.c: 293: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	294
	
l10397:	
;drive.c: 294: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u3621
	goto	u3620
u3621:
	goto	l5880
u3620:
	line	295
	
l10399:	
;drive.c: 295: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l5880
	
l5879:	
	line	296
	
l5880:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_getCurrentY
psect	text1286,local,class=CODE,delta=2
global __ptext1286
__ptext1286:

;; *************** function _getCurrentY *****************
;; Defined at:
;;		line 462 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1286
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	462
	global	__size_of_getCurrentY
	__size_of_getCurrentY	equ	__end_of_getCurrentY-_getCurrentY
	
_getCurrentY:	
	opt	stack 3
; Regs used in _getCurrentY: [wreg]
	line	463
	
l10391:	
;main.c: 463: return yCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	goto	l6828
	
l10393:	
	line	464
	
l6828:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentY
	__end_of_getCurrentY:
;; =============== function _getCurrentY ends ============

	signat	_getCurrentY,89
	global	_getCurrentX
psect	text1287,local,class=CODE,delta=2
global __ptext1287
__ptext1287:

;; *************** function _getCurrentX *****************
;; Defined at:
;;		line 457 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1287
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	457
	global	__size_of_getCurrentX
	__size_of_getCurrentX	equ	__end_of_getCurrentX-_getCurrentX
	
_getCurrentX:	
	opt	stack 3
; Regs used in _getCurrentX: [wreg]
	line	458
	
l10387:	
;main.c: 458: return xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	goto	l6825
	
l10389:	
	line	459
	
l6825:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentX
	__end_of_getCurrentX:
;; =============== function _getCurrentX ends ============

	signat	_getCurrentX,89
	global	_clearSuccessfulDrive
psect	text1288,local,class=CODE,delta=2
global __ptext1288
__ptext1288:

;; *************** function _clearSuccessfulDrive *****************
;; Defined at:
;;		line 188 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
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
psect	text1288
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\drive.c"
	line	188
	global	__size_of_clearSuccessfulDrive
	__size_of_clearSuccessfulDrive	equ	__end_of_clearSuccessfulDrive-_clearSuccessfulDrive
	
_clearSuccessfulDrive:	
	opt	stack 4
; Regs used in _clearSuccessfulDrive: []
	line	189
	
l10385:	
;drive.c: 189: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	190
	
l5842:	
	return
	opt stack 0
GLOBAL	__end_of_clearSuccessfulDrive
	__end_of_clearSuccessfulDrive:
;; =============== function _clearSuccessfulDrive ends ============

	signat	_clearSuccessfulDrive,88
	global	_ser_init
psect	text1289,local,class=CODE,delta=2
global __ptext1289
__ptext1289:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
psect	text1289
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 5
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l10359:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l10361:	
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
	
l10363:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l10365:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l10367:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l10369:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l10371:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l10373:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l10375:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l10377:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l10379:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l10381:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l10383:	
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
psect	text1290,local,class=CODE,delta=2
global __ptext1290
__ptext1290:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
psect	text1290
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 3
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l10311:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u3551
	goto	u3550
u3551:
	goto	l10319
u3550:
	line	50
	
l10313:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l10315:	
;ser.c: 52: return 0;
	clrc
	
	goto	l3633
	
l10317:	
	goto	l3633
	line	53
	
l3632:	
	line	54
	
l10319:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u3561
	goto	u3560
u3561:
	goto	l10323
u3560:
	
l10321:	
	clrc
	
	goto	l3633
	
l10147:	
	
l10323:	
	setc
	
	goto	l3633
	
l10149:	
	goto	l3633
	
l10325:	
	line	55
	
l3633:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_getVictimZone
psect	text1291,local,class=CODE,delta=2
global __ptext1291
__ptext1291:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
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
psect	text1291
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
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
	
l10263:	
;map.c: 163: switch (victimX)
	goto	l10305
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l2900:	
	line	166
;map.c: 166: switch (victimY)
	goto	l10271
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l2902:	
	line	169
	
l10265:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	170
;map.c: 170: break;
	goto	l10307
	line	171
;map.c: 171: case 1:
	
l2904:	
	line	172
	
l10267:	
;map.c: 172: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	173
;map.c: 173: break;
	goto	l10307
	line	178
;map.c: 178: default:
	
l2905:	
	line	179
;map.c: 179: break;
	goto	l10307
	line	180
	
l10269:	
;map.c: 180: }
	goto	l10307
	line	166
	
l2901:	
	
l10271:	
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
	goto	l10265
	xorlw	1^0	; case 1
	skipnz
	goto	l10267
	goto	l10307
	opt asmopt_on

	line	180
	
l2903:	
	line	181
;map.c: 181: break;
	goto	l10307
	line	183
;map.c: 183: case 1:
	
l2907:	
	line	184
;map.c: 184: switch (victimY)
	goto	l10279
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l2909:	
	line	187
	
l10273:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	188
;map.c: 188: break;
	goto	l10307
	line	189
;map.c: 189: case 1:
	
l2911:	
	line	190
	
l10275:	
;map.c: 190: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	191
;map.c: 191: break;
	goto	l10307
	line	196
;map.c: 196: default:
	
l2912:	
	line	197
;map.c: 197: break;
	goto	l10307
	line	198
	
l10277:	
;map.c: 198: }
	goto	l10307
	line	184
	
l2908:	
	
l10279:	
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
	goto	l10273
	xorlw	1^0	; case 1
	skipnz
	goto	l10275
	goto	l10307
	opt asmopt_on

	line	198
	
l2910:	
	line	199
;map.c: 199: break;
	goto	l10307
	line	201
;map.c: 201: case 2:
	
l2913:	
	line	202
;map.c: 202: switch (victimY)
	goto	l10287
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l2915:	
	line	207
	
l10281:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	208
;map.c: 208: break;
	goto	l10307
	line	211
;map.c: 211: case 3:
	
l2917:	
	line	212
	
l10283:	
;map.c: 212: vicZone = 1;
	clrf	(_vicZone)
	bsf	status,0
	rlf	(_vicZone),f
	line	213
;map.c: 213: break;
	goto	l10307
	line	214
;map.c: 214: default:
	
l2918:	
	line	215
;map.c: 215: break;
	goto	l10307
	line	216
	
l10285:	
;map.c: 216: }
	goto	l10307
	line	202
	
l2914:	
	
l10287:	
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
	goto	l10281
	xorlw	3^1	; case 3
	skipnz
	goto	l10283
	goto	l10307
	opt asmopt_on

	line	216
	
l2916:	
	line	217
;map.c: 217: break;
	goto	l10307
	line	219
;map.c: 219: case 3:
	
l2919:	
	line	220
;map.c: 220: switch (victimY)
	goto	l10295
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l2921:	
	line	225
	
l10289:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	226
;map.c: 226: break;
	goto	l10307
	line	229
;map.c: 229: case 3:
	
l2923:	
	line	230
	
l10291:	
;map.c: 230: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	231
;map.c: 231: break;
	goto	l10307
	line	232
;map.c: 232: default:
	
l2924:	
	line	233
;map.c: 233: break;
	goto	l10307
	line	234
	
l10293:	
;map.c: 234: }
	goto	l10307
	line	220
	
l2920:	
	
l10295:	
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
	goto	l10289
	xorlw	3^1	; case 3
	skipnz
	goto	l10291
	goto	l10307
	opt asmopt_on

	line	234
	
l2922:	
	line	235
;map.c: 235: break;
	goto	l10307
	line	237
;map.c: 237: case 4:
	
l2925:	
	line	238
;map.c: 238: switch (victimY)
	goto	l10301
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l2927:	
	line	247
	
l10297:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	248
;map.c: 248: break;
	goto	l10307
	line	249
;map.c: 249: default:
	
l2929:	
	line	250
;map.c: 250: break;
	goto	l10307
	line	251
	
l10299:	
;map.c: 251: }
	goto	l10307
	line	238
	
l2926:	
	
l10301:	
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
	goto	l10297
	goto	l10307
	opt asmopt_on

	line	251
	
l2928:	
	line	252
;map.c: 252: break;
	goto	l10307
	line	254
;map.c: 254: default:
	
l2930:	
	line	255
;map.c: 255: break;
	goto	l10307
	line	256
	
l10303:	
;map.c: 256: }
	goto	l10307
	line	163
	
l2899:	
	
l10305:	
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
	goto	l10271
	xorlw	1^0	; case 1
	skipnz
	goto	l10279
	xorlw	2^1	; case 2
	skipnz
	goto	l10287
	xorlw	3^2	; case 3
	skipnz
	goto	l10295
	xorlw	4^3	; case 4
	skipnz
	goto	l10301
	goto	l10307
	opt asmopt_on

	line	256
	
l2906:	
	line	258
	
l10307:	
;map.c: 258: return vicZone;
	movf	(_vicZone),w
	goto	l2931
	
l10309:	
	line	259
	
l2931:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text1292,local,class=CODE,delta=2
global __ptext1292
__ptext1292:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
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
psect	text1292
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 5
; Regs used in _getFinalY: [wreg]
	line	153
	
l10259:	
;map.c: 153: return finalY;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	goto	l2896
	
l10261:	
	line	154
	
l2896:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text1293,local,class=CODE,delta=2
global __ptext1293
__ptext1293:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
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
psect	text1293
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 5
; Regs used in _getFinalX: [wreg]
	line	148
	
l10255:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l2893
	
l10257:	
	line	149
	
l2893:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_putch
psect	text1294,local,class=CODE,delta=2
global __ptext1294
__ptext1294:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
psect	text1294
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\ser.c"
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
	
l10227:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l10229
	
l3649:	
	line	83
;ser.c: 83: continue;
	goto	l10229
	
l3648:	
	line	82
	
l10229:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u3521
	goto	u3520
u3521:
	goto	l10229
u3520:
	
l3650:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l10231:	
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
	
l10233:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l10235:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l10237:	
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
psect	text1295,local,class=CODE,delta=2
global __ptext1295
__ptext1295:

;; *************** function _initEEPROMMode *****************
;; Defined at:
;;		line 21 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
psect	text1295
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
	line	21
	global	__size_of_initEEPROMMode
	__size_of_initEEPROMMode	equ	__end_of_initEEPROMMode-_initEEPROMMode
	
_initEEPROMMode:	
	opt	stack 3
; Regs used in _initEEPROMMode: [wreg+status,2+status,0]
	line	22
	
l10159:	
;eeprom.c: 22: PORTC &= 0b11111100;
	movlw	(0FCh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_initEEPROMMode+0)+0
	movf	(??_initEEPROMMode+0)+0,w
	andwf	(7),f	;volatile
	line	23
	
l10161:	
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
psect	text1296,local,class=CODE,delta=2
global __ptext1296
__ptext1296:

;; *************** function _writeSPIByte *****************
;; Defined at:
;;		line 14 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
psect	text1296
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\eeprom.c"
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
	
l10155:	
;eeprom.c: 15: SSPIF = 0;
	bcf	(99/8),(99)&7
	line	16
	
l10157:	
;eeprom.c: 16: SSPBUF = data;
	movf	(writeSPIByte@data),w
	movwf	(19)	;volatile
	line	17
;eeprom.c: 17: while(!SSPIF);
	goto	l1401
	
l1402:	
	
l1401:	
	btfss	(99/8),(99)&7
	goto	u3451
	goto	u3450
u3451:
	goto	l1401
u3450:
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
psect	text1297,local,class=CODE,delta=2
global __ptext1297
__ptext1297:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 62 in file "C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
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
psect	text1297
	file	"C:\Users\11014065\Desktop\30 May WORKING FILE\main.c"
	line	62
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
psect	text1297
	line	64
	
i1l10433:	
;main.c: 64: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u367_21
	goto	u367_20
u367_21:
	goto	i1l6708
u367_20:
	line	66
	
i1l10435:	
;main.c: 65: {
;main.c: 66: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	67
	
i1l10437:	
;main.c: 67: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	80
	
i1l10439:	
;main.c: 80: if(!RB0)
	btfsc	(48/8),(48)&7
	goto	u368_21
	goto	u368_20
u368_21:
	goto	i1l6701
u368_20:
	line	82
	
i1l10441:	
;main.c: 81: {
;main.c: 82: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	83
	
i1l10443:	
;main.c: 83: if(start.debounceCount >= 10 & start.released)
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
	goto	u369_21
	goto	u369_20
u369_21:
	goto	i1l10451
u369_20:
	line	85
	
i1l10445:	
;main.c: 84: {
;main.c: 85: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	86
	
i1l10447:	
;main.c: 86: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l10451
	line	87
	
i1l6702:	
	line	88
;main.c: 87: }
;main.c: 88: }
	goto	i1l10451
	line	89
	
i1l6701:	
	line	91
;main.c: 89: else
;main.c: 90: {
;main.c: 91: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	92
	
i1l10449:	
;main.c: 92: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l10451
	line	93
	
i1l6703:	
	line	95
	
i1l10451:	
;main.c: 93: }
;main.c: 95: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u370_21
	goto	u370_20
u370_21:
	goto	i1l10461
u370_20:
	
i1l10453:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	
i1l10455:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l10457:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u371_21
	goto	u371_20
u371_21:
	goto	i1l10461
u371_20:
	
i1l10459:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l10461
	
i1l6705:	
	goto	i1l10461
	
i1l6704:	
	
i1l10461:	
	btfss	(100/8),(100)&7
	goto	u372_21
	goto	u372_20
u372_21:
	goto	i1l6708
u372_20:
	
i1l10463:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u373_21
	goto	u373_20
u373_21:
	goto	i1l6708
u373_20:
	
i1l10465:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	movwf	(25)	;volatile
	
i1l10467:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l10469:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l10471:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u374_21
	goto	u374_20
u374_21:
	goto	i1l6708
u374_20:
	
i1l10473:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l6708
	
i1l6707:	
	goto	i1l6708
	
i1l6706:	
	goto	i1l6708
	line	96
	
i1l6700:	
	line	97
	
i1l6708:	
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
psect	text1298,local,class=CODE,delta=2
global __ptext1298
__ptext1298:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

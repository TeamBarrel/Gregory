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
# 21 "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_findWalls
	FNCALL	_main,_goParallel
	FNCALL	_main,_rotateIR
	FNCALL	_main,_goToNextCell
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
	FNCALL	_goForward,_driveForDistance
	FNCALL	_goBackward,_lcd_set_cursor
	FNCALL	_goBackward,_lcd_write_data
	FNCALL	_goBackward,_turnAround
	FNCALL	_goBackward,_updateOrientation
	FNCALL	_goBackward,_driveForDistance
	FNCALL	_goParallel,_readIR
	FNCALL	_goParallel,_rotateIR
	FNCALL	_goParallel,_play_iCreate_song
	FNCALL	_goParallel,___lbtoft
	FNCALL	_goParallel,___ftmul
	FNCALL	_goParallel,___fttol
	FNCALL	_goParallel,_drive
	FNCALL	_goParallel,_waitFor
	FNCALL	_findWall,_readIR
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
	FNCALL	_lcd_write_string,_lcd_write_data
	FNCALL	_lcd_set_cursor,_lcd_write_control
	FNCALL	_adc_read_channel,_adc_read
	FNCALL	___lbtoft,___ftpack
	FNCALL	___ftmul,___ftpack
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
	global	_somethingInTheWay
	global	_xCoord
	global	_yCoord
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
	global	_beep
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	14

;initializer for _somethingInTheWay
	retlw	02h
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	50

;initializer for _xCoord
	retlw	01h
	line	51

;initializer for _yCoord
	retlw	03h
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
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
	global	_RTC_Counter
	global	_currentOrientation
	global	_finalX
	global	_finalY
	global	_lastMove
	global	_ser_tmp
	global	_stepPosition
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
	global	_rightWall
	global	_successfulDrive
	global	_rxfifo
	global	_txfifo
	global	_start
	global	_closestObject
	global	_node
	global	_stepsToPerpendicular
	global	_vicZone
	global	_victimZone
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
	
STR_1:	
	retlw	40	;'('
	retlw	45	;'-'
	retlw	44	;','
	retlw	45	;'-'
	retlw	41	;')'
	retlw	32	;' '
	retlw	69	;'E'
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
	
STR_2:	
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	32	;' '
	retlw	40	;'('
	retlw	48	;'0'
	retlw	44	;','
	retlw	48	;'0'
	retlw	41	;')'
	retlw	32	;' '
	retlw	71	;'G'
	retlw	82	;'R'
	retlw	69	;'E'
	retlw	71	;'G'
	retlw	0
psect	strings
	file	"COMPv0.4.as"
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

_currentOrientation:
       ds      1

_finalX:
       ds      1

_finalY:
       ds      1

_lastMove:
       ds      1

_ser_tmp:
       ds      1

_stepPosition:
       ds      1

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
_somethingInTheWay:
       ds      1

psect	dataBANK0
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	50
_xCoord:
       ds      1

psect	dataBANK0
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	51
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

_stepsToPerpendicular:
       ds      1

_vicZone:
       ds      1

_victimZone:
       ds      1

_wayWent:
       ds      1

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK1
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
	line	14
_beep:
       ds      5

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
	line	12
_finalCountdown:
       ds      27

psect	dataBANK3
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
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
	clrf	((__pbssCOMMON)+2)&07Fh
	clrf	((__pbssCOMMON)+3)&07Fh
; Clear objects allocated to BANK0
psect cinit,class=CODE,delta=2
	bcf	status, 7	;select IRP bank0
	movlw	low(__pbssBANK0)
	movwf	fsr
	movlw	low((__pbssBANK0)+08h)
	fcall	clear_ram
; Clear objects allocated to BANK1
psect cinit,class=CODE,delta=2
	movlw	low(__pbssBANK1)
	movwf	fsr
	movlw	low((__pbssBANK1)+02Ah)
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
	movlw low(__pdataBANK1+26)
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
	global	goParallel@stepsToWall
goParallel@stepsToWall:	; 1 bytes @ 0x0
	ds	1
	global	goParallel@shortestDistance
goParallel@shortestDistance:	; 2 bytes @ 0x1
	ds	2
	global	goParallel@angleHighByte
goParallel@angleHighByte:	; 1 bytes @ 0x3
	ds	1
	global	goParallel@angleLowByte
goParallel@angleLowByte:	; 1 bytes @ 0x4
	ds	1
	global	goParallel@distance
goParallel@distance:	; 2 bytes @ 0x5
	ds	2
	global	goParallel@step
goParallel@step:	; 2 bytes @ 0x7
	ds	2
	global	goParallel@angleParallelToWall
goParallel@angleParallelToWall:	; 2 bytes @ 0x9
	ds	2
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
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
	global	?_lcd_write_string
?_lcd_write_string:	; 0 bytes @ 0x0
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
	global	?___wmul
?___wmul:	; 2 bytes @ 0xA
	global	?___ftpack
?___ftpack:	; 3 bytes @ 0xA
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	global	___ftpack@arg
___ftpack@arg:	; 3 bytes @ 0xA
	ds	1
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
	global	??_lcd_set_cursor
??_lcd_set_cursor:	; 0 bytes @ 0xD
	global	??_lcd_write_string
??_lcd_write_string:	; 0 bytes @ 0xD
	global	??_lcd_write_1_digit_bcd
??_lcd_write_1_digit_bcd:	; 0 bytes @ 0xD
	global	??_lcd_init
??_lcd_init:	; 0 bytes @ 0xD
	global	lcd_set_cursor@address
lcd_set_cursor@address:	; 1 bytes @ 0xD
	global	lcd_write_1_digit_bcd@data
lcd_write_1_digit_bcd@data:	; 1 bytes @ 0xD
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
	global	rotateIR@steps
rotateIR@steps:	; 1 bytes @ 0xE
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
	global	rotateIR@stepNum
rotateIR@stepNum:	; 1 bytes @ 0xF
	global	lookForVictim@victim
lookForVictim@victim:	; 1 bytes @ 0xF
	ds	1
	global	??_findFinalDestination
??_findFinalDestination:	; 0 bytes @ 0x10
	global	??_ser_putArr
??_ser_putArr:	; 0 bytes @ 0x10
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x10
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
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x2D
	ds	1
	global	??___ftmul
??___ftmul:	; 0 bytes @ 0x2E
	ds	4
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
	global	??_goParallel
??_goParallel:	; 0 bytes @ 0x38
	ds	2
	global	??_main
??_main:	; 0 bytes @ 0x3A
	ds	1
;;Data sizes: Strings 34, constant 0, data 110, bss 54, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     59      70
;; BANK1           80     11      79
;; BANK3           96      0      81
;; BANK2           96      0       0

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
;;		 -> beep(BANK1[5]), champions(BANK1[21]), lookingForU2(BANK3[29]), superMarioBros(BANK3[25]), 
;;		 -> finalCountdown(BANK3[27]), 
;;
;; lcd_write_string@s	PTR const unsigned char  size(1) Largest target is 17
;;		 -> STR_2(CODE[17]), STR_1(CODE[17]), 
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
;;   _main->_goParallel
;;   _goToNextCell->_goRight
;;   _goToNextCell->_goBackward
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _goParallel->___ftmul
;;   _findWall->_readIR
;;   _driveForDistance->_goReverse
;;   _driveForDistance->_turnRight90
;;   _driveForDistance->_turnLeft90
;;   _updateLocation->_lcd_set_cursor
;;   _updateLocation->_lcd_write_1_digit_bcd
;;   _lookForVictim->_lcd_set_cursor
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
;;   _lcd_write_string->_lcd_write_data
;;   _lcd_set_cursor->_lcd_write_control
;;   _adc_read_channel->_convert
;;   ___lbtoft->___fttol
;;   ___ftmul->___lbtoft
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
;; (0) _main                                                 1     1      0   16335
;;                                             58 BANK0      1     1      0
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                          _findWalls
;;                         _goParallel
;;                           _rotateIR
;;                       _goToNextCell
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
;; (1) _findWalls                                            1     1      0    1785
;;                                             45 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                           _rotateIR
;;                  _play_iCreate_song
;; ---------------------------------------------------------------------------------
;; (2) _goRight                                              1     1      0    2370
;;                                             34 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goLeft                                               0     0      0    2370
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goForward                                            0     0      0    2060
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
;; (1) _goParallel                                          13    13      0    3952
;;                                             56 BANK0      2     2      0
;;                                              0 BANK1     11    11      0
;;                             _readIR
;;                           _rotateIR
;;                  _play_iCreate_song
;;                           ___lbtoft
;;                            ___ftmul
;;                            ___fttol
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1528
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (3) _driveForDistance                                    12    10      2    1964
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
;; (1) _lookForVictim                                        2     2      0     257
;;                                             14 BANK0      2     2      0
;;                          _ser_putch
;;                          _ser_getch
;;                  _play_iCreate_song
;;                     _lcd_set_cursor
;;                     _lcd_write_data
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
;; (4) _goReverse                                            3     3      0     375
;;                                             19 BANK0      3     3      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _readIR                                               4     2      2    1528
;;                                             41 BANK0      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (4) _findFinalDestination                                 4     2      2     406
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
;; (4) _turnLeft90                                           3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (4) _turnRight90                                          3     3      0     279
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
;; (1) _lcd_write_string                                     2     2      0      96
;;                                             13 BANK0      2     2      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _lcd_set_cursor                                       1     1      0      65
;;                                             13 BANK0      1     1      0
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (3) _adc_read_channel                                     4     2      2     510
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
;; (2) _initIRobot                                           3     3      0      31
;;                                             12 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (5) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _drive                                                7     4      3     155
;;                                             12 BANK0      7     4      3
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (1) _rotateIR                                             6     5      1      99
;;                                             10 BANK0      6     5      1
;; ---------------------------------------------------------------------------------
;; (3) _convert                                              4     2      2     984
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
;; (4) _ser_getch                                            2     2      0      34
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
;; (4) _adc_read                                             8     6      2     479
;;                                             25 BANK0      8     6      2
;;                            ___awdiv
;; ---------------------------------------------------------------------------------
;; (4) ___awdiv                                              9     5      4     445
;;                                             16 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (2) ___fttol                                             14    10      4     371
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
;; (4) _getCurrentY                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (4) _getCurrentX                                          0     0      0       0
;; ---------------------------------------------------------------------------------
;; (4) _updateOrientation                                    2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (2) _ser_init                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (3) _ser_putch                                            2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (5) _ser_isrx                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getFinalY                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getFinalX                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 5
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
;;     _play_iCreate_song
;;       _ser_putch
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
;;     ___fttol
;;       ___ftpack (ARG)
;;     _drive
;;       _ser_putch
;;     _waitFor
;;       _ser_putch
;;   _rotateIR
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
;;BANK1               50      B      4F       7       98.8%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      FC      12        0.0%
;;ABS                  0      0      F2       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       A       2        0.0%
;;BANK0               50     3B      46       5       87.5%
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
;;		line 339 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_init
;;		_drive
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_goParallel
;;		_rotateIR
;;		_goToNextCell
;;		_getSuccessfulDrive
;;		_updateLocation
;;		_updateNode
;;		_checkIfHome
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	339
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	340
	
l11941:	
;main.c: 340: init();
	fcall	_init
	line	341
;main.c: 341: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	343
	
l11943:	
;main.c: 343: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	344
	
l11945:	
;main.c: 344: lcd_write_string("(-,-) E -- --- -");
	movlw	((STR_1-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	345
;main.c: 345: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	346
	
l11947:	
;main.c: 346: lcd_write_string("- - - (0,0) GREG");
	movlw	((STR_2-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	350
;main.c: 350: while(!home)
	goto	l11973
	
l6023:	
	line	353
	
l11949:	
;main.c: 351: {
;main.c: 353: if(start.pressed)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(_start)^080h,w
	skipz
	goto	u6780
	goto	l11973
u6780:
	line	355
	
l11951:	
;main.c: 354: {
;main.c: 355: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	356
;main.c: 356: lookForVictim();
	fcall	_lookForVictim
	line	357
	
l11953:	
;main.c: 357: findWalls();
	fcall	_findWalls
	line	358
	
l11955:	
;main.c: 358: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u6791
	goto	u6790
u6791:
	goto	l11959
u6790:
	line	359
	
l11957:	
;main.c: 359: goParallel();
	fcall	_goParallel
	goto	l11961
	line	360
	
l6025:	
	line	361
	
l11959:	
;main.c: 360: else
;main.c: 361: rotateIR(12, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_main+0)+0
	movf	(??_main+0)+0,w
	movwf	(?_rotateIR)
	movlw	(0Ch)
	fcall	_rotateIR
	goto	l11961
	
l6026:	
	line	365
	
l11961:	
;main.c: 365: goToNextCell();
	fcall	_goToNextCell
	line	380
	
l11963:	
;main.c: 380: if(getSuccessfulDrive())
	fcall	_getSuccessfulDrive
	btfss	status,0
	goto	u6801
	goto	u6800
u6801:
	goto	l11973
u6800:
	line	382
	
l11965:	
;main.c: 381: {
;main.c: 382: updateLocation();
	fcall	_updateLocation
	line	383
	
l11967:	
;main.c: 383: updateNode();
	fcall	_updateNode
	line	384
	
l11969:	
;main.c: 384: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u6811
	goto	u6810
u6811:
	goto	l11973
u6810:
	line	385
	
l11971:	
;main.c: 385: checkIfHome();
	fcall	_checkIfHome
	goto	l11973
	
l6028:	
	goto	l11973
	line	386
	
l6027:	
	goto	l11973
	line	387
	
l6024:	
	goto	l11973
	line	388
	
l6022:	
	line	350
	
l11973:	
	btfss	(_home/8),(_home)&7
	goto	u6821
	goto	u6820
u6821:
	goto	l11949
u6820:
	goto	l6030
	
l6029:	
	line	390
	
l6030:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text1873,local,class=CODE,delta=2
global __ptext1873
__ptext1873:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 272 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1873
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	272
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 0
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	273
	
l11925:	
;main.c: 273: if(!leftWall && (getSomethingInTheWay() != LEFT))
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u6721
	goto	u6720
u6721:
	goto	l5990
u6720:
	
l11927:	
	fcall	_getSomethingInTheWay
	xorlw	01h
	skipnz
	goto	u6731
	goto	u6730
u6731:
	goto	l5990
u6730:
	line	274
	
l11929:	
;main.c: 274: goLeft();
	fcall	_goLeft
	goto	l5996
	line	275
	
l5990:	
;main.c: 275: else if(!frontWall && (getSomethingInTheWay() != FORWARD))
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u6741
	goto	u6740
u6741:
	goto	l5992
u6740:
	
l11931:	
	fcall	_getSomethingInTheWay
	xorlw	0
	skipnz
	goto	u6751
	goto	u6750
u6751:
	goto	l5992
u6750:
	line	276
	
l11933:	
;main.c: 276: goForward();
	fcall	_goForward
	goto	l5996
	line	277
	
l5992:	
;main.c: 277: else if(!rightWall && (getSomethingInTheWay() != RIGHT))
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u6761
	goto	u6760
u6761:
	goto	l11939
u6760:
	
l11935:	
	fcall	_getSomethingInTheWay
	xorlw	03h
	skipnz
	goto	u6771
	goto	u6770
u6771:
	goto	l11939
u6770:
	line	278
	
l11937:	
;main.c: 278: goRight();
	fcall	_goRight
	goto	l5996
	line	279
	
l5994:	
	line	280
	
l11939:	
;main.c: 279: else
;main.c: 280: goBackward();
	fcall	_goBackward
	goto	l5996
	
l5995:	
	goto	l5996
	
l5993:	
	goto	l5996
	
l5991:	
	line	281
	
l5996:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1874,local,class=CODE,delta=2
global __ptext1874
__ptext1874:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 174 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1874
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	174
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	175
	
l11893:	
;main.c: 175: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	177
	
l11895:	
;main.c: 177: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u6631
	goto	u6630
	
u6631:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u6644
u6630:
	bcf	(_leftWall/8),(_leftWall)&7
u6644:
	line	178
	
l11897:	
;main.c: 178: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u6651
	goto	u6650
u6651:
	goto	l11901
u6650:
	line	180
	
l11899:	
;main.c: 179: {
;main.c: 180: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	181
;main.c: 181: }
	goto	l5974
	line	182
	
l5973:	
	line	183
	
l11901:	
;main.c: 182: else
;main.c: 183: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l5974:	
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
	
l11903:	
;main.c: 185: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	186
	
l11905:	
;main.c: 186: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u6661
	goto	u6660
	
u6661:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u6674
u6660:
	bcf	(_frontWall/8),(_frontWall)&7
u6674:
	line	187
	
l11907:	
;main.c: 187: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u6681
	goto	u6680
u6681:
	goto	l11911
u6680:
	line	189
	
l11909:	
;main.c: 188: {
;main.c: 189: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	191
;main.c: 191: }
	goto	l5976
	line	192
	
l5975:	
	line	193
	
l11911:	
;main.c: 192: else
;main.c: 193: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l5976:	
	line	194
;main.c: 194: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	195
	
l11913:	
;main.c: 195: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	196
	
l11915:	
;main.c: 196: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u6691
	goto	u6690
	
u6691:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u6704
u6690:
	bcf	(_rightWall/8),(_rightWall)&7
u6704:
	line	197
	
l11917:	
;main.c: 197: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u6711
	goto	u6710
u6711:
	goto	l11921
u6710:
	line	199
	
l11919:	
;main.c: 198: {
;main.c: 199: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	201
;main.c: 201: }
	goto	l5978
	line	202
	
l5977:	
	line	203
	
l11921:	
;main.c: 202: else
;main.c: 203: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l5978:	
	line	204
;main.c: 204: rotateIR(36, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_findWalls+0)+0
	movf	(??_findWalls+0)+0,w
	movwf	(?_rotateIR)
	movlw	(024h)
	fcall	_rotateIR
	line	205
	
l11923:	
;main.c: 205: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	206
	
l5979:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text1875,local,class=CODE,delta=2
global __ptext1875
__ptext1875:

;; *************** function _goRight *****************
;; Defined at:
;;		line 198 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1875
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	198
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 0
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	199
	
l11883:	
;drive.c: 199: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	200
;drive.c: 200: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	201
	
l11885:	
;drive.c: 201: turnRight90();
	fcall	_turnRight90
	line	202
	
l11887:	
;drive.c: 202: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	203
	
l11889:	
;drive.c: 203: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	204
	
l11891:	
;drive.c: 204: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	205
	
l5129:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1876,local,class=CODE,delta=2
global __ptext1876
__ptext1876:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 177 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1876
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	177
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 0
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	178
	
l11873:	
;drive.c: 178: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	179
;drive.c: 179: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	180
	
l11875:	
;drive.c: 180: turnLeft90();
	fcall	_turnLeft90
	line	181
	
l11877:	
;drive.c: 181: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	182
	
l11879:	
;drive.c: 182: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	183
	
l11881:	
;drive.c: 183: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	184
	
l5123:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1877,local,class=CODE,delta=2
global __ptext1877
__ptext1877:

;; *************** function _goForward *****************
;; Defined at:
;;		line 168 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1877
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	168
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 0
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	169
	
l11867:	
;drive.c: 169: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	170
;drive.c: 170: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	171
	
l11869:	
;drive.c: 171: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	172
	
l11871:	
;drive.c: 172: driveForDistance(1000);
	movlw	low(03E8h)
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	173
	
l5120:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1878,local,class=CODE,delta=2
global __ptext1878
__ptext1878:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 157 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1878
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	157
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 0
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	158
	
l11857:	
;drive.c: 158: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	159
;drive.c: 159: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	160
	
l11859:	
;drive.c: 160: turnAround();
	fcall	_turnAround
	line	161
	
l11861:	
;drive.c: 161: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	162
	
l11863:	
;drive.c: 162: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	163
	
l11865:	
;drive.c: 163: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	164
	
l5117:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_goParallel
psect	text1879,local,class=CODE,delta=2
global __ptext1879
__ptext1879:

;; *************** function _goParallel *****************
;; Defined at:
;;		line 209 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  step            2    7[BANK1 ] int 
;;  angleParalle    2    9[BANK1 ] int 
;;  distance        2    5[BANK1 ] int 
;;  shortestDist    2    1[BANK1 ] int 
;;  angleLowByte    1    4[BANK1 ] unsigned char 
;;  angleHighByt    1    3[BANK1 ] unsigned char 
;;  stepsToWall     1    0[BANK1 ] unsigned char 
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
;;      Temps:          0       2       0       0       0
;;      Totals:         0       2      11       0       0
;;Total ram usage:       13 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_readIR
;;		_rotateIR
;;		_play_iCreate_song
;;		___lbtoft
;;		___ftmul
;;		___fttol
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1879
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	209
	global	__size_of_goParallel
	__size_of_goParallel	equ	__end_of_goParallel-_goParallel
	
_goParallel:	
	opt	stack 1
; Regs used in _goParallel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	210
	
l11807:	
;main.c: 210: PORTC |= 0b00000011;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	iorwf	(7),f	;volatile
	line	212
	
l11809:	
;main.c: 212: int distance, shortestDistance = 999;
	movlw	low(03E7h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@shortestDistance)^080h
	movlw	high(03E7h)
	movwf	((goParallel@shortestDistance)^080h)+1
	line	215
	
l11811:	
;main.c: 213: char stepsToWall;
;main.c: 215: for (int step = -12; step < 12; step++)
	movlw	low(-12)
	movwf	(goParallel@step)^080h
	movlw	high(-12)
	movwf	((goParallel@step)^080h)+1
	
l11813:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Ch))^80h
	subwf	btemp+1,w
	skipz
	goto	u6585
	movlw	low(0Ch)
	subwf	(goParallel@step)^080h,w
u6585:

	skipc
	goto	u6581
	goto	u6580
u6581:
	goto	l11817
u6580:
	goto	l11831
	
l11815:	
	goto	l11831
	line	216
	
l5982:	
	line	217
	
l11817:	
;main.c: 216: {
;main.c: 217: distance = readIR();
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

	line	218
	
l11819:	
;main.c: 218: if(distance < shortestDistance)
	movf	(goParallel@distance+1)^080h,w
	xorlw	80h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@shortestDistance+1)^080h,w
	xorlw	80h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(??_goParallel+0)+0,w
	skipz
	goto	u6595
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@shortestDistance)^080h,w
	subwf	(goParallel@distance)^080h,w
u6595:

	skipnc
	goto	u6591
	goto	u6590
u6591:
	goto	l11825
u6590:
	line	220
	
l11821:	
;main.c: 219: {
;main.c: 220: stepsToWall = step;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@step)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@stepsToWall)^080h
	line	221
	
l11823:	
;main.c: 221: shortestDistance = distance;
	movf	(goParallel@distance+1)^080h,w
	clrf	(goParallel@shortestDistance+1)^080h
	addwf	(goParallel@shortestDistance+1)^080h
	movf	(goParallel@distance)^080h,w
	clrf	(goParallel@shortestDistance)^080h
	addwf	(goParallel@shortestDistance)^080h

	goto	l11825
	line	222
	
l5984:	
	line	223
	
l11825:	
;main.c: 222: }
;main.c: 223: rotateIR(1, 0b00001101);
	movlw	(0Dh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	movwf	(?_rotateIR)
	movlw	(01h)
	fcall	_rotateIR
	line	215
	
l11827:	
	movlw	low(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(goParallel@step)^080h,f
	skipnc
	incf	(goParallel@step+1)^080h,f
	movlw	high(01h)
	addwf	(goParallel@step+1)^080h,f
	
l11829:	
	movf	(goParallel@step+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0Ch))^80h
	subwf	btemp+1,w
	skipz
	goto	u6605
	movlw	low(0Ch)
	subwf	(goParallel@step)^080h,w
u6605:

	skipc
	goto	u6601
	goto	u6600
u6601:
	goto	l11817
u6600:
	goto	l11831
	
l5983:	
	line	225
	
l11831:	
;main.c: 224: }
;main.c: 225: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	226
	
l11833:	
;main.c: 226: rotateIR(12, 0b00001111);
	movlw	(0Fh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	movwf	(?_rotateIR)
	movlw	(0Ch)
	fcall	_rotateIR
	line	227
	
l11835:	
;main.c: 227: play_iCreate_song(5);
	movlw	(05h)
	fcall	_play_iCreate_song
	line	229
;main.c: 229: int angleParallelToWall = (int)((stepsToWall*3.75)*.944);
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
	movlw	0x8f
	movwf	(?___ftmul)
	movlw	0x62
	movwf	(?___ftmul+1)
	movlw	0x40
	movwf	(?___ftmul+2)
	fcall	___ftmul
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?___ftmul)),w
	movwf	(?___fttol)
	movf	(1+(?___ftmul)),w
	movwf	(?___fttol+1)
	movf	(2+(?___ftmul)),w
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

	line	230
	
l11837:	
;main.c: 230: char angleHighByte = 0;
	clrf	(goParallel@angleHighByte)^080h
	line	231
	
l11839:	
;main.c: 231: char angleLowByte = (char) angleParallelToWall;
	movf	(goParallel@angleParallelToWall)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@angleLowByte)^080h
	line	233
	
l11841:	
;main.c: 233: if(angleParallelToWall < 0)
	btfss	(goParallel@angleParallelToWall+1)^080h,7
	goto	u6611
	goto	u6610
u6611:
	goto	l11845
u6610:
	line	234
	
l11843:	
;main.c: 234: angleParallelToWall = 360 - angleParallelToWall;
	comf	(goParallel@angleParallelToWall)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	comf	(goParallel@angleParallelToWall+1)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	((??_goParallel+0)+0+1)
	incf	(??_goParallel+0)+0,f
	skipnz
	incf	((??_goParallel+0)+0+1),f
	movf	0+(??_goParallel+0)+0,w
	addlw	low(0168h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@angleParallelToWall)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	1+(??_goParallel+0)+0,w
	skipnc
	addlw	1
	addlw	high(0168h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	1+(goParallel@angleParallelToWall)^080h
	goto	l11845
	
l5985:	
	line	236
	
l11845:	
;main.c: 236: if(angleParallelToWall > 255)
	movf	(goParallel@angleParallelToWall+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6625
	movlw	low(0100h)
	subwf	(goParallel@angleParallelToWall)^080h,w
u6625:

	skipc
	goto	u6621
	goto	u6620
u6621:
	goto	l11851
u6620:
	line	238
	
l11847:	
;main.c: 237: {
;main.c: 238: angleHighByte = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(goParallel@angleHighByte)^080h
	bsf	status,0
	rlf	(goParallel@angleHighByte)^080h,f
	line	239
	
l11849:	
;main.c: 239: angleLowByte = (char)(angleParallelToWall - 255);
	movf	(goParallel@angleParallelToWall)^080h,w
	addlw	01h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(goParallel@angleLowByte)^080h
	goto	l11851
	line	240
	
l5986:	
	line	241
	
l11851:	
;main.c: 240: }
;main.c: 241: drive(0, 25, 0, 1);
	movlw	(019h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	movwf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	bsf	status,0
	rlf	0+(?_drive)+02h,f
	movlw	(0)
	fcall	_drive
	line	242
	
l11853:	
;main.c: 242: waitFor(157,angleHighByte,angleLowByte);
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleHighByte)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+0)+0
	movf	(??_goParallel+0)+0,w
	movwf	(?_waitFor)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(goParallel@angleLowByte)^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goParallel+1)+0
	movf	(??_goParallel+1)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	243
	
l11855:	
;main.c: 243: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	269
	
l5987:	
	return
	opt stack 0
GLOBAL	__end_of_goParallel
	__end_of_goParallel:
;; =============== function _goParallel ends ============

	signat	_goParallel,88
	global	_findWall
psect	text1880,local,class=CODE,delta=2
global __ptext1880
__ptext1880:

;; *************** function _findWall *****************
;; Defined at:
;;		line 396 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1880
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	396
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	397
	
l11795:	
;main.c: 397: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6575
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u6575:

	skipc
	goto	u6571
	goto	u6570
u6571:
	goto	l11803
u6570:
	line	398
	
l11797:	
;main.c: 398: return 0;
	clrc
	
	goto	l6034
	
l11799:	
	goto	l6034
	
l11801:	
	goto	l6034
	line	399
	
l6033:	
	line	400
	
l11803:	
;main.c: 399: else
;main.c: 400: return 1;
	setc
	
	goto	l6034
	
l11805:	
	goto	l6034
	
l6035:	
	line	401
	
l6034:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_driveForDistance
psect	text1881,local,class=CODE,delta=2
global __ptext1881
__ptext1881:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 31 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1881
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	31
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 0
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l11711:	
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
	
l11713:	
;drive.c: 37: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	38
	
l11715:	
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
	
l11717:	
;drive.c: 39: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	41
;drive.c: 41: while(moving)
	goto	l11793
	
l5085:	
	line	43
	
l11719:	
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
	goto	u6455
	movlw	low(064h)
	subwf	(driveForDistance@distance),w
u6455:

	skipc
	goto	u6451
	goto	u6450
u6451:
	goto	l11755
u6450:
	line	46
	
l11721:	
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
	
l11723:	
;drive.c: 49: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6461
	goto	u6460
u6461:
	goto	l11735
u6460:
	line	51
	
l11725:	
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
	
l11727:	
;drive.c: 54: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6471
	goto	u6470
u6471:
	goto	l11735
u6470:
	line	56
	
l11729:	
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
	
l11731:	
;drive.c: 59: if(cliff == 0)
	movf	(driveForDistance@cliff),f
	skipz	;volatile
	goto	u6481
	goto	u6480
u6481:
	goto	l11735
u6480:
	line	61
	
l11733:	
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
	goto	l11735
	line	64
	
l5089:	
	goto	l11735
	line	65
	
l5088:	
	goto	l11735
	line	66
	
l5087:	
	line	67
	
l11735:	
;drive.c: 64: }
;drive.c: 65: }
;drive.c: 66: }
;drive.c: 67: if(cliff == 1)
	movf	(driveForDistance@cliff),w	;volatile
	xorlw	01h
	skipz
	goto	u6491
	goto	u6490
u6491:
	goto	l11755
u6490:
	line	69
	
l11737:	
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
	
l11739:	
;drive.c: 72: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u6501
	goto	u6500
u6501:
	goto	l11747
u6500:
	line	74
	
l11741:	
;drive.c: 73: {
;drive.c: 74: somethingInTheWay = LEFT;
	clrf	(_somethingInTheWay)	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay),f	;volatile
	line	75
	
l11743:	
;drive.c: 75: turnRight90();
	fcall	_turnRight90
	line	76
	
l11745:	
;drive.c: 76: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	77
;drive.c: 77: }
	goto	l5092
	line	78
	
l5091:	
	
l11747:	
;drive.c: 78: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u6511
	goto	u6510
u6511:
	goto	l5093
u6510:
	line	80
	
l11749:	
;drive.c: 79: {
;drive.c: 80: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	line	81
	
l11751:	
;drive.c: 81: turnLeft90();
	fcall	_turnLeft90
	line	82
	
l11753:	
;drive.c: 82: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	83
;drive.c: 83: }
	goto	l5092
	line	84
	
l5093:	
	line	85
;drive.c: 84: else
;drive.c: 85: somethingInTheWay = FORWARD;
	clrf	(_somethingInTheWay)	;volatile
	goto	l5092
	
l5094:	
	
l5092:	
	line	86
;drive.c: 86: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11755
	line	87
	
l5090:	
	goto	l11755
	line	88
	
l5086:	
	line	91
	
l11755:	
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
	
l11757:	
;drive.c: 94: if(virtualWall == 1)
	movf	(driveForDistance@virtualWall),w	;volatile
	xorlw	01h
	skipz
	goto	u6521
	goto	u6520
u6521:
	goto	l11777
u6520:
	line	96
	
l11759:	
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
	
l11761:	
;drive.c: 100: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u6531
	goto	u6530
u6531:
	goto	l11769
u6530:
	line	102
	
l11763:	
;drive.c: 101: {
;drive.c: 102: somethingInTheWay = LEFT;
	clrf	(_somethingInTheWay)	;volatile
	bsf	status,0
	rlf	(_somethingInTheWay),f	;volatile
	line	103
	
l11765:	
;drive.c: 103: turnRight90();
	fcall	_turnRight90
	line	104
	
l11767:	
;drive.c: 104: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	105
;drive.c: 105: }
	goto	l5097
	line	106
	
l5096:	
	
l11769:	
;drive.c: 106: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u6541
	goto	u6540
u6541:
	goto	l5098
u6540:
	line	108
	
l11771:	
;drive.c: 107: {
;drive.c: 108: somethingInTheWay = RIGHT;
	movlw	(03h)
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	line	109
	
l11773:	
;drive.c: 109: turnLeft90();
	fcall	_turnLeft90
	line	110
	
l11775:	
;drive.c: 110: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	111
;drive.c: 111: }
	goto	l5097
	line	112
	
l5098:	
	line	113
;drive.c: 112: else
;drive.c: 113: somethingInTheWay = FORWARD;
	clrf	(_somethingInTheWay)	;volatile
	goto	l5097
	
l5099:	
	
l5097:	
	line	114
;drive.c: 114: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l11777
	line	115
	
l5095:	
	line	118
	
l11777:	
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
	
l11779:	
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
	
l11781:	
;drive.c: 123: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	124
	
l11783:	
;drive.c: 124: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u6555
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u6555:

	skipc
	goto	u6551
	goto	u6550
u6551:
	goto	l11793
u6550:
	line	126
	
l11785:	
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
	
l11787:	
;drive.c: 127: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	line	128
	
l11789:	
;drive.c: 128: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	129
	
l11791:	
;drive.c: 129: somethingInTheWay = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(_somethingInTheWay)	;volatile
	goto	l11793
	line	130
	
l5100:	
	goto	l11793
	line	131
	
l5084:	
	line	41
	
l11793:	
	btfsc	(_moving/8),(_moving)&7
	goto	u6561
	goto	u6560
u6561:
	goto	l11719
u6560:
	goto	l5102
	
l5101:	
	line	132
	
l5102:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_updateLocation
psect	text1882,local,class=CODE,delta=2
global __ptext1882
__ptext1882:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 284 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1882
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	284
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	285
	
l11687:	
;main.c: 285: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	286
;main.c: 286: switch(getOrientation())
	goto	l11707
	line	288
;main.c: 287: {
;main.c: 288: case NORTH:
	
l6000:	
	line	289
	
l11689:	
;main.c: 289: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	290
	
l11691:	
;main.c: 290: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	291
;main.c: 291: break;
	goto	l11709
	line	292
;main.c: 292: case SOUTH:
	
l6002:	
	line	293
	
l11693:	
;main.c: 293: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	294
	
l11695:	
;main.c: 294: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	295
;main.c: 295: break;
	goto	l11709
	line	296
;main.c: 296: case EAST:
	
l6003:	
	line	297
	
l11697:	
;main.c: 297: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	298
	
l11699:	
;main.c: 298: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	299
;main.c: 299: break;
	goto	l11709
	line	300
;main.c: 300: case WEST:
	
l6004:	
	line	301
	
l11701:	
;main.c: 301: --xCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_xCoord),f	;volatile
	line	302
	
l11703:	
;main.c: 302: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	303
;main.c: 303: break;
	goto	l11709
	line	304
;main.c: 304: default:
	
l6005:	
	line	305
;main.c: 305: break;
	goto	l11709
	line	306
	
l11705:	
;main.c: 306: }
	goto	l11709
	line	286
	
l5999:	
	
l11707:	
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
	goto	l11701
	xorlw	1^0	; case 1
	skipnz
	goto	l11693
	xorlw	2^1	; case 2
	skipnz
	goto	l11697
	xorlw	3^2	; case 3
	skipnz
	goto	l11689
	goto	l11709
	opt asmopt_on

	line	306
	
l6001:	
	line	308
	
l11709:	
;main.c: 308: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	309
;main.c: 309: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	310
;main.c: 310: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	311
;main.c: 311: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	312
	
l6006:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text1883,local,class=CODE,delta=2
global __ptext1883
__ptext1883:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 149 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  victim          1   15[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_ser_putch
;;		_ser_getch
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1883
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	149
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 3
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	150
	
l11675:	
;main.c: 150: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	151
;main.c: 151: ser_putch(17);
	movlw	(011h)
	fcall	_ser_putch
	line	152
;main.c: 152: char victim = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lookForVictim+0)+0
	movf	(??_lookForVictim+0)+0,w
	movwf	(lookForVictim@victim)
	line	154
	
l11677:	
;main.c: 154: if(victim == 254)
	movf	(lookForVictim@victim),w
	xorlw	0FEh
	skipz
	goto	u6441
	goto	u6440
u6441:
	goto	l5970
u6440:
	line	158
	
l11679:	
;main.c: 155: {
;main.c: 158: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	159
	
l11681:	
;main.c: 159: victimZone = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_victimZone)^080h	;volatile
	line	160
	
l11683:	
;main.c: 160: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	161
	
l11685:	
;main.c: 161: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	goto	l5970
	line	169
	
l5969:	
	line	170
	
l5970:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text1884,local,class=CODE,delta=2
global __ptext1884
__ptext1884:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 138 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1884
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	138
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 3
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	139
	
l11661:	
;main.c: 139: if(!goingHome && (xCoord == getFinalX()) && (yCoord == getFinalY()))
	btfsc	(_goingHome/8),(_goingHome)&7
	goto	u6411
	goto	u6410
u6411:
	goto	l5966
u6410:
	
l11663:	
	fcall	_getFinalX
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u6421
	goto	u6420
u6421:
	goto	l5966
u6420:
	
l11665:	
	fcall	_getFinalY
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u6431
	goto	u6430
u6431:
	goto	l5966
u6430:
	line	141
	
l11667:	
;main.c: 140: {
;main.c: 141: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	142
	
l11669:	
;main.c: 142: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	143
	
l11671:	
;main.c: 143: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	144
	
l11673:	
;main.c: 144: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l5966
	line	145
	
l5965:	
	line	146
	
l5966:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text1885,local,class=CODE,delta=2
global __ptext1885
__ptext1885:

;; *************** function _init *****************
;; Defined at:
;;		line 101 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1885
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	101
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	102
	
l11629:	
;main.c: 102: start.pressed = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	line	103
	
l11631:	
;main.c: 103: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	line	105
	
l11633:	
;main.c: 105: init_adc();
	fcall	_init_adc
	line	106
	
l11635:	
;main.c: 106: lcd_init();
	fcall	_lcd_init
	line	108
	
l11637:	
;main.c: 108: TRISB = 0b00000001;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	111
	
l11639:	
;main.c: 111: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	113
	
l11641:	
;main.c: 113: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	114
	
l11643:	
;main.c: 114: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	115
	
l11645:	
;main.c: 115: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	116
	
l11647:	
;main.c: 116: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	117
	
l11649:	
;main.c: 117: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	120
	
l11651:	
;main.c: 120: PEIE = 1;
	bsf	(94/8),(94)&7
	line	121
	
l11653:	
;main.c: 121: GIE = 1;
	bsf	(95/8),(95)&7
	line	123
	
l11655:	
;main.c: 123: ser_init();
	fcall	_ser_init
	line	124
	
l11657:	
;main.c: 124: initIRobot();
	fcall	_initIRobot
	line	125
	
l11659:	
;main.c: 125: initSongs();
	fcall	_initSongs
	line	126
	
l5959:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_goReverse
psect	text1886,local,class=CODE,delta=2
global __ptext1886
__ptext1886:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 187 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1886
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	187
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 0
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	188
	
l11619:	
;drive.c: 188: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	189
;drive.c: 189: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	190
	
l11621:	
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
	
l11623:	
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
	
l11625:	
;drive.c: 192: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	193
	
l11627:	
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
u6837:
	decfsz	((??_goReverse+0)+0),f
	goto	u6837
	decfsz	((??_goReverse+0)+0+1),f
	goto	u6837
	decfsz	((??_goReverse+0)+0+2),f
	goto	u6837
opt asmopt_on

	line	194
	
l5126:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_readIR
psect	text1887,local,class=CODE,delta=2
global __ptext1887
__ptext1887:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
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
;;		_goParallel
;;		_findWall
;;		_frontWallCorrect
;;		_scanIR
;; This function uses a non-reentrant model
;;
psect	text1887
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 1
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l11613:	
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
	
l11615:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l4348
	
l11617:	
	line	36
	
l4348:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_findFinalDestination
psect	text1888,local,class=CODE,delta=2
global __ptext1888
__ptext1888:

;; *************** function _findFinalDestination *****************
;; Defined at:
;;		line 12 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
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
psect	text1888
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
	line	12
	global	__size_of_findFinalDestination
	__size_of_findFinalDestination	equ	__end_of_findFinalDestination-_findFinalDestination
	
_findFinalDestination:	
	opt	stack 0
; Regs used in _findFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;findFinalDestination@virtualWallX stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(findFinalDestination@virtualWallX)
	line	13
	
l11533:	
;map.c: 13: switch (virtualWallX)
	goto	l11609
	line	15
;map.c: 14: {
;map.c: 15: case 0:
	
l2116:	
	line	16
;map.c: 16: switch (virtualWallY)
	goto	l11543
	line	20
;map.c: 17: {
;map.c: 20: case 1:
	
l2118:	
	line	21
;map.c: 21: finalX = 0;
	clrf	(_finalX)
	line	22
	
l11535:	
;map.c: 22: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	23
;map.c: 23: break;
	goto	l11611
	line	24
;map.c: 24: case 2:
	
l2120:	
	line	25
;map.c: 25: finalX = 0;
	clrf	(_finalX)
	line	26
	
l11537:	
;map.c: 26: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	27
;map.c: 27: break;
	goto	l11611
	line	28
;map.c: 28: case 3:
	
l2121:	
	line	29
;map.c: 29: finalX = 0;
	clrf	(_finalX)
	line	30
	
l11539:	
;map.c: 30: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	31
;map.c: 31: break;
	goto	l11611
	line	32
;map.c: 32: default:
	
l2122:	
	line	33
;map.c: 33: break;
	goto	l11611
	line	34
	
l11541:	
;map.c: 34: }
	goto	l11611
	line	16
	
l2117:	
	
l11543:	
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
	goto	l2118
	xorlw	2^1	; case 2
	skipnz
	goto	l2120
	xorlw	3^2	; case 3
	skipnz
	goto	l2121
	goto	l11611
	opt asmopt_on

	line	34
	
l2119:	
	line	35
;map.c: 35: break;
	goto	l11611
	line	37
;map.c: 37: case 1:
	
l2124:	
	line	38
;map.c: 38: switch (virtualWallY)
	goto	l11561
	line	40
;map.c: 39: {
;map.c: 40: case 0:
	
l2126:	
	line	41
	
l11545:	
;map.c: 41: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	42
	
l11547:	
;map.c: 42: finalY = 0;
	clrf	(_finalY)
	line	43
;map.c: 43: break;
	goto	l11611
	line	44
;map.c: 44: case 1:
	
l2128:	
	line	45
	
l11549:	
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
	goto	l11611
	line	48
;map.c: 48: case 2:
	
l2129:	
	line	49
	
l11551:	
;map.c: 49: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	50
	
l11553:	
;map.c: 50: finalY = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	51
;map.c: 51: break;
	goto	l11611
	line	52
;map.c: 52: case 3:
	
l2130:	
	line	53
	
l11555:	
;map.c: 53: finalX = 1;
	clrf	(_finalX)
	bsf	status,0
	rlf	(_finalX),f
	line	54
	
l11557:	
;map.c: 54: finalY = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalY)
	line	55
;map.c: 55: break;
	goto	l11611
	line	56
;map.c: 56: default:
	
l2131:	
	line	57
;map.c: 57: break;
	goto	l11611
	line	58
	
l11559:	
;map.c: 58: }
	goto	l11611
	line	38
	
l2125:	
	
l11561:	
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
	goto	l11545
	xorlw	1^0	; case 1
	skipnz
	goto	l11549
	xorlw	2^1	; case 2
	skipnz
	goto	l11551
	xorlw	3^2	; case 3
	skipnz
	goto	l11555
	goto	l11611
	opt asmopt_on

	line	58
	
l2127:	
	line	59
;map.c: 59: break;
	goto	l11611
	line	61
;map.c: 61: case 2:
	
l2132:	
	line	62
;map.c: 62: switch (virtualWallY)
	goto	l11579
	line	64
;map.c: 63: {
;map.c: 64: case 0:
	
l2134:	
	line	65
	
l11563:	
;map.c: 65: if(robotOrientation == WEST)
	movf	(findFinalDestination@robotOrientation),f
	skipz
	goto	u6381
	goto	u6380
u6381:
	goto	l11611
u6380:
	line	67
	
l11565:	
;map.c: 66: {
;map.c: 67: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	68
	
l11567:	
;map.c: 68: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	goto	l11611
	line	69
	
l2135:	
	line	70
;map.c: 69: }
;map.c: 70: break;
	goto	l11611
	line	71
;map.c: 71: case 1:
	
l2137:	
	line	72
	
l11569:	
;map.c: 72: finalX = 2;
	movlw	(02h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	73
	
l11571:	
;map.c: 73: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	74
;map.c: 74: break;
	goto	l11611
	line	75
;map.c: 75: case 2:
	
l2138:	
	line	76
	
l11573:	
;map.c: 76: if(robotOrientation == EAST)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	02h
	skipz
	goto	u6391
	goto	u6390
u6391:
	goto	l11611
u6390:
	line	78
	
l11575:	
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
	goto	l11611
	line	80
	
l2139:	
	line	81
;map.c: 80: }
;map.c: 81: break;
	goto	l11611
	line	84
;map.c: 84: default:
	
l2140:	
	line	85
;map.c: 85: break;
	goto	l11611
	line	86
	
l11577:	
;map.c: 86: }
	goto	l11611
	line	62
	
l2133:	
	
l11579:	
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
	goto	l11563
	xorlw	1^0	; case 1
	skipnz
	goto	l11569
	xorlw	2^1	; case 2
	skipnz
	goto	l11573
	goto	l11611
	opt asmopt_on

	line	86
	
l2136:	
	line	87
;map.c: 87: break;
	goto	l11611
	line	89
;map.c: 89: case 3:
	
l2141:	
	line	90
;map.c: 90: switch (virtualWallY)
	goto	l11589
	line	92
;map.c: 91: {
;map.c: 92: case 0:
	
l2143:	
	line	93
	
l11581:	
;map.c: 93: finalX = 3;
	movlw	(03h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	94
	
l11583:	
;map.c: 94: finalY = 0;
	clrf	(_finalY)
	line	95
;map.c: 95: break;
	goto	l11611
	line	98
;map.c: 98: case 2:
	
l2145:	
	line	99
	
l11585:	
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
	goto	l11611
	line	104
;map.c: 104: default:
	
l2146:	
	line	105
;map.c: 105: break;
	goto	l11611
	line	106
	
l11587:	
;map.c: 106: }
	goto	l11611
	line	90
	
l2142:	
	
l11589:	
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
	goto	l11581
	xorlw	2^0	; case 2
	skipnz
	goto	l11585
	goto	l11611
	opt asmopt_on

	line	106
	
l2144:	
	line	107
;map.c: 107: break;
	goto	l11611
	line	109
;map.c: 109: case 4:
	
l2147:	
	line	110
;map.c: 110: switch (virtualWallY)
	goto	l11605
	line	112
;map.c: 111: {
;map.c: 112: case 0:
	
l2149:	
	line	113
	
l11591:	
;map.c: 113: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	114
	
l11593:	
;map.c: 114: finalY = 0;
	clrf	(_finalY)
	line	115
;map.c: 115: break;
	goto	l11611
	line	116
;map.c: 116: case 1:
	
l2151:	
	line	117
	
l11595:	
;map.c: 117: finalX = 4;
	movlw	(04h)
	movwf	(??_findFinalDestination+0)+0
	movf	(??_findFinalDestination+0)+0,w
	movwf	(_finalX)
	line	118
	
l11597:	
;map.c: 118: finalY = 1;
	clrf	(_finalY)
	bsf	status,0
	rlf	(_finalY),f
	line	119
;map.c: 119: break;
	goto	l11611
	line	120
;map.c: 120: case 2:
	
l2152:	
	line	121
	
l11599:	
;map.c: 121: if (robotOrientation == SOUTH)
	movf	(findFinalDestination@robotOrientation),w
	xorlw	01h
	skipz
	goto	u6401
	goto	u6400
u6401:
	goto	l11611
u6400:
	line	123
	
l11601:	
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
	goto	l11611
	line	125
	
l2153:	
	line	126
;map.c: 125: }
;map.c: 126: break;
	goto	l11611
	line	127
;map.c: 127: case 3:
	
l2154:	
	line	128
;map.c: 128: finalX = 0;
	clrf	(_finalX)
	line	129
;map.c: 129: finalY = 0;
	clrf	(_finalY)
	line	130
;map.c: 130: break;
	goto	l11611
	line	131
;map.c: 131: default:
	
l2155:	
	line	132
;map.c: 132: break;
	goto	l11611
	line	133
	
l11603:	
;map.c: 133: }
	goto	l11611
	line	110
	
l2148:	
	
l11605:	
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
	goto	l11591
	xorlw	1^0	; case 1
	skipnz
	goto	l11595
	xorlw	2^1	; case 2
	skipnz
	goto	l11599
	xorlw	3^2	; case 3
	skipnz
	goto	l2154
	goto	l11611
	opt asmopt_on

	line	133
	
l2150:	
	line	134
;map.c: 134: break;
	goto	l11611
	line	136
;map.c: 136: default:
	
l2156:	
	line	137
;map.c: 137: break;
	goto	l11611
	line	138
	
l11607:	
;map.c: 138: }
	goto	l11611
	line	13
	
l2115:	
	
l11609:	
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
	goto	l11543
	xorlw	1^0	; case 1
	skipnz
	goto	l11561
	xorlw	2^1	; case 2
	skipnz
	goto	l11579
	xorlw	3^2	; case 3
	skipnz
	goto	l11589
	xorlw	4^3	; case 4
	skipnz
	goto	l11605
	goto	l11611
	opt asmopt_on

	line	138
	
l2123:	
	line	140
	
l11611:	
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
	
l2157:	
	return
	opt stack 0
GLOBAL	__end_of_findFinalDestination
	__end_of_findFinalDestination:
;; =============== function _findFinalDestination ends ============

	signat	_findFinalDestination,12408
	global	_checkIfHome
psect	text1889,local,class=CODE,delta=2
global __ptext1889
__ptext1889:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 327 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1889
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	327
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 3
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	328
	
l11525:	
;main.c: 328: if((xCoord == 1) && (yCoord == 3))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u6361
	goto	u6360
u6361:
	goto	l6019
u6360:
	
l11527:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u6371
	goto	u6370
u6371:
	goto	l6019
u6370:
	line	330
	
l11529:	
;main.c: 329: {
;main.c: 330: drive(0, 0, 0, 0);
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	331
;main.c: 331: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	332
	
l11531:	
;main.c: 332: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l6019
	line	333
	
l6018:	
	line	334
	
l6019:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_turnAround
psect	text1890,local,class=CODE,delta=2
global __ptext1890
__ptext1890:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 208 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1890
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	208
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	209
	
l11519:	
;drive.c: 209: drive(0, 25, 0, 1);
	movlw	(019h)
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
	
l11521:	
;drive.c: 212: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u6847:
	decfsz	((??_turnAround+0)+0),f
	goto	u6847
	decfsz	((??_turnAround+0)+0+1),f
	goto	u6847
	decfsz	((??_turnAround+0)+0+2),f
	goto	u6847
opt asmopt_on

	line	213
	
l11523:	
;drive.c: 213: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u6857:
	decfsz	((??_turnAround+0)+0),f
	goto	u6857
	decfsz	((??_turnAround+0)+0+1),f
	goto	u6857
	decfsz	((??_turnAround+0)+0+2),f
	goto	u6857
opt asmopt_on

	line	214
	
l5132:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_turnLeft90
psect	text1891,local,class=CODE,delta=2
global __ptext1891
__ptext1891:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 217 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1891
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	217
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 0
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	218
	
l11515:	
;drive.c: 218: drive(0, 25, 0, 1);
	movlw	(019h)
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
	
l11517:	
;drive.c: 221: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	224
movwf	((??_turnLeft90+0)+0+1),f
	movlw	254
movwf	((??_turnLeft90+0)+0),f
u6867:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u6867
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u6867
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u6867
opt asmopt_on

	line	222
	
l5135:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnRight90
psect	text1892,local,class=CODE,delta=2
global __ptext1892
__ptext1892:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 225 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1892
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	225
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 0
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	226
	
l11511:	
;drive.c: 226: drive(0, 25, 255, 255);
	movlw	(019h)
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
	
l11513:	
;drive.c: 229: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	224
movwf	((??_turnRight90+0)+0+1),f
	movlw	254
movwf	((??_turnRight90+0)+0),f
u6877:
	decfsz	((??_turnRight90+0)+0),f
	goto	u6877
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u6877
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u6877
opt asmopt_on

	line	230
	
l5138:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_initSongs
psect	text1893,local,class=CODE,delta=2
global __ptext1893
__ptext1893:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 31 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
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
psect	text1893
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
	line	31
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	32
	
l11509:	
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
	movlw	(0x3/2)
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
	movlw	(0x1/2)
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
	
l3642:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text1894,local,class=CODE,delta=2
global __ptext1894
__ptext1894:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
psect	text1894
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l11489:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l11491:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l11493:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l11495:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l11497:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l11499:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l11501:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l11503:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l11505:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l11507:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l1420:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1895,local,class=CODE,delta=2
global __ptext1895
__ptext1895:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text1895
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
	
l11487:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l1408:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_write_string
psect	text1896,local,class=CODE,delta=2
global __ptext1896
__ptext1896:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
;; Parameters:    Size  Location     Type
;;  s               1    wreg     PTR const unsigned char 
;;		 -> STR_2(17), STR_1(17), 
;; Auto vars:     Size  Location     Type
;;  s               1   14[BANK0 ] PTR const unsigned char 
;;		 -> STR_2(17), STR_1(17), 
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
;; This function uses a non-reentrant model
;;
psect	text1896
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
	
l11479:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l11485
	
l1403:	
	
l11481:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l11483:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l11485
	
l1402:	
	
l11485:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u6351
	goto	u6350
u6351:
	goto	l11481
u6350:
	goto	l1405
	
l1404:	
	line	41
	
l1405:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_lcd_set_cursor
psect	text1897,local,class=CODE,delta=2
global __ptext1897
__ptext1897:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
psect	text1897
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
	
l11475:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l11477:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l1399:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_adc_read_channel
psect	text1898,local,class=CODE,delta=2
global __ptext1898
__ptext1898:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
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
psect	text1898
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
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
	
l11459:	
;adc.c: 8: switch(channel)
	goto	l11467
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
	goto	l11469
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
	goto	l11469
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
	goto	l11469
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
	goto	l11469
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
	goto	l11469
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l11461:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l11463:	
	goto	l697
	line	39
	
l11465:	
;adc.c: 39: }
	goto	l11469
	line	8
	
l689:	
	
l11467:	
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
	goto	l11461
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l11469:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u6887:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u6887
opt asmopt_on

	line	43
	
l11471:	
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
	
l11473:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	___lbtoft
psect	text1899,local,class=CODE,delta=2
global __ptext1899
__ptext1899:

;; *************** function ___lbtoft *****************
;; Defined at:
;;		line 28 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lbtoft.c"
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
psect	text1899
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lbtoft.c"
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
	
l11455:	
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
	goto	l6858
	
l11457:	
	line	30
	
l6858:	
	return
	opt stack 0
GLOBAL	__end_of___lbtoft
	__end_of___lbtoft:
;; =============== function ___lbtoft ends ============

	signat	___lbtoft,4219
	global	___ftmul
psect	text1900,local,class=CODE,delta=2
global __ptext1900
__ptext1900:

;; *************** function ___ftmul *****************
;; Defined at:
;;		line 52 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\ftmul.c"
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
psect	text1900
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\ftmul.c"
	line	52
	global	__size_of___ftmul
	__size_of___ftmul	equ	__end_of___ftmul-___ftmul
	
___ftmul:	
	opt	stack 3
; Regs used in ___ftmul: [wreg+status,2+status,0+pclath+cstack]
	line	56
	
l11405:	
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
	goto	u6211
	goto	u6210
u6211:
	goto	l11411
u6210:
	line	57
	
l11407:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l6832
	
l11409:	
	goto	l6832
	
l6831:	
	line	58
	
l11411:	
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
	goto	u6221
	goto	u6220
u6221:
	goto	l11417
u6220:
	line	59
	
l11413:	
	movlw	0x0
	movwf	(?___ftmul)
	movlw	0x0
	movwf	(?___ftmul+1)
	movlw	0x0
	movwf	(?___ftmul+2)
	goto	l6832
	
l11415:	
	goto	l6832
	
l6833:	
	line	60
	
l11417:	
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
u6235:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6230:
	addlw	-1
	skipz
	goto	u6235
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
u6245:
	clrc
	rrf	(??___ftmul+0)+2,f
	rrf	(??___ftmul+0)+1,f
	rrf	(??___ftmul+0)+0,f
u6240:
	addlw	-1
	skipz
	goto	u6245
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
	
l11419:	
	bsf	(___ftmul@f1)+(15/8),(15)&7
	line	66
	
l11421:	
	bsf	(___ftmul@f2)+(15/8),(15)&7
	line	67
	
l11423:	
	movlw	0FFh
	andwf	(___ftmul@f2),f
	movlw	0FFh
	andwf	(___ftmul@f2+1),f
	movlw	0
	andwf	(___ftmul@f2+2),f
	line	68
	
l11425:	
	movlw	0
	movwf	(___ftmul@f3_as_product)
	movlw	0
	movwf	(___ftmul@f3_as_product+1)
	movlw	0
	movwf	(___ftmul@f3_as_product+2)
	line	69
	
l11427:	
	movlw	(07h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l11429
	line	70
	
l6834:	
	line	71
	
l11429:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6251
	goto	u6250
u6251:
	goto	l11433
u6250:
	line	72
	
l11431:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6261
	addwf	(___ftmul@f3_as_product+1),f
u6261:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6262
	addwf	(___ftmul@f3_as_product+2),f
u6262:

	goto	l11433
	
l6835:	
	line	73
	
l11433:	
	movlw	01h
u6275:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6275

	line	74
	
l11435:	
	movlw	01h
u6285:
	clrc
	rlf	(___ftmul@f2),f
	rlf	(___ftmul@f2+1),f
	rlf	(___ftmul@f2+2),f
	addlw	-1
	skipz
	goto	u6285
	line	75
	
l11437:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6291
	goto	u6290
u6291:
	goto	l11429
u6290:
	goto	l11439
	
l6836:	
	line	76
	
l11439:	
	movlw	(09h)
	movwf	(??___ftmul+0)+0
	movf	(??___ftmul+0)+0,w
	movwf	(___ftmul@cntr)
	goto	l11441
	line	77
	
l6837:	
	line	78
	
l11441:	
	btfss	(___ftmul@f1),(0)&7
	goto	u6301
	goto	u6300
u6301:
	goto	l11445
u6300:
	line	79
	
l11443:	
	movf	(___ftmul@f2),w
	addwf	(___ftmul@f3_as_product),f
	movf	(___ftmul@f2+1),w
	clrz
	skipnc
	incf	(___ftmul@f2+1),w
	skipnz
	goto	u6311
	addwf	(___ftmul@f3_as_product+1),f
u6311:
	movf	(___ftmul@f2+2),w
	clrz
	skipnc
	incf	(___ftmul@f2+2),w
	skipnz
	goto	u6312
	addwf	(___ftmul@f3_as_product+2),f
u6312:

	goto	l11445
	
l6838:	
	line	80
	
l11445:	
	movlw	01h
u6325:
	clrc
	rrf	(___ftmul@f1+2),f
	rrf	(___ftmul@f1+1),f
	rrf	(___ftmul@f1),f
	addlw	-1
	skipz
	goto	u6325

	line	81
	
l11447:	
	movlw	01h
u6335:
	clrc
	rrf	(___ftmul@f3_as_product+2),f
	rrf	(___ftmul@f3_as_product+1),f
	rrf	(___ftmul@f3_as_product),f
	addlw	-1
	skipz
	goto	u6335

	line	82
	
l11449:	
	movlw	low(01h)
	subwf	(___ftmul@cntr),f
	btfss	status,2
	goto	u6341
	goto	u6340
u6341:
	goto	l11441
u6340:
	goto	l11451
	
l6839:	
	line	83
	
l11451:	
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
	goto	l6832
	
l11453:	
	line	84
	
l6832:	
	return
	opt stack 0
GLOBAL	__end_of___ftmul
	__end_of___ftmul:
;; =============== function ___ftmul ends ============

	signat	___ftmul,8315
	global	_initIRobot
psect	text1901,local,class=CODE,delta=2
global __ptext1901
__ptext1901:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 129 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1901
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	129
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	130
	
l11399:	
;main.c: 130: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u6897:
	decfsz	((??_initIRobot+0)+0),f
	goto	u6897
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u6897
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u6897
	nop2
opt asmopt_on

	line	131
	
l11401:	
;main.c: 131: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	132
	
l11403:	
;main.c: 132: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	133
	
l5962:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_waitFor
psect	text1902,local,class=CODE,delta=2
global __ptext1902
__ptext1902:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 240 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1902
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	240
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 0
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	241
	
l11391:	
;drive.c: 241: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u6907:
	decfsz	((??_waitFor+0)+0),f
	goto	u6907
	decfsz	((??_waitFor+0)+0+1),f
	goto	u6907
	decfsz	((??_waitFor+0)+0+2),f
	goto	u6907
	nop2
opt asmopt_on

	line	242
	
l11393:	
;drive.c: 242: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	243
	
l11395:	
;drive.c: 243: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	244
	
l11397:	
;drive.c: 244: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	245
	
l5145:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_drive
psect	text1903,local,class=CODE,delta=2
global __ptext1903
__ptext1903:

;; *************** function _drive *****************
;; Defined at:
;;		line 21 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;;		_goParallel
;;		_checkIfHome
;;		_main
;;		_frontWallCorrect
;; This function uses a non-reentrant model
;;
psect	text1903
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
	
l11379:	
;drive.c: 22: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u6917:
	decfsz	((??_drive+0)+0),f
	goto	u6917
	decfsz	((??_drive+0)+0+1),f
	goto	u6917
	decfsz	((??_drive+0)+0+2),f
	goto	u6917
	nop2
opt asmopt_on

	line	23
	
l11381:	
;drive.c: 23: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	24
	
l11383:	
;drive.c: 24: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	25
	
l11385:	
;drive.c: 25: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	26
	
l11387:	
;drive.c: 26: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	27
	
l11389:	
;drive.c: 27: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	28
	
l5081:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_rotateIR
psect	text1904,local,class=CODE,delta=2
global __ptext1904
__ptext1904:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
;; Parameters:    Size  Location     Type
;;  steps           1    wreg     unsigned char 
;;  direction       1   10[BANK0 ] unsigned char 
;; Auto vars:     Size  Location     Type
;;  steps           1   14[BANK0 ] unsigned char 
;;  stepNum         1   15[BANK0 ] unsigned char 
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
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_findWalls
;;		_goParallel
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1904
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
	line	39
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 5
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	40
	
l11361:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l11363:	
;ir.c: 41: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	42
	
l11365:	
;ir.c: 42: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u6927:
	decfsz	((??_rotateIR+0)+0),f
	goto	u6927
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u6927
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u6927
	nop2
opt asmopt_on

	line	44
	
l11367:	
;ir.c: 44: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l4351
	line	45
	
l4352:	
	line	46
;ir.c: 45: {
;ir.c: 46: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	47
	
l11369:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l11371:	
;ir.c: 48: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u6937:
	decfsz	((??_rotateIR+0)+0),f
	goto	u6937
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u6937
	nop2
opt asmopt_on

	line	44
	
l11373:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l4351:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u6201
	goto	u6200
u6201:
	goto	l4352
u6200:
	goto	l11375
	
l4353:	
	line	51
	
l11375:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l11377:	
;ir.c: 52: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u6947:
	decfsz	((??_rotateIR+0)+0),f
	goto	u6947
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u6947
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u6947
	nop2
opt asmopt_on

	line	54
	
l4354:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_convert
psect	text1905,local,class=CODE,delta=2
global __ptext1905
__ptext1905:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
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
psect	text1905
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 2
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l11301:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6135
	movlw	low(052h)
	subwf	(convert@adc_value),w
u6135:

	skipnc
	goto	u6131
	goto	u6130
u6131:
	goto	l11309
u6130:
	line	13
	
l11303:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l4332
	
l11305:	
	goto	l4332
	
l11307:	
	goto	l4332
	line	14
	
l4331:	
	
l11309:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6145
	movlw	low(085h)
	subwf	(convert@adc_value),w
u6145:

	skipnc
	goto	u6141
	goto	u6140
u6141:
	goto	l11317
u6140:
	line	15
	
l11311:	
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
	goto	l4332
	
l11313:	
	goto	l4332
	
l11315:	
	goto	l4332
	line	16
	
l4334:	
	
l11317:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6155
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u6155:

	skipnc
	goto	u6151
	goto	u6150
u6151:
	goto	l11325
u6150:
	line	17
	
l11319:	
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
	goto	l4332
	
l11321:	
	goto	l4332
	
l11323:	
	goto	l4332
	line	18
	
l4336:	
	
l11325:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u6165
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u6165:

	skipnc
	goto	u6161
	goto	u6160
u6161:
	goto	l11333
u6160:
	line	19
	
l11327:	
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
	goto	l4332
	
l11329:	
	goto	l4332
	
l11331:	
	goto	l4332
	line	20
	
l4338:	
	
l11333:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u6175
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u6175:

	skipnc
	goto	u6171
	goto	u6170
u6171:
	goto	l11341
u6170:
	line	21
	
l11335:	
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
	goto	l4332
	
l11337:	
	goto	l4332
	
l11339:	
	goto	l4332
	line	22
	
l4340:	
	
l11341:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6185
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u6185:

	skipnc
	goto	u6181
	goto	u6180
u6181:
	goto	l11349
u6180:
	line	23
	
l11343:	
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
	goto	l4332
	
l11345:	
	goto	l4332
	
l11347:	
	goto	l4332
	line	24
	
l4342:	
	
l11349:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u6195
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u6195:

	skipnc
	goto	u6191
	goto	u6190
u6191:
	goto	l11357
u6190:
	line	25
	
l11351:	
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
	goto	l4332
	
l11353:	
	goto	l4332
	
l11355:	
	goto	l4332
	line	26
	
l4344:	
	
l11357:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l4332
	
l11359:	
	goto	l4332
	
l4345:	
	goto	l4332
	
l4343:	
	goto	l4332
	
l4341:	
	goto	l4332
	
l4339:	
	goto	l4332
	
l4337:	
	goto	l4332
	
l4335:	
	goto	l4332
	
l4333:	
	line	27
	
l4332:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_play_iCreate_song
psect	text1906,local,class=CODE,delta=2
global __ptext1906
__ptext1906:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 25 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
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
;;		_goParallel
;;		_checkIfHome
;; This function uses a non-reentrant model
;;
psect	text1906
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\songs.c"
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
	
l11299:	
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
	
l3639:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_ser_putArr
psect	text1907,local,class=CODE,delta=2
global __ptext1907
__ptext1907:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
psect	text1907
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l11291:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l11297
	line	75
	
l2910:	
	line	76
	
l11293:	
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
	goto	u6110
	decf	(??_ser_putArr+0)+0,f
u6110:
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
	
l11295:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l11297
	
l2909:	
	
l11297:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u6125
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u6125:

	skipc
	goto	u6121
	goto	u6120
u6121:
	goto	l11293
u6120:
	goto	l2912
	
l2911:	
	line	78
	
l2912:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putArr
	__end_of_ser_putArr:
;; =============== function _ser_putArr ends ============

	signat	_ser_putArr,8312
	global	_ser_getch
psect	text1908,local,class=CODE,delta=2
global __ptext1908
__ptext1908:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
psect	text1908
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 1
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l11275:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l11277
	
l2904:	
	line	62
;ser.c: 62: continue;
	goto	l11277
	
l2903:	
	line	61
	
l11277:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u6101
	goto	u6100
u6101:
	goto	l11277
u6100:
	
l2905:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l11279:	
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
	
l11281:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l11283:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l11285:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l11287:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l2906
	
l11289:	
	line	70
	
l2906:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_lcd_write_data
psect	text1909,local,class=CODE,delta=2
global __ptext1909
__ptext1909:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1909
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
	
l11267:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l11269:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l11271:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l11273:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u6957:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u6957
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u6957
opt asmopt_on

	line	28
	
l1396:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_lcd_write_control
psect	text1910,local,class=CODE,delta=2
global __ptext1910
__ptext1910:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
psect	text1910
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\lcd.c"
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
	
l11259:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l11261:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l11263:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l11265:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u6967:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u6967
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u6967
	nop2
opt asmopt_on

	line	16
	
l1393:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_init_adc
psect	text1911,local,class=CODE,delta=2
global __ptext1911
__ptext1911:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
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
psect	text1911
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l11249:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l11251:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l11253:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l11255:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l11257:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u6977:
decfsz	(??_init_adc+0)+0,f
	goto	u6977
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
psect	text1912,local,class=CODE,delta=2
global __ptext1912
__ptext1912:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
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
psect	text1912
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 1
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l11239:	
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
	
l11241:	
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
	goto	u6081
	goto	u6080
u6081:
	goto	l703
u6080:
	goto	l11243
	
l705:	
	line	75
	
l11243:	
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
u6095:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u6095
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l11245:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l11247:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1913,local,class=CODE,delta=2
global __ptext1913
__ptext1913:

;; *************** function ___awdiv *****************
;; Defined at:
;;		line 5 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
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
psect	text1913
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 2
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l11153:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u5921
	goto	u5920
u5921:
	goto	l11157
u5920:
	line	11
	
l11155:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l11157
	line	13
	
l6873:	
	line	14
	
l11157:	
	btfss	(___awdiv@dividend+1),7
	goto	u5931
	goto	u5930
u5931:
	goto	l11163
u5930:
	line	15
	
l11159:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l11161:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l11163
	line	17
	
l6874:	
	line	18
	
l11163:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l11165:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u5941
	goto	u5940
u5941:
	goto	l11185
u5940:
	line	20
	
l11167:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l11173
	
l6877:	
	line	22
	
l11169:	
	movlw	01h
	
u5955:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u5955
	line	23
	
l11171:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l11173
	line	24
	
l6876:	
	line	21
	
l11173:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u5961
	goto	u5960
u5961:
	goto	l11169
u5960:
	goto	l11175
	
l6878:	
	goto	l11175
	line	25
	
l6879:	
	line	26
	
l11175:	
	movlw	01h
	
u5975:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u5975
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u5985
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u5985:
	skipc
	goto	u5981
	goto	u5980
u5981:
	goto	l11181
u5980:
	line	28
	
l11177:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l11179:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l11181
	line	30
	
l6880:	
	line	31
	
l11181:	
	movlw	01h
	
u5995:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u5995
	line	32
	
l11183:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u6001
	goto	u6000
u6001:
	goto	l11175
u6000:
	goto	l11185
	
l6881:	
	goto	l11185
	line	33
	
l6875:	
	line	34
	
l11185:	
	movf	(___awdiv@sign),w
	skipz
	goto	u6010
	goto	l11189
u6010:
	line	35
	
l11187:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l11189
	
l6882:	
	line	36
	
l11189:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l6883
	
l11191:	
	line	37
	
l6883:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___fttol
psect	text1914,local,class=CODE,delta=2
global __ptext1914
__ptext1914:

;; *************** function ___fttol *****************
;; Defined at:
;;		line 45 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\fttol.c"
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
psect	text1914
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\fttol.c"
	line	45
	global	__size_of___fttol
	__size_of___fttol	equ	__end_of___fttol-___fttol
	
___fttol:	
	opt	stack 4
; Regs used in ___fttol: [wreg+status,2+status,0]
	line	49
	
l11093:	
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
	goto	u5801
	goto	u5800
u5801:
	goto	l11099
u5800:
	line	50
	
l11095:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6843
	
l11097:	
	goto	l6843
	
l6842:	
	line	51
	
l11099:	
	movf	(___fttol@f1),w
	movwf	((??___fttol+0)+0)
	movf	(___fttol@f1+1),w
	movwf	((??___fttol+0)+0+1)
	movf	(___fttol@f1+2),w
	movwf	((??___fttol+0)+0+2)
	movlw	017h
u5815:
	clrc
	rrf	(??___fttol+0)+2,f
	rrf	(??___fttol+0)+1,f
	rrf	(??___fttol+0)+0,f
u5810:
	addlw	-1
	skipz
	goto	u5815
	movf	0+(??___fttol+0)+0,w
	movwf	(??___fttol+3)+0
	movf	(??___fttol+3)+0,w
	movwf	(___fttol@sign1)
	line	52
	
l11101:	
	bsf	(___fttol@f1)+(15/8),(15)&7
	line	53
	
l11103:	
	movlw	0FFh
	andwf	(___fttol@f1),f
	movlw	0FFh
	andwf	(___fttol@f1+1),f
	movlw	0
	andwf	(___fttol@f1+2),f
	line	54
	
l11105:	
	movf	(___fttol@f1),w
	movwf	(___fttol@lval)
	movf	(___fttol@f1+1),w
	movwf	((___fttol@lval))+1
	movf	(___fttol@f1+2),w
	movwf	((___fttol@lval))+2
	clrf	((___fttol@lval))+3
	line	55
	
l11107:	
	movlw	low(08Eh)
	subwf	(___fttol@exp1),f
	line	56
	
l11109:	
	btfss	(___fttol@exp1),7
	goto	u5821
	goto	u5820
u5821:
	goto	l11119
u5820:
	line	57
	
l11111:	
	movf	(___fttol@exp1),w
	xorlw	80h
	addlw	-((-15)^80h)
	skipnc
	goto	u5831
	goto	u5830
u5831:
	goto	l11117
u5830:
	line	58
	
l11113:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6843
	
l11115:	
	goto	l6843
	
l6845:	
	goto	l11117
	line	59
	
l6846:	
	line	60
	
l11117:	
	movlw	01h
u5845:
	clrc
	rrf	(___fttol@lval+3),f
	rrf	(___fttol@lval+2),f
	rrf	(___fttol@lval+1),f
	rrf	(___fttol@lval),f
	addlw	-1
	skipz
	goto	u5845

	line	61
	movlw	(01h)
	movwf	(??___fttol+0)+0
	movf	(??___fttol+0)+0,w
	addwf	(___fttol@exp1),f
	btfss	status,2
	goto	u5851
	goto	u5850
u5851:
	goto	l11117
u5850:
	goto	l11129
	
l6847:	
	line	62
	goto	l11129
	
l6844:	
	line	63
	
l11119:	
	movlw	(018h)
	subwf	(___fttol@exp1),w
	skipc
	goto	u5861
	goto	u5860
u5861:
	goto	l11127
u5860:
	line	64
	
l11121:	
	movlw	0
	movwf	(?___fttol+3)
	movlw	0
	movwf	(?___fttol+2)
	movlw	0
	movwf	(?___fttol+1)
	movlw	0
	movwf	(?___fttol)

	goto	l6843
	
l11123:	
	goto	l6843
	
l6849:	
	line	65
	goto	l11127
	
l6851:	
	line	66
	
l11125:	
	movlw	01h
	movwf	(??___fttol+0)+0
u5875:
	clrc
	rlf	(___fttol@lval),f
	rlf	(___fttol@lval+1),f
	rlf	(___fttol@lval+2),f
	rlf	(___fttol@lval+3),f
	decfsz	(??___fttol+0)+0
	goto	u5875
	line	67
	movlw	low(01h)
	subwf	(___fttol@exp1),f
	goto	l11127
	line	68
	
l6850:	
	line	65
	
l11127:	
	movf	(___fttol@exp1),f
	skipz
	goto	u5881
	goto	u5880
u5881:
	goto	l11125
u5880:
	goto	l11129
	
l6852:	
	goto	l11129
	line	69
	
l6848:	
	line	70
	
l11129:	
	movf	(___fttol@sign1),w
	skipz
	goto	u5890
	goto	l11133
u5890:
	line	71
	
l11131:	
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
	goto	l11133
	
l6853:	
	line	72
	
l11133:	
	movf	(___fttol@lval+3),w
	movwf	(?___fttol+3)
	movf	(___fttol@lval+2),w
	movwf	(?___fttol+2)
	movf	(___fttol@lval+1),w
	movwf	(?___fttol+1)
	movf	(___fttol@lval),w
	movwf	(?___fttol)

	goto	l6843
	
l11135:	
	line	73
	
l6843:	
	return
	opt stack 0
GLOBAL	__end_of___fttol
	__end_of___fttol:
;; =============== function ___fttol ends ============

	signat	___fttol,4220
	global	___ftpack
psect	text1915,local,class=CODE,delta=2
global __ptext1915
__ptext1915:

;; *************** function ___ftpack *****************
;; Defined at:
;;		line 63 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\float.c"
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
;;		___ftmul
;;		___lbtoft
;;		___ftadd
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
psect	text1915
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\float.c"
	line	63
	global	__size_of___ftpack
	__size_of___ftpack	equ	__end_of___ftpack-___ftpack
	
___ftpack:	
	opt	stack 3
; Regs used in ___ftpack: [wreg+status,2+status,0]
	line	64
	
l11019:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(___ftpack@exp),w
	skipz
	goto	u5610
	goto	l11023
u5610:
	
l11021:	
	movf	(___ftpack@arg+2),w
	iorwf	(___ftpack@arg+1),w
	iorwf	(___ftpack@arg),w
	skipz
	goto	u5621
	goto	u5620
u5621:
	goto	l11029
u5620:
	goto	l11023
	
l7067:	
	line	65
	
l11023:	
	movlw	0x0
	movwf	(?___ftpack)
	movlw	0x0
	movwf	(?___ftpack+1)
	movlw	0x0
	movwf	(?___ftpack+2)
	goto	l7068
	
l11025:	
	goto	l7068
	
l7065:	
	line	66
	goto	l11029
	
l7070:	
	line	67
	
l11027:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	68
	movlw	01h
u5635:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u5635

	goto	l11029
	line	69
	
l7069:	
	line	66
	
l11029:	
	movlw	low highword(0FE0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u5641
	goto	u5640
u5641:
	goto	l11027
u5640:
	goto	l7072
	
l7071:	
	line	70
	goto	l7072
	
l7073:	
	line	71
	
l11031:	
	movlw	(01h)
	movwf	(??___ftpack+0)+0
	movf	(??___ftpack+0)+0,w
	addwf	(___ftpack@exp),f
	line	72
	
l11033:	
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
	
l11035:	
	movlw	01h
u5655:
	clrc
	rrf	(___ftpack@arg+2),f
	rrf	(___ftpack@arg+1),f
	rrf	(___ftpack@arg),f
	addlw	-1
	skipz
	goto	u5655

	line	74
	
l7072:	
	line	70
	movlw	low highword(0FF0000h)
	andwf	(___ftpack@arg+2),w
	btfss	status,2
	goto	u5661
	goto	u5660
u5661:
	goto	l11031
u5660:
	goto	l11039
	
l7074:	
	line	75
	goto	l11039
	
l7076:	
	line	76
	
l11037:	
	movlw	low(01h)
	subwf	(___ftpack@exp),f
	line	77
	movlw	01h
u5675:
	clrc
	rlf	(___ftpack@arg),f
	rlf	(___ftpack@arg+1),f
	rlf	(___ftpack@arg+2),f
	addlw	-1
	skipz
	goto	u5675
	goto	l11039
	line	78
	
l7075:	
	line	75
	
l11039:	
	btfss	(___ftpack@arg+1),(15)&7
	goto	u5681
	goto	u5680
u5681:
	goto	l11037
u5680:
	
l7077:	
	line	79
	btfsc	(___ftpack@exp),(0)&7
	goto	u5691
	goto	u5690
u5691:
	goto	l7078
u5690:
	line	80
	
l11041:	
	movlw	0FFh
	andwf	(___ftpack@arg),f
	movlw	07Fh
	andwf	(___ftpack@arg+1),f
	movlw	0FFh
	andwf	(___ftpack@arg+2),f
	
l7078:	
	line	81
	clrc
	rrf	(___ftpack@exp),f

	line	82
	
l11043:	
	movf	(___ftpack@exp),w
	movwf	((??___ftpack+0)+0)
	clrf	((??___ftpack+0)+0+1)
	clrf	((??___ftpack+0)+0+2)
	movlw	010h
u5705:
	clrc
	rlf	(??___ftpack+0)+0,f
	rlf	(??___ftpack+0)+1,f
	rlf	(??___ftpack+0)+2,f
u5700:
	addlw	-1
	skipz
	goto	u5705
	movf	0+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg),f
	movf	1+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+1),f
	movf	2+(??___ftpack+0)+0,w
	iorwf	(___ftpack@arg+2),f
	line	83
	
l11045:	
	movf	(___ftpack@sign),w
	skipz
	goto	u5710
	goto	l7079
u5710:
	line	84
	
l11047:	
	bsf	(___ftpack@arg)+(23/8),(23)&7
	
l7079:	
	line	85
	line	86
	
l7068:	
	return
	opt stack 0
GLOBAL	__end_of___ftpack
	__end_of___ftpack:
;; =============== function ___ftpack ends ============

	signat	___ftpack,12411
	global	___wmul
psect	text1916,local,class=CODE,delta=2
global __ptext1916
__ptext1916:

;; *************** function ___wmul *****************
;; Defined at:
;;		line 3 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
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
psect	text1916
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 2
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l10937:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l10939
	line	6
	
l6733:	
	line	7
	
l10939:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u5331
	goto	u5330
u5331:
	goto	l6734
u5330:
	line	8
	
l10941:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l6734:	
	line	9
	movlw	01h
	
u5345:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u5345
	line	10
	
l10943:	
	movlw	01h
	
u5355:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u5355
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u5361
	goto	u5360
u5361:
	goto	l10939
u5360:
	goto	l10945
	
l6735:	
	line	12
	
l10945:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l6736
	
l10947:	
	line	13
	
l6736:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_updateNode
psect	text1917,local,class=CODE,delta=2
global __ptext1917
__ptext1917:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 315 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1917
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	315
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 5
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	316
	
l10887:	
;main.c: 316: if((xCoord == 2) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5231
	goto	u5230
u5231:
	goto	l10893
u5230:
	
l10889:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5241
	goto	u5240
u5241:
	goto	l10893
u5240:
	line	317
	
l10891:	
;main.c: 317: node = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	bsf	status,0
	rlf	(_node)^080h,f	;volatile
	goto	l6015
	line	318
	
l6009:	
	
l10893:	
;main.c: 318: else if((xCoord == 4) && (yCoord == 2))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u5251
	goto	u5250
u5251:
	goto	l10899
u5250:
	
l10895:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5261
	goto	u5260
u5261:
	goto	l10899
u5260:
	line	319
	
l10897:	
;main.c: 319: node = 2;
	movlw	(02h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6015
	line	320
	
l6011:	
	
l10899:	
;main.c: 320: else if((xCoord == 2) && (yCoord == 0))
	bcf	status, 5	;RP0=0, select bank0
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u5271
	goto	u5270
u5271:
	goto	l6013
u5270:
	
l10901:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u5281
	goto	u5280
u5281:
	goto	l6013
u5280:
	line	321
	
l10903:	
;main.c: 321: node = 3;
	movlw	(03h)
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(_node)^080h	;volatile
	goto	l6015
	line	322
	
l6013:	
	line	323
;main.c: 322: else
;main.c: 323: node = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_node)^080h	;volatile
	goto	l6015
	
l6014:	
	goto	l6015
	
l6012:	
	goto	l6015
	
l6010:	
	line	324
	
l6015:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getSuccessfulDrive
psect	text1918,local,class=CODE,delta=2
global __ptext1918
__ptext1918:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 146 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1918
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	146
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 5
; Regs used in _getSuccessfulDrive: [status]
	line	147
	
l10863:	
;drive.c: 147: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u5191
	goto	u5190
u5191:
	goto	l10867
u5190:
	
l10865:	
	clrc
	
	goto	l5111
	
l10701:	
	
l10867:	
	setc
	
	goto	l5111
	
l10703:	
	goto	l5111
	
l10869:	
	line	148
	
l5111:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_getSomethingInTheWay
psect	text1919,local,class=CODE,delta=2
global __ptext1919
__ptext1919:

;; *************** function _getSomethingInTheWay *****************
;; Defined at:
;;		line 140 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1919
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	140
	global	__size_of_getSomethingInTheWay
	__size_of_getSomethingInTheWay	equ	__end_of_getSomethingInTheWay-_getSomethingInTheWay
	
_getSomethingInTheWay:	
	opt	stack 4
; Regs used in _getSomethingInTheWay: [wreg]
	line	141
	
l10859:	
;drive.c: 141: return somethingInTheWay;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_somethingInTheWay),w	;volatile
	goto	l5108
	
l10861:	
	line	142
	
l5108:	
	return
	opt stack 0
GLOBAL	__end_of_getSomethingInTheWay
	__end_of_getSomethingInTheWay:
;; =============== function _getSomethingInTheWay ends ============

	signat	_getSomethingInTheWay,89
	global	_getOrientation
psect	text1920,local,class=CODE,delta=2
global __ptext1920
__ptext1920:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 135 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1920
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	135
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 4
; Regs used in _getOrientation: [wreg]
	line	136
	
l10855:	
;drive.c: 136: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l5105
	
l10857:	
	line	137
	
l5105:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_getCurrentY
psect	text1921,local,class=CODE,delta=2
global __ptext1921
__ptext1921:

;; *************** function _getCurrentY *****************
;; Defined at:
;;		line 409 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1921
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	409
	global	__size_of_getCurrentY
	__size_of_getCurrentY	equ	__end_of_getCurrentY-_getCurrentY
	
_getCurrentY:	
	opt	stack 2
; Regs used in _getCurrentY: [wreg]
	line	410
	
l10851:	
;main.c: 410: return yCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	goto	l6041
	
l10853:	
	line	411
	
l6041:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentY
	__end_of_getCurrentY:
;; =============== function _getCurrentY ends ============

	signat	_getCurrentY,89
	global	_getCurrentX
psect	text1922,local,class=CODE,delta=2
global __ptext1922
__ptext1922:

;; *************** function _getCurrentX *****************
;; Defined at:
;;		line 404 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1922
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	404
	global	__size_of_getCurrentX
	__size_of_getCurrentX	equ	__end_of_getCurrentX-_getCurrentX
	
_getCurrentX:	
	opt	stack 2
; Regs used in _getCurrentX: [wreg]
	line	405
	
l10847:	
;main.c: 405: return xCoord;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	goto	l6038
	
l10849:	
	line	406
	
l6038:	
	return
	opt stack 0
GLOBAL	__end_of_getCurrentX
	__end_of_getCurrentX:
;; =============== function _getCurrentX ends ============

	signat	_getCurrentX,89
	global	_updateOrientation
psect	text1923,local,class=CODE,delta=2
global __ptext1923
__ptext1923:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 233 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
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
psect	text1923
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\drive.c"
	line	233
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 2
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	234
	
l10841:	
;drive.c: 234: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	235
	
l10843:	
;drive.c: 235: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u5181
	goto	u5180
u5181:
	goto	l5142
u5180:
	line	236
	
l10845:	
;drive.c: 236: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l5142
	
l5141:	
	line	237
	
l5142:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_ser_init
psect	text1924,local,class=CODE,delta=2
global __ptext1924
__ptext1924:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
psect	text1924
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l10815:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l10817:	
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
	
l10819:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l10821:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l10823:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l10825:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l10827:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l10829:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l10831:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l10833:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l10835:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l10837:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l10839:	
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
	
l2940:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_ser_putch
psect	text1925,local,class=CODE,delta=2
global __ptext1925
__ptext1925:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
;;		_ser_puts
;;		_ser_puts2
;;		_ser_puthex
;; This function uses a non-reentrant model
;;
psect	text1925
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
	
l10771:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l10773
	
l2916:	
	line	83
;ser.c: 83: continue;
	goto	l10773
	
l2915:	
	line	82
	
l10773:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u5121
	goto	u5120
u5121:
	goto	l10773
u5120:
	
l2917:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l10775:	
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
	
l10777:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l10779:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l10781:	
;ser.c: 88: GIE=1;
	bsf	(95/8),(95)&7
	line	89
	
l2918:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_ser_isrx
psect	text1926,local,class=CODE,delta=2
global __ptext1926
__ptext1926:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
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
psect	text1926
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 1
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l10755:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u5101
	goto	u5100
u5101:
	goto	l10763
u5100:
	line	50
	
l10757:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l10759:	
;ser.c: 52: return 0;
	clrc
	
	goto	l2900
	
l10761:	
	goto	l2900
	line	53
	
l2899:	
	line	54
	
l10763:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u5111
	goto	u5110
u5111:
	goto	l10767
u5110:
	
l10765:	
	clrc
	
	goto	l2900
	
l10691:	
	
l10767:	
	setc
	
	goto	l2900
	
l10693:	
	goto	l2900
	
l10769:	
	line	55
	
l2900:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_getFinalY
psect	text1927,local,class=CODE,delta=2
global __ptext1927
__ptext1927:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
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
psect	text1927
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 4
; Regs used in _getFinalY: [wreg]
	line	153
	
l10751:	
;map.c: 153: return finalY;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	goto	l2163
	
l10753:	
	line	154
	
l2163:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text1928,local,class=CODE,delta=2
global __ptext1928
__ptext1928:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
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
psect	text1928
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 4
; Regs used in _getFinalX: [wreg]
	line	148
	
l10747:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l2160
	
l10749:	
	line	149
	
l2160:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_isr1
psect	text1929,local,class=CODE,delta=2
global __ptext1929
__ptext1929:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 61 in file "C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
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
psect	text1929
	file	"C:\Users\11000294\Desktop\COMPETITIONv0.6\main.c"
	line	61
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
psect	text1929
	line	63
	
i1l9529:	
;main.c: 63: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u332_21
	goto	u332_20
u332_21:
	goto	i1l5956
u332_20:
	line	65
	
i1l9531:	
;main.c: 64: {
;main.c: 65: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	66
	
i1l9533:	
;main.c: 66: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	68
;main.c: 68: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	70
	
i1l9535:	
;main.c: 70: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	72
	
i1l9537:	
;main.c: 72: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
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
	goto	u333_21
	goto	u333_20
u333_21:
	goto	i1l9541
u333_20:
	
i1l9539:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l9541
	
i1l5946:	
	line	73
	
i1l9541:	
;main.c: 73: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
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
	goto	u334_21
	goto	u334_20
u334_21:
	goto	i1l9545
u334_20:
	
i1l9543:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l9545
	
i1l5947:	
	line	74
	
i1l9545:	
;main.c: 74: if(RTC_Counter % 500 == 0)
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
	line	77
;main.c: 75: {
	
i1l5948:	
	line	79
;main.c: 77: }
;main.c: 79: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u335_21
	goto	u335_20
u335_21:
	goto	i1l5949
u335_20:
	line	81
	
i1l9547:	
;main.c: 80: {
;main.c: 81: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	0+(_start)^080h+02h,f
	line	82
	
i1l9549:	
;main.c: 82: if(start.debounceCount >= 10 & start.released)
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
	goto	u336_21
	goto	u336_20
u336_21:
	goto	i1l9557
u336_20:
	line	84
	
i1l9551:	
;main.c: 83: {
;main.c: 84: start.pressed = 1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(_start)^080h
	bsf	status,0
	rlf	(_start)^080h,f
	line	85
	
i1l9553:	
;main.c: 85: start.released = 0;
	clrf	0+(_start)^080h+01h
	goto	i1l9557
	line	86
	
i1l5950:	
	line	87
;main.c: 86: }
;main.c: 87: }
	goto	i1l9557
	line	88
	
i1l5949:	
	line	90
;main.c: 88: else
;main.c: 89: {
;main.c: 90: start.debounceCount = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	0+(_start)^080h+02h
	line	91
	
i1l9555:	
;main.c: 91: start.released = 1;
	clrf	0+(_start)^080h+01h
	bsf	status,0
	rlf	0+(_start)^080h+01h,f
	goto	i1l9557
	line	92
	
i1l5951:	
	line	94
	
i1l9557:	
;main.c: 92: }
;main.c: 94: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	bcf	status, 5	;RP0=0, select bank0
	btfss	(101/8),(101)&7
	goto	u337_21
	goto	u337_20
u337_21:
	goto	i1l9567
u337_20:
	
i1l9559:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	
i1l9561:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l9563:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u338_21
	goto	u338_20
u338_21:
	goto	i1l9567
u338_20:
	
i1l9565:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l9567
	
i1l5953:	
	goto	i1l9567
	
i1l5952:	
	
i1l9567:	
	btfss	(100/8),(100)&7
	goto	u339_21
	goto	u339_20
u339_21:
	goto	i1l5956
u339_20:
	
i1l9569:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u340_21
	goto	u340_20
u340_21:
	goto	i1l5956
u340_20:
	
i1l9571:	
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(25)	;volatile
	
i1l9573:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l9575:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l9577:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u341_21
	goto	u341_20
u341_21:
	goto	i1l5956
u341_20:
	
i1l9579:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l5956
	
i1l5955:	
	goto	i1l5956
	
i1l5954:	
	goto	i1l5956
	line	95
	
i1l5945:	
	line	96
	
i1l5956:	
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
psect	text1930,local,class=CODE,delta=2
global __ptext1930
__ptext1930:

;; *************** function ___lwmod *****************
;; Defined at:
;;		line 5 in file "C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lwmod.c"
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
psect	text1930
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l9643:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u356_21
	goto	u356_20
u356_21:
	goto	i1l9661
u356_20:
	line	9
	
i1l9645:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l9651
	
i1l6751:	
	line	11
	
i1l9647:	
	movlw	01h
	
u357_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u357_25
	line	12
	
i1l9649:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l9651
	line	13
	
i1l6750:	
	line	10
	
i1l9651:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u358_21
	goto	u358_20
u358_21:
	goto	i1l9647
u358_20:
	goto	i1l9653
	
i1l6752:	
	goto	i1l9653
	line	14
	
i1l6753:	
	line	15
	
i1l9653:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u359_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u359_25:
	skipc
	goto	u359_21
	goto	u359_20
u359_21:
	goto	i1l9657
u359_20:
	line	16
	
i1l9655:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l9657
	
i1l6754:	
	line	17
	
i1l9657:	
	movlw	01h
	
u360_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u360_25
	line	18
	
i1l9659:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u361_21
	goto	u361_20
u361_21:
	goto	i1l9653
u361_20:
	goto	i1l9661
	
i1l6755:	
	goto	i1l9661
	line	19
	
i1l6749:	
	line	20
	
i1l9661:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l6756
	
i1l9663:	
	line	21
	
i1l6756:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text1931,local,class=CODE,delta=2
global __ptext1931
__ptext1931:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

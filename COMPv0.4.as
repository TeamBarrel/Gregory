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
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_findWalls
	FNCALL	_main,_goToNextCell
	FNCALL	_main,_getSuccessfulDrive
	FNCALL	_main,_updateLocation
	FNCALL	_main,_updateNode
	FNCALL	_main,_checkIfHome
	FNCALL	_goToNextCell,_goLeft
	FNCALL	_goToNextCell,_goForward
	FNCALL	_goToNextCell,_goRight
	FNCALL	_goToNextCell,_goBackward
	FNCALL	_findWalls,_lcd_set_cursor
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_findWalls,_rotateIR
	FNCALL	_findWalls,_frontWallCorrect
	FNCALL	_findWalls,_rightWallCorrect
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
	FNCALL	_findWall,_readIR
	FNCALL	_frontWallCorrect,_drive
	FNCALL	_frontWallCorrect,_readIR
	FNCALL	_rightWallCorrect,_turnRight90
	FNCALL	_rightWallCorrect,_rotateIR
	FNCALL	_rightWallCorrect,_drive
	FNCALL	_rightWallCorrect,_readIR
	FNCALL	_rightWallCorrect,_turnLeft90
	FNCALL	_driveForDistance,_drive
	FNCALL	_driveForDistance,_ser_putch
	FNCALL	_driveForDistance,_ser_getch
	FNCALL	_driveForDistance,_detectCliff
	FNCALL	_driveForDistance,_goReverse
	FNCALL	_updateLocation,_lcd_set_cursor
	FNCALL	_updateLocation,_lcd_write_data
	FNCALL	_updateLocation,_getOrientation
	FNCALL	_updateLocation,_lcd_write_1_digit_bcd
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
	FNCALL	_readIR,_adc_read_channel
	FNCALL	_readIR,_convert
	FNCALL	_goReverse,_lcd_set_cursor
	FNCALL	_goReverse,_lcd_write_data
	FNCALL	_goReverse,_drive
	FNCALL	_goReverse,_waitFor
	FNCALL	_goReverse,_updateOrientation
	FNCALL	_checkIfHome,_drive
	FNCALL	_checkIfHome,_play_iCreate_song
	FNCALL	_initSongs,_ser_putArr
	FNCALL	_lcd_init,_lcd_write_control
	FNCALL	_lcd_write_1_digit_bcd,_lcd_write_data
	FNCALL	_lcd_write_string,_lcd_write_data
	FNCALL	_turnRight90,_drive
	FNCALL	_turnRight90,_waitFor
	FNCALL	_turnLeft90,_drive
	FNCALL	_turnLeft90,_waitFor
	FNCALL	_turnAround,_drive
	FNCALL	_turnAround,_waitFor
	FNCALL	_lcd_set_cursor,_lcd_write_control
	FNCALL	_adc_read_channel,_adc_read
	FNCALL	_detectCliff,_ser_putch
	FNCALL	_detectCliff,_ser_getch
	FNCALL	_convert,___wmul
	FNCALL	_convert,___awdiv
	FNCALL	_ser_putArr,_ser_putch
	FNCALL	_play_iCreate_song,_ser_putch
	FNCALL	_initIRobot,_ser_putch
	FNCALL	_waitFor,_ser_putch
	FNCALL	_drive,_ser_putch
	FNCALL	_adc_read,___awdiv
	FNCALL	_ser_getch,_ser_isrx
	FNROOT	_main
	FNCALL	_isr1,___lwmod
	FNCALL	intlevel1,_isr1
	global	intlevel1
	FNROOT	intlevel1
	global	_xCoord
	global	_yCoord
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
psect	idataCOMMON,class=CODE,space=0,delta=2
global __pidataCOMMON
__pidataCOMMON:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	36

;initializer for _xCoord
	retlw	01h
	line	37

;initializer for _yCoord
	retlw	03h
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
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
	global	_rxfifo
	global	_start
	global	_RTC_Counter
	global	_currentOrientation
	global	_finalX
	global	_finalY
	global	_lastMove
	global	_node
	global	_rxoptr
	global	_sensorData
	global	_ser_tmp
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
	global	_rightWall
	global	_sensorDetected
	global	_successfulDrive
	global	_victimFound
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

_sensorDetected:
       ds      1

_successfulDrive:
       ds      1

_victimFound:
       ds      1

psect	bssCOMMON,class=COMMON,space=1
global __pbssCOMMON
__pbssCOMMON:
_rxiptr:
       ds      1

_txiptr:
       ds      1

psect	dataCOMMON,class=COMMON,space=1
global __pdataCOMMON
__pdataCOMMON:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	36
_xCoord:
       ds      1

psect	dataCOMMON
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	37
_yCoord:
       ds      1

psect	bssBANK0,class=BANK0,space=1
global __pbssBANK0
__pbssBANK0:
_rxfifo:
       ds      16

_start:
       ds      3

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

_node:
       ds      1

_rxoptr:
       ds      1

_sensorData:
       ds      1

_ser_tmp:
       ds      1

_txoptr:
       ds      1

_vicZone:
       ds      1

_victimZone:
       ds      1

_wayWent:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_txfifo:
       ds      16

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK1
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
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
	movlw	low((__pbssBANK0)+021h)
	fcall	clear_ram
; Clear objects allocated to BANK1
psect cinit,class=CODE,delta=2
	movlw	low(__pbssBANK1)
	movwf	fsr
	movlw	low((__pbssBANK1)+010h)
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
; Initialize objects allocated to COMMON
	global __pidataCOMMON
psect cinit,class=CODE,delta=2
	fcall	__pidataCOMMON+0		;fetch initializer
	movwf	__pdataCOMMON+0&07fh		
	fcall	__pidataCOMMON+1		;fetch initializer
	movwf	__pdataCOMMON+1&07fh		
psect cinit,class=CODE,delta=2
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
clrf status
ljmp _main	;jump to C main() function
psect	cstackBANK1,class=BANK1,space=1
global __pcstackBANK1
__pcstackBANK1:
	global	?_adc_read_channel
?_adc_read_channel:	; 2 bytes @ 0x0
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x2
	ds	1
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x3
	ds	1
	global	?_readIR
?_readIR:	; 2 bytes @ 0x4
	ds	2
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x6
	ds	2
	global	??_rightWallCorrect
??_rightWallCorrect:	; 0 bytes @ 0x8
	global	??_frontWallCorrect
??_frontWallCorrect:	; 0 bytes @ 0x8
	ds	2
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0xA
	ds	1
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
	global	?_goReverse
?_goReverse:	; 0 bytes @ 0x0
	global	?_lcd_set_cursor
?_lcd_set_cursor:	; 0 bytes @ 0x0
	global	?_lcd_write_data
?_lcd_write_data:	; 0 bytes @ 0x0
	global	?_turnAround
?_turnAround:	; 0 bytes @ 0x0
	global	?_updateOrientation
?_updateOrientation:	; 0 bytes @ 0x0
	global	?_turnLeft90
?_turnLeft90:	; 0 bytes @ 0x0
	global	?_turnRight90
?_turnRight90:	; 0 bytes @ 0x0
	global	?_ser_init
?_ser_init:	; 0 bytes @ 0x0
	global	?_initIRobot
?_initIRobot:	; 0 bytes @ 0x0
	global	?_initSongs
?_initSongs:	; 0 bytes @ 0x0
	global	?_play_iCreate_song
?_play_iCreate_song:	; 0 bytes @ 0x0
	global	?_findWall
?_findWall:	; 1 bit 
	global	?_detectCliff
?_detectCliff:	; 1 bit 
	global	?_init_adc
?_init_adc:	; 0 bytes @ 0x0
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
	global	?_rightWallCorrect
?_rightWallCorrect:	; 0 bytes @ 0x0
	global	?_frontWallCorrect
?_frontWallCorrect:	; 0 bytes @ 0x0
	global	?_lcd_write_control
?_lcd_write_control:	; 0 bytes @ 0x0
	global	?_lcd_write_string
?_lcd_write_string:	; 0 bytes @ 0x0
	global	?_lcd_write_1_digit_bcd
?_lcd_write_1_digit_bcd:	; 0 bytes @ 0x0
	global	?_lcd_init
?_lcd_init:	; 0 bytes @ 0x0
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
	global	?_ser_isrx
?_ser_isrx:	; 1 bit 
	global	?_ser_getch
?_ser_getch:	; 1 bytes @ 0x0
	global	?_getFinalX
?_getFinalX:	; 1 bytes @ 0x0
	global	?_getFinalY
?_getFinalY:	; 1 bytes @ 0x0
	global	?_getOrientation
?_getOrientation:	; 1 bytes @ 0x0
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
	global	??_ser_getch
??_ser_getch:	; 0 bytes @ 0xA
	global	??_lcd_write_data
??_lcd_write_data:	; 0 bytes @ 0xA
	global	??_updateOrientation
??_updateOrientation:	; 0 bytes @ 0xA
	global	?_rotateIR
?_rotateIR:	; 0 bytes @ 0xA
	global	??_ser_init
??_ser_init:	; 0 bytes @ 0xA
	global	??_getFinalX
??_getFinalX:	; 0 bytes @ 0xA
	global	??_getFinalY
??_getFinalY:	; 0 bytes @ 0xA
	global	??_init_adc
??_init_adc:	; 0 bytes @ 0xA
	global	??_getSuccessfulDrive
??_getSuccessfulDrive:	; 0 bytes @ 0xA
	global	??_getOrientation
??_getOrientation:	; 0 bytes @ 0xA
	global	??_lcd_write_control
??_lcd_write_control:	; 0 bytes @ 0xA
	global	??_updateNode
??_updateNode:	; 0 bytes @ 0xA
	global	??_ser_isrx
??_ser_isrx:	; 0 bytes @ 0xA
	global	?_getVictimZone
?_getVictimZone:	; 1 bytes @ 0xA
	global	?___wmul
?___wmul:	; 2 bytes @ 0xA
	global	getVictimZone@victimY
getVictimZone@victimY:	; 1 bytes @ 0xA
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	ds	1
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0xB
	global	??_getVictimZone
??_getVictimZone:	; 0 bytes @ 0xB
	global	updateOrientation@moved
updateOrientation@moved:	; 1 bytes @ 0xB
	global	ser_getch@c
ser_getch@c:	; 1 bytes @ 0xB
	global	ser_putch@c
ser_putch@c:	; 1 bytes @ 0xB
	ds	1
	global	?_waitFor
?_waitFor:	; 0 bytes @ 0xC
	global	??_initIRobot
??_initIRobot:	; 0 bytes @ 0xC
	global	??_play_iCreate_song
??_play_iCreate_song:	; 0 bytes @ 0xC
	global	??_detectCliff
??_detectCliff:	; 0 bytes @ 0xC
	global	?_drive
?_drive:	; 0 bytes @ 0xC
	global	?_ser_putArr
?_ser_putArr:	; 0 bytes @ 0xC
	global	drive@lowByteSpeed
drive@lowByteSpeed:	; 1 bytes @ 0xC
	global	waitFor@highByte
waitFor@highByte:	; 1 bytes @ 0xC
	global	lcd_write_control@databyte
lcd_write_control@databyte:	; 1 bytes @ 0xC
	global	lcd_write_data@databyte
lcd_write_data@databyte:	; 1 bytes @ 0xC
	global	getVictimZone@victimX
getVictimZone@victimX:	; 1 bytes @ 0xC
	global	play_iCreate_song@song
play_iCreate_song@song:	; 1 bytes @ 0xC
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
	global	drive@highByteRadius
drive@highByteRadius:	; 1 bytes @ 0xD
	global	waitFor@lowByte
waitFor@lowByte:	; 1 bytes @ 0xD
	global	lcd_set_cursor@address
lcd_set_cursor@address:	; 1 bytes @ 0xD
	global	lcd_write_1_digit_bcd@data
lcd_write_1_digit_bcd@data:	; 1 bytes @ 0xD
	ds	1
	global	??_waitFor
??_waitFor:	; 0 bytes @ 0xE
	global	??_checkForFinalDestination
??_checkForFinalDestination:	; 0 bytes @ 0xE
	global	??_lookForVictim
??_lookForVictim:	; 0 bytes @ 0xE
	global	??_updateLocation
??_updateLocation:	; 0 bytes @ 0xE
	global	??___wmul
??___wmul:	; 0 bytes @ 0xE
	global	drive@lowByteRadius
drive@lowByteRadius:	; 1 bytes @ 0xE
	global	lcd_write_string@s
lcd_write_string@s:	; 1 bytes @ 0xE
	global	rotateIR@steps
rotateIR@steps:	; 1 bytes @ 0xE
	global	ser_putArr@length
ser_putArr@length:	; 2 bytes @ 0xE
	global	___wmul@product
___wmul@product:	; 2 bytes @ 0xE
	ds	1
	global	??_drive
??_drive:	; 0 bytes @ 0xF
	global	rotateIR@stepNum
rotateIR@stepNum:	; 1 bytes @ 0xF
	ds	1
	global	??_ser_putArr
??_ser_putArr:	; 0 bytes @ 0x10
	global	?___awdiv
?___awdiv:	; 2 bytes @ 0x10
	global	___awdiv@divisor
___awdiv@divisor:	; 2 bytes @ 0x10
	ds	1
	global	waitFor@type
waitFor@type:	; 1 bytes @ 0x11
	ds	1
	global	drive@highByteSpeed
drive@highByteSpeed:	; 1 bytes @ 0x12
	global	___awdiv@dividend
___awdiv@dividend:	; 2 bytes @ 0x12
	ds	1
	global	??_goReverse
??_goReverse:	; 0 bytes @ 0x13
	global	??_turnAround
??_turnAround:	; 0 bytes @ 0x13
	global	??_turnLeft90
??_turnLeft90:	; 0 bytes @ 0x13
	global	??_turnRight90
??_turnRight90:	; 0 bytes @ 0x13
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
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0x15
	global	??_init
??_init:	; 0 bytes @ 0x15
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0x15
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0x15
	ds	1
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0x16
	ds	1
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0x17
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0x17
	ds	2
	global	?_adc_read
?_adc_read:	; 2 bytes @ 0x19
	global	driveForDistance@deltaDistance
driveForDistance@deltaDistance:	; 2 bytes @ 0x19
	ds	2
	global	??_adc_read
??_adc_read:	; 0 bytes @ 0x1B
	global	driveForDistance@distance
driveForDistance@distance:	; 2 bytes @ 0x1B
	ds	2
	global	driveForDistance@high
driveForDistance@high:	; 1 bytes @ 0x1D
	ds	1
	global	driveForDistance@low
driveForDistance@low:	; 1 bytes @ 0x1E
	ds	1
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x1F
	global	??_goForward
??_goForward:	; 0 bytes @ 0x1F
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x1F
	global	??_goRight
??_goRight:	; 0 bytes @ 0x1F
	global	adc_read@adc_value
adc_read@adc_value:	; 2 bytes @ 0x1F
	ds	1
	global	??_goToNextCell
??_goToNextCell:	; 0 bytes @ 0x20
	ds	1
	global	?_convert
?_convert:	; 2 bytes @ 0x21
	global	convert@adc_value
convert@adc_value:	; 2 bytes @ 0x21
	ds	2
	global	??_convert
??_convert:	; 0 bytes @ 0x23
	ds	2
	global	??_readIR
??_readIR:	; 0 bytes @ 0x25
	global	??_findWall
??_findWall:	; 0 bytes @ 0x25
	global	??_main
??_main:	; 0 bytes @ 0x25
;;Data sizes: Strings 34, constant 0, data 104, bss 51, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     37      70
;; BANK1           80     11      73
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
;; ?_lcd_write_1_bcd	int  size(1) Largest target is 0
;;
;; ?___lwmod	unsigned int  size(1) Largest target is 0
;;
;; ?_adc_read	int  size(1) Largest target is 0
;;
;; ?___awdiv	int  size(1) Largest target is 0
;;
;; ?_adc_read_channel	int  size(1) Largest target is 0
;;
;; ser_putArr@array	PTR unsigned char  size(2) Largest target is 29
;;		 -> champions(BANK1[21]), lookingForU2(BANK3[29]), superMarioBros(BANK1[25]), finalCountdown(BANK3[27]), 
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
;;   _goToNextCell->_goRight
;;   _goToNextCell->_goBackward
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _driveForDistance->_goReverse
;;   _updateLocation->_lcd_set_cursor
;;   _updateLocation->_lcd_write_1_digit_bcd
;;   _lookForVictim->_lcd_set_cursor
;;   _lookForVictim->_lcd_write_1_digit_bcd
;;   _checkForFinalDestination->_lcd_set_cursor
;;   _readIR->_convert
;;   _goReverse->_drive
;;   _checkIfHome->_drive
;;   _initSongs->_ser_putArr
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_write_string->_lcd_write_data
;;   _turnRight90->_drive
;;   _turnLeft90->_drive
;;   _turnAround->_drive
;;   _lcd_set_cursor->_lcd_write_control
;;   _adc_read_channel->_convert
;;   _detectCliff->_ser_putch
;;   _detectCliff->_ser_getch
;;   _convert->_adc_read
;;   _ser_putArr->_ser_putch
;;   _play_iCreate_song->_ser_putch
;;   _initIRobot->_ser_putch
;;   _waitFor->_ser_putch
;;   _drive->_ser_putch
;;   _adc_read->___awdiv
;;   ___awdiv->___wmul
;;
;; Critical Paths under _isr1 in BANK0
;;
;;   None.
;;
;; Critical Paths under _main in BANK1
;;
;;   _main->_findWalls
;;   _findWalls->_frontWallCorrect
;;   _findWalls->_rightWallCorrect
;;   _findWall->_readIR
;;   _frontWallCorrect->_readIR
;;   _rightWallCorrect->_readIR
;;   _readIR->_adc_read_channel
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
;;Main: autosize = 0, tempsize = 0, incstack = 0, save=0
;;

;;
;;Call Graph Tables:
;;
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (0) _main                                                 0     0      0   11994
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                          _findWalls
;;                       _goToNextCell
;;                 _getSuccessfulDrive
;;                     _updateLocation
;;                         _updateNode
;;                        _checkIfHome
;; ---------------------------------------------------------------------------------
;; (1) _goToNextCell                                         0     0      0    4770
;;                             _goLeft
;;                          _goForward
;;                            _goRight
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (1) _findWalls                                            1     1      0    5746
;;                                             10 BANK1      1     1      0
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                           _rotateIR
;;                   _frontWallCorrect
;;                   _rightWallCorrect
;; ---------------------------------------------------------------------------------
;; (2) _goRight                                              1     1      0    1270
;;                                             31 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goLeft                                               0     0      0    1270
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goForward                                            0     0      0     960
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0    1270
;;                                             31 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnAround
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1528
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (2) _frontWallCorrect                                     2     2      0    1683
;;                                              8 BANK1      2     2      0
;;                              _drive
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (2) _rightWallCorrect                                     2     2      0    2340
;;                                              8 BANK1      2     2      0
;;                        _turnRight90
;;                           _rotateIR
;;                              _drive
;;                             _readIR
;;                         _turnLeft90
;; ---------------------------------------------------------------------------------
;; (3) _driveForDistance                                    10     8      2     864
;;                                             21 BANK0     10     8      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;;                        _detectCliff
;;                          _goReverse
;; ---------------------------------------------------------------------------------
;; (1) _updateLocation                                       1     1      0     158
;;                                             14 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                     _getOrientation
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (1) _lookForVictim                                        2     2      0     406
;;                                             14 BANK0      2     2      0
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
;; (3) _readIR                                               4     2      2    1528
;;                                              4 BANK1      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (4) _goReverse                                            2     2      0     406
;;                                             19 BANK0      2     2      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                              _drive
;;                            _waitFor
;;                  _updateOrientation
;; ---------------------------------------------------------------------------------
;; (1) _checkIfHome                                          0     0      0     217
;;                              _drive
;;                  _play_iCreate_song
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
;; (3) _turnRight90                                          3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnLeft90                                           3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (3) _turnAround                                           3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (2) _lcd_set_cursor                                       1     1      0      65
;;                                             13 BANK0      1     1      0
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (4) _adc_read_channel                                     4     2      2     510
;;                                              0 BANK1      4     2      2
;;                           _adc_read
;;                            _convert (ARG)
;; ---------------------------------------------------------------------------------
;; (4) _detectCliff                                          1     1      0      65
;;                                             12 BANK0      1     1      0
;;                          _ser_putch
;;                          _ser_getch
;; ---------------------------------------------------------------------------------
;; (4) _convert                                              4     2      2     984
;;                                             33 BANK0      4     2      2
;;                             ___wmul
;;                            ___awdiv
;;                           _adc_read (ARG)
;; ---------------------------------------------------------------------------------
;; (3) _ser_putArr                                           9     5      4     161
;;                                             12 BANK0      9     5      4
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _play_iCreate_song                                    1     1      0      62
;;                                             12 BANK0      1     1      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _initIRobot                                           3     3      0      31
;;                                             12 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_control                                    3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (3) _rotateIR                                             6     5      1      99
;;                                             10 BANK0      6     5      1
;; ---------------------------------------------------------------------------------
;; (5) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_data                                       3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (2) _drive                                                7     4      3     155
;;                                             12 BANK0      7     4      3
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _init_adc                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (5) _adc_read                                             8     6      2     479
;;                                             25 BANK0      8     6      2
;;                            ___awdiv
;; ---------------------------------------------------------------------------------
;; (5) _ser_getch                                            2     2      0      34
;;                                             10 BANK0      2     2      0
;;                           _ser_isrx
;; ---------------------------------------------------------------------------------
;; (5) ___awdiv                                              9     5      4     445
;;                                             16 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (5) ___wmul                                               6     2      4     136
;;                                             10 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (6) _ser_isrx                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (1) _updateNode                                           1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (2) _getVictimZone                                        3     2      1     186
;;                                             10 BANK0      3     2      1
;; ---------------------------------------------------------------------------------
;; (2) _getFinalY                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _getFinalX                                            0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _ser_init                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (5) _updateOrientation                                    2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (2) _getOrientation                                       0     0      0       0
;; ---------------------------------------------------------------------------------
;; (1) _getSuccessfulDrive                                   0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _ser_putch                                            2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 6
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
;;     _play_iCreate_song
;;       _ser_putch
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _getVictimZone
;;     _lcd_write_1_digit_bcd
;;       _lcd_write_data
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
;;     _frontWallCorrect
;;       _drive
;;         _ser_putch
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
;;     _rightWallCorrect
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _rotateIR
;;       _drive
;;         _ser_putch
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
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;   _goToNextCell
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
;;         _detectCliff
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;           _updateOrientation
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
;;         _detectCliff
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;           _updateOrientation
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
;;         _detectCliff
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;           _updateOrientation
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
;;         _detectCliff
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _goReverse
;;           _lcd_set_cursor
;;             _lcd_write_control
;;           _lcd_write_data
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;           _updateOrientation
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
;;BANK1               50      B      49       7       91.3%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      DD      12        0.0%
;;ABS                  0      0      D3       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       A       2        0.0%
;;BANK0               50     25      46       5       87.5%
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
;;		line 257 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       0       0       0
;;Total ram usage:        0 bytes
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_init
;;		_drive
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
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
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	257
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	258
	
l11776:	
;main.c: 258: init();
	fcall	_init
	line	259
;main.c: 259: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	261
	
l11778:	
;main.c: 261: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	262
	
l11780:	
;main.c: 262: lcd_write_string("(-,-) E -- --- -");
	movlw	((STR_1-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	263
;main.c: 263: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	264
	
l11782:	
;main.c: 264: lcd_write_string("- - - (0,0) GREG");
	movlw	((STR_2-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	268
;main.c: 268: while(!home)
	goto	l11806
	
l3769:	
	line	270
	
l11784:	
;main.c: 269: {
;main.c: 270: if(start.pressed)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u5000
	goto	l11806
u5000:
	line	272
	
l11786:	
;main.c: 271: {
;main.c: 272: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	273
;main.c: 273: lookForVictim();
	fcall	_lookForVictim
	line	274
	
l11788:	
;main.c: 274: findWalls();
	fcall	_findWalls
	line	275
;main.c: 275: switch(node)
	goto	l11794
	line	277
;main.c: 276: {
;main.c: 277: case 1:
	
l3772:	
	line	279
;main.c: 279: break;
	goto	l11796
	line	280
;main.c: 280: case 2:
	
l3774:	
	line	282
;main.c: 282: break;
	goto	l11796
	line	283
;main.c: 283: case 3:
	
l3775:	
	line	285
;main.c: 285: break;
	goto	l11796
	line	286
;main.c: 286: default:
	
l3776:	
	line	287
	
l11790:	
;main.c: 287: goToNextCell();
	fcall	_goToNextCell
	line	288
;main.c: 288: break;
	goto	l11796
	line	289
	
l11792:	
;main.c: 289: }
	goto	l11796
	line	275
	
l3771:	
	
l11794:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_node),w	;volatile
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
	goto	l11796
	xorlw	2^1	; case 2
	skipnz
	goto	l11796
	xorlw	3^2	; case 3
	skipnz
	goto	l11796
	goto	l11790
	opt asmopt_on

	line	289
	
l3773:	
	line	290
	
l11796:	
;main.c: 290: if(getSuccessfulDrive());
	fcall	_getSuccessfulDrive
	goto	l11798
	
l3777:	
	line	293
	
l11798:	
;main.c: 291: {
;main.c: 293: updateLocation();
	fcall	_updateLocation
	line	294
	
l11800:	
;main.c: 294: updateNode();
	fcall	_updateNode
	line	295
	
l11802:	
;main.c: 295: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u5011
	goto	u5010
u5011:
	goto	l11806
u5010:
	line	296
	
l11804:	
;main.c: 296: checkIfHome();
	fcall	_checkIfHome
	goto	l11806
	
l3778:	
	goto	l11806
	line	298
	
l3770:	
	goto	l11806
	line	299
	
l3768:	
	line	268
	
l11806:	
	btfss	(_home/8),(_home)&7
	goto	u5021
	goto	u5020
u5021:
	goto	l11784
u5020:
	goto	l3780
	
l3779:	
	line	301
	
l3780:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_goToNextCell
psect	text1622,local,class=CODE,delta=2
global __ptext1622
__ptext1622:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 190 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
;;		_goLeft
;;		_goForward
;;		_goRight
;;		_goBackward
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1622
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	190
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 0
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	191
	
l11766:	
;main.c: 191: if(!leftWall)
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u4971
	goto	u4970
u4971:
	goto	l3736
u4970:
	line	192
	
l11768:	
;main.c: 192: goLeft();
	fcall	_goLeft
	goto	l3742
	line	193
	
l3736:	
;main.c: 193: else if(!frontWall)
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u4981
	goto	u4980
u4981:
	goto	l3738
u4980:
	line	194
	
l11770:	
;main.c: 194: goForward();
	fcall	_goForward
	goto	l3742
	line	195
	
l3738:	
;main.c: 195: else if(!rightWall)
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u4991
	goto	u4990
u4991:
	goto	l11774
u4990:
	line	196
	
l11772:	
;main.c: 196: goRight();
	fcall	_goRight
	goto	l3742
	line	197
	
l3740:	
	line	198
	
l11774:	
;main.c: 197: else
;main.c: 198: goBackward();
	fcall	_goBackward
	goto	l3742
	
l3741:	
	goto	l3742
	
l3739:	
	goto	l3742
	
l3737:	
	line	199
	
l3742:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1623,local,class=CODE,delta=2
global __ptext1623
__ptext1623:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 156 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
;;      Temps:          0       0       1       0       0
;;      Totals:         0       0       1       0       0
;;Total ram usage:        1 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_lcd_set_cursor
;;		_findWall
;;		_lcd_write_data
;;		_rotateIR
;;		_frontWallCorrect
;;		_rightWallCorrect
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1623
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	156
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	157
	
l11736:	
;main.c: 157: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	159
	
l11738:	
;main.c: 159: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4881
	goto	u4880
	
u4881:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u4894
u4880:
	bcf	(_leftWall/8),(_leftWall)&7
u4894:
	line	160
	
l11740:	
;main.c: 160: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4901
	goto	u4900
u4901:
	goto	l11744
u4900:
	line	162
	
l11742:	
;main.c: 161: {
;main.c: 162: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	163
;main.c: 163: }
	goto	l3728
	line	164
	
l3727:	
	line	165
	
l11744:	
;main.c: 164: else
;main.c: 165: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3728:	
	line	166
;main.c: 166: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	168
	
l11746:	
;main.c: 168: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4911
	goto	u4910
	
u4911:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u4924
u4910:
	bcf	(_frontWall/8),(_frontWall)&7
u4924:
	line	169
	
l11748:	
;main.c: 169: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4931
	goto	u4930
u4931:
	goto	l11754
u4930:
	line	171
	
l11750:	
;main.c: 170: {
;main.c: 171: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	172
	
l11752:	
;main.c: 172: frontWallCorrect();
	fcall	_frontWallCorrect
	line	173
;main.c: 173: }
	goto	l3730
	line	174
	
l3729:	
	line	175
	
l11754:	
;main.c: 174: else
;main.c: 175: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3730:	
	line	176
;main.c: 176: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	178
	
l11756:	
;main.c: 178: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4941
	goto	u4940
	
u4941:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u4954
u4940:
	bcf	(_rightWall/8),(_rightWall)&7
u4954:
	line	179
	
l11758:	
;main.c: 179: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u4961
	goto	u4960
u4961:
	goto	l11764
u4960:
	line	181
	
l11760:	
;main.c: 180: {
;main.c: 181: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	182
	
l11762:	
;main.c: 182: rightWallCorrect();
	fcall	_rightWallCorrect
	line	183
;main.c: 183: }
	goto	l3732
	line	184
	
l3731:	
	line	185
	
l11764:	
;main.c: 184: else
;main.c: 185: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3732:	
	line	186
;main.c: 186: rotateIR(48, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(030h)
	fcall	_rotateIR
	line	187
	
l3733:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goRight
psect	text1624,local,class=CODE,delta=2
global __ptext1624
__ptext1624:

;; *************** function _goRight *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1624
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	147
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 0
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	148
	
l11726:	
;drive.c: 148: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	149
;drive.c: 149: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	150
	
l11728:	
;drive.c: 150: turnRight90();
	fcall	_turnRight90
	line	151
	
l11730:	
;drive.c: 151: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	152
	
l11732:	
;drive.c: 152: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	153
	
l11734:	
;drive.c: 153: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	154
	
l2164:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1625,local,class=CODE,delta=2
global __ptext1625
__ptext1625:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 113 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1625
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	113
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 0
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	114
	
l11716:	
;drive.c: 114: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	115
;drive.c: 115: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	116
	
l11718:	
;drive.c: 116: turnLeft90();
	fcall	_turnLeft90
	line	117
	
l11720:	
;drive.c: 117: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	118
	
l11722:	
;drive.c: 118: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	119
	
l11724:	
;drive.c: 119: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	120
	
l2155:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1626,local,class=CODE,delta=2
global __ptext1626
__ptext1626:

;; *************** function _goForward *****************
;; Defined at:
;;		line 104 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1626
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	104
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 0
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	105
	
l11710:	
;drive.c: 105: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	106
;drive.c: 106: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	107
	
l11712:	
;drive.c: 107: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	108
	
l11714:	
;drive.c: 108: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	109
	
l2152:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1627,local,class=CODE,delta=2
global __ptext1627
__ptext1627:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 93 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1627
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	93
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 0
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	94
	
l11700:	
;drive.c: 94: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	95
;drive.c: 95: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	96
	
l11702:	
;drive.c: 96: turnAround();
	fcall	_turnAround
	line	97
	
l11704:	
;drive.c: 97: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	98
	
l11706:	
;drive.c: 98: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	99
	
l11708:	
;drive.c: 99: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	100
	
l2149:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_findWall
psect	text1628,local,class=CODE,delta=2
global __ptext1628
__ptext1628:

;; *************** function _findWall *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
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
psect	text1628
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
	line	32
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	33
	
l11688:	
;ir.c: 33: if(readIR() > 100)
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4875
	movlw	low(065h)
	subwf	(0+(?_readIR))^080h,w
u4875:

	skipc
	goto	u4871
	goto	u4870
u4871:
	goto	l11696
u4870:
	line	34
	
l11690:	
;ir.c: 34: return 0;
	clrc
	
	goto	l6715
	
l11692:	
	goto	l6715
	
l11694:	
	goto	l6715
	line	35
	
l6714:	
	line	36
	
l11696:	
;ir.c: 35: else
;ir.c: 36: return 1;
	setc
	
	goto	l6715
	
l11698:	
	goto	l6715
	
l6716:	
	line	37
	
l6715:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text1629,local,class=CODE,delta=2
global __ptext1629
__ptext1629:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 214 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;      Temps:          0       0       2       0       0
;;      Totals:         0       0       2       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_drive
;;		_readIR
;; This function is called by:
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text1629
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	214
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 0
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	215
	
l11672:	
;drive.c: 215: while(readIR() < 50)
	goto	l11676
	
l2193:	
	line	217
	
l11674:	
;drive.c: 216: {
;drive.c: 217: drive(255, 131, 128, 0);
	movlw	(083h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_frontWallCorrect+0)^080h+0
	movf	(??_frontWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_drive)
	movlw	(080h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_frontWallCorrect+1)^080h+0
	movf	(??_frontWallCorrect+1)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0FFh)
	fcall	_drive
	goto	l11676
	line	218
	
l2192:	
	line	215
	
l11676:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(032h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4845
	movlw	low(032h)
	subwf	(0+(?_readIR))^080h,w
u4845:

	skipc
	goto	u4841
	goto	u4840
u4841:
	goto	l11674
u4840:
	goto	l11678
	
l2194:	
	line	219
	
l11678:	
;drive.c: 218: }
;drive.c: 219: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	221
;drive.c: 221: while(readIR() > 55 && readIR() < 100)
	goto	l11682
	
l2196:	
	line	223
	
l11680:	
;drive.c: 222: {
;drive.c: 223: drive(0, 250, 128, 0);
	movlw	(0FAh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_frontWallCorrect+0)^080h+0
	movf	(??_frontWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_drive)
	movlw	(080h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_frontWallCorrect+1)^080h+0
	movf	(??_frontWallCorrect+1)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l11682
	line	224
	
l2195:	
	line	221
	
l11682:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4855
	movlw	low(038h)
	subwf	(0+(?_readIR))^080h,w
u4855:

	skipc
	goto	u4851
	goto	u4850
u4851:
	goto	l11686
u4850:
	
l11684:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4865
	movlw	low(064h)
	subwf	(0+(?_readIR))^080h,w
u4865:

	skipc
	goto	u4861
	goto	u4860
u4861:
	goto	l11680
u4860:
	goto	l11686
	
l2198:	
	goto	l11686
	
l2199:	
	line	225
	
l11686:	
;drive.c: 224: }
;drive.c: 225: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	226
	
l2200:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_rightWallCorrect
psect	text1630,local,class=CODE,delta=2
global __ptext1630
__ptext1630:

;; *************** function _rightWallCorrect *****************
;; Defined at:
;;		line 197 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;      Temps:          0       0       2       0       0
;;      Totals:         0       0       2       0       0
;;Total ram usage:        2 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    6
;; This function calls:
;;		_turnRight90
;;		_rotateIR
;;		_drive
;;		_readIR
;;		_turnLeft90
;; This function is called by:
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text1630
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	197
	global	__size_of_rightWallCorrect
	__size_of_rightWallCorrect	equ	__end_of_rightWallCorrect-_rightWallCorrect
	
_rightWallCorrect:	
	opt	stack 0
; Regs used in _rightWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	198
	
l11654:	
;drive.c: 198: turnRight90();
	fcall	_turnRight90
	line	199
	
l11656:	
;drive.c: 199: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+0)^080h+0
	movf	(??_rightWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	200
;drive.c: 200: while(readIR() <45)
	goto	l11660
	
l2184:	
	line	202
	
l11658:	
;drive.c: 201: {
;drive.c: 202: drive(255, 131, 128, 0);
	movlw	(083h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+0)^080h+0
	movf	(??_rightWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_drive)
	movlw	(080h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+1)^080h+0
	movf	(??_rightWallCorrect+1)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0FFh)
	fcall	_drive
	goto	l11660
	line	203
	
l2183:	
	line	200
	
l11660:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4825
	movlw	low(02Dh)
	subwf	(0+(?_readIR))^080h,w
u4825:

	skipc
	goto	u4821
	goto	u4820
u4821:
	goto	l11658
u4820:
	goto	l11664
	
l2185:	
	line	204
;drive.c: 203: }
;drive.c: 204: while(readIR() >55)
	goto	l11664
	
l2187:	
	line	206
	
l11662:	
;drive.c: 205: {
;drive.c: 206: drive(0, 250, 128, 0);
	movlw	(0FAh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+0)^080h+0
	movf	(??_rightWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_drive)
	movlw	(080h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+1)^080h+0
	movf	(??_rightWallCorrect+1)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l11664
	line	207
	
l2186:	
	line	204
	
l11664:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4835
	movlw	low(038h)
	subwf	(0+(?_readIR))^080h,w
u4835:

	skipnc
	goto	u4831
	goto	u4830
u4831:
	goto	l11662
u4830:
	goto	l11666
	
l2188:	
	line	208
	
l11666:	
;drive.c: 207: }
;drive.c: 208: turnLeft90();
	fcall	_turnLeft90
	line	209
	
l11668:	
;drive.c: 209: rotateIR(24, 0b00001111);
	movlw	(0Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_rightWallCorrect+0)^080h+0
	movf	(??_rightWallCorrect+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	210
	
l11670:	
;drive.c: 210: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	211
	
l2189:	
	return
	opt stack 0
GLOBAL	__end_of_rightWallCorrect
	__end_of_rightWallCorrect:
;; =============== function _rightWallCorrect ends ============

	signat	_rightWallCorrect,88
	global	_driveForDistance
psect	text1631,local,class=CODE,delta=2
global __ptext1631
__ptext1631:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 35 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
;; Parameters:    Size  Location     Type
;;  moveDistance    2   21[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  distance        2   27[BANK0 ] int 
;;  deltaDistanc    2   25[BANK0 ] int 
;;  low             1   30[BANK0 ] volatile unsigned char 
;;  high            1   29[BANK0 ] volatile unsigned char 
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
;;      Locals:         0       6       0       0       0
;;      Temps:          0       2       0       0       0
;;      Totals:         0      10       0       0       0
;;Total ram usage:       10 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_drive
;;		_ser_putch
;;		_ser_getch
;;		_detectCliff
;;		_goReverse
;; This function is called by:
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1631
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	35
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 0
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	38
	
l11626:	
;drive.c: 37: volatile char high, low;
;drive.c: 38: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	39
;drive.c: 39: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	41
	
l11628:	
;drive.c: 41: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	42
	
l11630:	
;drive.c: 42: drive(0, 250, 128, 0);
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
	line	44
;drive.c: 44: while(moving)
	goto	l11652
	
l2136:	
	line	46
	
l11632:	
;drive.c: 45: {
;drive.c: 46: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	47
;drive.c: 47: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	48
;drive.c: 48: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	49
;drive.c: 49: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	50
	
l11634:	
;drive.c: 50: deltaDistance = high*256 + low;
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
	line	51
	
l11636:	
;drive.c: 51: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	53
	
l11638:	
;drive.c: 53: if(detectCliff())
	fcall	_detectCliff
	btfss	status,0
	goto	u4791
	goto	u4790
u4791:
	goto	l11644
u4790:
	line	55
	
l11640:	
;drive.c: 54: {
;drive.c: 55: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	56
;drive.c: 56: goReverse();
	fcall	_goReverse
	line	58
	
l11642:	
;drive.c: 58: successfulDrive = 0;
	bcf	(_successfulDrive/8),(_successfulDrive)&7
	line	59
;drive.c: 59: break;
	goto	l2140
	line	60
	
l2137:	
	line	72
	
l11644:	
;drive.c: 60: }
;drive.c: 72: if(distance >= moveDistance)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u4805
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u4805:

	skipc
	goto	u4801
	goto	u4800
u4801:
	goto	l11652
u4800:
	line	74
	
l11646:	
;drive.c: 73: {
;drive.c: 74: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	75
	
l11648:	
;drive.c: 75: moving = 0;
	bcf	(_moving/8),(_moving)&7
	line	76
	
l11650:	
;drive.c: 76: successfulDrive = 1;
	bsf	(_successfulDrive/8),(_successfulDrive)&7
	goto	l11652
	line	77
	
l2139:	
	goto	l11652
	line	78
	
l2135:	
	line	44
	
l11652:	
	btfsc	(_moving/8),(_moving)&7
	goto	u4811
	goto	u4810
u4811:
	goto	l11632
u4810:
	goto	l2140
	
l2138:	
	line	79
	
l2140:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_updateLocation
psect	text1632,local,class=CODE,delta=2
global __ptext1632
__ptext1632:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 202 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1632
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	202
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	203
	
l11602:	
;main.c: 203: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	204
;main.c: 204: switch(getOrientation())
	goto	l11622
	line	206
;main.c: 205: {
;main.c: 206: case NORTH:
	
l3746:	
	line	207
	
l11604:	
;main.c: 207: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	208
	
l11606:	
;main.c: 208: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	209
;main.c: 209: break;
	goto	l11624
	line	210
;main.c: 210: case SOUTH:
	
l3748:	
	line	211
	
l11608:	
;main.c: 211: --yCoord;
	movlw	low(01h)
	subwf	(_yCoord),f	;volatile
	line	212
	
l11610:	
;main.c: 212: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	213
;main.c: 213: break;
	goto	l11624
	line	214
;main.c: 214: case EAST:
	
l3749:	
	line	215
	
l11612:	
;main.c: 215: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	216
	
l11614:	
;main.c: 216: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	217
;main.c: 217: break;
	goto	l11624
	line	218
;main.c: 218: case WEST:
	
l3750:	
	line	219
	
l11616:	
;main.c: 219: --xCoord;
	movlw	low(01h)
	subwf	(_xCoord),f	;volatile
	line	220
	
l11618:	
;main.c: 220: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	221
;main.c: 221: break;
	goto	l11624
	line	222
;main.c: 222: default:
	
l3751:	
	line	223
;main.c: 223: break;
	goto	l11624
	line	224
	
l11620:	
;main.c: 224: }
	goto	l11624
	line	204
	
l3745:	
	
l11622:	
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
	goto	l11616
	xorlw	1^0	; case 1
	skipnz
	goto	l11608
	xorlw	2^1	; case 2
	skipnz
	goto	l11612
	xorlw	3^2	; case 3
	skipnz
	goto	l11604
	goto	l11624
	opt asmopt_on

	line	224
	
l3747:	
	line	226
	
l11624:	
;main.c: 226: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	227
;main.c: 227: lcd_write_1_digit_bcd(xCoord);
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	228
;main.c: 228: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	229
;main.c: 229: lcd_write_1_digit_bcd(yCoord);
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	230
	
l3752:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text1633,local,class=CODE,delta=2
global __ptext1633
__ptext1633:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 134 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_play_iCreate_song
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_getVictimZone
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1633
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	134
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 3
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	136
	
l11584:	
;main.c: 136: if(victimFound)
	btfss	(_victimFound/8),(_victimFound)&7
	goto	u4771
	goto	u4770
u4771:
	goto	l3724
u4770:
	line	138
	
l11586:	
;main.c: 137: {
;main.c: 138: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4781
	goto	u4780
u4781:
	goto	l11596
u4780:
	line	140
	
l11588:	
;main.c: 139: {
;main.c: 140: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	141
	
l11590:	
;main.c: 141: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	142
	
l11592:	
;main.c: 142: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	143
	
l11594:	
;main.c: 143: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	144
;main.c: 144: }
	goto	l3724
	line	145
	
l3722:	
	line	147
	
l11596:	
;main.c: 145: else
;main.c: 146: {
;main.c: 147: victimZone = getVictimZone(xCoord, yCoord);
	movf	(_yCoord),w	;volatile
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
	line	148
	
l11598:	
;main.c: 148: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	149
	
l11600:	
;main.c: 149: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l3724
	line	150
	
l3723:	
	goto	l3724
	line	151
	
l3721:	
	line	152
	
l3724:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text1634,local,class=CODE,delta=2
global __ptext1634
__ptext1634:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 123 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1634
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	123
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 3
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	124
	
l11570:	
;main.c: 124: if(!goingHome && (xCoord == getFinalX()) && (yCoord == getFinalY()))
	btfsc	(_goingHome/8),(_goingHome)&7
	goto	u4741
	goto	u4740
u4741:
	goto	l3718
u4740:
	
l11572:	
	fcall	_getFinalX
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u4751
	goto	u4750
u4751:
	goto	l3718
u4750:
	
l11574:	
	fcall	_getFinalY
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u4761
	goto	u4760
u4761:
	goto	l3718
u4760:
	line	126
	
l11576:	
;main.c: 125: {
;main.c: 126: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	127
	
l11578:	
;main.c: 127: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	128
	
l11580:	
;main.c: 128: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	129
	
l11582:	
;main.c: 129: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l3718
	line	130
	
l3717:	
	line	131
	
l3718:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text1635,local,class=CODE,delta=2
global __ptext1635
__ptext1635:

;; *************** function _init *****************
;; Defined at:
;;		line 86 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1635
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	86
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	87
	
l11538:	
;main.c: 87: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	88
	
l11540:	
;main.c: 88: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	90
	
l11542:	
;main.c: 90: init_adc();
	fcall	_init_adc
	line	91
	
l11544:	
;main.c: 91: lcd_init();
	fcall	_lcd_init
	line	93
	
l11546:	
;main.c: 93: TRISB = 0b00000001;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	96
	
l11548:	
;main.c: 96: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	98
	
l11550:	
;main.c: 98: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	99
	
l11552:	
;main.c: 99: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	100
	
l11554:	
;main.c: 100: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	101
	
l11556:	
;main.c: 101: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	102
	
l11558:	
;main.c: 102: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	105
	
l11560:	
;main.c: 105: PEIE = 1;
	bsf	(94/8),(94)&7
	line	106
	
l11562:	
;main.c: 106: GIE = 1;
	bsf	(95/8),(95)&7
	line	108
	
l11564:	
;main.c: 108: ser_init();
	fcall	_ser_init
	line	109
	
l11566:	
;main.c: 109: initIRobot();
	fcall	_initIRobot
	line	110
	
l11568:	
;main.c: 110: initSongs();
	fcall	_initSongs
	line	111
	
l3711:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_readIR
psect	text1636,local,class=CODE,delta=2
global __ptext1636
__ptext1636:

;; *************** function _readIR *****************
;; Defined at:
;;		line 43 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  cm              2    6[BANK1 ] int 
;; Return value:  Size  Location     Type
;;                  2    4[BANK1 ] int 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       2       0       0
;;      Locals:         0       0       2       0       0
;;      Temps:          0       0       0       0       0
;;      Totals:         0       0       4       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    5
;; This function calls:
;;		_adc_read_channel
;;		_convert
;; This function is called by:
;;		_rightWallCorrect
;;		_frontWallCorrect
;;		_findWall
;; This function uses a non-reentrant model
;;
psect	text1636
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
	line	43
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 0
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	44
	
l11532:	
;ir.c: 44: int cm = convert(adc_read_channel(0));
	movlw	(0)
	fcall	_adc_read_channel
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_adc_read_channel))^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert+1)
	addwf	(?_convert+1)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(0+(?_adc_read_channel))^080h,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	addwf	(?_convert)

	fcall	_convert
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_convert)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(readIR@cm+1)^080h
	addwf	(readIR@cm+1)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?_convert)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(readIR@cm)^080h
	addwf	(readIR@cm)^080h

	line	45
	
l11534:	
;ir.c: 45: return cm;
	movf	(readIR@cm+1)^080h,w
	clrf	(?_readIR+1)^080h
	addwf	(?_readIR+1)^080h
	movf	(readIR@cm)^080h,w
	clrf	(?_readIR)^080h
	addwf	(?_readIR)^080h

	goto	l6719
	
l11536:	
	line	46
	
l6719:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_goReverse
psect	text1637,local,class=CODE,delta=2
global __ptext1637
__ptext1637:

;; *************** function _goReverse *****************
;; Defined at:
;;		line 123 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_data
;;		_drive
;;		_waitFor
;;		_updateOrientation
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1637
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	123
	global	__size_of_goReverse
	__size_of_goReverse	equ	__end_of_goReverse-_goReverse
	
_goReverse:	
	opt	stack 0
; Regs used in _goReverse: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	124
	
l11516:	
;drive.c: 124: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	125
;drive.c: 125: lcd_write_data('!');
	movlw	(021h)
	fcall	_lcd_write_data
	line	126
	
l11518:	
;drive.c: 126: drive(255, 131, 128, 0);
	movlw	(083h)
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
	line	127
	
l11520:	
;drive.c: 127: waitFor(156,1,244);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	bsf	status,0
	rlf	(?_waitFor),f
	movlw	(0F4h)
	movwf	(??_goReverse+0)+0
	movf	(??_goReverse+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Ch)
	fcall	_waitFor
	line	128
	
l11522:	
;drive.c: 128: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	129
	
l11524:	
;drive.c: 129: if(lastMove == LEFT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	01h
	skipz
	goto	u4721
	goto	u4720
u4721:
	goto	l11528
u4720:
	line	131
	
l11526:	
;drive.c: 130: {
;drive.c: 131: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	132
;drive.c: 132: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	134
;drive.c: 134: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	135
;drive.c: 135: }
	goto	l2161
	line	136
	
l2158:	
	
l11528:	
;drive.c: 136: else if (lastMove == RIGHT)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_lastMove),w	;volatile
	xorlw	03h
	skipz
	goto	u4731
	goto	u4730
u4731:
	goto	l2161
u4730:
	line	138
	
l11530:	
;drive.c: 137: {
;drive.c: 138: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	139
;drive.c: 139: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	141
;drive.c: 141: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	goto	l2161
	line	142
	
l2160:	
	goto	l2161
	line	143
	
l2159:	
	
l2161:	
	return
	opt stack 0
GLOBAL	__end_of_goReverse
	__end_of_goReverse:
;; =============== function _goReverse ends ============

	signat	_goReverse,88
	global	_checkIfHome
psect	text1638,local,class=CODE,delta=2
global __ptext1638
__ptext1638:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 245 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1638
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	245
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 3
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	246
	
l11508:	
;main.c: 246: if((xCoord == 1) && (yCoord == 3))
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u4701
	goto	u4700
u4701:
	goto	l3765
u4700:
	
l11510:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4711
	goto	u4710
u4711:
	goto	l3765
u4710:
	line	248
	
l11512:	
;main.c: 247: {
;main.c: 248: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	249
;main.c: 249: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	250
	
l11514:	
;main.c: 250: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l3765
	line	251
	
l3764:	
	line	252
	
l3765:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_initSongs
psect	text1639,local,class=CODE,delta=2
global __ptext1639
__ptext1639:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 30 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
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
psect	text1639
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
	line	30
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	31
	
l11506:	
;songs.c: 31: ser_putArr(finalCountdown, 27);
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
	line	32
;songs.c: 32: ser_putArr(superMarioBros, 25);
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
	line	33
;songs.c: 33: ser_putArr(lookingForU2, 29);
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
	line	34
;songs.c: 34: ser_putArr(champions, 21);
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
	line	35
	
l6006:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text1640,local,class=CODE,delta=2
global __ptext1640
__ptext1640:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
psect	text1640
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l11486:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l11488:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l11490:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l11492:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l11494:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l11496:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l11498:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l11500:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l11502:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l11504:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l2914:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1641,local,class=CODE,delta=2
global __ptext1641
__ptext1641:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
;;		_lookForVictim
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text1641
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
	
l11484:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l2902:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_write_string
psect	text1642,local,class=CODE,delta=2
global __ptext1642
__ptext1642:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
psect	text1642
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
	
l11476:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l11482
	
l2897:	
	
l11478:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l11480:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l11482
	
l2896:	
	
l11482:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u4691
	goto	u4690
u4691:
	goto	l11478
u4690:
	goto	l2899
	
l2898:	
	line	41
	
l2899:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_turnRight90
psect	text1643,local,class=CODE,delta=2
global __ptext1643
__ptext1643:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 174 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;		_goRight
;;		_rightWallCorrect
;; This function uses a non-reentrant model
;;
psect	text1643
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	174
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 1
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	175
	
l11472:	
;drive.c: 175: drive(0, 25, 255, 255);
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
	line	176
;drive.c: 176: waitFor(157,255,169);
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
	line	177
;drive.c: 177: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	178
	
l11474:	
;drive.c: 178: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	224
movwf	((??_turnRight90+0)+0+1),f
	movlw	254
movwf	((??_turnRight90+0)+0),f
u5037:
	decfsz	((??_turnRight90+0)+0),f
	goto	u5037
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u5037
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u5037
opt asmopt_on

	line	179
	
l2173:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_turnLeft90
psect	text1644,local,class=CODE,delta=2
global __ptext1644
__ptext1644:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 166 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;		_goLeft
;;		_rightWallCorrect
;; This function uses a non-reentrant model
;;
psect	text1644
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	166
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 1
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	167
	
l11468:	
;drive.c: 167: drive(0, 25, 0, 1);
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
	line	168
;drive.c: 168: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	169
;drive.c: 169: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	170
	
l11470:	
;drive.c: 170: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	224
movwf	((??_turnLeft90+0)+0+1),f
	movlw	254
movwf	((??_turnLeft90+0)+0),f
u5047:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u5047
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u5047
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u5047
opt asmopt_on

	line	171
	
l2170:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnAround
psect	text1645,local,class=CODE,delta=2
global __ptext1645
__ptext1645:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1645
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	157
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	158
	
l11462:	
;drive.c: 158: drive(0, 25, 0, 1);
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
	line	159
;drive.c: 159: waitFor(157,0,170);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0AAh)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	160
;drive.c: 160: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	161
	
l11464:	
;drive.c: 161: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u5057:
	decfsz	((??_turnAround+0)+0),f
	goto	u5057
	decfsz	((??_turnAround+0)+0+1),f
	goto	u5057
	decfsz	((??_turnAround+0)+0+2),f
	goto	u5057
opt asmopt_on

	line	162
	
l11466:	
;drive.c: 162: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u5067:
	decfsz	((??_turnAround+0)+0),f
	goto	u5067
	decfsz	((??_turnAround+0)+0+1),f
	goto	u5067
	decfsz	((??_turnAround+0)+0+2),f
	goto	u5067
opt asmopt_on

	line	163
	
l2167:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_lcd_set_cursor
psect	text1646,local,class=CODE,delta=2
global __ptext1646
__ptext1646:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
;;		_findFinalDestination
;; This function uses a non-reentrant model
;;
psect	text1646
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
	
l11458:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l11460:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l2893:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_adc_read_channel
psect	text1647,local,class=CODE,delta=2
global __ptext1647
__ptext1647:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
;; Parameters:    Size  Location     Type
;;  channel         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  channel         1    3[BANK1 ] unsigned char 
;; Return value:  Size  Location     Type
;;                  2    0[BANK1 ] int 
;; Registers used:
;;		wreg, fsr0l, fsr0h, status,2, status,0, btemp+1, pclath, cstack
;; Tracked objects:
;;		On entry : 0/0
;;		On exit  : 0/0
;;		Unchanged: 0/0
;; Data sizes:     COMMON   BANK0   BANK1   BANK3   BANK2
;;      Params:         0       0       2       0       0
;;      Locals:         0       0       1       0       0
;;      Temps:          0       0       1       0       0
;;      Totals:         0       0       4       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_adc_read
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text1647
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
	line	7
	global	__size_of_adc_read_channel
	__size_of_adc_read_channel	equ	__end_of_adc_read_channel-_adc_read_channel
	
_adc_read_channel:	
	opt	stack 0
; Regs used in _adc_read_channel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
;adc_read_channel@channel stored from wreg
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(adc_read_channel@channel)^080h
	line	8
	
l11442:	
;adc.c: 8: switch(channel)
	goto	l11450
	line	10
;adc.c: 9: {
;adc.c: 10: case 0:
	
l1390:	
	line	11
;adc.c: 11: CHS0 = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(251/8),(251)&7
	line	12
;adc.c: 12: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	13
;adc.c: 13: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	14
;adc.c: 14: break;
	goto	l11452
	line	15
;adc.c: 15: case 1:
	
l1392:	
	line	16
;adc.c: 16: CHS0 = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(251/8),(251)&7
	line	17
;adc.c: 17: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	18
;adc.c: 18: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	19
;adc.c: 19: break;
	goto	l11452
	line	20
;adc.c: 20: case 2:
	
l1393:	
	line	21
;adc.c: 21: CHS0 = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(251/8),(251)&7
	line	22
;adc.c: 22: CHS1 = 1;
	bsf	(252/8),(252)&7
	line	23
;adc.c: 23: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	24
;adc.c: 24: break;
	goto	l11452
	line	25
;adc.c: 25: case 3:
	
l1394:	
	line	26
;adc.c: 26: CHS0 = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(251/8),(251)&7
	line	27
;adc.c: 27: CHS1 = 1;
	bsf	(252/8),(252)&7
	line	28
;adc.c: 28: CHS2 = 0;
	bcf	(253/8),(253)&7
	line	29
;adc.c: 29: break;
	goto	l11452
	line	30
;adc.c: 30: case 4:
	
l1395:	
	line	31
;adc.c: 31: CHS0 = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(251/8),(251)&7
	line	32
;adc.c: 32: CHS1 = 0;
	bcf	(252/8),(252)&7
	line	33
;adc.c: 33: CHS2 = 1;
	bsf	(253/8),(253)&7
	line	34
;adc.c: 34: break;
	goto	l11452
	line	37
;adc.c: 37: default:
	
l1396:	
	line	38
	
l11444:	
;adc.c: 38: return 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(?_adc_read_channel)^080h
	clrf	(?_adc_read_channel+1)^080h
	goto	l1397
	
l11446:	
	goto	l1397
	line	39
	
l11448:	
;adc.c: 39: }
	goto	l11452
	line	8
	
l1389:	
	
l11450:	
	movf	(adc_read_channel@channel)^080h,w
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
	goto	l1390
	xorlw	1^0	; case 1
	skipnz
	goto	l1392
	xorlw	2^1	; case 2
	skipnz
	goto	l1393
	xorlw	3^2	; case 3
	skipnz
	goto	l1394
	xorlw	4^3	; case 4
	skipnz
	goto	l1395
	goto	l11444
	opt asmopt_on

	line	39
	
l1391:	
	line	41
	
l11452:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
movwf	(??_adc_read_channel+0)^080h+0,f
u5077:
decfsz	(??_adc_read_channel+0)^080h+0,f
	goto	u5077
opt asmopt_on

	line	43
	
l11454:	
;adc.c: 43: return adc_read();
	fcall	_adc_read
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_adc_read)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(?_adc_read_channel+1)^080h
	addwf	(?_adc_read_channel+1)^080h
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(0+(?_adc_read)),w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(?_adc_read_channel)^080h
	addwf	(?_adc_read_channel)^080h

	goto	l1397
	
l11456:	
	line	45
	
l1397:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_detectCliff
psect	text1648,local,class=CODE,delta=2
global __ptext1648
__ptext1648:

;; *************** function _detectCliff *****************
;; Defined at:
;;		line 12 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\sensors.c"
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
;;		_ser_putch
;;		_ser_getch
;; This function is called by:
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1648
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\sensors.c"
	line	12
	global	__size_of_detectCliff
	__size_of_detectCliff	equ	__end_of_detectCliff-_detectCliff
	
_detectCliff:	
	opt	stack 0
; Regs used in _detectCliff: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	13
	
l11404:	
;sensors.c: 13: sensorDetected = 0;
	bcf	(_sensorDetected/8),(_sensorDetected)&7
	line	14
	
l11406:	
;sensors.c: 14: ser_putch(149);
	movlw	(095h)
	fcall	_ser_putch
	line	15
;sensors.c: 15: ser_putch(4);
	movlw	(04h)
	fcall	_ser_putch
	line	16
;sensors.c: 16: ser_putch(9);
	movlw	(09h)
	fcall	_ser_putch
	line	17
;sensors.c: 17: ser_putch(10);
	movlw	(0Ah)
	fcall	_ser_putch
	line	18
;sensors.c: 18: ser_putch(11);
	movlw	(0Bh)
	fcall	_ser_putch
	line	19
;sensors.c: 19: ser_putch(12);
	movlw	(0Ch)
	fcall	_ser_putch
	line	21
;sensors.c: 21: sensorData = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_detectCliff+0)+0
	movf	(??_detectCliff+0)+0,w
	movwf	(_sensorData)
	line	22
	
l11408:	
;sensors.c: 22: if(sensorData == 1)
	movf	(_sensorData),w
	xorlw	01h
	skipz
	goto	u4611
	goto	u4610
u4611:
	goto	l11412
u4610:
	line	23
	
l11410:	
;sensors.c: 23: sensorDetected = 1;
	bsf	(_sensorDetected/8),(_sensorDetected)&7
	goto	l11412
	
l693:	
	line	25
	
l11412:	
;sensors.c: 25: sensorData = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_detectCliff+0)+0
	movf	(??_detectCliff+0)+0,w
	movwf	(_sensorData)
	line	26
	
l11414:	
;sensors.c: 26: if(sensorData == 1 && !sensorDetected)
	movf	(_sensorData),w
	xorlw	01h
	skipz
	goto	u4621
	goto	u4620
u4621:
	goto	l11420
u4620:
	
l11416:	
	btfsc	(_sensorDetected/8),(_sensorDetected)&7
	goto	u4631
	goto	u4630
u4631:
	goto	l11420
u4630:
	line	27
	
l11418:	
;sensors.c: 27: sensorDetected = 1;
	bsf	(_sensorDetected/8),(_sensorDetected)&7
	goto	l11420
	
l694:	
	line	29
	
l11420:	
;sensors.c: 29: sensorData = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_detectCliff+0)+0
	movf	(??_detectCliff+0)+0,w
	movwf	(_sensorData)
	line	30
	
l11422:	
;sensors.c: 30: if(sensorData == 1 && !sensorDetected)
	movf	(_sensorData),w
	xorlw	01h
	skipz
	goto	u4641
	goto	u4640
u4641:
	goto	l11428
u4640:
	
l11424:	
	btfsc	(_sensorDetected/8),(_sensorDetected)&7
	goto	u4651
	goto	u4650
u4651:
	goto	l11428
u4650:
	line	31
	
l11426:	
;sensors.c: 31: sensorDetected = 1;
	bsf	(_sensorDetected/8),(_sensorDetected)&7
	goto	l11428
	
l695:	
	line	33
	
l11428:	
;sensors.c: 33: sensorData = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_detectCliff+0)+0
	movf	(??_detectCliff+0)+0,w
	movwf	(_sensorData)
	line	34
	
l11430:	
;sensors.c: 34: if(sensorData == 1 && !sensorDetected)
	movf	(_sensorData),w
	xorlw	01h
	skipz
	goto	u4661
	goto	u4660
u4661:
	goto	l696
u4660:
	
l11432:	
	btfsc	(_sensorDetected/8),(_sensorDetected)&7
	goto	u4671
	goto	u4670
u4671:
	goto	l696
u4670:
	line	35
	
l11434:	
;sensors.c: 35: sensorDetected = 1;
	bsf	(_sensorDetected/8),(_sensorDetected)&7
	
l696:	
	line	37
;sensors.c: 37: return sensorDetected;
	btfsc	(_sensorDetected/8),(_sensorDetected)&7
	goto	u4681
	goto	u4680
u4681:
	goto	l11438
u4680:
	
l11436:	
	clrc
	
	goto	l697
	
l10870:	
	
l11438:	
	setc
	
	goto	l697
	
l10872:	
	goto	l697
	
l11440:	
	line	38
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_detectCliff
	__end_of_detectCliff:
;; =============== function _detectCliff ends ============

	signat	_detectCliff,88
	global	_convert
psect	text1649,local,class=CODE,delta=2
global __ptext1649
__ptext1649:

;; *************** function _convert *****************
;; Defined at:
;;		line 12 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
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
psect	text1649
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
	line	12
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 1
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	13
	
l11344:	
;ir.c: 13: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4545
	movlw	low(052h)
	subwf	(convert@adc_value),w
u4545:

	skipnc
	goto	u4541
	goto	u4540
u4541:
	goto	l11352
u4540:
	line	14
	
l11346:	
;ir.c: 14: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l6698
	
l11348:	
	goto	l6698
	
l11350:	
	goto	l6698
	line	15
	
l6697:	
	
l11352:	
;ir.c: 15: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4555
	movlw	low(085h)
	subwf	(convert@adc_value),w
u4555:

	skipnc
	goto	u4551
	goto	u4550
u4551:
	goto	l11360
u4550:
	line	16
	
l11354:	
;ir.c: 16: return (100 + (150-100)*(133 - adc_value)/(133 - 82) - 3);
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
	goto	l6698
	
l11356:	
	goto	l6698
	
l11358:	
	goto	l6698
	line	17
	
l6700:	
	
l11360:	
;ir.c: 17: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4565
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u4565:

	skipnc
	goto	u4561
	goto	u4560
u4561:
	goto	l11368
u4560:
	line	18
	
l11362:	
;ir.c: 18: return (70 + (100-70)*(184 - adc_value)/(184 - 133) - 3);
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
	goto	l6698
	
l11364:	
	goto	l6698
	
l11366:	
	goto	l6698
	line	19
	
l6702:	
	
l11368:	
;ir.c: 19: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4575
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u4575:

	skipnc
	goto	u4571
	goto	u4570
u4571:
	goto	l11376
u4570:
	line	20
	
l11370:	
;ir.c: 20: return (50 + (70-50)*(256 - adc_value)/(256 - 184) - 4);
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
	goto	l6698
	
l11372:	
	goto	l6698
	
l11374:	
	goto	l6698
	line	21
	
l6704:	
	
l11376:	
;ir.c: 21: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4585
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u4585:

	skipnc
	goto	u4581
	goto	u4580
u4581:
	goto	l11384
u4580:
	line	22
	
l11378:	
;ir.c: 22: return (40 + (50-40)*(317 - adc_value)/(317 - 256) - 3);
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
	goto	l6698
	
l11380:	
	goto	l6698
	
l11382:	
	goto	l6698
	line	23
	
l6706:	
	
l11384:	
;ir.c: 23: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4595
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u4595:

	skipnc
	goto	u4591
	goto	u4590
u4591:
	goto	l11392
u4590:
	line	24
	
l11386:	
;ir.c: 24: return (30 + (40-30)*(410 - adc_value)/(410 - 317) - 2);
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
	goto	l6698
	
l11388:	
	goto	l6698
	
l11390:	
	goto	l6698
	line	25
	
l6708:	
	
l11392:	
;ir.c: 25: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4605
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u4605:

	skipnc
	goto	u4601
	goto	u4600
u4601:
	goto	l11400
u4600:
	line	26
	
l11394:	
;ir.c: 26: return (20 + (30-20)*(522 - adc_value)/(522 - 410) - 2);
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
	goto	l6698
	
l11396:	
	goto	l6698
	
l11398:	
	goto	l6698
	line	27
	
l6710:	
	
l11400:	
;ir.c: 27: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l6698
	
l11402:	
	goto	l6698
	
l6711:	
	goto	l6698
	
l6709:	
	goto	l6698
	
l6707:	
	goto	l6698
	
l6705:	
	goto	l6698
	
l6703:	
	goto	l6698
	
l6701:	
	goto	l6698
	
l6699:	
	line	28
	
l6698:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_ser_putArr
psect	text1650,local,class=CODE,delta=2
global __ptext1650
__ptext1650:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
;; Parameters:    Size  Location     Type
;;  array           2   12[BANK0 ] PTR unsigned char 
;;		 -> champions(21), lookingForU2(29), superMarioBros(25), finalCountdown(27), 
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
psect	text1650
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l11336:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l11342
	line	75
	
l5276:	
	line	76
	
l11338:	
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
	goto	u4520
	decf	(??_ser_putArr+0)+0,f
u4520:
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
	
l11340:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l11342
	
l5275:	
	
l11342:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u4535
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u4535:

	skipc
	goto	u4531
	goto	u4530
u4531:
	goto	l11338
u4530:
	goto	l5278
	
l5277:	
	line	78
	
l5278:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putArr
	__end_of_ser_putArr:
;; =============== function _ser_putArr ends ============

	signat	_ser_putArr,8312
	global	_play_iCreate_song
psect	text1651,local,class=CODE,delta=2
global __ptext1651
__ptext1651:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 24 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
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
;; This function uses a non-reentrant model
;;
psect	text1651
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\songs.c"
	line	24
	global	__size_of_play_iCreate_song
	__size_of_play_iCreate_song	equ	__end_of_play_iCreate_song-_play_iCreate_song
	
_play_iCreate_song:	
	opt	stack 3
; Regs used in _play_iCreate_song: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;play_iCreate_song@song stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(play_iCreate_song@song)
	line	25
	
l11334:	
;songs.c: 25: ser_putch(141);
	movlw	(08Dh)
	fcall	_ser_putch
	line	26
;songs.c: 26: ser_putch(song);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(play_iCreate_song@song),w
	fcall	_ser_putch
	line	27
	
l6003:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_initIRobot
psect	text1652,local,class=CODE,delta=2
global __ptext1652
__ptext1652:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 114 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1652
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	114
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	115
	
l11328:	
;main.c: 115: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u5087:
	decfsz	((??_initIRobot+0)+0),f
	goto	u5087
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u5087
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u5087
	nop2
opt asmopt_on

	line	116
	
l11330:	
;main.c: 116: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	117
	
l11332:	
;main.c: 117: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	118
	
l3714:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_lcd_write_control
psect	text1653,local,class=CODE,delta=2
global __ptext1653
__ptext1653:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
psect	text1653
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
	
l11320:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l11322:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l11324:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l11326:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u5097:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u5097
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u5097
	nop2
opt asmopt_on

	line	16
	
l2887:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_rotateIR
psect	text1654,local,class=CODE,delta=2
global __ptext1654
__ptext1654:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 49 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
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
;;		_rightWallCorrect
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text1654
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\ir.c"
	line	49
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 3
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	50
	
l11302:	
;ir.c: 50: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	51
	
l11304:	
;ir.c: 51: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	52
	
l11306:	
;ir.c: 52: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u5107:
	decfsz	((??_rotateIR+0)+0),f
	goto	u5107
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u5107
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u5107
	nop2
opt asmopt_on

	line	54
	
l11308:	
;ir.c: 54: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l6722
	line	55
	
l6723:	
	line	56
;ir.c: 55: {
;ir.c: 56: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	57
	
l11310:	
;ir.c: 57: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	58
	
l11312:	
;ir.c: 58: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u5117:
	decfsz	((??_rotateIR+0)+0),f
	goto	u5117
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u5117
	nop2
opt asmopt_on

	line	54
	
l11314:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l6722:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u4511
	goto	u4510
u4511:
	goto	l6723
u4510:
	goto	l11316
	
l6724:	
	line	61
	
l11316:	
;ir.c: 59: }
;ir.c: 61: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	62
	
l11318:	
;ir.c: 62: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u5127:
	decfsz	((??_rotateIR+0)+0),f
	goto	u5127
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u5127
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u5127
	nop2
opt asmopt_on

	line	64
	
l6725:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_waitFor
psect	text1655,local,class=CODE,delta=2
global __ptext1655
__ptext1655:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 189 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1655
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	189
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 0
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	190
	
l11294:	
;drive.c: 190: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u5137:
	decfsz	((??_waitFor+0)+0),f
	goto	u5137
	decfsz	((??_waitFor+0)+0+1),f
	goto	u5137
	decfsz	((??_waitFor+0)+0+2),f
	goto	u5137
	nop2
opt asmopt_on

	line	191
	
l11296:	
;drive.c: 191: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	192
	
l11298:	
;drive.c: 192: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	193
	
l11300:	
;drive.c: 193: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	194
	
l2180:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_lcd_write_data
psect	text1656,local,class=CODE,delta=2
global __ptext1656
__ptext1656:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goReverse
;;		_goRight
;;		_lcd_write_string
;;		_lcd_write_1_digit_bcd
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_updateLocation
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1656
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\lcd.c"
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
	
l11286:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l11288:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l11290:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l11292:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u5147:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u5147
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u5147
opt asmopt_on

	line	28
	
l2890:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_drive
psect	text1657,local,class=CODE,delta=2
global __ptext1657
__ptext1657:

;; *************** function _drive *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;		_rightWallCorrect
;;		_frontWallCorrect
;;		_checkIfHome
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1657
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	20
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 3
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	21
	
l11274:	
;drive.c: 21: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u5157:
	decfsz	((??_drive+0)+0),f
	goto	u5157
	decfsz	((??_drive+0)+0+1),f
	goto	u5157
	decfsz	((??_drive+0)+0+2),f
	goto	u5157
	nop2
opt asmopt_on

	line	22
	
l11276:	
;drive.c: 22: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	23
	
l11278:	
;drive.c: 23: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	24
	
l11280:	
;drive.c: 24: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	25
	
l11282:	
;drive.c: 25: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	26
	
l11284:	
;drive.c: 26: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	27
	
l2129:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_init_adc
psect	text1658,local,class=CODE,delta=2
global __ptext1658
__ptext1658:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
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
psect	text1658
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l11264:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l11266:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l11268:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l11270:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l11272:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u5167:
decfsz	(??_init_adc+0)+0,f
	goto	u5167
opt asmopt_on

	line	58
	
l1400:	
	return
	opt stack 0
GLOBAL	__end_of_init_adc
	__end_of_init_adc:
;; =============== function _init_adc ends ============

	signat	_init_adc,88
	global	_adc_read
psect	text1659,local,class=CODE,delta=2
global __ptext1659
__ptext1659:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
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
psect	text1659
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 0
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l11254:	
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
	
l11256:	
;adc.c: 68: GO = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(250/8),(250)&7
	line	69
;adc.c: 69: while(GO) continue;
	goto	l1403
	
l1404:	
	
l1403:	
	btfsc	(250/8),(250)&7
	goto	u4491
	goto	u4490
u4491:
	goto	l1403
u4490:
	goto	l11258
	
l1405:	
	line	75
	
l11258:	
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
u4505:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4505
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l11260:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l1406
	
l11262:	
	line	78
	
l1406:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	_ser_getch
psect	text1660,local,class=CODE,delta=2
global __ptext1660
__ptext1660:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
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
;;		_detectCliff
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1660
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 0
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l11238:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l11240
	
l5270:	
	line	62
;ser.c: 62: continue;
	goto	l11240
	
l5269:	
	line	61
	
l11240:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u4481
	goto	u4480
u4481:
	goto	l11240
u4480:
	
l5271:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l11242:	
;ser.c: 65: c=rxfifo[rxoptr];
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_rxoptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank0
	movf	indf,w
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	movwf	(ser_getch@c)
	line	66
	
l11244:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l11246:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l11248:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l11250:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l5272
	
l11252:	
	line	70
	
l5272:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	___awdiv
psect	text1661,local,class=CODE,delta=2
global __ptext1661
__ptext1661:

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
psect	text1661
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 1
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l11198:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u4381
	goto	u4380
u4381:
	goto	l11202
u4380:
	line	11
	
l11200:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l11202
	line	13
	
l7548:	
	line	14
	
l11202:	
	btfss	(___awdiv@dividend+1),7
	goto	u4391
	goto	u4390
u4391:
	goto	l11208
u4390:
	line	15
	
l11204:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l11206:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l11208
	line	17
	
l7549:	
	line	18
	
l11208:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l11210:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u4401
	goto	u4400
u4401:
	goto	l11230
u4400:
	line	20
	
l11212:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l11218
	
l7552:	
	line	22
	
l11214:	
	movlw	01h
	
u4415:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u4415
	line	23
	
l11216:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l11218
	line	24
	
l7551:	
	line	21
	
l11218:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u4421
	goto	u4420
u4421:
	goto	l11214
u4420:
	goto	l11220
	
l7553:	
	goto	l11220
	line	25
	
l7554:	
	line	26
	
l11220:	
	movlw	01h
	
u4435:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u4435
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u4445
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u4445:
	skipc
	goto	u4441
	goto	u4440
u4441:
	goto	l11226
u4440:
	line	28
	
l11222:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l11224:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l11226
	line	30
	
l7555:	
	line	31
	
l11226:	
	movlw	01h
	
u4455:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u4455
	line	32
	
l11228:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u4461
	goto	u4460
u4461:
	goto	l11220
u4460:
	goto	l11230
	
l7556:	
	goto	l11230
	line	33
	
l7550:	
	line	34
	
l11230:	
	movf	(___awdiv@sign),w
	skipz
	goto	u4470
	goto	l11234
u4470:
	line	35
	
l11232:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l11234
	
l7557:	
	line	36
	
l11234:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l7558
	
l11236:	
	line	37
	
l7558:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1662,local,class=CODE,delta=2
global __ptext1662
__ptext1662:

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
psect	text1662
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 1
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l11186:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l11188
	line	6
	
l7408:	
	line	7
	
l11188:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u4341
	goto	u4340
u4341:
	goto	l7409
u4340:
	line	8
	
l11190:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l7409:	
	line	9
	movlw	01h
	
u4355:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u4355
	line	10
	
l11192:	
	movlw	01h
	
u4365:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u4365
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u4371
	goto	u4370
u4371:
	goto	l11188
u4370:
	goto	l11194
	
l7410:	
	line	12
	
l11194:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l7411
	
l11196:	
	line	13
	
l7411:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_ser_isrx
psect	text1663,local,class=CODE,delta=2
global __ptext1663
__ptext1663:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
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
psect	text1663
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 0
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l11138:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u4271
	goto	u4270
u4271:
	goto	l11146
u4270:
	line	50
	
l11140:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l11142:	
;ser.c: 52: return 0;
	clrc
	
	goto	l5266
	
l11144:	
	goto	l5266
	line	53
	
l5265:	
	line	54
	
l11146:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u4281
	goto	u4280
u4281:
	goto	l11150
u4280:
	
l11148:	
	clrc
	
	goto	l5266
	
l10878:	
	
l11150:	
	setc
	
	goto	l5266
	
l10880:	
	goto	l5266
	
l11152:	
	line	55
	
l5266:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_updateNode
psect	text1664,local,class=CODE,delta=2
global __ptext1664
__ptext1664:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 233 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1664
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	233
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 5
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	234
	
l11036:	
;main.c: 234: if((xCoord == 2) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4181
	goto	u4180
u4181:
	goto	l11042
u4180:
	
l11038:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4191
	goto	u4190
u4191:
	goto	l11042
u4190:
	line	235
	
l11040:	
;main.c: 235: node = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_node)	;volatile
	bsf	status,0
	rlf	(_node),f	;volatile
	goto	l3761
	line	236
	
l3755:	
	
l11042:	
;main.c: 236: else if((xCoord == 4) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u4201
	goto	u4200
u4201:
	goto	l11048
u4200:
	
l11044:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4211
	goto	u4210
u4211:
	goto	l11048
u4210:
	line	237
	
l11046:	
;main.c: 237: node = 2;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l3761
	line	238
	
l3757:	
	
l11048:	
;main.c: 238: else if((xCoord == 2) && (yCoord == 0))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4221
	goto	u4220
u4221:
	goto	l3759
u4220:
	
l11050:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u4231
	goto	u4230
u4231:
	goto	l3759
u4230:
	line	239
	
l11052:	
;main.c: 239: node = 3;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l3761
	line	240
	
l3759:	
	line	241
;main.c: 240: else
;main.c: 241: node = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_node)	;volatile
	goto	l3761
	
l3760:	
	goto	l3761
	
l3758:	
	goto	l3761
	
l3756:	
	line	242
	
l3761:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getVictimZone
psect	text1665,local,class=CODE,delta=2
global __ptext1665
__ptext1665:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
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
psect	text1665
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
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
	
l10988:	
;map.c: 163: switch (victimX)
	goto	l11030
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l4533:	
	line	166
;map.c: 166: switch (victimY)
	goto	l10996
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l4535:	
	line	169
	
l10990:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	170
;map.c: 170: break;
	goto	l11032
	line	171
;map.c: 171: case 1:
	
l4537:	
	line	172
	
l10992:	
;map.c: 172: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	173
;map.c: 173: break;
	goto	l11032
	line	178
;map.c: 178: default:
	
l4538:	
	line	179
;map.c: 179: break;
	goto	l11032
	line	180
	
l10994:	
;map.c: 180: }
	goto	l11032
	line	166
	
l4534:	
	
l10996:	
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
	goto	l10990
	xorlw	1^0	; case 1
	skipnz
	goto	l10992
	goto	l11032
	opt asmopt_on

	line	180
	
l4536:	
	line	181
;map.c: 181: break;
	goto	l11032
	line	183
;map.c: 183: case 1:
	
l4540:	
	line	184
;map.c: 184: switch (victimY)
	goto	l11004
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l4542:	
	line	187
	
l10998:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	188
;map.c: 188: break;
	goto	l11032
	line	189
;map.c: 189: case 1:
	
l4544:	
	line	190
	
l11000:	
;map.c: 190: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	191
;map.c: 191: break;
	goto	l11032
	line	196
;map.c: 196: default:
	
l4545:	
	line	197
;map.c: 197: break;
	goto	l11032
	line	198
	
l11002:	
;map.c: 198: }
	goto	l11032
	line	184
	
l4541:	
	
l11004:	
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
	goto	l10998
	xorlw	1^0	; case 1
	skipnz
	goto	l11000
	goto	l11032
	opt asmopt_on

	line	198
	
l4543:	
	line	199
;map.c: 199: break;
	goto	l11032
	line	201
;map.c: 201: case 2:
	
l4546:	
	line	202
;map.c: 202: switch (victimY)
	goto	l11012
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l4548:	
	line	207
	
l11006:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	208
;map.c: 208: break;
	goto	l11032
	line	211
;map.c: 211: case 3:
	
l4550:	
	line	212
	
l11008:	
;map.c: 212: vicZone = 1;
	clrf	(_vicZone)
	bsf	status,0
	rlf	(_vicZone),f
	line	213
;map.c: 213: break;
	goto	l11032
	line	214
;map.c: 214: default:
	
l4551:	
	line	215
;map.c: 215: break;
	goto	l11032
	line	216
	
l11010:	
;map.c: 216: }
	goto	l11032
	line	202
	
l4547:	
	
l11012:	
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
	goto	l11006
	xorlw	3^1	; case 3
	skipnz
	goto	l11008
	goto	l11032
	opt asmopt_on

	line	216
	
l4549:	
	line	217
;map.c: 217: break;
	goto	l11032
	line	219
;map.c: 219: case 3:
	
l4552:	
	line	220
;map.c: 220: switch (victimY)
	goto	l11020
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l4554:	
	line	225
	
l11014:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	226
;map.c: 226: break;
	goto	l11032
	line	229
;map.c: 229: case 3:
	
l4556:	
	line	230
	
l11016:	
;map.c: 230: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	231
;map.c: 231: break;
	goto	l11032
	line	232
;map.c: 232: default:
	
l4557:	
	line	233
;map.c: 233: break;
	goto	l11032
	line	234
	
l11018:	
;map.c: 234: }
	goto	l11032
	line	220
	
l4553:	
	
l11020:	
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
	goto	l11014
	xorlw	3^1	; case 3
	skipnz
	goto	l11016
	goto	l11032
	opt asmopt_on

	line	234
	
l4555:	
	line	235
;map.c: 235: break;
	goto	l11032
	line	237
;map.c: 237: case 4:
	
l4558:	
	line	238
;map.c: 238: switch (victimY)
	goto	l11026
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l4560:	
	line	247
	
l11022:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	248
;map.c: 248: break;
	goto	l11032
	line	249
;map.c: 249: default:
	
l4562:	
	line	250
;map.c: 250: break;
	goto	l11032
	line	251
	
l11024:	
;map.c: 251: }
	goto	l11032
	line	238
	
l4559:	
	
l11026:	
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
	goto	l11022
	goto	l11032
	opt asmopt_on

	line	251
	
l4561:	
	line	252
;map.c: 252: break;
	goto	l11032
	line	254
;map.c: 254: default:
	
l4563:	
	line	255
;map.c: 255: break;
	goto	l11032
	line	256
	
l11028:	
;map.c: 256: }
	goto	l11032
	line	163
	
l4532:	
	
l11030:	
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
	goto	l10996
	xorlw	1^0	; case 1
	skipnz
	goto	l11004
	xorlw	2^1	; case 2
	skipnz
	goto	l11012
	xorlw	3^2	; case 3
	skipnz
	goto	l11020
	xorlw	4^3	; case 4
	skipnz
	goto	l11026
	goto	l11032
	opt asmopt_on

	line	256
	
l4539:	
	line	258
	
l11032:	
;map.c: 258: return vicZone;
	movf	(_vicZone),w
	goto	l4564
	
l11034:	
	line	259
	
l4564:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text1666,local,class=CODE,delta=2
global __ptext1666
__ptext1666:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
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
psect	text1666
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 4
; Regs used in _getFinalY: [wreg]
	line	153
	
l10984:	
;map.c: 153: return finalY;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	goto	l4529
	
l10986:	
	line	154
	
l4529:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text1667,local,class=CODE,delta=2
global __ptext1667
__ptext1667:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
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
psect	text1667
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 4
; Regs used in _getFinalX: [wreg]
	line	148
	
l10980:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l4526
	
l10982:	
	line	149
	
l4526:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_init
psect	text1668,local,class=CODE,delta=2
global __ptext1668
__ptext1668:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
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
psect	text1668
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l10954:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l10956:	
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
	
l10958:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l10960:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l10962:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l10964:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l10966:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l10968:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l10970:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l10972:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l10974:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l10976:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l10978:	
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
	
l5306:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_updateOrientation
psect	text1669,local,class=CODE,delta=2
global __ptext1669
__ptext1669:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 182 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
;;		_goBackward
;;		_goLeft
;;		_goReverse
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1669
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	182
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 1
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	183
	
l10932:	
;drive.c: 183: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	184
	
l10934:	
;drive.c: 184: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u4151
	goto	u4150
u4151:
	goto	l2177
u4150:
	line	185
	
l10936:	
;drive.c: 185: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l2177
	
l2176:	
	line	186
	
l2177:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_getOrientation
psect	text1670,local,class=CODE,delta=2
global __ptext1670
__ptext1670:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 82 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1670
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	82
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 4
; Regs used in _getOrientation: [wreg]
	line	83
	
l10928:	
;drive.c: 83: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l2143
	
l10930:	
	line	84
	
l2143:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_getSuccessfulDrive
psect	text1671,local,class=CODE,delta=2
global __ptext1671
__ptext1671:

;; *************** function _getSuccessfulDrive *****************
;; Defined at:
;;		line 30 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
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
psect	text1671
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\drive.c"
	line	30
	global	__size_of_getSuccessfulDrive
	__size_of_getSuccessfulDrive	equ	__end_of_getSuccessfulDrive-_getSuccessfulDrive
	
_getSuccessfulDrive:	
	opt	stack 5
; Regs used in _getSuccessfulDrive: [status]
	line	31
	
l10920:	
;drive.c: 31: return successfulDrive;
	btfsc	(_successfulDrive/8),(_successfulDrive)&7
	goto	u4141
	goto	u4140
u4141:
	goto	l10924
u4140:
	
l10922:	
	clrc
	
	goto	l2132
	
l10874:	
	
l10924:	
	setc
	
	goto	l2132
	
l10876:	
	goto	l2132
	
l10926:	
	line	32
	
l2132:	
	return
	opt stack 0
GLOBAL	__end_of_getSuccessfulDrive
	__end_of_getSuccessfulDrive:
;; =============== function _getSuccessfulDrive ends ============

	signat	_getSuccessfulDrive,88
	global	_ser_putch
psect	text1672,local,class=CODE,delta=2
global __ptext1672
__ptext1672:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
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
;;		_detectCliff
;;		_drive
;;		_driveForDistance
;;		_waitFor
;;		_initIRobot
;;		_ser_putArr
;;		_play_iCreate_song
;;		_ser_puts
;;		_ser_puts2
;;		_ser_puthex
;; This function uses a non-reentrant model
;;
psect	text1672
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
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
	
l10908:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l10910
	
l5282:	
	line	83
;ser.c: 83: continue;
	goto	l10910
	
l5281:	
	line	82
	
l10910:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u4131
	goto	u4130
u4131:
	goto	l10910
u4130:
	
l5283:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l10912:	
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
	
l10914:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l10916:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l10918:	
;ser.c: 88: GIE=1;
	bsf	(95/8),(95)&7
	line	89
	
l5284:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_isr1
psect	text1673,local,class=CODE,delta=2
global __ptext1673
__ptext1673:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 46 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
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
psect	text1673
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.4\main.c"
	line	46
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
psect	text1673
	line	48
	
i1l9942:	
;main.c: 48: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u312_21
	goto	u312_20
u312_21:
	goto	i1l3708
u312_20:
	line	50
	
i1l9944:	
;main.c: 49: {
;main.c: 50: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	51
	
i1l9946:	
;main.c: 51: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	53
;main.c: 53: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	55
	
i1l9948:	
;main.c: 55: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	57
	
i1l9950:	
;main.c: 57: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
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
	goto	u313_21
	goto	u313_20
u313_21:
	goto	i1l9954
u313_20:
	
i1l9952:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l9954
	
i1l3698:	
	line	58
	
i1l9954:	
;main.c: 58: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
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
	goto	u314_21
	goto	u314_20
u314_21:
	goto	i1l9958
u314_20:
	
i1l9956:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l9958
	
i1l3699:	
	line	59
	
i1l9958:	
;main.c: 59: if(RTC_Counter % 500 == 0)
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
	line	62
;main.c: 60: {
	
i1l3700:	
	line	64
;main.c: 62: }
;main.c: 64: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u315_21
	goto	u315_20
u315_21:
	goto	i1l3701
u315_20:
	line	66
	
i1l9960:	
;main.c: 65: {
;main.c: 66: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	67
	
i1l9962:	
;main.c: 67: if(start.debounceCount >= 10 & start.released)
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
	goto	u316_21
	goto	u316_20
u316_21:
	goto	i1l9970
u316_20:
	line	69
	
i1l9964:	
;main.c: 68: {
;main.c: 69: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	70
	
i1l9966:	
;main.c: 70: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l9970
	line	71
	
i1l3702:	
	line	72
;main.c: 71: }
;main.c: 72: }
	goto	i1l9970
	line	73
	
i1l3701:	
	line	75
;main.c: 73: else
;main.c: 74: {
;main.c: 75: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	76
	
i1l9968:	
;main.c: 76: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l9970
	line	77
	
i1l3703:	
	line	79
	
i1l9970:	
;main.c: 77: }
;main.c: 79: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u317_21
	goto	u317_20
u317_21:
	goto	i1l9980
u317_20:
	
i1l9972:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	
i1l9974:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l9976:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u318_21
	goto	u318_20
u318_21:
	goto	i1l9980
u318_20:
	
i1l9978:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l9980
	
i1l3705:	
	goto	i1l9980
	
i1l3704:	
	
i1l9980:	
	btfss	(100/8),(100)&7
	goto	u319_21
	goto	u319_20
u319_21:
	goto	i1l3708
u319_20:
	
i1l9982:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u320_21
	goto	u320_20
u320_21:
	goto	i1l3708
u320_20:
	
i1l9984:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	movwf	(25)	;volatile
	
i1l9986:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l9988:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l9990:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u321_21
	goto	u321_20
u321_21:
	goto	i1l3708
u321_20:
	
i1l9992:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l3708
	
i1l3707:	
	goto	i1l3708
	
i1l3706:	
	goto	i1l3708
	line	80
	
i1l3697:	
	line	81
	
i1l3708:	
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
psect	text1674,local,class=CODE,delta=2
global __ptext1674
__ptext1674:

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
psect	text1674
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l10238:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u342_21
	goto	u342_20
u342_21:
	goto	i1l10256
u342_20:
	line	9
	
i1l10240:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l10246
	
i1l7426:	
	line	11
	
i1l10242:	
	movlw	01h
	
u343_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u343_25
	line	12
	
i1l10244:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l10246
	line	13
	
i1l7425:	
	line	10
	
i1l10246:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u344_21
	goto	u344_20
u344_21:
	goto	i1l10242
u344_20:
	goto	i1l10248
	
i1l7427:	
	goto	i1l10248
	line	14
	
i1l7428:	
	line	15
	
i1l10248:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u345_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u345_25:
	skipc
	goto	u345_21
	goto	u345_20
u345_21:
	goto	i1l10252
u345_20:
	line	16
	
i1l10250:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l10252
	
i1l7429:	
	line	17
	
i1l10252:	
	movlw	01h
	
u346_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u346_25
	line	18
	
i1l10254:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u347_21
	goto	u347_20
u347_21:
	goto	i1l10248
u347_20:
	goto	i1l10256
	
i1l7430:	
	goto	i1l10256
	line	19
	
i1l7424:	
	line	20
	
i1l10256:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l7431
	
i1l10258:	
	line	21
	
i1l7431:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text1675,local,class=CODE,delta=2
global __ptext1675
__ptext1675:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

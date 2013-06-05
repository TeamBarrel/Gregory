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
# 21 "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_checkForFinalDestination
	FNCALL	_main,_lookForVictim
	FNCALL	_main,_findWalls
	FNCALL	_main,_goToNextCell
	FNCALL	_main,_updateLocation
	FNCALL	_main,_updateNode
	FNCALL	_main,_checkIfHome
	FNCALL	_findWalls,_lcd_set_cursor
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_findWalls,_rotateIR
	FNCALL	_findWalls,_frontWallCorrect
	FNCALL	_findWalls,_rightWallCorrect
	FNCALL	_goToNextCell,_goLeft
	FNCALL	_goToNextCell,_goForward
	FNCALL	_goToNextCell,_goRight
	FNCALL	_goToNextCell,_goBackward
	FNCALL	_findWall,_readIR
	FNCALL	_frontWallCorrect,_drive
	FNCALL	_frontWallCorrect,_readIR
	FNCALL	_rightWallCorrect,_turnRight90
	FNCALL	_rightWallCorrect,_rotateIR
	FNCALL	_rightWallCorrect,_drive
	FNCALL	_rightWallCorrect,_readIR
	FNCALL	_rightWallCorrect,_turnLeft90
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
	FNCALL	_driveForDistance,_drive
	FNCALL	_driveForDistance,_ser_putch
	FNCALL	_driveForDistance,_ser_getch
	FNCALL	_adc_read_channel,_adc_read
	FNCALL	_convert,___wmul
	FNCALL	_convert,___awdiv
	FNCALL	_ser_putArr,_ser_putch
	FNCALL	_play_iCreate_song,_ser_putch
	FNCALL	_initIRobot,_ser_putch
	FNCALL	_waitFor,_ser_putch
	FNCALL	_ser_getch,_ser_isrx
	FNCALL	_drive,_ser_putch
	FNCALL	_adc_read,___awdiv
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
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	36

;initializer for _xCoord
	retlw	01h
	line	37

;initializer for _yCoord
	retlw	03h
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
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
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	36
_xCoord:
       ds      1

psect	dataCOMMON
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK1
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
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
	movlw	low((__pbssBANK0)+020h)
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
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x2
	ds	1
	global	?_readIR
?_readIR:	; 2 bytes @ 0x3
	ds	2
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x5
	ds	2
	global	??_rightWallCorrect
??_rightWallCorrect:	; 0 bytes @ 0x7
	global	??_frontWallCorrect
??_frontWallCorrect:	; 0 bytes @ 0x7
	ds	2
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x9
	ds	1
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
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
	global	?_init_adc
?_init_adc:	; 0 bytes @ 0x0
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
	global	??_turnAround
??_turnAround:	; 0 bytes @ 0x13
	global	??_turnLeft90
??_turnLeft90:	; 0 bytes @ 0x13
	global	??_turnRight90
??_turnRight90:	; 0 bytes @ 0x13
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0x13
	global	??_checkIfHome
??_checkIfHome:	; 0 bytes @ 0x13
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0x13
	global	ser_putArr@i
ser_putArr@i:	; 2 bytes @ 0x13
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0x14
	ds	1
	global	??_initSongs
??_initSongs:	; 0 bytes @ 0x15
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0x15
	global	??_init
??_init:	; 0 bytes @ 0x15
	global	___awdiv@counter
___awdiv@counter:	; 1 bytes @ 0x15
	ds	1
	global	___awdiv@sign
___awdiv@sign:	; 1 bytes @ 0x16
	ds	1
	global	driveForDistance@deltaDistance
driveForDistance@deltaDistance:	; 2 bytes @ 0x17
	global	___awdiv@quotient
___awdiv@quotient:	; 2 bytes @ 0x17
	ds	2
	global	?_adc_read
?_adc_read:	; 2 bytes @ 0x19
	global	driveForDistance@distance
driveForDistance@distance:	; 2 bytes @ 0x19
	ds	2
	global	??_adc_read
??_adc_read:	; 0 bytes @ 0x1B
	global	driveForDistance@high
driveForDistance@high:	; 1 bytes @ 0x1B
	ds	1
	global	driveForDistance@low
driveForDistance@low:	; 1 bytes @ 0x1C
	ds	1
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x1D
	global	??_goForward
??_goForward:	; 0 bytes @ 0x1D
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x1D
	global	??_goRight
??_goRight:	; 0 bytes @ 0x1D
	ds	1
	global	??_goToNextCell
??_goToNextCell:	; 0 bytes @ 0x1E
	ds	1
	global	adc_read@adc_value
adc_read@adc_value:	; 2 bytes @ 0x1F
	ds	2
	global	?_convert
?_convert:	; 2 bytes @ 0x21
	global	convert@adc_value
convert@adc_value:	; 2 bytes @ 0x21
	ds	2
	global	??_convert
??_convert:	; 0 bytes @ 0x23
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x25
	ds	1
	global	??_readIR
??_readIR:	; 0 bytes @ 0x26
	global	??_findWall
??_findWall:	; 0 bytes @ 0x26
	global	??_main
??_main:	; 0 bytes @ 0x26
;;Data sizes: Strings 34, constant 0, data 104, bss 50, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     38      70
;; BANK1           80     10      72
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
;;   _updateLocation->_lcd_set_cursor
;;   _updateLocation->_lcd_write_1_digit_bcd
;;   _lookForVictim->_lcd_set_cursor
;;   _lookForVictim->_lcd_write_1_digit_bcd
;;   _checkForFinalDestination->_lcd_set_cursor
;;   _readIR->_adc_read_channel
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _checkIfHome->_drive
;;   _initSongs->_ser_putArr
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_write_string->_lcd_write_data
;;   _turnRight90->_drive
;;   _turnLeft90->_drive
;;   _turnAround->_drive
;;   _lcd_set_cursor->_lcd_write_control
;;   _driveForDistance->_drive
;;   _adc_read_channel->_convert
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
;; (0) _main                                                 0     0      0   10110
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;           _checkForFinalDestination
;;                      _lookForVictim
;;                          _findWalls
;;                       _goToNextCell
;;                     _updateLocation
;;                         _updateNode
;;                        _checkIfHome
;; ---------------------------------------------------------------------------------
;; (1) _findWalls                                            1     1      0    5746
;;                                              9 BANK1      1     1      0
;;                     _lcd_set_cursor
;;                           _findWall
;;                     _lcd_write_data
;;                           _rotateIR
;;                   _frontWallCorrect
;;                   _rightWallCorrect
;; ---------------------------------------------------------------------------------
;; (1) _goToNextCell                                         0     0      0    2886
;;                             _goLeft
;;                          _goForward
;;                            _goRight
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1528
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (2) _frontWallCorrect                                     2     2      0    1683
;;                                              7 BANK1      2     2      0
;;                              _drive
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (2) _rightWallCorrect                                     2     2      0    2340
;;                                              7 BANK1      2     2      0
;;                        _turnRight90
;;                           _rotateIR
;;                              _drive
;;                             _readIR
;;                         _turnLeft90
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
;;                                              3 BANK1      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (2) _goRight                                              1     1      0     799
;;                                             29 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                        _turnRight90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goLeft                                               0     0      0     799
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnLeft90
;;                  _updateOrientation
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goForward                                            0     0      0     489
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                   _driveForDistance
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0     799
;;                                             29 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;                         _turnAround
;;                  _updateOrientation
;;                   _driveForDistance
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
;; (3) _driveForDistance                                    10     8      2     393
;;                                             19 BANK0     10     8      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;; ---------------------------------------------------------------------------------
;; (4) _adc_read_channel                                     4     2      2     510
;;                                             37 BANK0      1     1      0
;;                                              0 BANK1      3     1      2
;;                           _adc_read
;;                            _convert (ARG)
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
;; (4) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_data                                       3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (4) _ser_getch                                            2     2      0      34
;;                                             10 BANK0      2     2      0
;;                           _ser_isrx
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
;; (5) ___awdiv                                              9     5      4     445
;;                                             16 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (5) ___wmul                                               6     2      4     136
;;                                             10 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (5) _ser_isrx                                             0     0      0       0
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
;; (3) _updateOrientation                                    2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; (2) _getOrientation                                       0     0      0       0
;; ---------------------------------------------------------------------------------
;; (3) _ser_putch                                            2     2      0      31
;;                                             10 BANK0      2     2      0
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
;;BANK1               50      A      48       7       90.0%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      DC      12        0.0%
;;ABS                  0      0      D2       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       A       2        0.0%
;;BANK0               50     26      46       5       87.5%
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
;;		line 258 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
;;		_updateLocation
;;		_updateNode
;;		_checkIfHome
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	258
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	259
	
l10844:	
;main.c: 259: init();
	fcall	_init
	line	260
;main.c: 260: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	262
	
l10846:	
;main.c: 262: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	263
	
l10848:	
;main.c: 263: lcd_write_string("(-,-) E -- --- -");
	movlw	((STR_1-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	264
;main.c: 264: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	265
	
l10850:	
;main.c: 265: lcd_write_string("- - - (0,0) GREG");
	movlw	((STR_2-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	269
;main.c: 269: while(!home)
	goto	l10878
	
l3059:	
	line	271
	
l10852:	
;main.c: 270: {
;main.c: 271: if(start.pressed)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u4700
	goto	l10878
u4700:
	line	273
	
l10854:	
;main.c: 272: {
;main.c: 273: checkForFinalDestination();
	fcall	_checkForFinalDestination
	line	274
;main.c: 274: lookForVictim();
	fcall	_lookForVictim
	line	275
	
l10856:	
;main.c: 275: findWalls();
	fcall	_findWalls
	line	276
;main.c: 276: switch(node)
	goto	l10868
	line	278
;main.c: 277: {
;main.c: 278: case 0:
	
l3062:	
	line	279
	
l10858:	
;main.c: 279: goToNextCell();
	fcall	_goToNextCell
	line	280
;main.c: 280: break;
	goto	l10870
	line	281
;main.c: 281: case 1:
	
l3064:	
	line	282
	
l10860:	
;main.c: 282: goToNextCell();
	fcall	_goToNextCell
	line	283
;main.c: 283: break;
	goto	l10870
	line	284
;main.c: 284: case 2:
	
l3065:	
	line	285
	
l10862:	
;main.c: 285: goToNextCell();
	fcall	_goToNextCell
	line	286
;main.c: 286: break;
	goto	l10870
	line	287
;main.c: 287: case 3:
	
l3066:	
	line	288
	
l10864:	
;main.c: 288: goToNextCell();
	fcall	_goToNextCell
	line	289
;main.c: 289: break;
	goto	l10870
	line	290
;main.c: 290: default:
	
l3067:	
	line	291
;main.c: 291: break;
	goto	l10870
	line	292
	
l10866:	
;main.c: 292: }
	goto	l10870
	line	276
	
l3061:	
	
l10868:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_node),w	;volatile
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
	goto	l10858
	xorlw	1^0	; case 1
	skipnz
	goto	l10860
	xorlw	2^1	; case 2
	skipnz
	goto	l10862
	xorlw	3^2	; case 3
	skipnz
	goto	l10864
	goto	l10870
	opt asmopt_on

	line	292
	
l3063:	
	line	294
	
l10870:	
;main.c: 294: updateLocation();
	fcall	_updateLocation
	line	295
	
l10872:	
;main.c: 295: updateNode();
	fcall	_updateNode
	line	296
	
l10874:	
;main.c: 296: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4711
	goto	u4710
u4711:
	goto	l10878
u4710:
	line	297
	
l10876:	
;main.c: 297: checkIfHome();
	fcall	_checkIfHome
	goto	l10878
	
l3068:	
	goto	l10878
	line	298
	
l3060:	
	goto	l10878
	line	299
	
l3058:	
	line	269
	
l10878:	
	btfss	(_home/8),(_home)&7
	goto	u4721
	goto	u4720
u4721:
	goto	l10852
u4720:
	goto	l3070
	
l3069:	
	line	301
	
l3070:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_findWalls
psect	text1548,local,class=CODE,delta=2
global __ptext1548
__ptext1548:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 157 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1548
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	157
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	158
	
l10814:	
;main.c: 158: lcd_set_cursor(0x0B);
	movlw	(0Bh)
	fcall	_lcd_set_cursor
	line	160
	
l10816:	
;main.c: 160: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4611
	goto	u4610
	
u4611:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u4624
u4610:
	bcf	(_leftWall/8),(_leftWall)&7
u4624:
	line	161
	
l10818:	
;main.c: 161: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4631
	goto	u4630
u4631:
	goto	l10822
u4630:
	line	163
	
l10820:	
;main.c: 162: {
;main.c: 163: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	164
;main.c: 164: }
	goto	l3018
	line	165
	
l3017:	
	line	166
	
l10822:	
;main.c: 165: else
;main.c: 166: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3018:	
	line	167
;main.c: 167: rotateIR(24, 0b00001111);
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
	line	169
	
l10824:	
;main.c: 169: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4641
	goto	u4640
	
u4641:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u4654
u4640:
	bcf	(_frontWall/8),(_frontWall)&7
u4654:
	line	170
	
l10826:	
;main.c: 170: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4661
	goto	u4660
u4661:
	goto	l10832
u4660:
	line	172
	
l10828:	
;main.c: 171: {
;main.c: 172: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	173
	
l10830:	
;main.c: 173: frontWallCorrect();
	fcall	_frontWallCorrect
	line	174
;main.c: 174: }
	goto	l3020
	line	175
	
l3019:	
	line	176
	
l10832:	
;main.c: 175: else
;main.c: 176: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3020:	
	line	177
;main.c: 177: rotateIR(24, 0b00001111);
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
	line	179
	
l10834:	
;main.c: 179: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4671
	goto	u4670
	
u4671:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u4684
u4670:
	bcf	(_rightWall/8),(_rightWall)&7
u4684:
	line	180
	
l10836:	
;main.c: 180: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u4691
	goto	u4690
u4691:
	goto	l10842
u4690:
	line	182
	
l10838:	
;main.c: 181: {
;main.c: 182: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	183
	
l10840:	
;main.c: 183: rightWallCorrect();
	fcall	_rightWallCorrect
	line	184
;main.c: 184: }
	goto	l3022
	line	185
	
l3021:	
	line	186
	
l10842:	
;main.c: 185: else
;main.c: 186: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	
l3022:	
	line	187
;main.c: 187: rotateIR(48, 0b00001101);
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
	line	188
	
l3023:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goToNextCell
psect	text1549,local,class=CODE,delta=2
global __ptext1549
__ptext1549:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 191 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
;;		_goLeft
;;		_goForward
;;		_goRight
;;		_goBackward
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1549
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	191
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 1
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	192
	
l10804:	
;main.c: 192: if(!leftWall)
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u4581
	goto	u4580
u4581:
	goto	l3026
u4580:
	line	193
	
l10806:	
;main.c: 193: goLeft();
	fcall	_goLeft
	goto	l3032
	line	194
	
l3026:	
;main.c: 194: else if(!frontWall)
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u4591
	goto	u4590
u4591:
	goto	l3028
u4590:
	line	195
	
l10808:	
;main.c: 195: goForward();
	fcall	_goForward
	goto	l3032
	line	196
	
l3028:	
;main.c: 196: else if(!rightWall)
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u4601
	goto	u4600
u4601:
	goto	l10812
u4600:
	line	197
	
l10810:	
;main.c: 197: goRight();
	fcall	_goRight
	goto	l3032
	line	198
	
l3030:	
	line	199
	
l10812:	
;main.c: 198: else
;main.c: 199: goBackward();
	fcall	_goBackward
	goto	l3032
	
l3031:	
	goto	l3032
	
l3029:	
	goto	l3032
	
l3027:	
	line	200
	
l3032:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWall
psect	text1550,local,class=CODE,delta=2
global __ptext1550
__ptext1550:

;; *************** function _findWall *****************
;; Defined at:
;;		line 307 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1550
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	307
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	308
	
l10792:	
;main.c: 308: if(readIR() > 100)
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4575
	movlw	low(065h)
	subwf	(0+(?_readIR))^080h,w
u4575:

	skipc
	goto	u4571
	goto	u4570
u4571:
	goto	l10800
u4570:
	line	309
	
l10794:	
;main.c: 309: return 0;
	clrc
	
	goto	l3074
	
l10796:	
	goto	l3074
	
l10798:	
	goto	l3074
	line	310
	
l3073:	
	line	311
	
l10800:	
;main.c: 310: else
;main.c: 311: return 1;
	setc
	
	goto	l3074
	
l10802:	
	goto	l3074
	
l3075:	
	line	312
	
l3074:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_frontWallCorrect
psect	text1551,local,class=CODE,delta=2
global __ptext1551
__ptext1551:

;; *************** function _frontWallCorrect *****************
;; Defined at:
;;		line 203 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
psect	text1551
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	203
	global	__size_of_frontWallCorrect
	__size_of_frontWallCorrect	equ	__end_of_frontWallCorrect-_frontWallCorrect
	
_frontWallCorrect:	
	opt	stack 0
; Regs used in _frontWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	204
	
l10776:	
;drive.c: 204: while(readIR() < 50)
	goto	l10780
	
l1483:	
	line	206
	
l10778:	
;drive.c: 205: {
;drive.c: 206: drive(255, 131, 128, 0);
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
	goto	l10780
	line	207
	
l1482:	
	line	204
	
l10780:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(032h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4545
	movlw	low(032h)
	subwf	(0+(?_readIR))^080h,w
u4545:

	skipc
	goto	u4541
	goto	u4540
u4541:
	goto	l10778
u4540:
	goto	l10782
	
l1484:	
	line	208
	
l10782:	
;drive.c: 207: }
;drive.c: 208: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	210
;drive.c: 210: while(readIR() > 55 && readIR() < 100)
	goto	l10786
	
l1486:	
	line	212
	
l10784:	
;drive.c: 211: {
;drive.c: 212: drive(0, 250, 128, 0);
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
	goto	l10786
	line	213
	
l1485:	
	line	210
	
l10786:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4555
	movlw	low(038h)
	subwf	(0+(?_readIR))^080h,w
u4555:

	skipc
	goto	u4551
	goto	u4550
u4551:
	goto	l10790
u4550:
	
l10788:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(064h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4565
	movlw	low(064h)
	subwf	(0+(?_readIR))^080h,w
u4565:

	skipc
	goto	u4561
	goto	u4560
u4561:
	goto	l10784
u4560:
	goto	l10790
	
l1488:	
	goto	l10790
	
l1489:	
	line	214
	
l10790:	
;drive.c: 213: }
;drive.c: 214: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	215
	
l1490:	
	return
	opt stack 0
GLOBAL	__end_of_frontWallCorrect
	__end_of_frontWallCorrect:
;; =============== function _frontWallCorrect ends ============

	signat	_frontWallCorrect,88
	global	_rightWallCorrect
psect	text1552,local,class=CODE,delta=2
global __ptext1552
__ptext1552:

;; *************** function _rightWallCorrect *****************
;; Defined at:
;;		line 186 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
psect	text1552
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	186
	global	__size_of_rightWallCorrect
	__size_of_rightWallCorrect	equ	__end_of_rightWallCorrect-_rightWallCorrect
	
_rightWallCorrect:	
	opt	stack 0
; Regs used in _rightWallCorrect: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	187
	
l10758:	
;drive.c: 187: turnRight90();
	fcall	_turnRight90
	line	188
	
l10760:	
;drive.c: 188: rotateIR(24, 0b00001101);
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
	line	189
;drive.c: 189: while(readIR() <45)
	goto	l10764
	
l1474:	
	line	191
	
l10762:	
;drive.c: 190: {
;drive.c: 191: drive(255, 131, 128, 0);
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
	goto	l10764
	line	192
	
l1473:	
	line	189
	
l10764:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(02Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4525
	movlw	low(02Dh)
	subwf	(0+(?_readIR))^080h,w
u4525:

	skipc
	goto	u4521
	goto	u4520
u4521:
	goto	l10762
u4520:
	goto	l10768
	
l1475:	
	line	193
;drive.c: 192: }
;drive.c: 193: while(readIR() >55)
	goto	l10768
	
l1477:	
	line	195
	
l10766:	
;drive.c: 194: {
;drive.c: 195: drive(0, 250, 128, 0);
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
	goto	l10768
	line	196
	
l1476:	
	line	193
	
l10768:	
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(038h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4535
	movlw	low(038h)
	subwf	(0+(?_readIR))^080h,w
u4535:

	skipnc
	goto	u4531
	goto	u4530
u4531:
	goto	l10766
u4530:
	goto	l10770
	
l1478:	
	line	197
	
l10770:	
;drive.c: 196: }
;drive.c: 197: turnLeft90();
	fcall	_turnLeft90
	line	198
	
l10772:	
;drive.c: 198: rotateIR(24, 0b00001111);
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
	line	199
	
l10774:	
;drive.c: 199: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	200
	
l1479:	
	return
	opt stack 0
GLOBAL	__end_of_rightWallCorrect
	__end_of_rightWallCorrect:
;; =============== function _rightWallCorrect ends ============

	signat	_rightWallCorrect,88
	global	_updateLocation
psect	text1553,local,class=CODE,delta=2
global __ptext1553
__ptext1553:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 203 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1553
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	203
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	204
	
l10734:	
;main.c: 204: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	205
;main.c: 205: switch(getOrientation())
	goto	l10754
	line	207
;main.c: 206: {
;main.c: 207: case NORTH:
	
l3036:	
	line	208
	
l10736:	
;main.c: 208: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	209
	
l10738:	
;main.c: 209: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	210
;main.c: 210: break;
	goto	l10756
	line	211
;main.c: 211: case SOUTH:
	
l3038:	
	line	212
	
l10740:	
;main.c: 212: --yCoord;
	movlw	low(01h)
	subwf	(_yCoord),f	;volatile
	line	213
	
l10742:	
;main.c: 213: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	214
;main.c: 214: break;
	goto	l10756
	line	215
;main.c: 215: case EAST:
	
l3039:	
	line	216
	
l10744:	
;main.c: 216: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	217
	
l10746:	
;main.c: 217: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	218
;main.c: 218: break;
	goto	l10756
	line	219
;main.c: 219: case WEST:
	
l3040:	
	line	220
	
l10748:	
;main.c: 220: --xCoord;
	movlw	low(01h)
	subwf	(_xCoord),f	;volatile
	line	221
	
l10750:	
;main.c: 221: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	222
;main.c: 222: break;
	goto	l10756
	line	223
;main.c: 223: default:
	
l3041:	
	line	224
;main.c: 224: break;
	goto	l10756
	line	225
	
l10752:	
;main.c: 225: }
	goto	l10756
	line	205
	
l3035:	
	
l10754:	
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
	goto	l10748
	xorlw	1^0	; case 1
	skipnz
	goto	l10740
	xorlw	2^1	; case 2
	skipnz
	goto	l10744
	xorlw	3^2	; case 3
	skipnz
	goto	l10736
	goto	l10756
	opt asmopt_on

	line	225
	
l3037:	
	line	227
	
l10756:	
;main.c: 227: lcd_set_cursor(0x01);
	movlw	(01h)
	fcall	_lcd_set_cursor
	line	228
;main.c: 228: lcd_write_1_digit_bcd(xCoord);
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	229
;main.c: 229: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	230
;main.c: 230: lcd_write_1_digit_bcd(yCoord);
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	231
	
l3042:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_lookForVictim
psect	text1554,local,class=CODE,delta=2
global __ptext1554
__ptext1554:

;; *************** function _lookForVictim *****************
;; Defined at:
;;		line 135 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1554
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	135
	global	__size_of_lookForVictim
	__size_of_lookForVictim	equ	__end_of_lookForVictim-_lookForVictim
	
_lookForVictim:	
	opt	stack 3
; Regs used in _lookForVictim: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	137
	
l10716:	
;main.c: 137: if(victimFound)
	btfss	(_victimFound/8),(_victimFound)&7
	goto	u4501
	goto	u4500
u4501:
	goto	l3014
u4500:
	line	139
	
l10718:	
;main.c: 138: {
;main.c: 139: if(goingHome)
	btfss	(_goingHome/8),(_goingHome)&7
	goto	u4511
	goto	u4510
u4511:
	goto	l10728
u4510:
	line	141
	
l10720:	
;main.c: 140: {
;main.c: 141: play_iCreate_song(3);
	movlw	(03h)
	fcall	_play_iCreate_song
	line	142
	
l10722:	
;main.c: 142: victimZone = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_victimZone)	;volatile
	line	143
	
l10724:	
;main.c: 143: lcd_set_cursor(0x09);
	movlw	(09h)
	fcall	_lcd_set_cursor
	line	144
	
l10726:	
;main.c: 144: lcd_write_data('V');
	movlw	(056h)
	fcall	_lcd_write_data
	line	145
;main.c: 145: }
	goto	l3014
	line	146
	
l3012:	
	line	148
	
l10728:	
;main.c: 146: else
;main.c: 147: {
;main.c: 148: victimZone = getVictimZone(xCoord, yCoord);
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
	line	149
	
l10730:	
;main.c: 149: lcd_set_cursor(0x08);
	movlw	(08h)
	fcall	_lcd_set_cursor
	line	150
	
l10732:	
;main.c: 150: lcd_write_1_digit_bcd(victimZone);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_victimZone),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	goto	l3014
	line	151
	
l3013:	
	goto	l3014
	line	152
	
l3011:	
	line	153
	
l3014:	
	return
	opt stack 0
GLOBAL	__end_of_lookForVictim
	__end_of_lookForVictim:
;; =============== function _lookForVictim ends ============

	signat	_lookForVictim,88
	global	_checkForFinalDestination
psect	text1555,local,class=CODE,delta=2
global __ptext1555
__ptext1555:

;; *************** function _checkForFinalDestination *****************
;; Defined at:
;;		line 124 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1555
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	124
	global	__size_of_checkForFinalDestination
	__size_of_checkForFinalDestination	equ	__end_of_checkForFinalDestination-_checkForFinalDestination
	
_checkForFinalDestination:	
	opt	stack 3
; Regs used in _checkForFinalDestination: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	125
	
l10702:	
;main.c: 125: if(!goingHome && (xCoord == getFinalX()) && (yCoord == getFinalY()))
	btfsc	(_goingHome/8),(_goingHome)&7
	goto	u4471
	goto	u4470
u4471:
	goto	l3008
u4470:
	
l10704:	
	fcall	_getFinalX
	xorwf	(_xCoord),w	;volatile
	skipz
	goto	u4481
	goto	u4480
u4481:
	goto	l3008
u4480:
	
l10706:	
	fcall	_getFinalY
	xorwf	(_yCoord),w	;volatile
	skipz
	goto	u4491
	goto	u4490
u4491:
	goto	l3008
u4490:
	line	127
	
l10708:	
;main.c: 126: {
;main.c: 127: play_iCreate_song(2);
	movlw	(02h)
	fcall	_play_iCreate_song
	line	128
	
l10710:	
;main.c: 128: goingHome = 1;
	bsf	(_goingHome/8),(_goingHome)&7
	line	129
	
l10712:	
;main.c: 129: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	130
	
l10714:	
;main.c: 130: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l3008
	line	131
	
l3007:	
	line	132
	
l3008:	
	return
	opt stack 0
GLOBAL	__end_of_checkForFinalDestination
	__end_of_checkForFinalDestination:
;; =============== function _checkForFinalDestination ends ============

	signat	_checkForFinalDestination,88
	global	_init
psect	text1556,local,class=CODE,delta=2
global __ptext1556
__ptext1556:

;; *************** function _init *****************
;; Defined at:
;;		line 87 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1556
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	87
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	88
	
l10670:	
;main.c: 88: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	89
	
l10672:	
;main.c: 89: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	91
	
l10674:	
;main.c: 91: init_adc();
	fcall	_init_adc
	line	92
	
l10676:	
;main.c: 92: lcd_init();
	fcall	_lcd_init
	line	94
	
l10678:	
;main.c: 94: TRISB = 0b00000001;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	97
	
l10680:	
;main.c: 97: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	99
	
l10682:	
;main.c: 99: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	100
	
l10684:	
;main.c: 100: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	101
	
l10686:	
;main.c: 101: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	102
	
l10688:	
;main.c: 102: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	103
	
l10690:	
;main.c: 103: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	106
	
l10692:	
;main.c: 106: PEIE = 1;
	bsf	(94/8),(94)&7
	line	107
	
l10694:	
;main.c: 107: GIE = 1;
	bsf	(95/8),(95)&7
	line	109
	
l10696:	
;main.c: 109: ser_init();
	fcall	_ser_init
	line	110
	
l10698:	
;main.c: 110: initIRobot();
	fcall	_initIRobot
	line	111
	
l10700:	
;main.c: 111: initSongs();
	fcall	_initSongs
	line	112
	
l3001:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_readIR
psect	text1557,local,class=CODE,delta=2
global __ptext1557
__ptext1557:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  cm              2    5[BANK1 ] int 
;; Return value:  Size  Location     Type
;;                  2    3[BANK1 ] int 
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
psect	text1557
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 0
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l10664:	
;ir.c: 34: int cm = convert(adc_read_channel(0));
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

	line	35
	
l10666:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1)^080h,w
	clrf	(?_readIR+1)^080h
	addwf	(?_readIR+1)^080h
	movf	(readIR@cm)^080h,w
	clrf	(?_readIR)^080h
	addwf	(?_readIR)^080h

	goto	l6007
	
l10668:	
	line	36
	
l6007:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_goRight
psect	text1558,local,class=CODE,delta=2
global __ptext1558
__ptext1558:

;; *************** function _goRight *****************
;; Defined at:
;;		line 136 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1558
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	136
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 1
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	137
	
l10654:	
;drive.c: 137: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	138
;drive.c: 138: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	139
	
l10656:	
;drive.c: 139: turnRight90();
	fcall	_turnRight90
	line	140
	
l10658:	
;drive.c: 140: updateOrientation(RIGHT);
	movlw	(03h)
	fcall	_updateOrientation
	line	141
	
l10660:	
;drive.c: 141: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	142
	
l10662:	
;drive.c: 142: lastMove = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_lastMove)	;volatile
	line	143
	
l1454:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1559,local,class=CODE,delta=2
global __ptext1559
__ptext1559:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 102 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1559
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	102
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 1
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	103
	
l10644:	
;drive.c: 103: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	104
;drive.c: 104: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	105
	
l10646:	
;drive.c: 105: turnLeft90();
	fcall	_turnLeft90
	line	106
	
l10648:	
;drive.c: 106: updateOrientation(LEFT);
	movlw	(01h)
	fcall	_updateOrientation
	line	107
	
l10650:	
;drive.c: 107: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	108
	
l10652:	
;drive.c: 108: lastMove = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	bsf	status,0
	rlf	(_lastMove),f	;volatile
	line	109
	
l1445:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1560,local,class=CODE,delta=2
global __ptext1560
__ptext1560:

;; *************** function _goForward *****************
;; Defined at:
;;		line 93 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_driveForDistance
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1560
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	93
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 1
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	94
	
l10638:	
;drive.c: 94: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	95
;drive.c: 95: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	96
	
l10640:	
;drive.c: 96: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	97
	
l10642:	
;drive.c: 97: lastMove = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_lastMove)	;volatile
	line	98
	
l1442:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1561,local,class=CODE,delta=2
global __ptext1561
__ptext1561:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 82 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
psect	text1561
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	82
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 1
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	83
	
l10628:	
;drive.c: 83: lcd_set_cursor(0x0F);
	movlw	(0Fh)
	fcall	_lcd_set_cursor
	line	84
;drive.c: 84: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	85
	
l10630:	
;drive.c: 85: turnAround();
	fcall	_turnAround
	line	86
	
l10632:	
;drive.c: 86: updateOrientation(BACKWARD);
	movlw	(02h)
	fcall	_updateOrientation
	line	87
	
l10634:	
;drive.c: 87: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	88
	
l10636:	
;drive.c: 88: lastMove = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_lastMove)	;volatile
	line	89
	
l1439:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_checkIfHome
psect	text1562,local,class=CODE,delta=2
global __ptext1562
__ptext1562:

;; *************** function _checkIfHome *****************
;; Defined at:
;;		line 246 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1562
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	246
	global	__size_of_checkIfHome
	__size_of_checkIfHome	equ	__end_of_checkIfHome-_checkIfHome
	
_checkIfHome:	
	opt	stack 3
; Regs used in _checkIfHome: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	247
	
l10620:	
;main.c: 247: if((xCoord == 1) && (yCoord == 3))
	movf	(_xCoord),w	;volatile
	xorlw	01h
	skipz
	goto	u4451
	goto	u4450
u4451:
	goto	l3055
u4450:
	
l10622:	
	movf	(_yCoord),w	;volatile
	xorlw	03h
	skipz
	goto	u4461
	goto	u4460
u4461:
	goto	l3055
u4460:
	line	249
	
l10624:	
;main.c: 248: {
;main.c: 249: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	250
;main.c: 250: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	line	251
	
l10626:	
;main.c: 251: home = 1;
	bsf	(_home/8),(_home)&7
	goto	l3055
	line	252
	
l3054:	
	line	253
	
l3055:	
	return
	opt stack 0
GLOBAL	__end_of_checkIfHome
	__end_of_checkIfHome:
;; =============== function _checkIfHome ends ============

	signat	_checkIfHome,88
	global	_initSongs
psect	text1563,local,class=CODE,delta=2
global __ptext1563
__ptext1563:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 30 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
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
psect	text1563
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
	line	30
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	31
	
l10618:	
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
	
l5301:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text1564,local,class=CODE,delta=2
global __ptext1564
__ptext1564:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
psect	text1564
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l10598:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l10600:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l10602:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l10604:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l10606:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l10608:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l10610:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l10612:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l10614:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l10616:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l2204:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1565,local,class=CODE,delta=2
global __ptext1565
__ptext1565:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
psect	text1565
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
	
l10596:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l2192:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_write_string
psect	text1566,local,class=CODE,delta=2
global __ptext1566
__ptext1566:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
psect	text1566
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
	
l10588:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l10594
	
l2187:	
	
l10590:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l10592:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l10594
	
l2186:	
	
l10594:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u4441
	goto	u4440
u4441:
	goto	l10590
u4440:
	goto	l2189
	
l2188:	
	line	41
	
l2189:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_turnRight90
psect	text1567,local,class=CODE,delta=2
global __ptext1567
__ptext1567:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 163 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_goReverse
;; This function uses a non-reentrant model
;;
psect	text1567
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	163
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 1
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	164
	
l10584:	
;drive.c: 164: drive(0, 25, 255, 255);
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
	line	165
;drive.c: 165: waitFor(157,255,169);
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
	line	166
;drive.c: 166: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	167
	
l10586:	
;drive.c: 167: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	224
movwf	((??_turnRight90+0)+0+1),f
	movlw	254
movwf	((??_turnRight90+0)+0),f
u4737:
	decfsz	((??_turnRight90+0)+0),f
	goto	u4737
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u4737
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u4737
opt asmopt_on

	line	168
	
l1463:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_turnLeft90
psect	text1568,local,class=CODE,delta=2
global __ptext1568
__ptext1568:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 155 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_goReverse
;; This function uses a non-reentrant model
;;
psect	text1568
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	155
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 1
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	156
	
l10580:	
;drive.c: 156: drive(0, 25, 0, 1);
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
	line	157
;drive.c: 157: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	158
;drive.c: 158: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	159
	
l10582:	
;drive.c: 159: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	224
movwf	((??_turnLeft90+0)+0+1),f
	movlw	254
movwf	((??_turnLeft90+0)+0),f
u4747:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u4747
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u4747
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u4747
opt asmopt_on

	line	160
	
l1460:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnAround
psect	text1569,local,class=CODE,delta=2
global __ptext1569
__ptext1569:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 146 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
psect	text1569
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	146
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	147
	
l10574:	
;drive.c: 147: drive(0, 25, 0, 1);
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
	line	148
;drive.c: 148: waitFor(157,0,170);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0AAh)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	149
;drive.c: 149: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	150
	
l10576:	
;drive.c: 150: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u4757:
	decfsz	((??_turnAround+0)+0),f
	goto	u4757
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4757
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4757
opt asmopt_on

	line	151
	
l10578:	
;drive.c: 151: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u4767:
	decfsz	((??_turnAround+0)+0),f
	goto	u4767
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4767
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4767
opt asmopt_on

	line	152
	
l1457:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_lcd_set_cursor
psect	text1570,local,class=CODE,delta=2
global __ptext1570
__ptext1570:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
;;		_goRight
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_updateLocation
;;		_main
;;		_goReverse
;;		_findFinalDestination
;; This function uses a non-reentrant model
;;
psect	text1570
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
	
l10570:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l10572:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l2183:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_driveForDistance
psect	text1571,local,class=CODE,delta=2
global __ptext1571
__ptext1571:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 29 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
;; Parameters:    Size  Location     Type
;;  moveDistance    2   19[BANK0 ] int 
;; Auto vars:     Size  Location     Type
;;  distance        2   25[BANK0 ] int 
;;  deltaDistanc    2   23[BANK0 ] int 
;;  low             1   28[BANK0 ] volatile unsigned char 
;;  high            1   27[BANK0 ] volatile unsigned char 
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
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_drive
;;		_ser_putch
;;		_ser_getch
;; This function is called by:
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;; This function uses a non-reentrant model
;;
psect	text1571
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	29
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 1
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	32
	
l10552:	
;drive.c: 31: volatile char high, low;
;drive.c: 32: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	33
;drive.c: 33: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	35
	
l10554:	
;drive.c: 35: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	36
	
l10556:	
;drive.c: 36: drive(0, 250, 128, 0);
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
	line	38
;drive.c: 38: while(moving)
	goto	l1426
	
l1427:	
	line	40
	
l10558:	
;drive.c: 39: {
;drive.c: 40: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	41
;drive.c: 41: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	42
;drive.c: 42: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	43
;drive.c: 43: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	44
	
l10560:	
;drive.c: 44: deltaDistance = high*256 + low;
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
	line	45
	
l10562:	
;drive.c: 45: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	64
	
l10564:	
;drive.c: 64: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u4425
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u4425:

	skipc
	goto	u4421
	goto	u4420
u4421:
	goto	l1426
u4420:
	line	65
	
l10566:	
;drive.c: 65: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l1426
	
l1428:	
	line	66
	
l1426:	
	line	38
	btfsc	(_moving/8),(_moving)&7
	goto	u4431
	goto	u4430
u4431:
	goto	l10558
u4430:
	goto	l10568
	
l1429:	
	line	67
	
l10568:	
;drive.c: 66: }
;drive.c: 67: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	68
	
l1430:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_adc_read_channel
psect	text1572,local,class=CODE,delta=2
global __ptext1572
__ptext1572:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
;; Parameters:    Size  Location     Type
;;  channel         1    wreg     unsigned char 
;; Auto vars:     Size  Location     Type
;;  channel         1    2[BANK1 ] unsigned char 
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
;;      Temps:          0       1       0       0       0
;;      Totals:         0       1       3       0       0
;;Total ram usage:        4 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    4
;; This function calls:
;;		_adc_read
;; This function is called by:
;;		_readIR
;; This function uses a non-reentrant model
;;
psect	text1572
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
	
l10536:	
;adc.c: 8: switch(channel)
	goto	l10544
	line	10
;adc.c: 9: {
;adc.c: 10: case 0:
	
l690:	
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
	goto	l10546
	line	15
;adc.c: 15: case 1:
	
l692:	
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
	goto	l10546
	line	20
;adc.c: 20: case 2:
	
l693:	
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
	goto	l10546
	line	25
;adc.c: 25: case 3:
	
l694:	
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
	goto	l10546
	line	30
;adc.c: 30: case 4:
	
l695:	
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
	goto	l10546
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l10538:	
;adc.c: 38: return 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(?_adc_read_channel)^080h
	clrf	(?_adc_read_channel+1)^080h
	goto	l697
	
l10540:	
	goto	l697
	line	39
	
l10542:	
;adc.c: 39: }
	goto	l10546
	line	8
	
l689:	
	
l10544:	
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
	goto	l10538
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l10546:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
movwf	(??_adc_read_channel+0)+0,f
u4777:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u4777
opt asmopt_on

	line	43
	
l10548:	
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

	goto	l697
	
l10550:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_convert
psect	text1573,local,class=CODE,delta=2
global __ptext1573
__ptext1573:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
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
psect	text1573
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 1
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l10476:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4355
	movlw	low(052h)
	subwf	(convert@adc_value),w
u4355:

	skipnc
	goto	u4351
	goto	u4350
u4351:
	goto	l10484
u4350:
	line	13
	
l10478:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5991
	
l10480:	
	goto	l5991
	
l10482:	
	goto	l5991
	line	14
	
l5990:	
	
l10484:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4365
	movlw	low(085h)
	subwf	(convert@adc_value),w
u4365:

	skipnc
	goto	u4361
	goto	u4360
u4361:
	goto	l10492
u4360:
	line	15
	
l10486:	
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
	goto	l5991
	
l10488:	
	goto	l5991
	
l10490:	
	goto	l5991
	line	16
	
l5993:	
	
l10492:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4375
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u4375:

	skipnc
	goto	u4371
	goto	u4370
u4371:
	goto	l10500
u4370:
	line	17
	
l10494:	
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
	goto	l5991
	
l10496:	
	goto	l5991
	
l10498:	
	goto	l5991
	line	18
	
l5995:	
	
l10500:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4385
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u4385:

	skipnc
	goto	u4381
	goto	u4380
u4381:
	goto	l10508
u4380:
	line	19
	
l10502:	
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
	goto	l5991
	
l10504:	
	goto	l5991
	
l10506:	
	goto	l5991
	line	20
	
l5997:	
	
l10508:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u4395
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u4395:

	skipnc
	goto	u4391
	goto	u4390
u4391:
	goto	l10516
u4390:
	line	21
	
l10510:	
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
	goto	l5991
	
l10512:	
	goto	l5991
	
l10514:	
	goto	l5991
	line	22
	
l5999:	
	
l10516:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4405
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u4405:

	skipnc
	goto	u4401
	goto	u4400
u4401:
	goto	l10524
u4400:
	line	23
	
l10518:	
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
	goto	l5991
	
l10520:	
	goto	l5991
	
l10522:	
	goto	l5991
	line	24
	
l6001:	
	
l10524:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u4415
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u4415:

	skipnc
	goto	u4411
	goto	u4410
u4411:
	goto	l10532
u4410:
	line	25
	
l10526:	
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
	goto	l5991
	
l10528:	
	goto	l5991
	
l10530:	
	goto	l5991
	line	26
	
l6003:	
	
l10532:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5991
	
l10534:	
	goto	l5991
	
l6004:	
	goto	l5991
	
l6002:	
	goto	l5991
	
l6000:	
	goto	l5991
	
l5998:	
	goto	l5991
	
l5996:	
	goto	l5991
	
l5994:	
	goto	l5991
	
l5992:	
	line	27
	
l5991:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_ser_putArr
psect	text1574,local,class=CODE,delta=2
global __ptext1574
__ptext1574:

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
psect	text1574
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l10468:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l10474
	line	75
	
l4571:	
	line	76
	
l10470:	
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
	goto	u4330
	decf	(??_ser_putArr+0)+0,f
u4330:
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
	
l10472:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l10474
	
l4570:	
	
l10474:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u4345
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u4345:

	skipc
	goto	u4341
	goto	u4340
u4341:
	goto	l10470
u4340:
	goto	l4573
	
l4572:	
	line	78
	
l4573:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putArr
	__end_of_ser_putArr:
;; =============== function _ser_putArr ends ============

	signat	_ser_putArr,8312
	global	_play_iCreate_song
psect	text1575,local,class=CODE,delta=2
global __ptext1575
__ptext1575:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 24 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
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
psect	text1575
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\songs.c"
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
	
l10466:	
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
	
l5298:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_initIRobot
psect	text1576,local,class=CODE,delta=2
global __ptext1576
__ptext1576:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 115 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1576
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	115
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	116
	
l10460:	
;main.c: 116: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u4787:
	decfsz	((??_initIRobot+0)+0),f
	goto	u4787
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u4787
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u4787
	nop2
opt asmopt_on

	line	117
	
l10462:	
;main.c: 117: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	118
	
l10464:	
;main.c: 118: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	119
	
l3004:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_lcd_write_control
psect	text1577,local,class=CODE,delta=2
global __ptext1577
__ptext1577:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
psect	text1577
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
	
l10452:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l10454:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l10456:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l10458:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u4797:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u4797
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u4797
	nop2
opt asmopt_on

	line	16
	
l2177:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_rotateIR
psect	text1578,local,class=CODE,delta=2
global __ptext1578
__ptext1578:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
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
psect	text1578
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ir.c"
	line	39
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 3
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	40
	
l10434:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l10436:	
;ir.c: 41: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	42
	
l10438:	
;ir.c: 42: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u4807:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4807
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4807
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4807
	nop2
opt asmopt_on

	line	44
	
l10440:	
;ir.c: 44: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l6010
	line	45
	
l6011:	
	line	46
;ir.c: 45: {
;ir.c: 46: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	47
	
l10442:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l10444:	
;ir.c: 48: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u4817:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4817
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4817
	nop2
opt asmopt_on

	line	44
	
l10446:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l6010:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u4321
	goto	u4320
u4321:
	goto	l6011
u4320:
	goto	l10448
	
l6012:	
	line	51
	
l10448:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l10450:	
;ir.c: 52: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u4827:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4827
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4827
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4827
	nop2
opt asmopt_on

	line	54
	
l6013:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_waitFor
psect	text1579,local,class=CODE,delta=2
global __ptext1579
__ptext1579:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 178 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_goReverse
;; This function uses a non-reentrant model
;;
psect	text1579
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	178
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 1
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	179
	
l10426:	
;drive.c: 179: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u4837:
	decfsz	((??_waitFor+0)+0),f
	goto	u4837
	decfsz	((??_waitFor+0)+0+1),f
	goto	u4837
	decfsz	((??_waitFor+0)+0+2),f
	goto	u4837
	nop2
opt asmopt_on

	line	180
	
l10428:	
;drive.c: 180: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	181
	
l10430:	
;drive.c: 181: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	182
	
l10432:	
;drive.c: 182: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	183
	
l1470:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_lcd_write_data
psect	text1580,local,class=CODE,delta=2
global __ptext1580
__ptext1580:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
;;		_goRight
;;		_lcd_write_string
;;		_lcd_write_1_digit_bcd
;;		_checkForFinalDestination
;;		_lookForVictim
;;		_findWalls
;;		_updateLocation
;;		_goReverse
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1580
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\lcd.c"
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
	
l10418:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l10420:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l10422:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l10424:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u4847:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u4847
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u4847
opt asmopt_on

	line	28
	
l2180:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_ser_getch
psect	text1581,local,class=CODE,delta=2
global __ptext1581
__ptext1581:

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
;;		_driveForDistance
;; This function uses a non-reentrant model
;;
psect	text1581
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 1
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l10402:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l10404
	
l4565:	
	line	62
;ser.c: 62: continue;
	goto	l10404
	
l4564:	
	line	61
	
l10404:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u4311
	goto	u4310
u4311:
	goto	l10404
u4310:
	
l4566:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l10406:	
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
	
l10408:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l10410:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l10412:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l10414:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l4567
	
l10416:	
	line	70
	
l4567:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_drive
psect	text1582,local,class=CODE,delta=2
global __ptext1582
__ptext1582:

;; *************** function _drive *****************
;; Defined at:
;;		line 19 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_turnAround
;;		_turnLeft90
;;		_turnRight90
;;		_rightWallCorrect
;;		_frontWallCorrect
;;		_checkIfHome
;;		_main
;;		_goReverse
;; This function uses a non-reentrant model
;;
psect	text1582
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	19
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 3
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	20
	
l10390:	
;drive.c: 20: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u4857:
	decfsz	((??_drive+0)+0),f
	goto	u4857
	decfsz	((??_drive+0)+0+1),f
	goto	u4857
	decfsz	((??_drive+0)+0+2),f
	goto	u4857
	nop2
opt asmopt_on

	line	21
	
l10392:	
;drive.c: 21: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	22
	
l10394:	
;drive.c: 22: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	23
	
l10396:	
;drive.c: 23: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	24
	
l10398:	
;drive.c: 24: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	25
	
l10400:	
;drive.c: 25: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	26
	
l1423:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_init_adc
psect	text1583,local,class=CODE,delta=2
global __ptext1583
__ptext1583:

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
psect	text1583
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l10380:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l10382:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l10384:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l10386:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l10388:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u4867:
decfsz	(??_init_adc+0)+0,f
	goto	u4867
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
psect	text1584,local,class=CODE,delta=2
global __ptext1584
__ptext1584:

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
psect	text1584
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 0
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l10370:	
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
	
l10372:	
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
	goto	u4291
	goto	u4290
u4291:
	goto	l703
u4290:
	goto	l10374
	
l705:	
	line	75
	
l10374:	
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
u4305:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u4305
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l10376:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l10378:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1585,local,class=CODE,delta=2
global __ptext1585
__ptext1585:

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
psect	text1585
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 1
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l10330:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u4191
	goto	u4190
u4191:
	goto	l10334
u4190:
	line	11
	
l10332:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l10334
	line	13
	
l6836:	
	line	14
	
l10334:	
	btfss	(___awdiv@dividend+1),7
	goto	u4201
	goto	u4200
u4201:
	goto	l10340
u4200:
	line	15
	
l10336:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l10338:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l10340
	line	17
	
l6837:	
	line	18
	
l10340:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l10342:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u4211
	goto	u4210
u4211:
	goto	l10362
u4210:
	line	20
	
l10344:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l10350
	
l6840:	
	line	22
	
l10346:	
	movlw	01h
	
u4225:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u4225
	line	23
	
l10348:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l10350
	line	24
	
l6839:	
	line	21
	
l10350:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u4231
	goto	u4230
u4231:
	goto	l10346
u4230:
	goto	l10352
	
l6841:	
	goto	l10352
	line	25
	
l6842:	
	line	26
	
l10352:	
	movlw	01h
	
u4245:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u4245
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u4255
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u4255:
	skipc
	goto	u4251
	goto	u4250
u4251:
	goto	l10358
u4250:
	line	28
	
l10354:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l10356:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l10358
	line	30
	
l6843:	
	line	31
	
l10358:	
	movlw	01h
	
u4265:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u4265
	line	32
	
l10360:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u4271
	goto	u4270
u4271:
	goto	l10352
u4270:
	goto	l10362
	
l6844:	
	goto	l10362
	line	33
	
l6838:	
	line	34
	
l10362:	
	movf	(___awdiv@sign),w
	skipz
	goto	u4280
	goto	l10366
u4280:
	line	35
	
l10364:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l10366
	
l6845:	
	line	36
	
l10366:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l6846
	
l10368:	
	line	37
	
l6846:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1586,local,class=CODE,delta=2
global __ptext1586
__ptext1586:

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
psect	text1586
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 1
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l10318:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l10320
	line	6
	
l6696:	
	line	7
	
l10320:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u4151
	goto	u4150
u4151:
	goto	l6697
u4150:
	line	8
	
l10322:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l6697:	
	line	9
	movlw	01h
	
u4165:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u4165
	line	10
	
l10324:	
	movlw	01h
	
u4175:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u4175
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u4181
	goto	u4180
u4181:
	goto	l10320
u4180:
	goto	l10326
	
l6698:	
	line	12
	
l10326:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l6699
	
l10328:	
	line	13
	
l6699:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_ser_isrx
psect	text1587,local,class=CODE,delta=2
global __ptext1587
__ptext1587:

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
psect	text1587
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 1
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l10270:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u4081
	goto	u4080
u4081:
	goto	l10278
u4080:
	line	50
	
l10272:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l10274:	
;ser.c: 52: return 0;
	clrc
	
	goto	l4561
	
l10276:	
	goto	l4561
	line	53
	
l4560:	
	line	54
	
l10278:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u4091
	goto	u4090
u4091:
	goto	l10282
u4090:
	
l10280:	
	clrc
	
	goto	l4561
	
l9994:	
	
l10282:	
	setc
	
	goto	l4561
	
l9996:	
	goto	l4561
	
l10284:	
	line	55
	
l4561:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_updateNode
psect	text1588,local,class=CODE,delta=2
global __ptext1588
__ptext1588:

;; *************** function _updateNode *****************
;; Defined at:
;;		line 234 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1588
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	234
	global	__size_of_updateNode
	__size_of_updateNode	equ	__end_of_updateNode-_updateNode
	
_updateNode:	
	opt	stack 5
; Regs used in _updateNode: [wreg+status,2+status,0]
	line	235
	
l10168:	
;main.c: 235: if((xCoord == 2) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u3991
	goto	u3990
u3991:
	goto	l10174
u3990:
	
l10170:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4001
	goto	u4000
u4001:
	goto	l10174
u4000:
	line	236
	
l10172:	
;main.c: 236: node = 1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_node)	;volatile
	bsf	status,0
	rlf	(_node),f	;volatile
	goto	l3051
	line	237
	
l3045:	
	
l10174:	
;main.c: 237: else if((xCoord == 4) && (yCoord == 2))
	movf	(_xCoord),w	;volatile
	xorlw	04h
	skipz
	goto	u4011
	goto	u4010
u4011:
	goto	l10180
u4010:
	
l10176:	
	movf	(_yCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4021
	goto	u4020
u4021:
	goto	l10180
u4020:
	line	238
	
l10178:	
;main.c: 238: node = 2;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l3051
	line	239
	
l3047:	
	
l10180:	
;main.c: 239: else if((xCoord == 2) && (yCoord == 0))
	movf	(_xCoord),w	;volatile
	xorlw	02h
	skipz
	goto	u4031
	goto	u4030
u4031:
	goto	l3049
u4030:
	
l10182:	
	movf	(_yCoord),f
	skipz	;volatile
	goto	u4041
	goto	u4040
u4041:
	goto	l3049
u4040:
	line	240
	
l10184:	
;main.c: 240: node = 3;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateNode+0)+0
	movf	(??_updateNode+0)+0,w
	movwf	(_node)	;volatile
	goto	l3051
	line	241
	
l3049:	
	line	242
;main.c: 241: else
;main.c: 242: node = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_node)	;volatile
	goto	l3051
	
l3050:	
	goto	l3051
	
l3048:	
	goto	l3051
	
l3046:	
	line	243
	
l3051:	
	return
	opt stack 0
GLOBAL	__end_of_updateNode
	__end_of_updateNode:
;; =============== function _updateNode ends ============

	signat	_updateNode,88
	global	_getVictimZone
psect	text1589,local,class=CODE,delta=2
global __ptext1589
__ptext1589:

;; *************** function _getVictimZone *****************
;; Defined at:
;;		line 157 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
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
psect	text1589
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
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
	
l10120:	
;map.c: 163: switch (victimX)
	goto	l10162
	line	165
;map.c: 164: {
;map.c: 165: case 0:
	
l3828:	
	line	166
;map.c: 166: switch (victimY)
	goto	l10128
	line	168
;map.c: 167: {
;map.c: 168: case 0:
	
l3830:	
	line	169
	
l10122:	
;map.c: 169: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	170
;map.c: 170: break;
	goto	l10164
	line	171
;map.c: 171: case 1:
	
l3832:	
	line	172
	
l10124:	
;map.c: 172: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	173
;map.c: 173: break;
	goto	l10164
	line	178
;map.c: 178: default:
	
l3833:	
	line	179
;map.c: 179: break;
	goto	l10164
	line	180
	
l10126:	
;map.c: 180: }
	goto	l10164
	line	166
	
l3829:	
	
l10128:	
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
	goto	l10122
	xorlw	1^0	; case 1
	skipnz
	goto	l10124
	goto	l10164
	opt asmopt_on

	line	180
	
l3831:	
	line	181
;map.c: 181: break;
	goto	l10164
	line	183
;map.c: 183: case 1:
	
l3835:	
	line	184
;map.c: 184: switch (victimY)
	goto	l10136
	line	186
;map.c: 185: {
;map.c: 186: case 0:
	
l3837:	
	line	187
	
l10130:	
;map.c: 187: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	188
;map.c: 188: break;
	goto	l10164
	line	189
;map.c: 189: case 1:
	
l3839:	
	line	190
	
l10132:	
;map.c: 190: vicZone = 4;
	movlw	(04h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	191
;map.c: 191: break;
	goto	l10164
	line	196
;map.c: 196: default:
	
l3840:	
	line	197
;map.c: 197: break;
	goto	l10164
	line	198
	
l10134:	
;map.c: 198: }
	goto	l10164
	line	184
	
l3836:	
	
l10136:	
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
	goto	l10130
	xorlw	1^0	; case 1
	skipnz
	goto	l10132
	goto	l10164
	opt asmopt_on

	line	198
	
l3838:	
	line	199
;map.c: 199: break;
	goto	l10164
	line	201
;map.c: 201: case 2:
	
l3841:	
	line	202
;map.c: 202: switch (victimY)
	goto	l10144
	line	206
;map.c: 203: {
;map.c: 206: case 1:
	
l3843:	
	line	207
	
l10138:	
;map.c: 207: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	208
;map.c: 208: break;
	goto	l10164
	line	211
;map.c: 211: case 3:
	
l3845:	
	line	212
	
l10140:	
;map.c: 212: vicZone = 1;
	clrf	(_vicZone)
	bsf	status,0
	rlf	(_vicZone),f
	line	213
;map.c: 213: break;
	goto	l10164
	line	214
;map.c: 214: default:
	
l3846:	
	line	215
;map.c: 215: break;
	goto	l10164
	line	216
	
l10142:	
;map.c: 216: }
	goto	l10164
	line	202
	
l3842:	
	
l10144:	
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
	goto	l10138
	xorlw	3^1	; case 3
	skipnz
	goto	l10140
	goto	l10164
	opt asmopt_on

	line	216
	
l3844:	
	line	217
;map.c: 217: break;
	goto	l10164
	line	219
;map.c: 219: case 3:
	
l3847:	
	line	220
;map.c: 220: switch (victimY)
	goto	l10152
	line	224
;map.c: 221: {
;map.c: 224: case 1:
	
l3849:	
	line	225
	
l10146:	
;map.c: 225: vicZone = 3;
	movlw	(03h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	226
;map.c: 226: break;
	goto	l10164
	line	229
;map.c: 229: case 3:
	
l3851:	
	line	230
	
l10148:	
;map.c: 230: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	231
;map.c: 231: break;
	goto	l10164
	line	232
;map.c: 232: default:
	
l3852:	
	line	233
;map.c: 233: break;
	goto	l10164
	line	234
	
l10150:	
;map.c: 234: }
	goto	l10164
	line	220
	
l3848:	
	
l10152:	
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
	goto	l10146
	xorlw	3^1	; case 3
	skipnz
	goto	l10148
	goto	l10164
	opt asmopt_on

	line	234
	
l3850:	
	line	235
;map.c: 235: break;
	goto	l10164
	line	237
;map.c: 237: case 4:
	
l3853:	
	line	238
;map.c: 238: switch (victimY)
	goto	l10158
	line	246
;map.c: 239: {
;map.c: 246: case 3:
	
l3855:	
	line	247
	
l10154:	
;map.c: 247: vicZone = 2;
	movlw	(02h)
	movwf	(??_getVictimZone+0)+0
	movf	(??_getVictimZone+0)+0,w
	movwf	(_vicZone)
	line	248
;map.c: 248: break;
	goto	l10164
	line	249
;map.c: 249: default:
	
l3857:	
	line	250
;map.c: 250: break;
	goto	l10164
	line	251
	
l10156:	
;map.c: 251: }
	goto	l10164
	line	238
	
l3854:	
	
l10158:	
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
	goto	l10154
	goto	l10164
	opt asmopt_on

	line	251
	
l3856:	
	line	252
;map.c: 252: break;
	goto	l10164
	line	254
;map.c: 254: default:
	
l3858:	
	line	255
;map.c: 255: break;
	goto	l10164
	line	256
	
l10160:	
;map.c: 256: }
	goto	l10164
	line	163
	
l3827:	
	
l10162:	
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
	goto	l10128
	xorlw	1^0	; case 1
	skipnz
	goto	l10136
	xorlw	2^1	; case 2
	skipnz
	goto	l10144
	xorlw	3^2	; case 3
	skipnz
	goto	l10152
	xorlw	4^3	; case 4
	skipnz
	goto	l10158
	goto	l10164
	opt asmopt_on

	line	256
	
l3834:	
	line	258
	
l10164:	
;map.c: 258: return vicZone;
	movf	(_vicZone),w
	goto	l3859
	
l10166:	
	line	259
	
l3859:	
	return
	opt stack 0
GLOBAL	__end_of_getVictimZone
	__end_of_getVictimZone:
;; =============== function _getVictimZone ends ============

	signat	_getVictimZone,8313
	global	_getFinalY
psect	text1590,local,class=CODE,delta=2
global __ptext1590
__ptext1590:

;; *************** function _getFinalY *****************
;; Defined at:
;;		line 152 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
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
psect	text1590
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
	line	152
	global	__size_of_getFinalY
	__size_of_getFinalY	equ	__end_of_getFinalY-_getFinalY
	
_getFinalY:	
	opt	stack 4
; Regs used in _getFinalY: [wreg]
	line	153
	
l10116:	
;map.c: 153: return finalY;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalY),w
	goto	l3824
	
l10118:	
	line	154
	
l3824:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalY
	__end_of_getFinalY:
;; =============== function _getFinalY ends ============

	signat	_getFinalY,89
	global	_getFinalX
psect	text1591,local,class=CODE,delta=2
global __ptext1591
__ptext1591:

;; *************** function _getFinalX *****************
;; Defined at:
;;		line 147 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
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
psect	text1591
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\map.c"
	line	147
	global	__size_of_getFinalX
	__size_of_getFinalX	equ	__end_of_getFinalX-_getFinalX
	
_getFinalX:	
	opt	stack 4
; Regs used in _getFinalX: [wreg]
	line	148
	
l10112:	
;map.c: 148: return finalX;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_finalX),w
	goto	l3821
	
l10114:	
	line	149
	
l3821:	
	return
	opt stack 0
GLOBAL	__end_of_getFinalX
	__end_of_getFinalX:
;; =============== function _getFinalX ends ============

	signat	_getFinalX,89
	global	_ser_init
psect	text1592,local,class=CODE,delta=2
global __ptext1592
__ptext1592:

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
psect	text1592
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l10086:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l10088:	
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
	
l10090:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l10092:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l10094:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l10096:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l10098:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l10100:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l10102:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l10104:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l10106:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l10108:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l10110:	
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
	
l4601:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_updateOrientation
psect	text1593,local,class=CODE,delta=2
global __ptext1593
__ptext1593:

;; *************** function _updateOrientation *****************
;; Defined at:
;;		line 171 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
;;		_goRight
;;		_goReverse
;; This function uses a non-reentrant model
;;
psect	text1593
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	171
	global	__size_of_updateOrientation
	__size_of_updateOrientation	equ	__end_of_updateOrientation-_updateOrientation
	
_updateOrientation:	
	opt	stack 3
; Regs used in _updateOrientation: [wreg+status,2+status,0]
;updateOrientation@moved stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(updateOrientation@moved)
	line	172
	
l10040:	
;drive.c: 172: currentOrientation += moved;
	movf	(updateOrientation@moved),w	;volatile
	movwf	(??_updateOrientation+0)+0
	movf	(??_updateOrientation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	173
	
l10042:	
;drive.c: 173: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u3941
	goto	u3940
u3941:
	goto	l1467
u3940:
	line	174
	
l10044:	
;drive.c: 174: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l1467
	
l1466:	
	line	175
	
l1467:	
	return
	opt stack 0
GLOBAL	__end_of_updateOrientation
	__end_of_updateOrientation:
;; =============== function _updateOrientation ends ============

	signat	_updateOrientation,4216
	global	_getOrientation
psect	text1594,local,class=CODE,delta=2
global __ptext1594
__ptext1594:

;; *************** function _getOrientation *****************
;; Defined at:
;;		line 71 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
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
psect	text1594
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\drive.c"
	line	71
	global	__size_of_getOrientation
	__size_of_getOrientation	equ	__end_of_getOrientation-_getOrientation
	
_getOrientation:	
	opt	stack 4
; Regs used in _getOrientation: [wreg]
	line	72
	
l10036:	
;drive.c: 72: return currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	goto	l1433
	
l10038:	
	line	73
	
l1433:	
	return
	opt stack 0
GLOBAL	__end_of_getOrientation
	__end_of_getOrientation:
;; =============== function _getOrientation ends ============

	signat	_getOrientation,89
	global	_ser_putch
psect	text1595,local,class=CODE,delta=2
global __ptext1595
__ptext1595:

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
psect	text1595
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
	
l10024:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l10026
	
l4577:	
	line	83
;ser.c: 83: continue;
	goto	l10026
	
l4576:	
	line	82
	
l10026:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u3931
	goto	u3930
u3931:
	goto	l10026
u3930:
	
l4578:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l10028:	
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
	
l10030:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l10032:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l10034:	
;ser.c: 88: GIE=1;
	bsf	(95/8),(95)&7
	line	89
	
l4579:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_isr1
psect	text1596,local,class=CODE,delta=2
global __ptext1596
__ptext1596:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 47 in file "C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
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
psect	text1596
	file	"C:\Users\10999959\Desktop\COMPETITIONv0.4\main.c"
	line	47
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
psect	text1596
	line	49
	
i1l9126:	
;main.c: 49: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u303_21
	goto	u303_20
u303_21:
	goto	i1l2998
u303_20:
	line	51
	
i1l9128:	
;main.c: 50: {
;main.c: 51: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	52
	
i1l9130:	
;main.c: 52: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	54
;main.c: 54: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	56
	
i1l9132:	
;main.c: 56: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	58
	
i1l9134:	
;main.c: 58: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
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
	goto	u304_21
	goto	u304_20
u304_21:
	goto	i1l9138
u304_20:
	
i1l9136:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l9138
	
i1l2988:	
	line	59
	
i1l9138:	
;main.c: 59: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
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
	goto	u305_21
	goto	u305_20
u305_21:
	goto	i1l9142
u305_20:
	
i1l9140:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l9142
	
i1l2989:	
	line	60
	
i1l9142:	
;main.c: 60: if(RTC_Counter % 500 == 0)
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
	line	63
;main.c: 61: {
	
i1l2990:	
	line	65
;main.c: 63: }
;main.c: 65: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u306_21
	goto	u306_20
u306_21:
	goto	i1l2991
u306_20:
	line	67
	
i1l9144:	
;main.c: 66: {
;main.c: 67: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	68
	
i1l9146:	
;main.c: 68: if(start.debounceCount >= 10 & start.released)
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
	goto	u307_21
	goto	u307_20
u307_21:
	goto	i1l9154
u307_20:
	line	70
	
i1l9148:	
;main.c: 69: {
;main.c: 70: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	71
	
i1l9150:	
;main.c: 71: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l9154
	line	72
	
i1l2992:	
	line	73
;main.c: 72: }
;main.c: 73: }
	goto	i1l9154
	line	74
	
i1l2991:	
	line	76
;main.c: 74: else
;main.c: 75: {
;main.c: 76: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	77
	
i1l9152:	
;main.c: 77: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l9154
	line	78
	
i1l2993:	
	line	80
	
i1l9154:	
;main.c: 78: }
;main.c: 80: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u308_21
	goto	u308_20
u308_21:
	goto	i1l9164
u308_20:
	
i1l9156:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	
i1l9158:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l9160:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u309_21
	goto	u309_20
u309_21:
	goto	i1l9164
u309_20:
	
i1l9162:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l9164
	
i1l2995:	
	goto	i1l9164
	
i1l2994:	
	
i1l9164:	
	btfss	(100/8),(100)&7
	goto	u310_21
	goto	u310_20
u310_21:
	goto	i1l2998
u310_20:
	
i1l9166:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u311_21
	goto	u311_20
u311_21:
	goto	i1l2998
u311_20:
	
i1l9168:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	movwf	(25)	;volatile
	
i1l9170:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l9172:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l9174:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u312_21
	goto	u312_20
u312_21:
	goto	i1l2998
u312_20:
	
i1l9176:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l2998
	
i1l2997:	
	goto	i1l2998
	
i1l2996:	
	goto	i1l2998
	line	81
	
i1l2987:	
	line	82
	
i1l2998:	
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
psect	text1597,local,class=CODE,delta=2
global __ptext1597
__ptext1597:

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
psect	text1597
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l9422:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u333_21
	goto	u333_20
u333_21:
	goto	i1l9440
u333_20:
	line	9
	
i1l9424:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l9430
	
i1l6714:	
	line	11
	
i1l9426:	
	movlw	01h
	
u334_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u334_25
	line	12
	
i1l9428:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l9430
	line	13
	
i1l6713:	
	line	10
	
i1l9430:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u335_21
	goto	u335_20
u335_21:
	goto	i1l9426
u335_20:
	goto	i1l9432
	
i1l6715:	
	goto	i1l9432
	line	14
	
i1l6716:	
	line	15
	
i1l9432:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u336_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u336_25:
	skipc
	goto	u336_21
	goto	u336_20
u336_21:
	goto	i1l9436
u336_20:
	line	16
	
i1l9434:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l9436
	
i1l6717:	
	line	17
	
i1l9436:	
	movlw	01h
	
u337_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u337_25
	line	18
	
i1l9438:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u338_21
	goto	u338_20
u338_21:
	goto	i1l9432
u338_20:
	goto	i1l9440
	
i1l6718:	
	goto	i1l9440
	line	19
	
i1l6712:	
	line	20
	
i1l9440:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l6719
	
i1l9442:	
	line	21
	
i1l6719:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text1598,local,class=CODE,delta=2
global __ptext1598
__ptext1598:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

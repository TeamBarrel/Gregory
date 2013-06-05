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
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_run
	FNCALL	_run,_lcd_set_cursor
	FNCALL	_run,_lcd_write_string
	FNCALL	_run,_goToNextCell
	FNCALL	_run,_updateLocation
	FNCALL	_goToNextCell,_findWalls
	FNCALL	_goToNextCell,_goRight
	FNCALL	_goToNextCell,_goForward
	FNCALL	_goToNextCell,_goLeft
	FNCALL	_goToNextCell,_goBackward
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_rotateIR
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_findWall,_readIR
	FNCALL	_updateLocation,_lcd_set_cursor
	FNCALL	_updateLocation,_lcd_write_data
	FNCALL	_updateLocation,_lcd_write_1_digit_bcd
	FNCALL	_goRight,_turnRight90
	FNCALL	_goRight,_driveForDistance
	FNCALL	_goRight,_lcd_set_cursor
	FNCALL	_goRight,_lcd_write_data
	FNCALL	_goLeft,_turnLeft90
	FNCALL	_goLeft,_driveForDistance
	FNCALL	_goLeft,_lcd_set_cursor
	FNCALL	_goLeft,_lcd_write_data
	FNCALL	_goForward,_driveForDistance
	FNCALL	_goForward,_lcd_set_cursor
	FNCALL	_goForward,_lcd_write_data
	FNCALL	_goBackward,_turnAround
	FNCALL	_goBackward,_driveForDistance
	FNCALL	_goBackward,_lcd_set_cursor
	FNCALL	_goBackward,_lcd_write_data
	FNCALL	_readIR,_adc_read_channel
	FNCALL	_readIR,_convert
	FNCALL	_init,_init_adc
	FNCALL	_init,_lcd_init
	FNCALL	_init,_ser_init
	FNCALL	_init,_initIRobot
	FNCALL	_lcd_init,_lcd_write_control
	FNCALL	_lcd_write_1_digit_bcd,_lcd_write_data
	FNCALL	_lcd_write_string,_lcd_write_data
	FNCALL	_lcd_set_cursor,_lcd_write_control
	FNCALL	_turnRight90,_drive
	FNCALL	_turnRight90,_waitFor
	FNCALL	_turnLeft90,_drive
	FNCALL	_turnLeft90,_waitFor
	FNCALL	_turnAround,_drive
	FNCALL	_turnAround,_waitFor
	FNCALL	_driveForDistance,_drive
	FNCALL	_driveForDistance,_ser_putch
	FNCALL	_driveForDistance,_ser_getch
	FNCALL	_adc_read_channel,_adc_read
	FNCALL	_convert,___wmul
	FNCALL	_convert,___awdiv
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
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	39

;initializer for _xCoord
	retlw	01h
	global	_rxfifo
	global	_start
	global	_RTC_Counter
	global	_directionMoved
	global	_ser_tmp
	global	_yCoord
	global	_currentOrientation
	global	_rxiptr
	global	_rxoptr
	global	_txiptr
	global	_txoptr
	global	_RTC_FLAG_10MS
	global	_RTC_FLAG_1MS
	global	_RTC_FLAG_500MS
	global	_RTC_FLAG_50MS
	global	_frontWall
	global	_leftWall
	global	_rightWall
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
	retlw	87	;'W'
	retlw	97	;'a'
	retlw	108	;'l'
	retlw	108	;'l'
	retlw	115	;'s'
	retlw	64	;'@'
	retlw	32	;' '
	retlw	82	;'R'
	retlw	32	;' '
	retlw	76	;'L'
	retlw	32	;' '
	retlw	40	;'('
	retlw	49	;'1'
	retlw	44	;','
	retlw	48	;'0'
	retlw	41	;')'
	retlw	0
psect	strings
	
STR_2:	
	retlw	99	;'c'
	retlw	117	;'u'
	retlw	79	;'O'
	retlw	114	;'r'
	retlw	58	;':'
	retlw	32	;' '
	retlw	87	;'W'
	retlw	32	;' '
	retlw	100	;'d'
	retlw	105	;'i'
	retlw	114	;'r'
	retlw	77	;'M'
	retlw	111	;'o'
	retlw	58	;':'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	0
psect	strings
	file	"COMPv0.1.as"
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

_leftWall:
       ds      1

_rightWall:
       ds      1

psect	bssCOMMON,class=COMMON,space=1
global __pbssCOMMON
__pbssCOMMON:
_currentOrientation:
       ds      1

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
_rxfifo:
       ds      16

_start:
       ds      3

_RTC_Counter:
       ds      2

_directionMoved:
       ds      1

_ser_tmp:
       ds      1

_yCoord:
       ds      1

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
_xCoord:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_txfifo:
       ds      16

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
; Clear objects allocated to COMMON
psect cinit,class=CODE,delta=2
	clrf	((__pbssCOMMON)+0)&07Fh
	clrf	((__pbssCOMMON)+1)&07Fh
	clrf	((__pbssCOMMON)+2)&07Fh
	clrf	((__pbssCOMMON)+3)&07Fh
	clrf	((__pbssCOMMON)+4)&07Fh
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
	movlw	low((__pbssBANK1)+010h)
	fcall	clear_ram
; Initialize objects allocated to BANK0
	global __pidataBANK0
psect cinit,class=CODE,delta=2
	fcall	__pidataBANK0+0		;fetch initializer
	movwf	__pdataBANK0+0&07fh		
psect cinit,class=CODE,delta=2
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
clrf status
ljmp _main	;jump to C main() function
psect	cstackBANK1,class=BANK1,space=1
global __pcstackBANK1
__pcstackBANK1:
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x0
	ds	3
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
	global	?_ser_init
?_ser_init:	; 0 bytes @ 0x0
	global	?_initIRobot
?_initIRobot:	; 0 bytes @ 0x0
	global	?_rotateIR
?_rotateIR:	; 0 bytes @ 0x0
	global	?_goRight
?_goRight:	; 0 bytes @ 0x0
	global	?_updateLocation
?_updateLocation:	; 0 bytes @ 0x0
	global	?_init_adc
?_init_adc:	; 0 bytes @ 0x0
	global	?_turnAround
?_turnAround:	; 0 bytes @ 0x0
	global	?_turnLeft90
?_turnLeft90:	; 0 bytes @ 0x0
	global	?_turnRight90
?_turnRight90:	; 0 bytes @ 0x0
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
	global	?_isr1
?_isr1:	; 0 bytes @ 0x0
	global	?_init
?_init:	; 0 bytes @ 0x0
	global	?_findWall
?_findWall:	; 1 bit 
	global	?_findWalls
?_findWalls:	; 0 bytes @ 0x0
	global	?_goBackward
?_goBackward:	; 0 bytes @ 0x0
	global	?_goForward
?_goForward:	; 0 bytes @ 0x0
	global	?_goLeft
?_goLeft:	; 0 bytes @ 0x0
	global	?_goToNextCell
?_goToNextCell:	; 0 bytes @ 0x0
	global	?_run
?_run:	; 0 bytes @ 0x0
	global	?_main
?_main:	; 0 bytes @ 0x0
	global	?_ser_isrx
?_ser_isrx:	; 1 bit 
	global	?_ser_getch
?_ser_getch:	; 1 bytes @ 0x0
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
	global	??_ser_init
??_ser_init:	; 0 bytes @ 0xA
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0xA
	global	??_init_adc
??_init_adc:	; 0 bytes @ 0xA
	global	??_lcd_write_control
??_lcd_write_control:	; 0 bytes @ 0xA
	global	??_lcd_write_data
??_lcd_write_data:	; 0 bytes @ 0xA
	global	??_ser_isrx
??_ser_isrx:	; 0 bytes @ 0xA
	global	?___wmul
?___wmul:	; 2 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	ds	1
	global	ser_getch@c
ser_getch@c:	; 1 bytes @ 0xB
	global	ser_putch@c
ser_putch@c:	; 1 bytes @ 0xB
	ds	1
	global	?_waitFor
?_waitFor:	; 0 bytes @ 0xC
	global	??_initIRobot
??_initIRobot:	; 0 bytes @ 0xC
	global	?_drive
?_drive:	; 0 bytes @ 0xC
	global	drive@lowByteSpeed
drive@lowByteSpeed:	; 1 bytes @ 0xC
	global	waitFor@highByte
waitFor@highByte:	; 1 bytes @ 0xC
	global	lcd_write_control@databyte
lcd_write_control@databyte:	; 1 bytes @ 0xC
	global	lcd_write_data@databyte
lcd_write_data@databyte:	; 1 bytes @ 0xC
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
	global	rotateIR@steps
rotateIR@steps:	; 1 bytes @ 0xD
	ds	1
	global	??_waitFor
??_waitFor:	; 0 bytes @ 0xE
	global	??_updateLocation
??_updateLocation:	; 0 bytes @ 0xE
	global	??___wmul
??___wmul:	; 0 bytes @ 0xE
	global	drive@lowByteRadius
drive@lowByteRadius:	; 1 bytes @ 0xE
	global	lcd_write_string@s
lcd_write_string@s:	; 1 bytes @ 0xE
	global	rotateIR@stepNum
rotateIR@stepNum:	; 1 bytes @ 0xE
	global	___wmul@product
___wmul@product:	; 2 bytes @ 0xE
	ds	1
	global	??_drive
??_drive:	; 0 bytes @ 0xF
	global	??_init
??_init:	; 0 bytes @ 0xF
	ds	1
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
	global	?_driveForDistance
?_driveForDistance:	; 0 bytes @ 0x13
	global	??_turnAround
??_turnAround:	; 0 bytes @ 0x13
	global	??_turnLeft90
??_turnLeft90:	; 0 bytes @ 0x13
	global	??_turnRight90
??_turnRight90:	; 0 bytes @ 0x13
	global	driveForDistance@moveDistance
driveForDistance@moveDistance:	; 2 bytes @ 0x13
	ds	1
	global	??___awdiv
??___awdiv:	; 0 bytes @ 0x14
	ds	1
	global	??_driveForDistance
??_driveForDistance:	; 0 bytes @ 0x15
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
	global	??_goRight
??_goRight:	; 0 bytes @ 0x1D
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x1D
	global	??_goForward
??_goForward:	; 0 bytes @ 0x1D
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x1D
	ds	2
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
	global	?_adc_read_channel
?_adc_read_channel:	; 2 bytes @ 0x25
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x27
	ds	1
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x28
	ds	1
	global	?_readIR
?_readIR:	; 2 bytes @ 0x29
	ds	2
	global	??_readIR
??_readIR:	; 0 bytes @ 0x2B
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x2B
	ds	2
	global	??_findWall
??_findWall:	; 0 bytes @ 0x2D
	global	??_goToNextCell
??_goToNextCell:	; 0 bytes @ 0x2D
	global	??_run
??_run:	; 0 bytes @ 0x2D
	global	??_main
??_main:	; 0 bytes @ 0x2D
;;Data sizes: Strings 34, constant 0, data 1, bss 45, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     45      70
;; BANK1           80      3      19
;; BANK3           96      0       0
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
;; ?___lwmod	unsigned int  size(1) Largest target is 0
;;
;; ?_adc_read	int  size(1) Largest target is 0
;;
;; ?___awdiv	int  size(1) Largest target is 0
;;
;; ?_adc_read_channel	int  size(1) Largest target is 0
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
;;   _findWall->_readIR
;;   _updateLocation->_lcd_set_cursor
;;   _updateLocation->_lcd_write_1_digit_bcd
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _readIR->_adc_read_channel
;;   _init->_initIRobot
;;   _lcd_init->_lcd_write_control
;;   _lcd_write_1_digit_bcd->_lcd_write_data
;;   _lcd_write_string->_lcd_write_data
;;   _lcd_set_cursor->_lcd_write_control
;;   _turnRight90->_drive
;;   _turnLeft90->_drive
;;   _turnAround->_drive
;;   _driveForDistance->_drive
;;   _adc_read_channel->_convert
;;   _convert->_adc_read
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
;;   _goToNextCell->_findWalls
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
;; (0) _main                                                 0     0      0    4646
;;                               _init
;;                              _drive
;;                                _run
;; ---------------------------------------------------------------------------------
;; (1) _run                                                  0     0      0    4429
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;                       _goToNextCell
;;                     _updateLocation
;; ---------------------------------------------------------------------------------
;; (2) _goToNextCell                                         0     0      0    4110
;;                          _findWalls
;;                            _goRight
;;                          _goForward
;;                             _goLeft
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (3) _findWalls                                            3     3      0    1317
;;                                              0 BANK1      3     3      0
;;                           _findWall
;;                           _rotateIR
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (4) _findWall                                             0     0      0    1218
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (2) _updateLocation                                       1     1      0     158
;;                                             14 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (3) _goRight                                              1     1      0     768
;;                                             29 BANK0      1     1      0
;;                        _turnRight90
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _goLeft                                               0     0      0     768
;;                         _turnLeft90
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _goForward                                            0     0      0     489
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _goBackward                                           1     1      0     768
;;                                             29 BANK0      1     1      0
;;                         _turnAround
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (5) _readIR                                               4     2      2    1218
;;                                             41 BANK0      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (1) _init                                                 0     0      0      62
;;                           _init_adc
;;                           _lcd_init
;;                           _ser_init
;;                         _initIRobot
;; ---------------------------------------------------------------------------------
;; (2) _lcd_init                                             0     0      0      31
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_1_digit_bcd                                1     1      0      62
;;                                             13 BANK0      1     1      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _lcd_write_string                                     2     2      0      96
;;                                             13 BANK0      2     2      0
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _lcd_set_cursor                                       1     1      0      65
;;                                             13 BANK0      1     1      0
;;                  _lcd_write_control
;; ---------------------------------------------------------------------------------
;; (4) _turnRight90                                          3     3      0     279
;;                                             19 BANK0      3     3      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (4) _turnLeft90                                           1     1      0     279
;;                                             19 BANK0      1     1      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (4) _turnAround                                           1     1      0     279
;;                                             19 BANK0      1     1      0
;;                              _drive
;;                            _waitFor
;; ---------------------------------------------------------------------------------
;; (4) _driveForDistance                                    10     8      2     393
;;                                             19 BANK0     10     8      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;; ---------------------------------------------------------------------------------
;; (6) _adc_read_channel                                     4     2      2     345
;;                                             37 BANK0      4     2      2
;;                           _adc_read
;;                            _convert (ARG)
;; ---------------------------------------------------------------------------------
;; (6) _convert                                              4     2      2     839
;;                                             33 BANK0      4     2      2
;;                             ___wmul
;;                            ___awdiv
;;                           _adc_read (ARG)
;; ---------------------------------------------------------------------------------
;; (4) _rotateIR                                             5     5      0      68
;;                                             10 BANK0      5     5      0
;; ---------------------------------------------------------------------------------
;; (2) _initIRobot                                           3     3      0      31
;;                                             12 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (4) _lcd_write_data                                       3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (4) _lcd_write_control                                    3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (5) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (5) _ser_getch                                            2     2      0      34
;;                                             10 BANK0      2     2      0
;;                           _ser_isrx
;; ---------------------------------------------------------------------------------
;; (5) _drive                                                7     4      3     155
;;                                             12 BANK0      7     4      3
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _init_adc                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (7) _adc_read                                             8     6      2     323
;;                                             25 BANK0      8     6      2
;;                            ___awdiv
;; ---------------------------------------------------------------------------------
;; (7) ___awdiv                                              9     5      4     300
;;                                             16 BANK0      9     5      4
;;                             ___wmul (ARG)
;; ---------------------------------------------------------------------------------
;; (7) ___wmul                                               6     2      4     136
;;                                             10 BANK0      6     2      4
;; ---------------------------------------------------------------------------------
;; (6) _ser_isrx                                             0     0      0       0
;; ---------------------------------------------------------------------------------
;; (2) _ser_init                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (5) _ser_putch                                            2     2      0      31
;;                                             10 BANK0      2     2      0
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 7
;; ---------------------------------------------------------------------------------
;; (Depth) Function   	        Calls       Base Space   Used Autos Params    Refs
;; ---------------------------------------------------------------------------------
;; (9) _isr1                                                10    10      0     159
;;                                              0 BANK0     10    10      0
;;                            ___lwmod
;; ---------------------------------------------------------------------------------
;; (10) ___lwmod                                             6     2      4     159
;;                                              0 COMMON     6     2      4
;; ---------------------------------------------------------------------------------
;; Estimated maximum stack depth 10
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
;;   _drive
;;     _ser_putch
;;   _run
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_string
;;       _lcd_write_data
;;     _goToNextCell
;;       _findWalls
;;         _findWall
;;           _readIR
;;             _adc_read_channel
;;               _adc_read
;;                 ___awdiv
;;                   ___wmul (ARG)
;;               _convert (ARG)
;;                 ___wmul
;;                 ___awdiv
;;                   ___wmul (ARG)
;;                 _adc_read (ARG)
;;                   ___awdiv
;;                     ___wmul (ARG)
;;             _convert
;;               ___wmul
;;               ___awdiv
;;                 ___wmul (ARG)
;;               _adc_read (ARG)
;;                 ___awdiv
;;                   ___wmul (ARG)
;;         _rotateIR
;;         _lcd_write_data
;;       _goRight
;;         _turnRight90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _driveForDistance
;;           _drive
;;             _ser_putch
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;       _goForward
;;         _driveForDistance
;;           _drive
;;             _ser_putch
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;       _goLeft
;;         _turnLeft90
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _driveForDistance
;;           _drive
;;             _ser_putch
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;       _goBackward
;;         _turnAround
;;           _drive
;;             _ser_putch
;;           _waitFor
;;             _ser_putch
;;         _driveForDistance
;;           _drive
;;             _ser_putch
;;           _ser_putch
;;           _ser_getch
;;             _ser_isrx
;;         _lcd_set_cursor
;;           _lcd_write_control
;;         _lcd_write_data
;;     _updateLocation
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;       _lcd_write_1_digit_bcd
;;         _lcd_write_data
;;
;; _isr1 (ROOT)
;;   ___lwmod
;;

;; Address spaces:

;;Name               Size   Autos  Total    Cost      Usage
;;BANK3               60      0       0       9        0.0%
;;BITBANK3            60      0       0       8        0.0%
;;SFR3                 0      0       0       4        0.0%
;;BITSFR3              0      0       0       4        0.0%
;;BANK2               60      0       0      11        0.0%
;;BITBANK2            60      0       0      10        0.0%
;;SFR2                 0      0       0       5        0.0%
;;BITSFR2              0      0       0       5        0.0%
;;SFR1                 0      0       0       2        0.0%
;;BITSFR1              0      0       0       2        0.0%
;;BANK1               50      3      13       7       23.8%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      71      12        0.0%
;;ABS                  0      0      65       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       C       2        0.0%
;;BANK0               50     2D      46       5       87.5%
;;BITBANK0            50      0       0       4        0.0%
;;SFR0                 0      0       0       1        0.0%
;;BITSFR0              0      0       0       1        0.0%
;;COMMON               E      6       C       1       85.7%
;;BITCOMMON            E      0       1       0        7.1%
;;EEDATA             100      0       0       0        0.0%

	global	_main
psect	maintext,global,class=CODE,delta=2
global __pmaintext
__pmaintext:

;; *************** function _main *****************
;; Defined at:
;;		line 273 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;; Hardware stack levels required when called:   10
;; This function calls:
;;		_init
;;		_drive
;;		_run
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	273
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	274
	
l8316:	
;main.c: 274: init();
	fcall	_init
	line	275
;main.c: 275: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	goto	l8318
	line	276
;main.c: 276: while(1)
	
l2969:	
	line	278
	
l8318:	
;main.c: 277: {
;main.c: 278: if(start.pressed)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u3790
	goto	l8318
u3790:
	line	279
	
l8320:	
;main.c: 279: run();
	fcall	_run
	goto	l8318
	
l2970:	
	goto	l8318
	line	280
	
l2971:	
	line	276
	goto	l8318
	
l2972:	
	line	281
	
l2973:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_run
psect	text1218,local,class=CODE,delta=2
global __ptext1218
__ptext1218:

;; *************** function _run *****************
;; Defined at:
;;		line 223 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;; Hardware stack levels required when called:    9
;; This function calls:
;;		_lcd_set_cursor
;;		_lcd_write_string
;;		_goToNextCell
;;		_updateLocation
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1218
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	223
	global	__size_of_run
	__size_of_run	equ	__end_of_run-_run
	
_run:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _run: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	224
	
l8306:	
;main.c: 224: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	225
	
l8308:	
;main.c: 225: lcd_write_string("Walls@ R L (1,0)");
	movlw	((STR_1-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	226
	
l8310:	
;main.c: 226: lcd_write_string("cuOr: W dirMo: -");
	movlw	((STR_2-__stringbase))&0ffh
	fcall	_lcd_write_string
	goto	l8312
	line	227
;main.c: 227: while(1)
	
l2952:	
	line	229
	
l8312:	
;main.c: 228: {
;main.c: 229: goToNextCell();
	fcall	_goToNextCell
	line	230
	
l8314:	
;main.c: 230: updateLocation();
	fcall	_updateLocation
	goto	l8312
	line	232
	
l2953:	
	line	227
	goto	l8312
	
l2954:	
	line	233
	
l2955:	
	return
	opt stack 0
GLOBAL	__end_of_run
	__end_of_run:
;; =============== function _run ends ============

	signat	_run,88
	global	_goToNextCell
psect	text1219,local,class=CODE,delta=2
global __ptext1219
__ptext1219:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 200 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;; Hardware stack levels required when called:    8
;; This function calls:
;;		_findWalls
;;		_goRight
;;		_goForward
;;		_goLeft
;;		_goBackward
;; This function is called by:
;;		_run
;; This function uses a non-reentrant model
;;
psect	text1219
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	200
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	201
	
l8294:	
;main.c: 201: findWalls();
	fcall	_findWalls
	line	202
	
l8296:	
;main.c: 202: if(!rightWall)
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u3761
	goto	u3760
u3761:
	goto	l2940
u3760:
	line	203
	
l8298:	
;main.c: 203: goRight();
	fcall	_goRight
	goto	l2946
	line	204
	
l2940:	
;main.c: 204: else if(!frontWall)
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u3771
	goto	u3770
u3771:
	goto	l2942
u3770:
	line	205
	
l8300:	
;main.c: 205: goForward();
	fcall	_goForward
	goto	l2946
	line	206
	
l2942:	
;main.c: 206: else if(!leftWall)
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u3781
	goto	u3780
u3781:
	goto	l8304
u3780:
	line	207
	
l8302:	
;main.c: 207: goLeft();
	fcall	_goLeft
	goto	l2946
	line	208
	
l2944:	
	line	209
	
l8304:	
;main.c: 208: else
;main.c: 209: goBackward();
	fcall	_goBackward
	goto	l2946
	
l2945:	
	goto	l2946
	
l2943:	
	goto	l2946
	
l2941:	
	line	210
	
l2946:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWalls
psect	text1220,local,class=CODE,delta=2
global __ptext1220
__ptext1220:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 137 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;      Temps:          0       0       3       0       0
;;      Totals:         0       0       3       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_findWall
;;		_rotateIR
;;		_lcd_write_data
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1220
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	137
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	138
	
l8258:	
;main.c: 138: PORTC |= 0b00000011;
	movlw	(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	iorwf	(7),f	;volatile
	line	139
	
l8260:	
;main.c: 139: SSPBUF = 0b00001111;
	movlw	(0Fh)
	movwf	(19)	;volatile
	line	140
	
l8262:	
;main.c: 140: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
movwf	((??_findWalls+0)^080h+0+2),f
movlw	19
movwf	((??_findWalls+0)^080h+0+1),f
	movlw	177
movwf	((??_findWalls+0)^080h+0),f
u3807:
	decfsz	((??_findWalls+0)^080h+0),f
	goto	u3807
	decfsz	((??_findWalls+0)^080h+0+1),f
	goto	u3807
	decfsz	((??_findWalls+0)^080h+0+2),f
	goto	u3807
	nop2
opt asmopt_on

	line	142
	
l8264:	
;main.c: 142: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u3671
	goto	u3670
	
u3671:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u3684
u3670:
	bcf	(_frontWall/8),(_frontWall)&7
u3684:
	line	143
	
l8266:	
;main.c: 143: rotateIR(24);
	movlw	(018h)
	fcall	_rotateIR
	line	144
	
l8268:	
;main.c: 144: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u3691
	goto	u3690
	
u3691:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u3704
u3690:
	bcf	(_rightWall/8),(_rightWall)&7
u3704:
	line	145
	
l8270:	
;main.c: 145: rotateIR(48);
	movlw	(030h)
	fcall	_rotateIR
	line	146
	
l8272:	
;main.c: 146: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u3711
	goto	u3710
	
u3711:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u3724
u3710:
	bcf	(_leftWall/8),(_leftWall)&7
u3724:
	line	147
	
l8274:	
;main.c: 147: rotateIR(24);
	movlw	(018h)
	fcall	_rotateIR
	line	149
	
l8276:	
;main.c: 149: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u3731
	goto	u3730
u3731:
	goto	l8280
u3730:
	line	150
	
l8278:	
;main.c: 150: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	goto	l8282
	line	151
	
l2919:	
	line	152
	
l8280:	
;main.c: 151: else
;main.c: 152: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l8282
	
l2920:	
	line	153
	
l8282:	
;main.c: 153: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u3741
	goto	u3740
u3741:
	goto	l8286
u3740:
	line	154
	
l8284:	
;main.c: 154: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	goto	l8288
	line	155
	
l2921:	
	line	156
	
l8286:	
;main.c: 155: else
;main.c: 156: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l8288
	
l2922:	
	line	157
	
l8288:	
;main.c: 157: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u3751
	goto	u3750
u3751:
	goto	l8292
u3750:
	line	158
	
l8290:	
;main.c: 158: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	goto	l2925
	line	159
	
l2923:	
	line	160
	
l8292:	
;main.c: 159: else
;main.c: 160: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l2925
	
l2924:	
	line	161
	
l2925:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_findWall
psect	text1221,local,class=CODE,delta=2
global __ptext1221
__ptext1221:

;; *************** function _findWall *****************
;; Defined at:
;;		line 128 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
psect	text1221
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	128
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	129
	
l8246:	
;main.c: 129: if(readIR() > 100)
	fcall	_readIR
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(1+(?_readIR)),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3665
	movlw	low(065h)
	subwf	(0+(?_readIR)),w
u3665:

	skipc
	goto	u3661
	goto	u3660
u3661:
	goto	l8254
u3660:
	line	130
	
l8248:	
;main.c: 130: return 0;
	clrc
	
	goto	l2915
	
l8250:	
	goto	l2915
	
l8252:	
	goto	l2915
	line	131
	
l2914:	
	line	132
	
l8254:	
;main.c: 131: else
;main.c: 132: return 1;
	setc
	
	goto	l2915
	
l8256:	
	goto	l2915
	
l2916:	
	line	133
	
l2915:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_updateLocation
psect	text1222,local,class=CODE,delta=2
global __ptext1222
__ptext1222:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 236 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;		_lcd_write_1_digit_bcd
;; This function is called by:
;;		_run
;; This function uses a non-reentrant model
;;
psect	text1222
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	236
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 2
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	237
	
l8216:	
;main.c: 237: currentOrientation += directionMoved;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_directionMoved),w	;volatile
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	239
	
l8218:	
;main.c: 239: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u3651
	goto	u3650
u3651:
	goto	l8222
u3650:
	line	240
	
l8220:	
;main.c: 240: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l8222
	
l2958:	
	line	242
	
l8222:	
;main.c: 242: lcd_set_cursor(0x06);
	movlw	(06h)
	fcall	_lcd_set_cursor
	line	243
;main.c: 243: switch(currentOrientation)
	goto	l8242
	line	245
;main.c: 244: {
;main.c: 245: case NORTH:
	
l2960:	
	line	246
	
l8224:	
;main.c: 246: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	247
	
l8226:	
;main.c: 247: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	248
;main.c: 248: break;
	goto	l8244
	line	249
;main.c: 249: case SOUTH:
	
l2962:	
	line	250
	
l8228:	
;main.c: 250: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	251
	
l8230:	
;main.c: 251: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	252
;main.c: 252: break;
	goto	l8244
	line	253
;main.c: 253: case EAST:
	
l2963:	
	line	254
	
l8232:	
;main.c: 254: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	255
	
l8234:	
;main.c: 255: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	256
;main.c: 256: break;
	goto	l8244
	line	257
;main.c: 257: case WEST:
	
l2964:	
	line	258
	
l8236:	
;main.c: 258: --xCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_xCoord),f	;volatile
	line	259
	
l8238:	
;main.c: 259: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	260
;main.c: 260: break;
	goto	l8244
	line	261
;main.c: 261: default:
	
l2965:	
	line	262
;main.c: 262: break;
	goto	l8244
	line	263
	
l8240:	
;main.c: 263: }
	goto	l8244
	line	243
	
l2959:	
	
l8242:	
	movf	(_currentOrientation),w	;volatile
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
	goto	l8236
	xorlw	1^0	; case 1
	skipnz
	goto	l8228
	xorlw	2^1	; case 2
	skipnz
	goto	l8232
	xorlw	3^2	; case 3
	skipnz
	goto	l8224
	goto	l8244
	opt asmopt_on

	line	263
	
l2961:	
	line	265
	
l8244:	
;main.c: 265: lcd_set_cursor(0x0C);
	movlw	(0Ch)
	fcall	_lcd_set_cursor
	line	266
;main.c: 266: lcd_write_1_digit_bcd(xCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	267
;main.c: 267: lcd_set_cursor(0x03);
	movlw	(03h)
	fcall	_lcd_set_cursor
	line	268
;main.c: 268: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	270
	
l2966:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_goRight
psect	text1223,local,class=CODE,delta=2
global __ptext1223
__ptext1223:

;; *************** function _goRight *****************
;; Defined at:
;;		line 214 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;		_turnRight90
;;		_driveForDistance
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1223
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	214
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 0
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	215
	
l8206:	
;main.c: 215: turnRight90();
	fcall	_turnRight90
	line	216
	
l8208:	
;main.c: 216: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	217
	
l8210:	
;main.c: 217: directionMoved = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_directionMoved)	;volatile
	line	218
	
l8212:	
;main.c: 218: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	219
	
l8214:	
;main.c: 219: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	220
	
l2949:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1224,local,class=CODE,delta=2
global __ptext1224
__ptext1224:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 190 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;		_turnLeft90
;;		_driveForDistance
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1224
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	190
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 0
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	191
	
l8196:	
;main.c: 191: turnLeft90();
	fcall	_turnLeft90
	line	192
	
l8198:	
;main.c: 192: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	193
	
l8200:	
;main.c: 193: directionMoved = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_directionMoved)	;volatile
	bsf	status,0
	rlf	(_directionMoved),f	;volatile
	line	194
	
l8202:	
;main.c: 194: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	195
	
l8204:	
;main.c: 195: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	196
	
l2937:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1225,local,class=CODE,delta=2
global __ptext1225
__ptext1225:

;; *************** function _goForward *****************
;; Defined at:
;;		line 181 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;		_driveForDistance
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1225
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	181
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 0
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	182
	
l8188:	
;main.c: 182: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	183
	
l8190:	
;main.c: 183: directionMoved = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_directionMoved)	;volatile
	line	184
	
l8192:	
;main.c: 184: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	185
	
l8194:	
;main.c: 185: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	186
	
l2934:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1226,local,class=CODE,delta=2
global __ptext1226
__ptext1226:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 171 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;;		_turnAround
;;		_driveForDistance
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_goToNextCell
;; This function uses a non-reentrant model
;;
psect	text1226
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	171
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 0
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	172
	
l8178:	
;main.c: 172: turnAround();
	fcall	_turnAround
	line	173
	
l8180:	
;main.c: 173: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	174
	
l8182:	
;main.c: 174: directionMoved = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_directionMoved)	;volatile
	line	175
	
l8184:	
;main.c: 175: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	176
	
l8186:	
;main.c: 176: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	177
	
l2931:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_readIR
psect	text1227,local,class=CODE,delta=2
global __ptext1227
__ptext1227:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
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
;;		_findWall
;; This function uses a non-reentrant model
;;
psect	text1227
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l8172:	
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
	
l8174:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1),w
	clrf	(?_readIR+1)
	addwf	(?_readIR+1)
	movf	(readIR@cm),w
	clrf	(?_readIR)
	addwf	(?_readIR)

	goto	l4413
	
l8176:	
	line	36
	
l4413:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_init
psect	text1228,local,class=CODE,delta=2
global __ptext1228
__ptext1228:

;; *************** function _init *****************
;; Defined at:
;;		line 90 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1228
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	90
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 3
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	91
	
l8142:	
;main.c: 91: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	92
	
l8144:	
;main.c: 92: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	94
	
l8146:	
;main.c: 94: init_adc();
	fcall	_init_adc
	line	95
	
l8148:	
;main.c: 95: lcd_init();
	fcall	_lcd_init
	line	97
	
l8150:	
;main.c: 97: TRISB = 0b00011110;
	movlw	(01Eh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	100
	
l8152:	
;main.c: 100: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	102
	
l8154:	
;main.c: 102: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	103
	
l8156:	
;main.c: 103: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	104
	
l8158:	
;main.c: 104: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	105
	
l8160:	
;main.c: 105: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	106
	
l8162:	
;main.c: 106: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	109
	
l8164:	
;main.c: 109: PEIE = 1;
	bsf	(94/8),(94)&7
	line	110
	
l8166:	
;main.c: 110: GIE = 1;
	bsf	(95/8),(95)&7
	line	112
	
l8168:	
;main.c: 112: ser_init();
	fcall	_ser_init
	line	113
	
l8170:	
;main.c: 113: initIRobot();
	fcall	_initIRobot
	line	114
	
l2908:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_lcd_init
psect	text1229,local,class=CODE,delta=2
global __ptext1229
__ptext1229:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
psect	text1229
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l8122:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l8124:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l8126:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l8128:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l8130:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l8132:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l8134:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l8136:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l8138:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l8140:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l2131:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1230,local,class=CODE,delta=2
global __ptext1230
__ptext1230:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text1230
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	44
	global	__size_of_lcd_write_1_digit_bcd
	__size_of_lcd_write_1_digit_bcd	equ	__end_of_lcd_write_1_digit_bcd-_lcd_write_1_digit_bcd
	
_lcd_write_1_digit_bcd:	
	opt	stack 2
; Regs used in _lcd_write_1_digit_bcd: [wreg+status,2+status,0+pclath+cstack]
;lcd_write_1_digit_bcd@data stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_1_digit_bcd@data)
	line	45
	
l8120:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l2119:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_write_string
psect	text1231,local,class=CODE,delta=2
global __ptext1231
__ptext1231:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
;;		_run
;; This function uses a non-reentrant model
;;
psect	text1231
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	38
	global	__size_of_lcd_write_string
	__size_of_lcd_write_string	equ	__end_of_lcd_write_string-_lcd_write_string
	
_lcd_write_string:	
	opt	stack 3
; Regs used in _lcd_write_string: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;lcd_write_string@s stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_string@s)
	line	40
	
l8112:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l8118
	
l2114:	
	
l8114:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l8116:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l8118
	
l2113:	
	
l8118:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u3641
	goto	u3640
u3641:
	goto	l8114
u3640:
	goto	l2116
	
l2115:	
	line	41
	
l2116:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_lcd_set_cursor
psect	text1232,local,class=CODE,delta=2
global __ptext1232
__ptext1232:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
;;		_run
;;		_updateLocation
;; This function uses a non-reentrant model
;;
psect	text1232
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	32
	global	__size_of_lcd_set_cursor
	__size_of_lcd_set_cursor	equ	__end_of_lcd_set_cursor-_lcd_set_cursor
	
_lcd_set_cursor:	
	opt	stack 2
; Regs used in _lcd_set_cursor: [wreg+status,2+status,0+pclath+cstack]
;lcd_set_cursor@address stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_set_cursor@address)
	line	33
	
l8108:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l8110:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l2110:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_turnRight90
psect	text1233,local,class=CODE,delta=2
global __ptext1233
__ptext1233:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 54 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1233
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	54
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 0
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	55
	
l8106:	
;drive.c: 55: drive(0, 25, 255, 255);
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
	line	56
;drive.c: 56: waitFor(157,255,169);
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
	line	57
;drive.c: 57: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	58
	
l1414:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_turnLeft90
psect	text1234,local,class=CODE,delta=2
global __ptext1234
__ptext1234:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 47 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_goLeft
;; This function uses a non-reentrant model
;;
psect	text1234
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	47
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 0
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	48
	
l8104:	
;drive.c: 48: drive(0, 25, 0, 1);
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
	line	49
;drive.c: 49: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	50
;drive.c: 50: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	51
	
l1411:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnAround
psect	text1235,local,class=CODE,delta=2
global __ptext1235
__ptext1235:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 40 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
;;		_drive
;;		_waitFor
;; This function is called by:
;;		_goBackward
;; This function uses a non-reentrant model
;;
psect	text1235
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	40
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 0
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	41
	
l8102:	
;drive.c: 41: drive(0, 25, 0, 1);
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
	line	42
;drive.c: 42: waitFor(157,0,170);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0AAh)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	43
;drive.c: 43: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	44
	
l1408:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_driveForDistance
psect	text1236,local,class=CODE,delta=2
global __ptext1236
__ptext1236:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 19 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
psect	text1236
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	19
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 0
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	22
	
l8088:	
;drive.c: 21: volatile char high, low;
;drive.c: 22: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	23
;drive.c: 23: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	25
	
l8090:	
;drive.c: 25: drive(0, 125, 128, 0);
	movlw	(07Dh)
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
	line	27
;drive.c: 27: while(distance >= moveDistance)
	goto	l8098
	
l1403:	
	line	29
	
l8092:	
;drive.c: 28: {
;drive.c: 29: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	30
;drive.c: 30: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	31
;drive.c: 31: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	32
;drive.c: 32: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	33
	
l8094:	
;drive.c: 33: deltaDistance = high*256 + low;
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
	line	34
	
l8096:	
;drive.c: 34: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	goto	l8098
	line	35
	
l1402:	
	line	27
	
l8098:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u3635
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u3635:

	skipnc
	goto	u3631
	goto	u3630
u3631:
	goto	l8092
u3630:
	goto	l8100
	
l1404:	
	line	36
	
l8100:	
;drive.c: 35: }
;drive.c: 36: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	37
	
l1405:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_adc_read_channel
psect	text1237,local,class=CODE,delta=2
global __ptext1237
__ptext1237:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
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
psect	text1237
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
	line	7
	global	__size_of_adc_read_channel
	__size_of_adc_read_channel	equ	__end_of_adc_read_channel-_adc_read_channel
	
_adc_read_channel:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _adc_read_channel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
;adc_read_channel@channel stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(adc_read_channel@channel)
	line	8
	
l7560:	
;adc.c: 8: switch(channel)
	goto	l7568
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
	goto	l7570
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
	goto	l7570
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
	goto	l7570
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
	goto	l7570
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
	goto	l7570
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l7562:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l7564:	
	goto	l697
	line	39
	
l7566:	
;adc.c: 39: }
	goto	l7570
	line	8
	
l689:	
	
l7568:	
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
	goto	l7562
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l7570:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u3817:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u3817
opt asmopt_on

	line	43
	
l7572:	
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
	
l7574:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_convert
psect	text1238,local,class=CODE,delta=2
global __ptext1238
__ptext1238:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
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
psect	text1238
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l8028:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3565
	movlw	low(052h)
	subwf	(convert@adc_value),w
u3565:

	skipnc
	goto	u3561
	goto	u3560
u3561:
	goto	l8036
u3560:
	line	13
	
l8030:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l4397
	
l8032:	
	goto	l4397
	
l8034:	
	goto	l4397
	line	14
	
l4396:	
	
l8036:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3575
	movlw	low(085h)
	subwf	(convert@adc_value),w
u3575:

	skipnc
	goto	u3571
	goto	u3570
u3571:
	goto	l8044
u3570:
	line	15
	
l8038:	
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
	goto	l4397
	
l8040:	
	goto	l4397
	
l8042:	
	goto	l4397
	line	16
	
l4399:	
	
l8044:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3585
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u3585:

	skipnc
	goto	u3581
	goto	u3580
u3581:
	goto	l8052
u3580:
	line	17
	
l8046:	
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
	goto	l4397
	
l8048:	
	goto	l4397
	
l8050:	
	goto	l4397
	line	18
	
l4401:	
	
l8052:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3595
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u3595:

	skipnc
	goto	u3591
	goto	u3590
u3591:
	goto	l8060
u3590:
	line	19
	
l8054:	
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
	goto	l4397
	
l8056:	
	goto	l4397
	
l8058:	
	goto	l4397
	line	20
	
l4403:	
	
l8060:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u3605
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u3605:

	skipnc
	goto	u3601
	goto	u3600
u3601:
	goto	l8068
u3600:
	line	21
	
l8062:	
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
	goto	l4397
	
l8064:	
	goto	l4397
	
l8066:	
	goto	l4397
	line	22
	
l4405:	
	
l8068:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u3615
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u3615:

	skipnc
	goto	u3611
	goto	u3610
u3611:
	goto	l8076
u3610:
	line	23
	
l8070:	
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
	goto	l4397
	
l8072:	
	goto	l4397
	
l8074:	
	goto	l4397
	line	24
	
l4407:	
	
l8076:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u3625
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u3625:

	skipnc
	goto	u3621
	goto	u3620
u3621:
	goto	l8084
u3620:
	line	25
	
l8078:	
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
	goto	l4397
	
l8080:	
	goto	l4397
	
l8082:	
	goto	l4397
	line	26
	
l4409:	
	
l8084:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l4397
	
l8086:	
	goto	l4397
	
l4410:	
	goto	l4397
	
l4408:	
	goto	l4397
	
l4406:	
	goto	l4397
	
l4404:	
	goto	l4397
	
l4402:	
	goto	l4397
	
l4400:	
	goto	l4397
	
l4398:	
	line	27
	
l4397:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_rotateIR
psect	text1239,local,class=CODE,delta=2
global __ptext1239
__ptext1239:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
;; Parameters:    Size  Location     Type
;;  steps           1    wreg     unsigned char 
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
;;      Params:         0       0       0       0       0
;;      Locals:         0       2       0       0       0
;;      Temps:          0       3       0       0       0
;;      Totals:         0       5       0       0       0
;;Total ram usage:        5 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    2
;; This function calls:
;;		Nothing
;; This function is called by:
;;		_findWalls
;; This function uses a non-reentrant model
;;
psect	text1239
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ir.c"
	line	39
	global	__size_of_rotateIR
	__size_of_rotateIR	equ	__end_of_rotateIR-_rotateIR
	
_rotateIR:	
	opt	stack 2
; Regs used in _rotateIR: [wreg+status,2+status,0]
;rotateIR@steps stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(rotateIR@steps)
	line	40
	
l8020:	
;ir.c: 40: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l4416
	line	41
	
l4417:	
	line	42
;ir.c: 41: {
;ir.c: 42: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	43
	
l8022:	
;ir.c: 43: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	44
	
l8024:	
;ir.c: 44: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_rotateIR+0)+0+2),f
movlw	138
movwf	((??_rotateIR+0)+0+1),f
	movlw	86
movwf	((??_rotateIR+0)+0),f
u3827:
	decfsz	((??_rotateIR+0)+0),f
	goto	u3827
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u3827
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u3827
	nop2
opt asmopt_on

	line	40
	
l8026:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l4416:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u3551
	goto	u3550
u3551:
	goto	l4417
u3550:
	goto	l4419
	
l4418:	
	line	46
	
l4419:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,4216
	global	_initIRobot
psect	text1240,local,class=CODE,delta=2
global __ptext1240
__ptext1240:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 117 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
psect	text1240
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	117
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	118
	
l8014:	
;main.c: 118: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u3837:
	decfsz	((??_initIRobot+0)+0),f
	goto	u3837
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u3837
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u3837
	nop2
opt asmopt_on

	line	119
	
l8016:	
;main.c: 119: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	120
	
l8018:	
;main.c: 120: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	121
	
l2911:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_lcd_write_data
psect	text1241,local,class=CODE,delta=2
global __ptext1241
__ptext1241:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
;;		_findWalls
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;;		_updateLocation
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1241
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	20
	global	__size_of_lcd_write_data
	__size_of_lcd_write_data	equ	__end_of_lcd_write_data-_lcd_write_data
	
_lcd_write_data:	
	opt	stack 2
; Regs used in _lcd_write_data: [wreg]
;lcd_write_data@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_data@databyte)
	line	21
	
l8006:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l8008:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l8010:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l8012:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u3847:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u3847
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u3847
opt asmopt_on

	line	28
	
l2107:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_lcd_write_control
psect	text1242,local,class=CODE,delta=2
global __ptext1242
__ptext1242:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
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
psect	text1242
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\lcd.c"
	line	8
	global	__size_of_lcd_write_control
	__size_of_lcd_write_control	equ	__end_of_lcd_write_control-_lcd_write_control
	
_lcd_write_control:	
	opt	stack 2
; Regs used in _lcd_write_control: [wreg]
;lcd_write_control@databyte stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(lcd_write_control@databyte)
	line	9
	
l7998:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l8000:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l8002:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l8004:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u3857:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u3857
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u3857
	nop2
opt asmopt_on

	line	16
	
l2104:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_waitFor
psect	text1243,local,class=CODE,delta=2
global __ptext1243
__ptext1243:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 61 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1243
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	61
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 0
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	62
	
l7990:	
;drive.c: 62: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u3867:
	decfsz	((??_waitFor+0)+0),f
	goto	u3867
	decfsz	((??_waitFor+0)+0+1),f
	goto	u3867
	decfsz	((??_waitFor+0)+0+2),f
	goto	u3867
	nop2
opt asmopt_on

	line	63
	
l7992:	
;drive.c: 63: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	64
	
l7994:	
;drive.c: 64: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	65
	
l7996:	
;drive.c: 65: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	66
	
l1417:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_ser_getch
psect	text1244,local,class=CODE,delta=2
global __ptext1244
__ptext1244:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
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
psect	text1244
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 0
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l7974:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l7976
	
l3677:	
	line	62
;ser.c: 62: continue;
	goto	l7976
	
l3676:	
	line	61
	
l7976:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u3541
	goto	u3540
u3541:
	goto	l7976
u3540:
	
l3678:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l7978:	
;ser.c: 65: c=rxfifo[rxoptr];
	movf	(_rxoptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank0
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	movwf	(ser_getch@c)
	line	66
	
l7980:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l7982:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l7984:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l7986:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l3679
	
l7988:	
	line	70
	
l3679:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_drive
psect	text1245,local,class=CODE,delta=2
global __ptext1245
__ptext1245:

;; *************** function _drive *****************
;; Defined at:
;;		line 9 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
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
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1245
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\drive.c"
	line	9
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 0
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	10
	
l7962:	
;drive.c: 10: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u3877:
	decfsz	((??_drive+0)+0),f
	goto	u3877
	decfsz	((??_drive+0)+0+1),f
	goto	u3877
	decfsz	((??_drive+0)+0+2),f
	goto	u3877
	nop2
opt asmopt_on

	line	11
	
l7964:	
;drive.c: 11: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	12
	
l7966:	
;drive.c: 12: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	13
	
l7968:	
;drive.c: 13: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	14
	
l7970:	
;drive.c: 14: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	15
	
l7972:	
;drive.c: 15: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	16
	
l1399:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_init_adc
psect	text1246,local,class=CODE,delta=2
global __ptext1246
__ptext1246:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
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
psect	text1246
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l7952:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l7954:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l7956:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l7958:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l7960:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u3887:
decfsz	(??_init_adc+0)+0,f
	goto	u3887
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
psect	text1247,local,class=CODE,delta=2
global __ptext1247
__ptext1247:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
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
psect	text1247
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l7414:	
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
	
l7416:	
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
	goto	u3121
	goto	u3120
u3121:
	goto	l703
u3120:
	goto	l7418
	
l705:	
	line	75
	
l7418:	
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
u3135:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u3135
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l7420:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l7422:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1248,local,class=CODE,delta=2
global __ptext1248
__ptext1248:

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
psect	text1248
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l7374:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u3021
	goto	u3020
u3021:
	goto	l7378
u3020:
	line	11
	
l7376:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l7378
	line	13
	
l5242:	
	line	14
	
l7378:	
	btfss	(___awdiv@dividend+1),7
	goto	u3031
	goto	u3030
u3031:
	goto	l7384
u3030:
	line	15
	
l7380:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l7382:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l7384
	line	17
	
l5243:	
	line	18
	
l7384:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l7386:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u3041
	goto	u3040
u3041:
	goto	l7406
u3040:
	line	20
	
l7388:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l7394
	
l5246:	
	line	22
	
l7390:	
	movlw	01h
	
u3055:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u3055
	line	23
	
l7392:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l7394
	line	24
	
l5245:	
	line	21
	
l7394:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u3061
	goto	u3060
u3061:
	goto	l7390
u3060:
	goto	l7396
	
l5247:	
	goto	l7396
	line	25
	
l5248:	
	line	26
	
l7396:	
	movlw	01h
	
u3075:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u3075
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u3085
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u3085:
	skipc
	goto	u3081
	goto	u3080
u3081:
	goto	l7402
u3080:
	line	28
	
l7398:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l7400:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l7402
	line	30
	
l5249:	
	line	31
	
l7402:	
	movlw	01h
	
u3095:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u3095
	line	32
	
l7404:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u3101
	goto	u3100
u3101:
	goto	l7396
u3100:
	goto	l7406
	
l5250:	
	goto	l7406
	line	33
	
l5244:	
	line	34
	
l7406:	
	movf	(___awdiv@sign),w
	skipz
	goto	u3110
	goto	l7410
u3110:
	line	35
	
l7408:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l7410
	
l5251:	
	line	36
	
l7410:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l5252
	
l7412:	
	line	37
	
l5252:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1249,local,class=CODE,delta=2
global __ptext1249
__ptext1249:

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
psect	text1249
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l7940:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l7942
	line	6
	
l5102:	
	line	7
	
l7942:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u3501
	goto	u3500
u3501:
	goto	l5103
u3500:
	line	8
	
l7944:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l5103:	
	line	9
	movlw	01h
	
u3515:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u3515
	line	10
	
l7946:	
	movlw	01h
	
u3525:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u3525
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u3531
	goto	u3530
u3531:
	goto	l7942
u3530:
	goto	l7948
	
l5104:	
	line	12
	
l7948:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l5105
	
l7950:	
	line	13
	
l5105:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_ser_isrx
psect	text1250,local,class=CODE,delta=2
global __ptext1250
__ptext1250:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
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
psect	text1250
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 0
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l7892:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u3431
	goto	u3430
u3431:
	goto	l7900
u3430:
	line	50
	
l7894:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l7896:	
;ser.c: 52: return 0;
	clrc
	
	goto	l3673
	
l7898:	
	goto	l3673
	line	53
	
l3672:	
	line	54
	
l7900:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u3441
	goto	u3440
u3441:
	goto	l7904
u3440:
	
l7902:	
	clrc
	
	goto	l3673
	
l7810:	
	
l7904:	
	setc
	
	goto	l3673
	
l7812:	
	goto	l3673
	
l7906:	
	line	55
	
l3673:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_ser_init
psect	text1251,local,class=CODE,delta=2
global __ptext1251
__ptext1251:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 116 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
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
psect	text1251
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
	line	116
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	117
	
l7866:	
;ser.c: 117: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	118
	
l7868:	
;ser.c: 118: TRISC &= 0b10111111;
	movlw	(0BFh)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_ser_init+0)+0
	movf	(??_ser_init+0)+0,w
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	andwf	(135)^080h,f	;volatile
	line	119
	
l7870:	
;ser.c: 119: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	121
	
l7872:	
;ser.c: 121: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	124
	
l7874:	
;ser.c: 124: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	125
	
l7876:	
;ser.c: 125: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	127
	
l7878:	
;ser.c: 127: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	128
	
l7880:	
;ser.c: 128: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	129
	
l7882:	
;ser.c: 129: CREN=1;
	bsf	(196/8),(196)&7
	line	130
	
l7884:	
;ser.c: 130: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	131
	
l7886:	
;ser.c: 131: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	132
	
l7888:	
;ser.c: 132: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	135
	
l7890:	
;ser.c: 135: rxiptr=rxoptr=txiptr=txoptr=0;
	movlw	(0)
	movwf	(_txoptr)	;volatile
	movwf	(_txiptr)	;volatile
	movwf	(_rxoptr)	;volatile
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_ser_init+0)+0
	movf	(??_ser_init+0)+0,w
	movwf	(_rxiptr)	;volatile
	line	136
	
l3707:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_ser_putch
psect	text1252,local,class=CODE,delta=2
global __ptext1252
__ptext1252:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 73 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
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
;;		_ser_puts
;;		_ser_puts2
;;		_ser_puthex
;; This function uses a non-reentrant model
;;
psect	text1252
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\ser.c"
	line	73
	global	__size_of_ser_putch
	__size_of_ser_putch	equ	__end_of_ser_putch-_ser_putch
	
_ser_putch:	
	opt	stack 1
; Regs used in _ser_putch: [wreg-fsr0h+status,2+status,0]
;ser_putch@c stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(ser_putch@c)
	line	74
	
l7838:	
;ser.c: 74: while (((txiptr+1) & (16-1))==txoptr)
	goto	l7840
	
l3683:	
	line	75
;ser.c: 75: continue;
	goto	l7840
	
l3682:	
	line	74
	
l7840:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u3401
	goto	u3400
u3401:
	goto	l7840
u3400:
	
l3684:	
	line	76
;ser.c: 76: GIE=0;
	bcf	(95/8),(95)&7
	line	77
	
l7842:	
;ser.c: 77: txfifo[txiptr] = c;
	movf	(ser_putch@c),w
	movwf	(??_ser_putch+0)+0
	movf	(_txiptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	movf	(??_ser_putch+0)+0,w
	bcf	status, 7	;select IRP bank1
	movwf	indf
	line	78
	
l7844:	
;ser.c: 78: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	79
	
l7846:	
;ser.c: 79: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	80
	
l7848:	
;ser.c: 80: GIE=1;
	bsf	(95/8),(95)&7
	line	81
	
l3685:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_isr1
psect	text1253,local,class=CODE,delta=2
global __ptext1253
__ptext1253:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 48 in file "C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
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
psect	text1253
	file	"C:\Documents and Settings\User\My Documents\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.1\main.c"
	line	48
	global	__size_of_isr1
	__size_of_isr1	equ	__end_of_isr1-_isr1
	
_isr1:	
;; hardware stack exceeded
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
psect	text1253
	line	50
	
i1l7208:	
;main.c: 50: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u274_21
	goto	u274_20
u274_21:
	goto	i1l2905
u274_20:
	line	52
	
i1l7210:	
;main.c: 51: {
;main.c: 52: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	53
	
i1l7212:	
;main.c: 53: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	55
;main.c: 55: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	57
	
i1l7214:	
;main.c: 57: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	59
	
i1l7216:	
;main.c: 59: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
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
	goto	u275_21
	goto	u275_20
u275_21:
	goto	i1l7220
u275_20:
	
i1l7218:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l7220
	
i1l2895:	
	line	60
	
i1l7220:	
;main.c: 60: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
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
	goto	u276_21
	goto	u276_20
u276_21:
	goto	i1l7224
u276_20:
	
i1l7222:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l7224
	
i1l2896:	
	line	61
	
i1l7224:	
;main.c: 61: if(RTC_Counter % 500 == 0)
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
	movf	((1+(?___lwmod))),w
	iorwf	((0+(?___lwmod))),w
	skipz
	goto	u277_21
	goto	u277_20
u277_21:
	goto	i1l7230
u277_20:
	line	63
	
i1l7226:	
;main.c: 62: {
;main.c: 63: RTC_FLAG_500MS = 1;
	bsf	(_RTC_FLAG_500MS/8),(_RTC_FLAG_500MS)&7
	line	64
	
i1l7228:	
;main.c: 64: RTC_Counter = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_RTC_Counter)	;volatile
	clrf	(_RTC_Counter+1)	;volatile
	goto	i1l7230
	line	66
	
i1l2897:	
	line	68
	
i1l7230:	
;main.c: 66: }
;main.c: 68: if(RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(48/8),(48)&7
	goto	u278_21
	goto	u278_20
u278_21:
	goto	i1l2898
u278_20:
	line	70
	
i1l7232:	
;main.c: 69: {
;main.c: 70: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	71
	
i1l7234:	
;main.c: 71: if(start.debounceCount >= 10 & start.released)
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
	goto	u279_21
	goto	u279_20
u279_21:
	goto	i1l7242
u279_20:
	line	73
	
i1l7236:	
;main.c: 72: {
;main.c: 73: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	74
	
i1l7238:	
;main.c: 74: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l7242
	line	75
	
i1l2899:	
	line	76
;main.c: 75: }
;main.c: 76: }
	goto	i1l7242
	line	77
	
i1l2898:	
	line	79
;main.c: 77: else
;main.c: 78: {
;main.c: 79: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	80
	
i1l7240:	
;main.c: 80: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l7242
	line	81
	
i1l2900:	
	line	83
	
i1l7242:	
;main.c: 81: }
;main.c: 83: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u280_21
	goto	u280_20
u280_21:
	goto	i1l7252
u280_20:
	
i1l7244:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	
i1l7246:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l7248:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u281_21
	goto	u281_20
u281_21:
	goto	i1l7252
u281_20:
	
i1l7250:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l7252
	
i1l2902:	
	goto	i1l7252
	
i1l2901:	
	
i1l7252:	
	btfss	(100/8),(100)&7
	goto	u282_21
	goto	u282_20
u282_21:
	goto	i1l2905
u282_20:
	
i1l7254:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u283_21
	goto	u283_20
u283_21:
	goto	i1l2905
u283_20:
	
i1l7256:	
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(25)	;volatile
	
i1l7258:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l7260:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l7262:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u284_21
	goto	u284_20
u284_21:
	goto	i1l2905
u284_20:
	
i1l7264:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l2905
	
i1l2904:	
	goto	i1l2905
	
i1l2903:	
	goto	i1l2905
	line	84
	
i1l2894:	
	line	85
	
i1l2905:	
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
psect	text1254,local,class=CODE,delta=2
global __ptext1254
__ptext1254:

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
psect	text1254
	file	"C:\Program Files\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
;; hardware stack exceeded
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l7352:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u296_21
	goto	u296_20
u296_21:
	goto	i1l7370
u296_20:
	line	9
	
i1l7354:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l7360
	
i1l5120:	
	line	11
	
i1l7356:	
	movlw	01h
	
u297_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u297_25
	line	12
	
i1l7358:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l7360
	line	13
	
i1l5119:	
	line	10
	
i1l7360:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u298_21
	goto	u298_20
u298_21:
	goto	i1l7356
u298_20:
	goto	i1l7362
	
i1l5121:	
	goto	i1l7362
	line	14
	
i1l5122:	
	line	15
	
i1l7362:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u299_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u299_25:
	skipc
	goto	u299_21
	goto	u299_20
u299_21:
	goto	i1l7366
u299_20:
	line	16
	
i1l7364:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l7366
	
i1l5123:	
	line	17
	
i1l7366:	
	movlw	01h
	
u300_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u300_25
	line	18
	
i1l7368:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u301_21
	goto	u301_20
u301_21:
	goto	i1l7362
u301_20:
	goto	i1l7370
	
i1l5124:	
	goto	i1l7370
	line	19
	
i1l5118:	
	line	20
	
i1l7370:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l5125
	
i1l7372:	
	line	21
	
i1l5125:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text1255,local,class=CODE,delta=2
global __ptext1255
__ptext1255:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

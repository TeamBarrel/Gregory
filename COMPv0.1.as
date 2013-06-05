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
# 21 "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	psect config,class=CONFIG,delta=2 ;#
# 21 "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	dw 0xFFFE & 0xFFFB & 0xFFFF & 0xFFBF & 0xFFF7 & 0xFFFF & 0xFF7F & 0xFFFF ;#
	FNCALL	_main,_init
	FNCALL	_main,_drive
	FNCALL	_main,_lcd_set_cursor
	FNCALL	_main,_lcd_write_string
	FNCALL	_main,_play_iCreate_song
	FNCALL	_main,_findWalls
	FNCALL	_main,_goToNextCell
	FNCALL	_main,_updateLocation
	FNCALL	_findWalls,_rotateIR
	FNCALL	_findWalls,_findWall
	FNCALL	_findWalls,_lcd_set_cursor
	FNCALL	_findWalls,_lcd_write_data
	FNCALL	_goToNextCell,_goRight
	FNCALL	_goToNextCell,_goForward
	FNCALL	_goToNextCell,_goLeft
	FNCALL	_goToNextCell,_goBackward
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
	FNCALL	_init,_initSongs
	FNCALL	_initSongs,_ser_putArr
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
	global	_walls
	global	_yCoord
	global	_xCoord
	global	_lookingForU2
	global	_finalCountdown
	global	_superMarioBros
	global	_champions
psect	idataBANK0,class=CODE,space=0,delta=2
global __pidataBANK0
__pidataBANK0:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	35

;initializer for _walls
	retlw	01h
	retlw	01h
	retlw	01h
	retlw	0
	line	42

;initializer for _yCoord
	retlw	04h
psect	idataCOMMON,class=CODE,space=0,delta=2
global __pidataCOMMON
__pidataCOMMON:
	line	41

;initializer for _xCoord
	retlw	01h
psect	idataBANK3,class=CODE,space=0,delta=2
global __pidataBANK3
__pidataBANK3:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
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
	global	_directionMoved
	global	_ser_tmp
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
	global	_moving
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
	retlw	87	;'W'
	retlw	97	;'a'
	retlw	108	;'l'
	retlw	108	;'l'
	retlw	115	;'s'
	retlw	64	;'@'
	retlw	32	;' '
	retlw	45	;'-'
	retlw	45	;'-'
	retlw	45	;'-'
	retlw	32	;' '
	retlw	40	;'('
	retlw	49	;'1'
	retlw	44	;','
	retlw	48	;'0'
	retlw	41	;')'
	retlw	0
psect	strings
	
STR_4:	
	retlw	99	;'c'
	retlw	117	;'u'
	retlw	79	;'O'
	retlw	114	;'r'
	retlw	58	;':'
	retlw	32	;' '
	retlw	45	;'-'
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
	
STR_1:	
	retlw	69	;'E'
	retlw	69	;'E'
	retlw	80	;'P'
	retlw	82	;'R'
	retlw	79	;'O'
	retlw	77	;'M'
	retlw	32	;' '
	retlw	87	;'W'
	retlw	111	;'o'
	retlw	114	;'r'
	retlw	107	;'k'
	retlw	101	;'e'
	retlw	100	;'d'
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

_moving:
       ds      1

_rightWall:
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

psect	dataCOMMON,class=COMMON,space=1
global __pdataCOMMON
__pdataCOMMON:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	41
_xCoord:
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

_directionMoved:
       ds      1

_ser_tmp:
       ds      1

psect	dataBANK0,class=BANK0,space=1
global __pdataBANK0
__pdataBANK0:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	35
_walls:
       ds      4

psect	dataBANK0
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	42
_yCoord:
       ds      1

psect	bssBANK1,class=BANK1,space=1
global __pbssBANK1
__pbssBANK1:
_txfifo:
       ds      16

psect	dataBANK1,class=BANK1,space=1
global __pdataBANK1
__pdataBANK1:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
	line	10
_superMarioBros:
       ds      25

psect	dataBANK1
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
	line	13
_champions:
       ds      21

psect	dataBANK3,class=BANK3,space=1
global __pdataBANK3
__pdataBANK3:
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
	line	11
_lookingForU2:
       ds      29

psect	dataBANK3
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
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
	movlw	low((__pbssBANK0)+018h)
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
; Initialize objects allocated to BANK0
psect cinit,class=CODE,delta=2
global init_ram, __pidataBANK0
	movlw low(__pdataBANK0+5)
	movwf btemp-1,f
	movlw high(__pidataBANK0)
	movwf btemp,f
	movlw low(__pidataBANK0)
	movwf btemp+1,f
	movlw low(__pdataBANK0)
	movwf fsr,f
	fcall init_ram
; Initialize objects allocated to COMMON
	global __pidataCOMMON
psect cinit,class=CODE,delta=2
	fcall	__pidataCOMMON+0		;fetch initializer
	movwf	__pdataCOMMON+0&07fh		
psect cinit,class=CODE,delta=2
global end_of_initialization

;End of C runtime variable initialization code

end_of_initialization:
clrf status
ljmp _main	;jump to C main() function
psect	cstackBANK1,class=BANK1,space=1
global __pcstackBANK1
__pcstackBANK1:
	global	?_readIR
?_readIR:	; 2 bytes @ 0x0
	ds	2
	global	readIR@cm
readIR@cm:	; 2 bytes @ 0x2
	ds	2
	global	??_findWalls
??_findWalls:	; 0 bytes @ 0x4
	ds	1
	global	findWalls@wallAtOrientation
findWalls@wallAtOrientation:	; 2 bytes @ 0x5
	ds	2
psect	cstackCOMMON,class=COMMON,space=1
global __pcstackCOMMON
__pcstackCOMMON:
	global	?_ser_putch
?_ser_putch:	; 0 bytes @ 0x0
	global	?_lcd_write_data
?_lcd_write_data:	; 0 bytes @ 0x0
	global	?_ser_init
?_ser_init:	; 0 bytes @ 0x0
	global	?_initIRobot
?_initIRobot:	; 0 bytes @ 0x0
	global	?_initSongs
?_initSongs:	; 0 bytes @ 0x0
	global	?_goRight
?_goRight:	; 0 bytes @ 0x0
	global	?_play_iCreate_song
?_play_iCreate_song:	; 0 bytes @ 0x0
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
	global	?_updateLocation
?_updateLocation:	; 0 bytes @ 0x0
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
	global	??_lcd_write_data
??_lcd_write_data:	; 0 bytes @ 0xA
	global	??_ser_init
??_ser_init:	; 0 bytes @ 0xA
	global	?_rotateIR
?_rotateIR:	; 0 bytes @ 0xA
	global	??_init_adc
??_init_adc:	; 0 bytes @ 0xA
	global	??_lcd_write_control
??_lcd_write_control:	; 0 bytes @ 0xA
	global	??_ser_isrx
??_ser_isrx:	; 0 bytes @ 0xA
	global	?___wmul
?___wmul:	; 2 bytes @ 0xA
	global	rotateIR@direction
rotateIR@direction:	; 1 bytes @ 0xA
	global	___wmul@multiplier
___wmul@multiplier:	; 2 bytes @ 0xA
	ds	1
	global	??_rotateIR
??_rotateIR:	; 0 bytes @ 0xB
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
	global	??_goRight
??_goRight:	; 0 bytes @ 0x1D
	global	??_goBackward
??_goBackward:	; 0 bytes @ 0x1D
	global	??_goForward
??_goForward:	; 0 bytes @ 0x1D
	global	??_goLeft
??_goLeft:	; 0 bytes @ 0x1D
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
	global	?_adc_read_channel
?_adc_read_channel:	; 2 bytes @ 0x25
	ds	2
	global	??_adc_read_channel
??_adc_read_channel:	; 0 bytes @ 0x27
	ds	1
	global	adc_read_channel@channel
adc_read_channel@channel:	; 1 bytes @ 0x28
	ds	1
	global	??_readIR
??_readIR:	; 0 bytes @ 0x29
	global	??_findWall
??_findWall:	; 0 bytes @ 0x29
	global	??_main
??_main:	; 0 bytes @ 0x29
;;Data sizes: Strings 62, constant 0, data 108, bss 44, persistent 0 stack 0
;;Auto spaces:   Size  Autos    Used
;; COMMON          14      6      12
;; BANK0           80     41      70
;; BANK1           80      7      69
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
;;		 -> STR_4(CODE[17]), STR_3(CODE[17]), 
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
;;   _goRight->_driveForDistance
;;   _goLeft->_driveForDistance
;;   _goForward->_driveForDistance
;;   _goBackward->_driveForDistance
;;   _readIR->_adc_read_channel
;;   _initSongs->_ser_putArr
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
;;   _findWall->_readIR
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
;; (0) _main                                                 0     0      0    5414
;;                               _init
;;                              _drive
;;                     _lcd_set_cursor
;;                   _lcd_write_string
;;                  _play_iCreate_song
;;                          _findWalls
;;                       _goToNextCell
;;                     _updateLocation
;; ---------------------------------------------------------------------------------
;; (1) _findWalls                                            3     3      0    1862
;;                                              4 BANK1      3     3      0
;;                           _rotateIR
;;                           _findWall
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (1) _goToNextCell                                         0     0      0    2793
;;                            _goRight
;;                          _goForward
;;                             _goLeft
;;                         _goBackward
;; ---------------------------------------------------------------------------------
;; (2) _findWall                                             0     0      0    1528
;;                             _readIR
;; ---------------------------------------------------------------------------------
;; (1) _updateLocation                                       1     1      0     158
;;                                             14 BANK0      1     1      0
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;;              _lcd_write_1_digit_bcd
;; ---------------------------------------------------------------------------------
;; (2) _goRight                                              1     1      0     768
;;                                             29 BANK0      1     1      0
;;                        _turnRight90
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _goLeft                                               0     0      0     768
;;                         _turnLeft90
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _goForward                                            0     0      0     489
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (2) _goBackward                                           1     1      0     768
;;                                             29 BANK0      1     1      0
;;                         _turnAround
;;                   _driveForDistance
;;                     _lcd_set_cursor
;;                     _lcd_write_data
;; ---------------------------------------------------------------------------------
;; (3) _readIR                                               4     2      2    1528
;;                                              0 BANK1      4     2      2
;;                   _adc_read_channel
;;                            _convert
;; ---------------------------------------------------------------------------------
;; (1) _init                                                 0     0      0     223
;;                           _init_adc
;;                           _lcd_init
;;                           _ser_init
;;                         _initIRobot
;;                          _initSongs
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
;; (3) _driveForDistance                                    10     8      2     393
;;                                             19 BANK0     10     8      2
;;                              _drive
;;                          _ser_putch
;;                          _ser_getch
;; ---------------------------------------------------------------------------------
;; (4) _adc_read_channel                                     4     2      2     510
;;                                             37 BANK0      4     2      2
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
;; (1) _play_iCreate_song                                    1     1      0      62
;;                                             12 BANK0      1     1      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (2) _rotateIR                                             6     5      1      99
;;                                             10 BANK0      6     5      1
;; ---------------------------------------------------------------------------------
;; (2) _initIRobot                                           3     3      0      31
;;                                             12 BANK0      3     3      0
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_control                                    3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (3) _lcd_write_data                                       3     3      0      31
;;                                             10 BANK0      3     3      0
;; ---------------------------------------------------------------------------------
;; (4) _waitFor                                              6     4      2     124
;;                                             12 BANK0      6     4      2
;;                          _ser_putch
;; ---------------------------------------------------------------------------------
;; (4) _ser_getch                                            2     2      0      34
;;                                             10 BANK0      2     2      0
;;                           _ser_isrx
;; ---------------------------------------------------------------------------------
;; (4) _drive                                                7     4      3     155
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
;; (2) _ser_init                                             1     1      0       0
;;                                             10 BANK0      1     1      0
;; ---------------------------------------------------------------------------------
;; (4) _ser_putch                                            2     2      0      31
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
;;   _play_iCreate_song
;;     _ser_putch
;;   _findWalls
;;     _rotateIR
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
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;   _goToNextCell
;;     _goRight
;;       _turnRight90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;     _goForward
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;     _goLeft
;;       _turnLeft90
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;     _goBackward
;;       _turnAround
;;         _drive
;;           _ser_putch
;;         _waitFor
;;           _ser_putch
;;       _driveForDistance
;;         _drive
;;           _ser_putch
;;         _ser_putch
;;         _ser_getch
;;           _ser_isrx
;;       _lcd_set_cursor
;;         _lcd_write_control
;;       _lcd_write_data
;;   _updateLocation
;;     _lcd_set_cursor
;;       _lcd_write_control
;;     _lcd_write_data
;;     _lcd_write_1_digit_bcd
;;       _lcd_write_data
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
;;BANK1               50      7      45       7       86.3%
;;BITBANK1            50      0       0       6        0.0%
;;CODE                 0      0       0       0        0.0%
;;DATA                 0      0      D9      12        0.0%
;;ABS                  0      0      CF       3        0.0%
;;NULL                 0      0       0       0        0.0%
;;STACK                0      0       A       2        0.0%
;;BANK0               50     29      46       5       87.5%
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
;;		line 277 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
;;		_play_iCreate_song
;;		_findWalls
;;		_goToNextCell
;;		_updateLocation
;; This function is called by:
;;		Startup code after reset
;; This function uses a non-reentrant model
;;
psect	maintext
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	277
	global	__size_of_main
	__size_of_main	equ	__end_of_main-_main
	
_main:	
	opt	stack 0
; Regs used in _main: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	278
	
l10045:	
;main.c: 278: init();
	fcall	_init
	line	279
;main.c: 279: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	283
	
l10047:	
;main.c: 283: lcd_set_cursor(0x00);
	movlw	(0)
	fcall	_lcd_set_cursor
	line	284
	
l10049:	
;main.c: 284: lcd_write_string("Walls@ --- (1,0)");
	movlw	((STR_3-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	285
;main.c: 285: lcd_set_cursor(0x40);
	movlw	(040h)
	fcall	_lcd_set_cursor
	line	286
	
l10051:	
;main.c: 286: lcd_write_string("cuOr: - dirMo: -");
	movlw	((STR_4-__stringbase))&0ffh
	fcall	_lcd_write_string
	line	287
	
l10053:	
;main.c: 287: play_iCreate_song(4);
	movlw	(04h)
	fcall	_play_iCreate_song
	goto	l10055
	line	288
;main.c: 288: while(1)
	
l3676:	
	line	290
	
l10055:	
;main.c: 289: {
;main.c: 290: if(start.pressed)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_start),w
	skipz
	goto	u4070
	goto	l10055
u4070:
	line	294
	
l10057:	
;main.c: 291: {
;main.c: 294: findWalls();
	fcall	_findWalls
	line	296
;main.c: 296: goToNextCell();
	fcall	_goToNextCell
	line	297
	
l10059:	
;main.c: 297: updateLocation();
	fcall	_updateLocation
	goto	l10055
	line	300
	
l3677:	
	goto	l10055
	line	301
	
l3678:	
	line	288
	goto	l10055
	
l3679:	
	line	302
	
l3680:	
	global	start
	ljmp	start
	opt stack 0
GLOBAL	__end_of_main
	__end_of_main:
;; =============== function _main ends ============

	signat	_main,88
	global	_findWalls
psect	text1302,local,class=CODE,delta=2
global __ptext1302
__ptext1302:

;; *************** function _findWalls *****************
;; Defined at:
;;		line 142 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  wallAtOrient    2    5[BANK1 ] int 
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
;;      Locals:         0       0       2       0       0
;;      Temps:          0       0       1       0       0
;;      Totals:         0       0       3       0       0
;;Total ram usage:        3 bytes
;; Hardware stack levels used:    1
;; Hardware stack levels required when called:    7
;; This function calls:
;;		_rotateIR
;;		_findWall
;;		_lcd_set_cursor
;;		_lcd_write_data
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1302
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	142
	global	__size_of_findWalls
	__size_of_findWalls	equ	__end_of_findWalls-_findWalls
	
_findWalls:	
	opt	stack 0
; Regs used in _findWalls: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	145
	
l9995:	
;main.c: 145: rotateIR(24, 0b00001111);
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
	line	146
	
l9997:	
;main.c: 146: rightWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u3961
	goto	u3960
	
u3961:
	bsf	(_rightWall/8),(_rightWall)&7
	goto	u3974
u3960:
	bcf	(_rightWall/8),(_rightWall)&7
u3974:
	line	147
	
l9999:	
;main.c: 147: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	148
;main.c: 148: frontWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u3981
	goto	u3980
	
u3981:
	bsf	(_frontWall/8),(_frontWall)&7
	goto	u3994
u3980:
	bcf	(_frontWall/8),(_frontWall)&7
u3994:
	line	149
	
l10001:	
;main.c: 149: rotateIR(24, 0b00001101);
	movlw	(0Dh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_rotateIR)
	movlw	(018h)
	fcall	_rotateIR
	line	150
	
l10003:	
;main.c: 150: leftWall = findWall();
	fcall	_findWall
	btfsc	status,0
	goto	u4001
	goto	u4000
	
u4001:
	bsf	(_leftWall/8),(_leftWall)&7
	goto	u4014
u4000:
	bcf	(_leftWall/8),(_leftWall)&7
u4014:
	line	151
;main.c: 151: rotateIR(24, 0b00001111);
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
	line	153
	
l10005:	
;main.c: 153: int wallAtOrientation = 0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(findWalls@wallAtOrientation)^080h
	clrf	(findWalls@wallAtOrientation+1)^080h
	line	154
	
l10007:	
;main.c: 154: lcd_set_cursor(0x07);
	movlw	(07h)
	fcall	_lcd_set_cursor
	line	155
	
l10009:	
;main.c: 155: if(rightWall)
	btfss	(_rightWall/8),(_rightWall)&7
	goto	u4021
	goto	u4020
u4021:
	goto	l10021
u4020:
	line	157
	
l10011:	
;main.c: 156: {
;main.c: 157: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	158
	
l10013:	
;main.c: 158: wallAtOrientation = RIGHT + currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	addlw	low(03h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(findWalls@wallAtOrientation)^080h
	movlw	high(03h)
	skipnc
	movlw	(high(03h)+1)&0ffh
	movwf	((findWalls@wallAtOrientation)^080h)+1
	line	159
	
l10015:	
;main.c: 159: if(wallAtOrientation >= 4)
	movf	(findWalls@wallAtOrientation+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(04h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4035
	movlw	low(04h)
	subwf	(findWalls@wallAtOrientation)^080h,w
u4035:

	skipc
	goto	u4031
	goto	u4030
u4031:
	goto	l10019
u4030:
	line	160
	
l10017:	
;main.c: 160: wallAtOrientation -= 4;
	movlw	low(-4)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(findWalls@wallAtOrientation)^080h,f
	skipnc
	incf	(findWalls@wallAtOrientation+1)^080h,f
	movlw	high(-4)
	addwf	(findWalls@wallAtOrientation+1)^080h,f
	goto	l10019
	
l3634:	
	line	161
	
l10019:	
;main.c: 161: walls[wallAtOrientation] = 1;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(findWalls@wallAtOrientation)^080h,w
	addlw	_walls&0ffh
	movwf	fsr0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	line	162
;main.c: 162: }
	goto	l10023
	line	163
	
l3633:	
	line	165
	
l10021:	
;main.c: 163: else
;main.c: 164: {
;main.c: 165: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l10023
	line	166
	
l3635:	
	line	167
	
l10023:	
;main.c: 166: }
;main.c: 167: if(frontWall)
	btfss	(_frontWall/8),(_frontWall)&7
	goto	u4041
	goto	u4040
u4041:
	goto	l10029
u4040:
	line	169
	
l10025:	
;main.c: 168: {
;main.c: 169: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	170
	
l10027:	
;main.c: 170: walls[FORWARD+currentOrientation] = 1;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w
	addlw	_walls&0ffh
	movwf	fsr0
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	line	171
;main.c: 171: }
	goto	l10031
	line	172
	
l3636:	
	line	173
	
l10029:	
;main.c: 172: else
;main.c: 173: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l10031
	
l3637:	
	line	174
	
l10031:	
;main.c: 174: if(leftWall)
	btfss	(_leftWall/8),(_leftWall)&7
	goto	u4051
	goto	u4050
u4051:
	goto	l10043
u4050:
	line	176
	
l10033:	
;main.c: 175: {
;main.c: 176: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	177
	
l10035:	
;main.c: 177: wallAtOrientation = LEFT + currentOrientation;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_currentOrientation),w	;volatile
	addlw	low(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(findWalls@wallAtOrientation)^080h
	movlw	high(01h)
	skipnc
	movlw	(high(01h)+1)&0ffh
	movwf	((findWalls@wallAtOrientation)^080h)+1
	line	178
	
l10037:	
;main.c: 178: if(wallAtOrientation >= 4)
	movf	(findWalls@wallAtOrientation+1)^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(04h))^80h
	subwf	btemp+1,w
	skipz
	goto	u4065
	movlw	low(04h)
	subwf	(findWalls@wallAtOrientation)^080h,w
u4065:

	skipc
	goto	u4061
	goto	u4060
u4061:
	goto	l10041
u4060:
	line	179
	
l10039:	
;main.c: 179: wallAtOrientation -= 4;
	movlw	low(-4)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	addwf	(findWalls@wallAtOrientation)^080h,f
	skipnc
	incf	(findWalls@wallAtOrientation+1)^080h,f
	movlw	high(-4)
	addwf	(findWalls@wallAtOrientation+1)^080h,f
	goto	l10041
	
l3639:	
	line	180
	
l10041:	
;main.c: 180: walls[wallAtOrientation] = 1;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(??_findWalls+0)^080h+0
	movf	(findWalls@wallAtOrientation)^080h,w
	addlw	_walls&0ffh
	movwf	fsr0
	movf	(??_findWalls+0)^080h+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	line	181
;main.c: 181: }
	goto	l3641
	line	182
	
l3638:	
	line	183
	
l10043:	
;main.c: 182: else
;main.c: 183: lcd_write_data(' ');
	movlw	(020h)
	fcall	_lcd_write_data
	goto	l3641
	
l3640:	
	line	184
	
l3641:	
	return
	opt stack 0
GLOBAL	__end_of_findWalls
	__end_of_findWalls:
;; =============== function _findWalls ends ============

	signat	_findWalls,88
	global	_goToNextCell
psect	text1303,local,class=CODE,delta=2
global __ptext1303
__ptext1303:

;; *************** function _goToNextCell *****************
;; Defined at:
;;		line 218 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
;;		_goRight
;;		_goForward
;;		_goLeft
;;		_goBackward
;; This function is called by:
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1303
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	218
	global	__size_of_goToNextCell
	__size_of_goToNextCell	equ	__end_of_goToNextCell-_goToNextCell
	
_goToNextCell:	
	opt	stack 1
; Regs used in _goToNextCell: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	219
	
l9985:	
;main.c: 219: if(!rightWall)
	btfsc	(_rightWall/8),(_rightWall)&7
	goto	u3931
	goto	u3930
u3931:
	goto	l3653
u3930:
	line	220
	
l9987:	
;main.c: 220: goRight();
	fcall	_goRight
	goto	l3659
	line	221
	
l3653:	
;main.c: 221: else if(!frontWall)
	btfsc	(_frontWall/8),(_frontWall)&7
	goto	u3941
	goto	u3940
u3941:
	goto	l3655
u3940:
	line	222
	
l9989:	
;main.c: 222: goForward();
	fcall	_goForward
	goto	l3659
	line	223
	
l3655:	
;main.c: 223: else if(!leftWall)
	btfsc	(_leftWall/8),(_leftWall)&7
	goto	u3951
	goto	u3950
u3951:
	goto	l9993
u3950:
	line	224
	
l9991:	
;main.c: 224: goLeft();
	fcall	_goLeft
	goto	l3659
	line	225
	
l3657:	
	line	226
	
l9993:	
;main.c: 225: else
;main.c: 226: goBackward();
	fcall	_goBackward
	goto	l3659
	
l3658:	
	goto	l3659
	
l3656:	
	goto	l3659
	
l3654:	
	line	227
	
l3659:	
	return
	opt stack 0
GLOBAL	__end_of_goToNextCell
	__end_of_goToNextCell:
;; =============== function _goToNextCell ends ============

	signat	_goToNextCell,88
	global	_findWall
psect	text1304,local,class=CODE,delta=2
global __ptext1304
__ptext1304:

;; *************** function _findWall *****************
;; Defined at:
;;		line 133 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1304
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	133
	global	__size_of_findWall
	__size_of_findWall	equ	__end_of_findWall-_findWall
	
_findWall:	
	opt	stack 0
; Regs used in _findWall: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	134
	
l9973:	
;main.c: 134: if(readIR() > 100)
	fcall	_readIR
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movf	(1+(?_readIR))^080h,w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(065h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3925
	movlw	low(065h)
	subwf	(0+(?_readIR))^080h,w
u3925:

	skipc
	goto	u3921
	goto	u3920
u3921:
	goto	l9981
u3920:
	line	135
	
l9975:	
;main.c: 135: return 0;
	clrc
	
	goto	l3629
	
l9977:	
	goto	l3629
	
l9979:	
	goto	l3629
	line	136
	
l3628:	
	line	137
	
l9981:	
;main.c: 136: else
;main.c: 137: return 1;
	setc
	
	goto	l3629
	
l9983:	
	goto	l3629
	
l3630:	
	line	138
	
l3629:	
	return
	opt stack 0
GLOBAL	__end_of_findWall
	__end_of_findWall:
;; =============== function _findWall ends ============

	signat	_findWall,88
	global	_updateLocation
psect	text1305,local,class=CODE,delta=2
global __ptext1305
__ptext1305:

;; *************** function _updateLocation *****************
;; Defined at:
;;		line 240 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1305
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	240
	global	__size_of_updateLocation
	__size_of_updateLocation	equ	__end_of_updateLocation-_updateLocation
	
_updateLocation:	
	opt	stack 3
; Regs used in _updateLocation: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	241
	
l9943:	
;main.c: 241: currentOrientation += directionMoved;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_directionMoved),w	;volatile
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_currentOrientation),f	;volatile
	line	243
	
l9945:	
;main.c: 243: if(currentOrientation >= 4)
	movlw	(04h)
	subwf	(_currentOrientation),w	;volatile
	skipc
	goto	u3911
	goto	u3910
u3911:
	goto	l9949
u3910:
	line	244
	
l9947:	
;main.c: 244: currentOrientation -= 4;
	movlw	low(04h)
	subwf	(_currentOrientation),f	;volatile
	goto	l9949
	
l3665:	
	line	246
	
l9949:	
;main.c: 246: lcd_set_cursor(0x46);
	movlw	(046h)
	fcall	_lcd_set_cursor
	line	247
;main.c: 247: switch(currentOrientation)
	goto	l9969
	line	249
;main.c: 248: {
;main.c: 249: case NORTH:
	
l3667:	
	line	250
	
l9951:	
;main.c: 250: ++yCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_yCoord),f	;volatile
	line	251
	
l9953:	
;main.c: 251: lcd_write_data('N');
	movlw	(04Eh)
	fcall	_lcd_write_data
	line	252
;main.c: 252: break;
	goto	l9971
	line	253
;main.c: 253: case SOUTH:
	
l3669:	
	line	254
	
l9955:	
;main.c: 254: --yCoord;
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	subwf	(_yCoord),f	;volatile
	line	255
	
l9957:	
;main.c: 255: lcd_write_data('S');
	movlw	(053h)
	fcall	_lcd_write_data
	line	256
;main.c: 256: break;
	goto	l9971
	line	257
;main.c: 257: case EAST:
	
l3670:	
	line	258
	
l9959:	
;main.c: 258: ++xCoord;
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_updateLocation+0)+0
	movf	(??_updateLocation+0)+0,w
	addwf	(_xCoord),f	;volatile
	line	259
	
l9961:	
;main.c: 259: lcd_write_data('E');
	movlw	(045h)
	fcall	_lcd_write_data
	line	260
;main.c: 260: break;
	goto	l9971
	line	261
;main.c: 261: case WEST:
	
l3671:	
	line	262
	
l9963:	
;main.c: 262: --xCoord;
	movlw	low(01h)
	subwf	(_xCoord),f	;volatile
	line	263
	
l9965:	
;main.c: 263: lcd_write_data('W');
	movlw	(057h)
	fcall	_lcd_write_data
	line	264
;main.c: 264: break;
	goto	l9971
	line	265
;main.c: 265: default:
	
l3672:	
	line	266
;main.c: 266: break;
	goto	l9971
	line	267
	
l9967:	
;main.c: 267: }
	goto	l9971
	line	247
	
l3666:	
	
l9969:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
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
	goto	l9963
	xorlw	1^0	; case 1
	skipnz
	goto	l9955
	xorlw	2^1	; case 2
	skipnz
	goto	l9959
	xorlw	3^2	; case 3
	skipnz
	goto	l9951
	goto	l9971
	opt asmopt_on

	line	267
	
l3668:	
	line	269
	
l9971:	
;main.c: 269: lcd_set_cursor(0x0C);
	movlw	(0Ch)
	fcall	_lcd_set_cursor
	line	270
;main.c: 270: lcd_write_1_digit_bcd(xCoord);
	movf	(_xCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	271
;main.c: 271: lcd_set_cursor(0x0E);
	movlw	(0Eh)
	fcall	_lcd_set_cursor
	line	272
;main.c: 272: lcd_write_1_digit_bcd(yCoord);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(_yCoord),w	;volatile
	fcall	_lcd_write_1_digit_bcd
	line	274
	
l3673:	
	return
	opt stack 0
GLOBAL	__end_of_updateLocation
	__end_of_updateLocation:
;; =============== function _updateLocation ends ============

	signat	_updateLocation,88
	global	_goRight
psect	text1306,local,class=CODE,delta=2
global __ptext1306
__ptext1306:

;; *************** function _goRight *****************
;; Defined at:
;;		line 231 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1306
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	231
	global	__size_of_goRight
	__size_of_goRight	equ	__end_of_goRight-_goRight
	
_goRight:	
	opt	stack 1
; Regs used in _goRight: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	232
	
l9933:	
;main.c: 232: turnRight90();
	fcall	_turnRight90
	line	233
	
l9935:	
;main.c: 233: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	234
	
l9937:	
;main.c: 234: directionMoved = RIGHT;
	movlw	(03h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goRight+0)+0
	movf	(??_goRight+0)+0,w
	movwf	(_directionMoved)	;volatile
	line	235
	
l9939:	
;main.c: 235: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	236
	
l9941:	
;main.c: 236: lcd_write_data('R');
	movlw	(052h)
	fcall	_lcd_write_data
	line	237
	
l3662:	
	return
	opt stack 0
GLOBAL	__end_of_goRight
	__end_of_goRight:
;; =============== function _goRight ends ============

	signat	_goRight,88
	global	_goLeft
psect	text1307,local,class=CODE,delta=2
global __ptext1307
__ptext1307:

;; *************** function _goLeft *****************
;; Defined at:
;;		line 208 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1307
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	208
	global	__size_of_goLeft
	__size_of_goLeft	equ	__end_of_goLeft-_goLeft
	
_goLeft:	
	opt	stack 1
; Regs used in _goLeft: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	209
	
l9923:	
;main.c: 209: turnLeft90();
	fcall	_turnLeft90
	line	210
	
l9925:	
;main.c: 210: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	211
	
l9927:	
;main.c: 211: directionMoved = LEFT;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_directionMoved)	;volatile
	bsf	status,0
	rlf	(_directionMoved),f	;volatile
	line	212
	
l9929:	
;main.c: 212: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	213
	
l9931:	
;main.c: 213: lcd_write_data('L');
	movlw	(04Ch)
	fcall	_lcd_write_data
	line	214
	
l3650:	
	return
	opt stack 0
GLOBAL	__end_of_goLeft
	__end_of_goLeft:
;; =============== function _goLeft ends ============

	signat	_goLeft,88
	global	_goForward
psect	text1308,local,class=CODE,delta=2
global __ptext1308
__ptext1308:

;; *************** function _goForward *****************
;; Defined at:
;;		line 199 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1308
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	199
	global	__size_of_goForward
	__size_of_goForward	equ	__end_of_goForward-_goForward
	
_goForward:	
	opt	stack 1
; Regs used in _goForward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	200
	
l9915:	
;main.c: 200: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	201
	
l9917:	
;main.c: 201: directionMoved = FORWARD;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_directionMoved)	;volatile
	line	202
	
l9919:	
;main.c: 202: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	203
	
l9921:	
;main.c: 203: lcd_write_data('F');
	movlw	(046h)
	fcall	_lcd_write_data
	line	204
	
l3647:	
	return
	opt stack 0
GLOBAL	__end_of_goForward
	__end_of_goForward:
;; =============== function _goForward ends ============

	signat	_goForward,88
	global	_goBackward
psect	text1309,local,class=CODE,delta=2
global __ptext1309
__ptext1309:

;; *************** function _goBackward *****************
;; Defined at:
;;		line 189 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1309
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	189
	global	__size_of_goBackward
	__size_of_goBackward	equ	__end_of_goBackward-_goBackward
	
_goBackward:	
	opt	stack 1
; Regs used in _goBackward: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	190
	
l9905:	
;main.c: 190: turnAround();
	fcall	_turnAround
	line	191
	
l9907:	
;main.c: 191: driveForDistance(1000);
	movlw	low(03E8h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_driveForDistance)
	movlw	high(03E8h)
	movwf	((?_driveForDistance))+1
	fcall	_driveForDistance
	line	192
	
l9909:	
;main.c: 192: directionMoved = BACKWARD;
	movlw	(02h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_goBackward+0)+0
	movf	(??_goBackward+0)+0,w
	movwf	(_directionMoved)	;volatile
	line	193
	
l9911:	
;main.c: 193: lcd_set_cursor(0x4F);
	movlw	(04Fh)
	fcall	_lcd_set_cursor
	line	194
	
l9913:	
;main.c: 194: lcd_write_data('B');
	movlw	(042h)
	fcall	_lcd_write_data
	line	195
	
l3644:	
	return
	opt stack 0
GLOBAL	__end_of_goBackward
	__end_of_goBackward:
;; =============== function _goBackward ends ============

	signat	_goBackward,88
	global	_readIR
psect	text1310,local,class=CODE,delta=2
global __ptext1310
__ptext1310:

;; *************** function _readIR *****************
;; Defined at:
;;		line 33 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
;; Parameters:    Size  Location     Type
;;		None
;; Auto vars:     Size  Location     Type
;;  cm              2    2[BANK1 ] int 
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
;;		_findWall
;; This function uses a non-reentrant model
;;
psect	text1310
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
	line	33
	global	__size_of_readIR
	__size_of_readIR	equ	__end_of_readIR-_readIR
	
_readIR:	
	opt	stack 0
; Regs used in _readIR: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	34
	
l9899:	
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
	
l9901:	
;ir.c: 35: return cm;
	movf	(readIR@cm+1)^080h,w
	clrf	(?_readIR+1)^080h
	addwf	(?_readIR+1)^080h
	movf	(readIR@cm)^080h,w
	clrf	(?_readIR)^080h
	addwf	(?_readIR)^080h

	goto	l5828
	
l9903:	
	line	36
	
l5828:	
	return
	opt stack 0
GLOBAL	__end_of_readIR
	__end_of_readIR:
;; =============== function _readIR ends ============

	signat	_readIR,90
	global	_init
psect	text1311,local,class=CODE,delta=2
global __ptext1311
__ptext1311:

;; *************** function _init *****************
;; Defined at:
;;		line 93 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1311
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	93
	global	__size_of_init
	__size_of_init	equ	__end_of_init-_init
	
_init:	
	opt	stack 2
; Regs used in _init: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	94
	
l9867:	
;main.c: 94: start.pressed = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_start)
	line	95
	
l9869:	
;main.c: 95: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	line	97
	
l9871:	
;main.c: 97: init_adc();
	fcall	_init_adc
	line	98
	
l9873:	
;main.c: 98: lcd_init();
	fcall	_lcd_init
	line	101
	
l9875:	
;main.c: 101: TRISB = 0b00000001;
	movlw	(01h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(134)^080h	;volatile
	line	104
	
l9877:	
;main.c: 104: OPTION_REG = 0b00000100;
	movlw	(04h)
	movwf	(129)^080h	;volatile
	line	106
	
l9879:	
;main.c: 106: TMR0IE = 1;
	bsf	(93/8),(93)&7
	line	107
	
l9881:	
;main.c: 107: SSPSTAT = 0b01000000;
	movlw	(040h)
	movwf	(148)^080h	;volatile
	line	108
	
l9883:	
;main.c: 108: SSPCON = 0b00100010;
	movlw	(022h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(20)	;volatile
	line	109
	
l9885:	
;main.c: 109: TRISC = 0b10010000;
	movlw	(090h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(135)^080h	;volatile
	line	110
	
l9887:	
;main.c: 110: PORTC = 0b00000000;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(7)	;volatile
	line	113
	
l9889:	
;main.c: 113: PEIE = 1;
	bsf	(94/8),(94)&7
	line	114
	
l9891:	
;main.c: 114: GIE = 1;
	bsf	(95/8),(95)&7
	line	116
	
l9893:	
;main.c: 116: ser_init();
	fcall	_ser_init
	line	117
	
l9895:	
;main.c: 117: initIRobot();
	fcall	_initIRobot
	line	118
	
l9897:	
;main.c: 118: initSongs();
	fcall	_initSongs
	line	119
	
l3622:	
	return
	opt stack 0
GLOBAL	__end_of_init
	__end_of_init:
;; =============== function _init ends ============

	signat	_init,88
	global	_initSongs
psect	text1312,local,class=CODE,delta=2
global __ptext1312
__ptext1312:

;; *************** function _initSongs *****************
;; Defined at:
;;		line 30 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
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
psect	text1312
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
	line	30
	global	__size_of_initSongs
	__size_of_initSongs	equ	__end_of_initSongs-_initSongs
	
_initSongs:	
	opt	stack 2
; Regs used in _initSongs: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	31
	
l9865:	
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
	
l5122:	
	return
	opt stack 0
GLOBAL	__end_of_initSongs
	__end_of_initSongs:
;; =============== function _initSongs ends ============

	signat	_initSongs,88
	global	_lcd_init
psect	text1313,local,class=CODE,delta=2
global __ptext1313
__ptext1313:

;; *************** function _lcd_init *****************
;; Defined at:
;;		line 78 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
psect	text1313
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
	line	78
	global	__size_of_lcd_init
	__size_of_lcd_init	equ	__end_of_lcd_init-_lcd_init
	
_lcd_init:	
	opt	stack 3
; Regs used in _lcd_init: [wreg+status,2+status,0+pclath+cstack]
	line	82
	
l9845:	
;lcd.c: 82: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	85
	
l9847:	
;lcd.c: 85: PORTD = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(8)	;volatile
	line	86
	
l9849:	
;lcd.c: 86: PORTE = 0;
	clrf	(9)	;volatile
	line	88
	
l9851:	
;lcd.c: 88: TRISD = 0b00000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	clrf	(136)^080h	;volatile
	line	89
	
l9853:	
;lcd.c: 89: TRISE = 0b00000000;
	clrf	(137)^080h	;volatile
	line	92
	
l9855:	
;lcd.c: 92: lcd_write_control(0b00000001);
	movlw	(01h)
	fcall	_lcd_write_control
	line	93
	
l9857:	
;lcd.c: 93: lcd_write_control(0b00111000);
	movlw	(038h)
	fcall	_lcd_write_control
	line	94
	
l9859:	
;lcd.c: 94: lcd_write_control(0b00001100);
	movlw	(0Ch)
	fcall	_lcd_write_control
	line	95
	
l9861:	
;lcd.c: 95: lcd_write_control(0b00000110);
	movlw	(06h)
	fcall	_lcd_write_control
	line	96
	
l9863:	
;lcd.c: 96: lcd_write_control(0b00000010);
	movlw	(02h)
	fcall	_lcd_write_control
	line	98
	
l2841:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_init
	__end_of_lcd_init:
;; =============== function _lcd_init ends ============

	signat	_lcd_init,88
	global	_lcd_write_1_digit_bcd
psect	text1314,local,class=CODE,delta=2
global __ptext1314
__ptext1314:

;; *************** function _lcd_write_1_digit_bcd *****************
;; Defined at:
;;		line 44 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
psect	text1314
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
	
l9843:	
;lcd.c: 45: lcd_write_data(data + 48);
	movf	(lcd_write_1_digit_bcd@data),w
	addlw	030h
	fcall	_lcd_write_data
	line	46
	
l2829:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_1_digit_bcd
	__end_of_lcd_write_1_digit_bcd:
;; =============== function _lcd_write_1_digit_bcd ends ============

	signat	_lcd_write_1_digit_bcd,4216
	global	_lcd_write_string
psect	text1315,local,class=CODE,delta=2
global __ptext1315
__ptext1315:

;; *************** function _lcd_write_string *****************
;; Defined at:
;;		line 38 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
;; Parameters:    Size  Location     Type
;;  s               1    wreg     PTR const unsigned char 
;;		 -> STR_4(17), STR_3(17), 
;; Auto vars:     Size  Location     Type
;;  s               1   14[BANK0 ] PTR const unsigned char 
;;		 -> STR_4(17), STR_3(17), 
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
psect	text1315
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
	
l9835:	
;lcd.c: 40: while(*s) lcd_write_data(*s++);
	goto	l9841
	
l2824:	
	
l9837:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	fcall	_lcd_write_data
	
l9839:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_lcd_write_string+0)+0
	movf	(??_lcd_write_string+0)+0,w
	addwf	(lcd_write_string@s),f
	goto	l9841
	
l2823:	
	
l9841:	
	movf	(lcd_write_string@s),w
	movwf	fsr0
	fcall	stringdir
	iorlw	0
	skipz
	goto	u3901
	goto	u3900
u3901:
	goto	l9837
u3900:
	goto	l2826
	
l2825:	
	line	41
	
l2826:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_string
	__end_of_lcd_write_string:
;; =============== function _lcd_write_string ends ============

	signat	_lcd_write_string,4216
	global	_lcd_set_cursor
psect	text1316,local,class=CODE,delta=2
global __ptext1316
__ptext1316:

;; *************** function _lcd_set_cursor *****************
;; Defined at:
;;		line 32 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
;;		_findWalls
;;		_goBackward
;;		_goForward
;;		_goLeft
;;		_goRight
;;		_updateLocation
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1316
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
	
l9831:	
;lcd.c: 33: address |= 0b10000000;
	bsf	(lcd_set_cursor@address)+(7/8),(7)&7
	line	34
	
l9833:	
;lcd.c: 34: lcd_write_control(address);
	movf	(lcd_set_cursor@address),w
	fcall	_lcd_write_control
	line	35
	
l2820:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_set_cursor
	__end_of_lcd_set_cursor:
;; =============== function _lcd_set_cursor ends ============

	signat	_lcd_set_cursor,4216
	global	_turnRight90
psect	text1317,local,class=CODE,delta=2
global __ptext1317
__ptext1317:

;; *************** function _turnRight90 *****************
;; Defined at:
;;		line 74 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
psect	text1317
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	74
	global	__size_of_turnRight90
	__size_of_turnRight90	equ	__end_of_turnRight90-_turnRight90
	
_turnRight90:	
	opt	stack 1
; Regs used in _turnRight90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	75
	
l9827:	
;drive.c: 75: drive(0, 25, 255, 255);
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
	line	76
;drive.c: 76: waitFor(157,255,169);
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
	line	77
;drive.c: 77: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	78
	
l9829:	
;drive.c: 78: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnRight90+0)+0+2),f
movlw	224
movwf	((??_turnRight90+0)+0+1),f
	movlw	254
movwf	((??_turnRight90+0)+0),f
u4087:
	decfsz	((??_turnRight90+0)+0),f
	goto	u4087
	decfsz	((??_turnRight90+0)+0+1),f
	goto	u4087
	decfsz	((??_turnRight90+0)+0+2),f
	goto	u4087
opt asmopt_on

	line	79
	
l1420:	
	return
	opt stack 0
GLOBAL	__end_of_turnRight90
	__end_of_turnRight90:
;; =============== function _turnRight90 ends ============

	signat	_turnRight90,88
	global	_turnLeft90
psect	text1318,local,class=CODE,delta=2
global __ptext1318
__ptext1318:

;; *************** function _turnLeft90 *****************
;; Defined at:
;;		line 66 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
;; This function uses a non-reentrant model
;;
psect	text1318
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	66
	global	__size_of_turnLeft90
	__size_of_turnLeft90	equ	__end_of_turnLeft90-_turnLeft90
	
_turnLeft90:	
	opt	stack 1
; Regs used in _turnLeft90: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	67
	
l9823:	
;drive.c: 67: drive(0, 25, 0, 1);
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
	line	68
;drive.c: 68: waitFor(157,0,85);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(055h)
	movwf	(??_turnLeft90+0)+0
	movf	(??_turnLeft90+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	69
;drive.c: 69: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	70
	
l9825:	
;drive.c: 70: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnLeft90+0)+0+2),f
movlw	224
movwf	((??_turnLeft90+0)+0+1),f
	movlw	254
movwf	((??_turnLeft90+0)+0),f
u4097:
	decfsz	((??_turnLeft90+0)+0),f
	goto	u4097
	decfsz	((??_turnLeft90+0)+0+1),f
	goto	u4097
	decfsz	((??_turnLeft90+0)+0+2),f
	goto	u4097
opt asmopt_on

	line	71
	
l1417:	
	return
	opt stack 0
GLOBAL	__end_of_turnLeft90
	__end_of_turnLeft90:
;; =============== function _turnLeft90 ends ============

	signat	_turnLeft90,88
	global	_turnAround
psect	text1319,local,class=CODE,delta=2
global __ptext1319
__ptext1319:

;; *************** function _turnAround *****************
;; Defined at:
;;		line 57 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
psect	text1319
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	57
	global	__size_of_turnAround
	__size_of_turnAround	equ	__end_of_turnAround-_turnAround
	
_turnAround:	
	opt	stack 1
; Regs used in _turnAround: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	58
	
l9817:	
;drive.c: 58: drive(0, 25, 0, 1);
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
	line	59
;drive.c: 59: waitFor(157,0,170);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_waitFor)
	movlw	(0AAh)
	movwf	(??_turnAround+0)+0
	movf	(??_turnAround+0)+0,w
	movwf	0+(?_waitFor)+01h
	movlw	(09Dh)
	fcall	_waitFor
	line	60
;drive.c: 60: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	61
	
l9819:	
;drive.c: 61: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u4107:
	decfsz	((??_turnAround+0)+0),f
	goto	u4107
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4107
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4107
opt asmopt_on

	line	62
	
l9821:	
;drive.c: 62: _delay((unsigned long)((6500)*(20000000/4000.0)));
	opt asmopt_off
movlw  165
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_turnAround+0)+0+2),f
movlw	224
movwf	((??_turnAround+0)+0+1),f
	movlw	254
movwf	((??_turnAround+0)+0),f
u4117:
	decfsz	((??_turnAround+0)+0),f
	goto	u4117
	decfsz	((??_turnAround+0)+0+1),f
	goto	u4117
	decfsz	((??_turnAround+0)+0+2),f
	goto	u4117
opt asmopt_on

	line	63
	
l1414:	
	return
	opt stack 0
GLOBAL	__end_of_turnAround
	__end_of_turnAround:
;; =============== function _turnAround ends ============

	signat	_turnAround,88
	global	_driveForDistance
psect	text1320,local,class=CODE,delta=2
global __ptext1320
__ptext1320:

;; *************** function _driveForDistance *****************
;; Defined at:
;;		line 22 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
psect	text1320
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	22
	global	__size_of_driveForDistance
	__size_of_driveForDistance	equ	__end_of_driveForDistance-_driveForDistance
	
_driveForDistance:	
	opt	stack 1
; Regs used in _driveForDistance: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
	line	25
	
l9797:	
;drive.c: 24: volatile char high, low;
;drive.c: 25: int deltaDistance = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(driveForDistance@deltaDistance)
	clrf	(driveForDistance@deltaDistance+1)
	line	26
;drive.c: 26: int distance = 0;
	clrf	(driveForDistance@distance)
	clrf	(driveForDistance@distance+1)
	line	28
	
l9799:	
;drive.c: 28: moving = 1;
	bsf	(_moving/8),(_moving)&7
	line	29
	
l9801:	
;drive.c: 29: drive(0, 250, 128, 0);
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
	line	32
;drive.c: 32: while(moving)
	goto	l9815
	
l1405:	
	line	36
	
l9803:	
;drive.c: 33: {
;drive.c: 36: ser_putch(142);
	movlw	(08Eh)
	fcall	_ser_putch
	line	37
;drive.c: 37: ser_putch(19);
	movlw	(013h)
	fcall	_ser_putch
	line	38
;drive.c: 38: high = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@high)	;volatile
	line	39
;drive.c: 39: low = ser_getch();
	fcall	_ser_getch
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_driveForDistance+0)+0
	movf	(??_driveForDistance+0)+0,w
	movwf	(driveForDistance@low)	;volatile
	line	40
	
l9805:	
;drive.c: 40: deltaDistance = high*256 + low;
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
	line	41
	
l9807:	
;drive.c: 41: distance += deltaDistance;
	movf	(driveForDistance@deltaDistance),w
	addwf	(driveForDistance@distance),f
	skipnc
	incf	(driveForDistance@distance+1),f
	movf	(driveForDistance@deltaDistance+1),w
	addwf	(driveForDistance@distance+1),f
	line	43
	
l9809:	
;drive.c: 43: if(distance >= moveDistance)
	movf	(driveForDistance@distance+1),w
	xorlw	80h
	movwf	(??_driveForDistance+0)+0
	movf	(driveForDistance@moveDistance+1),w
	xorlw	80h
	subwf	(??_driveForDistance+0)+0,w
	skipz
	goto	u3885
	movf	(driveForDistance@moveDistance),w
	subwf	(driveForDistance@distance),w
u3885:

	skipc
	goto	u3881
	goto	u3880
u3881:
	goto	l9815
u3880:
	line	45
	
l9811:	
;drive.c: 44: {
;drive.c: 45: drive(0, 0, 0, 0);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_drive)
	clrf	0+(?_drive)+01h
	clrf	0+(?_drive)+02h
	movlw	(0)
	fcall	_drive
	line	46
	
l9813:	
;drive.c: 46: moving = 0;
	bcf	(_moving/8),(_moving)&7
	goto	l9815
	line	47
	
l1406:	
	goto	l9815
	line	48
	
l1404:	
	line	32
	
l9815:	
	btfsc	(_moving/8),(_moving)&7
	goto	u3891
	goto	u3890
u3891:
	goto	l9803
u3890:
	goto	l1408
	
l1407:	
	line	49
	
l1408:	
	return
	opt stack 0
GLOBAL	__end_of_driveForDistance
	__end_of_driveForDistance:
;; =============== function _driveForDistance ends ============

	signat	_driveForDistance,4216
	global	_adc_read_channel
psect	text1321,local,class=CODE,delta=2
global __ptext1321
__ptext1321:

;; *************** function _adc_read_channel *****************
;; Defined at:
;;		line 7 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
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
psect	text1321
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
	line	7
	global	__size_of_adc_read_channel
	__size_of_adc_read_channel	equ	__end_of_adc_read_channel-_adc_read_channel
	
_adc_read_channel:	
	opt	stack 0
; Regs used in _adc_read_channel: [wreg-fsr0h+status,2+status,0+btemp+1+pclath+cstack]
;adc_read_channel@channel stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(adc_read_channel@channel)
	line	8
	
l9781:	
;adc.c: 8: switch(channel)
	goto	l9789
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
	goto	l9791
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
	goto	l9791
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
	goto	l9791
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
	goto	l9791
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
	goto	l9791
	line	37
;adc.c: 37: default:
	
l696:	
	line	38
	
l9783:	
;adc.c: 38: return 0;
	clrf	(?_adc_read_channel)
	clrf	(?_adc_read_channel+1)
	goto	l697
	
l9785:	
	goto	l697
	line	39
	
l9787:	
;adc.c: 39: }
	goto	l9791
	line	8
	
l689:	
	
l9789:	
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
	goto	l9783
	opt asmopt_on

	line	39
	
l691:	
	line	41
	
l9791:	
;adc.c: 41: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
movwf	(??_adc_read_channel+0)+0,f
u4127:
decfsz	(??_adc_read_channel+0)+0,f
	goto	u4127
opt asmopt_on

	line	43
	
l9793:	
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
	
l9795:	
	line	45
	
l697:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read_channel
	__end_of_adc_read_channel:
;; =============== function _adc_read_channel ends ============

	signat	_adc_read_channel,4218
	global	_convert
psect	text1322,local,class=CODE,delta=2
global __ptext1322
__ptext1322:

;; *************** function _convert *****************
;; Defined at:
;;		line 11 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
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
psect	text1322
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
	line	11
	global	__size_of_convert
	__size_of_convert	equ	__end_of_convert-_convert
	
_convert:	
	opt	stack 1
; Regs used in _convert: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	12
	
l9721:	
;ir.c: 12: if(adc_value < 82)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(052h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3815
	movlw	low(052h)
	subwf	(convert@adc_value),w
u3815:

	skipnc
	goto	u3811
	goto	u3810
u3811:
	goto	l9729
u3810:
	line	13
	
l9723:	
;ir.c: 13: return 999;
	movlw	low(03E7h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(?_convert)
	movlw	high(03E7h)
	movwf	((?_convert))+1
	goto	l5812
	
l9725:	
	goto	l5812
	
l9727:	
	goto	l5812
	line	14
	
l5811:	
	
l9729:	
;ir.c: 14: else if(adc_value < 133)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(085h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3825
	movlw	low(085h)
	subwf	(convert@adc_value),w
u3825:

	skipnc
	goto	u3821
	goto	u3820
u3821:
	goto	l9737
u3820:
	line	15
	
l9731:	
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
	goto	l5812
	
l9733:	
	goto	l5812
	
l9735:	
	goto	l5812
	line	16
	
l5814:	
	
l9737:	
;ir.c: 16: else if(adc_value < 184)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0B8h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3835
	movlw	low(0B8h)
	subwf	(convert@adc_value),w
u3835:

	skipnc
	goto	u3831
	goto	u3830
u3831:
	goto	l9745
u3830:
	line	17
	
l9739:	
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
	goto	l5812
	
l9741:	
	goto	l5812
	
l9743:	
	goto	l5812
	line	18
	
l5816:	
	
l9745:	
;ir.c: 18: else if(adc_value < 256)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(0100h))^80h
	subwf	btemp+1,w
	skipz
	goto	u3845
	movlw	low(0100h)
	subwf	(convert@adc_value),w
u3845:

	skipnc
	goto	u3841
	goto	u3840
u3841:
	goto	l9753
u3840:
	line	19
	
l9747:	
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
	goto	l5812
	
l9749:	
	goto	l5812
	
l9751:	
	goto	l5812
	line	20
	
l5818:	
	
l9753:	
;ir.c: 20: else if(adc_value < 317)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(013Dh))^80h
	subwf	btemp+1,w
	skipz
	goto	u3855
	movlw	low(013Dh)
	subwf	(convert@adc_value),w
u3855:

	skipnc
	goto	u3851
	goto	u3850
u3851:
	goto	l9761
u3850:
	line	21
	
l9755:	
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
	goto	l5812
	
l9757:	
	goto	l5812
	
l9759:	
	goto	l5812
	line	22
	
l5820:	
	
l9761:	
;ir.c: 22: else if(adc_value < 410)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(019Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u3865
	movlw	low(019Ah)
	subwf	(convert@adc_value),w
u3865:

	skipnc
	goto	u3861
	goto	u3860
u3861:
	goto	l9769
u3860:
	line	23
	
l9763:	
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
	goto	l5812
	
l9765:	
	goto	l5812
	
l9767:	
	goto	l5812
	line	24
	
l5822:	
	
l9769:	
;ir.c: 24: else if(adc_value < 522)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(convert@adc_value+1),w
	xorlw	80h
	movwf	btemp+1
	movlw	(high(020Ah))^80h
	subwf	btemp+1,w
	skipz
	goto	u3875
	movlw	low(020Ah)
	subwf	(convert@adc_value),w
u3875:

	skipnc
	goto	u3871
	goto	u3870
u3871:
	goto	l9777
u3870:
	line	25
	
l9771:	
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
	goto	l5812
	
l9773:	
	goto	l5812
	
l9775:	
	goto	l5812
	line	26
	
l5824:	
	
l9777:	
;ir.c: 26: else return 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(?_convert)
	clrf	(?_convert+1)
	goto	l5812
	
l9779:	
	goto	l5812
	
l5825:	
	goto	l5812
	
l5823:	
	goto	l5812
	
l5821:	
	goto	l5812
	
l5819:	
	goto	l5812
	
l5817:	
	goto	l5812
	
l5815:	
	goto	l5812
	
l5813:	
	line	27
	
l5812:	
	return
	opt stack 0
GLOBAL	__end_of_convert
	__end_of_convert:
;; =============== function _convert ends ============

	signat	_convert,4218
	global	_ser_putArr
psect	text1323,local,class=CODE,delta=2
global __ptext1323
__ptext1323:

;; *************** function _ser_putArr *****************
;; Defined at:
;;		line 73 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
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
psect	text1323
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
	line	73
	global	__size_of_ser_putArr
	__size_of_ser_putArr	equ	__end_of_ser_putArr-_ser_putArr
	
_ser_putArr:	
	opt	stack 2
; Regs used in _ser_putArr: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	74
	
l9713:	
;ser.c: 74: for(int i =0; i< length; i++)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(ser_putArr@i)
	clrf	(ser_putArr@i+1)
	goto	l9719
	line	75
	
l4392:	
	line	76
	
l9715:	
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
	goto	u3790
	decf	(??_ser_putArr+0)+0,f
u3790:
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
	
l9717:	
	movlw	low(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	addwf	(ser_putArr@i),f
	skipnc
	incf	(ser_putArr@i+1),f
	movlw	high(01h)
	addwf	(ser_putArr@i+1),f
	goto	l9719
	
l4391:	
	
l9719:	
	movf	(ser_putArr@i+1),w
	xorlw	80h
	movwf	(??_ser_putArr+0)+0
	movf	(ser_putArr@length+1),w
	xorlw	80h
	subwf	(??_ser_putArr+0)+0,w
	skipz
	goto	u3805
	movf	(ser_putArr@length),w
	subwf	(ser_putArr@i),w
u3805:

	skipc
	goto	u3801
	goto	u3800
u3801:
	goto	l9715
u3800:
	goto	l4394
	
l4393:	
	line	78
	
l4394:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putArr
	__end_of_ser_putArr:
;; =============== function _ser_putArr ends ============

	signat	_ser_putArr,8312
	global	_play_iCreate_song
psect	text1324,local,class=CODE,delta=2
global __ptext1324
__ptext1324:

;; *************** function _play_iCreate_song *****************
;; Defined at:
;;		line 24 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
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
;;		_main
;; This function uses a non-reentrant model
;;
psect	text1324
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\songs.c"
	line	24
	global	__size_of_play_iCreate_song
	__size_of_play_iCreate_song	equ	__end_of_play_iCreate_song-_play_iCreate_song
	
_play_iCreate_song:	
	opt	stack 4
; Regs used in _play_iCreate_song: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;play_iCreate_song@song stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(play_iCreate_song@song)
	line	25
	
l9711:	
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
	
l5119:	
	return
	opt stack 0
GLOBAL	__end_of_play_iCreate_song
	__end_of_play_iCreate_song:
;; =============== function _play_iCreate_song ends ============

	signat	_play_iCreate_song,4216
	global	_rotateIR
psect	text1325,local,class=CODE,delta=2
global __ptext1325
__ptext1325:

;; *************** function _rotateIR *****************
;; Defined at:
;;		line 39 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
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
;; This function uses a non-reentrant model
;;
psect	text1325
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ir.c"
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
	
l9693:	
;ir.c: 40: PORTC |= 0b00000011;
	movlw	(03h)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	iorwf	(7),f	;volatile
	line	41
	
l9695:	
;ir.c: 41: SSPBUF = direction;
	movf	(rotateIR@direction),w
	movwf	(19)	;volatile
	line	42
	
l9697:	
;ir.c: 42: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u4137:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4137
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4137
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4137
	nop2
opt asmopt_on

	line	44
	
l9699:	
;ir.c: 44: for (char stepNum = 1; stepNum <= steps; ++stepNum)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(rotateIR@stepNum)
	bsf	status,0
	rlf	(rotateIR@stepNum),f
	goto	l5831
	line	45
	
l5832:	
	line	46
;ir.c: 45: {
;ir.c: 46: PORTC |= 0b00000100;
	bsf	(7)+(2/8),(2)&7	;volatile
	line	47
	
l9701:	
;ir.c: 47: PORTC &= 0b11111011;
	movlw	(0FBh)
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	andwf	(7),f	;volatile
	line	48
	
l9703:	
;ir.c: 48: _delay((unsigned long)((20)*(20000000/4000.0)));
	opt asmopt_off
movlw	130
movwf	((??_rotateIR+0)+0+1),f
	movlw	221
movwf	((??_rotateIR+0)+0),f
u4147:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4147
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4147
	nop2
opt asmopt_on

	line	44
	
l9705:	
	movlw	(01h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(??_rotateIR+0)+0
	movf	(??_rotateIR+0)+0,w
	addwf	(rotateIR@stepNum),f
	
l5831:	
	movf	(rotateIR@stepNum),w
	subwf	(rotateIR@steps),w
	skipnc
	goto	u3781
	goto	u3780
u3781:
	goto	l5832
u3780:
	goto	l9707
	
l5833:	
	line	51
	
l9707:	
;ir.c: 49: }
;ir.c: 51: SSPBUF = 0b00000000;
	clrf	(19)	;volatile
	line	52
	
l9709:	
;ir.c: 52: _delay((unsigned long)((200)*(20000000/4000.0)));
	opt asmopt_off
movlw  6
movwf	((??_rotateIR+0)+0+2),f
movlw	19
movwf	((??_rotateIR+0)+0+1),f
	movlw	177
movwf	((??_rotateIR+0)+0),f
u4157:
	decfsz	((??_rotateIR+0)+0),f
	goto	u4157
	decfsz	((??_rotateIR+0)+0+1),f
	goto	u4157
	decfsz	((??_rotateIR+0)+0+2),f
	goto	u4157
	nop2
opt asmopt_on

	line	54
	
l5834:	
	return
	opt stack 0
GLOBAL	__end_of_rotateIR
	__end_of_rotateIR:
;; =============== function _rotateIR ends ============

	signat	_rotateIR,8312
	global	_initIRobot
psect	text1326,local,class=CODE,delta=2
global __ptext1326
__ptext1326:

;; *************** function _initIRobot *****************
;; Defined at:
;;		line 122 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1326
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	122
	global	__size_of_initIRobot
	__size_of_initIRobot	equ	__end_of_initIRobot-_initIRobot
	
_initIRobot:	
	opt	stack 3
; Regs used in _initIRobot: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	123
	
l9687:	
;main.c: 123: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	((??_initIRobot+0)+0+2),f
movlw	138
movwf	((??_initIRobot+0)+0+1),f
	movlw	86
movwf	((??_initIRobot+0)+0),f
u4167:
	decfsz	((??_initIRobot+0)+0),f
	goto	u4167
	decfsz	((??_initIRobot+0)+0+1),f
	goto	u4167
	decfsz	((??_initIRobot+0)+0+2),f
	goto	u4167
	nop2
opt asmopt_on

	line	124
	
l9689:	
;main.c: 124: ser_putch(128);
	movlw	(080h)
	fcall	_ser_putch
	line	125
	
l9691:	
;main.c: 125: ser_putch(132);
	movlw	(084h)
	fcall	_ser_putch
	line	126
	
l3625:	
	return
	opt stack 0
GLOBAL	__end_of_initIRobot
	__end_of_initIRobot:
;; =============== function _initIRobot ends ============

	signat	_initIRobot,88
	global	_lcd_write_control
psect	text1327,local,class=CODE,delta=2
global __ptext1327
__ptext1327:

;; *************** function _lcd_write_control *****************
;; Defined at:
;;		line 8 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
psect	text1327
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
	
l9679:	
;lcd.c: 9: RE2 = 0;
	bcf	(74/8),(74)&7
	line	10
;lcd.c: 10: RE1 = 0;
	bcf	(73/8),(73)&7
	line	11
;lcd.c: 11: RE0 = 0;
	bcf	(72/8),(72)&7
	line	12
	
l9681:	
;lcd.c: 12: PORTD = databyte;
	movf	(lcd_write_control@databyte),w
	movwf	(8)	;volatile
	line	13
	
l9683:	
;lcd.c: 13: RE2 = 1;
	bsf	(74/8),(74)&7
	line	14
	
l9685:	
;lcd.c: 14: RE2 = 0;
	bcf	(74/8),(74)&7
	line	15
;lcd.c: 15: _delay((unsigned long)((2)*(20000000/4000.0)));
	opt asmopt_off
movlw	13
movwf	((??_lcd_write_control+0)+0+1),f
	movlw	251
movwf	((??_lcd_write_control+0)+0),f
u4177:
	decfsz	((??_lcd_write_control+0)+0),f
	goto	u4177
	decfsz	((??_lcd_write_control+0)+0+1),f
	goto	u4177
	nop2
opt asmopt_on

	line	16
	
l2814:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_control
	__end_of_lcd_write_control:
;; =============== function _lcd_write_control ends ============

	signat	_lcd_write_control,4216
	global	_lcd_write_data
psect	text1328,local,class=CODE,delta=2
global __ptext1328
__ptext1328:

;; *************** function _lcd_write_data *****************
;; Defined at:
;;		line 20 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
;;		_testEEPROM
;;		_lcd_write_3_digit_bcd
;; This function uses a non-reentrant model
;;
psect	text1328
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\lcd.c"
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
	
l9671:	
;lcd.c: 21: RE2 = 0;
	bcf	(74/8),(74)&7
	line	22
;lcd.c: 22: RE1 = 0;
	bcf	(73/8),(73)&7
	line	23
;lcd.c: 23: RE0 = 1;
	bsf	(72/8),(72)&7
	line	24
	
l9673:	
;lcd.c: 24: PORTD = databyte;
	movf	(lcd_write_data@databyte),w
	movwf	(8)	;volatile
	line	25
	
l9675:	
;lcd.c: 25: RE2 = 1;
	bsf	(74/8),(74)&7
	line	26
	
l9677:	
;lcd.c: 26: RE2 = 0;
	bcf	(74/8),(74)&7
	line	27
;lcd.c: 27: _delay((unsigned long)((1)*(20000000/4000.0)));
	opt asmopt_off
movlw	7
movwf	((??_lcd_write_data+0)+0+1),f
	movlw	125
movwf	((??_lcd_write_data+0)+0),f
u4187:
	decfsz	((??_lcd_write_data+0)+0),f
	goto	u4187
	decfsz	((??_lcd_write_data+0)+0+1),f
	goto	u4187
opt asmopt_on

	line	28
	
l2817:	
	return
	opt stack 0
GLOBAL	__end_of_lcd_write_data
	__end_of_lcd_write_data:
;; =============== function _lcd_write_data ends ============

	signat	_lcd_write_data,4216
	global	_waitFor
psect	text1329,local,class=CODE,delta=2
global __ptext1329
__ptext1329:

;; *************** function _waitFor *****************
;; Defined at:
;;		line 82 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
psect	text1329
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	82
	global	__size_of_waitFor
	__size_of_waitFor	equ	__end_of_waitFor-_waitFor
	
_waitFor:	
	opt	stack 1
; Regs used in _waitFor: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;waitFor@type stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(waitFor@type)
	line	83
	
l9663:	
;drive.c: 83: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_waitFor+0)+0+2),f
movlw	138
movwf	((??_waitFor+0)+0+1),f
	movlw	86
movwf	((??_waitFor+0)+0),f
u4197:
	decfsz	((??_waitFor+0)+0),f
	goto	u4197
	decfsz	((??_waitFor+0)+0+1),f
	goto	u4197
	decfsz	((??_waitFor+0)+0+2),f
	goto	u4197
	nop2
opt asmopt_on

	line	84
	
l9665:	
;drive.c: 84: ser_putch(type);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@type),w
	fcall	_ser_putch
	line	85
	
l9667:	
;drive.c: 85: ser_putch(highByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@highByte),w
	fcall	_ser_putch
	line	86
	
l9669:	
;drive.c: 86: ser_putch(lowByte);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(waitFor@lowByte),w
	fcall	_ser_putch
	line	87
	
l1423:	
	return
	opt stack 0
GLOBAL	__end_of_waitFor
	__end_of_waitFor:
;; =============== function _waitFor ends ============

	signat	_waitFor,12408
	global	_ser_getch
psect	text1330,local,class=CODE,delta=2
global __ptext1330
__ptext1330:

;; *************** function _ser_getch *****************
;; Defined at:
;;		line 58 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
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
psect	text1330
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
	line	58
	global	__size_of_ser_getch
	__size_of_ser_getch	equ	__end_of_ser_getch-_ser_getch
	
_ser_getch:	
	opt	stack 1
; Regs used in _ser_getch: [wreg-fsr0h+status,2+status,0+pclath+cstack]
	line	61
	
l9647:	
;ser.c: 59: unsigned char c;
;ser.c: 61: while (ser_isrx()==0)
	goto	l9649
	
l4386:	
	line	62
;ser.c: 62: continue;
	goto	l9649
	
l4385:	
	line	61
	
l9649:	
	fcall	_ser_isrx
	btfss	status,0
	goto	u3771
	goto	u3770
u3771:
	goto	l9649
u3770:
	
l4387:	
	line	64
;ser.c: 64: GIE=0;
	bcf	(95/8),(95)&7
	line	65
	
l9651:	
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
	
l9653:	
;ser.c: 66: ++rxoptr;
	movlw	(01h)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	addwf	(_rxoptr),f	;volatile
	line	67
	
l9655:	
;ser.c: 67: rxoptr &= (16-1);
	movlw	(0Fh)
	movwf	(??_ser_getch+0)+0
	movf	(??_ser_getch+0)+0,w
	andwf	(_rxoptr),f	;volatile
	line	68
	
l9657:	
;ser.c: 68: GIE=1;
	bsf	(95/8),(95)&7
	line	69
	
l9659:	
;ser.c: 69: return c;
	movf	(ser_getch@c),w
	goto	l4388
	
l9661:	
	line	70
	
l4388:	
	return
	opt stack 0
GLOBAL	__end_of_ser_getch
	__end_of_ser_getch:
;; =============== function _ser_getch ends ============

	signat	_ser_getch,89
	global	_drive
psect	text1331,local,class=CODE,delta=2
global __ptext1331
__ptext1331:

;; *************** function _drive *****************
;; Defined at:
;;		line 12 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
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
psect	text1331
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\drive.c"
	line	12
	global	__size_of_drive
	__size_of_drive	equ	__end_of_drive-_drive
	
_drive:	
	opt	stack 1
; Regs used in _drive: [wreg-fsr0h+status,2+status,0+pclath+cstack]
;drive@highByteSpeed stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(drive@highByteSpeed)
	line	13
	
l9635:	
;drive.c: 13: _delay((unsigned long)((100)*(20000000/4000.0)));
	opt asmopt_off
movlw  3
movwf	((??_drive+0)+0+2),f
movlw	138
movwf	((??_drive+0)+0+1),f
	movlw	86
movwf	((??_drive+0)+0),f
u4207:
	decfsz	((??_drive+0)+0),f
	goto	u4207
	decfsz	((??_drive+0)+0+1),f
	goto	u4207
	decfsz	((??_drive+0)+0+2),f
	goto	u4207
	nop2
opt asmopt_on

	line	14
	
l9637:	
;drive.c: 14: ser_putch(137);
	movlw	(089h)
	fcall	_ser_putch
	line	15
	
l9639:	
;drive.c: 15: ser_putch(highByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteSpeed),w
	fcall	_ser_putch
	line	16
	
l9641:	
;drive.c: 16: ser_putch(lowByteSpeed);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteSpeed),w
	fcall	_ser_putch
	line	17
	
l9643:	
;drive.c: 17: ser_putch(highByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@highByteRadius),w
	fcall	_ser_putch
	line	18
	
l9645:	
;drive.c: 18: ser_putch(lowByteRadius);
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movf	(drive@lowByteRadius),w
	fcall	_ser_putch
	line	19
	
l1401:	
	return
	opt stack 0
GLOBAL	__end_of_drive
	__end_of_drive:
;; =============== function _drive ends ============

	signat	_drive,16504
	global	_init_adc
psect	text1332,local,class=CODE,delta=2
global __ptext1332
__ptext1332:

;; *************** function _init_adc *****************
;; Defined at:
;;		line 48 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
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
psect	text1332
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
	line	48
	global	__size_of_init_adc
	__size_of_init_adc	equ	__end_of_init_adc-_init_adc
	
_init_adc:	
	opt	stack 4
; Regs used in _init_adc: [wreg+status,2]
	line	50
	
l9625:	
;adc.c: 50: PORTA = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(5)	;volatile
	line	51
	
l9627:	
;adc.c: 51: TRISA = 0b00111111;
	movlw	(03Fh)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(133)^080h	;volatile
	line	54
	
l9629:	
;adc.c: 54: ADCON0 = 0b10100001;
	movlw	(0A1h)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(31)	;volatile
	line	55
	
l9631:	
;adc.c: 55: ADCON1 = 0b00000010;
	movlw	(02h)
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	movwf	(159)^080h	;volatile
	line	57
	
l9633:	
;adc.c: 57: _delay((unsigned long)((50)*(20000000/4000000.0)));
	opt asmopt_off
movlw	83
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
movwf	(??_init_adc+0)+0,f
u4217:
decfsz	(??_init_adc+0)+0,f
	goto	u4217
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
psect	text1333,local,class=CODE,delta=2
global __ptext1333
__ptext1333:

;; *************** function _adc_read *****************
;; Defined at:
;;		line 62 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
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
psect	text1333
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\adc.c"
	line	62
	global	__size_of_adc_read
	__size_of_adc_read	equ	__end_of_adc_read-_adc_read
	
_adc_read:	
	opt	stack 0
; Regs used in _adc_read: [wreg+status,2+status,0+btemp+1+pclath+cstack]
	line	65
	
l9615:	
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
	
l9617:	
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
	goto	u3751
	goto	u3750
u3751:
	goto	l703
u3750:
	goto	l9619
	
l705:	
	line	75
	
l9619:	
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
u3765:
	clrc
	rlf	(??_adc_read+2)+0,f
	rlf	(??_adc_read+2)+1,f
	decfsz	btemp+1,f
	goto	u3765
	movf	(0+(?___awdiv)),w
	addwf	0+(??_adc_read+2)+0,w
	movwf	(adc_read@adc_value)	;volatile
	movf	(1+(?___awdiv)),w
	skipnc
	incf	(1+(?___awdiv)),w
	addwf	1+(??_adc_read+2)+0,w
	movwf	1+(adc_read@adc_value)	;volatile
	line	77
	
l9621:	
;adc.c: 77: return (adc_value);
	movf	(adc_read@adc_value+1),w	;volatile
	clrf	(?_adc_read+1)
	addwf	(?_adc_read+1)
	movf	(adc_read@adc_value),w	;volatile
	clrf	(?_adc_read)
	addwf	(?_adc_read)

	goto	l706
	
l9623:	
	line	78
	
l706:	
	return
	opt stack 0
GLOBAL	__end_of_adc_read
	__end_of_adc_read:
;; =============== function _adc_read ends ============

	signat	_adc_read,90
	global	___awdiv
psect	text1334,local,class=CODE,delta=2
global __ptext1334
__ptext1334:

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
psect	text1334
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\awdiv.c"
	line	5
	global	__size_of___awdiv
	__size_of___awdiv	equ	__end_of___awdiv-___awdiv
	
___awdiv:	
	opt	stack 1
; Regs used in ___awdiv: [wreg+status,2+status,0]
	line	9
	
l9575:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___awdiv@sign)
	line	10
	btfss	(___awdiv@divisor+1),7
	goto	u3651
	goto	u3650
u3651:
	goto	l9579
u3650:
	line	11
	
l9577:	
	comf	(___awdiv@divisor),f
	comf	(___awdiv@divisor+1),f
	incf	(___awdiv@divisor),f
	skipnz
	incf	(___awdiv@divisor+1),f
	line	12
	clrf	(___awdiv@sign)
	bsf	status,0
	rlf	(___awdiv@sign),f
	goto	l9579
	line	13
	
l6657:	
	line	14
	
l9579:	
	btfss	(___awdiv@dividend+1),7
	goto	u3661
	goto	u3660
u3661:
	goto	l9585
u3660:
	line	15
	
l9581:	
	comf	(___awdiv@dividend),f
	comf	(___awdiv@dividend+1),f
	incf	(___awdiv@dividend),f
	skipnz
	incf	(___awdiv@dividend+1),f
	line	16
	
l9583:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	xorwf	(___awdiv@sign),f
	goto	l9585
	line	17
	
l6658:	
	line	18
	
l9585:	
	clrf	(___awdiv@quotient)
	clrf	(___awdiv@quotient+1)
	line	19
	
l9587:	
	movf	(___awdiv@divisor+1),w
	iorwf	(___awdiv@divisor),w
	skipnz
	goto	u3671
	goto	u3670
u3671:
	goto	l9607
u3670:
	line	20
	
l9589:	
	clrf	(___awdiv@counter)
	bsf	status,0
	rlf	(___awdiv@counter),f
	line	21
	goto	l9595
	
l6661:	
	line	22
	
l9591:	
	movlw	01h
	
u3685:
	clrc
	rlf	(___awdiv@divisor),f
	rlf	(___awdiv@divisor+1),f
	addlw	-1
	skipz
	goto	u3685
	line	23
	
l9593:	
	movlw	(01h)
	movwf	(??___awdiv+0)+0
	movf	(??___awdiv+0)+0,w
	addwf	(___awdiv@counter),f
	goto	l9595
	line	24
	
l6660:	
	line	21
	
l9595:	
	btfss	(___awdiv@divisor+1),(15)&7
	goto	u3691
	goto	u3690
u3691:
	goto	l9591
u3690:
	goto	l9597
	
l6662:	
	goto	l9597
	line	25
	
l6663:	
	line	26
	
l9597:	
	movlw	01h
	
u3705:
	clrc
	rlf	(___awdiv@quotient),f
	rlf	(___awdiv@quotient+1),f
	addlw	-1
	skipz
	goto	u3705
	line	27
	movf	(___awdiv@divisor+1),w
	subwf	(___awdiv@dividend+1),w
	skipz
	goto	u3715
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),w
u3715:
	skipc
	goto	u3711
	goto	u3710
u3711:
	goto	l9603
u3710:
	line	28
	
l9599:	
	movf	(___awdiv@divisor),w
	subwf	(___awdiv@dividend),f
	movf	(___awdiv@divisor+1),w
	skipc
	decf	(___awdiv@dividend+1),f
	subwf	(___awdiv@dividend+1),f
	line	29
	
l9601:	
	bsf	(___awdiv@quotient)+(0/8),(0)&7
	goto	l9603
	line	30
	
l6664:	
	line	31
	
l9603:	
	movlw	01h
	
u3725:
	clrc
	rrf	(___awdiv@divisor+1),f
	rrf	(___awdiv@divisor),f
	addlw	-1
	skipz
	goto	u3725
	line	32
	
l9605:	
	movlw	low(01h)
	subwf	(___awdiv@counter),f
	btfss	status,2
	goto	u3731
	goto	u3730
u3731:
	goto	l9597
u3730:
	goto	l9607
	
l6665:	
	goto	l9607
	line	33
	
l6659:	
	line	34
	
l9607:	
	movf	(___awdiv@sign),w
	skipz
	goto	u3740
	goto	l9611
u3740:
	line	35
	
l9609:	
	comf	(___awdiv@quotient),f
	comf	(___awdiv@quotient+1),f
	incf	(___awdiv@quotient),f
	skipnz
	incf	(___awdiv@quotient+1),f
	goto	l9611
	
l6666:	
	line	36
	
l9611:	
	movf	(___awdiv@quotient+1),w
	clrf	(?___awdiv+1)
	addwf	(?___awdiv+1)
	movf	(___awdiv@quotient),w
	clrf	(?___awdiv)
	addwf	(?___awdiv)

	goto	l6667
	
l9613:	
	line	37
	
l6667:	
	return
	opt stack 0
GLOBAL	__end_of___awdiv
	__end_of___awdiv:
;; =============== function ___awdiv ends ============

	signat	___awdiv,8314
	global	___wmul
psect	text1335,local,class=CODE,delta=2
global __ptext1335
__ptext1335:

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
psect	text1335
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\wmul.c"
	line	3
	global	__size_of___wmul
	__size_of___wmul	equ	__end_of___wmul-___wmul
	
___wmul:	
	opt	stack 1
; Regs used in ___wmul: [wreg+status,2+status,0]
	line	4
	
l9563:	
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(___wmul@product)
	clrf	(___wmul@product+1)
	goto	l9565
	line	6
	
l6517:	
	line	7
	
l9565:	
	btfss	(___wmul@multiplier),(0)&7
	goto	u3611
	goto	u3610
u3611:
	goto	l6518
u3610:
	line	8
	
l9567:	
	movf	(___wmul@multiplicand),w
	addwf	(___wmul@product),f
	skipnc
	incf	(___wmul@product+1),f
	movf	(___wmul@multiplicand+1),w
	addwf	(___wmul@product+1),f
	
l6518:	
	line	9
	movlw	01h
	
u3625:
	clrc
	rlf	(___wmul@multiplicand),f
	rlf	(___wmul@multiplicand+1),f
	addlw	-1
	skipz
	goto	u3625
	line	10
	
l9569:	
	movlw	01h
	
u3635:
	clrc
	rrf	(___wmul@multiplier+1),f
	rrf	(___wmul@multiplier),f
	addlw	-1
	skipz
	goto	u3635
	line	11
	movf	((___wmul@multiplier+1)),w
	iorwf	((___wmul@multiplier)),w
	skipz
	goto	u3641
	goto	u3640
u3641:
	goto	l9565
u3640:
	goto	l9571
	
l6519:	
	line	12
	
l9571:	
	movf	(___wmul@product+1),w
	clrf	(?___wmul+1)
	addwf	(?___wmul+1)
	movf	(___wmul@product),w
	clrf	(?___wmul)
	addwf	(?___wmul)

	goto	l6520
	
l9573:	
	line	13
	
l6520:	
	return
	opt stack 0
GLOBAL	__end_of___wmul
	__end_of___wmul:
;; =============== function ___wmul ends ============

	signat	___wmul,8314
	global	_ser_isrx
psect	text1336,local,class=CODE,delta=2
global __ptext1336
__ptext1336:

;; *************** function _ser_isrx *****************
;; Defined at:
;;		line 48 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
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
psect	text1336
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
	line	48
	global	__size_of_ser_isrx
	__size_of_ser_isrx	equ	__end_of_ser_isrx-_ser_isrx
	
_ser_isrx:	
	opt	stack 1
; Regs used in _ser_isrx: [wreg+status,2+status,0]
	line	49
	
l9515:	
;ser.c: 49: if(OERR) {
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfss	(193/8),(193)&7
	goto	u3541
	goto	u3540
u3541:
	goto	l9523
u3540:
	line	50
	
l9517:	
;ser.c: 50: CREN = 0;
	bcf	(196/8),(196)&7
	line	51
;ser.c: 51: CREN = 1;
	bsf	(196/8),(196)&7
	line	52
	
l9519:	
;ser.c: 52: return 0;
	clrc
	
	goto	l4382
	
l9521:	
	goto	l4382
	line	53
	
l4381:	
	line	54
	
l9523:	
;ser.c: 53: }
;ser.c: 54: return (rxiptr!=rxoptr);
	movf	(_rxiptr),w	;volatile
	xorwf	(_rxoptr),w	;volatile
	skipz
	goto	u3551
	goto	u3550
u3551:
	goto	l9527
u3550:
	
l9525:	
	clrc
	
	goto	l4382
	
l9411:	
	
l9527:	
	setc
	
	goto	l4382
	
l9413:	
	goto	l4382
	
l9529:	
	line	55
	
l4382:	
	return
	opt stack 0
GLOBAL	__end_of_ser_isrx
	__end_of_ser_isrx:
;; =============== function _ser_isrx ends ============

	signat	_ser_isrx,88
	global	_ser_init
psect	text1337,local,class=CODE,delta=2
global __ptext1337
__ptext1337:

;; *************** function _ser_init *****************
;; Defined at:
;;		line 124 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
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
psect	text1337
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
	line	124
	global	__size_of_ser_init
	__size_of_ser_init	equ	__end_of_ser_init-_ser_init
	
_ser_init:	
	opt	stack 4
; Regs used in _ser_init: [wreg+status,2+status,0]
	line	125
	
l9489:	
;ser.c: 125: TRISC |= 0b10000000;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(135)^080h+(7/8),(7)&7	;volatile
	line	126
	
l9491:	
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
	
l9493:	
;ser.c: 127: BRGH=1;
	bsf	(1218/8)^080h,(1218)&7
	line	129
	
l9495:	
;ser.c: 129: SPBRG=20;
	movlw	(014h)
	movwf	(153)^080h	;volatile
	line	132
	
l9497:	
;ser.c: 132: TX9=0;
	bcf	(1222/8)^080h,(1222)&7
	line	133
	
l9499:	
;ser.c: 133: RX9=0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bcf	(198/8),(198)&7
	line	135
	
l9501:	
;ser.c: 135: SYNC=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1220/8)^080h,(1220)&7
	line	136
	
l9503:	
;ser.c: 136: SPEN=1;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	bsf	(199/8),(199)&7
	line	137
	
l9505:	
;ser.c: 137: CREN=1;
	bsf	(196/8),(196)&7
	line	138
	
l9507:	
;ser.c: 138: TXIE=0;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	line	139
	
l9509:	
;ser.c: 139: RCIE=1;
	bsf	(1125/8)^080h,(1125)&7
	line	140
	
l9511:	
;ser.c: 140: TXEN=1;
	bsf	(1221/8)^080h,(1221)&7
	line	143
	
l9513:	
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
	
l4422:	
	return
	opt stack 0
GLOBAL	__end_of_ser_init
	__end_of_ser_init:
;; =============== function _ser_init ends ============

	signat	_ser_init,88
	global	_ser_putch
psect	text1338,local,class=CODE,delta=2
global __ptext1338
__ptext1338:

;; *************** function _ser_putch *****************
;; Defined at:
;;		line 81 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
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
psect	text1338
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\ser.c"
	line	81
	global	__size_of_ser_putch
	__size_of_ser_putch	equ	__end_of_ser_putch-_ser_putch
	
_ser_putch:	
	opt	stack 2
; Regs used in _ser_putch: [wreg-fsr0h+status,2+status,0]
;ser_putch@c stored from wreg
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(ser_putch@c)
	line	82
	
l9441:	
;ser.c: 82: while (((txiptr+1) & (16-1))==txoptr)
	goto	l9443
	
l4398:	
	line	83
;ser.c: 83: continue;
	goto	l9443
	
l4397:	
	line	82
	
l9443:	
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	xorwf	(_txoptr),w	;volatile
	skipnz
	goto	u3511
	goto	u3510
u3511:
	goto	l9443
u3510:
	
l4399:	
	line	84
;ser.c: 84: GIE=0;
	bcf	(95/8),(95)&7
	line	85
	
l9445:	
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
	
l9447:	
;ser.c: 86: txiptr=(txiptr+1) & (16-1);
	movf	(_txiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_ser_putch+0)+0
	movf	(??_ser_putch+0)+0,w
	movwf	(_txiptr)	;volatile
	line	87
	
l9449:	
;ser.c: 87: TXIE=1;
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bsf	(1124/8)^080h,(1124)&7
	line	88
	
l9451:	
;ser.c: 88: GIE=1;
	bsf	(95/8),(95)&7
	line	89
	
l4400:	
	return
	opt stack 0
GLOBAL	__end_of_ser_putch
	__end_of_ser_putch:
;; =============== function _ser_putch ends ============

	signat	_ser_putch,4216
	global	_isr1
psect	text1339,local,class=CODE,delta=2
global __ptext1339
__ptext1339:

;; *************** function _isr1 *****************
;; Defined at:
;;		line 51 in file "C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
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
psect	text1339
	file	"C:\Users\Lili\Dropbox\Mechatronics 2\COMPETITION\COMPETITIONv0.3\main.c"
	line	51
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
psect	text1339
	line	53
	
i1l8759:	
;main.c: 53: if(TMR0IF)
	btfss	(90/8),(90)&7
	goto	u280_21
	goto	u280_20
u280_21:
	goto	i1l3619
u280_20:
	line	55
	
i1l8761:	
;main.c: 54: {
;main.c: 55: TMR0IF = 0;
	bcf	(90/8),(90)&7
	line	56
	
i1l8763:	
;main.c: 56: TMR0 = 100;
	movlw	(064h)
	movwf	(1)	;volatile
	line	58
;main.c: 58: RTC_Counter++;
	movlw	low(01h)
	addwf	(_RTC_Counter),f	;volatile
	skipnc
	incf	(_RTC_Counter+1),f	;volatile
	movlw	high(01h)
	addwf	(_RTC_Counter+1),f	;volatile
	line	60
	
i1l8765:	
;main.c: 60: RTC_FLAG_1MS = 1;
	bsf	(_RTC_FLAG_1MS/8),(_RTC_FLAG_1MS)&7
	line	62
	
i1l8767:	
;main.c: 62: if(RTC_Counter % 10 == 0) RTC_FLAG_10MS = 1;
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
	goto	u281_21
	goto	u281_20
u281_21:
	goto	i1l8771
u281_20:
	
i1l8769:	
	bsf	(_RTC_FLAG_10MS/8),(_RTC_FLAG_10MS)&7
	goto	i1l8771
	
i1l3609:	
	line	63
	
i1l8771:	
;main.c: 63: if(RTC_Counter % 50 == 0) RTC_FLAG_50MS = 1;
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
	goto	u282_21
	goto	u282_20
u282_21:
	goto	i1l8775
u282_20:
	
i1l8773:	
	bsf	(_RTC_FLAG_50MS/8),(_RTC_FLAG_50MS)&7
	goto	i1l8775
	
i1l3610:	
	line	64
	
i1l8775:	
;main.c: 64: if(RTC_Counter % 500 == 0)
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
	goto	u283_21
	goto	u283_20
u283_21:
	goto	i1l8781
u283_20:
	line	66
	
i1l8777:	
;main.c: 65: {
;main.c: 66: RTC_FLAG_500MS = 1;
	bsf	(_RTC_FLAG_500MS/8),(_RTC_FLAG_500MS)&7
	line	67
	
i1l8779:	
;main.c: 67: RTC_Counter = 0;
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	clrf	(_RTC_Counter)	;volatile
	clrf	(_RTC_Counter+1)	;volatile
	goto	i1l8781
	line	69
	
i1l3611:	
	line	71
	
i1l8781:	
;main.c: 69: }
;main.c: 71: if(!RB0)
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	btfsc	(48/8),(48)&7
	goto	u284_21
	goto	u284_20
u284_21:
	goto	i1l3612
u284_20:
	line	73
	
i1l8783:	
;main.c: 72: {
;main.c: 73: start.debounceCount++;
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	0+(_start)+02h,f
	line	74
	
i1l8785:	
;main.c: 74: if(start.debounceCount >= 10 & start.released)
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
	goto	u285_21
	goto	u285_20
u285_21:
	goto	i1l8793
u285_20:
	line	76
	
i1l8787:	
;main.c: 75: {
;main.c: 76: start.pressed = 1;
	clrf	(_start)
	bsf	status,0
	rlf	(_start),f
	line	77
	
i1l8789:	
;main.c: 77: start.released = 0;
	clrf	0+(_start)+01h
	goto	i1l8793
	line	78
	
i1l3613:	
	line	79
;main.c: 78: }
;main.c: 79: }
	goto	i1l8793
	line	80
	
i1l3612:	
	line	82
;main.c: 80: else
;main.c: 81: {
;main.c: 82: start.debounceCount = 0;
	clrf	0+(_start)+02h
	line	83
	
i1l8791:	
;main.c: 83: start.released = 1;
	clrf	0+(_start)+01h
	bsf	status,0
	rlf	0+(_start)+01h,f
	goto	i1l8793
	line	84
	
i1l3614:	
	line	86
	
i1l8793:	
;main.c: 84: }
;main.c: 86: if (RCIF) { rxfifo[rxiptr]=RCREG; ser_tmp=(rxiptr+1) & (16-1); if (ser_tmp!=rxoptr) rxiptr=ser_tmp; } if (TXIF && TXIE) { TXREG = txfifo[txoptr]; ++txoptr; txoptr &= (16-1); if (txoptr==txiptr) { TXIE = 0; } };
	btfss	(101/8),(101)&7
	goto	u286_21
	goto	u286_20
u286_21:
	goto	i1l8803
u286_20:
	
i1l8795:	
	movf	(26),w	;volatile
	movwf	(??_isr1+0)+0
	movf	(_rxiptr),w
	addlw	_rxfifo&0ffh
	movwf	fsr0
	movf	(??_isr1+0)+0,w
	bcf	status, 7	;select IRP bank0
	movwf	indf
	
i1l8797:	
	movf	(_rxiptr),w	;volatile
	addlw	01h
	andlw	0Fh
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_ser_tmp)
	
i1l8799:	
	movf	(_ser_tmp),w
	xorwf	(_rxoptr),w	;volatile
	skipnz
	goto	u287_21
	goto	u287_20
u287_21:
	goto	i1l8803
u287_20:
	
i1l8801:	
	movf	(_ser_tmp),w
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	movwf	(_rxiptr)	;volatile
	goto	i1l8803
	
i1l3616:	
	goto	i1l8803
	
i1l3615:	
	
i1l8803:	
	btfss	(100/8),(100)&7
	goto	u288_21
	goto	u288_20
u288_21:
	goto	i1l3619
u288_20:
	
i1l8805:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	btfss	(1124/8)^080h,(1124)&7
	goto	u289_21
	goto	u289_20
u289_21:
	goto	i1l3619
u289_20:
	
i1l8807:	
	movf	(_txoptr),w
	addlw	_txfifo&0ffh
	movwf	fsr0
	bcf	status, 7	;select IRP bank1
	movf	indf,w
	bcf	status, 5	;RP0=0, select bank0
	bcf	status, 6	;RP1=0, select bank0
	movwf	(25)	;volatile
	
i1l8809:	
	movlw	(01h)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	addwf	(_txoptr),f	;volatile
	
i1l8811:	
	movlw	(0Fh)
	movwf	(??_isr1+0)+0
	movf	(??_isr1+0)+0,w
	andwf	(_txoptr),f	;volatile
	
i1l8813:	
	movf	(_txoptr),w	;volatile
	xorwf	(_txiptr),w	;volatile
	skipz
	goto	u290_21
	goto	u290_20
u290_21:
	goto	i1l3619
u290_20:
	
i1l8815:	
	bsf	status, 5	;RP0=1, select bank1
	bcf	status, 6	;RP1=0, select bank1
	bcf	(1124/8)^080h,(1124)&7
	goto	i1l3619
	
i1l3618:	
	goto	i1l3619
	
i1l3617:	
	goto	i1l3619
	line	87
	
i1l3608:	
	line	88
	
i1l3619:	
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
psect	text1340,local,class=CODE,delta=2
global __ptext1340
__ptext1340:

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
psect	text1340
	file	"C:\Program Files (x86)\HI-TECH Software\PICC\9.82\sources\lwmod.c"
	line	5
	global	__size_of___lwmod
	__size_of___lwmod	equ	__end_of___lwmod-___lwmod
	
___lwmod:	
	opt	stack 0
; Regs used in ___lwmod: [wreg+status,2+status,0]
	line	8
	
i1l8903:	
	movf	(___lwmod@divisor+1),w
	iorwf	(___lwmod@divisor),w
	skipnz
	goto	u302_21
	goto	u302_20
u302_21:
	goto	i1l8921
u302_20:
	line	9
	
i1l8905:	
	clrf	(___lwmod@counter)
	bsf	status,0
	rlf	(___lwmod@counter),f
	line	10
	goto	i1l8911
	
i1l6535:	
	line	11
	
i1l8907:	
	movlw	01h
	
u303_25:
	clrc
	rlf	(___lwmod@divisor),f
	rlf	(___lwmod@divisor+1),f
	addlw	-1
	skipz
	goto	u303_25
	line	12
	
i1l8909:	
	movlw	(01h)
	movwf	(??___lwmod+0)+0
	movf	(??___lwmod+0)+0,w
	addwf	(___lwmod@counter),f
	goto	i1l8911
	line	13
	
i1l6534:	
	line	10
	
i1l8911:	
	btfss	(___lwmod@divisor+1),(15)&7
	goto	u304_21
	goto	u304_20
u304_21:
	goto	i1l8907
u304_20:
	goto	i1l8913
	
i1l6536:	
	goto	i1l8913
	line	14
	
i1l6537:	
	line	15
	
i1l8913:	
	movf	(___lwmod@divisor+1),w
	subwf	(___lwmod@dividend+1),w
	skipz
	goto	u305_25
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),w
u305_25:
	skipc
	goto	u305_21
	goto	u305_20
u305_21:
	goto	i1l8917
u305_20:
	line	16
	
i1l8915:	
	movf	(___lwmod@divisor),w
	subwf	(___lwmod@dividend),f
	movf	(___lwmod@divisor+1),w
	skipc
	decf	(___lwmod@dividend+1),f
	subwf	(___lwmod@dividend+1),f
	goto	i1l8917
	
i1l6538:	
	line	17
	
i1l8917:	
	movlw	01h
	
u306_25:
	clrc
	rrf	(___lwmod@divisor+1),f
	rrf	(___lwmod@divisor),f
	addlw	-1
	skipz
	goto	u306_25
	line	18
	
i1l8919:	
	movlw	low(01h)
	subwf	(___lwmod@counter),f
	btfss	status,2
	goto	u307_21
	goto	u307_20
u307_21:
	goto	i1l8913
u307_20:
	goto	i1l8921
	
i1l6539:	
	goto	i1l8921
	line	19
	
i1l6533:	
	line	20
	
i1l8921:	
	movf	(___lwmod@dividend+1),w
	clrf	(?___lwmod+1)
	addwf	(?___lwmod+1)
	movf	(___lwmod@dividend),w
	clrf	(?___lwmod)
	addwf	(?___lwmod)

	goto	i1l6540
	
i1l8923:	
	line	21
	
i1l6540:	
	return
	opt stack 0
GLOBAL	__end_of___lwmod
	__end_of___lwmod:
;; =============== function ___lwmod ends ============

	signat	___lwmod,8314
psect	text1341,local,class=CODE,delta=2
global __ptext1341
__ptext1341:
	global	btemp
	btemp set 07Eh

	DABS	1,126,2	;btemp
	global	wtemp0
	wtemp0 set btemp
	end

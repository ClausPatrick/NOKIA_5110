;
; Serial_to_LCD.asm
;
; Created: 18.07.2020 11:37:12



.equ	varL	= 0x0060 ;
.equ	varH	= 0x022F ;                                    

.equ	mem_adc_in_L = 0x0060
.equ	mem_adc_in_H = 0x0254

.equ	mem_lcd_out_L = 0x0255
.equ	mem_lcd_out_H = 0x044d

.equ	baud	= 250000			; baudrate
.equ	bps		= (8000000/16/baud) - 1	; baud prescale   
.equ	stop_symbol = 0x15

                                    
;**** Global Register Variables ****
                          
.def	data_bufl = r0
.def	data_bufh = r1
.def	mem_count = r2
.def	togreg = r3
.def	datareg = r4
.def	adc_data = r5
.def	xpos = r6
.def	ypos = r7
.def	ms10 = r8
.def	ct10 = r9


.def	temp = r16
.def	temp2 = r17
.def	temp3 = r18
.def	lcd_data = r19
.def	traag1 = r20
.def	traag2 = r21       
.def	swreg = r22                   
                                    
;**** Interrupt Vectors ****

.org		0
	rjmp	RESET		

.ORG		0x00B		
	rjmp	UART_RXCP

.ORG		0x00D		
	rjmp	UART_TXCP

.org		0x00E
	rjmp	ADC_read	

;**************************************************************

RESET:





	ldi		temp,	high(RAMEND);
	out		SPH,	temp
	ldi		temp,	low(RAMEND)
	out		SPL,	temp			
	ldi		XL,LOW(mem_adc_in_L)		; initialize X pointer
	ldi		XH,HIGH(mem_adc_in_L)		; to ADC data in.
	ldi		YL,LOW(mem_lcd_out_L)		; initialize Y pointer
	ldi		YH,HIGH(mem_lcd_out_L)		; to ADC data in.


	ldi		temp, 0
	mov		temp2, temp
	mov		temp3, temp
	mov		ct10, temp
	mov		ms10, temp


	ldi		temp, 0xff
	mov		data_bufh, temp
	clr		temp


	ldi	temp, 0x0f
	mov	togreg, temp

	clr		temp
	mov		mem_count, temp
	ldi		r16, bps
	ldi		r17, 0

	out		UBRRL,	r16			; load baud prescale
	out		UBRRH,	r17			; to UBRR0
	ldi		temp,	0b11011000
	out		UCSRB,	temp			; and receiver	
	
	sbi		ddrd,	3	;LCD_CE	- Chip Enable
	sbi		ddrd,	2	;LCD_DC - Data/Command
	sbi		ddrb,	2	;!SS
	sbi		ddrb,	3	;MOSI / LCD_Din
	sbi		ddrb,	5	;SCK / LCD_Sck
	sbi		ddrb,	1	;LCD_Rst	
	cbi		ddrd,	4	;sw1
	cbi		ddrd,	5	;sw2
	cbi		ddrd,	6	;sw3		
	sbi		ddrc,	3	;LED5, blue
	sbi		ddrb,	4	;LED4, blue
	sbi		ddrb,	0	;LED2, oran
	sbi		ddrd,	7	;LED1, oran

	ldi		temp,	0b01010010
	out		SPCR,	temp


	sbi		portc,	3	;LED5, blue
	sbi		portb,	4	;LED4, blue
	sbi		portb,	0	;LED2, oran
	sbi		portd,	7	;LED1, oran
	
	cbi		portd,	2	;Sending Command - Lowing this bit by default.
	sbi		portd,	3	;Disable LCD_controller

	cbi		portb,	1	;LCD_Rst / LCD down
	sbi		portb,	2
	cbi		portd,	3	;ensable LCD_controller
	nop
	nop
	nop
	nop

	sbi		portd,	3	;Disable LCD_controller
	sbi		portb,	1	;LCD_Rst / LCD booting

	;Initialise ADC:
	ldi		Temp, 0b01100001
	out		ADMUX, Temp
	ldi		Temp, 0b10001000
	out		ADCSRA, Temp
	ldi		Temp, 0b11001000			
	out		ADCSRA, Temp
	
	rcall	lcd_init

	cbi		portc,	3	;LED5, blue
	cbi		portb,	4	;LED4, blue
	;cbi		portb,	5	;LED3, blue
	cbi		portb,	0	;LED2, oran
	cbi		portd,	7	;LED1, oran


	ldi		temp, 0
	mov		temp2, temp
	mov		lcd_data, temp
	mov		togreg, temp
	mov		mem_count, temp


	
	 		
main:
		sei
		;cbi		portc,	3	;LED5, blue
		cbi		portb,	4	;LED4, blue
		cbi		portb,	0	;LED2, oran
		cbi		portd,	7	;LED1, oran
		rcall	check_sw
		;cpi		lcd_data, stop_symbol		;Temporary storage into memory. 
		;breq	mem_process

					
		rjmp	main

lcd_init:	
		ldi		lcd_data, 0x21	;Extended commands, PD = 0 and V = 0, 
		rcall	print_lcd_command
		ldi		lcd_data, 0xe0	;Setting LCD Vop 5/4/0 = 1
		rcall	print_lcd_command
		ldi		lcd_data, 0x04	;Setting temp coeff tc0, tc1 = 0
		rcall	print_lcd_command
		ldi		lcd_data, 0x14	;LCD bias mode (1:48) bs2 = 1
		rcall	print_lcd_command
		ldi		lcd_data, 0x22	;Basic commands, PD = 0 and V = 1, 
		rcall	print_lcd_command
		ldi		lcd_data, 0x0c	;Normal mode, D = 1 and E = 0
		rcall	print_lcd_command

		ret

;Subroutine to keep track of switch toggles. No implementations yet.
check_sw:
			   sbis		pind,	4
			   rcall	test_hl_flank_0 ;goto if  released
			   sbic		pind,	4
			   rcall	test_lh_flank_0 ;goto if pressed
			   ret

test_lh_flank_0:
                sbrs	swreg, 0
                rjmp	lh_flank_0
				;sbi		portb,	4	;LED4, blue
                ret
 
                lh_flank_0:
                        ori     swreg, 0b00000001 ;flank bit
						ret
 
test_hl_flank_0:
 
				sbrc	swreg, 0
                rjmp	hl_flank_0
				;cbi		portb,	4	;LED4, blue
				ret
 
                hl_flank_0:
						
						andi	swreg, 0b11111110 ;flank bit
						in		temp, pinb
						sbrs	temp, 4
						sbi		portb, 4
						sbrc	temp, 4
						cbi		portb, 4
						ret
;Memory organisation. Not being used at the moment as data is written directly through to the screen, see UART interrupt routine.

mem_process:
			sbi		portc,	3	;LED5, blue
			sbi		portb,	0	;LED2, oran
			ldi		XL,LOW(varL)			; initialize X pointer
			ldi		XH,HIGH(varL)			; to var address
			ld		lcd_data, X+			;StartByte1
			ori		lcd_data, 0b10000000	;Specify X address and Preparing command
			rcall	print_lcd_command
			ld		lcd_data, X+			;StartByte2
			ori		lcd_data, 0b01000000	;Specify Y address and Preparing command
			andi	lcd_data, 0b01000111	;Specify Y address and Preparing command
			rcall	print_lcd_command
loop_mem_process:
			ld		lcd_data, X+			;Data loop
			cpi		lcd_data,	stop_symbol
			breq	end_message
			rcall	print_lcd_data
			rjmp	loop_mem_process
	end_message:
			clr		temp
			clr		lcd_data
			mov		mem_count, temp
			ldi		XL,LOW(varL)			; initialize X pointer
			ldi		XH,HIGH(varL)			; to var address
			ret	
	
;DATA for LCD is expected in reg 'lcd_data'
print_lcd_data:
				sbi		portd,	2	;Sending Data
print_lcd_command:
				cbi		portd,	3	;LCD_CE Enable LCD_controller
				;cbi		portd,	2	;Sending Command. Dont need this as is down by default.	
				out		SPDR,	lcd_data
	Wait_Transmit: ; Wait for transmission complete
				sbis	SPSR,SPIF
				rjmp	Wait_Transmit	
				sbi		portd,	3	;LCD_CE Disable LCD_controller
				cbi		portd,	2	;Sending Command - Lowing this bit by default.			
				ret

;Received data is written directly into screen out of the interrupt routine which is NOT best practice but it seems to work fine. 
;Alternative would be to store image in SRAM but it is bothersome to organise the limited memory to acccomodate bitmap.


UART_RXCP:
				sbi		portb,	4	;LED4, blue
				push	temp
				in		temp,	UDR

				out		UDR,	temp
				mov		lcd_data, temp
				rcall	print_lcd_data

	gets_end:

				pop		temp
				reti

UART_TXCP:		
		reti



;AD converter not implemented.
ADC_read:
				in		temp, ADCH
				mov		data_bufh, temp
				in		temp, ADCL
				mov		data_bufl, temp


		reti

;Delay routine using traag1 and traag2.
			delayr:
								ldi traag1, 0x05
							subtra2:
								ldi traag2, 0xff

							subtra1:
								dec traag2
								brne subtra1
								dec traag1
								brne subtra2

								ret
/*
message:
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x00fe, 0x00fd, 0x00fc, 0x00fb, 0x00fa, 0x00f9, 0x00f8, 0x00f7
.dw 0x0000, 0x00fd, 0x00fc, 0x00fb, 0x01fa, 0x00f9, 0x00f8, 0x00f7

 */


;ADC Multiplexer Selection Register –ADMUX
;REFS1 REFS0 ADLAR – MUX3 MUX2 MUX1 MUX0
;MUX3..0 Single Ended Input
;0000 ADC0
;0001 ADC1
;0010 ADC2
;0011 ADC3
;0100 ADC4
;0101 ADC5
;0110 ADC6
;0111 ADC7
;ADC Control andStatus Register A –ADCSRA
;ADEN ADSC ADFR ADIF ADIE ADPS2 ADPS1 ADPS0
;ADPS2 ADPS1 ADPS0 Division Factor
;0 0 0 2
;0 0 1 2
;0 1 0 4
;0 1 1 8
;1 0 0 16
;1 0 1 32
;1 1 0 64
;1 1 1 128

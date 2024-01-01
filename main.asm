; Main.asm file for PIC16F84A Tachometer
; Created:   Wed Dec 27 2023
; Processor: PIC16F84A
; Compiler:  MPASM (MPLAB)

; Definitions
#include p16f84a.inc    ; Include register definition file
list p=16F84A
#include <p16f84a.inc>
__CONFIG _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC
org 0x0000
goto Start
org 0x0004

; Define constants
DIPSW_PORT  equ 0x05    ; Port for DIP switch (Change to match your hardware)
FAN_DC_PORT equ 0x06    ; Port for controlling the fan (Change to match your hardware)
LM016L_PORT equ 0x07    ; Port for LCD (LM016L) (Change to match your hardware)
TC77_SIM    equ 0x08    ; Port for simulating TC77 sensor (Change to match your hardware)
BUZZER_PORT equ 0x09    ; Port for the buzzer (Change to match your hardware)

; Define constants for DIP switch positions
DIPSW_00    equ 0b00
DIPSW_01    equ 0b01
DIPSW_10    equ 0b10
DIPSW_11    equ 0b11

; Define constants for threshold values
THRESHOLD_63  equ 63
THRESHOLD_127 equ 127
THRESHOLD_191 equ 191
THRESHOLD_255 equ 255

; Define constants for speed percentages
SPEED_25  equ 25
SPEED_50  equ 50
SPEED_75  equ 75
SPEED_100 equ 100

; Variables
count       equ 0x20   ; Variable to store the count
rpm         equ 0x21   ; Variable to store RPM value
blade_count equ 0x22   ; Variable to store the number of blades
distance    equ 0x23   ; Variable to store the distance from IR sensor
threshold   equ 0x24   ; Variable to store the selected threshold
speed       equ 0x25   ; Variable to store the selected speed percentage

; RESET and INTERRUPT VECTORS

; Reset Vector
RST code 0x00
    goto Start

; CODE SEGMENT

PGM code

Start
    ; Initialize ports and variables
    clrf  PORTA      ; Clear all PORTA pins
    clrf  PORTB      ; Clear all PORTB pins
    clrf  PORTC      ; Clear all PORTC pins
    bsf   STATUS, RP0 ; Select bank 1
    movlw b'00001111' ; Set PORTA pins as digital inputs
    movwf TRISA      ; Set TRISA register
    movlw b'00000000' ; Set PORTB pins as digital outputs
    movwf TRISB      ; Set TRISB register
    movlw b'00000000' ; Set PORTC pins as digital outputs
    movwf TRISC      ; Set TRISC register
    bcf   STATUS, RP0 ; Select bank 0
    clrf  count       ; Clear count
    clrf  rpm         ; Clear rpm
    clrf  blade_count ; Clear blade_count
    clrf  distance    ; Clear distance
    clrf  threshold   ; Clear threshold
    clrf  speed       ; Clear speed

Loop
    ; Read DIP switch position
    bsf   PORTA, DIPSW_00 ; Pull-up resistor for DIP switch
    btfsc PORTA, DIPSW_00 ; Check DIP switch bit 0
    goto  DIPSW_01_SELECTED
    btfsc PORTA, DIPSW_01 ; Check DIP switch bit 1
    goto  DIPSW_10_SELECTED
    goto  DIPSW_00_SELECTED

DIPSW_00_SELECTED
    movlw THRESHOLD_63  ; Set threshold value
    movwf threshold
    movlw SPEED_25      ; Set speed percentage
    movwf speed
    goto  Check_IR

DIPSW_01_SELECTED
    movlw THRESHOLD_127 ; Set threshold value
    movwf threshold
    movlw SPEED_50      ; Set speed percentage
    movwf speed
    goto  Check_IR

DIPSW_10_SELECTED
    movlw THRESHOLD_191 ; Set threshold value
    movwf threshold
    movlw SPEED_75      ; Set speed percentage
    movwf speed
    goto  Check_IR

DIPSW_11_SELECTED
    movlw THRESHOLD_255 ; Set threshold value
    movwf threshold
    movlw SPEED_100     ; Set speed percentage
    movwf speed

Check_IR
    ; Read IR sensor input
    btfss PORTA, IR_SENSOR ; Check IR sensor input
    goto  IR_NOT_DETECTED
    incf  blade_count, F  ; Increment blade count
    goto  IR_DETECTED

IR_NOT_DETECTED
    ; IR sensor not detected, reset blade count
    clrf  blade_count

IR_DETECTED
    ; Calculate RPM
    movf  blade_count, W
    mulwf speed, W      ; Multiply by speed percentage
    movwf rpm
    movlw 12            ; Constant factor for RPM calculation
    divwf rpm, F

    ; Display RPM on LM016L
    call Init_LCD        ; Initialize LCD
    call lcd_send_byte   ; Send RPM value to LCD

    ; Control the fan based on RPM
    call set_fan_speed

    ; Simulate TC77 sensor
    call simulate_tc77

    ; Sound the buzzer if RPM exceeds threshold
    movf  rpm, W
    subwf threshold, W
    btfss STATUS, C
    goto  RPM_BELOW_THRESHOLD
    bsf   PORTB, BUZZER_PORT  ; Sound the buzzer
    goto  RPM_ABOVE_THRESHOLD

RPM_BELOW_THRESHOLD
    bcf   PORTB, BUZZER_PORT  ; Turn off the buzzer

RPM_ABOVE_THRESHOLD
; Define constants for timer values (adjust as needed)
TMR_PRELOAD equ 65536 - (5000000 / 4) ; Preload value for a 1-second delay (assuming a 4 MHz clock)

; ... (Other code and definitions)

; TIMER0 interrupt service routine
ISR_TMR0:
    ; Clear timer0 interrupt flag
    bcf INTCON, T0IF

    ; Implement a 1-second timer using timer0
    movlw TMR_PRELOAD & 0xFF   ; Load low byte of preload value
    movwf TMR0                 ; Preload timer0
    movlw TMR_PRELOAD >> 8     ; Load high byte of preload value
    movwf T0_REG               ; Store high byte in a variable

    ; Decrement a 5-second delay counter
    decfsz DELAY_COUNTER, F
    retfie                     ; Return if not yet 5 seconds

    ; 5 seconds have passed, reset counter and proceed with RPM measurement
    clrf DELAY_COUNTER

    retfie

; Initialize timer0 for a 1-second interrupt
Init_Timer0:
    ; Set up timer0 for 1-second interrupt
    movlw 0x87    ; Configure timer0 with 256 prescaler
    movwf OPTION_REG
    bsf INTCON, T0IE  ; Enable timer0 interrupt
    clrf T0_REG    ; Clear the high byte of preload value
    movlw TMR_PRELOAD & 0xFF   ; Load low byte of preload value
    movwf TMR0   ; Preload timer0
    bcf INTCON, T0IF   ; Clear timer0 interrupt flag
    bsf INTCON, GIE    ; Enable global interrupts
    return

Start

    ; Initialize timer0
    call Init_Timer0

Loop
    
    ; Go back to the main loop
    goto Loop


; Initialize LCD
Init_LCD:
    ; Implement the LCD initialization sequence here.
    ; Configure function, display settings, etc.
    ; Example (for 4-bit mode, 2 lines, 5x7 font size):
    ; Initialize LCD for 4-bit mode, 2 lines, 5x7 font size
    bsf   LM016L_PORT, RS   ; Set RS high for command
    bcf   LM016L_PORT, RW   ; Set RW low for write
    movlw 0x03              ; Function Set command
    call  lcd_send_byte     ; Send the command
    __delay_ms 2            ; Delay for 2ms
    movlw 0x02              ; Function Set command to set 4-bit mode
    call  lcd_send_byte     ; Send the command
    __delay_ms 2            ; Delay for 2ms
    movlw 0x28              ; Function Set command for 2 lines, 5x7 font
    call  lcd_send_byte     ; Send the command
    __delay_ms 2            ; Delay for 2ms
    ; Add more commands for initialization as needed
    return

; Send RPM value to LCD
lcd_send_byte:
    ; Implement sending data to the LCD here.
    ; Depending on your LCD module and communication protocol, this will vary.
    ; You may need to set RS, RW, and E pins appropriately.
    ; Load data into W before sending.
    ; Example (assuming 4-bit mode):
    bcf   LM016L_PORT, RS   ; Set RS low for data
    bcf   LM016L_PORT, RW   ; Set RW low for write
    movwf LM016L_PORT       ; Send data to LCD
    bsf   LM016L_PORT, E    ; Enable pulse
    nop                     ; Delay for E pulse
    bcf   LM016L_PORT, E    ; Disable E
    __delay_ms 2            ; Delay for 2ms 
    return

; Set fan speed based on RPM
set_fan_speed:
    ; Load RPM value from rpm variable
    ; Calculate the required fan speed control value (e.g., PWM duty cycle)
    ; Set the fan speed control value using your hardware-specific method
    ; Example (for PWM control):
    movf  rpm, W             ; Load RPM value into W
    ; Calculate PWM duty cycle based on RPM, adjust the formula as needed
    ; Note: You need to configure and use your PWM module
    movwf CCP1CON            ; Set PWM duty cycle
    return

; Simulate TC77 sensor and store data in a variable
simulate_tc77:
    ; Generate simulated temperature data (e.g., incrementing temperature values)
    ; Store the simulated temperature data in the TC77_SIM variable.
    ; Example (incrementing temperature simulation):
    movf  TC77_SIM, W        ; Load current simulated temperature
    addlw 1                  ; Increment temperature value 
    movwf TC77_SIM           ; Store the new temperature value
    return

; Delay Implementation
; Implement a delay routine for consistent RPM measurement intervals.
; This can be done using loop counters or timer modules of the PIC16F84A.
; Example (delay using software loop):
delay_ms:
    ; Implement a delay of a specified number of milliseconds here.
    ; Input: W contains the number of milliseconds to delay
    ; Example using a software loop:
    delay_loop:
        movlw 250           ; Delay for 1ms 
        call  delay_us      ; Call delay_us subroutine
        decfsz W, F         ; Decrement W and skip if zero
        goto  delay_loop    ; Repeat the delay
    return

delay_us:
    ; Implement a delay of a specified number of microseconds here.
    ; Input: W contains the number of microseconds to delay
    ; Example using a software loop :
    delay_us_loop:
        nop                ; Adjust NOPs for the desired delay
        nop
        nop
        decfsz W, F        ; Decrement W and skip if zero
        goto  delay_us_loop ; Repeat the delay
    return

;====================================================================
END



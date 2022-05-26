/*
 * Archivo: main_slave1.c
 * Dispositivo: PIC16F887
 * Compilador:  XC8, MPLABX v5.40
 * Autor: José Fernando de León González
 * Programa: Slave I del robot de pelea (control de servos) 
 * 
 * Hardware:  servomotores FUTABA S3003 en RC1 (miembro RP1) y RC2 (miembro LP1)
 *             
 * 
 * Creado: 25/05/22
 * Última modificación: 25/05/22
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * Constantes
------------------------------------------------------------------------------*/
#define _XTAL_FREQ 500000       // Oscilador de 500 kHz

/*------------------------------------------------------------------------------
 * Variables
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Interrupciones
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Ciclo principal
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Configuración
------------------------------------------------------------------------------*/
void setup(void){
    
    //Configuración SPI SLAVE de los puertos
    TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
    PORTC = 0;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b011;    // IRCF <2:0> 011 -> 500 kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
      
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // RC2 -> CCP1 como entrada
    TRISCbits.TRISC1 = 1;       // RC1 -> CCP2 como entrada
    PR2 = 156;                  // periodo de 20 ms
    
    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    
    CCP1CONbits.CCP1M = 0b1100; // Asignación de modo a PWM1
    CCP2CONbits.CCP2M = 0b1100; // Asignación de modo a PWM2
    
    CCPR1L = 155>>2;
    CCP1CONbits.DC1B = 155 & 0b11;    // Valor inicial del duty cycle PWM1
    
    CCPR2L = 155>>2;
    CCP2CONbits.DC2B0 = 155 & 0b01;
    CCP2CONbits.DC2B1 = 155 & 0b10;   //  Valor inicial del duty cycle PWM2
            
    
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un ciclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // RC2 -> CCP1 como salida del PWM2
    TRISCbits.TRISC1 = 0;       // RC1 -> CCP2 como salida del PWM2
    
    //Configuración del SPI (SLAVE)
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
        
    // Configuracion interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
   
}
/*------------------------------------------------------------------------------
 * Funciones
------------------------------------------------------------------------------*/

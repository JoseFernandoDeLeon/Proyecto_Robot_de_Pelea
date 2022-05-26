/*
 * Archivo: main_master.c
 * Dispositivo: PIC16F887
 * Compilador:  XC8, MPLABX v5.40
 * Autor: José Fernando de León González
 * Programa: Master del robot de pelea (Entradas analógicas y control de servos) 
 * 
 * Hardware:  potenciómetros de posicionamiento en RA0, RA1, RA2 y RA3, selector de esclavo en RA6 y RA7, 
 *            servomotores FUTABA S3003 en RC1 (miembro RA1) y RC2 (miembro LA1) botón MODO en RB0, 
 *            botones de acción y EDITAR/ACEPTAR en RB1 y RB2 & Joysticks en RA4 (Rueda derecha)
 *            y RA5 (rueda izquierda)
 * 
 * Creado: 25/05/22
 * Última modificación: 25/05/22
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 20               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 80              // Valor máximo de ancho de pulso de señal PWM

/*------------------------------------------------------------------------------
 * Variables
------------------------------------------------------------------------------*/
unsigned short CCPR_1 = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal
unsigned short CCPR_2 = 0;        // Variable para almacenar ancho de pulso al hacer la interpolación lineal

/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/
void setup (void);

unsigned short interpole(uint8_t value, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

void move_servo(uint8_t servo, unsigned short CCPR);

void send_data (uint8_t data, uint8_t slave);

/*------------------------------------------------------------------------------
 * Interrupciones
------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(PIR1bits.ADIF){                      // Fue interrupción del ADC?
        
        if(ADCON0bits.CHS == 0b0000){            // Verificamos sea AN0 el canal seleccionado
            move_servo (1,CCPR_1);
        }
        else if(ADCON0bits.CHS == 0b0001){
            PORTDbits.RD1 = 0;
            send_data(ADRESH,1);
        }
        else if (ADCON0bits.CHS == 0b0010){
            move_servo (2,CCPR_2);
        }
        else{
            PORTDbits.RD1 = 1;
            send_data(ADRESH,1);
        }
        PIR1bits.ADIF = 0;                  // Limpiamos bandera de interrupción
    }
    return;
}
/*------------------------------------------------------------------------------
 * Ciclo principal
------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if (ADCON0bits.GO == 0) {          // Verificar si se debe hacer una conversión
        if (ADCON0bits.CHS == 0b0000)       
            ADCON0bits.CHS = 0b0001;       // Convertir el valor del potenciómetro RP2
        
        else if (ADCON0bits.CHS == 0b0001)
            ADCON0bits.CHS = 0b0010;       // Convertir el valor del potenciómetro LA1
        
        else if (ADCON0bits.CHS == 0b0010)
            ADCON0bits.CHS = 0b0011;       // Convertir el valor del potenciómetro LP2
        
        else
            ADCON0bits.CHS = 0b0000;       // Convertir el valor del potenciómetro RA1 
        
        __delay_us(1000);
        ADCON0bits.GO = 1;
        
        } 
    }
    return;
}
/*------------------------------------------------------------------------------
 * Configuración
------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b11111111;        // PORTA y PORTE como entrada analógica
    ANSELH = 0;                // I/O digitales
    
    TRISA = 0b11111111;        // PORTA como entrada
    PORTA = 0;                 // Limpiamos PORTA  
            
    TRISD = 0b00000000;        // TRISD como salida
    PORTD = 0b00000000;        // RD0 habilitado (SS SLAVE I)
    
    //Configuración SPI MASTER de puertos
    TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
    PORTC = 0;
        
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b011;    // IRCF <2:0> 011 -> 500 kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
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
    
    // Configuración del SPI (MASTER)
    
    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
    SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
    SSPBUF = 0xFF;              // Enviamos un dato inicial
        
    // Configuracion interrupciones
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    
}
/*------------------------------------------------------------------------------
 * Funciones
------------------------------------------------------------------------------*/

/* Función para hacer la interpolación lineal del valor de la entrada analógica 
*  usando solo el registro ADRESH (8 bits) al ancho de pulso del PWM (10 bits), 
* usando la ecuación:
*  y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
*  -------------------------------------------------------------------
*  | x0 -> valor mínimo de ADC | y0 -> valor mínimo de ancho de pulso|
*  | x  -> valor actual de ADC | y  -> resultado de la interpolación | 
*  | x1 -> valor máximo de ADC | y1 -> valor máximo de ancho de puslo|
*  ------------------------------------------------------------------- 
*/
unsigned short interpole(uint8_t value, uint8_t in_min, uint8_t in_max, 
                         unsigned short out_min, unsigned short out_max){
    
    return (unsigned short)(out_min+((float)(out_max-out_min)/(in_max-in_min))*(value-in_min));
}

/*Función para posicionar un servomotor en base al valor de un potenciómetro*/
void move_servo(uint8_t servo,  unsigned short CCPR) {
        
    if (servo == 1){
        CCPR = interpole(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
        CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
        CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
    }
    else {
        CCPR = interpole(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
        CCPR2L = (uint8_t) (CCPR>>2);
        CCP2CONbits.DC2B0 = CCPR & 0b10;
        CCP2CONbits.DC2B1 = CCPR & 0b01;   
    }
    return;
}

void send_data (uint8_t data, uint8_t slave){
    
    if (slave == 1){
    PORTDbits.RD0 = 0;              // Habilitamos el esclavo para recibir datos
    SSPBUF = data;                  // Cargamos valor del contador al buffer
    while(!SSPSTATbits.BF){}        // Esperamos a que termine el envio
    PORTDbits.RD0 = 1;              // Deshabilitamos el esclavo hasta el siguiente envío.
    
    }
    return;
}


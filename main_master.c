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
#define _XTAL_FREQ 8000000      // Oscilador de 8 MHz

/*------------------------------------------------------------------------------
 * Variables
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Prototipos de funciones
------------------------------------------------------------------------------*/
void setup (void);

/*------------------------------------------------------------------------------
 * Interrupciones
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Ciclo principal
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Configuración
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Funciones
------------------------------------------------------------------------------*/
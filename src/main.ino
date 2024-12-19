#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lib/lcdarduino.h"
#include <util/delay.h>

/*
Pin-Zuordnung:
PB3 - MISO - in
PB2 - MOSI - out
PB1 - SCK - out
PB0 - Chip Select (CS) Input - out
PA0 - Chip Select (CS) Output - out
PA1 - Latch Ausgang - out
*/

// ADC-Daten
volatile int adcHighByte = 0;
volatile int adcLowByte = 0;

// Menü- und LCD-Steuerung
volatile unsigned char menuStatus = 1;
volatile unsigned char previousMenuStatus = 0;
char displayBuffer[16];
volatile char debounceFlag = 0;
volatile char buttonLock = 0;

// Filter-Parameter
volatile float previousOutput = 0;    // Vorheriger Filter-Ausgangswert
volatile float currentOutput = 0;     // Aktueller Filter-Ausgangswert
volatile float filterFrequency = 100;
volatile float filterGain = 1;
volatile float updatedFrequency = 100;
volatile float updatedGain = 1;
volatile float a0 = 0;
volatile float b1 = 0;

// SPI Interrupt Service Routine
ISR(SPI_STC_vect) {
}

// Timer0 Compare Match A Interrupt für den Abtastzyklus
ISR (TIMER0_COMPA_vect) {
    // Start der SPI-Kommunikation mit dem ADC
    PORTB &= ~(1 << PB0);  // Chip Select aktivieren
    SPDR = 0b00000110;     // ADC-Datenanforderung senden
    while (!(SPSR & (1 << SPIF)));

    // Weiteren Taktzyklus starten, um Daten zu empfangen
    SPDR = 0b00000000;
    while (!(SPSR & (1 << SPIF)));

    // Oberen 4 Bits des ADC-Wertes lesen
    adcHighByte = (SPDR) & ~(0xF0);
    SPDR = 0b00000000;  // Weitere Taktzyklen für die unteren Bits
    while (!(SPSR & (1 << SPIF)));

    // Unteren 8 Bits des ADC-Wertes lesen
    adcLowByte = SPDR;
    PORTB |= (1 << PB0);  // Chip Select deaktivieren

    // ADC-Daten zu einem Wert zusammenführen
    float currentInput = (adcHighByte << 8) | adcLowByte;

    // Filterberechnung für den Tiefpassfilter
    currentOutput = (b1 * currentInput + a0 * previousOutput);

    // Aktualisierung der vorherigen Ausgangswerte
    previousOutput = currentOutput;

    // Berechnen und Begrenzen des Ausgangswertes
    int outputSignal = updatedGain * (int)currentOutput - (updatedGain - 1) * 2048;

    if (outputSignal <= 0) { outputSignal = 0; }
    if (outputSignal >= 4095) { outputSignal = 4095; }

    // Übertragung des Ausgangswertes zum DAC
    PORTA &= ~(1 << PA0);  // DAC Chip Select aktivieren
    SPDR = (0b00010000) | ((outputSignal >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = outputSignal & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA0);  // DAC Chip Select deaktivieren

    // Latch-Ausgang setzen und zurücksetzen
    PORTA &= ~(1 << PA1);
    PORTA |= (1 << PA1);
}

void setup() {
    // Initialisierung der Ports
    DDRB = 0x07;
    DDRA = 0x03;

    // SPI-Konfiguration für Master-Modus
    SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR1);
    SPSR = 0;

    // Timer0-Konfiguration für 1 ms Abtastrate
    TCCR0A = 2 << WGM00;  // CTC Modus
    TCCR0B = 3 << CS00;   // Prescaler=64
    OCR0A = 249;          // Zählerwert für 1 ms
    TIMSK0 = 1 << OCIE0A; // Interrupt aktivieren

    // Globale Interrupts aktivieren
    sei();

    // Chip Select auf Ausgang high setzen
    PORTB |= (1 << PB0);
    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    PORTB &= ~(1 << PB2); // MOSI initialisieren

    // LCD initialisieren und Startwerte anzeigen
    lcd_init(LCD_DISP_ON_CURSOR);
    lcd_clrscr();
    sprintf(displayBuffer, "Frq: %d s-1", int(filterFrequency));
    lcd_puts(displayBuffer);
    sprintf(displayBuffer, "\nGain: %d", int(filterGain));
    lcd_puts(displayBuffer);
}

void loop() {
    lcd_home();

    // Tastenabfrage zur Steuerung von Frequenz und Gain
    char buttonPress = get_button();
    switch (buttonPress) {
        case button_up:
            if (buttonLock == 0) {
                filterGain++; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_down:
            if (buttonLock == 0) {
                filterGain--; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_left:
            if (buttonLock == 0) {
                filterFrequency--; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_right:
            if (buttonLock == 0) {
                filterFrequency++; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_ok:
            if (buttonLock == 0) {
                buttonLock = 1; previousMenuStatus = 2;
            } break;
        default: buttonLock = 0; break;
    }

    // LCD-Anzeige aktualisieren, wenn Menü geändert wurde
    if (menuStatus != previousMenuStatus) {
        if (previousMenuStatus == 0) {
            sprintf(displayBuffer, "Frq: %d s-1", int(filterFrequency));
            lcd_puts(displayBuffer);
            sprintf(displayBuffer, "\nGain: %d", int(filterGain));
            lcd_puts(displayBuffer);
            previousMenuStatus = 1;
        }

        // Aktualisierte Filterparameter anwenden
        if (previousMenuStatus == 2) {
            updatedFrequency = filterFrequency;
            updatedGain = filterGain;

            // Berechnen der neuen Tiefpass-Koeffizienten
            a0 = (1 / updatedFrequency) / (0.001 + 1 / updatedFrequency);
            b1 = (0.001) / (0.001 + 1 / updatedFrequency);

            previousOutput = currentOutput;
            previousMenuStatus = 1;
        }
    }
}

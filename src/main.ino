#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lib/lcdarduino.h"
#include <util/delay.h>

// Pin-Zuordnung
#define CS_EINGANG_PIN PB0
#define CS_AUSGANG_PIN PA0
#define LATCH_PIN PA1

// ADC-Daten
volatile int adcOberesByte = 0;
volatile int adcUnteresByte = 0;

// Menü- und LCD-Steuerung
volatile unsigned char menueStatus = 1;
volatile unsigned char vorherigerMenueStatus = 0;
char anzeigePuffer[16];
volatile char entprellFlagge = 0;
volatile char tastenSperre = 0;

// Filter-Parameter
volatile float vorherigerAusgang = 0;
volatile float aktuellerAusgang = 0;
volatile float filterFrequenz = 100;
volatile float filterVerstaerkung = 1;
volatile float aktualisierteFrequenz = 100;
volatile float aktualisierteVerstaerkung = 1;
volatile float a0 = 0;
volatile float b1 = 0;

// SPI Interrupt Service Routine
ISR(SPI_STC_vect) {}

// Timer0 Compare Match A Interrupt für den Abtastzyklus
ISR(TIMER0_COMPA_vect) {
    // Start der SPI-Kommunikation mit dem ADC
    PORTB &= ~(1 << CS_EINGANG_PIN);
    SPDR = 0b00000110;
    while (!(SPSR & (1 << SPIF)));

    // Weiteren Taktzyklus starten, um Daten zu empfangen
    SPDR = 0b00000000;
    while (!(SPSR & (1 << SPIF)));

    // Oberen 4 Bits des ADC-Wertes lesen
    adcOberesByte = (SPDR) & ~(0xF0);
    SPDR = 0b00000000;
    while (!(SPSR & (1 << SPIF)));

    // Unteren 8 Bits des ADC-Wertes lesen
    adcUnteresByte = SPDR;
    PORTB |= (1 << CS_EINGANG_PIN);

    // ADC-Daten zu einem Wert zusammenführen
    float aktuellerEingang = (adcOberesByte << 8) | adcUnteresByte;

    // Filterberechnung für den Tiefpassfilter
    aktuellerAusgang = (b1 * aktuellerEingang + a0 * vorherigerAusgang);
    vorherigerAusgang = aktuellerAusgang;

    // Berechnen und Begrenzen des Ausgangswertes
    int ausgangsSignal = aktualisierteVerstaerkung * (int)aktuellerAusgang - (aktualisierteVerstaerkung - 1) * 2048;
    ausgangsSignal = constrain(ausgangsSignal, 0, 4095);

    // Übertragung des Ausgangswertes zum DAC
    PORTA &= ~(1 << CS_AUSGANG_PIN);
    SPDR = (0b00110000) | ((ausgangsSignal >> 8) & 0x0F);
    while (!(SPSR & (1 << SPIF)));
    SPDR = ausgangsSignal & 0xFF;
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << CS_AUSGANG_PIN);

    // Latch-Ausgang setzen und zurücksetzen
    PORTA &= ~(1 << LATCH_PIN);
    PORTA |= (1 << LATCH_PIN);
}

void setup() {
    // Initialisierung der Ports
    DDRB = 0x07;
    DDRA = 0x03;

    // SPI-Konfiguration für Master-Modus
    SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR1);
    SPSR = 0;

    // Timer0-Konfiguration für 1 ms Abtastrate
    TCCR0A = 2 << WGM00;
    TCCR0B = 3 << CS00;
    OCR0A = 249;
    TIMSK0 = 1 << OCIE0A;

    // Globale Interrupts aktivieren
    sei();

    // Chip Select auf Ausgang high setzen
    PORTB |= (1 << CS_EINGANG_PIN);
    PORTA |= (1 << CS_AUSGANG_PIN);
    PORTA |= (1 << LATCH_PIN);
    PORTB &= ~(1 << PB2);

    // LCD initialisieren und Startwerte anzeigen
    lcd_init(LCD_DISP_ON_CURSOR);
    lcd_clrscr();
    sprintf(anzeigePuffer, "Frq: %d s-1", int(filterFrequenz));
    lcd_puts(anzeigePuffer);
    sprintf(anzeigePuffer, "\nGain: %d", int(filterVerstaerkung));
    lcd_puts(anzeigePuffer);

    // Initialisiere Filterparameter
    aktualisierteFrequenz = filterFrequenz;
    aktualisierteVerstaerkung = filterVerstaerkung;

    // Berechnen der neuen Tiefpass-Koeffizienten
    a0 = (1 / aktualisierteFrequenz) / (0.001 + 1 / aktualisierteFrequenz);
    b1 = (0.001) / (0.001 + 1 / aktualisierteFrequenz);

    vorherigerAusgang = aktuellerAusgang;
}
void loop() {
    lcd_home();

    // Tastenabfrage zur Steuerung von Frequenz und Verstärkung
    char tastenDruck = get_button();

    switch (tastenDruck) {
        case button_up:
            if (tastenSperre == 0) {
                filterVerstaerkung++;
                tastenSperre = 1;
                vorherigerMenueStatus = 0;
            }
            break;
        case button_down:
            if (tastenSperre == 0) {
                filterVerstaerkung--;
                tastenSperre = 1;
                vorherigerMenueStatus = 0;
            }
            break;
        case button_left:
            if (tastenSperre == 0) {
                filterFrequenz -= 10;
                tastenSperre = 1;
                vorherigerMenueStatus = 0;
            }
            break;
        case button_right:
            if (tastenSperre == 0) {
                filterFrequenz += 10;
                tastenSperre = 1;
                vorherigerMenueStatus = 0;
            }
            break;
        default:
            tastenSperre = 0;
            break;
    }

    // Aktualisierte Filterparameter anwenden
    aktualisierteFrequenz = filterFrequenz;
    aktualisierteVerstaerkung = filterVerstaerkung;

    // Berechnen der neuen Tiefpass-Koeffizienten
    a0 = (1 / aktualisierteFrequenz) / (0.001 + 1 / aktualisierteFrequenz);
    b1 = (0.001) / (0.001 + 1 / aktualisierteFrequenz);

    vorherigerAusgang = aktuellerAusgang;

    // LCD-Anzeige aktualisieren, wenn Menü geändert wurde
    if (menueStatus != vorherigerMenueStatus) {
        if (vorherigerMenueStatus == 0) {
            sprintf(anzeigePuffer, "Frq.:%06d s-1", int(filterFrequenz));
            lcd_puts(anzeigePuffer);
            sprintf(anzeigePuffer, "\nVerst.:%04d", int(filterVerstaerkung));
            lcd_puts(anzeigePuffer);
            vorherigerMenueStatus = 1;
        }
    }
}
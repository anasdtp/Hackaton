#include "timerAsserBas.h"
#include "CRAC_utility.h"

#include <avr/io.h>
#include <avr/interrupt.h>


ISR(TIMER1_COMPA_vect) {
  mscount++; // Incrémente le compteur de millisecondes à chaque interruption
}

void init_Timer() {
  TCCR1A = 0; // Clear TCCR1A register
  TCCR1B = (1 << WGM12) | (1 << CS11); // Set WGM12 for CTC mode, set prescaler to 8
  OCR1A = 199; // Set the compare value to achieve 10 kHz frequency
  TIMSK1 = (1 << OCIE1A); // Enable timer compare interrupt

  sei(); // Enable global interrupts
}
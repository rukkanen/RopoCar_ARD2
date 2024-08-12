// SerialRedirect.h
#ifndef SERIAL_REDIRECT_H
#define SERIAL_REDIRECT_H

#include <Arduino.h>

// Custom putc function to redirect stdout to Serial
int serial_putc(char c, FILE *file) {
    Serial.write(c);
    return c;
}

// Custom putc function to redirect stderr to Serial
int serial_puterr(char c, FILE *file) {
    Serial.write(c);
    return c;
}

// Function to set up the redirection
void setupSerialRedirect() {
    // Redirect stdout to Serial
    fdevopen(&serial_putc, NULL);

    // Redirect stderr to Serial
    fdevopen(&serial_puterr, NULL);
}

#endif // SERIAL_REDIRECT_H

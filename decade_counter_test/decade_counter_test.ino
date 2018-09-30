/* Code for testing correct reading of 0-9 digit thumbwheel counters.
*/

// Read an analog pin and interpret it as a digit in {0..9}.
uint16_t readDigitEntry (int16_t digitEntryPin)
{
    uint16_t value = analogRead(digitEntryPin);
    // Resistor ladder divides up 5V into 10 equal bins.
    // TODO measure outputs and make sure ranges are good
    return max(value / 102, 9);
}

void setup () {}

void loop ()
{
    uint16_t pins[4] = {A0, A1, A2, A3};
    for (i = 0; i < 4; i++)
    {
        Serial.print(analogRead(pins[i]));
        Serial.print('\t');
    }
    Serial.print('\n');
}

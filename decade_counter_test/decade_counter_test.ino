/* Code for testing correct reading of 0-9 digit thumbwheel counters.
*/

// The typical ADC value for each digit of the thumbwheel counters.
// uint16_t digitValues = {0, 118, 192, 272, 340, 397, 432, 474, 511, 543};
// Edges between bins for reading thumbwheels.
uint16_t binEdges[10] = {50, 155, 232, 305, 368, 414, 453, 492, 527, 0xFFFF};

// Read an analog pin and interpret it as a digit in {0..9}.
uint16_t readDigitEntry (int16_t digitEntryPin)
{
    uint16_t value = analogRead(digitEntryPin);
    for (uint8_t i = 0; i < 10; i++) {
        if (value < binEdges[i]) {
            return i;
        }
    }
    // Should be unreachable.
    return 9;
}

void setup ()
{
    Serial.begin(9600);
}
void loop ()
{
    uint16_t pins[4] = {A0, A1, A2, A3};
    for (int i = 0; i < 4; i++)
    {
        Serial.print(readDigitEntry(pins[i]));
        Serial.print('\t');
    }
    Serial.print('\n');
}

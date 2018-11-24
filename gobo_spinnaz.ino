/* WE GOBO SPINNAZ, WE GOBO SPINNAZ GOBO SPINNAZ (THEY DONT STOP)
*
* The hardware controls are four 0-9 digit thumbwheels, biased with
* a resistor ladder and read out using an analog voltage.
*
* On startup, loop until we have a valid state on the controls:
* - first digit is nonzero -> run in standalone
* - first digit is zero -> try to parse the last three digits as
*         a valid DMX address.  If not valid, continue looping.  If valid,
*         start running.
* Power cycle has to happen to change the mode or the DMX address.
*
* In standalone mode, each digit dial sets the speed of one motor.
* DMX mode: 8 channels (2 per motor), first channel is direction,
* second is speed.
*
*/
#include "DMXSerial.h"
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

// may change this to an external LED at some point
#define LED_PIN LED_BUILTIN

// Blink the LED on and off several times with some duration of flash.
void blink (uint8_t times, uint16_t duration)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(duration);
        digitalWrite(LED_PIN, LOW);
        delay(duration);
    }
}

// setting this to -1 runs in standalone
int16_t dmxAddress;

// Keep track of when we last got a DMX frame.
// Blink the LED if we have good DMX coming in.
// Stop blinking the LED if we haven't seen DMX in dmxTimeout ms.
const uint16_t dmxTimeout = 1000u;

// Small state machine that blinks the LED if we're getting good DMX.
bool ledOn = false;
uint32_t lastLedTransition = 0ul;
const uint16_t blinkDuration = 500;  // Nice, slow blinking.

// Make the LED blink if we have good DMX.
void serviceLedState ()
{
    uint32_t now = millis();
    uint32_t timeSinceLastPacket = DMXSerial.noDataSince();
    // If we haven't seen a frame in a while, shut LED off.
    if (timeSinceLastPacket > dmxTimeout)
    {
        digitalWrite(LED_PIN, LOW);
    }
    // Otherwise, possibly execute a state transition if it's been
    // a while since the last one.
    else if ((lastLedTransition + blinkDuration) < now)
    {
        // Toggle the state.
        ledOn = !ledOn;
        // Set the LED to this new state.
        digitalWrite(LED_PIN, ledOn);
        // Record that we just transitioned.
        lastLedTransition = now;
    }
}

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

// Correspondence between digit on {0..9} and motor speed for standalone.
// Generated as the series 255*tan(x/7)/tan(9/7).
// Roughly linear for the first 4-5 values, then curving upward.
uint8_t standaloneValues[10] = {0, 11, 22, 34, 48, 65, 86, 116, 163, 255};

// Get an 8-bit speed from reading a thumbwheel.
uint8_t speedFromDigit (int16_t pin)
{
    return standaloneValues[readDigitEntry(pin)];
}

// Create the motor shield object with the default I2C address
Adafruit_MotorShield motorShield = Adafruit_MotorShield();

// Four motors, motor selection is indexed from 1.
Adafruit_DCMotor *motor0 = motorShield.getMotor(1);
Adafruit_DCMotor *motor1 = motorShield.getMotor(2);
Adafruit_DCMotor *motor2 = motorShield.getMotor(3);
Adafruit_DCMotor *motor3 = motorShield.getMotor(4);

// Set speed and direction for a given motor.
void setMotorState (Adafruit_DCMotor* motor, uint8_t speed, uint8_t direction)
{
    motor->setSpeed(speed);
    motor->run(direction);
}

// Given a channel offset, set motor state from DMX values.
void setMotorStateFromDmx (Adafruit_DCMotor* motor, uint16_t channelOffset)
{
    uint16_t startChannel = dmxAddress + channelOffset;
    // Speed on second channel.
    uint8_t speed = DMXSerial.read(startChannel + 1);
    // First channel is direction, forward if less than 127.
    if (DMXSerial.read(startChannel) < 128)
    {
        setMotorState(motor, speed, FORWARD);
    }
    else
    {
        setMotorState(motor, speed, BACKWARD);
    }
}

void setup ()
{

    pinMode(LED_PIN, OUTPUT);

    // configure as DMX or standalone
    bool configured = false;

    while (!configured) {

        uint16_t select0Digit = readDigitEntry(A0);
        if (select0Digit > 0)
        {
            // running in standalone mode, set address to -1
            dmxAddress = -1;
            // blink the LED 3 times slowly
            blink(3, 500);
            configured = true;
        }
        // not standalone, check if we have a valid DMX address
        else
        {
            uint16_t readAddress =
                (100 * readDigitEntry(A1))
                + (10 * readDigitEntry(A2))
                + readDigitEntry(A3);

            if ((readAddress > 0) && (readAddress < 506))
            {
                // valid address, set it and move on
                dmxAddress = readAddress;
                // blink the LED 6 times quickly
                blink(6, 250);
                configured = true;
            }
        }
        if (!configured)
        {
            // furiously blink the LED to indicate addressing failure
            blink(50, 20);
        }
    }

    // configure the motor shield and release the motors
    motorShield.begin();  // create with the default frequency 1.6KHz
    //motorShield.begin(1000);  // OR with a different frequency, say 1KHz

    setMotorState(motor0, 0, FORWARD);
    setMotorState(motor1, 0, FORWARD);
    setMotorState(motor2, 0, FORWARD);
    setMotorState(motor3, 0, FORWARD);

    motor0->run(RELEASE);
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);

    // if we're not in standalone, configure DMX
    if (dmxAddress > 0)
    {
        DMXSerial.maxChannel(512);
        DMXSerial.init(DMXReceiver);
    }
}

void loop ()
{
    // If we're in standalone mode, read the control values and set state.
    if (-1 == dmxAddress)
    {
        setMotorState(motor0, speedFromDigit(A0), FORWARD);
        setMotorState(motor1, speedFromDigit(A1), FORWARD);
        setMotorState(motor2, speedFromDigit(A2), FORWARD);
        setMotorState(motor3, speedFromDigit(A3), FORWARD);
    }
    // If we're running in DMX mode, read values and update LED indicator.
    else
    {
        setMotorStateFromDmx(motor0, 0);
        setMotorStateFromDmx(motor1, 2);
        setMotorStateFromDmx(motor2, 4);
        setMotorStateFromDmx(motor3, 6);
        serviceLedState();
    }

    // Run at ~50 fps.
    delay(20);
}

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
*
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

/// Analog pins used to read the four digit entry thumbwheels.
int16_t thumbwheelPins[4] = {A0, A1, A2, A3};

// The typical ADC value for each digit of the thumbwheel counters.
// uint16_t digitValues = {0, 118, 192, 272, 340, 397, 432, 474, 511, 543};

// Edges between bins for reading thumbwheels.
uint16_t binEdges[10] = {50, 155, 232, 305, 368, 414, 453, 492, 527, 0xFFFF};

// Read thumbwheel i and interpret it as a digit in {0..9}.
uint8_t readThumbwheel (uint8_t thumbwheel)
{
    uint16_t value = analogRead(thumbwheelPins[thumbwheel]);
    for (uint8_t i = 0; i < 10; i++)
    {
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
uint8_t standaloneSpeed[10] = {0, 11, 22, 34, 48, 65, 86, 116, 163, 255};

// Create the motor shield object with the default I2C address
Adafruit_MotorShield motorShield = Adafruit_MotorShield();

// Four motors, motor selection is indexed from 1.
Adafruit_DCMotor *motors[4] = {
    motorShield.getMotor(1),
    motorShield.getMotor(2),
    motorShield.getMotor(3),
    motorShield.getMotor(4),
};

// Given a motor index, set motor state by reading a thumbwheel.
// Run in "twinspin" mode where even motors run forward and odd motors run
// backward.
void setMotorStateStandalone (uint8_t motorIndex)
{
    Adafruit_DCMotor *motor = motors[motorIndex];
    uint8_t thumbwheelSetting = readThumbwheel(motorIndex);
    uint8_t speed = standaloneSpeed[thumbwheelSetting];
    motor->setSpeed(speed);
    if (motorIndex % 2 == 0)
    {
        motor->run(FORWARD);
    }
    else
    {
        motor->run(BACKWARD);
    }
}

// Given a channel offset, set motor state from DMX values.
void setMotorStateFromDmx (Adafruit_DCMotor* motor, uint16_t channelOffset)
{
    uint16_t startChannel = dmxAddress + channelOffset;

    bool forward = DMXSerial.read(startChannel) < 128;
    uint8_t speed = DMXSerial.read(startChannel + 1);
    motor->setSpeed(speed);

    // First channel is direction, forward if less than 127.
    if (forward)
    {
        motor->run(FORWARD);
    }
    else
    {
        motor->run(BACKWARD);
    }
}

void setup ()
{

    pinMode(LED_PIN, OUTPUT);

    // configure as DMX or standalone
    bool configured = false;

    while (!configured) {

        if (readThumbwheel(0) > 0)
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
                (100 * readThumbwheel(1))
                + (10 * readThumbwheel(2))
                + readThumbwheel(3);

            if ((readAddress > 0) && (readAddress < 506))
            {
                dmxAddress = readAddress;

                // Configure DMX receiver.
                DMXSerial.maxChannel(512);
                DMXSerial.init(DMXReceiver);

                // Blink the LED 6 times quickly to indicate DMX mode success.
                blink(6, 250);
                configured = true;
            }
        }
        if (!configured)
        {
            // Furiously blink the LED to indicate addressing failure.
            blink(10, 20);
        }
    }

    motorShield.begin();

    for (uint8_t i = 0; i < 4; i++)
    {
        Adafruit_DCMotor *motor = motors[i];
        motor->setSpeed(0);
    }
}

void loop ()
{
    // If we're in standalone mode, read new control values and set state for
    // each motor.
    if (-1 == dmxAddress)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            setMotorStateStandalone(i);
        }
    }
    // If we're running in DMX mode, read values and update LED indicator.
    else
    {
        for (uint8_t i = 0; i < 4; i++) {
            setMotorStateFromDmx(motors[i], i*2);
        }
        serviceLedState();
    }

    // Run at ~50 fps.
    delay(20);
}

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
#include "Conceptinetics/Conceptinetics.h"
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "Types.h"

//
// CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD JUMPER INSTRUCTIONS
//
// If you are using the above mentioned shield you should
// place the RXEN jumper towards G (Ground), This will turn
// the shield into read mode without using up an IO pin
//
// The !EN Jumper should be either placed in the G (GROUND)
// position to enable the shield circuitry
//   OR
// if one of the pins is selected the selected pin should be
// set to OUTPUT mode and set to LOGIC LOW in order for the
// shield to work
//

#define MOTOR_COUNT 4
#define CHAN_PER_MOTOR 2
#define DMX_CHANNEL_COUNT (MOTOR_COUNT * CHAN_PER_MOTOR)

// may change this to an external LED at some point
#define LED_PIN LED_BUILTIN

// Blink the LED on and off several times with some duration of flash.
void blink(int times, int duration) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(duration);
        digitalWrite(LED_PIN, LOW);
        delay(duration);
    }
}

// DMX slave receiver
DMX_Slave dmxSlave(DMX_CHANNEL_COUNT);

// setting this to -1 runs in standalone
int dmxAddress;

// Read an analog pin and interpret it as a digit in {0..9}.
int readDigitEntry(int digitEntryPin) {
    int value = analogRead(digitEntryPin);
    // Resistor ladder divides up 5V into 10 equal bins.
    // TODO measure outputs and make sure ranges are good
    return max(value / 102, 9);
}

// Correspondence between digit on {0..9} and motor speed for standalone.
// Generated as the series 255*tan(x/7)/tan(9/7).
// Roughly linear for the first 4-5 values, then curving upward.
uint8_t standaloneValues[10] = {0, 11, 22, 34, 48, 65, 86, 116, 163, 255};

// Get an 8-bit speed from reading a thumbwheel.
uint8_t speedFromDigit(int pin) {
    return standaloneValues[readDigitEntry(pin)];
}

// stored state of each motor
MotorState motor0State = MotorState {FORWARD, 0};
MotorState motor1State = MotorState {FORWARD, 0};
MotorState motor2State = MotorState {FORWARD, 0};
MotorState motor3State = MotorState {FORWARD, 0};

// Create the motor shield object with the default I2C address
Adafruit_MotorShield motorShield = Adafruit_MotorShield();

// Four motors, motor selection is indexed from 1.
Adafruit_DCMotor *motor0 = motorShield.getMotor(1);
Adafruit_DCMotor *motor1 = motorShield.getMotor(2);
Adafruit_DCMotor *motor2 = motorShield.getMotor(3);
Adafruit_DCMotor *motor3 = motorShield.getMotor(4);

// Use a MotorState to set the state of a motor.
void pushMotorState(Adafruit_DCMotor* motor, MotorState motorState) {
    motor->setSpeed(motorState.speed);
    motor->run(motorState.direction);
}

// Push state to all of the motors.
void pushMotorStates() {
    pushMotorState(motor0, motor0State);
    pushMotorState(motor1, motor1State);
    pushMotorState(motor2, motor2State);
    pushMotorState(motor3, motor3State);
}

void setup() {

    pinMode(LED_PIN, OUTPUT);

    // configure as DMX or standalone
    while (true) {
        int select0Digit = readDigitEntry(A0);
        if (select0Digit > 0) {
            // running in standalone mode, set address to -1
            dmxAddress = -1;
            // blink the LED 3 times slowly
            blink(3, 500);
            break;
        }
        // not standalone, check if we have a valid DMX address
        int readAddress = readDigitEntry(A1) * 100 + readDigitEntry(A2) * 10 + readDigitEntry(A3);
        if (readAddress > 0 && readAddress < 512) {
            // valid address, set it and move on
            dmxAddress = readAddress;
            // blink the LED 6 times quickly
            blink(6, 250);
            break;
        }
        // furiously blink the LED to indicate addressing failure
        blink(50, 20);
        // loop and try again
    }

    // configure the motor shield and release the motors
    motorShield.begin();  // create with the default frequency 1.6KHz
    //motorShield.begin(1000);  // OR with a different frequency, say 1KHz

    pushMotorStates();

    motor0 ->run(RELEASE);
    motor1 ->run(RELEASE);
    motor2 ->run(RELEASE);
    motor3 ->run(RELEASE);

    // if we're not in standalone, configure DMX
    if (dmxAddress > 0) {
        // Enable DMX slave interface and start listening to DMX data.
        dmxSlave.enable();
        dmxSlave.setStartAddress(dmxAddress);
        // Register the interrupt handler to run on frame completion.
        dmxSlave.onReceiveComplete(handleDmxFrame);
    }

}

void loop() {
    // If we're in standalone mode, read the control values.
    if (dmxAddress == -1) {
        motor0State.speed = speedFromDigit(A0);
        motor1State.speed = speedFromDigit(A1);
        motor2State.speed = speedFromDigit(A2);
        motor3State.speed = speedFromDigit(A3);
    }
    // DMX mode sets the motor states in an interrupt.

    // Render the motor states to the motor controller.
    pushMotorStates();

    // Run at ~50 fps.
    delay(20);
}

// Given a channel offset, set the motor state from DMX values.
void setMotorStateFromDmx(int channelOffset, MotorState* motorState) {
    // First channel is direction, forward if less than 127.
    if (dmxSlave.getChannelValue(channelOffset + 1) > 127) {
        motorState->direction = BACKWARD;
    }
    else {
        motorState->direction = FORWARD;
    }
    // Second channel is speed.
    motorState->speed = dmxSlave.getChannelValue(channelOffset + 2);
}

// Interrupt handler for when the DMX interface has read a frame.
void handleDmxFrame(unsigned short channelsReceived) {
    if (channelsReceived == DMX_CHANNEL_COUNT) {
        // All channels have been received, update the stored motor state.
        setMotorStateFromDmx(0, &motor0State);
        setMotorStateFromDmx(1, &motor1State);
        setMotorStateFromDmx(2, &motor2State);
        setMotorStateFromDmx(3, &motor3State);
    }
    else {
        // We have received a frame but not all channels we where
        // waiting for, master might have transmitted less
        // channels.  Ignore.
    }
}


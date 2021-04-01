package org.firstinspires.ftc.teamcode.gamepad;

/**
 * A strategy is a method of interacting and retrieving specific button and joystick activities from
 * a gamepad over time as they are manipulated by human players.
 * A boolean strategy works with a changing boolean value to return another separate boolean value.
 */
interface BooleanStrategy {
    void update(boolean next);
}

/**
 * A strategy is a method of interacting and retrieving specific button and joystick activities from
 * a gamepad over time as they are manipulated by human players.
 * A float strategy works off a changing floating point value and returns another separate floating point value.
 */
interface FloatStrategy {
    void update(float next);
}

/**
 * A simple boolean strategy where the implementation simply returns a boolean describing state.
 */
interface RawBooleanStrategy extends BooleanStrategy {
    boolean read();
}

/**
 * A simple float strategy where the implementation simply returns a float describing state.
 */
interface RawFloatStrategy extends FloatStrategy {
    float read();
}

/**
 * PolledStrategy describes a implementation where a poll/peek system allows for mainloop style
 * code to receive and discard event returns from the TimedGamepad.
 * <p>
 * Polling a value causes it to reset to a 'default' value.
 * Peeking a value does not affect the value.
 * <p>
 * This system is intended for use in main loop implementations where events must be detected once.
 */
interface PolledStrategy extends BooleanStrategy {
    /**
     * Polling a strategy causes the next return to change, even if the state has not changed.
     *
     * @return Poll the current state.
     */
    boolean poll();

    /**
     * Return does NOT change unless state also changes. Ignores polling state entirely.
     *
     * @return Peek the current state.
     */
    boolean peek();
}

/**
 * ListenedEventStrategy is for solving a specific type of issue that comes up with manipulating
 * hardware from multiple sources.
 * Normally, one might use something like RawBooleanStrategy where you only have a boolean output
 * and no idea if it has changed or not. This causes issues when using things like setPower or
 * setPosition on servos. If you simply set the position every tick, only one setPosition works at a
 * time for the entire program. Using a event-listening based strategy works better and allows multiple
 * sources to manipulate and use a single piece of hardware during a program.
 *
 * When a change in state is seen, it is picked up in the program by constantly listening to the
 * changed() function's return, which is really just a getter. When changed() returns true,
 * the program knows the output of read() will have changed from last time, or at the very least,
 * be necessary as internal state has changed enough to warrant another activation of state.
 * One can still use read() regardless, which is why ListenedEventStrategy is a interface extension
 * on RawBooleanStrategy.
 */
interface ListenedEventStrategy extends RawBooleanStrategy {
    boolean changed();
}
package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

// TODO: DelayedScroll - One update before continuous updates a second or two after holding down.

/**
 * A ftc.firstinspires.Gamepad wrapper that assists with carefully controlling interactions upon the
 * Gamepad over time in a smart and expandable way.
 */
public class TimedGamepad {
    // Gamepad
    private Gamepad gamepad;

    // General
    public InstallableFloat left_stick_x;
    public InstallableFloat left_stick_y;
    public InstallableFloat right_stick_x;
    public InstallableFloat right_stick_y;
    public InstallableFloat left_trigger;
    public InstallableFloat right_trigger;
    public InstallableBoolean dpad_up;
    public InstallableBoolean dpad_down;
    public InstallableBoolean dpad_left;
    public InstallableBoolean dpad_right;
    public InstallableBoolean left_bumper;
    public InstallableBoolean right_bumper;

    // Xbox controller
    public InstallableBoolean x;
    public InstallableBoolean y;
    public InstallableBoolean a;
    public InstallableBoolean b;
    public InstallableBoolean back;
    public InstallableBoolean start;
    public InstallableBoolean guide;

    // Playstation controller
    public InstallableBoolean square; // Alike to Xbox button X
    public InstallableBoolean triangle; // Alike to Xbox button Y
    public InstallableBoolean cross; // Alike to Xbox button A
    public InstallableBoolean circle; // Alike to Xbox button B
    public InstallableBoolean share; // Alike to Xbox button Back
    public InstallableBoolean options; // Alike to Xbox button Start
    public InstallableBoolean ps; // Alike to Xbox button Guide
    public InstallableBoolean touchpad;

    // Installable lists for iterating - only contains non-duplicate instances
    public InstallableFloat[] installableFloats;
    public InstallableBoolean[] installableBooleans;

    TimedGamepad() {
        left_stick_x = new InstallableFloat();
        left_stick_y = new InstallableFloat();
        right_stick_x = new InstallableFloat();
        right_stick_y = new InstallableFloat();

        left_trigger = new InstallableFloat();
        right_trigger = new InstallableFloat();

        dpad_up = new InstallableBoolean();
        dpad_down = new InstallableBoolean();
        dpad_left = new InstallableBoolean();
        dpad_right = new InstallableBoolean();

        left_bumper = new InstallableBoolean();
        right_bumper = new InstallableBoolean();

        x = new InstallableBoolean();
        y = new InstallableBoolean();
        a = new InstallableBoolean();
        b = new InstallableBoolean();
        back = new InstallableBoolean();
        start = new InstallableBoolean();
        guide = new InstallableBoolean();

        square = x;
        triangle = y;
        cross = a;
        circle = b;
        share = back;
        options = start;
        ps = guide;

        touchpad = new InstallableBoolean();

        installableFloats = new InstallableFloat[]{left_stick_x, left_stick_y, right_stick_x, right_stick_y,
                left_trigger, right_trigger};
        installableBooleans = new InstallableBoolean[]{dpad_up, dpad_down, dpad_left, dpad_right,
                x, y, a, b, back, start, guide, touchpad};
    }

    public void tick() {
        left_stick_x.update(gamepad.left_stick_x);
        left_stick_y.update(gamepad.left_stick_y);
        right_stick_x.update(gamepad.right_stick_x);
        right_stick_y.update(gamepad.right_stick_y);

        left_trigger.update(gamepad.left_trigger);
        right_trigger.update(gamepad.right_trigger);

        dpad_up.update(gamepad.dpad_up);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
        dpad_right.update(gamepad.dpad_right);

        left_bumper.update(gamepad.left_bumper);
        right_bumper.update(gamepad.right_bumper);

        x.update(gamepad.x);
        y.update(gamepad.y);
        a.update(gamepad.a);
        b.update(gamepad.b);
        back.update(gamepad.back);
        start.update(gamepad.start);
        guide.update(gamepad.guide);

        touchpad.update(gamepad.touchpad);
    }
}

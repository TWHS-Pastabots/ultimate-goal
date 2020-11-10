package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A class to help with the use of a controlled drive train. This allows a {@link Gamepad} to be
 * added to a drive train. The main use of this is to allow the reuse of generic code across
 * {@link OpMode}'s.
 *
 * @see DriveTrain
 * @author Matt Provost
 */
public class ControlledDriveTrain extends DriveTrain {
    private Gamepad gamepad;

    // If the smoothed move method should be used
    private boolean smoothed = false;
    // If telemetry should be added
    private boolean addTelemetry = false;

    /**
     * Set the gamepad to use for inputs. This will provide the drive train with the gamepad that
     * will be used to poll inputs from. It should probably be one of the gamepads from the
     * {@link OpMode} class (i.e. one of {@link OpMode#gamepad1} or {@link OpMode#gamepad2}). If
     * this is set to a null gamepad, a {@link NullPointerException} is thrown.
     *
     * @param gamepad the gamepad to use for input
     * @throws NullPointerException if the given gamepad is null
     */
    public void setGamepad(Gamepad gamepad) {
        if (gamepad == null) {
            throw new NullPointerException("The gamepad cannot be null");
        }

        this.gamepad = gamepad;
    }

    /**
     * Enable telemetry. This will cause telemetry to be added for each loop for the inputs. This
     * just causes the {@literal addTelemetry} parameter of the
     * {@link #move(double, double, double, boolean) move} method to be set to true.
     *
     * @see #dontAddTelemetry()
     * @see #toggleTelemetry()
     * @see #setAddTelemetry(boolean)
     */
    public void addTelemetry() {
        addTelemetry = true;
    }

    /**
     * Disable telemetry. This just causes the {@literal addTelemetry} parameter of the
     * {@link #move(double, double, double, boolean) move} method to be set to true.
     *
     * @see #addTelemetry()
     * @see #toggleTelemetry()
     * @see #setAddTelemetry(boolean)
     */
    public void dontAddTelemetry() {
        addTelemetry = false;
    }

    /**
     * Toggle telemetry. This will toggle the internal variable for adding telemetry. See the other
     * methods to get a better understanding of what this does.
     *
     * @see #addTelemetry()
     * @see #dontAddTelemetry()
     * @see #setAddTelemetry(boolean)
     */
    public void toggleTelemetry() {
        addTelemetry = !addTelemetry;
    }

    /**
     * Set whether or not telemetry should be added.This just causes the {@literal addTelemetry}
     * parameter of the {@link #move(double, double, double, boolean) move} method to be set to a
     * specific value.
     *
     * @param addTelemetry whether or not telemetry should be added
     * @see #addTelemetry()
     * @see #dontAddTelemetry()
     * @see #toggleTelemetry()
     */
    public void setAddTelemetry(boolean addTelemetry) {
        this.addTelemetry = addTelemetry;
    }

    /**
     * Set whether or not the input should be smoothed. This will change the method being called
     * between {@link #move(double, double, double, boolean) move} and
     * {@link #smoothMove(double, double, double, boolean) smoothMove}.
     *
     * @param smoothed whether or not smoothing should be applied
     */
    public void setSmoothed(boolean smoothed) {
        this.smoothed = smoothed;
    }

    /**
     * Perform movement. This will get the inputs for the gamepad sticks and move the drive train
     * using those inputs. Depending on if smoothing has been enabled through
     * {@link #setSmoothed(boolean) setSmoothed}, it will choose between the
     * {@link #move(double, double, double, boolean) move} and
     * {@link #smoothMove(double, double, double, boolean) smoothMove} methods.
     *
     * The default controls are the left stick for movement and the right stick for turning.
     *
     * @throws NullPointerException if the gamepad has not been set
     */
    public void loop() {
        // Check if the gamepad is null
        if (gamepad == null) {
            throw new NullPointerException("The gamepad has not been set");
        }

        // Get the horizontal and vertical inputs
        double horizontal = gamepad.left_stick_x;
        double vertical = gamepad.left_stick_y;

        // Get the turn angle input
        double turnAngle = gamepad.right_stick_x;

        // Call the correct method depending on if smoothing should be applied
        if (smoothed) {
            this.smoothMove(horizontal, vertical, turnAngle, addTelemetry);
        } else {
            this.move(horizontal, vertical, turnAngle, addTelemetry);
        }
    }
}

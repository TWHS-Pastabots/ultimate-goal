package org.firstinspires.ftc.team16910.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * Contains utility functions related to opmodes.
 * <p>
 * There are certain things that are used repetitively regarding opmodes, so this
 * contains a number of useful functions implementing those things. For example,
 * there are methods to safely sleep for a certain amount of time, etc.
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
public class OpmodeUtil {
    /**
     * Safely sleeps for a certain amount of time.
     * <p>
     * Internally, {@link #sleep(double, LinearOpMode, Supplier)} is called with a function that
     * simply returns false. This makes the loop simply run as long as the {@code maxTimeout}
     * and break after that, or when it should to comply with the changing opmode state.
     *
     * @param maxTimeout the maximum number of seconds to wait for this task to finish
     * @param opMode     the opmode that is calling this
     * @see #sleep(double, LinearOpMode, Supplier) sleep
     */
    public static void sleep(double maxTimeout, LinearOpMode opMode) {
        sleep(maxTimeout, opMode, () -> false);
    }

    /**
     * Safely sleeps for a certain amount of time.
     * <p>
     * Usually timed loops and sleep functions aren't safe to use in linear opmodes. The reason
     * is that it expects the opmode to immediately react when the state changes, i.e. when
     * the start or stop button is pressed. It is for this reason that traditional {@code sleep}
     * methods aren't extremely safe in a linear opmode.
     * <p>
     * A timer is used to track if the {@code maxTimeout} has passed. If it has, the loop will break.
     * Also, if {@link LinearOpMode#opModeIsActive()} returns false, the loop will break.
     * <p>
     * The {@code func} can be used to break the loop prematurely. This allows for functions to be created
     * that only run within a maximum timeout but can still break prematurely. For example,
     * {@link MotorUtil#waitToSpinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) MotorUtil.waitToSpinTo}
     * uses this functionality.
     *
     * @param maxTimeout the maximum number of seconds to wait for this task to finish
     * @param opMode     the opmode that is calling this
     * @param func       the function to run
     * @see LinearOpMode#opModeIsActive()
     */
    public static void sleep(double maxTimeout, LinearOpMode opMode, Supplier<Boolean> func) {
        // Keep track of how long has passed
        ElapsedTime time = new ElapsedTime();

        // Check if the opmode is still running and the timeout has not been reached
        while (opMode.opModeIsActive() && time.seconds() < maxTimeout) {
            // Check if the function wants the loop to break
            if (func.get()) {
                break;
            }
        }
    }
}

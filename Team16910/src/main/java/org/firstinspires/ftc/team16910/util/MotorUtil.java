package org.firstinspires.ftc.team16910.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * Contains utility functions related to {@link DcMotorEx}.
 * <p>
 * There are certain things that are used repetitively regarding motors, so this
 * contains a number of useful functions implementing those things. For example,
 * there are methods to convert motor velocities to and from power levels, spin motors
 * to specific velocities, etc.
 * <p>
 * A few constants are exposed as static fields. They are defined as so to allow for configuration
 * through the dashboard, provided by the {@link Config} annotation.
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
@Config
public class MotorUtil {
    // Constants related to the motors we're using
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;

    /**
     * Converts the motor power level to encoder ticks per second.
     * <p>
     * This calls {@link #fromMotorPower(double, double, double)} with default parameters
     * that are common for most motors. This uses {@link #MOTOR_MAX_RPM} for {@code maxRpm}
     * and {@link #MOTOR_TICKS_PER_SECOND} for {@code ticksPerSecond}.
     *
     * @param power the power level as a decimal
     * @return motor ticks per second
     * @see #fromMotorPower(double, double, double) fromMotorPower
     * @see #toMotorPower(double) toMotorPower
     */
    public static double fromMotorPower(double power) {
        return fromMotorPower(power, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * Converts the motor power level to encoder ticks per second. Takes a {@code [-1, 1]} range power level
     * and manipulates its numeric value to convert it into a velocity that can be used by
     * <code>{@link DcMotorEx#setVelocity(double)}</code>.
     * <p>
     * What actually happens is the maximum motor RPM from {@code maxRpm} is converted into
     * the max rotations per second by dividing it by {@code 60}. Then it is multiplied by the power level specified
     * to get the target number of rotations per second and then multiplied by {@code ticksPerSecond}
     * to convert it to the number of ticks per second used by the motor.
     *
     * @param power          the power level as a decimal
     * @param maxRpm         the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     * @return motor ticks per second
     * @see #toMotorPower(double, double, double) toMotorPower
     */
    public static double fromMotorPower(double power, double maxRpm, double ticksPerSecond) {
        return maxRpm / 60 * power * ticksPerSecond;
    }

    /**
     * Converts the motor velocity in ticks per second to a power level.
     * <p>
     * This calls {@link #toMotorPower(double, double, double)} with default parameters
     * that are common for most motors. This uses {@link #MOTOR_MAX_RPM} for {@code maxRpm}
     * and {@link #MOTOR_TICKS_PER_SECOND} for {@code ticksPerSecond}.
     *
     * @param ticks the velocity as ticks per second
     * @return motor power level
     * @see #toMotorPower(double, double, double) toMotorPower
     * @see #fromMotorPower(double) fromMotorPower
     */
    public static double toMotorPower(double ticks) {
        return toMotorPower(ticks, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * Converts the motor velocity in ticks per second to a power level. Takes a velocity
     * from the motor, most likely from {@link DcMotorEx#getVelocity()}, and manipulates its numeric
     * value to convert it into a value in the range of {@code [-1, 1]}.
     * <p>
     * What actually happens is {@code ticks} are divided by {@code ticksPerSecond} to get the rotations
     * per second. It is then divided by the motor's maximum rotations per second, which is calculated
     * by dividing {@code maxRpm} by {@code 60}.
     *
     * @param ticks          the velocity as ticks per second
     * @param maxRpm         the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     * @return motor power level
     * @see #fromMotorPower(double, double, double) fromMotorPower
     */
    public static double toMotorPower(double ticks, double maxRpm, double ticksPerSecond) {
        return ticks / ticksPerSecond / (maxRpm / 60);
    }

    /**
     * Spins a motor to a velocity within a threshold within a certain amount of time.
     * <p>
     * Internally, {@link #spinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double)}
     * is called with default parameters that are common for most motors. This uses {@link #MOTOR_MAX_RPM}
     * for {@code maxRpm} and {@link #MOTOR_TICKS_PER_SECOND} for {@code ticksPerSecond}.
     *
     * @param motor         the motor to spin
     * @param power         the power level to spin to
     * @param threshold     the percent error that is acceptable
     * @param maxTimeout    the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode        the opmode that is spinning it
     * @see #spinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) spinTo
     */
    public static void spinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode) {
        spinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * Spins a motor to a velocity within a threshold within a certain amount of time.
     * <p>
     * The {@code motor} velocity is set through {@link DcMotorEx#setVelocity(double)}, by converting the
     * {@code power}, {@code maxRpm}, and {@code ticksPerSecond} into a velocity using
     * {@link #fromMotorPower(double, double, double) fromMotorPower}. After that, the current thread
     * is blocked until the motor is spinning at the correct speed by calling
     * {@link #waitToSpinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) waitToSpinTo}.
     *
     * @param motor          the motor to spin
     * @param power          the power level to spin to
     * @param threshold      the percent error that is acceptable
     * @param maxTimeout     the maximum amount of time to spend
     * @param stableTimeout  the amount of time for the speed to be considered stable
     * @param opMode         the opmode that is spinning it
     * @param maxRpm         the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     */
    public static void spinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode, double maxRpm, double ticksPerSecond) {
        motor.setVelocity(fromMotorPower(power, maxRpm, ticksPerSecond));
        waitToSpinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, maxRpm, ticksPerSecond);
    }

    /**
     * Blocks the current thread until the {@code motor} is at the correct velocity.
     * <p>
     * Internally calls
     * {@link #waitToSpinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) waitToSpinTo}
     * with default parameters common to most motors. This uses {@link #MOTOR_MAX_RPM} for {@code maxRpm} and
     * {@link #MOTOR_TICKS_PER_SECOND} for {@code ticksPerSecond}.
     * <p>
     * Note that the {@code motor} velocity is not set. The {@code motor} is only used to get the current
     * velocity. To set the velocity as well, use
     * {@link #spinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) spinTo} instead.
     *
     * @param motor         the motor to spin
     * @param power         the power level to spin to
     * @param threshold     the percent error that is acceptable
     * @param maxTimeout    the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode        the opmode that is spinning it
     * @see #waitToSpinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) waitToSpinTo
     */
    public static void waitToSpinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode) {
        waitToSpinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * Blocks the current thread until the {@code motor} is at the correct velocity.
     * <p>
     * The current thread is blocked until the {@code motor} achieves the target velocity. The blocking is
     * achieved by using {@link OpmodeUtil#sleep(double, LinearOpMode, Supplier)} to safely block the thread
     * while the velocity isn't within the target range. The target velocity is calculated by passing
     * {@code power}, {@code maxRpm}, and {@code ticksPerSecond} to
     * {@link #fromMotorPower(double, double, double) fromMotorPower}.
     * <p>
     * The check to continue waiting uses the {@code threshold}, {@code stableTimeout}, and a timer. First,
     * each loop the error is calculated and compared to the {@code threshold}. If the error is within the threshold,
     * the {@code stableTimeout} is checked. If the error has been within the {@code threshold} for as long or
     * longer than the {@code stableTimeout}, the loop is broken and the thread continues. If the error is not within
     * the {@code threshold}, the stabilization timer is reset.
     * <p>
     * Note that the {@code motor} velocity is not set. The {@code motor} is only used to get the current
     * velocity. To set the velocity as well, use
     * {@link #spinTo(DcMotorEx, double, double, double, double, LinearOpMode, double, double) spinTo} instead.
     *
     * @param motor          the motor to spin
     * @param power          the power level to spin to
     * @param threshold      the percent error that is acceptable
     * @param maxTimeout     the maximum amount of time to spend
     * @param stableTimeout  the amount of time for the speed to be considered stable
     * @param opMode         the opmode that is spinning it
     * @param maxRpm         the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     */
    public static void waitToSpinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode, double maxRpm, double ticksPerSecond) {
        // Calculate the target velocity and initialize the stabilization timer
        double target = fromMotorPower(power, maxRpm, ticksPerSecond);
        ElapsedTime stabilizationTimer = new ElapsedTime();

        // Run the lambda function for the maximum timeout, breaking out when the velocity
        // is within the threshold for the specified amount of time
        OpmodeUtil.sleep(maxTimeout, opMode, () -> {
            // Get the current motor velocity and calculate the error
            double velocity = motor.getVelocity();
            double error = Math.abs(target - velocity) / target;

            // Check if the error is within the specified threshold
            if (error <= threshold) {
                // If the error is within the threshold, check if the stabilization timeout
                // has finished. If it has finished, break out of the loop, otherwise continue
                return stabilizationTimer.seconds() >= stableTimeout;
            } else {
                // Reset the stabilization timer if the error is not within the threshold
                stabilizationTimer.reset();
            }

            // Continue the loop by default
            return false;
        });
    }
}

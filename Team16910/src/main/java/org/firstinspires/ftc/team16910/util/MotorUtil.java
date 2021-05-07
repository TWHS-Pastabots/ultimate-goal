package org.firstinspires.ftc.team16910.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TODO(BSFishy): document this
 */
public class MotorUtil {
    // Constants related to the motors we're using
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;

    /**
     * Convert the motor power level to encoder ticks per second. This takes a {@code [0, 1]} range power level
     * and manipulates its numeric value to convert it into a velocity that can be used by
     * {@link DcMotorEx#setVelocity(double)}.
     * <p>
     * What this actually does is takes the maximum motor RPM from {@link #MOTOR_MAX_RPM} and converts it to
     * the max rotations per second by dividing it by {@code 60}. It then multiplies it by the power level specified
     * to get the target number of rotations per second and then multiplies it by {@link #MOTOR_TICKS_PER_SECOND}
     * to convert it to the number of ticks per second used by the motor.
     *
     * @param power the power level as a decimal
     * @return motor ticks per second
     */
    public static double fromMotorPower(double power) {
        return fromMotorPower(power, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param power the power level as a decimal
     * @param maxRpm the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     * @return motor ticks per second
     */
    public static double fromMotorPower(double power, double maxRpm, double ticksPerSecond) {
        return maxRpm / 60 * power * ticksPerSecond;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param ticks the velocity as ticks per second
     * @return motor power level
     */
    public static double toMotorPower(double ticks) {
        return toMotorPower(ticks, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param ticks the velocity as ticks per second
     * @param maxRpm the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     * @return motor power level
     */
    public static double toMotorPower(double ticks, double maxRpm, double ticksPerSecond) {
        return ticks / ticksPerSecond / (maxRpm / 60);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param motor the motor to spin
     * @param power the power level to spin to
     * @param threshold the percent error that is acceptable
     * @param maxTimeout the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode the opmode that is spinning it
     */
    public static void spinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode) {
        spinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param motor the motor to spin
     * @param power the power level to spin to
     * @param threshold the percent error that is acceptable
     * @param maxTimeout the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode the opmode that is spinning it
     * @param maxRpm the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     */
    public static void spinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode, double maxRpm, double ticksPerSecond) {
        motor.setVelocity(fromMotorPower(power, maxRpm, ticksPerSecond));
        waitToSpinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, maxRpm, ticksPerSecond);

//        double target = fromMotorPower(power, maxRpm, ticksPerSecond);
//        motor.setVelocity(fromMotorPower(power, maxRpm, ticksPerSecond));
//
//        ElapsedTime stabilizationTimer = new ElapsedTime();
//        OpmodeUtil.sleep(maxTimeout, opMode, () -> {
//            double velocity = motor.getVelocity();
//            double error = Math.abs(target - velocity) / target;
//
//            if (error <= threshold) {
//                return stabilizationTimer.seconds() >= stableTimeout;
//            } else {
//                stabilizationTimer.reset();
//            }
//
//            return false;
//        });
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param motor the motor to spin
     * @param power the power level to spin to
     * @param threshold the percent error that is acceptable
     * @param maxTimeout the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode the opmode that is spinning it
     */
    public static void waitToSpinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode) {
        waitToSpinTo(motor, power, threshold, maxTimeout, stableTimeout, opMode, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param motor the motor to spin
     * @param power the power level to spin to
     * @param threshold the percent error that is acceptable
     * @param maxTimeout the maximum amount of time to spend
     * @param stableTimeout the amount of time for the speed to be considered stable
     * @param opMode the opmode that is spinning it
     * @param maxRpm the motor's maximum achievable RPM
     * @param ticksPerSecond the motor's encoder's ticks per second
     */
    public static void waitToSpinTo(DcMotorEx motor, double power, double threshold, double maxTimeout, double stableTimeout, LinearOpMode opMode, double maxRpm, double ticksPerSecond) {
        double target = fromMotorPower(power, maxRpm, ticksPerSecond);
        ElapsedTime stabilizationTimer = new ElapsedTime();

        OpmodeUtil.sleep(maxTimeout, opMode, () -> {
            double velocity = motor.getVelocity();
            double error = Math.abs(target - velocity) / target;

            if (error <= threshold) {
                return stabilizationTimer.seconds() >= stableTimeout;
            } else {
                stabilizationTimer.reset();
            }

            return false;
        });
    }
}

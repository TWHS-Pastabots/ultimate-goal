package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A class to help with the use of the drive train. This allows horizontal and vertical inputs to
 * be given and will apply the proper amount of power to each of the motors to facilitate movement
 * in that direction. This works best when given the input directly from a controller, but can also
 * be used as if the input was from a controller.
 *
 * @author Matt Provost
 */
public class DriveTrain {
    private Telemetry telemetry = null;
    private RobotHardware robot;

    // An array of motors that correspond to the inputs
    private DcMotor[] motors;

    // A multiplier for the inputs of the motors
    private double multiplier = 1f;

    // These are default constants to use with the move methods
    private static final double DEFAULT_TURN_ANGLE = 0;
    private static final boolean DEFAULT_ADD_TELEMETRY = false;

    /**
     * Initialize the drive train class. This is equivalent to calling the
     * {@link #setTelemetry(Telemetry)} and {@link #setRobotHardware(RobotHardware)} methods.
     *
     * @param telemetry the telemetry to use for logging
     * @param robot the robot hardware for the motors (cannot be null)
     * @see #setTelemetry(Telemetry)
     * @see #setRobotHardware(RobotHardware)
     */
    public void init(Telemetry telemetry, RobotHardware robot) {
        setTelemetry(telemetry);
        setRobotHardware(robot);
    }

    /**
     * Set the telemetry to use for logging. Telemetry can be set to null, which will disable
     * adding telemetry even if methods are called to use it.
     *
     * @param telemetry the telemetry to use for logging
     */
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Set the robot hardware. This provides the drive train with the motors for the wheels. If the
     * given {@literal robot} parameter is null, a {@link NullPointerException} will be thrown.
     *
     * @param robot the robot hardware for the motors (cannot be null)
     * @throws NullPointerException if the {@literal robot} parameter is null
     */
    public void setRobotHardware(RobotHardware robot) {
        if (robot == null) {
            throw new NullPointerException("The robot hardware cannot be null");
        }

        this.robot = robot;
        this.motors = new DcMotor[] {
                robot.motorLeftFront,   // front left
                robot.motorRightFront,  // front right
                robot.motorLeftRear,    // back left
                robot.motorRightRear    // back right
        };
    }

    /**
     * Get the motor power multiplier. This is the number the motor power levels are multiplied by
     * before the power level is set.
     *
     * @return the motor power multiplier
     * @see #setMultiplier(double)
     */
    public double getMultiplier() {
        return multiplier;
    }

    /**
     * Set the motor power multiplier. This is the number the motor power levels are multiplied by
     * before the power level is set.
     *
     * @param multiplier the new motor power multiplier
     * @see #getMultiplier()
     */
    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }

    /**
     * Move the robot with smoothing.
     *
     * This is an overload function for {@link #smoothMove(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #smoothMove(double, double, double, boolean)
     */
    public void smoothMove(double horizontal, double vertical) {
        smoothMove(horizontal, vertical, DEFAULT_TURN_ANGLE, DEFAULT_ADD_TELEMETRY);
    }

    /**
     * Move the robot with smoothing.
     *
     * This is an overload function for {@link #smoothMove(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param addTelemetry if telemetry should be added about the power to the motors (default: {@code true})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #smoothMove(double, double, double, boolean)
     */
    public void smoothMove(double horizontal, double vertical, boolean addTelemetry) {
        smoothMove(horizontal, vertical, DEFAULT_TURN_ANGLE, addTelemetry);
    }

    /**
     * Move the robot with smoothing.
     *
     * This is an overload function for {@link #smoothMove(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param turnAngle the amount to turn the robot on an interval of [{@code -1}, {@code 1}] (default: {@code 0})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #smoothMove(double, double, double, boolean)
     */
    public void smoothMove(double horizontal, double vertical, double turnAngle) {
        smoothMove(horizontal, vertical, turnAngle, DEFAULT_ADD_TELEMETRY);
    }

    /**
     * Move the robot with smoothing. This is the same as
     * {@link #move(double, double, double, boolean) move}, but with {@link Util#easeIn(double)}
     * applied to it.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param turnAngle the amount to turn the robot on an interval of [{@code -1}, {@code 1}] (default: {@code 0})
     * @param addTelemetry if telemetry should be added about the power to the motors (default: {@code true})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #move(double, double, double, boolean)
     */
    public void smoothMove(double horizontal, double vertical, double turnAngle, boolean addTelemetry) {
        move(Util.easeIn(horizontal), Util.easeIn(vertical), Util.easeIn(turnAngle), addTelemetry);
    }

    /**
     * Move the robot.
     *
     * This is an overload function for {@link #move(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #move(double, double, double, boolean)
     *
     */
    public void move(double horizontal, double vertical) {
        move(horizontal, vertical, DEFAULT_TURN_ANGLE, DEFAULT_ADD_TELEMETRY);
    }

    /**
     * Move the robot.
     *
     * This is an overload function for {@link #move(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param addTelemetry if telemetry should be added about the power to the motors (default: {@code true})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #move(double, double, double, boolean)
     */
    public void move(double horizontal, double vertical, boolean addTelemetry) {
        move(horizontal, vertical, DEFAULT_TURN_ANGLE, addTelemetry);
    }

    /**
     * Move the robot.
     *
     * This is an overload function for {@link #move(double, double, double, boolean)}. It is merely
     * meant to make it easier to call this method, by providing default values.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param turnAngle the amount to turn the robot on an interval of [{@code -1}, {@code 1}] (default: {@code 0})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     * @see #move(double, double, double, boolean)
     */
    public void move(double horizontal, double vertical, double turnAngle) {
        move(horizontal, vertical, turnAngle, DEFAULT_ADD_TELEMETRY);
    }

    /**
     * Move the robot. This takes the horizontal and vertical inputs and converts them into power
     * levels for the motors to use. It also uses {@literal turnAngle} parameter in the calculation
     * to turn the robot.
     *
     * The basic math behind this method is to rotate the given horizontal and vertical components
     * by 135 degrees, and use the resulting (x,y) as inputs for the power.
     *
     * @param horizontal the amount to move the robot horizontally on an interval of [{@code -1}, {@code 1}]
     * @param vertical the amount to move the robot vertically on an interval of [{@code -1}, {@code 1}]
     * @param turnAngle the amount to turn the robot on an interval of [{@code -1}, {@code 1}] (default: {@code 0})
     * @param addTelemetry if telemetry should be added about the power to the motors (default: {@code true})
     * @throws NullPointerException if the robot hardware has not been set (see {@link #setRobotHardware(RobotHardware)})
     */
    public void move(double horizontal, double vertical, double turnAngle, boolean addTelemetry) {
        // Check if the robot hardware is null
        if (robot == null) {
            throw new NullPointerException("The robot hardware has not been set");
        }

        // Calculate the new radius and angle for the given inputs
        double radius = Math.hypot(horizontal, vertical);
        double ang = Math.atan2(vertical, horizontal) - Math.PI / 4;

        // Calculate the turn constant
        double turnCon = turnAngle * .75;

        // Calculate all of the new rotated power levels
        double[] inputs = new double[] {
                radius * Math.cos(ang) + turnCon,
                radius * Math.sin(ang) - turnCon,
                radius * Math.sin(ang) + turnCon,
                radius * Math.cos(ang) - turnCon
        };

        // Set the power levels of all the motors
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(inputs[i] * getMultiplier());
        }

        // If telemetry should be added, add it
        if (addTelemetry && telemetry != null) {
            for (double input : inputs) {
                telemetry.addLine(Util.createLevel((float) (input * getMultiplier())));
            }
        }

        // This was the old code that is now replaced with loops
//        double v1 = (radius * Math.cos(ang) + turnCon) * getMultiplier();
//        double v2 = (radius * Math.sin(ang) - turnCon) * getMultiplier();
//        double v3 = (radius * Math.sin(ang) + turnCon) * getMultiplier();
//        double v4 = (radius * Math.cos(ang) - turnCon) * getMultiplier();
//
//        robot.motorLeftFront.setPower(v1);
//        robot.motorRightFront.setPower(v2);
//        robot.motorLeftRear.setPower(v3);
//        robot.motorRightRear.setPower(v4);
//
//        if (addTelemetry && telemetry != null) {
//            telemetry.addLine(Util.createLevel((float) v1));
//            telemetry.addLine(Util.createLevel((float) v2));
//            telemetry.addLine(Util.createLevel((float) v3));
//            telemetry.addLine(Util.createLevel((float) v4));
//        }
    }
}

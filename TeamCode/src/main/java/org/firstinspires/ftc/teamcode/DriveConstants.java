 package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 150;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.77165; // in
    public static double GEAR_RATIO = 40; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 1.5748; // in
    public static double WHEEL_BASE = 2; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}

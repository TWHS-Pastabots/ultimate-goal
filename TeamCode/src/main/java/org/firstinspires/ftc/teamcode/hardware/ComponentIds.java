package org.firstinspires.ftc.teamcode.hardware;

/**
 * A simple class to hold constants for the Motor IDs.
 * Referencing Motor and other Sensors via HardwareMap.get should reference the constants here for consistency across the codebase.
 * @author Ryan Walters
 */
public class ComponentIds { 
    // Wheel Motors
    public static final String LEFT_FRONT_MOTOR = "leftFront";
    public static final String RIGHT_FRONT_MOTOR = "rightFront";
    public static final String LEFT_REAR_MOTOR = "leftRear";
    public static final String RIGHT_REAR_MOTOR = "rightRear";

    // Encoders
    public static final String LEFT_ENCODER = "leftFront";
    public static final String RIGHT_ENCODER = "rightFront";
    public static final String FRONT_ENCODER = "leftRear";

    public static final String IMU = "imu";  // Used by Road Runner Telop only
}

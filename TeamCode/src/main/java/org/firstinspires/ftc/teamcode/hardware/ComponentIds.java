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

class MacaroniIds {
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String LAUNCHER_MOTOR = "launcherMotor";
    public static final String LOWER_WOBBLE_SERVO = "lowerWobbleServo";
    public static final String BELT_MOTOR = "beltMotor";
    public static final String UPPER_WOBBLE_SERVO = "upperWobbleServo";
}

class SpaghettiIds {
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String LAUNCHER_MOTOR = "launcherMotor";
    public static final String LOWER_WOBBLE_SERVO = "lowerWobbleServo";
    public static final String WOBBLE_ARM_MOTOR = "wobbleArmMotor";
    public static final String LAUNCHER_SERVO = "launcherServo";  // Only used in Spaghetti Hardware
}

class RavioliIds {
    public static final String MOTOR_CONVEYOR = "conveyor";
    public static final String MOTOR_LAUNCHER = "launcher";
    public static final String MOTOR_INTAKE = "intake";
    public static final String MOTOR_CLAW = "motorClaw";
    public static final String SERVO_CLAW = "servo_claw";
    public static final String INTAKE_LOCK = "intakeLock";
}
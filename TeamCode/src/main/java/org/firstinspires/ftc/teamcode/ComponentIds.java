package org.firstinspires.ftc.teamcode;

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
    public static final String MOTOR_RIGHT_REAR = "rightRear";

    // Launcher-related Motors & Servos
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String LAUNCHER_MOTOR = "launcherMotor";
    public static final String INTAKE_SERVO = "intakeServo";
    public static final String BELT_MOTOR = "beltMotor";
    public static final String BELT_STOPPER = "beltStopper";

    // Wobble Goal-related Motors & Servos
    public static final String WOBBLE_MOTOR = "wobbleMotor";
    public static final String WOBBLE_SERVO = "wobbleServo";

}
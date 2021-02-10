package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class SpaghettiHardware extends RobotHardware {
    // Miscellaneous robot motors and servos
    public DcMotorEx intakeMotor = null;
    public Servo launcherServo = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    public DcMotorEx launcherMotor = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        // Initialize motors and servos
        intakeMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.INTAKE_MOTOR);
        launcherServo = hardwareMap.get(Servo.class, SpaghettiIds.LAUNCHER_SERVO);
        armServo = hardwareMap.get(Servo.class, SpaghettiIds.ARM_SERVO);
        clawServo = hardwareMap.get(Servo.class, SpaghettiIds.CLAW_SERVO);
        launcherMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LAUNCHER_MOTOR);

        // Setup the motors list
        motors.addAll(Arrays.asList(intakeMotor, launcherMotor));

        // Setup the servos list
        servos.addAll(Arrays.asList(launcherServo, armServo, clawServo));

        // Set the motor and servo directions
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize the components now that they are setup
        initializeComponents();
    }
}

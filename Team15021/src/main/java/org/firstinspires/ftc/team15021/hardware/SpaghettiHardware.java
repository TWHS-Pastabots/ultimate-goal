package org.firstinspires.ftc.team15021.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
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
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    public CRServo intakeServo = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        // Initialize motors and servos
        intakeMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.INTAKE_MOTOR);
        launcherServo = hardwareMap.get(Servo.class, SpaghettiIds.LAUNCHER_SERVO);
        armServo = hardwareMap.get(Servo.class, SpaghettiIds.ARM_SERVO);
        clawServo = hardwareMap.get(Servo.class, SpaghettiIds.CLAW_SERVO);
        launcherMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LAUNCHER_MOTOR);
        leftIntakeServo = hardwareMap.get(Servo.class, SpaghettiIds.LEFT_INTAKE_SERVO);
        rightIntakeServo = hardwareMap.get(Servo.class, SpaghettiIds.RIGHT_INTAKE_SERVO);
        intakeServo = hardwareMap.get(CRServo.class, SpaghettiIds.INTAKE_SERVO);

        // Setup the motors list
        motors.addAll(Arrays.asList(intakeMotor, launcherMotor));

        // Setup the servos list
        servos.addAll(Arrays.asList(launcherServo, armServo, clawServo, leftIntakeServo, rightIntakeServo));

        // Set the motor and servo directions
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftIntakeServo.setDirection(Servo.Direction.FORWARD);
        rightIntakeServo.setDirection(Servo.Direction.FORWARD);
        intakeServo.setDirection(CRServo.Direction.FORWARD);

        // Range scalings
        launcherServo.scaleRange(0, 0.225);
        armServo.scaleRange(0.42, 1);
        clawServo.scaleRange(0.2, 1);

        // Initialize the components now that they are setup
        initializeComponents();

        // Set motor modes
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize values
        armServo.setPosition(1);
        leftIntakeServo.setPosition(1);
        rightIntakeServo.setPosition(1);
    }
}

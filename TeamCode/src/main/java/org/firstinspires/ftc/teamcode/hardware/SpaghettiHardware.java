package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class SpaghettiHardware extends RobotHardware {
    // Miscellaneous robot motors and servos
    public DcMotorEx intakeMotor = null;
    public Servo launcherServo = null;
    public Servo wobbleArmServo = null;
    public DcMotorEx launcherMotor = null;
    public DcMotorEx wobbleArmMotor = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        // Initialize motors and servos
        intakeMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.INTAKE_MOTOR);
        launcherServo = hardwareMap.get(Servo.class, SpaghettiIds.LAUNCHER_SERVO);
        wobbleArmServo = hardwareMap.get(Servo.class, SpaghettiIds.LOWER_WOBBLE_SERVO);
        launcherMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LAUNCHER_MOTOR);
        wobbleArmMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.WOBBLE_ARM_MOTOR);

        // Setup the motors list
        motors.addAll(Arrays.asList(intakeMotor, launcherMotor, wobbleArmMotor));

        // Setup the servos list
        servos.addAll(Arrays.asList(launcherServo, wobbleArmServo));

        // Set the motor and servo directions
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        wobbleArmServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleArmMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize the components now that they are setup
        initializeComponents();
    }
}

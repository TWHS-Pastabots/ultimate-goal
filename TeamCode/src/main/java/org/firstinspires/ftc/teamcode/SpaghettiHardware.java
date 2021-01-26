package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.ComponentIds.*;

public class SpaghettiHardware extends RobotHardware {
    // Miscellaneous robot motors and servos
    public DcMotorEx intakeMotor = null;
    public Servo launcherServo = null;
    public DcMotorEx launcherMotor = null;
    public DcMotorEx wobbleArmMotor = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        // Initialize motors and servos
        intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);
        launcherServo = hardwareMap.get(Servo.class, LAUNCHER_SERVO);
        launcherMotor = hardwareMap.get(DcMotorEx.class, LAUNCHER_MOTOR);
        wobbleArmMotor = hardwareMap.get(DcMotorEx.class, WOBBLE_ARM_MOTOR);

        // Setup the motors list
        motors.addAll(Arrays.asList(intakeMotor, launcherMotor, wobbleArmMotor));

        // Setup the servos list
        servos.addAll(Arrays.asList(launcherServo));

        // Set the motor and servo directions
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        wobbleArmMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize the components now that they are setup
        initializeComponents();
    }
}

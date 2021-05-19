package org.firstinspires.ftc.team15021.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class RavioliHardware extends RobotHardware {
    public DcMotorEx motorConveyor = null;
    public DcMotor motorLauncher = null;
    public DcMotorEx motorIntake = null;

    public DcMotorEx motorClaw = null;
    public Servo servoClaw = null;
    public Servo servoIntake = null;


    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        motorConveyor = hardwareMap.get(DcMotorEx.class, RavioliIds.MOTOR_CONVEYOR);
        motorLauncher = hardwareMap.get(DcMotorEx.class, RavioliIds.MOTOR_LAUNCHER);
        motorIntake = hardwareMap.get(DcMotorEx.class, RavioliIds.MOTOR_INTAKE);

        motorClaw = hardwareMap.get(DcMotorEx.class, RavioliIds.MOTOR_CLAW);
        servoClaw = hardwareMap.get(Servo.class, RavioliIds.SERVO_CLAW);
        servoIntake = hardwareMap.get(Servo.class, RavioliIds.INTAKE_LOCK);

        motors.addAll(Arrays.asList(motorConveyor, motorIntake, motorClaw));
        servos.addAll(Arrays.asList(servoClaw, servoIntake));

        // Reverse specific motors
        motorConveyor.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorClaw.setDirection(DcMotorEx.Direction.REVERSE);
        motorLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        initializeComponents();
        //motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
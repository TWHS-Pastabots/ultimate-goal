package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.*;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class RavioliHardware extends RobotHardware {

    // Time function
    public DcMotorEx motorConveyor = null;
    public DcMotorEx motorLauncher = null;
    public DcMotorEx motorIntake = null;

    public DcMotorEx motorClaw = null;
    public Servo servoClaw = null;

    public Servo servoIntake = null;

    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        motorConveyor = hwMap.get(DcMotorEx.class, MOTOR_CONVEYOR);
        motorLauncher = hwMap.get(DcMotorEx.class, MOTOR_LAUNCHER);
        motorIntake = hwMap.get(DcMotorEx.class, MOTOR_INTAKE);

        motorClaw = hwMap.get(DcMotorEx.class, MOTOR_CLAW);
        servoClaw = hwMap.get(Servo.class, SERVO_CLAW);
        servoIntake = hwMap.get(Servo.class, INTAKE_LOCK);

        motors.addAll(Arrays.asList(motorConveyor, motorLauncher, motorIntake, motorClaw));
        servos.addAll(Arrays.asList(servoClaw, servoIntake));

        // Reverse specific motors
        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightRear.setDirection(DcMotorEx.Direction.REVERSE);
        motorConveyor.setDirection(DcMotorEx.Direction.REVERSE);
        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);

        initializeComponents();
    }
}
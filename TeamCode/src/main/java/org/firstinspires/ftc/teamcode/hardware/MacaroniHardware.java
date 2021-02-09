package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class MacaroniHardware extends RobotHardware {
    public DcMotorEx intakeMotor;
    public DcMotorEx beltMotor;
    public DcMotorEx launcherMotor;

    public Servo clawServo;
    public Servo armServo;

    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        intakeMotor = hwMap.get(DcMotorEx.class, MacaroniIds.INTAKE_MOTOR);
        launcherMotor = hwMap.get(DcMotorEx.class, MacaroniIds.LAUNCHER_MOTOR);
        beltMotor = hwMap.get(DcMotorEx.class, MacaroniIds.BELT_MOTOR);

        armServo = hwMap.get(Servo.class, MacaroniIds.ARM_SERVO);
        clawServo = hwMap.get(Servo.class, MacaroniIds.CLAW_SERVO);

        motors.addAll(Arrays.asList(intakeMotor, launcherMotor, beltMotor));
        servos.addAll(Arrays.asList(armServo, clawServo));

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightRear.setDirection(DcMotorEx.Direction.REVERSE);
        beltMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        initializeComponents();
    }
}

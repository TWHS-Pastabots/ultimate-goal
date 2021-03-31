package org.firstinspires.ftc.team16911.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team16911.hardware.MacaroniIds;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Arrays;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class MacaroniHardware extends RobotHardware {
    private DcMotorEx intakeMotor;
    private DcMotorEx beltMotor;
    private DcMotorEx launcherMotor;

    private ElapsedTime launcherElapsed;
    private Servo leftClawServo;
    private Servo rightClawServo;
    public Servo armServo;
    private Servo ringStopperServo;


    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        intakeMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.INTAKE_MOTOR);
        launcherMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.LAUNCHER_MOTOR);
        beltMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.BELT_MOTOR);

        armServo = hardwareMap.get(Servo.class, MacaroniIds.ARM_SERVO);
        leftClawServo = hardwareMap.get(Servo.class, MacaroniIds.LEFT_CLAW_SERVO);
        rightClawServo = hardwareMap.get(Servo.class, MacaroniIds.RIGHT_CLAW_SERVO);
        ringStopperServo = hardwareMap.get(Servo.class, MacaroniIds.RING_STOPPER_SERVO);

        motors.addAll(Arrays.asList(intakeMotor, launcherMotor, beltMotor));
        servos.addAll(Arrays.asList(armServo, leftClawServo, rightClawServo, ringStopperServo));

        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightRear.setDirection(DcMotorEx.Direction.REVERSE);

        beltMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        initializeComponents();


        armServo.setPosition(1);
        setClaws(0);
        ringStopperServo.setPosition(1);
    }

    public void setClaws(double position) {
        leftClawServo.setPosition(1 - position);
        rightClawServo.setPosition(position);
    }
}

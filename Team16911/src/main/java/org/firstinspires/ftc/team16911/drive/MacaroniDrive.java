package org.firstinspires.ftc.team16911.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team16911.hardware.MacaroniIds;

public class MacaroniDrive extends MacaroniMecanumDrive {
    public Servo leftClawServo;
    public Servo clawServo;

    public Servo armServo;
    public Servo ringStopperServo;

    public DcMotorEx intakeMotor;
    public DcMotorEx beltMotor;
    public DcMotorEx launcherMotor;

    public MacaroniDrive(HardwareMap hardwareMap) {
        super(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.INTAKE_MOTOR);
        launcherMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.LAUNCHER_MOTOR);
        beltMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.BELT_MOTOR);

        armServo = hardwareMap.get(Servo.class, MacaroniIds.ARM_SERVO);
        clawServo = hardwareMap.get(Servo.class, MacaroniIds.CLAW_SERVO);

        ringStopperServo = hardwareMap.get(Servo.class, MacaroniIds.RING_STOPPER_SERVO);

        beltMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        armServo.setPosition(1);
        setClaws(0);

        ringStopperServo.setPosition(1);
    }

    public void setClaws(double position) {
        clawServo.setPosition(position);
    }
}

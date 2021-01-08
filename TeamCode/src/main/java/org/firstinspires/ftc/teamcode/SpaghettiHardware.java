package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.ComponentIds.*;

public class SpaghettiHardware {
    // Primary wheel motors
    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;

    public DcMotor[] motors;
    public DcMotor[] wheels;
    public Servo[] servos;

    public DcMotor intakeMotor;
    public Servo launcherServo;
    public DcMotor launcherMotor;
    public DcMotor wobbleArmMotor;

    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        leftFrontMotor = hwMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        rightFrontMotor = hwMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);
        leftRearMotor = hwMap.get(DcMotor.class, LEFT_REAR_MOTOR);
        rightRearMotor = hwMap.get(DcMotor.class, MOTOR_RIGHT_REAR);

        intakeMotor = hwMap.get(DcMotor.class, OUTER_INTAKE_MOTOR);
        launcherServo = hwMap.get(Servo.class, LAUNCHER_SERVO);
        launcherMotor = hwMap.get(DcMotor.class, LAUNCHER_MOTOR);
        wobbleArmMotor = hwMap.get(DcMotor.class, WOBBLE_ARM_MOTOR);

        wheels = new DcMotor[]{leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        motors = new DcMotor[]{leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor, intakeMotor, wobbleArmMotor, launcherMotor};
        servos = new Servo[]{launcherServo};

        // Motor direction is FORWARD by default
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // These are built reversed
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Setup motors
        for (DcMotor motor : motors) {
            // Set all motors to zero power
            motor.setPower(0);
            // Motors will break on Zero power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Setup servos
        for (Servo servo : servos) {
            // Set all servos to zeroed out position
            servo.setPosition(0);
        }
    }
}

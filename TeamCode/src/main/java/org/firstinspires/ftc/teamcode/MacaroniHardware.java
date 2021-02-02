package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.ComponentIds.*;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class MacaroniHardware {
    // Primary wheel motors
    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;

    public DcMotor[] motors;
    public DcMotor[] wheels;

    public DcMotor intakeMotor;
    public DcMotor beltMotor;
    public DcMotor launcherMotor;

    public Servo upperWobbleServo;
    public Servo lowerWobbleServo;

    HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        leftFrontMotor = hwMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        rightFrontMotor = hwMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);
        leftRearMotor = hwMap.get(DcMotor.class, LEFT_REAR_MOTOR);
        rightRearMotor = hwMap.get(DcMotor.class, MOTOR_RIGHT_REAR);

        intakeMotor = hwMap.get(DcMotor.class, INTAKE_MOTOR);
        launcherMotor = hwMap.get(DcMotor.class, LAUNCHER_MOTOR);
        beltMotor = hwMap.get(DcMotor.class, BELT_MOTOR);

        lowerWobbleServo = hwMap.get(Servo.class, LOWER_WOBBLE_SERVO);
        upperWobbleServo = hwMap.get(Servo.class, UPPER_WOBBLE_SERVO);

        wheels = new DcMotor[]{leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor};
        motors = new DcMotor[]{leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor, intakeMotor, launcherMotor, beltMotor};

        // Motor direction is FORWARD by default
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // These are built reversed
        beltMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

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
    }
}

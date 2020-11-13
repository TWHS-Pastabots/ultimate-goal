package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.ComponentIds.*;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class RobotHardware {
    // Primary wheel motors
    public DcMotor motorLeftFront = null;
    public DcMotor motorLeftRear = null;
    public DcMotor motorRightFront = null;
    public DcMotor motorRightRear = null;
    public DcMotor[] motors;

    // Encoders
    public DcMotor encoderLeft = null;
    public DcMotor encoderRight = null;
    public DcMotor encoderHorizontal = null;

    HardwareMap hwMap = null;

    /* Constructor */
    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        motorLeftFront = hwMap.get(DcMotor.class, MOTOR_LEFT_FRONT);
        motorRightFront = hwMap.get(DcMotor.class, MOTOR_RIGHT_FRONT);
        motorLeftRear = hwMap.get(DcMotor.class, MOTOR_LEFT_REAR);
        motorRightRear = hwMap.get(DcMotor.class, MOTOR_RIGHT_REAR);
        motors = new DcMotor[]{motorLeftFront, motorRightFront, motorLeftRear, motorRightRear};

        // TODO: Better understanding of what exactly the encoders are being registered to, and how.
        // encoderHorizontal = hwMap.get(DcMotor.class, "tape_motor");
        // encoderLeft = hwMap.get(DcMotor.class, "left_rear");
        // encoderRight = hwMap.get(DcMotor.class, "right_rear");

        // Motor direction is FORWARD by default
        motorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightRear.setDirection(DcMotor.Direction.REVERSE);

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

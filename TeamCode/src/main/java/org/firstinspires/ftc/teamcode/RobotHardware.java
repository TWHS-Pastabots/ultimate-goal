package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static org.firstinspires.ftc.teamcode.ComponentIds.*;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
public class RobotHardware {
    // Primary wheel motors
    public DcMotorEx motorLeftFront = null;
    public DcMotorEx motorLeftRear = null;
    public DcMotorEx motorRightFront = null;
    public DcMotorEx motorRightRear = null;
    public DcMotorEx[] motors;

    // Primary encoders
    public DcMotorEx encoderLeft = null;
    public DcMotorEx encoderRight = null;
    public DcMotorEx encoderFront = null;

    // IMU
    public BNO055IMU imu;

    // Battery voltage sensor
    public VoltageSensor batteryVoltageSensor;

    HardwareMap hwMap = null;

    /* Constructor */
    public RobotHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        this.hwMap = hwMap;

        motorLeftFront = hwMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR);
        motorRightFront = hwMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR);
        motorLeftRear = hwMap.get(DcMotorEx.class, LEFT_REAR_MOTOR);
        motorRightRear = hwMap.get(DcMotorEx.class, MOTOR_RIGHT_REAR);

        motors = new DcMotorEx[]{motorLeftFront, motorRightFront, motorLeftRear, motorRightRear};

        encoderLeft = hwMap.get(DcMotorEx.class, LEFT_ENCODER);
        encoderRight = hwMap.get(DcMotorEx.class, RIGHT_ENCODER);
        encoderFront = hwMap.get(DcMotorEx.class, FRONT_ENCODER);

        imu = hwMap.get(BNO055IMU.class, IMU);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

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

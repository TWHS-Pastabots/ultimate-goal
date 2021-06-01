package org.firstinspires.ftc.team16910.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16910.util.MotorUtil;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Arrays;

/**
 * TODO(BSFishy): document this
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
@Config
public class SpaghettiHardware extends RobotHardware {
    // Useful symbolic on and off constants
    public static double ON = 1;
    public static double OFF = 0;

    public static double INTAKE_POWER = 0.15;
    public static double OUTER_INTAKE_POWER = 0.5;
    public static double LAUNCHER_POWER = 0.575;

    // Miscellaneous robot motors and servos
    public DcMotorEx intakeMotor = null;
    public Servo launcherServo = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    public DcMotorEx launcherMotor = null;
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    public DcMotorEx outerIntakeMotor = null;

    private Telemetry telemetry;

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        // Initialize motors and servos
        intakeMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.INTAKE_MOTOR);
        launcherServo = hardwareMap.get(Servo.class, SpaghettiIds.LAUNCHER_SERVO);
        armServo = hardwareMap.get(Servo.class, SpaghettiIds.ARM_SERVO);
        clawServo = hardwareMap.get(Servo.class, SpaghettiIds.CLAW_SERVO);
        launcherMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LAUNCHER_MOTOR);
        leftIntakeServo = hardwareMap.get(Servo.class, SpaghettiIds.LEFT_INTAKE_SERVO);
        rightIntakeServo = hardwareMap.get(Servo.class, SpaghettiIds.RIGHT_INTAKE_SERVO);
        outerIntakeMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.OUTER_INTAKE_MOTOR);

        // Setup the motors list
        motors.addAll(Arrays.asList(intakeMotor, outerIntakeMotor, launcherMotor));

        // Setup the servos list
        servos.addAll(Arrays.asList(launcherServo, armServo, clawServo, leftIntakeServo, rightIntakeServo));

        // Set the motor and servo directions
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherServo.setDirection(Servo.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftIntakeServo.setDirection(Servo.Direction.FORWARD);
        rightIntakeServo.setDirection(Servo.Direction.FORWARD);
        outerIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Range scalings
        launcherServo.scaleRange(0, 0.19);
        armServo.scaleRange(0.025, 0.5);
        clawServo.scaleRange(0.2, 1);

        // Initialize the components now that they are setup
        initializeComponents();

        // Set motor modes
        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize values
        armServo.setPosition(1);
        leftIntakeServo.setPosition(1);
        rightIntakeServo.setPosition(1);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void lowerWobbleArm() {
        armServo.setPosition(OFF);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void raiseWobbleArm() {
        armServo.setPosition(ON);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void openClaw() {
        clawServo.setPosition(OFF);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void closeClaw() {
        clawServo.setPosition(ON);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void enableIntake() {
        enableIntake(false);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param withTelemetry whether or not telemetry should be added
     */
    public void enableIntake(boolean withTelemetry) {
        double target = MotorUtil.fromMotorPower(INTAKE_POWER) * 5;
        intakeMotor.setVelocity(target);
        outerIntakeMotor.setPower(OUTER_INTAKE_POWER);

        if (withTelemetry && telemetry != null) {
            double velocity = intakeMotor.getVelocity();
            double error = Math.abs(target - velocity) / target;

            telemetry.addData("intake motor target", "%.02f", target);
            telemetry.addData("intake motor actual", "%.02f", velocity);
            telemetry.addData("intake motor error", "%.02f%%", error * 100);
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    public void disableIntake() {
        disableIntake(false);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param withTelemetry whether or not telemetry should be added
     */
    public void disableIntake(boolean withTelemetry) {
        double target = 0;
        intakeMotor.setVelocity(target);
        outerIntakeMotor.setPower(OFF);

        if (withTelemetry && telemetry != null) {
            double velocity = intakeMotor.getVelocity();
            double error = Math.abs(target - velocity) / 1;

            telemetry.addData("intake motor target", "%.02f", target);
            telemetry.addData("intake motor actual", "%.02f", velocity);
            telemetry.addData("intake motor error", "%.02f%%", error * 100);
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    public void raiseIntake() {
        leftIntakeServo.setPosition(OFF);
        rightIntakeServo.setPosition(OFF);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void lowerIntake() {
        leftIntakeServo.setPosition(ON);
        rightIntakeServo.setPosition(ON);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void extendLauncherServo() {
        launcherServo.setPosition(ON);
    }

    /**
     * TODO(BSFishy): document this
     */
    public void retractLauncherServo() {
        launcherServo.setPosition(OFF);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param power the amount of power the launcher should spin to
     */
    public void spinLauncher(double power) {
        spinLauncher(power, false);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param power         the amount of power the launcher should spin to
     * @param withTelemetry whether or not telemetry should be added
     */
    public void spinLauncher(double power, boolean withTelemetry) {
        spinLauncherRaw(MotorUtil.fromMotorPower(LAUNCHER_POWER) * power, withTelemetry);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param power the amount of power the launcher should spin to
     */
    public void spinLauncherRaw(double power) {
        spinLauncherRaw(power, false);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param power         the amount of power the launcher should spin to
     * @param withTelemetry whether or not telemetry should be added
     */
    public void spinLauncherRaw(double power, boolean withTelemetry) {
        launcherMotor.setVelocity(power);

        if (withTelemetry && telemetry != null) {
            double velocity = launcherMotor.getVelocity();
            double error = Math.abs(power - velocity) / power;

            telemetry.addData("launcher motor target", "%.02f", power);
            telemetry.addData("launcher motor actual", "%.02f", velocity);
            telemetry.addData("launcher motor error", "%.02f%%", error * 100);
        }
    }
}

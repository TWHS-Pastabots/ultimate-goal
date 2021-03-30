package org.firstinspires.ftc.team15021.telop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auton.SpaghettiAutonomous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SpaghettiHardware;

@TeleOp(name = "Spaghetti", group = "Linear OpMode")
@Config
public class Spaghetti extends OpMode {
    public static double NUDGE_AMOUNT = 0.25;

    public static double ON = 1;
    public static double OFF = 0;

    public static double INTAKE_ON = 0.175; // 0.2325
    public static double INTAKE_GEAR_RATIO = 5;

    private double lastLaunchTime = 0;
    public static double LAUNCH_TIME = 0.3;
    public static double LAUNCHER_POWER = 0.6375;
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;
//    public static double LAUNCHER_MOTOR_SCALE = 0.75d;

    /**
     * Convert the motor power level to encoder ticks per second. This takes a {@code [0, 1]} range power level
     * and manipulates its numeric value to convert it into a velocity that can be used by
     * {@link DcMotorEx#setVelocity(double)}.
     *
     * @param power the power level as a decimal
     * @return motor ticks per second
     */
    public static double motorPower(double power) {
        return MOTOR_MAX_RPM / 60 * power * MOTOR_TICKS_PER_SECOND;
    }

    public static double X_SCALE = 0.7;
    public static double Y_SCALE = 0.7;
    public static double TURN_SCALE = 0.85;

//    public static Pose2d farPosition = new Pose2d(-39, 48, 0);
    //    public static Pose2d middlePosition = new Pose2d(-26, 41, 0);
    public static Pose2d middlePosition = new Pose2d(-22.570475172975446, 53.49077586858332, Math.toRadians(-0.862970095));

    /* Declare OpMode members. */
    private final SpaghettiHardware robot = new SpaghettiHardware();
    private SampleMecanumDrive drive;

    private boolean previousIntakeButton = false;
    private boolean intakeState = true;

    private boolean previousClawButton = false;
    private boolean clawState = false;

    private boolean previousArmButton = false;
    private boolean armState = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.armServo.setPosition(ON);

        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(SpaghettiAutonomous.FINAL_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
    }

    @Override
    public void start() {
        telemetry.addData("Run Time", "reset");
    }

    @Override
    public void loop() {
        movement();
        operation();

        telemetry.update();
    }

    private void movement() {
        double x_nudge = gamepad1.dpad_right ? NUDGE_AMOUNT : (gamepad1.dpad_left ? -NUDGE_AMOUNT : 0);
        double y_nudge = gamepad1.dpad_up ? -NUDGE_AMOUNT : (gamepad1.dpad_down ? NUDGE_AMOUNT : 0);
        double ang_nudge = gamepad1.right_bumper ? NUDGE_AMOUNT : (gamepad1.left_bumper ? -NUDGE_AMOUNT : 0);

        if (!drive.isBusy()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y * Y_SCALE + y_nudge),
                            -(gamepad1.left_stick_x * X_SCALE + x_nudge),
                            -(gamepad1.right_stick_x * TURN_SCALE + ang_nudge)
                    )
            );
        }

//        if (gamepad1.a) {
//            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
//                    .splineToLinearHeading(farPosition, 0)
//                    .build());
//        }

        if (gamepad1.b) {
            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToLinearHeading(middlePosition, 90)
                    .build());
        }

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }

    private void operation() {
//        robot.launcherMotor.setPower(Util.cubicEasing(gamepad2.right_trigger) * LAUNCHER_MOTOR_SCALE);
//        robot.launcherMotor.setVelocity(motorPower(LAUNCHER_POWER) * gamepad2.right_trigger);
//        telemetry.addData("launcher motor actual", robot.launcherMotor.getVelocity());
        motorVelo(robot.launcherMotor, "launcher motor", motorPower(LAUNCHER_POWER) * gamepad2.right_trigger);

        if (gamepad2.left_bumper && lastLaunchTime + LAUNCH_TIME * 2 < getRuntime()) {
            robot.launcherServo.setPosition(ON);
            lastLaunchTime = getRuntime();
        }

        if (lastLaunchTime + LAUNCH_TIME < getRuntime()) {
            robot.launcherServo.setPosition(OFF);
        }

//        telemetry.addData("launcher servo position", robot.launcherServo.getPosition());

        if (gamepad2.right_bumper) {
            motorVelo(robot.intakeMotor, "intake motor", motorPower(INTAKE_ON) * INTAKE_GEAR_RATIO);
//            robot.intakeMotor.setVelocity(motorPower(INTAKE_ON) * INTAKE_GEAR_RATIO);
            robot.intakeServo.setPower(ON);
        } else {
//            robot.intakeMotor.setVelocity(OFF);
            motorVelo(robot.intakeMotor, "intake motor", OFF);
            robot.intakeServo.setPower(OFF);
        }

//        telemetry.addData("intake motor", robot.intakeMotor.getVelocity());

//        telemetry.addData("intake motor power", robot.intakeMotor.getPower());

        boolean clawButton = gamepad2.y;
        if (clawButton != previousClawButton) {
            if (clawButton) {
                clawState = !clawState;
                if (clawState) {
                    robot.clawServo.setPosition(ON);
                } else {
                    robot.clawServo.setPosition(OFF);
                }
            }

            previousClawButton = clawButton;
        }

//        telemetry.addData("claw servo position", robot.clawServo.getPosition());

        boolean armButton = gamepad2.b;
        if (armButton != previousArmButton) {
            if (armButton) {
                armState = !armState;
                if (armState) {
                    robot.armServo.setPosition(OFF);
                } else {
                    robot.armServo.setPosition(ON);
                }
            }

            previousArmButton = armButton;
        }

//        telemetry.addData("arm servo position", robot.armServo.getPosition());

        boolean intakeButton = gamepad2.a;
        if (intakeButton != previousIntakeButton) {
            if (intakeButton) {
                intakeState = !intakeState;
                if (intakeState) {
                    robot.leftIntakeServo.setPosition(ON);
                    robot.rightIntakeServo.setPosition(ON);
                } else {
                    robot.leftIntakeServo.setPosition(OFF);
                    robot.rightIntakeServo.setPosition(OFF);
                }
            }

            previousIntakeButton = intakeButton;
        }

//        telemetry.addData("intake state", intakeState);
    }

    private void motorVelo(DcMotorEx motor, String name, double amount) {
        motor.setVelocity(amount);

        double target = amount;
        double actual = motor.getVelocity();
        double error = Math.abs(target - actual) / target;

        telemetry.addData(String.format("%s target velocity", name), target);
        telemetry.addData(String.format("%s actual velocity", name), actual);
        telemetry.addData(String.format("%s velocity error", name), error);
    }
}

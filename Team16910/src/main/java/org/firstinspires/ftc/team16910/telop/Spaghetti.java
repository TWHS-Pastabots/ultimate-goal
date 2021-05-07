package org.firstinspires.ftc.team16910.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16910.drive.DriveConstants;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = Spaghetti.NAME, group = Spaghetti.GROUP)
@Config
public class Spaghetti extends OpMode {
    private enum Mode {
        DriverControl,
        AlignToPoint,
    }

    public static final String NAME = "Spaghetti";
    public static final String GROUP = "Spaghetti";

    public static double DRAWING_TARGET_RADIUS = 2;
    private Mode currentMode = Mode.DriverControl;
    private final PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d targetPosition = GOAL_POSITION;
    public static Vector2d GOAL_POSITION = new Vector2d(72, 36);
    public static Vector2d POWER_SHOT_POSITION = new Vector2d(72, 12);

    // The small amount to move the robot for fine-tuned movements
    public static double NUDGE_AMOUNT = 0.25;
    public static double TARGET_NUDGE_AMOUNT = 1;

    // Useful symbolic on and off constants
    public static double ON = 1;
    public static double OFF = 0;

    // Constants related to the intake
    public static double INTAKE_ON = 0.15;
    public static double INTAKE_GEAR_RATIO = 5;

    // Variables used for the launcher
    private double lastLaunchTime = 0;
    public static double LAUNCH_TIME = 0.6;
    public static double LAUNCHER_POWER = 0.675;

    // Constants related to the motors we're using
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;

    public static double LAUNCHER_AID_POWER = 0.5;

    /**
     * Convert the motor power level to encoder ticks per second. This takes a {@code [0, 1]} range power level
     * and manipulates its numeric value to convert it into a velocity that can be used by
     * {@link DcMotorEx#setVelocity(double)}.
     * <p>
     * What this actually does is takes the maximum motor RPM from {@link #MOTOR_MAX_RPM} and converts it to
     * the max rotations per second by dividing it by {@code 60}. It then multiplies it by the power level specified
     * to get the target number of rotations per second and then multiplies it by {@link #MOTOR_TICKS_PER_SECOND}
     * to convert it to the number of ticks per second used by the motor.
     *
     * @param power the power level as a decimal
     * @return motor ticks per second
     */
    public static double fromMotorPower(double power) {
//        return MOTOR_MAX_RPM / 60 * power * MOTOR_TICKS_PER_SECOND;
        return fromMotorPower(power, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    public static double fromMotorPower(double power, double maxRpm, double ticksPerSecond) {
        return maxRpm / 60 * power * ticksPerSecond;
    }

    public static double toMotorPower(double ticks) {
//        return ticks / MOTOR_TICKS_PER_SECOND / (MOTOR_MAX_RPM / 60);
        return toMotorPower(ticks, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    public static double toMotorPower(double ticks, double maxRpm, double ticksPerSecond) {
        return ticks / ticksPerSecond / (maxRpm / 60);
    }

    // Constants that scale the input power so movement isn't super jerky
    public static double X_SCALE = 0.9;
    public static double Y_SCALE = 0.9;
    public static double TURN_SCALE = 0.85;

//    // A position that is useful for launching rings from
//    // TODO: this should be deleted because it's super not recommended
//    public static Pose2d middlePosition = new Pose2d(-22.570475172975446, 53.49077586858332, Math.toRadians(-0.862970095));

    // Hardware-related variables
    private final SpaghettiHardware robot = new SpaghettiHardware();
    private SampleMecanumDrive drive;

//    private VuforiaUtil vuforia = new VuforiaUtil(true);
//    private final ElapsedTime vuforiaTimer = new ElapsedTime();
//    public static double VUFORIA_TIME = 0.75;
//    private int vuforiaTracks = 0;
//    private int vuforiaFinds = 0;

    // State tracking variables for the intake
    private boolean previousIntakeButton = false;
    private boolean intakeState = true;

    // State tracking variables for the claw
    private boolean previousClawButton = false;
    private boolean clawState = false;

    // State tracking variables for the arm
    private boolean previousArmButton = false;
    private boolean armState = false;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize the robot hardware
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.position);

        headingController.setInputBounds(-Math.PI, Math.PI);

//        vuforia.activate(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

//        updateVuforia();

        // Separate the movement and operation code into two separate functions
        // so that it is easier to understand the separation
        movement(packet);
        operation(packet);

//        telemetry.addData("vuforia tracks", vuforiaTracks);
//        telemetry.addData("vuforia finds", vuforiaFinds);

        // Update the telemetry
        dashboard.sendTelemetryPacket(packet);
//        telemetry.update();
    }

    @Override
    public void stop() {
//        vuforia.deactivate();
    }

//    private void updateVuforia() {
//        if (vuforiaTimer.seconds() < VUFORIA_TIME) {
//            return;
//        }
//
//        vuforiaTracks++;
//        vuforiaTimer.reset();
//
//        Pose2d pose = vuforia.getPosition();
//        if (pose == null) {
//            return;
//        }
//
//        vuforiaFinds++;
//        drive.setPoseEstimate(pose);
//    }

    /**
     *
     */
    private void movement(TelemetryPacket packet) {
        Canvas fieldOverlay = packet.fieldOverlay();
        packet.put("mode", currentMode);

        if (gamepad1.right_bumper) {
            targetPosition = GOAL_POSITION;
        } else if (gamepad1.left_bumper) {
            targetPosition = POWER_SHOT_POSITION;
        }

        // Draw the target on the field
        fieldOverlay.setStroke("#dd2c00");
        fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

        // Calculate nudge amounts for all of the axis
        double x_nudge = gamepad1.dpad_right ? NUDGE_AMOUNT : (gamepad1.dpad_left ? -NUDGE_AMOUNT : 0);
        double y_nudge = gamepad1.dpad_up ? -NUDGE_AMOUNT : (gamepad1.dpad_down ? NUDGE_AMOUNT : 0);
        double ang_nudge = gamepad1.right_trigger * NUDGE_AMOUNT + gamepad1.left_trigger * -NUDGE_AMOUNT;

        // Update he localizer
        drive.getLocalizer().update();

        // Read the current pose from the drive train
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Create a vector from the inputs and rotate it by the inverse of the heading of the robot
        // so that inputs are relative to the field rather than the robot
        // see: https://www.learnroadrunner.com/advanced.html#field-centric-drive
        Vector2d fieldFrameInput = new Vector2d(
                -(gamepad1.left_stick_y * Y_SCALE + y_nudge),
                -(gamepad1.left_stick_x * X_SCALE + x_nudge)
        );

        Vector2d robotFrameInput = fieldFrameInput
                .rotated(-poseEstimate.getHeading())
                .rotated(Math.toRadians(-90));

        Vector2d vectorInput = new Vector2d();
        double headingInput = 0;
        switch (currentMode) {
            case DriverControl:
                if (gamepad1.square) {
                    currentMode = Mode.AlignToPoint;
                }

                vectorInput = fieldFrameInput;
                headingInput = -(gamepad1.right_stick_x * TURN_SCALE + ang_nudge);

                break;
            case AlignToPoint:
                if (gamepad1.triangle) {
                    currentMode = Mode.DriverControl;
                }

                Vector2d targetNudge = new Vector2d(
                        -(gamepad1.right_stick_y * Y_SCALE),
                        -(gamepad1.right_stick_x * X_SCALE)
                ).times(TARGET_NUDGE_AMOUNT).rotated(Math.toRadians(-90));

                targetPosition = targetPosition.plus(targetNudge);

                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                double theta = difference.angle();

                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                headingController.setTargetPosition(theta);

                vectorInput = robotFrameInput;
                headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Draw lines to target
                fieldOverlay.setStroke("#b89eff");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                fieldOverlay.setStroke("#ffce7a");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                break;
        }

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        // Set the drive train power using the weighted method so that
        // if the numbers end up too large, they are scaled to actually
        // work well with the drive train
        drive.setWeightedDrivePower(new Pose2d(vectorInput, headingInput));

        headingController.update(poseEstimate.getHeading());

        // Write some telemetry, which is sometimes useful for debugging
        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("heading", poseEstimate.getHeading());
    }

    /**
     *
     */
    private void operation(TelemetryPacket packet) {
        robot.spinLauncher(gamepad2.right_trigger, true);
        // Set the motor velocity for the launcher motor
//        motorVelo(packet, robot.launcherMotor, "launcher motor", fromMotorPower(LAUNCHER_POWER) * gamepad2.right_trigger);

        robot.spinLauncherAid(gamepad2.dpad_up, gamepad2.dpad_down);
//        double launcherAidPower = (gamepad2.dpad_up ? 1 : 0) * LAUNCHER_AID_POWER
//                + (gamepad2.dpad_down ? 1 : 0) * -LAUNCHER_AID_POWER;
//        robot.launcherAidMotor.setPower(launcherAidPower);

        // Check if the launcher button is pressed, and the launcher should run
        if (gamepad2.left_bumper && lastLaunchTime + LAUNCH_TIME * 2 < getRuntime()) {
            // Run the launcher and store the time that the launcher is run so that we can
            // know when to stop it and start it again
//            robot.launcherServo.setPosition(ON);
            robot.extendLauncherServo();
            lastLaunchTime = getRuntime();
        }

        // Check if it is time to stop the launcher
        if (lastLaunchTime + LAUNCH_TIME < getRuntime()) {
            // Stop the launcher
//            robot.launcherServo.setPosition(OFF);
            robot.retractLauncherServo();
        }

        // Check if the intake should be turned on
        if (gamepad2.right_bumper) {
            robot.enableIntake(true);
//            // Turn on the intake motor using velocity, and turn on the intake servo
//            motorVelo(packet, robot.intakeMotor, "intake motor", fromMotorPower(INTAKE_ON) * INTAKE_GEAR_RATIO);
//            robot.intakeServo.setPower(ON);
        } else {
            robot.disableIntake(true);
            // Turn off the intake motor, and turn off the intake servo
//            motorVelo(packet, robot.intakeMotor, "intake motor", OFF);
//            robot.intakeServo.setPower(OFF);
        }

        // Check if the claw button is being pressed and see if that state is different from the
        // state it was in the last loop. This is essentially just checking if there was a change
        // in state from the last loop to this one
        boolean clawButton = gamepad2.y;
        if (clawButton != previousClawButton) {
            // Check if the claw button is actually down. This makes sure our state is changed
            // only when the button is just pressed, and no other time
            if (clawButton) {
                // Invert the claw state
                clawState = !clawState;

                // Update the claw servo to reflect the new claw state
                if (clawState) {
//                    robot.clawServo.setPosition(ON);
                    robot.openClaw();
                } else {
//                    robot.clawServo.setPosition(OFF);
                    robot.closeClaw();
                }
            }

            // Store the current claw button state as the previous one to be use for the next loop
            previousClawButton = clawButton;
        }

        // Check if the arm button is being pressed and see if that state is different from the
        // state it was in the last loop. This is essentially just checking if there was a change
        // in state from the last loop to this one
        boolean armButton = gamepad2.b;
        if (armButton != previousArmButton) {
            // Check if the arm button is actually down. This makes sure our state is changed
            // only when the button is just pressed, and no other time
            if (armButton) {
                // Invert the arm state
                armState = !armState;

                // Update the arm servo to reflect the new arm state
                if (armState) {
//                    robot.armServo.setPosition(OFF);
                    robot.lowerWobbleArm();
                } else {
//                    robot.armServo.setPosition(ON);
                    robot.raiseWobbleArm();
                }
            }

            // Store the current arm button state as the previous one to be use for the next loop
            previousArmButton = armButton;
        }

        // Check if the intake button is being pressed and see if that state is different from the
        // state it was in the last loop. This is essentially just checking if there was a change
        // in state from the last loop to this one
        boolean intakeButton = gamepad2.a;
        if (intakeButton != previousIntakeButton) {
            // Check if the intake button is actually down. This makes sure our state is changed
            // only when the button is just pressed, and no other time
            if (intakeButton) {
                // Invert the intake state
                intakeState = !intakeState;

                // Update the intake servos to reflect the new intake state
                if (intakeState) {
//                    robot.leftIntakeServo.setPosition(ON);
//                    robot.rightIntakeServo.setPosition(ON);
                    robot.lowerIntake();
                } else {
//                    robot.leftIntakeServo.setPosition(OFF);
//                    robot.rightIntakeServo.setPosition(OFF);
                    robot.raiseIntake();
                }
            }

            // Store the current intake button state as the previous one to be use for the next loop
            previousIntakeButton = intakeButton;
        }
    }

//    /**
//     * @param motor
//     * @param name
//     * @param amount
//     */
//    private void motorVelo(TelemetryPacket packet, DcMotorEx motor, String name, double amount) {
//        // Set the motor's velocity to the specified amount
//        motor.setVelocity(amount);
//
//        // Get the current and target velocity that the motor should be at and calculate the percent error
//        double target = amount;
//        double actual = motor.getVelocity();
//        double error = Math.abs(target - actual) / target;
//
//        // Log all of the information to telemetry. This is often very useful for debugging
//        packet.put(String.format("%s target velocity", name), target);
//        packet.put(String.format("%s actual velocity", name), actual);
//        packet.put(String.format("%s velocity error", name), error);
//    }
}

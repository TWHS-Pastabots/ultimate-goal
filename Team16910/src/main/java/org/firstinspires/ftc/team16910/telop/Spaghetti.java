package org.firstinspires.ftc.team16910.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team16910.drive.DriveConstants;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.util.PoseStorage;
import org.firstinspires.ftc.team16910.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/**
 * TODO(BSFishy): document this
 */
@TeleOp(name = Spaghetti.NAME, group = Spaghetti.GROUP)
@Config
public class Spaghetti extends OpMode {
    public static final String NAME = "Spaghetti";
    public static final String GROUP = "Spaghetti";
    public static double DRAWING_TARGET_RADIUS = 2;
    public static Vector2d GOAL_POSITION = new Vector2d(72, 36);
    public static Vector2d POWER_SHOT_POSITION = new Vector2d(72, 12);
    // The small amount to move the robot for fine-tuned movements
    public static double NUDGE_AMOUNT = 0.25;
    public static double TARGET_NUDGE_AMOUNT = 1;
    public static double LAUNCH_TIME = 0.6;
    // Constants that scale the input power so movement isn't super jerky
    public static double X_SCALE = 0.9;
    public static double Y_SCALE = 0.9;
    public static double TURN_SCALE = 0.85;
    private final PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    // Hardware-related variables
    private final SpaghettiHardware robot = new SpaghettiHardware();
    private Mode currentMode = Mode.DriverControl;
    private Vector2d targetPosition = GOAL_POSITION;
    // Variables used for the launcher
    private double lastLaunchTime = 0;
    private SampleMecanumDrive drive;
    // State tracking variables for the intake
    private boolean previousIntakeButton = false;
    private boolean intakeState = true;
    // State tracking variables for the claw
    private boolean previousClawButton = false;
    private boolean clawState = false;
    // State tracking variables for the arm
    private boolean previousArmButton = false;
    private boolean armState = false;

    /**
     * TODO(BSFishy): document this
     *
     * @return the telemetry cast as a {@link TelemetryHelper}
     */
    private TelemetryHelper getTelemetry() {
        return (TelemetryHelper) telemetry;
    }

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new TelemetryHelper(telemetry, dashboard);

        // Initialize the robot hardware
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.position);

        headingController.setInputBounds(-Math.PI, Math.PI);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        getTelemetry().newPacket();

        // Separate the movement and operation code into two separate functions
        // so that it is easier to understand the separation
        movement();
        operation();

        // Update the telemetry
        telemetry.update();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void movement() {
        Canvas fieldOverlay = getTelemetry().getPacket().fieldOverlay();
        telemetry.addData("mode", currentMode);

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
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }

    /**
     * TODO(BSFishy): document this
     */
    private void operation() {
        // Set the motor velocity for the launcher motor
        robot.spinLauncher(gamepad2.right_trigger, true);

        robot.spinLauncherAid(gamepad2.dpad_up, gamepad2.dpad_down);

        // Check if the launcher button is pressed, and the launcher should run
        if (gamepad2.left_bumper && lastLaunchTime + LAUNCH_TIME * 2 < getRuntime()) {
            // Run the launcher and store the time that the launcher is run so that we can
            // know when to stop it and start it again
            robot.extendLauncherServo();
            lastLaunchTime = getRuntime();
        }

        // Check if it is time to stop the launcher
        if (lastLaunchTime + LAUNCH_TIME < getRuntime()) {
            // Stop the launcher
            robot.retractLauncherServo();
        }

        // Check if the intake should be turned on
        if (gamepad2.right_bumper) {
            // Turn on the intake motor using velocity, and turn on the intake servo
            robot.enableIntake(true);
        } else {
            // Turn off the intake motor, and turn off the intake servo
            robot.disableIntake(true);
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
                    robot.openClaw();
                } else {
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
                    robot.lowerWobbleArm();
                } else {
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
                    robot.lowerIntake();
                } else {
                    robot.raiseIntake();
                }
            }

            // Store the current intake button state as the previous one to be use for the next loop
            previousIntakeButton = intakeButton;
        }
    }

    private enum Mode {
        DriverControl,
        AlignToPoint,
    }
}

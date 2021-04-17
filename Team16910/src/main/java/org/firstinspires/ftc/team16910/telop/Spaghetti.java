package org.firstinspires.ftc.team16910.telop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16910.auton.SpaghettiAutonomous;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;

@TeleOp(name = "Spaghetti", group = "Linear OpMode")
@Config
public class Spaghetti extends OpMode {
    // The small amount to move the robot for fine-tuned movements
    public static double NUDGE_AMOUNT = 0.25;

    // Useful symbolic on and off constants
    public static double ON = 1;
    public static double OFF = 0;

    // Constants related to the intake
    public static double INTAKE_ON = 0.15;
    public static double INTAKE_GEAR_RATIO = 5;

    // Variables used for the launcher
    private double lastLaunchTime = 0;
    public static double LAUNCH_TIME = 0.6;
    public static double LAUNCHER_POWER = 0.6375;

    // Constants related to the motors we're using
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;

    /**
     * Convert the motor power level to encoder ticks per second. This takes a {@code [0, 1]} range power level
     * and manipulates its numeric value to convert it into a velocity that can be used by
     * {@link DcMotorEx#setVelocity(double)}.
     *
     * What this actually does is takes the maximum motor RPM from {@link #MOTOR_MAX_RPM} and converts it to
     * the max rotations per second by dividing it by {@code 60}. It then multiplies it by the power level specified
     * to get the target number of rotations per second and then multiplies it by {@link #MOTOR_TICKS_PER_SECOND}
     * to convert it to the number of ticks per second used by the motor.
     *
     * @param power the power level as a decimal
     * @return motor ticks per second
     */
    public static double motorPower(double power) {
        return MOTOR_MAX_RPM / 60 * power * MOTOR_TICKS_PER_SECOND;
    }

    // Constants that scale the input power so movement isn't super jerky
    public static double X_SCALE = 0.7;
    public static double Y_SCALE = 0.7;
    public static double TURN_SCALE = 0.85;

    // A position that is useful for launching rings from
    // TODO: this should be deleted because it's super not recommended
    public static Pose2d middlePosition = new Pose2d(-22.570475172975446, 53.49077586858332, Math.toRadians(-0.862970095));

    // Hardware-related variables
    private final SpaghettiHardware robot = new SpaghettiHardware();
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

    @Override
    public void init() {
        // Initialize the robot hardware
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(SpaghettiAutonomous.FINAL_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // Separate the movement and operation code into two separate functions
        // so that it is easier to understand the separation
        movement();
        operation();

        // Update the telemetry
        telemetry.update();
    }

    /**
     *
     */
    private void movement() {
        // Calculate nudge amounts for all of the axis
        double x_nudge = gamepad1.dpad_right ? NUDGE_AMOUNT : (gamepad1.dpad_left ? -NUDGE_AMOUNT : 0);
        double y_nudge = gamepad1.dpad_up ? -NUDGE_AMOUNT : (gamepad1.dpad_down ? NUDGE_AMOUNT : 0);
        double ang_nudge = gamepad1.right_bumper ? NUDGE_AMOUNT : (gamepad1.left_bumper ? -NUDGE_AMOUNT : 0);

        // Check if the drive train is busy, i.e. running a trajectory
        if (!drive.isBusy()) {
            // Set the drive train power using the weighted method so that
            // if the numbers end up too large, they are scaled to actually
            // work well with the drive train
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y * Y_SCALE + y_nudge),
                            -(gamepad1.left_stick_x * X_SCALE + x_nudge),
                            -(gamepad1.right_stick_x * TURN_SCALE + ang_nudge)
                    )
            );
        }

        if (gamepad1.b) {
            try {
                // NOTE: apparently this is like super not recommended, but I guess
                // we can leave it because it isn't actually required, just something
                // that is there and can be used if needed
                // TODO: remove this
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(middlePosition, Math.toRadians(90))
                        .build());
            } catch (Exception e) {
                telemetry.addData("Error trying to follow trajectory", e);
            }
        }

        // Try the update to the drive train. This is surrounded by a try
        // block in case anything goes wrong in RoadRunner and it, for
        // whatever reason, throws an error. This makes sure the teleop
        // and/or phone doesn't crash, which actually happened to us
        // in a match :(
        try {
            // Update the drive train to actually set the motor powers
            drive.update();
        } catch (Exception e) {
            telemetry.addData("Error trying to update drive", e);
        }

        // Write some telemetry, which is sometimes useful for debugging
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }

    /**
     *
     */
    private void operation() {
        // Set the motor velocity for the launcher motor
        motorVelo(robot.launcherMotor, "launcher motor", motorPower(LAUNCHER_POWER) * gamepad2.right_trigger);

        // Check if the launcher button is pressed, and the launcher should run
        if (gamepad2.left_bumper && lastLaunchTime + LAUNCH_TIME * 2 < getRuntime()) {
            // Run the launcher and store the time that the launcher is run so that we can
            // know when to stop it and start it again
            robot.launcherServo.setPosition(ON);
            lastLaunchTime = getRuntime();
        }

        // Check if it is time to stop the launcher
        if (lastLaunchTime + LAUNCH_TIME < getRuntime()) {
            // Stop the launcher
            robot.launcherServo.setPosition(OFF);
        }

        // Check if the intake should be turned on
        if (gamepad2.right_bumper) {
            // Turn on the intake motor using velocity, and turn on the intake servo
            motorVelo(robot.intakeMotor, "intake motor", motorPower(INTAKE_ON) * INTAKE_GEAR_RATIO);
            robot.intakeServo.setPower(ON);
        } else {
            // Turn off the intake motor, and turn off the intake servo
            motorVelo(robot.intakeMotor, "intake motor", OFF);
            robot.intakeServo.setPower(OFF);
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
                    robot.clawServo.setPosition(ON);
                } else {
                    robot.clawServo.setPosition(OFF);
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
                    robot.armServo.setPosition(OFF);
                } else {
                    robot.armServo.setPosition(ON);
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
                    robot.leftIntakeServo.setPosition(ON);
                    robot.rightIntakeServo.setPosition(ON);
                } else {
                    robot.leftIntakeServo.setPosition(OFF);
                    robot.rightIntakeServo.setPosition(OFF);
                }
            }

            // Store the current intake button state as the previous one to be use for the next loop
            previousIntakeButton = intakeButton;
        }
    }

    /**
     *
     * @param motor
     * @param name
     * @param amount
     */
    private void motorVelo(DcMotorEx motor, String name, double amount) {
        // Set the motor's velocity to the specified amount
        motor.setVelocity(amount);

        // Get the current and target velocity that the motor should be at and calculate the percent error
        double target = amount;
        double actual = motor.getVelocity();
        double error = Math.abs(target - actual) / target;

        // Log all of the information to telemetry. This is often very useful for debugging
        telemetry.addData(String.format("%s target velocity", name), target);
        telemetry.addData(String.format("%s actual velocity", name), actual);
        telemetry.addData(String.format("%s velocity error", name), error);
    }
}

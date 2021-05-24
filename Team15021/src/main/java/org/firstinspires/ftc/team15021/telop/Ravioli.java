package org.firstinspires.ftc.team15021.telop;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.PoseStorage;
import org.firstinspires.ftc.team15021.drive.DriveConstants;
import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.teamcode.Util;

import java.text.DecimalFormat;
import java.util.Vector;


@TeleOp(name = "Ravioli", group = "Linear OpMode")
public class Ravioli extends LinearOpMode {
    /* Declare OpMode members. */
    final RavioliHardware robot = new RavioliHardware();
    private SampleMecanumDrive drive;
    final ElapsedTime runTime = new ElapsedTime();

    private final double ON = 1.0;
    private final double OFF = 0.0;

    private boolean open = true;

    private String front;
    private String back;
    private String right;
    private String left;

    double slowCon = 1.0;

    double vLauncher = 0.0;
    double vLauncherMult = 0.70;
    double vConveyor = 0.0;
    double vServoMotor = 0.0;
    int clawPos = 30;

    private double moveMult;

    // Constants related to the motors we're using
    public static double MOTOR_MAX_RPM = 6000;
    public static double MOTOR_TICKS_PER_SECOND = 28;

    private Trajectory toShoot;

    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d shootPoint = new Vector2d(72, 24);

    // True if in alignment mode and false if in normal mode
    private boolean setMode = false;


    // Loop on start()
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        localizer.setPoseEstimate(PoseStorage.currentPose);

        if (isStopRequested()) return;

        moveMult = .75;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");

        initTrajectories();

        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        runTime.reset();
        telemetry.addData("Run Time", "reset");

        telemetry.speak("Nice Cock!");
        telemetry.update();

        waitForStart();

        int clawTarget = 30;
        double clawSpeed = .8;

        while (opModeIsActive() && !isStopRequested()) {

            // Read pose
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if left and right bumpers are pressed
                    if (gamepad1.left_bumper && gamepad1.right_bumper && setMode == false) {
                        currentMode = Mode.ALIGN_TO_POINT;
                        setMode = true;
                    }

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if left and right bumpers are pressed
                    if (gamepad1.left_bumper && gamepad1.right_bumper && setMode == true) {
                        currentMode = Mode.NORMAL_CONTROL;
                        setMode = false;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = shootPoint.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    break;
            }

            // Motor multiplier
            if (gamepad1.b) slowCon = 0.25;
            if (gamepad1.a) slowCon = 0.5;
            if (gamepad1.x) slowCon = 0.75;
            if (gamepad1.y) slowCon = 1.0;


            if (gamepad2.square) vLauncherMult = 0.6;
            if (gamepad2.circle) vLauncherMult = 0.65;
            if (gamepad2.triangle) vLauncherMult = 0.70;

            if (gamepad2.left_bumper)
                vLauncher = -gamepad2.right_trigger * vLauncherMult;
            else
                vLauncher = gamepad2.right_trigger * vLauncherMult;

            if (gamepad2.dpad_up) vConveyor = 1.0;
            else if (gamepad2.dpad_down) vConveyor = -1.0;
            else if (gamepad1.left_trigger > 0) vConveyor = -1.0;
            else if (gamepad1.right_trigger > 0) vConveyor = 1.0;
            else vConveyor = 0.0;

            // robot.servoClaw.setPosition((1 - gamepad2.left_trigger) / 2);
            if (gamepad2.left_trigger > 0.5) {
                robot.servoClaw.setPosition(OFF);
                open = true;
            }
            else if (gamepad2.left_trigger < 0.5) {
                robot.servoClaw.setPosition(.5);
                open = false;
            }

            if (gamepad2.right_stick_y > .5 && clawPos <= 170) {
                clawPos++;
            } else if (gamepad2.right_stick_y < -.5 && clawPos >= 30) {
                clawPos--;
            }

            if (Math.abs(clawTarget - robot.motorClaw.getCurrentPosition()) < 30) {
                clawSpeed = .5;
            } else {
                clawSpeed = .8;
            }

//            robot.motorClaw.setTargetPositionTolerance(30);
//            robot.motorClaw.setTargetPosition(clawPos);
//            robot.motorClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.motorClaw.setPower(1);
            robot.motorClaw.setPower(gamepad2.right_stick_y);

            if (gamepad2.cross) robot.servoIntake.setPosition(0.6);
            else robot.servoIntake.setPosition(0);


            // Sets power of motor, spins wheels
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y * moveMult),
                            -(gamepad1.left_stick_x * moveMult),
                            -(gamepad1.right_stick_x * moveMult)
                    )
            );

            // Sets powers of Launcher and Conveyor Motors
            //robot.motorLauncher.setPower(vLauncher);
            robot.motorLauncher.setPower(vLauncher);
            robot.motorConveyor.setPower(vConveyor);
            robot.motorIntake.setPower(vConveyor);

            robot.motorClaw.setPower(vServoMotor);

            // Show motor output visually
            telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
            // telemetry.addLine(Telemetry.createLevel((float) v1));
            // telemetry.addLine(Telemetry.createLevel((float) v2));
            // telemetry.addLine(Telemetry.createLevel((float) v3));
            // telemetry.addLine(Telemetry.createLevel((float) v4));
            telemetry.addData("Launcher Speed: ", vLauncher);


            drive.update();
            telemetry.update();
        }
    }

    public static double fromMotorPower ( double power){
//        return MOTOR_MAX_RPM / 60 * power * MOTOR_TICKS_PER_SECOND;
        return fromMotorPower(power, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    public static double fromMotorPower ( double power, double maxRpm, double ticksPerSecond){
        return maxRpm / 60 * power * ticksPerSecond;
    }

    public static double toMotorPower ( double ticks){
//        return ticks / MOTOR_TICKS_PER_SECOND / (MOTOR_MAX_RPM / 60);
        return toMotorPower(ticks, MOTOR_MAX_RPM, MOTOR_TICKS_PER_SECOND);
    }

    public static double toMotorPower ( double ticks, double maxRpm, double ticksPerSecond){
        return ticks / ticksPerSecond / (maxRpm / 60);
    }

    private void initTrajectories () {

        // Trajectory to go to shooting position
        toShoot = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0, 30))
                .build();
    }
}
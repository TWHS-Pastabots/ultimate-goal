package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.SpaghettiHardware;
import org.firstinspires.ftc.teamcode.telop.Spaghetti;

@Config
@Autonomous(name = "Spaghetti Autonomous", preselectTeleOp = "Spaghetti")
public class SpaghettiAutonomous extends LinearOpMode {
    public static double ON = 1;
    public static double OFF = 0;

    //    public static double ARM_DEGREES = 90;
//    public static double CLAW_DEGREES = 90;
    public static double ARM_TIME = 500;
    public static double CLAW_TIME = 250;

    public static double LAUNCHER_POWER = 0.56;
    public static double LAUNCHER_SPINUP = 1000;
    public static double LAUNCHER_LAUNCH = 250;

//    public static double SERVO_SPEED = 0.002333333;

    public static Pose2d FINAL_POSITION = new Pose2d(12, 18, 0);

    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;

    private Trajectory toWobbleGoal, toPowerShot1, toPowerShot2, toPowerShot3, toFinish;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-60, 49, 0));

        // Prepare the trajectories needed throughout the opmode
        prepareTrajectories();

        // Initialize the arm
//        robot.armServo.setPosition(ON);
        robot.clawServo.setPosition(ON);

        // Wait until the opmode should start
        waitForStart();

        // Move to the wobble goal and drop it
        drive.followTrajectory(toWobbleGoal);
        dropWobbleGoal();

        // Move to the powershot goal and shoot the rings
        drive.followTrajectory(toPowerShot1);
        launchPowerShots();

        // Move to the finish position and end the opmode
        finish();
    }

    /**
     *
     */
    private void prepareTrajectories() {
        toWobbleGoal = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-6, 49), 0)
                .build();

        toPowerShot1 = drive.trajectoryBuilder(toWobbleGoal.end())
                .forward(-6)
                .splineToLinearHeading(new Pose2d(-6, 38, 0), 0)
                .build();

        toPowerShot2 = drive.trajectoryBuilder(toPowerShot1.end())
                .strafeRight(8)
                .build();

        toPowerShot3 = drive.trajectoryBuilder(toPowerShot2.end())
                .strafeRight(8)
                .build();

        toFinish = drive.trajectoryBuilder(toPowerShot3.end())
                .splineToLinearHeading(FINAL_POSITION, 0)
                .build();
    }

    /**
     *
     */
    private void dropWobbleGoal() {
        robot.armServo.setPosition(OFF);
        sleep((long) ARM_TIME);

        robot.clawServo.setPosition(OFF);
        sleep((long) CLAW_TIME);

        robot.armServo.setPosition(ON);
        sleep((long) ARM_TIME);
    }

    /**
     *
     */
    private void launchPowerShots() {
        robot.launcherMotor.setVelocity(Spaghetti.motorPower(LAUNCHER_POWER));
//        robot.launcherMotor.setPower(LAUNCHER_POWER);
        sleep((long) LAUNCHER_SPINUP);

        launch();
        drive.followTrajectory(toPowerShot2);
        launch();
        drive.followTrajectory(toPowerShot3);
        launch();

        robot.launcherMotor.setPower(OFF);
        sleep((long) LAUNCHER_SPINUP);
    }

    /**
     *
     */
    private void launch() {
        robot.launcherServo.setPosition(ON);
        sleep((long) LAUNCHER_LAUNCH);
        robot.launcherServo.setPosition(OFF);
        sleep((long) LAUNCHER_LAUNCH);
    }

    /**
     *
     */
    private void finish() {
        drive.followTrajectory(toFinish);

        while (!isStopRequested()) {
            idle();
        }
    }
}

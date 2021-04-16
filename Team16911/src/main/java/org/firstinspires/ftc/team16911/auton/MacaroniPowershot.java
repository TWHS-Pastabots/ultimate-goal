package org.firstinspires.ftc.team16911.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.MacaroniHardware;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Macaroni Powershot", group = "drive", preselectTeleOp = "Macaroni")
public class MacaroniPowershot extends LinearOpMode {
    public static final Pose2d START_POSITION = new Pose2d(-(72 - 10.5), 24);

    private static final double LAUNCHER_POWER = 0.73;
    private static final int PRESPIN_TIME = 4300;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MacaroniHardware robot = new MacaroniHardware();
        robot.init(hardwareMap);

        drive.setPoseEstimate(START_POSITION);

        Trajectory moveToWobblePlacement = drive.trajectoryBuilder(START_POSITION)
                .splineToSplineHeading(new Pose2d(12, 38, Math.toRadians(90)), 0)
                .build();

        Trajectory moveBehindShootingLine = drive.trajectoryBuilder(moveToWobblePlacement.end())
                .lineToSplineHeading(new Pose2d(0, 60, Math.toRadians(45)))
                .build();

        Trajectory parkOnLine = drive.trajectoryBuilder(moveBehindShootingLine.end())
                .lineTo(new Vector2d(12, 60))
                .build();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        // Drive forward
        telemetry.addData("Status", "Driving towards wobble goal placement position");
        drive.followTrajectory(moveToWobblePlacement);

        // Lower arms and drop wobble goal
        telemetry.addData("Status", "Lowering arm");
        robot.armServo.setPosition(0.2);
        sleep(700);

        telemetry.addData("Status", "Opening claws");
        robot.setClaws(false);
        sleep(500);

        telemetry.addData("Status", "Re-opening claws");
        robot.setClaws(0.2);
        sleep(250);

        telemetry.addData("Status", "Raising arm");
        robot.armServo.setPosition(1);
        sleep(400);

        // Drive back to launching line
        telemetry.addData("Status", "Moving towards shooting position");
        drive.followTrajectory(moveBehindShootingLine);

        // Prepare for shooting sequence using prespin
        robot.setLauncherPower(LAUNCHER_POWER);
        robot.setRingStopper(true);
        telemetry.addData("Status", "Pre-shooting sequence motor spinup");
        sleep(PRESPIN_TIME);
        // Pre-spin ended, shoot them using belt
        robot.setRingStopper(false);
        sleep(250);
        robot.setIntakePower(0.4, 0);
        telemetry.addData("Status", "Shooting sequence");
        sleep(3500);


        // End shooting sequence, spin down & stop
        robot.setLauncherPower(0);
        robot.setIntakePower(0, 0);
        telemetry.addData("Status", "Ending shooting sequence");
        sleep(500);

        // Move back and park on line
        telemetry.addData("Status", "Moving to parking line");
        drive.followTrajectory(parkOnLine);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Status", "Idle");
            idle();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}


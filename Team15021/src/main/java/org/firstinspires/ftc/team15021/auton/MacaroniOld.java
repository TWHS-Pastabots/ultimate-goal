package org.firstinspires.ftc.team15021.auton;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MacaroniDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class MacaroniOld extends LinearOpMode {

    private static final double LAUNCHER_POWER = 0.80;
    private static final int PRESPIN_TIME = 2500;
    private static final int RESPIN_TIME = 1500;
    private static final int RAMP_TIME = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        MacaroniDrive drive = new MacaroniDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 24, 0);
        drive.setPoseEstimate(startPose);
//
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(90)
//                .lineTo(new Vector2d(36, 24))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(34)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(18)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(7)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Drive forward
        drive.followTrajectory(traj1);

        // Lower arms and drop wobble goal
        sleep(400);
        drive.armServo.setPosition(0.6);
        sleep(700);
        drive.setClaws(1);
        sleep(1100);

        // Drive back to launching line
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        drive.launcherMotor.setPower(Math.max(1, LAUNCHER_POWER + 0.15));
        sleep(PRESPIN_TIME - 500);
        drive.launcherMotor.setPower(LAUNCHER_POWER);
        sleep(500);

        runRamp(drive, RAMP_TIME);
        sleep(RESPIN_TIME);

        runRamp(drive, RAMP_TIME);
        sleep(RESPIN_TIME);

        runRamp(drive, RAMP_TIME);
        sleep(250);

        drive.launcherMotor.setPower(0);
        sleep(500);

        drive.followTrajectory(traj4);

        while (!isStopRequested() && opModeIsActive()) {
            idle();
        };

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }

    public void runRamp(MacaroniDrive drive, float duration) {
        drive.beltMotor.setPower(1);
        sleep((long) duration);
        drive.beltMotor.setPower(0);

    }
}


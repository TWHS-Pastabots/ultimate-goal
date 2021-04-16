package org.firstinspires.ftc.team16911.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive", preselectTeleOp = "Macaroni v2")
@Disabled
public class MacaroniTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60, 24, 0);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(90)
//                .lineTo(new Vector2d(36, 24))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(31)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(18)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(7)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
        drive.followTrajectory(traj3);
        sleep(1000);
        drive.followTrajectory(traj4);
        sleep(1000);

        while (!isStopRequested() && opModeIsActive()) {
            idle();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}


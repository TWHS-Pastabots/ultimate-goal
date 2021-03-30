package org.firstinspires.ftc.team16911.auton;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.MacaroniHardware;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class MacaroniBasic extends LinearOpMode {

    private static final double LAUNCHER_POWER = 0.80;
    private static final int PRESPIN_TIME = 2500;
    private static final int RESPIN_TIME = 1500;
    private static final int RAMP_TIME = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MacaroniHardware robot = new MacaroniHardware();
        robot.init(hardwareMap);

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

        // Drive forward
        drive.followTrajectory(traj1);

        // Lower arms and drop wobble goal
        sleep(400);
        robot.armServo.setPosition(0.6);
        sleep(700);
//        robot.leftClawServo.setPosition(1);
        sleep(1100);

        // Drive back to launching line
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        robot.launcherMotor.setPower(Math.max(1, LAUNCHER_POWER + 0.15));
        sleep(PRESPIN_TIME - 500);
        robot.launcherMotor.setPower(LAUNCHER_POWER);
        sleep(500);

        robot.beltMotor.setPower(1);
        sleep((long) RAMP_TIME);
        robot.beltMotor.setPower(0);
        sleep(RESPIN_TIME);

        robot.beltMotor.setPower(1);
        sleep((long) RAMP_TIME);
        robot.beltMotor.setPower(0);
        sleep(RESPIN_TIME);

        robot.beltMotor.setPower(1);
        sleep((long) RAMP_TIME);
        robot.beltMotor.setPower(0);
        sleep(250);

        robot.launcherMotor.setPower(0);
        sleep(500);

        drive.followTrajectory(traj4);

        while (!isStopRequested() && opModeIsActive()) {
            idle();
        }
        ;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();
    }
}


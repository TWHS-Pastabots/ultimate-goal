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
    public static final Pose2d START_POSITION = new Pose2d(-(72 - 10.5), 24);

    private static final double LAUNCHER_POWER = 0.875;
    private static final int PRESPIN_TIME = 3500;
    private static final int RESPIN_TIME = 1500;
    private static final int RAMP_TIME = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MacaroniHardware robot = new MacaroniHardware();
        robot.init(hardwareMap);

        drive.setPoseEstimate(START_POSITION);

        Trajectory traj1 = drive.trajectoryBuilder(START_POSITION)
                .splineToLinearHeading(new Pose2d(36, 14), 0)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(0, 18), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(12, 24), 0)
                .build();

        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        // Drive forward
        telemetry.addData("Status", "Driving towards wobble goal placement position");
        drive.followTrajectory(traj1);

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
        drive.followTrajectory(traj2);

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
        drive.followTrajectory(traj3);

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


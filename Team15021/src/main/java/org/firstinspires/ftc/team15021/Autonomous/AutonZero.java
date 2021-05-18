package org.firstinspires.ftc.team15021.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.PoseStorage;
import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;


@Autonomous(name="0 Rings", preselectTeleOp = "Ravioli")
public class AutonZero extends LinearOpMode {

    public SampleMecanumDrive drive;
    private RavioliHardware robot;

    // Trajectories
    private Trajectory toWobble, toShoot, toFinish;

    // Constants
    private final double ON = 1;
    private final double OFF = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        // Instantiate robot
        robot = new RavioliHardware();
        robot.init(hardwareMap);

        // Instantiate drive and set start position
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-59.5418686772355, 27.493293851154135, Math.toRadians(358.0072303406526)));


        // Do shit
        initTrajectories();
        waitForStart();

        if (isStopRequested()) {
            return;
        }


        // All movements and doing things
        wobbleStuff();
        shootShots();
        drive.followTrajectory(toFinish);

        PoseStorage.currentPose = drive.getPoseEstimate();

    }

    // Initialize trajectories
    private void initTrajectories() {

        // Trajectory to drop wobble goal
        toWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(0, 54), Math.toRadians(45))
                .build();

        // Trajectory to shooting position
        toShoot = drive.trajectoryBuilder(toWobble.end())
                .splineToConstantHeading(new Vector2d(0, 27), 0)
                .build();

        // Trajectory to park robot
        toFinish = drive.trajectoryBuilder(toShoot.end())
                .forward(12)
                .build();

    }

    // Drop wobble goal
    private void dropWobble() {

        // Drop wobble goal
        robot.motorClaw.setPower(.4);
        sleep(1000);

        // Open claw
        robot.servoClaw.setPosition(.5);
        sleep(1000);

        // Return claw to original place
        robot.motorClaw.setPower(OFF);

    }

    // Do all wobble goal maneuvers
    private void wobbleStuff() {

        drive.followTrajectory(toWobble);
        dropWobble();

    }

    // Shoots high goal shots
    private void shootShots() {
        drive.followTrajectory(toShoot);
        robot.motorLauncher.setPower(.75);
        sleep(2000);
        robot.motorConveyor.setPower(.4);
        sleep(5000);

    }

}

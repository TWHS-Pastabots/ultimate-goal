package org.firstinspires.ftc.team15021.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.team15021.telop.Ravioli;


@Autonomous(name="Autonomous", preselectTeleOp = "Ravioli")
public class Auton extends LinearOpMode {

    public SampleMecanumDrive drive;
    private RavioliHardware robot;
    public ElapsedTime runTime = new ElapsedTime();

    private Trajectory toWobble, toShoot, toFinish;

    private final double ON = 1;
    private final double OFF = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RavioliHardware();
        robot.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-60.86617465525808, 32.65526013333349));

        // Wobble Goal 0: x: -1.3186793572157305, y: 44.95495961680901
        // High Goal Shot: x: -13.12811530133888, y: 38.411333789792316, heading (deg): 342.51777814868103
        // Finish: x: 11.532724155823113, y: 26.463119291587887

        initTrajectories();



        waitForStart();

        if (isStopRequested()) {
            return;
        }


        drive.followTrajectory(toWobble);
        dropWobble();
        sleep(1000);
        drive.followTrajectory(toShoot);
        shootShots();
        sleep(1000);
        drive.followTrajectory(toFinish);


    }

    // Initialize trajectories
    public void initTrajectories() {
        toWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(-1.3186793572157305, 44.95495961680901), Math.toRadians(45))
                .build();

        toShoot = drive.trajectoryBuilder(toWobble.end())
                .splineToConstantHeading(new Vector2d(1.2508025205683824, -2.160644337696294), Math.toRadians(45))
                .build();

        toFinish = drive.trajectoryBuilder(toShoot.end())
                .splineToConstantHeading(new Vector2d(11.532724155823113, 26.463119291587887), 0)
                .build();
    }


    public void shootShots() {

        robot.motorLauncher.setVelocity(Ravioli.fromMotorPower(.59));
        sleep(3250);
        robot.motorConveyor.setPower(.4);
        sleep(1750);
        robot.motorConveyor.setPower(OFF);
        robot.motorConveyor.setPower(.4);
        sleep(1750);
        robot.motorConveyor.setPower(OFF);
        robot.motorConveyor.setPower(.4);
        sleep(2000);
        robot.motorLauncher.setPower(OFF);
        robot.motorConveyor.setPower(OFF);

    }

    public void dropWobble() {
        robot.motorClaw.setPower(.4);
        sleep(1000);
        robot.servoClaw.setPosition(.5);
        sleep(1000);
        robot.motorClaw.setPower(OFF);
    }
}

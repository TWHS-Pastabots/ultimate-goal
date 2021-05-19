package org.firstinspires.ftc.team15021.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.PoseStorage;
import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;


@Autonomous(name="1 Ring", preselectTeleOp = "Ravioli")
public class AutonOne extends LinearOpMode {

    public SampleMecanumDrive drive;
    private RavioliHardware robot;

    // Trajectories
    private Trajectory toWobble, toShoot, toPickup, adjustPickup, shootLast, toFinish;

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
        pickupRing();
        shootFourth();
        drive.followTrajectory(toFinish);

        PoseStorage.currentPose = drive.getPoseEstimate();

    }

    // Initialize trajectories
    public void initTrajectories() {

        // Trajectory to wobble drop off area
        toWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(24, 30), Math.toRadians(90))
                .build();

        // Trajectory to shoot
        toShoot = drive.trajectoryBuilder(toWobble.end())
                .splineToConstantHeading(new Vector2d(0, 30), 0)
                .build();

        // Trajectory to pick up rings
        toPickup = drive.trajectoryBuilder(toShoot.end())
                .splineToConstantHeading(new Vector2d(-6, 40), Math.toRadians(135))
                .build();

        // Trajectory to adjust the pickup location
        adjustPickup = drive.trajectoryBuilder(toPickup.end())
                .forward(-9)
                .build();

        // Shoots final shot
        shootLast = drive.trajectoryBuilder(adjustPickup.end())
                .lineToConstantHeading(new Vector2d(0, 30))
                .build();

        // Trajectory to park
        toFinish = drive.trajectoryBuilder(shootLast.end())
                .lineToConstantHeading(new Vector2d(11.532724155823113, 26.463119291587887))
                .build();

    }

    // Drop wobble goal
    public void dropWobble() {

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
    public void wobbleStuff() {

        drive.followTrajectory(toWobble);
        dropWobble();

    }

    // Does the extra ring shit
    private void pickupRing() {

        // Drive to position
        drive.followTrajectory(toPickup);


        // Adjust the positioning to intake
        drive.followTrajectory(adjustPickup);

        // Drop intake
        robot.servoIntake.setPosition(.7);
        sleep(250);

        // Power conveyor belt and intake motor to take in 4th ring
        robot.motorConveyor.setPower(ON);
        robot.motorIntake.setPower(ON);

        // Ring is being taken up here
        sleep(2000);

        // Power conveyor off and reset servo
        robot.motorConveyor.setPower(OFF);
        robot.servoIntake.setPosition(OFF);
        robot.motorIntake.setPower(OFF);

    }

    // Shoots high goal shots
    private void shootShots() {
        drive.followTrajectory(toShoot);
        robot.motorLauncher.setPower(.75);
        sleep(2000);
        robot.motorConveyor.setPower(.4);
        sleep(3500);

    }

    // Shoots fourth shot
    private void shootFourth() {

        // Goes to shoot position
        drive.followTrajectory(shootLast);

        // Powers conveyor motor to shoot for 3 seconds
        robot.motorConveyor.setPower(ON);
        sleep(2000);
        robot.motorConveyor.setPower(OFF);

    }

}

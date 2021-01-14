package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * This is a simple routine to test turning capabilities.
 */
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        MecanumDriveTrain drive = new MecanumDriveTrain();
        drive.init(robot);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}

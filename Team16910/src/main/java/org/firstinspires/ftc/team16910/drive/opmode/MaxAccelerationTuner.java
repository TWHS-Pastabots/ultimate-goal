package org.firstinspires.ftc.team16910.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;

import java.util.Objects;

@Config
@Autonomous(group = "drive")
@Disabled
public class MaxAccelerationTuner extends LinearOpMode {
    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double maxAcceleration = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(1, 0, 0));
        timer = new ElapsedTime();

        ElapsedTime loopTime = new ElapsedTime();
        double previousVelocity = 0;

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            double velocity = poseVelo.vec().norm();

//            maxAcceleration = Math.max(poseVelo.vec().norm(), maxAcceleration);
            double acceleration = (velocity - previousVelocity) / loopTime.seconds();
            maxAcceleration = Math.max(acceleration, maxAcceleration);
            previousVelocity = velocity;

            loopTime.reset();
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Acceleration", maxAcceleration);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }
}

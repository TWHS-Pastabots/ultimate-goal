package org.firstinspires.ftc.team16910.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
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
public class MaxAngularAccelerationTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngAcceleration = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        ElapsedTime loopTime = new ElapsedTime();
        double previousVelocity = 0;

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            double velocity = poseVelo.vec().norm();

//            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
            double acceleration = (velocity - previousVelocity) / loopTime.seconds();
            maxAngAcceleration = Math.max(acceleration, maxAngAcceleration);
            previousVelocity = velocity;

            loopTime.reset();
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Acceleration (rad)", maxAngAcceleration);
        telemetry.addData("Max Angular Acceleration (deg)", Math.toDegrees(maxAngAcceleration));
        telemetry.update();

        while (!isStopRequested()) idle();
    }
}

package org.firstinspires.ftc.team16910.telop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.auton.SpaghettiAutonomous;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.util.MotorUtil;

/**
 * TODO(BSFishy): document this
 */
@Config
@TeleOp(name = "Launcher Tester", group = Spaghetti.GROUP)
public class LauncherTester extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SpaghettiHardware robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        telemetry.addLine("Press start to start testing");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        boolean spinned = false;
        boolean launching = false;
        boolean extending = false;
        boolean retracting = false;

        ElapsedTime launchTimer = new ElapsedTime();
        ElapsedTime launcherTimer = new ElapsedTime();
        ElapsedTime stabilizationTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double velocity = robot.launcherMotor.getVelocity();
            double error = Math.abs(SpaghettiAutonomous.LAUNCHER_TARGET - velocity) / SpaghettiAutonomous.LAUNCHER_TARGET;

            telemetry.addData("spinned", spinned ? "true" : "false");
            telemetry.addData("actual speed", "%.02f", MotorUtil.toMotorPower(velocity));
            telemetry.addData("error", "%.02f%%", error * 100);

            if (launching) {
                robot.launcherMotor.setVelocity(MotorUtil.fromMotorPower(SpaghettiAutonomous.LAUNCHER_POWER));
                telemetry.addData("target speed", "%.02f", SpaghettiAutonomous.LAUNCHER_POWER);

                if (!extending && !retracting) {
                    telemetry.addLine("Launching");
                    telemetry.addData("launcher timer", launcherTimer.toString());

                    if (launcherTimer.seconds() < SpaghettiAutonomous.LAUNCHER_SPINUP) {
                        telemetry.addData("stabilization timer", stabilizationTimer.toString());

                        if (error <= SpaghettiAutonomous.LAUNCHER_THRESHOLD) {
                            if (stabilizationTimer.seconds() >= SpaghettiAutonomous.LAUNCHER_STABILIZATION_TIMEOUT) {
                                spinned = true;
                                extending = true;

                                launchTimer.reset();
                            }
                        } else {
                            stabilizationTimer.reset();
                        }
                    } else {
                        spinned = false;
                        extending = true;

                        launchTimer.reset();
                    }
                }

                if (extending) {
                    telemetry.addLine("Extending");
                    telemetry.addData("launch timer", launcherTimer.toString());

                    if (launchTimer.milliseconds() < SpaghettiAutonomous.LAUNCHER_LAUNCH) {
                        robot.launcherServo.setPosition(SpaghettiAutonomous.ON);
                    } else {
                        extending = false;
                        retracting = true;

                        launchTimer.reset();
                    }
                }

                if (retracting) {
                    telemetry.addLine("Retracting");
                    telemetry.addData("launch timer", launcherTimer.toString());

                    if (launchTimer.milliseconds() < SpaghettiAutonomous.LAUNCHER_LAUNCH) {
                        robot.launcherServo.setPosition(SpaghettiAutonomous.OFF);
                    } else {
                        launching = false;
                        retracting = false;
                    }
                }
            } else {
                telemetry.addData("target speed", "%.02f", 0.0);
                robot.launcherMotor.setVelocity(0);

                if (gamepad1.a) {
                    launching = true;
                    extending = false;
                    retracting = false;

                    launchTimer.reset();
                    launcherTimer.reset();
                    stabilizationTimer.reset();
                }
            }

            telemetry.update();
        }
    }
}

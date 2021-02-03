package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name = "Macaroni", group = "Linear OpMode")
public class Macaroni extends LinearOpMode {

    /* Declare OpMode members. */
    final MacaroniHardware robot = new MacaroniHardware();
    final ElapsedTime runTime = new ElapsedTime();
    double lastControlsUpdate;
    double slowCon = 0.8;
    boolean toggleLauncher = false;
    double launcherPower = 1f;
    private static final double powerGranularity = 0.05;

    boolean lowerWobble = false;
    boolean upperWobble = false;
    boolean lastLowerWobbleState = false;
    boolean lastUpperWobbleState = false;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        lastControlsUpdate = runTime.seconds();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateDrivetrainMotors();
            telemetry.update();
            updateServos();
            updateIntake();

            // Poll the controller's inputs 0.33 seconds after the previous poll
            if ((gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.back || gamepad1.dpad_left || gamepad1.dpad_right) && controllerLoop(0.15)) {
                if (gamepad1.dpad_left)
                    launcherPower = 0.65;
                else if (gamepad1.dpad_right)
                    launcherPower = 0.75;

                // Change launcher motor power with D-Pad controls
                double powerChange = gamepad1.dpad_up ? powerGranularity : (gamepad1.dpad_down ? -powerGranularity : 0f);
                launcherPower = Util.clamp((float) (launcherPower + powerChange), 0, 1);

                // toggle launcher motor with back button
                if (gamepad1.share) toggleLauncher = !toggleLauncher;
            }


            // Gamepad X to spin up launcher motor
            if (gamepad1.square) robot.launcherMotor.setPower(1);
            else if (toggleLauncher) robot.launcherMotor.setPower(launcherPower);
            else robot.launcherMotor.setPower(0);


            // Show motor output visually
        }
    }

    private void updateIntake() {
        // Right trigger to run all intake motors/servos at desired speed
        if (gamepad1.right_trigger > 0) {
            double rt = Util.cubicEasing(gamepad1.right_trigger);
            robot.beltMotor.setPower(rt);
            robot.intakeMotor.setPower(rt);
        } else if (gamepad1.left_trigger > 0) {
            double lt = gamepad1.left_trigger;
            robot.beltMotor.setPower(-lt);
            robot.intakeMotor.setPower(-lt);
        } else {
            robot.intakeMotor.setPower(0);
            robot.beltMotor.setPower(0);
        }
    }

    /**
     * A simple helper function that assists with spacing out controller input polls.
     * <p>
     * Without a proper guard in place, controller input code for button presses will be ran hundreds of
     * times per second, resulting in strange behavior. By adding a guard that only lets the button input
     * code run every other fraction of a second, button input is much more reliable for humans.
     *
     * @param duration The minimum duration that must be elapsed before the controller loop is ran.
     * @return True if the controller should be polled now, false if the duration has not yet elapsed.
     */
    public boolean controllerLoop(double duration) {
        double now = runTime.seconds();
        if (now - lastControlsUpdate >= duration) {
            this.lastControlsUpdate = now;
            return true;
        }
        return false;
    }

    public void updateServos() {
        if (gamepad1.circle && !lastLowerWobbleState) {
            robot.lowerWobbleServo.setPosition(lowerWobble ? 0 : 1);
            lowerWobble = !lowerWobble;
        }
        lastLowerWobbleState = gamepad1.circle;

        if (gamepad1.triangle && !lastUpperWobbleState) {
            robot.upperWobbleServo.setPosition(upperWobble ? 0 : 1);
            upperWobble = !upperWobble;
        }
        lastUpperWobbleState = gamepad1.triangle;
    }

    public void updateDrivetrainMotors() {
        // Apply easing function to all stick inputs
        double left_stick_y = Util.cubicEasing(gamepad1.left_stick_y);
        double left_stick_x = Util.cubicEasing(gamepad1.left_stick_x);
        double right_stick_x = Util.cubicEasing(gamepad1.right_stick_x);
        right_stick_x = gamepad1.right_bumper ? 0.2 : (gamepad1.left_bumper ? -0.2 : right_stick_x);
        // double right_stick_y = Util.cubicEasing(gamepad1.right_stick_y);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, -left_stick_y);
        double ang = Math.atan2(-left_stick_y, left_stick_x) - Math.PI / 4;
        double turnCon = right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon) * slowCon;
        double v2 = (radius * Math.sin(ang) - turnCon) * slowCon;
        double v3 = (radius * Math.sin(ang) + turnCon) * slowCon;
        double v4 = (radius * Math.cos(ang) - turnCon) * slowCon;

        telemetry.addData("v1 leftFront", v1);
        telemetry.addData("v2 rightFront", v2);
        telemetry.addData("v3 leftRear", v3);
        telemetry.addData("v4 rightRear", v4);

        // Sets power of motor, spins wheels
        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);
    }
}


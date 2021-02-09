package org.firstinspires.ftc.teamcode.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.hardware.MacaroniHardware;

@TeleOp(name = "Macaroni", group = "Linear OpMode")
public class Macaroni extends LinearOpMode {
    final MacaroniHardware robot = new MacaroniHardware();
    final ElapsedTime runTime = new ElapsedTime();
    double lastControlsUpdate;

    private static final double powerGranularity = 0.05;
    double launcherPower = 1f;

    // Wobble Goal Servo states
    boolean armActive = false; // Current state of the Lower Wobble Goal (Arm)
    boolean clawActive = false; // Current state of the Upper Wobble Goal (Claw)
    boolean prevArmState = false;
    boolean prevClawState = false;

    // Other states
    boolean toggleLauncher = false;
    boolean intakeDirection = true;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        lastControlsUpdate = runTime.seconds();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driverControls();
            operatorControls();
            telemetry.update();
        }
    }

    private void operatorControls() {
        // Lower Wobble (Arm) controls
        if (gamepad2.x && !prevArmState) {
            robot.armServo.setPosition(armActive ? 0 : 1);
            armActive = !armActive;
        }
        prevArmState = gamepad2.x;

        // Upper Wobble (Claw) controls
        if (gamepad2.circle && !prevClawState) {
            robot.clawServo.setPosition(clawActive ? 0 : 1);
            clawActive = !clawActive;
        }
        prevClawState = gamepad2.circle;

        boolean controllerTouched = true;
        if (controllerLoop(0.15)) {
            if (gamepad2.dpad_left)
                launcherPower = 0.65;
            else if (gamepad2.dpad_right)
                launcherPower = 0.75;
            else if (gamepad2.dpad_up || gamepad2.dpad_down) {
                // Change launcher motor power with D-Pad controls
                double powerChange = gamepad2.dpad_up ? powerGranularity : -powerGranularity;
                launcherPower = Util.clamp((float) (launcherPower + powerChange), 0, 1);
            } else
                controllerTouched = false;

            if (!controllerTouched && gamepad2.right_bumper) {
                intakeDirection = !intakeDirection;
                controllerTouched = true;
            }

            if (controllerTouched)
                lastControlsUpdate = runTime.seconds();

            // Toggle launcher
            // if (gamepad2.share) toggleLauncher = !toggleLauncher;
        }
        telemetry.addData("launcherPower", launcherPower);

        // Right trigger to run all intake motors/servos at desired speed
        if (gamepad2.right_trigger > 0) {
            robot.beltMotor.setPower(intakeDirection ? 1 : -1);
            double rt = intakeDirection ? Util.cubicEasing(gamepad2.right_trigger) : -Util.cubicEasing(gamepad2.right_trigger);
            robot.intakeMotor.setPower(rt);
        } else {
            robot.intakeMotor.setPower(0);
            if (gamepad2.left_stick_y != 0)
                robot.beltMotor.setPower(gamepad2.left_stick_y > 0 ? 1 : -1);
        }

        telemetry.addData("beltMotor", robot.beltMotor.getPower());
        telemetry.addData("intakeMotor", robot.intakeMotor.getPower());
        telemetry.addData("beltMotor", gamepad2.left_stick_y);
        telemetry.addData("launcherMotor", robot.launcherMotor.getPower());
        robot.launcherMotor.setPower(gamepad2.left_trigger > 0 ? launcherPower : 0);
    }

    /**
     * Processes the driver gamepad's controls.
     * <p>
     * Left and Right Stick: Move robot
     * Left and Right Bumpers: Rotate robot slowly
     */
    private void driverControls() {
        // Apply easing function to all stick inputs
        double left_stick_y = Util.cubicEasing(gamepad1.left_stick_y);
        double left_stick_x = Util.cubicEasing(gamepad1.left_stick_x);
        double right_stick_x = Util.cubicEasing(gamepad1.right_stick_x);
        double right_stick_y = Util.cubicEasing(gamepad1.right_stick_y);

        // Left and Right bumper controls (slow rotate)
        right_stick_x = gamepad1.right_bumper ? 0.2 : (gamepad1.left_bumper ? -0.2 : right_stick_x);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, -left_stick_y);
        double ang = Math.atan2(-left_stick_y, left_stick_x) - Math.PI / 4;
        double turnCon = right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon);
        double v2 = (radius * Math.sin(ang) - turnCon);
        double v3 = (radius * Math.sin(ang) + turnCon);
        double v4 = (radius * Math.cos(ang) - turnCon);

        telemetry.addData("leftFront", v1);
        telemetry.addData("rightFront", v2);
        telemetry.addData("leftRear", v3);
        telemetry.addData("rightRear", v4);

        // Sets power of motor, spins wheels
        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);
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
        double waited = runTime.seconds() - lastControlsUpdate;
        telemetry.addData("controllerLoop", Util.clamp((float) (waited / duration), 0f, 1f));
        return waited >= duration;
    }
}

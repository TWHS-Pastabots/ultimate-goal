package org.firstinspires.ftc.team16911.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16911.hardware.MacaroniHardware;
import org.firstinspires.ftc.teamcode.Util;

@TeleOp(name = "Macaroni Old", group = "Linear OpMode")
@Disabled
public class MacaroniOld extends LinearOpMode {
    final MacaroniHardware robot = new MacaroniHardware();
    final ElapsedTime runTime =  new ElapsedTime();
    double lastControlsUpdate;

    double wheelPower = 1;
    private static final double powerGranularity = 0.05;
    double launcherPower = 1f;

    // Wobble Goal Servo states
    boolean armActive = false; // Current state of the Lower Wobble Goal (Arm)
    boolean clawActive = false; // Current state of the Upper Wobble Goal (Claw)
    boolean prevArmState = false;
    boolean prevClawState = false;

    // Ring Stopper Servo states
    boolean ringStopperActive = true;
    boolean prevRingStopper = true;

    // Other states
    boolean toggleLauncher = false;
    boolean intakeDirection = true;

    FtcDashboard dashboard;
    Telemetry dash_telemetry;

    @Override
    public void runOpMode() {
        lastControlsUpdate = runTime.seconds();
        dashboard = FtcDashboard.getInstance();
        dash_telemetry = dashboard.getTelemetry();

        waitForStart();

        robot.init(hardwareMap);

        // TODO: Implement Road Runner drivetrain controls
        // TODO: Motor speed shooting block

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driverControls();
            operatorControls();
            dash_telemetry.update();
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
//            robot.leftClawServo.setPosition(clawActive ? 1 : 0);
//            robot.rightClawServo.setPosition(clawActive ? 0 : 1);
            clawActive = !clawActive;
        }
        prevClawState = gamepad2.circle;

        // Ring Stopper controls
        if (gamepad2.triangle && !prevRingStopper) {
//            robot.ringStopperServo.setPosition(ringStopperActive ? 0 : 1);
            ringStopperActive = !ringStopperActive;
        }
        prevRingStopper = gamepad2.triangle;

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

            if (controllerTouched)
                lastControlsUpdate = runTime.seconds();

            // Toggle launcher
            // if (gamepad2.share) toggleLauncher = !toggleLauncher;
        }
        dash_telemetry.addData("launcherPower", launcherPower);
        telemetry.addData("Launcher Power", ((int) (launcherPower * 100)) + "%");
        telemetry.update();

        // Intake if Right Trigger
        if (gamepad1.right_trigger > 0) {
            double rt = intakeDirection ? Util.cubicEasing(gamepad2.right_trigger) : -Util.cubicEasing(gamepad2.right_trigger);
//            robot.intakeMotor.setPower(rt);
        }

        // Right trigger to run all intake motors/servos at desired speed
        if (gamepad1.left_bumper) {
//            robot.beltMotor.setPower(-1);
//            robot.intakeMotor.setPower(-1);
        } else if (gamepad1.right_bumper) {
//            robot.beltMotor.setPower(1);
//            robot.intakeMotor.setPower(1);
        } else {
//            robot.intakeMotor.setPower(0);
//            robot.beltMotor.setPower(gamepad2.left_stick_y > 0.05 ? 1 : (gamepad2.left_stick_y < -0.05 ? -1 : 0));
        }

//        dash_telemetry.addData("beltMotor", robot.beltMotor.getPower());
//        dash_telemetry.addData("intakeMotor", robot.intakeMotor.getPower());
        dash_telemetry.addData("beltMotor", gamepad2.left_stick_y);
//        dash_telemetry.addData("launcherMotor", robot.launcherMotor.getPower());
//        robot.launcherMotor.setPower(gamepad2.left_trigger > 0 ? launcherPower : 0);
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

        // Update wheel power multipier
        if (gamepad1.square)
            wheelPower = 1;
        else if (gamepad1.triangle)
            wheelPower = 0.25;
        else if (gamepad1.circle)
            wheelPower = 0.5;
        else if (gamepad1.cross)
            wheelPower = 0.75;

        if (gamepad1.dpad_up)
            left_stick_y = -1;
        else if (gamepad1.dpad_down)
            left_stick_y = 1;

        if (gamepad1.dpad_right)
            left_stick_x = 1;
        else if (gamepad1.dpad_left)
            left_stick_x = -1;


        // Left and Right bumper controls (slow rotate)
        right_stick_x = gamepad1.right_bumper ? 0.2 : (gamepad1.left_bumper ? -0.2 : right_stick_x);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, -left_stick_y);
        double ang = Math.atan2(-left_stick_y, left_stick_x) - Math.PI / 4;
        double turnCon = right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon) * wheelPower;
        double v2 = (radius * Math.sin(ang) - turnCon) * wheelPower;
        double v3 = (radius * Math.sin(ang) + turnCon) * wheelPower;
        double v4 = (radius * Math.cos(ang) - turnCon) * wheelPower;

        dash_telemetry.addData("leftFront", v1);
        dash_telemetry.addData("rightFront", v2);
        dash_telemetry.addData("leftRear", v3);
        dash_telemetry.addData("rightRear", v4);

        dash_telemetry.addData("leftEncoder", robot.encoderLeft.getCorrectedVelocity());
        dash_telemetry.addData("rightEncoder", robot.encoderRight.getCorrectedVelocity());
        dash_telemetry.addData("forwardEncoder", robot.encoderFront.getCorrectedVelocity());

        // Sets power of motor, spins wheels
        robot.motorLeftFront.setPower(v1);
        robot.motorRightFront.setPower(v2);
        robot.motorLeftRear.setPower(v3);
        robot.motorRightRear.setPower(v4);
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
        dash_telemetry.addData("controllerLoop", Util.clamp((float) (waited / duration), 0f, 1f));
        return waited >= duration;
    }
}

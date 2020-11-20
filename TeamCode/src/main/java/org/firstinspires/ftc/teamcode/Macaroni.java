package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name = "Macaroni", group = "Linear OpMode")
public class Macaroni extends OpMode {

    /* Declare OpMode members. */
    final MacaroniHardware robot = new MacaroniHardware();
    final ElapsedTime runTime = new ElapsedTime();
    double lastControlsUpdate;
    double slowCon = 0.8;
    boolean toggleLauncher = false;
    float launcherPower = 1f;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
        lastControlsUpdate = getRuntime();
    }

    // loop on init()
    @Override
    public void init_loop() {
    }

    // Run once on start()
    @Override
    public void start() {
        runTime.reset();
        telemetry.addData("Run Time", "reset");
    }

    /**
     * A simple helper function that assists with spacing out controller input polls.
     *
     * Without a proper guard in place, controller input code for button presses will be ran hundreds of
     * times per second, resulting in strange behavior. By adding a guard that only lets the button input
     * code run every other fraction of a second, button input is much more reliable for humans.
     *
     * @param duration The minimum duration that must be elapsed before the controller loop is ran.
     * @return True if the controller should be polled now, false if the duration has not yet elapsed.
     */
    public boolean controllerLoop(double duration) {
        double now = getRuntime();
        if (now - lastControlsUpdate >= duration) {
            this.lastControlsUpdate = now;
            return true;
        }
        return false;
    }

    // Loop on start()
    @Override
    public void loop() {
        // Motor multiplier
        if (gamepad1.b) slowCon = .4;
        if (gamepad1.a) slowCon = .8;
        if (gamepad1.x) slowCon = 1.0;

        // Apply easing function to all stick inputs
        double left_stick_y = Util.cubicEasing(gamepad1.left_stick_y);
        double left_stick_x = Util.cubicEasing(gamepad1.left_stick_x);
        double right_stick_x = Util.cubicEasing(gamepad1.right_stick_x);
        double right_stick_y = Util.cubicEasing(gamepad1.right_stick_y);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, left_stick_y);
        double ang = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;

        double turnCon = right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon) * slowCon;
        double v2 = (radius * Math.sin(ang) - turnCon) * slowCon;
        double v3 = (radius * Math.sin(ang) + turnCon) * slowCon;
        double v4 = (radius * Math.cos(ang) - turnCon) * slowCon;

        // Sets power of motor, spins wheels
        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);

        // Poll the controller's inputs 0.33 seconds after the previous poll
        if (controllerLoop(0.25)) {
            // Change launcher motor power with D-Pad controls
            float powerChange = gamepad1.dpad_up ? 0.1f : (gamepad1.dpad_down ? -0.1f : 0f);
            launcherPower = Util.clamp(launcherPower + powerChange, 0, 1);

            // toggle launcher motor with back button
            if (gamepad1.back) toggleLauncher = !toggleLauncher;
            if (!gamepad1.x) robot.launcherMotor.setPower(toggleLauncher ? launcherPower : 0);
        }

        // Gamepad X to spin up launcher motor
        if (gamepad1.x)
            robot.launcherMotor.setPower(1);
        else
            robot.launcherMotor.setPower(0);


        // Right trigger to run all intake motors/servos at desired speed
        if (gamepad1.right_trigger > 0) {
            robot.intakeServo.setPower(1); // Always run at max power
            double rt = gamepad1.right_trigger;
            robot.beltMotor.setPower(Util.cubicEasing(rt));
            robot.intakeMotor.setPower(Util.cubicEasing(rt));
        } else {
            robot.intakeServo.setPower(0);
            robot.intakeMotor.setPower(0);
            robot.beltMotor.setPower(0);
        }

        // Show motor output visually
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        telemetry.addLine(Telemetry.createLevel((float) v1));
        telemetry.addLine(Telemetry.createLevel((float) v2));
        telemetry.addLine(Telemetry.createLevel((float) v3));
        telemetry.addLine(Telemetry.createLevel((float) v4));
        telemetry.addLine(String.format(Locale.ENGLISH, "Launcher %s - %d%%", toggleLauncher ? "ON" : "OFF", (int) (launcherPower * 100)));

        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}


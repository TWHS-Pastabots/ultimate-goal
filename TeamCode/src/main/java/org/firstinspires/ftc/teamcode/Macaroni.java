package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Macaroni", group = "Linear OpMode")
public class Macaroni extends OpMode {

    /* Declare OpMode members. */
    final MacaroniHardware robot = new MacaroniHardware();
    final ElapsedTime runTime = new ElapsedTime();
    double slowCon = 0.8;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
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

        if (gamepad1.right_trigger > 0) {
            robot.intakeServo.setPower(1);
            double rt = gamepad1.right_trigger;
            robot.beltMotor.setPower(Util.cubicEasing(rt));
            robot.intakeMotor.setPower(Util.cubicEasing(rt));
            if (gamepad1.right_bumper)
                robot.launcherMotor.setPower(gamepad1.right_trigger);
        } else {
            robot.intakeServo.setPower(0);
            robot.intakeMotor.setPower(0);
            robot.beltMotor.setPower(0);
            robot.launcherMotor.setPower(0);
        }

        // Show motor output visually
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        telemetry.addLine(Telemetry.createLevel((float) v1));
        telemetry.addLine(Telemetry.createLevel((float) v2));
        telemetry.addLine(Telemetry.createLevel((float) v3));
        telemetry.addLine(Telemetry.createLevel((float) v4));

        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}


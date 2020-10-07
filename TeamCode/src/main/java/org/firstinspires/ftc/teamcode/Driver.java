package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver", group = "Linear OpMode")
public class Driver extends OpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime = new ElapsedTime();
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
        double radius = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double ang = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

        double turnCon = gamepad1.right_stick_x * .75;
        double v1 = radius * Math.cos(ang) + turnCon;
        double v2 = radius * Math.sin(ang) - turnCon;
        double v3 = radius * Math.sin(ang) + turnCon;
        double v4 = radius * Math.cos(ang) - turnCon;

        // Sets power of motor, spins wheels
        if (gamepad1.b) slowCon = .4;
        if (gamepad1.a) slowCon = .8;
        if (gamepad1.x) slowCon = 1.0;

        robot.motorLeftFront.setPower(v1 * slowCon);
        robot.motorRightFront.setPower(v2 * slowCon);
        robot.motorLeftRear.setPower(v3 * slowCon);
        robot.motorRightRear.setPower(v4 * slowCon);

        telemetry.addData("Powers:", v1);
        telemetry.addData("", v2);
        telemetry.addData("", v3);
        telemetry.addData("", v4);
        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}


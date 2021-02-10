package org.firstinspires.ftc.teamcode.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Telemetry;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.hardware.SpaghettiHardware;

@TeleOp(name = "Spaghetti", group = "Linear OpMode")
@Config
public class Spaghetti extends OpMode {

    /* Declare OpMode members. */
    private final SpaghettiHardware robot = new SpaghettiHardware();
    private FtcDashboard dashboard;
    private final ElapsedTime runTime = new ElapsedTime();
    private double slowCon = 0.8;

    private double launcherEndTime = 0.0d;
    private static double LAUNCHER_SERVO_TIME = 0.25d;

    private static double SERVO_ON = 1d;
    private static double SERVO_OFF = 0d;

    private static double INTAKE_ON = 1d;
    private static double INTAKE_OFF = 0d;

    //run once on init()
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

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
        TelemetryPacket packet = new TelemetryPacket();

        // Apply easing function to all stick inputs
        double left_stick_y = Util.cubicEasing(gamepad1.left_stick_y);
        double left_stick_x = Util.cubicEasing(gamepad1.left_stick_x);
        double right_stick_x = Util.cubicEasing(gamepad1.right_stick_x);
        double right_stick_y = Util.cubicEasing(gamepad1.right_stick_y);

        packet.put("left stick x", left_stick_x);
        packet.put("left stick y", left_stick_y);
        packet.put("right stick x", right_stick_x);
        packet.put("right stick y", right_stick_y);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, left_stick_y);
        double ang = Math.atan2(left_stick_y, -left_stick_x) - Math.PI / 4;

        double turnCon = right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon) * slowCon;
        double v2 = (radius * Math.sin(ang) - turnCon) * slowCon;
        double v3 = (radius * Math.sin(ang) + turnCon) * slowCon;
        double v4 = (radius * Math.cos(ang) - turnCon) * slowCon;

        // Sets power of motor, spins wheels
        robot.motorLeftFront.setPower(v1);
        robot.motorRightFront.setPower(v2);
        robot.motorLeftRear.setPower(v3);
        robot.motorRightRear.setPower(v4);

        robot.launcherMotor.setPower(Util.cubicEasing(gamepad1.right_trigger));

        if (gamepad1.right_bumper) {
            launcherEndTime = getRuntime() + LAUNCHER_SERVO_TIME;
        }

        if (getRuntime() < launcherEndTime) {
            robot.launcherServo.setPosition(SERVO_ON);
        } else {
            robot.launcherServo.setPosition(SERVO_OFF);
        }

//        if (gamepad1.right_bumper) {
//            robot.launcherServo.setPosition(SERVO_ON);
//        } else {
//            robot.launcherServo.setPosition(SERVO_OFF);
//        }

        packet.put("launcher servo position", robot.launcherServo.getPosition());

        if (gamepad1.left_bumper) {
            robot.intakeMotor.setPower(INTAKE_ON);
        } else {
            robot.intakeMotor.setPower(INTAKE_OFF);
        }

        packet.put("intake motor power", robot.intakeMotor.getPower());

        if (gamepad1.a) {
            robot.armServo.setPosition(SERVO_ON);
        } else {
            robot.armServo.setPosition(SERVO_OFF);
        }

        packet.put("arm servo position", robot.armServo.getPosition());

        if (gamepad1.b) {
            robot.clawServo.setPosition(SERVO_ON);
        } else {
            robot.clawServo.setPosition(SERVO_OFF);
        }

        packet.put("claw servo position", robot.clawServo.getPosition());

        // Show motor output visually
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        telemetry.addLine(Telemetry.createLevel((float) v1));
        telemetry.addLine(Telemetry.createLevel((float) v2));
        telemetry.addLine(Telemetry.createLevel((float) v3));
        telemetry.addLine(Telemetry.createLevel((float) v4));

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}

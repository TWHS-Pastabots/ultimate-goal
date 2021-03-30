package org.firstinspires.ftc.team15021.telop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.teamcode.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

@TeleOp(name = "Ravioli", group = "Linear OpMode")
public class Ravioli extends OpMode {

    /* Declare OpMode members. */
    final RavioliHardware robot = new RavioliHardware();
    final ElapsedTime runTime = new ElapsedTime();
    double slowCon = 1.0;

    double vLauncher = 0.0;
    double vLauncherMult = 1.0;
    double vConveyor = 0.0;
    double vServoMotor = 0.0;

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
        if (gamepad1.b) slowCon = 0.25;
        if (gamepad1.a) slowCon = 0.5;
        if (gamepad1.x) slowCon = 0.75;
        if (gamepad1.y) slowCon = 1.0;

        // Apply easing function to all stick inputs
        double left_stick_y = Util.cubicEasing(gamepad1.left_stick_y);
        double left_stick_x = Util.cubicEasing(gamepad1.left_stick_x);
        double right_stick_x = Util.squareEasing(gamepad1.right_stick_x);

        // Mechanum trig math
        double radius = Math.hypot(left_stick_x, left_stick_y);
        double ang = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;

        double turnCon = -right_stick_x * .75;

        // Final motor powers, with multiplier applied
        double v1 = (radius * Math.cos(ang) + turnCon) * slowCon;
        double v2 = (radius * Math.sin(ang) - turnCon) * slowCon;
        double v3 = (radius * Math.sin(ang) + turnCon) * slowCon;
        double v4 = (radius * Math.cos(ang) - turnCon) * slowCon;

        // Launcher and Conveyor Powers (R Trigger = Launcher, Up Button = Conveyor - Can be changed)
        if (gamepad2.b) vLauncherMult = 0.25;
        if (gamepad2.a) vLauncherMult = 0.5;
        if (gamepad2.x) vLauncherMult = 0.75;
        if (gamepad2.y) vLauncherMult = 1.0;
        vLauncher = gamepad2.right_trigger * vLauncherMult;

        if (gamepad2.dpad_up) vConveyor = 1.0;
        else if (gamepad2.dpad_down) vConveyor = -1.0;
        else if (gamepad1.left_bumper) vConveyor = -1.0;
        else if (gamepad1.right_bumper) vConveyor = 1.0;
        else vConveyor = 0.0;

        robot.servoClaw.setPosition(gamepad2.left_trigger);

        //servoMotor = left: up, right: down
        if (gamepad2.dpad_left) vServoMotor = 0.35;
        else if (gamepad2.dpad_right) vServoMotor = -0.35;
        else vServoMotor = 0.0;

        if (gamepad2.touchpad) robot.servoIntake.setPosition(0.6);
        else robot.servoIntake.setPosition(0);


        // Sets power of motor, spins wheels
        robot.motorLeftFront.setPower(-v1);
        robot.motorRightFront.setPower(-v2);
        robot.motorLeftRear.setPower(-v3);
        robot.motorRightRear.setPower(-v4);

        // Sets powers of Launcher and Conveyor Motors
        robot.motorLauncher.setPower(vLauncher);
        robot.motorConveyor.setPower(vConveyor);
        robot.motorIntake.setPower(vConveyor);

        robot.motorClaw.setPower(vServoMotor);

        // Show motor output visually
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        telemetry.addLine(Telemetry.createLevel((float) v1));
        telemetry.addLine(Telemetry.createLevel((float) v2));
        telemetry.addLine(Telemetry.createLevel((float) v3));
        telemetry.addLine(Telemetry.createLevel((float) v4));

        telemetry.addLine("Launcher Power: " + (int) (vLauncher * 100) + "%" + "\nLauncher Max Power: " + (int) (vLauncherMult * 100) + "%");
//        telemetry.addLine("Servo Position: " + robot.servoPos);

        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}

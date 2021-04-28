package org.firstinspires.ftc.team15021.telop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.teamcode.Util;

import java.text.DecimalFormat;


@TeleOp(name = "Ravioli", group = "Linear OpMode")
public class Ravioli extends OpMode {
    /* Declare OpMode members. */
    final RavioliHardware robot = new RavioliHardware();
    private SampleMecanumDrive drive;
    final ElapsedTime runTime = new ElapsedTime();

    private String front;
    private String back;
    private String right;
    private String left;

    double slowCon = 1.0;

    double vLauncher = 0.0;
    double vLauncherMult = 0.70;
    double vConveyor = 0.0;
    double vServoMotor = 0.0;

    DecimalFormat df = new DecimalFormat("#.##");

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

//        double leftPower = Util.cubicEasing(gamepad1.right_stick_x);
//        double rightPower = Util.cubicEasing(-gamepad1.right_stick_x);
//        double vertical = Util.cubicEasing(gamepad1.left_stick_y);


        // Launcher and Conveyor Powers (R Trigger = Launcher, Up Button = Conveyor - Can be changed)
        //if (gamepad2.b) vLauncherMult = 0.25;
        if (gamepad2.square) vLauncherMult = 0.60;
        if (gamepad2.circle) vLauncherMult = 0.65;
        if (gamepad2.triangle) vLauncherMult = 0.70;

        if (gamepad2.left_bumper)
            vLauncher = -gamepad2.right_trigger * vLauncherMult;
        else
            vLauncher = gamepad2.right_trigger * vLauncherMult;

        if (gamepad2.dpad_up) vConveyor = 1.0;
        else if (gamepad2.dpad_down) vConveyor = -1.0;
        else if (gamepad1.left_bumper) vConveyor = -1.0;
        else if (gamepad1.right_bumper) vConveyor = 1.0;
        else vConveyor = 0.0;

        robot.servoClaw.setPosition(1 - gamepad2.left_trigger);

        //servoMotor = left: up, right: down
        //if (gamepad2.dpad_left) vServoMotor = 0.35;
        //else if (gamepad2.dpad_right) vServoMotor = -0.35;
        //else vServoMotor = 0.0;
        vServoMotor = gamepad2.right_stick_y * .5;

        if (gamepad2.cross) robot.servoIntake.setPosition(0.6);
        else robot.servoIntake.setPosition(0);


        // Sets power of motor, spins wheels
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );


        // Sets powers of Launcher and Conveyor Motors
        robot.motorLauncher.setPower(vLauncher);
        robot.motorConveyor.setPower(vConveyor);
        robot.motorIntake.setPower(vConveyor);

        robot.motorClaw.setPower(vServoMotor);

        // Show motor output visually
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        // telemetry.addLine(Telemetry.createLevel((float) v1));
        // telemetry.addLine(Telemetry.createLevel((float) v2));
        // telemetry.addLine(Telemetry.createLevel((float) v3));
        // telemetry.addLine(Telemetry.createLevel((float) v4));


        drive.update();
        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }
}
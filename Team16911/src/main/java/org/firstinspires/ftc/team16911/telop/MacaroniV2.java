package org.firstinspires.ftc.team16911.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16911.auton.MacaroniBasic;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.MacaroniHardware;
import org.firstinspires.ftc.teamcode.gamepad.Lockup;
import org.firstinspires.ftc.teamcode.gamepad.SingleDown;
import org.firstinspires.ftc.teamcode.gamepad.TimedGamepad;
import org.firstinspires.ftc.teamcode.gamepad.Toggle;

@TeleOp(name = "Macaroni v2", group = "Linear OpMode")
public class MacaroniV2 extends OpMode {
    // Drivetrain, Robot Hardware and Telemetry controllers
    private SampleMecanumDrive drive;
    private final MacaroniHardware robot = new MacaroniHardware();
    private Telemetry dash_telemetry;

    // Gamepad State Control
    private TimedGamepad timedGamepad1;
    private TimedGamepad timedGamepad2;
    private Lockup gamepad1_lockup;
    private Lockup gamepad2_lockup;
    private Toggle armToggle;
    private Toggle clawToggle;
    private Toggle stopperToggle;
    private SingleDown dpad_up;
    private SingleDown dpad_down;

    // Tele-operated Configuration Constants
    public static double NUDGE_AMOUNT = 0.5;
    public static double X_SCALE = 0.7;
    public static double Y_SCALE = 0.7;
    public static double TURN_SCALE = 0.85;

    public double LAUNCHER_POWER = 0.85;


    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dash_telemetry = dashboard.getTelemetry();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(MacaroniBasic.START_POSITION);
        robot.init(hardwareMap);

        timedGamepad1 = new TimedGamepad(this.gamepad1);
        timedGamepad2 = new TimedGamepad(this.gamepad2);

        armToggle = new Toggle();
        clawToggle = new Toggle();
        stopperToggle = new Toggle();

        dpad_up = new SingleDown();
        dpad_down = new SingleDown();

        gamepad1_lockup = new Lockup();
        gamepad2_lockup = new Lockup();

        timedGamepad1.options.installStrategy(gamepad1_lockup);
        timedGamepad2.options.installStrategy(gamepad2_lockup);

        timedGamepad2.square.installStrategy(armToggle);
        timedGamepad2.circle.installStrategy(clawToggle);
        timedGamepad2.triangle.installStrategy(stopperToggle);

        timedGamepad2.dpad_up.installStrategy(dpad_up);
        timedGamepad2.dpad_down.installStrategy(dpad_down);
    }

    @Override
    public void loop() {
        robot.tick();
        timed_gamepads();
        driver();
        operator();
        stats();

        telemetry.update();
        dash_telemetry.update();
    }

    /**
     * Update the TimedGamepad instances.
     */
    private void timed_gamepads() {
        timedGamepad1.tick();
        timedGamepad2.tick();
    }

    /**
     * Processes operator (Gamepad 2) controls.
     */
    private void operator() {
        // Don't process this when the gamepad is locked up by pressing Gamepad2's Options button.
        if (!gamepad2_lockup.read()) {
            // Servo toggles
            if (armToggle.changed())
                robot.armServo.setPosition(armToggle.read() ? 0 : 1);
            if (clawToggle.changed())
                robot.setClaws(clawToggle.read() ? 0 : 1);
            if (stopperToggle.changed())
                robot.setRingStopper(stopperToggle.read());
        }

        // Spin the launcher with left trigger to the specified launcher power
        robot.setLauncherPower(gamepad2.left_trigger > 0.5 ? LAUNCHER_POWER : 0.0);

        // Intake with right trigger, outtake with right bumper
        if (gamepad2.right_trigger > 0.5)
            robot.setIntakePower(1, 1);
        else if (gamepad2.right_bumper)
            robot.setIntakePower(-1, -1);
        else
            robot.setIntakePower(0, 0);

        // Power change with DPAD up/down
        if (dpad_up.poll())
            LAUNCHER_POWER += 0.05;
        else if (dpad_down.poll())
            LAUNCHER_POWER -= 0.05;
        LAUNCHER_POWER = Math.max(0, Math.min(1, LAUNCHER_POWER));

        robot.setBeltPower(gamepad2.left_stick_y);

        telemetry.addData("Prespin Left", robot.prespinTelemetry());
        telemetry.addData("Ring Stopper", (robot.ringStopperActive() ? "Active" : "Inactive"));
        telemetry.addData("Launcher Power", ((int) (LAUNCHER_POWER * 100)) + "%");
    }

    private void driver() {
        // Nudge controls, dpad + left/righter trigger
        double x_nudge = gamepad1.dpad_right ? NUDGE_AMOUNT : (gamepad1.dpad_left ? -NUDGE_AMOUNT : 0);
        double y_nudge = gamepad1.dpad_up ? -NUDGE_AMOUNT : (gamepad1.dpad_down ? NUDGE_AMOUNT : 0);
        double ang_nudge = gamepad1.left_trigger > 0
                ? gamepad1.left_trigger * -NUDGE_AMOUNT
                : gamepad1.right_trigger * NUDGE_AMOUNT;

        // Road runner based controls with left/right stick XY
        if (!drive.isBusy()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y * Y_SCALE + y_nudge),
                            -(gamepad1.left_stick_x * X_SCALE + x_nudge),
                            -(gamepad1.right_stick_x * TURN_SCALE + ang_nudge)
                    )
            );
        }

        drive.update();

        if (!gamepad1_lockup.read()) {
            if (gamepad1.left_bumper) {
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(MacaroniBasic.START_POSITION)
                        .build());
            } else if (gamepad1.right_bumper) {
                drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(0, 0, 0))
                        .build()
                );
            }

            // Drive speed change
            if (gamepad1.triangle)
                X_SCALE = Y_SCALE = 0.25;
            else if (gamepad1.circle)
                X_SCALE = Y_SCALE = 0.5;
            else if (gamepad1.cross)
                X_SCALE = Y_SCALE = 0.75;
            else if (gamepad1.square)
                X_SCALE = Y_SCALE = 1.0;
        }

        telemetry.addData("X Scale", ((int) (X_SCALE * 100)) + "%");
        telemetry.addData("Y Scale", ((int) (Y_SCALE * 100)) + "%");
    }

    private void stats() {
        telemetry.addData("launcherSpeed", robot.launcherSpeed());
    }
}
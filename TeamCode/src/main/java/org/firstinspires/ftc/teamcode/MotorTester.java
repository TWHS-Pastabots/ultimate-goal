package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode {

    final RobotHardware robot = new RobotHardware();
    final ElapsedTime runTime = new ElapsedTime();

    int[] encoderPositions;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        this.encoderPositions = new int[]{0, 0, 0, 0}

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");    //
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
        // Calculate inputs using easeInSine easing function
        double[] inputs = {
                Util.cubicEasing(Util.maxMagnitude(gamepad1.left_stick_x, gamepad1.left_stick_y)),
                Util.cubicEasing(Util.maxMagnitude(gamepad1.right_stick_x, gamepad1.right_stick_y)),
                Util.cubicEasing(Util.maxMagnitude(gamepad2.left_stick_x, gamepad2.left_stick_y)),
                Util.cubicEasing(Util.maxMagnitude(gamepad2.right_stick_x, gamepad2.right_stick_y))
        };

        // Set power to inputs
        for (int i = 0; i < robot.motors.length; i++)
            robot.motors[i].setPower(inputs[i]);

        // Print input telemetry, power level bars
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        for (double input : inputs)
            telemetry.addLine(Telemetry.createLevel((float) input));

        // Encoder information
        for (int i = 0; i < robot.motors.length; i++) {
            DcMotor motor = robot.motors[i];
            if (motor != null) {
                int encoderPosition = motor.getCurrentPosition();
                telemetry.addLine(String.format(Locale.ENGLISH, "Motor %d Change: %d", i + 1, encoderPosition - this.encoderPositions[i]));
                this.encoderPositions[i] = encoderPosition;
            }
        }

        telemetry.update();
    }
}


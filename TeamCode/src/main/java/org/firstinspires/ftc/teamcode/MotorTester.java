package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode {

    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime = new ElapsedTime();
    double slowCon = 0.8;

    int motorOneEncoderPosition = 0;
    int motorTwoEncoderPosition = 0;
    int motorThreeEncoderPosition = 0;
    int motorFourEncoderPosition = 0;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);

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
        robot.motorLeftFront.setPower(inputs[0]);
        robot.motorRightFront.setPower(inputs[1]);
        robot.motorLeftRear.setPower(inputs[2]);
        robot.motorRightRear.setPower(inputs[3]);

        // Print input telemetry, power level bars
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        for (double input : inputs)
            telemetry.addLine(Telemetry.createLevel((float) input));

        // Encoder information
        if (robot.motorLeftFront != null) {
            int encoderPosition = robot.motorLeftFront.getCurrentPosition();
            telemetry.addLine(String.format(Locale.ENGLISH, "Motor One Change: %d", encoderPosition - this.motorOneEncoderPosition));
            this.motorOneEncoderPosition = encoderPosition;
        }

        telemetry.update();
    }
}


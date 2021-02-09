package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Locale;

@TeleOp(name = "Motor Tester")
public class MotorTester extends OpMode {

    final RobotHardware robot = new RobotHardware();
    final ElapsedTime runTime = new ElapsedTime();

    int[] encoderPositions;

    private boolean staticMotors = false;
    private float[] staticMotorOutput;
    private int motorIndex;
    private float motorGranularity;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        this.encoderPositions = new int[]{0, 0, 0, 0};

        this.motorIndex = 0;
        this.staticMotors = false;
        this.staticMotorOutput = new float[robot.motors.size()];
        this.motorGranularity = 0.2f;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
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
                Util.cubicEasing(Util.maxMagnitude(gamepad2.right_stick_x, gamepad2.right_stick_y)),
                0,
                0,
                0,
                0
        };

        // Handle static motor interaction
        if (gamepad1.back || gamepad2.back)
            this.staticMotors = !this.staticMotors;

        if (this.staticMotors) {
            // Increment or decrement the index
            if (gamepad1.dpad_left || gamepad2.dpad_left)
                motorIndex--;
            else if (gamepad1.dpad_right || gamepad2.dpad_right)
                motorIndex++;
            motorIndex = Util.wrap(motorIndex, this.staticMotorOutput.length);

            // Increment or decrement static motor output
            float motorChange = 0;
            if (gamepad1.dpad_up || gamepad2.dpad_up)
                motorChange = motorGranularity;
            else if (gamepad1.dpad_down || gamepad2.dpad_down)
                motorChange = -motorGranularity;
            this.staticMotorOutput[motorIndex] = Util.clamp(this.staticMotorOutput[motorIndex] + motorChange, -1, 1f);

            // Override last 4 inputs if possible
            for (int i = 0; i < inputs.length; i++)
                if (i < 4)
                    if (inputs[i] == 0)
                        inputs[i] = this.staticMotorOutput[i];
                    else if (this.staticMotorOutput[i] > 0)
                        // Check static motor output instead, no Gamepad input available
                        inputs[i] = this.staticMotorOutput[i];
        }

        // Set power to inputs
        for (int i = 0; i < robot.motors.size(); i++)
            robot.motors.get(i).setPower(inputs[i]);

        // Print input telemetry, power level bars
        telemetry.addData("Started", Util.getHumanDuration((float) runTime.seconds()) + " ago");
        for (int i = 0; i < robot.motors.size(); i++)
            telemetry.addLine(Telemetry.createLevel((float) inputs[i]));

        // Encoder information
        for (int i = 0; i < robot.motors.size(); i++) {
            DcMotor motor = robot.motors.get(i);
            if (motor != null) {
                int encoderPosition = motor.getCurrentPosition();
                telemetry.addLine(String.format(Locale.ENGLISH, "Motor %d: %d TPS", i + 1, encoderPosition - this.encoderPositions[i]));
                this.encoderPositions[i] = encoderPosition;
            }
        }

        telemetry.update();
    }
}


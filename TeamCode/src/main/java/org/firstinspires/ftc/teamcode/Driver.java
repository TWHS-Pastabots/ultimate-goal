package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Driver", group = "Linear OpMode")
public class Driver extends OpMode {

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime backTimer2 = new ElapsedTime();
    ElapsedTime intakeDelayTimer = new ElapsedTime();
    boolean isBlockPush = false;
    double slowCon = 0.8;
    int pos = 0;
    int closedSoundEffect;
    int openSoundEffect;

    //private TouchSensor touch;
    //DigitalChannel magnet;

    //run once on init()
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //touch = hardwareMap.touchSensor.get("touch");
        //magnet = hardwareMap.get(DigitalChannel.class, "magnet");
        //magnet.setMode(DigitalChannel.Mode.INPUT);

        robot.clawT.setPosition(1);

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
//        closedSoundEffect = hardwareMap.appContext.getResources().getIdentifier("closediggy",
//                "raw", hardwareMap.appContext.getPackageName());
//        openSoundEffect = hardwareMap.appContext.getResources().getIdentifier("openiggy",
//                "raw", hardwareMap.appContext.getPackageName());
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, openSoundEffect);
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

        // Intake code
        if (gamepad1.right_bumper) {
            if (isBlockPush) {
                isBlockPush = false;
                intakeDelayTimer.reset();
            }
            if (intakeDelayTimer.seconds() > 1.0) {
                robot.leftIn.setPower(-.7);
                robot.rightIn.setPower(-.7);
            }
        } else if (gamepad1.left_bumper) {
            robot.leftIn.setPower(.7);
            robot.rightIn.setPower(.7);
        } else {
            robot.leftIn.setPower(0);
            robot.rightIn.setPower(0);
        }

        // lift code
        if (gamepad2.dpad_up) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftMotor.setPower(-1);
            pos = robot.liftMotor.getCurrentPosition();
        } else if (gamepad2.dpad_down) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftMotor.setPower(0.4);
            pos = robot.liftMotor.getCurrentPosition();
        } else {
            robot.liftMotor.setPower(0.0);
            robot.liftMotor.setTargetPosition(pos);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor.setPower(0.5);
            telemetry.addData("Staying at:", robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }

        // Servos
        if (gamepad1.right_trigger > 0.9) {
            robot.leftH.setPosition(0.75);
            robot.rightH.setPosition(0.9);
        } else if (gamepad1.left_trigger > 0.9) {
            robot.leftH.setPosition(0);
            robot.rightH.setPosition(0);
        }

        if (gamepad2.back && backTimer2.seconds() > 1.0) {
            isBlockPush = !isBlockPush;
            if (isBlockPush)
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, closedSoundEffect);
            else
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, openSoundEffect);
            backTimer2.reset();
        }

        // Claw
        if (gamepad2.a) {
            robot.claw.setPosition(1);
        } else if (gamepad2.b) {
            robot.claw.setPosition(0.5);
        }

        if (gamepad2.dpad_right) {
            robot.clawT.setPosition(0);
        } else if (gamepad2.dpad_left) {
            robot.clawT.setPosition(1);
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.capstone.setDirection(CRServo.Direction.REVERSE);
            robot.capstone.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1) {
            robot.capstone.setDirection(CRServo.Direction.FORWARD);
            robot.capstone.setPower(1.0);
        } else {
            robot.capstone.setPower(0.0);
        }


        if (gamepad1.dpad_up) {
            robot.tape.setPower(-1.0);
        } else if (gamepad1.dpad_down) {
            robot.tape.setPower(1.0);
        } else
            robot.tape.setPower(0.0);

        robot.leftFront.setPower(v1 * slowCon);
        robot.rightFront.setPower(v2 * slowCon);
        robot.leftRear.setPower(v3 * slowCon);
        robot.rightRear.setPower(v4 * slowCon);

        if (isBlockPush)
            robot.servo_blockPush.setPosition(0.0);
        else
            robot.servo_blockPush.setPosition(1.0);

        telemetry.addData("Lift Motor Encoder Position:", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Color:", robot.colorSensor.blue());
        telemetry.addData("Powers:", v1);
        telemetry.addData("", v2);
        telemetry.addData("", v3);
        telemetry.addData("", v4);
        //telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
        telemetry.update();
    }

    // run once on stop()
    @Override
    public void stop() {
    }

}


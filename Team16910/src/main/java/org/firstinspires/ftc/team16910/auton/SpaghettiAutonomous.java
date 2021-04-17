package org.firstinspires.ftc.team16910.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.telop.Spaghetti;

import java.util.List;

@Config
@Autonomous(name = "Spaghetti Autonomous", preselectTeleOp = "Spaghetti")
public class SpaghettiAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.6;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static double ON = 1;
    public static double OFF = 0;

    public static double ARM_TIME = 600;
    public static double CLAW_TIME = 400;

    public static double LAUNCHER_POWER = 0.56;
    public static double LAUNCHER_SPINUP = 1000;
    public static double LAUNCHER_LAUNCH = 600;

    public static Pose2d WOBBLE_GOAL_1 = new Pose2d(-2.5300820012016763, 52.1048583194061, 0);
    public static Pose2d WOBBLE_GOAL_2 = new Pose2d(20.479536027344817, 28.077649944945843, 0);
    public static double WOBBLE_GOAL_2_TANGENT = Math.toRadians(-90);
    public static Pose2d WOBBLE_GOAL_3 = new Pose2d(44.020241167, 51.2568443, 0);
    private Pose2d wobbleGoalPosition;
    private double wobbleGoalTangent = 0;

    public static Pose2d FINAL_POSITION = new Pose2d(12, 18, 0);

    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;

    private Trajectory toWobbleGoal, toPowerShot1, toPowerShot1_2, toPowerShot2, toPowerShot3, toFinish;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        // Initialize the robot hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-62.358, 48.24, 0));

        activateTfod();

        // Initialize the arm
        robot.clawServo.setPosition(ON);

        // Wait until the opmode should start
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        detectRings();

        // Prepare the trajectories needed throughout the opmode
        prepareTrajectories();

        telemetry.update();

        // Move to the wobble goal and drop it
        drive.followTrajectory(toWobbleGoal);
        dropWobbleGoal();

        telemetry.update();

        // Move to the powershot goal and shoot the rings
        drive.followTrajectory(toPowerShot1_2);
        launchPowerShots();

        // Move to the finish position and end the opmode
        finish();
    }

    /**
     *
     */
    private void prepareTrajectories() {
        Pose2d wobbleGoal = wobbleGoalPosition;
        if (wobbleGoal == null) {
            wobbleGoal = WOBBLE_GOAL_1;
        }

        toWobbleGoal = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(wobbleGoal, wobbleGoalTangent)
                .build();

        toPowerShot1 = drive.trajectoryBuilder(toWobbleGoal.end())
                .forward(-12)
                .build();

        toPowerShot1_2 = drive.trajectoryBuilder(toPowerShot1.end())
                .splineToLinearHeading(new Pose2d(1.7205879405195255, 30.3902133119823317, 0), Math.toRadians(0))
                .build();

        toPowerShot2 = drive.trajectoryBuilder(toPowerShot1_2.end())
                .strafeRight(8)
                .build();

        toPowerShot3 = drive.trajectoryBuilder(toPowerShot2.end())
                .strafeRight(8)
                .build();

        toFinish = drive.trajectoryBuilder(toPowerShot3.end())
                .splineToLinearHeading(FINAL_POSITION, 0)
                .build();
    }

    /**
     *
     */
    private void dropWobbleGoal() {
        robot.armServo.setPosition(OFF);
        sleep((long) ARM_TIME);

        robot.clawServo.setPosition(OFF);
        sleep((long) CLAW_TIME);

        drive.followTrajectory(toPowerShot1);

        robot.armServo.setPosition(ON);
        sleep((long) ARM_TIME);
    }

    /**
     *
     */
    private void launchPowerShots() {
        robot.launcherMotor.setVelocity(Spaghetti.motorPower(LAUNCHER_POWER));
        sleep((long) LAUNCHER_SPINUP);

        launch();
        drive.followTrajectory(toPowerShot2);
        launch();
        drive.followTrajectory(toPowerShot3);
        launch();

        robot.launcherMotor.setPower(OFF);
        sleep((long) LAUNCHER_SPINUP);
    }

    /**
     *
     */
    private void launch() {
        robot.launcherServo.setPosition(ON);
        sleep((long) LAUNCHER_LAUNCH);
        robot.launcherServo.setPosition(OFF);
        sleep((long) LAUNCHER_LAUNCH);
    }

    /**
     *
     */
    private void finish() {
        drive.followTrajectory(toFinish);

        FINAL_POSITION = drive.getPoseEstimate();

        while (!isStopRequested()) {
            idle();
        }
    }

    /**
     *
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VuforiaKey.KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     *
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
    }

    private void activateTfod() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
    }

    @SuppressLint("DefaultLocale")
    private void detectRings() {
        if (tfod != null) {
            ElapsedTime time = new ElapsedTime();
            String label = null;

            while (opModeIsActive() && time.seconds() < 5) {
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.clear();
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 1) {
                        Recognition recognition = updatedRecognitions.get(0);
                        label = recognition.getLabel();

                        break;
                    } else {
                        telemetry.addLine(String.format("Found %d recognitions", updatedRecognitions.size()));
                        telemetry.update();
                    }
                }
            }

            if (label == null) {
                wobbleGoalPosition = WOBBLE_GOAL_1;
            } else if (label.equals(SINGLE_LABEL)) {
                wobbleGoalPosition = WOBBLE_GOAL_2;
                wobbleGoalTangent = WOBBLE_GOAL_2_TANGENT;
            } else if (label.equals(QUAD_LABEL)) {
                wobbleGoalPosition = WOBBLE_GOAL_3;
            } else {
                wobbleGoalPosition = WOBBLE_GOAL_1;
            }

            telemetry.addData("tensorflow time", time.seconds());
            telemetry.addData("recognition label", label);
            telemetry.addLine(String.format("moving to goal at (%.1f, %.1f)", wobbleGoalPosition.getX(), wobbleGoalPosition.getY()));
            telemetry.update();
        }
    }
}

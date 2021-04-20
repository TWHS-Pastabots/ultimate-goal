package org.firstinspires.ftc.team16911.auton;


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
import org.firstinspires.ftc.team16911.drive.MacaroniMecanumDrive;
import org.firstinspires.ftc.team16911.drive.PoseStorage;
import org.firstinspires.ftc.team16911.hardware.MacaroniHardware;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Macaroni Autonomous", group = "drive", preselectTeleOp = "Macaroni")
public class MacaroniAutonomous extends LinearOpMode {
    // Vuforia/TFOD related variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.6;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public static final Pose2d START_POSITION = new Pose2d(-(72 - 10.5), 48);
    private static final double LAUNCHER_POWER = 0.73;
    private static final int PRESPIN_TIME = 4300;

    public static Pose2d WOBBLE_GOAL_1 = new Pose2d(-2.5300820012016763, 52.1048583194061, 0);
    public static Pose2d WOBBLE_GOAL_2 = new Pose2d(20.479536027344817, 28.077649944945843, 0);
    public static double WOBBLE_GOAL_2_TANGENT = Math.toRadians(-90);
    public static Pose2d WOBBLE_GOAL_3 = new Pose2d(44.020241167, 51.2568443, 0);
    private Pose2d wobbleGoalPosition;
    private double wobbleGoalTangent = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MacaroniMecanumDrive drive = new MacaroniMecanumDrive(hardwareMap);
        MacaroniHardware robot = new MacaroniHardware();
        robot.init(hardwareMap);

        activateTfod();


        drive.setPoseEstimate(START_POSITION);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        detectRings();

        Trajectory moveToWobblePlacement = drive.trajectoryBuilder(START_POSITION)
                .splineToLinearHeading(wobbleGoalPosition, wobbleGoalTangent)
                .build();

        Trajectory moveBehindShootingLine = drive.trajectoryBuilder(moveToWobblePlacement.end())
                .lineToLinearHeading(new Pose2d(0, 24))
                .build();

        Trajectory parkOnLine = drive.trajectoryBuilder(moveBehindShootingLine.end())
                .splineToLinearHeading(new Pose2d(12, 24), 0)
                .build();

        // Drive forward
        telemetry.addData("Status", "Driving towards wobble goal placement position");
        drive.followTrajectory(moveToWobblePlacement);

        // Lower arms and drop wobble goal
        telemetry.addData("Status", "Lowering arm");
        robot.armServo.setPosition(0.2);
        sleep(700);

        telemetry.addData("Status", "Opening claws");
        robot.setClaws(false);
        sleep(500);

        telemetry.addData("Status", "Re-opening claws");
        robot.setClaws(0.2);
        sleep(250);

        telemetry.addData("Status", "Raising arm");
        robot.armServo.setPosition(1);
        sleep(400);

        // Drive back to launching line
        telemetry.addData("Status", "Moving towards shooting position");
        drive.followTrajectory(moveBehindShootingLine);

        // Prepare for shooting sequence using prespin
        robot.setLauncherPower(LAUNCHER_POWER);
        robot.setRingStopper(true);
        telemetry.addData("Status", "Pre-shooting sequence motor spinup");
        sleep(PRESPIN_TIME);
        // Pre-spin ended, shoot them using belt
        robot.setRingStopper(false);
        sleep(250);
        robot.setIntakePower(0.4, 0);
        telemetry.addData("Status", "Shooting sequence");
        sleep(3500);


        // End shooting sequence, spin down & stop
        robot.setLauncherPower(0);
        robot.setIntakePower(0, 0);
        telemetry.addData("Status", "Ending shooting sequence");
        sleep(500);

        // Move back and park on line
        telemetry.addData("Status", "Moving to parking line");
        drive.followTrajectory(parkOnLine);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Status", "Idle");
            idle();
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void activateTfod() {
        // Initialize Vuforia and TFOD
        initVuforia();
        initTfod();

        // Activate TFOD if it can be activated
        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = AssetUtil.loadVuforiaKey();;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
    }

    @SuppressLint("DefaultLocale")
    private void detectRings() {
        if (tfod != null) {
            ElapsedTime time = new ElapsedTime();
            String label = null;

            // Loop while the opmode is active but only for a maximum of 5 seconds
            while (opModeIsActive() && time.seconds() < 5) {
                // Get a list of recognitions from TFOD
                List<Recognition> updatedRecognitions = tfod.getRecognitions();

                // Make sure we are able to get recognitions
                if (updatedRecognitions != null) {
                    // Clear telemetry and add a bit of data
                    telemetry.clear();
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // Check if we only have one recognition. This is to ensure that we
                    // don't mistake a false-positive recognition with the actual rings
                    if (updatedRecognitions.size() == 1) {
                        // Get the recognition and remember its label
                        Recognition recognition = updatedRecognitions.get(0);
                        label = recognition.getLabel();

                        // Break out of the loop because we've found the rings
                        break;
                    } else {
                        // Log how many recognitions we've found (useful for debugging)
                        telemetry.addLine(String.format("Found %d recognitions", updatedRecognitions.size()));
                        telemetry.update();
                    }
                }
            }

            // Check the label and store the position we need to drive to
            if (label == null) {
                wobbleGoalPosition = WOBBLE_GOAL_1;
            } else if (label.equals(SINGLE_LABEL)) {
                wobbleGoalPosition = WOBBLE_GOAL_2;

                // Store the tangent for this position, because if we don't, the robot might drive
                // on top of the ring, which will mess up the encoders and put it in the wrong
                // position. This actually happened to us in a match :(
                wobbleGoalTangent = WOBBLE_GOAL_2_TANGENT;
            } else if (label.equals(QUAD_LABEL)) {
                wobbleGoalPosition = WOBBLE_GOAL_3;
            } else {
                wobbleGoalPosition = WOBBLE_GOAL_1;
            }

            // Display some more telemetry information, which is, again, useful for debugging
            telemetry.addData("TensorFlow Time", time.seconds());
            telemetry.addData("Recognition Label", label);
            telemetry.addLine(String.format("moving to goal at (%.1f, %.1f)", wobbleGoalPosition.getX(), wobbleGoalPosition.getY()));
            telemetry.update();
        }
    }
}


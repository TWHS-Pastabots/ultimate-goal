package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16910.R;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.telop.Spaghetti;
import org.firstinspires.ftc.team16910.util.MotorUtil;
import org.firstinspires.ftc.team16910.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.List;

/**
 * TODO(BSFishy): document this
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
@Config
@Autonomous(name = "Spaghetti Autonomous", group = Spaghetti.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiAutonomous extends LinearOpMode {
    // Vuforia/TFOD related variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.65;

    // Useful symbolic on and off constants
    public static double ON = 1;
    public static double OFF = 0;

    // Timings for arm and claw actions
    public static double ARM_TIME = 600;
    public static double CLAW_TIME = 400;

    // Launcher-related constants
    public static double LAUNCHER_POWER = 0.5275; // 0.56
    public static double LAUNCHER_TARGET = MotorUtil.fromMotorPower(LAUNCHER_POWER);
    public static double LAUNCHER_THRESHOLD = 0.05;
    public static double LAUNCHER_SPINUP = 1;
    public static double LAUNCHER_STABILIZATION_TIMEOUT = 0.2;
    public static double LAUNCHER_SPINDOWN = 0.5;
    public static double LAUNCHER_LAUNCH = 0.4;

    // Wobble goal related variables
    public static Pose2d WOBBLE_GOAL_1 = new Pose2d(-1.7014121587073083, 55.303271023102646, Math.toRadians(358.8521558517273));
    public static Pose2d WOBBLE_GOAL_2 = new Pose2d(21.531418267740545, 30.705377032838875, Math.toRadians(3.0078201762876087));
    public static double WOBBLE_GOAL_2_TANGENT = Math.toRadians(-90);
    public static Pose2d WOBBLE_GOAL_3 = new Pose2d(44.935919028994086, 56.22700991949062, Math.toRadians(4.4929899518084815));

    public static Vector2d POWER_SHOT_SHIFT = new Vector2d(0, -8);
    public static Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, 27.561349542087463, 0.08275128001363097);
    public static Vector2d POWER_SHOT_2 = POWER_SHOT_1.vec().plus(POWER_SHOT_SHIFT);
    public static Vector2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

    // The final position of the robot. This is an estimate to begin with,
    // but is set at the end of the opmode so that it can be used elsewhere,
    // i.e. the Spaghetti teleop
    public static Pose2d FINAL_POSITION = new Pose2d(12, 18, 0);

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Pose2d wobbleGoalPosition;
    private double wobbleGoalTangent = 0;

    // Hardware-related variables
    private FtcDashboard dashboard;
    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;

    // Trajectories
    private Trajectory toWobbleGoal, toPowerShot1, toPowerShot1_2, toPowerShot2, toPowerShot3, toFinish;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Don't automatically clear the telemetry
        telemetry.setAutoClear(false);

        // Initialize the robot hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-63.64800204729765, 46.39877819447857, Math.toRadians(3.5512892424129374)));

        // Activate TFOD
        activateTfod();

        // Initialize the arm
        robot.clawServo.setPosition(ON);

        // Wait until the opmode should start
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        // Detect rings using TFOD
        detectRings();

        if (tfod != null) {
            tfod.shutdown();
        }

        // Prepare the trajectories needed throughout the opmode
        prepareTrajectories();

        // Update telemetry to display TFOD info, if possible
        telemetry.update();

        // Move to the wobble goal and drop it
        drive.followTrajectory(toWobbleGoal);
        dropWobbleGoal();

        // Update telemetry to display TFOD info, if possible
        telemetry.update();

        // Move to the powershot goal and shoot the rings
        drive.followTrajectory(toPowerShot1_2);
        launchPowerShots();

        // Move to the finish position and end the opmode
        finish();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void prepareTrajectories() {
        // Get the wobble goal position to drive to, if TFOD was able to detect
        // one, or use the default position
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
                .splineToLinearHeading(POWER_SHOT_1, Math.toRadians(0))
                .build();

        toPowerShot2 = drive.trajectoryBuilder(toPowerShot1_2.end())
                .lineTo(POWER_SHOT_2)
                .build();

        toPowerShot3 = drive.trajectoryBuilder(toPowerShot2.end())
                .lineTo(POWER_SHOT_3)
                .build();

        toFinish = drive.trajectoryBuilder(toPowerShot3.end())
                .splineToLinearHeading(FINAL_POSITION, 0)
                .build();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void dropWobbleGoal() {
        // Drop the arm servo and sleep so that it has time to get to that position
        robot.armServo.setPosition(OFF);
        sleep((long) ARM_TIME);

        // Open the claw servo and sleep so that it has time to get to that position
        robot.clawServo.setPosition(OFF);
        sleep((long) CLAW_TIME);

        // Drive backwards so that we don't have any issues with
        // the wobble goal and raising the arm
        drive.followTrajectory(toPowerShot1);

        // Raise the arm
        robot.armServo.setPosition(ON);
        sleep((long) ARM_TIME);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void launchPowerShots() {
        // Spin up the launcher motor and pause until it can spin to full speed
        robot.launcherMotor.setVelocity(MotorUtil.fromMotorPower(LAUNCHER_POWER));

        // Launch the rings and move to the correct positions
        launch();
        drive.followTrajectory(toPowerShot2);
        launch();
        drive.followTrajectory(toPowerShot3);
        launch();

        // Turn the launcher motor off and pause until it is off
        robot.launcherMotor.setPower(OFF);
        sleep((long) LAUNCHER_SPINDOWN * 1000);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void launch() {
        ElapsedTime launcherTimer = new ElapsedTime();
        ElapsedTime stabilizationTimer = new ElapsedTime();
        while (opModeIsActive() && launcherTimer.seconds() < LAUNCHER_SPINUP) {
            double launcherVelocity = robot.launcherMotor.getVelocity();
            double error = Math.abs(LAUNCHER_TARGET - launcherVelocity) / LAUNCHER_TARGET;

            if (error <= LAUNCHER_THRESHOLD) {
                if (stabilizationTimer.seconds() >= LAUNCHER_STABILIZATION_TIMEOUT) {
                    break;
                }
            } else {
                stabilizationTimer.reset();
            }
        }

        telemetry.addData("launcher motor", MotorUtil.toMotorPower(robot.launcherMotor.getVelocity()));
        telemetry.update();

        // Turn the launcher servo on and pause until it extends all the way
        robot.launcherServo.setPosition(ON);
        sleep((long) LAUNCHER_LAUNCH * 1000);

        // Turn the launcher servo off and pause until it has retracted all the way
        robot.launcherServo.setPosition(OFF);
        sleep((long) LAUNCHER_LAUNCH * 1000);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void finish() {
        // Move to the final position
        drive.followTrajectory(toFinish);

        // Set the final position constant so it can be used in other opmodes
        PoseStorage.position = drive.getPoseEstimate();

        dashboard.stopCameraStream();

        // Idle until the opmode should stop
        while (!isStopRequested()) {
            idle();
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = AssetUtil.loadVuforiaKey();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
        dashboard.startCameraStream(tfod, 10);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void activateTfod() {
        // Initialize Vuforia and TFOD
        initVuforia();
        initTfod();

        // Activate TFOD if it can be activated
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.25, 16.0 / 9.0);
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private void detectRings() {
        if (tfod == null) {
            return;
        }

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
                    telemetry.addData("recognitions", updatedRecognitions.size());
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
        telemetry.addData("tensorflow time", time.seconds());
        telemetry.addData("recognition label", label);
        telemetry.addData("moving to", "(%.01f, %.01f)", wobbleGoalPosition.getX(), wobbleGoalPosition.getY());
        telemetry.update();
    }
}

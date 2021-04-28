package org.firstinspires.ftc.team16910.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16910.R;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.telop.PoseStorage;
import org.firstinspires.ftc.team16910.telop.Spaghetti;
import org.firstinspires.ftc.team16910.util.TfodUtil;
import org.firstinspires.ftc.team16910.util.VuforiaUtil;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Config
@Autonomous(name = "Spaghetti Autonomous V2", preselectTeleOp = "Spaghetti")
public class SpaghettiAutonomousV2 extends LinearOpMode {
    private enum State {
        Running,
        Sleeping,
        Paused,
    }

    private enum Action {
        WobbleGoal,
        PowerShot,
        Finished,
    }

    private enum WobbleGoalStage {
        Start,
        MovingToTrajectory,
        DropArm,
        OpenClaw,
        MoveToSafeSpot,
        MovingToSafeSpot,
        RaiseArm,
    }

    private enum PowerShotStage {
        Start,
        MovingToPowerShot1,
        SpinUpLauncher,
        LaunchPowerShot1,
        MoveToPowerShot2,
        MovingToPowerShot2,
        LaunchPowerShot2,
        MoveToPowerShot3,
        MovingToPowerShot3,
        LaunchPowerShot3,
        SpinDownLauncher,
    }

    private enum LauncherState {
        Extend,
        Retract,
        Finish,
    }

    private enum FinishedStage {
        Start,
        MovingToFinalPosition,
        Idle,
    }

    private State currentState = State.Running;
    private Action currentAction = Action.WobbleGoal;

    private final ElapsedTime timer = new ElapsedTime();
    private double duration = 0;

    private WobbleGoalStage wobbleGoalStage = WobbleGoalStage.Start;
    private PowerShotStage powerShotStage = PowerShotStage.Start;
    private FinishedStage finishedStage = FinishedStage.Start;

    private LauncherState launcherState = LauncherState.Extend;

//    // Vuforia/TFOD related variables
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String QUAD_LABEL = "Quad";
//    private static final String SINGLE_LABEL = "Single";
//    public static double MIN_CONFIDENCE = 0.6;
//    private VuforiaLocalizer vuforia;
//    private TFObjectDetector tfod;
//
    private final VuforiaUtil vuforia = new VuforiaUtil(true);
    private final TfodUtil tfod = new TfodUtil(vuforia);
    private final ElapsedTime vuforiaTimer = new ElapsedTime();
    public static double VUFORIA_TIME = 0.5f;
//    private static final float mmPerInch = 25.4f;
//    private static final float mmTargetHeight = (6) * mmPerInch;
//    private static final float halfField = 72 * mmPerInch;
//    private static final float quadField = 36 * mmPerInch;
//    private OpenGLMatrix lastLocation = null;
//    private boolean targetVisible = false;
//    private float phoneXRotate = 0;
//    private float phoneYRotate = 90;
//    private float phoneZRotate = 0;
//    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
//    private final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//    private final float CAMERA_VERTICAL_DISPLACEMENT = 7.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
//    private final float CAMERA_LEFT_DISPLACEMENT = 7.5f * mmPerInch;     // eg: Camera is ON the robot's center line
//    private VuforiaTrackables targetsUltimateGoal;
    private int vuforiaTracks = 0;
    private int vuforiaFound = 0;

    // Useful symbolic on and off constants
    public static double ON = 1;
    public static double OFF = 0;

    // Timings for arm and claw actions
    public static double ARM_TIME = 600;
    public static double CLAW_TIME = 400;

    // Launcher-related constants
    public static double LAUNCHER_POWER = 0.6; // 0.56
    public static double LAUNCHER_SPINUP = 1000;
    public static double LAUNCHER_LAUNCH = 600;

    // Wobble goal related variables
    public static Pose2d WOBBLE_GOAL_1 = new Pose2d(-2.5300820012016763, 52.1048583194061, 0);
    public static Pose2d WOBBLE_GOAL_2 = new Pose2d(20.479536027344817, 28.077649944945843, 0);
    public static double WOBBLE_GOAL_2_TANGENT = Math.toRadians(-90);
    public static Pose2d WOBBLE_GOAL_3 = new Pose2d(44.020241167, 51.2568443, 0);
    private Pose2d wobbleGoalPosition;
    private double wobbleGoalTangent = 0;

    // The final position of the robot. This is an estimate to begin with,
    // but is set at the end of the opmode so that it can be used elsewhere,
    // i.e. the Spaghetti teleop
    public static Pose2d FINAL_POSITION = new Pose2d(12, 18, 0);

    // Hardware-related variables
    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;

    // Trajectories
    private Trajectory toWobbleGoal, toSafeSpot, toPowerShot1, toPowerShot2, toPowerShot3, toFinish;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the robot hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-62.358, 48.24, 0));

        // Activate TFOD
        vuforia.activate(hardwareMap);
        tfod.activate(hardwareMap);
//        activateTfod();

        // Initialize the arm
        robot.clawServo.setPosition(ON);

        // Wait until the opmode should start
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        // Detect rings using TFOD
        detectRings();

        // Prepare the trajectories needed throughout the opmode
        prepareTrajectories();

        while (opModeIsActive()) {
            if (!drive.isBusy() || currentState == State.Sleeping) {
                detectVuforia();
            }

            addTelemetry();

            switch (currentState) {
                case Sleeping:
                    if (timer.milliseconds() >= duration) {
                        currentState = State.Running;
                    }

                    break;
                case Running:
                    runState();

                    break;
            }

            drive.update();
            telemetry.update();
        }

        tfod.deactivate();
        vuforia.deactivate();

        // Set the final position constant so it can be used in other opmodes
//        PoseStorage.position = drive.getPoseEstimate();

        // Idle until the opmode should stop
        while (!isStopRequested()) {
            idle();
        }
    }

    /**
     *
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

        toSafeSpot = drive.trajectoryBuilder(toWobbleGoal.end())
                .forward(-12)
                .build();

        toPowerShot1 = drive.trajectoryBuilder(toSafeSpot.end())
                .splineToLinearHeading(new Pose2d(1.7205879405195255, 30.3902133119823317, 0), Math.toRadians(0))
                .build();

        toPowerShot2 = drive.trajectoryBuilder(toPowerShot1.end())
                .strafeRight(8)
                .build();

        toPowerShot3 = drive.trajectoryBuilder(toPowerShot2.end())
                .strafeRight(8)
                .build();

        toFinish = drive.trajectoryBuilder(toPowerShot3.end())
                .splineToLinearHeading(FINAL_POSITION, 0)
                .build();
    }

    private void detectVuforia() {
        if (vuforiaTimer.seconds() < VUFORIA_TIME) {
            return;
        }

        vuforiaTimer.reset();
        vuforiaTracks++;

        Pose2d pose = vuforia.getPosition();
        if (pose == null) {
            return;
        }

        vuforiaFound++;
        drive.setPoseEstimate(pose);
    }

//    private void detectVuforia() {
//        if (vuforiaTimer.seconds() < VUFORIA_TIME) {
//            return;
//        }
//
//        vuforiaTimer.reset();
//        vuforiaTracks++;
//
//        targetVisible = false;
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                targetVisible = true;
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//
//                break;
//            }
//        }
//
//        if (targetVisible) {
//            vuforiaFound++;
//
//            VectorF translation = lastLocation.getTranslation();
//
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//
//            Pose2d pose = new Pose2d(
//                    translation.get(0) / mmPerInch,
//                    translation.get(1) / mmPerInch,
//                    Math.toRadians(rotation.thirdAngle)
//            );
//
//            drive.setPoseEstimate(pose);
//        }
//    }

    private void addTelemetry() {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", "%.02f", poseEstimate.getX());
        telemetry.addData("y", "%.02f", poseEstimate.getY());
        telemetry.addData("heading", "%.02f", poseEstimate.getHeading());

        telemetry.addData("state", currentState);
        if (currentState == State.Sleeping) {
            telemetry.addData("time", timer.toString());
        }

        telemetry.addData("action", currentAction);

        switch (currentAction) {
            case WobbleGoal:
                telemetry.addData("stage", wobbleGoalStage);

                break;
            case PowerShot:
                telemetry.addData("stage", powerShotStage);
                telemetry.addData("launcher state", launcherState);
                telemetry.addData("launcher power", Spaghetti.toMotorPower(robot.launcherMotor.getVelocity()));

                break;
            case Finished:
                break;
        }

        telemetry.addData("vuforia tracks", vuforiaTracks);
        telemetry.addData("vuforia found", vuforiaFound);
    }

    private void runState() {
        switch (currentAction) {
            case WobbleGoal:
                wobbleGoalAction();

                break;
            case PowerShot:
                powerShotAction();

                break;
            case Finished:
                finishedAction();

                break;
        }
    }

    private void wobbleGoalAction() {
        switch (wobbleGoalStage) {
            case Start:
                drive.followTrajectoryAsync(toWobbleGoal);
                wobbleGoalStage = WobbleGoalStage.MovingToTrajectory;

                break;
            case MovingToTrajectory:
                if (!drive.isBusy()) {
                    wobbleGoalStage = WobbleGoalStage.DropArm;
                }

                break;
            case DropArm:
                robot.armServo.setPosition(OFF);

                sleepState(ARM_TIME);
                wobbleGoalStage = WobbleGoalStage.OpenClaw;

                break;
            case OpenClaw:
                robot.clawServo.setPosition(OFF);

                sleepState(CLAW_TIME);
                wobbleGoalStage = WobbleGoalStage.MoveToSafeSpot;

                break;
            case MoveToSafeSpot:
                drive.followTrajectoryAsync(toSafeSpot);
                wobbleGoalStage = WobbleGoalStage.MovingToSafeSpot;

                break;
            case MovingToSafeSpot:
                if (!drive.isBusy()) {
                    wobbleGoalStage = WobbleGoalStage.RaiseArm;
                }

                break;
            case RaiseArm:
                robot.armServo.setPosition(ON);

                sleepState(ARM_TIME);
                currentAction = Action.PowerShot;

                break;
        }
    }

    private void powerShotAction() {
        switch (powerShotStage) {
            case Start:
                drive.followTrajectoryAsync(toPowerShot1);
                powerShotStage = PowerShotStage.MovingToPowerShot1;

                break;
            case MovingToPowerShot1:
                if (!drive.isBusy()) {
                    powerShotStage = PowerShotStage.SpinUpLauncher;
                }

                break;
            case SpinUpLauncher:
                robot.launcherMotor.setVelocity(Spaghetti.fromMotorPower(LAUNCHER_POWER));

                sleepState(LAUNCHER_SPINUP);
                powerShotStage = PowerShotStage.LaunchPowerShot1;

                break;
            case LaunchPowerShot1:
                if (launchAction()) {
                    powerShotStage = PowerShotStage.MoveToPowerShot2;
                }

                break;
            case MoveToPowerShot2:
                drive.followTrajectoryAsync(toPowerShot2);
                powerShotStage = PowerShotStage.MovingToPowerShot2;

                break;
            case MovingToPowerShot2:
                if (!drive.isBusy()) {
                    powerShotStage = PowerShotStage.LaunchPowerShot2;
                }

                break;
            case LaunchPowerShot2:
                if (launchAction()) {
                    powerShotStage = PowerShotStage.MoveToPowerShot3;
                }

                break;
            case MoveToPowerShot3:
                drive.followTrajectoryAsync(toPowerShot3);
                powerShotStage = PowerShotStage.MovingToPowerShot3;

                break;
            case MovingToPowerShot3:
                if (!drive.isBusy()) {
                    powerShotStage = PowerShotStage.LaunchPowerShot3;
                }

                break;
            case LaunchPowerShot3:
                if (launchAction()) {
                    powerShotStage = PowerShotStage.SpinDownLauncher;
                }

                break;
            case SpinDownLauncher:
                robot.launcherMotor.setPower(OFF);

                sleepState(LAUNCHER_SPINUP);
                currentAction = Action.Finished;

                break;
        }
    }

    private boolean launchAction() {
        switch (launcherState) {
            case Extend:
                robot.launcherServo.setPosition(ON);

                sleepState(LAUNCHER_LAUNCH);
                launcherState = LauncherState.Retract;

                break;
            case Retract:
                robot.launcherServo.setPosition(OFF);

                sleepState(LAUNCHER_LAUNCH);
                launcherState = LauncherState.Finish;

                break;
            case Finish:
                launcherState = LauncherState.Extend;

                return true;
        }

        return false;
    }

    private void finishedAction() {
        switch (finishedStage) {
            case Start:
                drive.followTrajectoryAsync(toFinish);
                finishedStage = FinishedStage.MovingToFinalPosition;

                break;
            case MovingToFinalPosition:
                if (!drive.isBusy()) {
                    PoseStorage.position = drive.getPoseEstimate();
                    finishedStage = FinishedStage.Idle;
                }

                break;
            case Idle:
                idle();

                break;
        }
    }

    private void sleepState(double duration) {
        timer.reset();
        this.duration = duration;

        currentState = State.Sleeping;
    }

    private void detectRings() {
        TfodUtil.Rings rings = tfod.getRings();

        switch (rings) {
            case None:
                wobbleGoalPosition = WOBBLE_GOAL_1;

                break;
            case Single:
                wobbleGoalPosition = WOBBLE_GOAL_2;
                wobbleGoalTangent = WOBBLE_GOAL_2_TANGENT;

                break;
            case Quad:
                wobbleGoalPosition = WOBBLE_GOAL_3;

                break;
        }
    }

//    /**
//     *
//     */
//    private void initVuforia() {
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = AssetUtil.loadVuforiaKey();
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
////        parameters.useExtendedTracking = true;
//
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
//        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
//        blueTowerGoalTarget.setName("Blue Tower Goal Target");
//        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
//        redTowerGoalTarget.setName("Red Tower Goal Target");
//        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
//        redAllianceTarget.setName("Red Alliance Target");
//        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
//        blueAllianceTarget.setName("Blue Alliance Target");
//        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
//        frontWallTarget.setName("Front Wall Target");
//
//        allTrackables.addAll(targetsUltimateGoal);
//
//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
//
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
//        }
//    }
//
//    /**
//     *
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
//    }
//
//    /**
//     *
//     */
//    private void activateTfod() {
//        // Initialize Vuforia and TFOD
//        initVuforia();
//        initTfod();
//
//        targetsUltimateGoal.activate();
//
//        // Activate TFOD if it can be activated
//        if (tfod != null) {
//            tfod.activate();
//        }
//    }

//    /**
//     *
//     */
//    @SuppressLint("DefaultLocale")
//    private void detectRings() {
//        if (tfod != null) {
//            ElapsedTime time = new ElapsedTime();
//            String label = null;
//
//            // Loop while the opmode is active but only for a maximum of 5 seconds
//            while (opModeIsActive() && time.seconds() < 5) {
//                // Get a list of recognitions from TFOD
//                List<Recognition> updatedRecognitions = tfod.getRecognitions();
//
//                // Make sure we are able to get recognitions
//                if (updatedRecognitions != null) {
//                    // Clear telemetry and add a bit of data
//                    telemetry.clear();
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                    // Check if we only have one recognition. This is to ensure that we
//                    // don't mistake a false-positive recognition with the actual rings
//                    if (updatedRecognitions.size() == 1) {
//                        // Get the recognition and remember its label
//                        Recognition recognition = updatedRecognitions.get(0);
//                        label = recognition.getLabel();
//
//                        // Break out of the loop because we've found the rings
//                        break;
//                    } else {
//                        // Log how many recognitions we've found (useful for debugging)
//                        telemetry.addLine(String.format("Found %d recognitions", updatedRecognitions.size()));
//                        telemetry.update();
//                    }
//                }
//            }
//
//            // Check the label and store the position we need to drive to
//            if (label == null) {
//                wobbleGoalPosition = WOBBLE_GOAL_1;
//            } else if (label.equals(SINGLE_LABEL)) {
//                wobbleGoalPosition = WOBBLE_GOAL_2;
//
//                // Store the tangent for this position, because if we don't, the robot might drive
//                // on top of the ring, which will mess up the encoders and put it in the wrong
//                // position. This actually happened to us in a match :(
//                wobbleGoalTangent = WOBBLE_GOAL_2_TANGENT;
//            } else if (label.equals(QUAD_LABEL)) {
//                wobbleGoalPosition = WOBBLE_GOAL_3;
//            } else {
//                wobbleGoalPosition = WOBBLE_GOAL_1;
//            }
//
//            // Display some more telemetry information, which is, again, useful for debugging
//            telemetry.addData("tensorflow time", time.seconds());
//            telemetry.addData("recognition label", label);
//            telemetry.addLine(String.format("moving to goal at (%.1f, %.1f)", wobbleGoalPosition.getX(), wobbleGoalPosition.getY()));
//            telemetry.update();
//        }
//    }
}

package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.team16910.R;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.util.MotorUtil;
import org.firstinspires.ftc.team16910.util.OpmodeUtil;
import org.firstinspires.ftc.team16910.util.PoseStorage;
import org.firstinspires.ftc.team16910.util.Position;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.List;

import static org.firstinspires.ftc.team16910.auton.Position.SAFE_SPOT_DISTANCE;

/**
 * TODO(BSFishy): document this
 */
@Config
public abstract class AbstractSpaghettiAutonomous extends LinearOpMode {
    // Vuforia/TFOD related variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.65;
    // Timings for arm and claw actions
    public static double ARM_TIME = 0.6;
    public static double CLAW_TIME = 0.4;
    // Launcher-related constants
    public static double LAUNCHER_POWER = 0.5;
    public static double LAUNCHER_THRESHOLD = 0.05;
    public static double LAUNCHER_SPINUP = 2.5;
    public static double LAUNCHER_STABILIZATION_TIMEOUT = 0.25;
    public static double LAUNCHER_SPINDOWN = 0.5;
    public static double LAUNCHER_LAUNCH = 0.6;
    public static double ZOOM_RATIO = 1.25;
    public static double ZOOM_ASPECT_RATIO = 16.0 / 9.0;
    // Trajectories
    protected Pose2d startPosition;
    protected Position powerShot1Position, powerShot2Position, powerShot3Position, finishPosition;
    protected Trajectory toWobbleGoal, toSafeSpot, toPowerShot1, toPowerShot2, toPowerShot3, toFinish;
    protected boolean performWobbleGoal = true;
    protected boolean performPowerShots = true;
    protected boolean performPark = true;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    // Hardware-related variables
    private FtcDashboard dashboard;
    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;
    private RingCount ringCount = RingCount.None;

    /**
     * TODO(BSFishy): document this
     *
     * @return whether or not the opmode is active without idling
     */
    protected boolean isActive() {
        return !this.isStopRequested() && this.isStarted();
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current dashboard instance
     */
    protected FtcDashboard getDashboard() {
        return dashboard;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current hardware instance
     */
    protected SpaghettiHardware getHardware() {
        return robot;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current drive train instance
     */
    protected SampleMecanumDrive getDriveTrain() {
        return drive;
    }

    /**
     * TODO(BSFishy): document this
     */
    protected abstract void prepare();

    /**
     * TODO(BSFishy): document this
     *
     * @return the position to move to the wobble goal
     */
    protected abstract Position getWobbleGoalPosition();

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

        prepare();

        Assert.assertNotNull(startPosition, "The start position must not be null");
        drive.setPoseEstimate(startPosition);

        activateTfod();

        // Initialize the arm
        robot.closeClaw();

        // Wait until the opmode should start
        waitForStart();
        if (!isActive()) {
            return;
        }

        detectRings();

        if (tfod != null) {
            tfod.shutdown();
        }

        prepareTrajectories();

        checkPositions();

        // Update telemetry to display TFOD info, if possible
        telemetry.update();

        run();

        finish();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void checkPositions() {
        Assert.assertNotNull(powerShot1Position, "You must initialize all of the positions including power shot 1");
        Assert.assertNotNull(powerShot2Position, "You must initialize all of the positions including power shot 2");
        Assert.assertNotNull(powerShot3Position, "You must initialize all of the positions including power shot 3");
        Assert.assertNotNull(finishPosition, "You must initialize all of the positions including finish");
    }

    /**
     * TODO(BSFishy): document this
     */
    private void prepareTrajectories() {
        Pose2d lastPose = drive.getPoseEstimate();

        if (performWobbleGoal) {
            Position wobbleGoalPosition = getWobbleGoalPosition();
            Assert.assertNotNull(wobbleGoalPosition, "You must initialize all of the positions including wobble goal");

            toWobbleGoal = wobbleGoalPosition
                    .toSpline(drive.trajectoryBuilder(lastPose))
                    .build();

            toSafeSpot = drive.trajectoryBuilder(toWobbleGoal.end())
                    .forward(-SAFE_SPOT_DISTANCE)
                    .build();

            lastPose = toSafeSpot.end();
        }

        if (performPowerShots) {
            toPowerShot1 = powerShot1Position
                    .toSpline(drive.trajectoryBuilder(lastPose))
                    .build();

            toPowerShot2 = powerShot2Position
                    .toLine(drive.trajectoryBuilder(toPowerShot1.end()))
                    .build();

            toPowerShot3 = powerShot3Position
                    .toLine(drive.trajectoryBuilder(toPowerShot2.end()))
                    .build();

            lastPose = toPowerShot3.end();
        }

        if (performPark) {
            toFinish = finishPosition
                    .toSpline(drive.trajectoryBuilder(lastPose))
                    .build();
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private void run() {
        if (performWobbleGoal) {
            doWobbleGoal();
        }

        if (performPowerShots) {
            doPowerShots();
        }

        if (performPark) {
            doPark();
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private void doWobbleGoal() {
        drive.followTrajectory(toWobbleGoal);

        lowerWobbleArm();

        openClaw();

        drive.followTrajectory(toSafeSpot);

        raiseWobbleArm();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void doPowerShots() {
        spinUpLauncherAid();
        spinUpLauncher();

        drive.followTrajectory(toPowerShot1);
        launch();

        drive.followTrajectory(toPowerShot2);
        launch();

        drive.followTrajectory(toPowerShot3);
        launch();

        spinDownLauncherAid();
        spinDownLauncher();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void doPark() {
        drive.followTrajectory(toFinish);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void lowerWobbleArm() {
        robot.lowerWobbleArm();
        OpmodeUtil.sleep(ARM_TIME, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void openClaw() {
        robot.openClaw();
        OpmodeUtil.sleep(CLAW_TIME, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void closeClaw() {
        robot.closeClaw();
        OpmodeUtil.sleep(CLAW_TIME, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void raiseWobbleArm() {
        robot.raiseWobbleArm();
        OpmodeUtil.sleep(ARM_TIME, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void spinUpLauncher() {
        // Spin up the launcher motor and pause until it can spin to full speed
        robot.spinLauncherRaw(MotorUtil.fromMotorPower(LAUNCHER_POWER));
    }

    protected void spinUpLauncherAid() {
        robot.spinLauncherAid(true, false);
    }

    protected void spinDownLauncherAid() {
        robot.spinLauncherAid(false, false);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void launch() {
        MotorUtil.waitToSpinTo(robot.launcherMotor, LAUNCHER_POWER, LAUNCHER_THRESHOLD, LAUNCHER_SPINUP, LAUNCHER_STABILIZATION_TIMEOUT, this);

        telemetry.addData("launcher motor", MotorUtil.toMotorPower(robot.launcherMotor.getVelocity()));
        telemetry.update();

        // Turn the launcher servo on and pause until it extends all the way
        robot.extendLauncherServo();
        OpmodeUtil.sleep(LAUNCHER_LAUNCH, this);

        // Turn the launcher servo off and pause until it has retracted all the way
        robot.retractLauncherServo();
        OpmodeUtil.sleep(LAUNCHER_LAUNCH, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    protected void spinDownLauncher() {
        // Turn the launcher motor off and pause until it is off
        MotorUtil.spinTo(robot.launcherMotor, 0, LAUNCHER_THRESHOLD, LAUNCHER_SPINDOWN, LAUNCHER_STABILIZATION_TIMEOUT, this);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void finish() {
        // Set the final position constant so it can be used in other opmodes
        PoseStorage.position = drive.getPoseEstimate();

        dashboard.stopCameraStream();

        telemetry.update();

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

            tfod.setZoom(ZOOM_RATIO, ZOOM_ASPECT_RATIO);
        }
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the number of detected wobble goals
     */
    protected RingCount getRingCount() {
        return ringCount;
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

        if (label == null) {
            ringCount = RingCount.None;
        } else if (label.equals(SINGLE_LABEL)) {
            ringCount = RingCount.Single;
        } else if (label.equals(QUAD_LABEL)) {
            ringCount = RingCount.Quad;
        }

        // Display some more telemetry information, which is, again, useful for debugging
        telemetry.addData("tensorflow time", time.seconds());
        telemetry.addData("recognition label", label);
        telemetry.update();
    }

    public enum RingCount {
        None,
        Single,
        Quad,
    }
}

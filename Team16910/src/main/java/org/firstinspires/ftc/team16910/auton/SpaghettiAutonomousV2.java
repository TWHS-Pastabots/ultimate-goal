package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16910.R;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.telop.Spaghetti;
import org.firstinspires.ftc.team16910.util.MotorUtil;
import org.firstinspires.ftc.team16910.util.OpmodeUtil;
import org.firstinspires.ftc.team16910.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.team16910.auton.Position.SAFE_SPOT_DISTANCE;

/**
 * TODO(BSFishy): document this
 * Game Manual 2, section 4.5.1
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
@Config
@Autonomous(name = "Spaghetti Autonomous V2", group = Spaghetti.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiAutonomousV2 extends LinearOpMode {
    // Vuforia/TFOD related variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.65;

    // Timings for arm and claw actions
    public static double ARM_TIME = 0.4;
    public static double CLAW_TIME = 0.3;

    // Launcher-related constants
    public static double LAUNCHER_POWER = SpaghettiAutonomous.LAUNCHER_POWER;
    public static double LAUNCHER_THRESHOLD = SpaghettiAutonomous.LAUNCHER_THRESHOLD;
    public static double LAUNCHER_SPINUP = SpaghettiAutonomous.LAUNCHER_SPINUP;
    public static double LAUNCHER_STABILIZATION_TIMEOUT = SpaghettiAutonomous.LAUNCHER_STABILIZATION_TIMEOUT;
    public static double LAUNCHER_SPINDOWN = SpaghettiAutonomous.LAUNCHER_SPINDOWN;
    public static double LAUNCHER_LAUNCH = SpaghettiAutonomous.LAUNCHER_LAUNCH;

    public static double RING_WAIT_TIME = 0.5;
    public static double ZOOM_RATIO = 1.5;
    public static double ZOOM_ASPECT_RATIO = 16.0 / 9.0;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Hardware-related variables
    private FtcDashboard dashboard;
    private SpaghettiHardware robot;
    private SampleMecanumDrive drive;
    private RingCount ringCount = null;

    private Alliance alliance = Alliance.Blue;
    private Location location = Location.Outer;
    private final List<Step> steps = new ArrayList<>(Arrays.asList(Step.WobbleGoal, Step.PowerShot, Step.Park));

    private Trajectory[][] trajectories;

    /**
     * TODO(BSFishy): document this
     *
     * @return whether or not the opmode is active without idling
     */
    protected boolean isActive() {
        return !this.isStopRequested() && this.isStarted();
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Don't automatically clear the telemetry
        telemetry.setAutoClear(false);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Initialize the robot hardware
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Initialize the drive train
        drive = new SampleMecanumDrive(hardwareMap);

        // Activate TFOD
        activateTfod();

        // Initialize the arm
        robot.closeClaw();

        configure();

        telemetry.clear();
        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait until the opmode should start
        waitForStart();
        if (!isActive()) {
            return;
        }

        prepareTrajectories();

        if (tfod != null) {
            tfod.shutdown();
        }

        for (int i = 0; i < steps.size(); i++) {
            Step step = steps.get(i);
            Trajectory[] t = trajectories[i];

            switch (step) {
                case WobbleGoal:
                    doWobbleGoal(t);

                    break;
                case PowerShot:
                    doPowerShots(t);

                    break;
                case Park:
                    doPark(t);

                    break;
                case Wait5:
                    doWait5();

                    break;
            }

            telemetry.update();
        }

        // Move to the finish position and end the opmode
        finish();
    }

    /**
     * TODO(BSFishy): document this
     */
    private void configure() {
        ConfigurationStep configStep = ConfigurationStep.Alliance;
        int currentStep = 0;

        boolean previousPreviousState = false;
        boolean previousNextState = false;

        boolean previousPreviousStepState = false;
        boolean previousNextStepState = false;

        boolean previousPreviousTypeState = false;
        boolean previousNextTypeState = false;

        boolean previousAddTypeState = false;
        boolean previousRemoveTypeState = false;

        config_loop:
        while (!isStarted() && !isStopRequested()) {
            telemetry.clear();

            boolean previousState = gamepad1.left_bumper;
            boolean nextState = gamepad1.right_bumper;
            boolean previousDown = previousPreviousState != previousState && previousState;
            boolean nextDown = previousNextState != nextState && nextState;

            boolean previousStepState = gamepad1.b;
            boolean nextStepState = gamepad1.a;
            boolean previousStepDown = previousPreviousStepState != previousStepState && previousStepState;
            boolean nextStepDown = previousNextStepState != nextStepState && nextStepState;

            boolean previousTypeState = gamepad1.dpad_left;
            boolean nextTypeState = gamepad1.dpad_right;
            boolean previousTypeDown = previousPreviousTypeState != previousTypeState && previousTypeState;
            boolean nextTypeDown = previousNextTypeState != nextTypeState && nextTypeState;

            boolean addTypeState = gamepad1.dpad_up;
            boolean removeTypeState = gamepad1.dpad_down;
            boolean addTypeDown = previousAddTypeState != addTypeState && addTypeState;
            boolean removeTypeDown = previousRemoveTypeState != removeTypeState && removeTypeState;

            switch (configStep) {
                case Alliance:
                    if (nextStepDown) {
                        alliance = alliance.next();
                    }

                    if (previousStepDown) {
                        alliance = alliance.previous();
                    }

                    break;
                case Location:
                    if (nextStepDown) {
                        location = location.next();
                    }

                    if (previousStepDown) {
                        location = location.previous();
                    }

                    break;
                case Steps:
                    if (nextStepDown) {
                        currentStep = Math.max(Math.min(currentStep + 1, steps.size() + 1), 0);
                    }

                    if (previousStepDown) {
                        currentStep = Math.max(Math.min(currentStep - 1, steps.size() + 1), 0);
                    }

                    if (addTypeDown) {
                        steps.add(currentStep, Step.values()[0]);
                    }

                    if (removeTypeDown) {
                        steps.remove(currentStep);
                    }

                    while (currentStep >= steps.size()) {
                        steps.add(Step.values()[0]);
                    }

                    Step step = steps.get(currentStep);
                    if (nextTypeDown) {
                        step = step.next();
                    }

                    if (previousTypeDown) {
                        step = step.previous();
                    }

                    steps.set(currentStep, step);

                    break;
                case Finish:
                    if (nextDown) {
                        break config_loop;
                    }

                    break;
            }

            if (configStep == ConfigurationStep.Alliance) {
                telemetry.addData("Alliance", "[<b>%s</b>]", alliance);
            } else {
                telemetry.addData("Alliance", alliance);
            }

            if (configStep == ConfigurationStep.Location) {
                telemetry.addData("Location", "[<b>%s</b>]", location);
            } else {
                telemetry.addData("Location", location);
            }

            StringBuilder stepsString = new StringBuilder();
            for (int i = 0; i < steps.size(); i++) {
                Step step = steps.get(i);

                if (i != 0) {
                    stepsString.append(", ");
                }

                if (i == currentStep && configStep == ConfigurationStep.Steps) {
                    stepsString.append("[<b>").append(step).append("</b>]");
                } else {
                    stepsString.append(step);
                }
            }

            telemetry.addData("Steps", stepsString.toString());

            if (configStep == ConfigurationStep.Steps) {
                telemetry.addLine();

                StringBuilder allStepsString = new StringBuilder();
                for (int i = 0; i < Step.values().length; i++) {
                    Step step = Step.values()[i];

                    if (i != 0) {
                        allStepsString.append(", ");
                    }

                    allStepsString.append(step);
                }

                telemetry.addData("All steps", allStepsString.toString());
            }

            if (configStep == ConfigurationStep.Finish) {
                telemetry.addLine("Press right bumper to finish setting up");
            }

            if (previousDown) {
                configStep = configStep.previous();
            }

            if (nextDown) {
                configStep = configStep.next();
            }

            previousPreviousState = previousState;
            previousNextState = nextState;

            previousPreviousStepState = previousStepState;
            previousNextStepState = nextStepState;

            previousPreviousTypeState = previousTypeState;
            previousNextTypeState = nextTypeState;

            previousAddTypeState = addTypeState;
            previousRemoveTypeState = removeTypeState;

            telemetry.update();

            idle();
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private void prepareTrajectories() {
        trajectories = new Trajectory[steps.size()][];

        switch (alliance) {
            case Blue:
                switch (location) {
                    case Inner:
                        drive.setPoseEstimate(Position.Blue.START_INNER);

                        break;
                    case Outer:
                        drive.setPoseEstimate(Position.Blue.START_OUTER);

                        break;
                }

                break;
            case Red:
                switch (location) {
                    case Inner:
                        drive.setPoseEstimate(Position.Red.START_INNER);

                        break;
                    case Outer:
                        drive.setPoseEstimate(Position.Red.START_OUTER);

                        break;
                }

                break;
        }

        Pose2d lastPose = drive.getPoseEstimate();
        for (int i = 0; i < steps.size(); i++) {
            Step step = steps.get(i);
            Trajectory[] t = null;

            switch (step) {
                case WobbleGoal:
                    t = new Trajectory[2];

                    if (ringCount == null) {
                        detectRings();
                    }

                    switch (alliance) {
                        case Blue:
                            switch (ringCount) {
                                case None:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Blue.WOBBLE_GOAL_A, 0)
                                            .build();

                                    break;
                                case Single:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Blue.WOBBLE_GOAL_B, Math.toRadians(90) * (location == Location.Outer ? -1 : 1))
                                            .build();

                                    break;
                                case Quad:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Blue.WOBBLE_GOAL_C, Math.toRadians(90) * (location == Location.Outer ? 0 : 1))
                                            .build();

                                    break;
                            }

                            break;
                        case Red:
                            switch (ringCount) {
                                case None:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Red.WOBBLE_GOAL_A, 0)
                                            .build();

                                    break;
                                case Single:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Red.WOBBLE_GOAL_B, Math.toRadians(90) * (location == Location.Outer ? 1 : -1))
                                            .build();

                                    break;
                                case Quad:
                                    t[0] = drive.trajectoryBuilder(lastPose)
                                            .splineToLinearHeading(Position.Red.WOBBLE_GOAL_C, Math.toRadians(90) * (location == Location.Outer ? 0 : -1))
                                            .build();

                                    break;
                            }

                            break;
                    }

                    t[1] = drive.trajectoryBuilder(t[0].end())
                            .forward(-SAFE_SPOT_DISTANCE)
                            .build();
                    lastPose = t[1].end();

                    break;
                case PowerShot:
                    t = new Trajectory[3];

                    switch (alliance) {
                        case Blue:
                            t[0] = drive.trajectoryBuilder(lastPose)
                                    .lineToLinearHeading(Position.Blue.POWER_SHOT_1)
                                    .build();

                            t[1] = drive.trajectoryBuilder(t[0].end())
                                    .lineToLinearHeading(Position.Blue.POWER_SHOT_2)
                                    .build();

                            t[2] = drive.trajectoryBuilder(t[1].end())
                                    .lineToLinearHeading(Position.Blue.POWER_SHOT_3)
                                    .build();

                            break;
                        case Red:
                            t[0] = drive.trajectoryBuilder(lastPose)
                                    .lineToLinearHeading(Position.Red.POWER_SHOT_1)
                                    .build();

                            t[1] = drive.trajectoryBuilder(t[0].end())
                                    .lineToLinearHeading(Position.Red.POWER_SHOT_2)
                                    .build();

                            t[2] = drive.trajectoryBuilder(t[1].end())
                                    .lineToLinearHeading(Position.Red.POWER_SHOT_3)
                                    .build();

                            break;
                    }

                    lastPose = t[2].end();

                    break;
                case Park:
                    t = new Trajectory[1];
                    Pose2d targetPosition = new Pose2d(Position.PARK_X, lastPose.getY(), 0);

                    t[0] = drive.trajectoryBuilder(lastPose)
                            .lineToLinearHeading(targetPosition)
                            .build();

//                    switch(alliance) {
//                        case Blue:
//                            Pose2d targetPosition = new Pose2d(Position.PARK_X, lastPose.getY(), 0);
//
//                            t[0] = drive.trajectoryBuilder(lastPose)
//                                    .lineToLinearHeading(targetPosition)
//                                    .build();
//
//                            break;
//                        case Red:
//                            t[0] = drive.trajectoryBuilder(lastPose)
//                                    .lineToLinearHeading(Position.Red.FINISH_OUTER)
//                                    .build();
//
//                            break;
//                    }

                    lastPose = t[0].end();

                    break;
            }

            trajectories[i] = t;
        }
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param trajectories the trajectories associated with this action
     */
    private void doWobbleGoal(Trajectory[] trajectories) {
        Trajectory toWobbleGoal = trajectories[0];
        Trajectory toSafeSpot = trajectories[1];

        drive.followTrajectory(toWobbleGoal);

        lowerWobbleArm();

        openClaw();

        drive.followTrajectory(toSafeSpot);

        raiseWobbleArm();
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param trajectories the trajectories associated with this action
     */
    private void doPowerShots(Trajectory[] trajectories) {
        Trajectory toPowerShot1 = trajectories[0];
        Trajectory toPowerShot2 = trajectories[1];
        Trajectory toPowerShot3 = trajectories[2];

        spinUpLauncher();

        drive.followTrajectory(toPowerShot1);
        launch();

        drive.followTrajectory(toPowerShot2);
        launch();

        drive.followTrajectory(toPowerShot3);
        launch();

        spinDownLauncher();
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param trajectories the trajectories associated with this action
     */
    private void doPark(Trajectory[] trajectories) {
        Trajectory toFinish = trajectories[0];

        drive.followTrajectory(toFinish);
    }

    /**
     * TODO(BSFishy): document this
     */
    private void doWait5() {
        OpmodeUtil.sleep(5, this);
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
     */
    private void detectRings() {
        if (tfod == null) {
            return;
        }

        ElapsedTime time = new ElapsedTime();
        String label = null;

        // Loop while the opmode is active but only for a maximum of 5 seconds
        while (opModeIsActive() && time.seconds() < RING_WAIT_TIME) {
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
        telemetry.addData("tensorflow time", "%.02f", time.seconds());
        telemetry.addData("recognition label", label);
        telemetry.update();
    }

    /**
     * TODO(BSFishy): document this
     */
    private enum ConfigurationStep {
        Alliance,
        Location,
        Steps,
        Finish;

        /**
         * TODO(BSFishy): document this
         *
         * @return the previous item
         */
        public ConfigurationStep previous() {
            return values()[Math.max(Math.min(ordinal() - 1, values().length - 1), 0)];
        }

        /**
         * TODO(BSFishy): document this
         *
         * @return the next item
         */
        public ConfigurationStep next() {
            return values()[Math.max(Math.min(ordinal() + 1, values().length - 1), 0)];
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private enum Alliance {
        Blue,
        Red;

        /**
         * TODO(BSFishy): document this
         *
         * @return the previous item
         */
        public Alliance previous() {
            return values()[(values().length + ordinal() - 1) % values().length];
        }

        /**
         * TODO(BSFishy): document this
         *
         * @return the next item
         */
        public Alliance next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private enum Location {
        Inner,
        Outer;

        /**
         * TODO(BSFishy): document this
         *
         * @return the previous item
         */
        public Location previous() {
            return values()[(values().length + ordinal() - 1) % values().length];
        }

        /**
         * TODO(BSFishy): document this
         *
         * @return the next item
         */
        public Location next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private enum Step {
        WobbleGoal,
        PowerShot,
        Park,
        Wait5;

        /**
         * TODO(BSFishy): document this
         *
         * @return the previous item
         */
        public Step previous() {
            return values()[(values().length + ordinal() - 1) % values().length];
        }

        /**
         * TODO(BSFishy): document this
         *
         * @return the next item
         */
        public Step next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    private enum RingCount {
        None,
        Single,
        Quad,
    }
}

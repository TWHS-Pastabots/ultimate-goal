package org.firstinspires.ftc.team15021.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team15021.PoseStorage;
import org.firstinspires.ftc.team15021.R;
import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.firstinspires.ftc.teamcode.util.AssetUtil;

import java.util.List;

@Autonomous(name="Synced Autonomous")
public class AutonSynced16910 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.65;

    private final int ON = 1;
    private final int OFF = 0;

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    private Pose2d wobbleGoalPosition;
    private double wobbleGoalTangent = 0;

    public static Pose2d WOBBLE_GOAL_1 = new Pose2d(0, 56);
    public static Pose2d WOBBLE_GOAL_2 = new Pose2d(24, 30);
    public static double WOBBLE_GOAL_2_TANGENT = Math.toRadians(90);
    public static Pose2d WOBBLE_GOAL_3_A = new Pose2d(24, 30);
    public static Pose2d WOBBLE_GOAL_3_B = new Pose2d(45, 48);

    private FtcDashboard dashboard;
    private RavioliHardware robot;
    private SampleMecanumDrive drive;

    private Trajectory toWobble, toWobbleB, toShoot, toPickup, adjustPickup, shootLast, toFinish;

    private int mode;

    @Override
    public void runOpMode() {


        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Don't automatically clear the telemetry
        telemetry.setAutoClear(false);

        // Initialize the robot hardware
        robot = new RavioliHardware();
        robot.init(hardwareMap);

        // Initialize the drive train and set start position
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-61.4633364016957, 27.462076691083404, Math.toRadians(0.06119830047682281)));

        // Activates TFOD
        activateTfod();

        // Detect rings using TFOD and initialize trajectories

        initTrajectories();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        detectRings();



        if (tfod != null) {
            tfod.shutdown();
        }


        // Determines which Autonomous to take
        if (mode == 1) AutonZero();
        else if (mode == 2) AutonOne();
        else if (mode == 3) AutonFour();
        else AutonZero();

        PoseStorage.currentPose = drive.getPoseEstimate();

        telemetry.addLine("Finished");
        telemetry.update();

    }

    private void initTrajectories() {

        // Trajectory to shoot
        toShoot = drive.trajectoryBuilder(new Pose2d(-12, 48))
                .splineToLinearHeading(new Pose2d(0, 30), 0)
                .build();

        // Trajectory to pick up rings
        toPickup = drive.trajectoryBuilder(toShoot.end())
                .splineToLinearHeading(new Pose2d(-6, 40), Math.toRadians(135))
                .build();

        // Trajectory to adjust the pickup location
        adjustPickup = drive.trajectoryBuilder(toPickup.end())
                .lineToLinearHeading(new Pose2d(-15, 36))
                .build();

        // Trajectory to shoot final shot
        shootLast = drive.trajectoryBuilder(adjustPickup.end())
                .lineToLinearHeading(new Pose2d(0, 33))
                .build();

        // Trajectory to park
        toFinish = drive.trajectoryBuilder(shootLast.end())
                .lineToLinearHeading(new Pose2d(11.532724155823113, 26.463119291587887))
                .build();

    }

    // Drop wobble goal
    public void dropWobble() {

        robot.motorClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorClaw.setPower(.1);

        telemetry.addLine("Setting motor claw power");
        telemetry.update();

        // Drop wobble goal
        robot.motorClaw.setTargetPosition(170);
        sleep(1000);

        telemetry.addLine("Moving servo claw");
        telemetry.update();

        // Open claw
        robot.servoClaw.setPosition(.5);
        sleep(1000);

        telemetry.addLine("Moving back motor claw");
        telemetry.update();

        robot.motorClaw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorClaw.setPower(.8);

        // Return claw to original place
        robot.motorClaw.setTargetPosition(30);

        telemetry.addLine("Finished dropping wobble");
        telemetry.update();



    }

    // Do all wobble goal maneuvers
    public void wobbleStuff() {

        telemetry.addLine("Moving to wobble");
        telemetry.update();

        drive.followTrajectory(toWobble);
        if (mode == 3) drive.followTrajectory(toWobbleB);

        telemetry.addLine("Dropping wobble");
        telemetry.update();

        dropWobble();

        telemetry.addLine("Dropped wobble");
        telemetry.update();

    }

    // Does the extra ring shit
    private void pickupRing() {

        // Drive to position
        drive.followTrajectory(toPickup);


        // Adjust the positioning to intake
        drive.followTrajectory(adjustPickup);

        // Drop intake
        robot.servoIntake.setPosition(.7);
        sleep(250);

        // Power conveyor belt and intake motor to take in 4th ring
        robot.motorConveyor.setPower(ON);
        robot.motorIntake.setPower(ON);

        // Ring is being taken up here
        sleep(2000);

        // Power conveyor off and reset servo
        robot.motorConveyor.setPower(OFF);
        robot.servoIntake.setPosition(OFF);
        robot.motorIntake.setPower(OFF);

    }

    // Shoots high goal shots
    private void shootShots() {
        // Synced
        // robot.motorLauncher.setPower(.675);
        drive.followTrajectory(toShoot);
        robot.motorLauncher.setPower(.8);
        sleep(3000);
        robot.motorConveyor.setPower(.4);
        sleep(4000);

    }

    // Shoots fourth shot
    private void shootFourth() {

        // Goes to shoot position
        drive.followTrajectory(shootLast);

        // Powers conveyor motor to shoot for 3 seconds
        robot.motorConveyor.setPower(ON);
        sleep(3000);
        robot.motorConveyor.setPower(OFF);

    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
        dashboard.startCameraStream(tfod, 10);
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = AssetUtil.loadVuforiaKey();
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
            mode = 1;
        } else if (label.equals(SINGLE_LABEL)) {
            wobbleGoalPosition = WOBBLE_GOAL_2;
            mode = 2;

            // Store the tangent for this position, because if we don't, the robot might drive
            // on top of the ring, which will mess up the encoders and put it in the wrong
            // position. This actually happened to us in a match :(
            wobbleGoalTangent = WOBBLE_GOAL_2_TANGENT;
        } else if (label.equals(QUAD_LABEL)) {
            wobbleGoalPosition = WOBBLE_GOAL_3_A;
            mode = 3;
        } else {
            wobbleGoalPosition = WOBBLE_GOAL_1;
            mode = 1;
        }

        Pose2d wobbleGoal = wobbleGoalPosition;
        if (wobbleGoal == null) {
            wobbleGoal = WOBBLE_GOAL_1;
        }

        if (mode == 3) wobbleGoalTangent = Math.toRadians(90);

        toWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                //        .splineToLinearHeading(wobbleGoal, wobbleGoalTangent)
                .splineToLinearHeading(wobbleGoal, wobbleGoalTangent)
                .build();

        if (mode == 3) {
            toWobbleB = drive.trajectoryBuilder(toWobble.end())
                    .lineToLinearHeading(WOBBLE_GOAL_3_B)
                    .build();
        }

        // Display some more telemetry information, which is, again, useful for debugging
        telemetry.addData("tensorflow time", time.seconds());
        telemetry.addData("recognition label", label);
        telemetry.addData("moving to", "(%.01f, %.01f)", wobbleGoalPosition.getX(), wobbleGoalPosition.getY());
        telemetry.update();
    }

    // Zero ring autonomous
    private void AutonZero() {



        // Does all wobble goal actions
        wobbleStuff();
        telemetry.addLine("Did wobble stuff");
        telemetry.update();

        // Shoots 3 rings into high goal
        shootShots();
        telemetry.addLine("Shot shots");
        telemetry.update();

        // Goes to finish
        drive.followTrajectory(toFinish);

        telemetry.addLine("Went to finish");
        telemetry.update();

    }

    // One ring autonomous
    private void AutonOne() {

        // All movements and doing things
        wobbleStuff();

        telemetry.addLine("Did wobble stuff");
        telemetry.update();

        // Shoots shots
        shootShots();

        telemetry.addLine("Shot shots");
        telemetry.update();

        pickupRing();

        telemetry.addLine("Picked up rings");
        telemetry.update();

        shootFourth();

        telemetry.addLine("Shot fourth");
        telemetry.update();

        drive.followTrajectory(toFinish);

        telemetry.addLine("Went to finish");
        telemetry.update();
    }

    // Four ring autonomous
    private void AutonFour() {

        // All movements and doing things
        wobbleStuff();

        telemetry.addLine("Did wobble stuff");
        telemetry.update();

        shootShots();

        telemetry.addLine("Shot shots");
        telemetry.update();

        drive.followTrajectory(toFinish);

        telemetry.addLine("Went to finish");
        telemetry.update();

    }
}

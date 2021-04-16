package org.firstinspires.ftc.team16911.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Arrays;
import java.util.Locale;

/**
 * Formal class for organizing all RobotHardware for usage across different Telop/Autonomous driving modes.
 * Contains references to HardwareMap and all motors, encoders and sensors used.
 */
@Config
public class MacaroniHardware extends RobotHardware {
    private DcMotorEx intakeMotor;
    public DcMotorEx beltMotor;
    private DcMotorEx launcherMotor;

    private ElapsedTime launcherElapsed;
    private double launcherWait = 0.0;

    private Servo leftClawServo;
    private Servo rightClawServo;
    public Servo armServo;
    private Servo ringStopperServo;

    private static final double LAUNCHER_STOPPER_THRESHOLD = 0.025;
    private static final double PRESPIN_FULL = 2;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap, false);

        intakeMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.INTAKE_MOTOR);
        launcherMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.LAUNCHER_MOTOR);
        beltMotor = hardwareMap.get(DcMotorEx.class, MacaroniIds.BELT_MOTOR);

        armServo = hardwareMap.get(Servo.class, MacaroniIds.ARM_SERVO);
        leftClawServo = hardwareMap.get(Servo.class, MacaroniIds.LEFT_CLAW_SERVO);
        rightClawServo = hardwareMap.get(Servo.class, MacaroniIds.RIGHT_CLAW_SERVO);
        ringStopperServo = hardwareMap.get(Servo.class, MacaroniIds.RING_STOPPER_SERVO);

        motors.addAll(Arrays.asList(intakeMotor, launcherMotor, beltMotor));
        servos.addAll(Arrays.asList(armServo, leftClawServo, rightClawServo, ringStopperServo));

        // Wheel directions
        motorRightFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Belt/intake motor directions
        beltMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // All setup.
        initializeComponents();

        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherElapsed = new ElapsedTime();

        // Move arms/claws/ring stopper into starting positions.
        armServo.setPosition(1);
        setClaws(true);
        setRingStopper(true);
    }

    /**
     * @param position 1.0 closes the claws, 0.0 opens them as wide as possible.
     */
    public void setClaws(double position) {
        leftClawServo.setPosition(1 - position);
        rightClawServo.setPosition(position);
    }

    /**
     * @param close If true, closes the claws. If false, opens the claws.
     */
    public void setClaws(boolean close) {
        setClaws(close ? 1.0 : 0.0);
    }

    /**
     * @param active If true, activates the ring stopper.
     */
    public void setRingStopper(boolean active) {
        ringStopperServo.setPosition(active ? 0 : 1);
    }

    /**
     * @return True if the ring stopper is up
     */
    public boolean ringStopperActive() {
        return ringStopperServo.getPosition() <= 0.10;
    }

    /**
     * Tick the current state of MacaroniHardware.
     *
     * Automatically controls the ring stopper in relation to the launcher.
     */
    public void tick() {
        // Set the ring stopper
//        if (launcherMotor.getPower() > 0) {
//            boolean stopperActive = ringStopperActive();
//            boolean elapsed = isLauncherReady();
//
//            if (stopperActive && elapsed)
//                // Launcher is ready to go, open ring stopper
//                setRingStopper(false);
//            else if (!stopperActive && !elapsed)
//                // Launcher is not ready, close ring stopper
//                setRingStopper(true);
//        } else
//            // Launcher is not spinning, close ring stopper
//            setRingStopper(true);
    }

    /**
     * A setter for the launcher motor power.
     * <p>
     * Setting the launcher power works with the ring stopper logic to automatically move the ring
     * stopper after a specific amount of time passes.
     *
     * @param power The launcher motor power.
     */
    public void setLauncherPower(double power) {
        // Change the power only if the value is new
        if (power != launcherMotor.getPower()) {
            // Reset the timer if the value is above the threshold
            if (power >= LAUNCHER_STOPPER_THRESHOLD)
                launcherElapsed.reset();
            launcherMotor.setPower(power);
        }
    }

    /**
     * @param belt   The motor power for the belt motor. [-1.0, 1.0]
     * @param intake The motor power for the intake motor. [-1.0, 1.0]
     */
    public void setIntakePower(double belt, double intake) {
        beltMotor.setPower(belt);
        intakeMotor.setPower(intake);
    }

    /**
     * @return True if the launcher has span the minimum amount of time for it's current power configuration.
     */
    private boolean isLauncherReady() {
        return launcherTimeLeft() <= 0;
    }

    /**
     * @return The amount of time the launcher must spin for before it's ready.
     */
    private double launcherTimeLeft() {
        return Math.max(0, (launcherMotor.getPower() * PRESPIN_FULL) - launcherElapsed.seconds());
    }

    /**
     * @return A string describing the current state of the prespin logic.
     */
    public String prespinTelemetry() {
        if (launcherMotor.getPower() <= LAUNCHER_STOPPER_THRESHOLD)
            return "Not spinning";

        double left = launcherTimeLeft();
        return left > 0 ? String.format(Locale.ENGLISH, "%.2fs", left) : "Elapsed";
    }

    /**
     * @return Returns the current velocity of the launcher motor.
     */
    public int launcherSpeed() {
        return (int) launcherMotor.getVelocity();
    }
}

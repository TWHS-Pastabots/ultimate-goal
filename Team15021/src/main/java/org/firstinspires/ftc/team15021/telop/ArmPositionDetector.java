package org.firstinspires.ftc.team15021.telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@TeleOp(name = "Arm Position Detector")
public class ArmPositionDetector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RavioliHardware robot = new RavioliHardware();
        robot.init(hardwareMap);

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        while (!isStopRequested()) {
            telemetry.addData("Arm position", robot.motorClaw.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}

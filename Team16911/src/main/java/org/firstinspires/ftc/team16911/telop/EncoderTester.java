package org.firstinspires.ftc.team16911.telop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team16911.drive.StandardTrackingWheelLocalizer;

import java.util.Locale;

/**
 * A simple OpMode used for diagnosing issues with encoders.
 */
@TeleOp(name = "Encoder Tester", group = "Linear OpMode")
public class EncoderTester extends OpMode {
    private StandardTrackingWheelLocalizer localizer;
    private final String[] encoderNames = new String[]{"Left", "Right", "Front"};


    @Override
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        int i = 0; // Encoder Index

        // Iterate through the velocity of each encoder wheel
        for (double velocity : localizer.getWheelVelocities()) {
            String caption = String.format("%s Encoder", encoderNames[i++]);
            String data;

            // Encoder is considered not moving under 5 ticks per second
            if (Math.abs(velocity) < 5)
                data = "Not moving.";
            else
                // Mark if it's moving forwards or backwards, and at what velocity
                data = String.format(Locale.ENGLISH, "Moving %s (%d)", velocity > 0 ? "Forward" : "Backwards", (int) velocity);
            telemetry.addData(caption, data);
        }
    }
}

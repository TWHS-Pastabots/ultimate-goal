package org.firstinspires.ftc.team16910.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.util.List;

@Config
public class TfodUtil {
    public enum Rings {
        None,
        Single,
        Quad,
    }

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";
    public static double MIN_CONFIDENCE = 0.65;

    private boolean active;

    private final VuforiaUtil vuforia;
    private TFObjectDetector tfod;

    public TfodUtil(VuforiaUtil vuforia) {
        Assert.assertNotNull(vuforia, "Vuforia must not be null");

        this.vuforia = vuforia;
    }

    public TFObjectDetector getTensorFlow() {
        return tfod;
    }

    public boolean isActive() {
        return active;
    }

    public void prepare(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia.getVuforia());
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD_LABEL, SINGLE_LABEL);
    }

    public void activate(HardwareMap hardwareMap) {
        if (!vuforia.isActive()) {
            vuforia.activate(hardwareMap);
        }

        prepare(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.25, 16.0/9.0);
        }

        active = true;
    }

    public Rings getRings() {
        Assert.assertNotNull(tfod, "TFOD must be activated before it is used");

        ElapsedTime time = new ElapsedTime();
        String label = null;

        // Loop while the opmode is active but only for a maximum of 5 seconds
        while (time.seconds() < 5) {
            // Get a list of recognitions from TFOD
            List<Recognition> updatedRecognitions = tfod.getRecognitions();

            // Make sure we are able to get recognitions
            if (updatedRecognitions != null) {
                // Check if we only have one recognition. This is to ensure that we
                // don't mistake a false-positive recognition with the actual rings
                if (updatedRecognitions.size() == 1) {
                    // Get the recognition and remember its label
                    Recognition recognition = updatedRecognitions.get(0);
                    label = recognition.getLabel();

                    // Break out of the loop because we've found the rings
                    break;
                }
            }
        }

        if (label == null) {
            return Rings.None;
        } else if (label.equals(SINGLE_LABEL)) {
            return Rings.Single;
        } else if (label.equals(QUAD_LABEL)) {
            return Rings.Quad;
        }

        return Rings.None;
    }

    public void deactivate() {
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }

        active = false;
    }
}

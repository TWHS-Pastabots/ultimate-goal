package org.firstinspires.ftc.team16910.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/**
 * TODO(BSFishy): document this
 */
public class OpmodeUtil {
    /**
     * TODO(BSFishy): document this
     *
     * @param maxTimeout the maximum number of seconds to wait for this task to finish
     * @param opMode the opmode that is calling this
     */
    public static void sleep(double maxTimeout, LinearOpMode opMode) {
        sleep(maxTimeout, opMode, () -> false);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param maxTimeout the maximum number of seconds to wait for this task to finish
     * @param opMode the opmode that is calling this
     * @param func the function to run
     */
    public static void sleep(double maxTimeout, LinearOpMode opMode, Supplier<Boolean> func) {
        ElapsedTime time = new ElapsedTime();
        while (opMode.opModeIsActive() && time.seconds() < maxTimeout) {
            if (func.get()) {
                break;
            }
        }
    }
}

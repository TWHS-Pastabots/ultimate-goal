package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * TODO(BSFishy): document this
 */
@Config
public class Position {
    public static double SAFE_SPOT_DISTANCE = 6;
    public static double POWER_SHOT_SHIFT_DISTANCE = 8;

//    public static Pose2d WOBBLE_GOAL_OFFSET = new Pose2d(14, 12.5);
    public static Pose2d WOBBLE_GOAL_OFFSET_1 = new Pose2d(17.75, 10.25);
    public static Pose2d WOBBLE_GOAL_OFFSET_2 = new Pose2d(-10.25, 17.75);

    public static double PARK_X = 12;

    /**
     * TODO(BSFishy): document this
     */
    public static class Blue {
        public static final Pose2d POWER_SHOT_SHIFT = new Pose2d(0, -POWER_SHOT_SHIFT_DISTANCE);

//        public static final Pose2d START_OUTER = new Pose2d(-61.5, 49.5, Math.toRadians(3.1601925531948454));
//        public static final Pose2d START_INNER = new Pose2d(-61.5, 26, Math.toRadians(3.1601925531948454));
        public static final Pose2d START_OUTER = new Pose2d(-63, 48.5);
        public static final Pose2d START_INNER = new Pose2d(-63, 24.5);

//        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-1.663571632019833, 55.97919956437621, Math.toRadians(4.16231851032346));
//        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(22.159610751087364, 34.05802073674887, Math.toRadians(4.199606918030278));
//        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(44.53042566915173, 60.45014818366927, Math.toRadians(2.5822222337244343));
        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(12, 60, Math.toRadians(90)).minus(WOBBLE_GOAL_OFFSET_2);
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(36, 36, Math.toRadians(90)).minus(WOBBLE_GOAL_OFFSET_2);
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(60, 60).minus(WOBBLE_GOAL_OFFSET_1);

//        public static final Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, 25.711349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_1 = new Pose2d(-1, 25.711349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, 48, 0);
        public static final Pose2d FINISH_INNER = new Pose2d(12, 24, 0);
    }

    /**
     * TODO(BSFishy): document this
     */
    public static class Red {
        public static final Pose2d POWER_SHOT_SHIFT = new Pose2d(0, POWER_SHOT_SHIFT_DISTANCE);
//        public static final Pose2d RED_WOBBLE_GOAL_OFFSET = new Pose2d(17.75, 10.25, Math.to)

        public static final Pose2d START_OUTER = new Pose2d(-63, -48.5);
        public static final Pose2d START_INNER = new Pose2d(-63, -24.5);

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-0.4925523115830537, -51.8292026834199, Math.toRadians(320.2705372352821));
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(26.165324630752682, -39.2811388956978, Math.toRadians(3.4154219758582727));
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(49.72420459432352, -51.093381241617216, Math.toRadians(313.63178148792804));

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-1, -27.561349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, -18, 0);
        public static final Pose2d FINISH_INNER = new Pose2d(12, -22, 0);
    }
}

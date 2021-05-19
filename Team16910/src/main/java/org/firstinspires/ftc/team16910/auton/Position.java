package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * TODO(BSFishy): document this
 */
@Config
public class Position {
    public static double SAFE_SPOT_DISTANCE = 12;
    public static double POWER_SHOT_SHIFT_DISTANCE = 8;

    /**
     * TODO(BSFishy): document this
     */
    public static class Blue {
        public static final Pose2d POWER_SHOT_SHIFT = new Pose2d(0, -POWER_SHOT_SHIFT_DISTANCE);

        public static final Pose2d START_OUTER = new Pose2d(-61.97443694686202, 48.63266549091789, Math.toRadians(0.6605960200263244));
        public static final Pose2d START_INNER = new Pose2d(-62.046867620290875, 24.869523594757442, Math.toRadians(0.0843314068120228));

        public static final Pose2d DETECT_RINGS = new Pose2d(-60, 36, Math.toRadians(0));

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-1.3509430442928019, 52.58991277925033, Math.toRadians(0.9463857875571164));
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(22.029464628634283, 30.249267482003386, Math.toRadians(1.15252922643069));
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(46.47365914671322, 54.198035555093604, Math.toRadians(1.8365506372438087));

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, 27.561349542087463, 0.08275128001363097);
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

        public static final Pose2d START_OUTER = new Pose2d(-59.13304881706739, -46.979932387077596, Math.toRadians(2.698605017976689));
        public static final Pose2d START_INNER = new Pose2d(-60.16300343609244, -22.71505126025306, Math.toRadians(1.0026067254300342));

        public static final Pose2d DETECT_RINGS = new Pose2d(-60, -36, Math.toRadians(0));

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-0.4925523115830537, -51.8292026834199, Math.toRadians(320.2705372352821));
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(26.165324630752682, -39.2811388956978, Math.toRadians(3.4154219758582727));
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(49.72420459432352, -51.093381241617216, Math.toRadians(313.63178148792804));

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, -27.561349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, -18, 0);
        public static final Pose2d FINISH_INNER = new Pose2d(12, -22, 0);
    }
}

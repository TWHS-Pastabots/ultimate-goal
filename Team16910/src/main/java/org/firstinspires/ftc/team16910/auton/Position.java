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

        public static final Pose2d START_OUTER = new Pose2d(-63.64800204729765, 46.39877819447857, Math.toRadians(3.5512892424129374));
        public static final Pose2d START_INNER = new Pose2d(-63.64800204729765, 22.39877819447857, Math.toRadians(3.5512892424129374));

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-1.7014121587073083, 55.303271023102646, Math.toRadians(358.8521558517273));
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(21.531418267740545, 30.705377032838875, Math.toRadians(3.0078201762876087));
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(44.935919028994086, 56.22700991949062, Math.toRadians(4.4929899518084815));

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, 27.561349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, 18, 0);
        public static final Pose2d FINISH_INNER = new Pose2d();
    }

    /**
     * TODO(BSFishy): document this
     */
    public static class Red {
        public static final Pose2d POWER_SHOT_SHIFT = new Pose2d(0, POWER_SHOT_SHIFT_DISTANCE);

        public static final Pose2d START_OUTER = new Pose2d(-63.64800204729765, -46.39877819447857, Math.toRadians(3.5512892424129374));
        public static final Pose2d START_INNER = new Pose2d(-63.64800204729765, -22.39877819447857, Math.toRadians(3.5512892424129374));

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(-1.7014121587073083, -55.303271023102646, Math.toRadians(358.8521558517273));
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(21.531418267740545, -30.705377032838875, Math.toRadians(3.0078201762876087));
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(44.935919028994086, -56.22700991949062, Math.toRadians(4.4929899518084815));

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-2.316608582048244, -27.561349542087463, 0.08275128001363097);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, -18, 0);
        public static final Pose2d FINISH_INNER = new Pose2d();
    }
}

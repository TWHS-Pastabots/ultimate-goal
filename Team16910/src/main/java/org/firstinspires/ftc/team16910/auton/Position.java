package org.firstinspires.ftc.team16910.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.team16910.util.Pose2dUtil;

/**
 * TODO(BSFishy): document this
 */
@Config
public class Position {
    public static double SAFE_SPOT_DISTANCE = 12;
    public static double POWER_SHOT_SHIFT_DISTANCE = 8;

    public static Pose2d WOBBLE_GOAL_OFFSET = new Pose2d(13, 8.25);
    public static Pose2d WOBBLE_GOAL_OFFSET_CW = Pose2dUtil.rotateAround(WOBBLE_GOAL_OFFSET, Math.toRadians(-90));
    public static Pose2d WOBBLE_GOAL_OFFSET_CCW = Pose2dUtil.rotateAround(WOBBLE_GOAL_OFFSET, Math.toRadians(90));

    public static double PARK_X = 12;

    /**
     * TODO(BSFishy): document this
     */
    public static class Blue {
        public static final Pose2d POWER_SHOT_SHIFT = new Pose2d(0, -POWER_SHOT_SHIFT_DISTANCE);

        public static final Pose2d START_OUTER = new Pose2d(-63, 48.5);
        public static final Pose2d START_INNER = new Pose2d(-63, 24.5);

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(12, 60).minus(WOBBLE_GOAL_OFFSET);
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(36, 36).minus(WOBBLE_GOAL_OFFSET);
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(60, 60).minus(WOBBLE_GOAL_OFFSET);

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-1, 25, 0);
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

        public static final Pose2d START_OUTER = new Pose2d(-63, -48.5);
        public static final Pose2d START_INNER = new Pose2d(-63, -24.5);

        public static final Pose2d WOBBLE_GOAL_A = new Pose2d(12, -60).minus(WOBBLE_GOAL_OFFSET_CW);
        public static final Pose2d WOBBLE_GOAL_B = new Pose2d(36, -36).minus(WOBBLE_GOAL_OFFSET);
        public static final Pose2d WOBBLE_GOAL_C = new Pose2d(60, -60).minus(WOBBLE_GOAL_OFFSET_CW);

        public static final Pose2d POWER_SHOT_1 = new Pose2d(-1, -7, 0);
        public static final Pose2d POWER_SHOT_2 = POWER_SHOT_1.plus(POWER_SHOT_SHIFT);
        public static final Pose2d POWER_SHOT_3 = POWER_SHOT_2.plus(POWER_SHOT_SHIFT);

        public static final Pose2d FINISH_OUTER = new Pose2d(12, -48, 0);
        public static final Pose2d FINISH_INNER = new Pose2d(12, -24, 0);
    }
}

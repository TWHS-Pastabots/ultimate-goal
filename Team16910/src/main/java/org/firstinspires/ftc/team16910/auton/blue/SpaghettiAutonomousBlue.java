package org.firstinspires.ftc.team16910.auton.blue;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.team16910.auton.AbstractSpaghettiAutonomous;
import org.firstinspires.ftc.team16910.util.Position;

import static org.firstinspires.ftc.team16910.auton.Position.Blue.POWER_SHOT_1;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.POWER_SHOT_2;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.POWER_SHOT_3;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.WOBBLE_GOAL_A;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.WOBBLE_GOAL_B;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.WOBBLE_GOAL_C;

/**
 * TODO(BSFishy): document this
 */
@Config
public abstract class SpaghettiAutonomousBlue extends AbstractSpaghettiAutonomous {
    public static final String GROUP = "Spaghetti Blue";

    /**
     * TODO(BSFishy): document this
     *
     * @return the starting position of the robot
     */
    public abstract Pose2d getStartPosition();

    /**
     * TODO(BSFishy): document this
     *
     * @return the ending position of the robot
     */
    public abstract Pose2d getEndPosition();

    @Override
    protected void prepare() {
        startPosition = getStartPosition();

        powerShot1Position = new Position(POWER_SHOT_1);
        powerShot2Position = new Position(POWER_SHOT_2);
        powerShot3Position = new Position(POWER_SHOT_3);

        finishPosition = new Position(getEndPosition());
    }

    @Override
    protected Position getWobbleGoalPosition() {
        switch (getRingCount()) {
            case Single:
                return new Position(WOBBLE_GOAL_B, Math.toRadians(-90));
            case Quad:
                return new Position(WOBBLE_GOAL_C, 0);
            case None:
            default:
                return new Position(WOBBLE_GOAL_A, 0);
        }
    }
}

package org.firstinspires.ftc.team16910.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Blue.FINISH_INNER;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.START_INNER;

/**
 * TODO(BSFishy): document this
 */
public class BlueInner {
    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Blue Inner Full", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiFull extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_INNER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_INNER;
        }

        @Override
        protected void prepare() {
            super.prepare();

            performWobbleGoal = true;
            performPowerShots = true;
            performPark = true;
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Blue Inner Park", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiPark extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_INNER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_INNER;
        }

        @Override
        protected void prepare() {
            super.prepare();

            performWobbleGoal = false;
            performPowerShots = false;
            performPark = true;
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Blue Inner Power Shot", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiPowerShot extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_INNER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_INNER;
        }

        @Override
        protected void prepare() {
            super.prepare();

            performWobbleGoal = false;
            performPowerShots = true;
            performPark = true;
        }
    }

    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Blue Inner Wobble Goal", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiWobbleGoal extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_INNER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_INNER;
        }

        @Override
        protected void prepare() {
            super.prepare();

            performWobbleGoal = true;
            performPowerShots = false;
            performPark = true;
        }
    }
}

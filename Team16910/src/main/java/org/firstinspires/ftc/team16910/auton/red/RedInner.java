package org.firstinspires.ftc.team16910.auton.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Red.FINISH_INNER;
import static org.firstinspires.ftc.team16910.auton.Position.Red.START_INNER;

/**
 * TODO(BSFishy): document this
 */
public class RedInner {
    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Red Inner Full", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiFull extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Inner Park", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiPark extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Inner Power Shot", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiPowerShot extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Inner Wobble Goal", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    public static class SpaghettiWobbleGoal extends SpaghettiAutonomousRed {
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

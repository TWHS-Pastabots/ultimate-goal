package org.firstinspires.ftc.team16910.auton.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Blue.FINISH_OUTER;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.START_OUTER;

/**
 * TODO(BSFishy): document this
 */
public class BlueOuter {
    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Blue Outer Full", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiFull extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_OUTER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_OUTER;
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
    @Autonomous(name = "Spaghetti Blue Outer Park", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiPark extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_OUTER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_OUTER;
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
    @Autonomous(name = "Spaghetti Blue Outer Power Shot", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiPowerShot extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_OUTER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_OUTER;
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
    @Autonomous(name = "Spaghetti Blue Outer Wobble Goal", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiWobbleGoal extends SpaghettiAutonomousBlue {
        @Override
        public Pose2d getStartPosition() {
            return START_OUTER;
        }

        @Override
        public Pose2d getEndPosition() {
            return FINISH_OUTER;
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

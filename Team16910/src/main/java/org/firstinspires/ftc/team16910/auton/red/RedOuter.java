package org.firstinspires.ftc.team16910.auton.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Red.FINISH_OUTER;
import static org.firstinspires.ftc.team16910.auton.Position.Red.START_OUTER;

/**
 * TODO(BSFishy): document this
 */
public class RedOuter {
    /**
     * TODO(BSFishy): document this
     */
    @Autonomous(name = "Spaghetti Red Outer Full", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiFull extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Outer Park", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiPark extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Outer Power Shot", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiPowerShot extends SpaghettiAutonomousRed {
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
    @Autonomous(name = "Spaghetti Red Outer Wobble Goal", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
    @Disabled
    public static class SpaghettiWobbleGoal extends SpaghettiAutonomousRed {
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

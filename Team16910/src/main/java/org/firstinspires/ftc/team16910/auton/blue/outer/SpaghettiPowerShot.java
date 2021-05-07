package org.firstinspires.ftc.team16910.auton.blue.outer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.auton.blue.SpaghettiAutonomousBlue;
import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Blue.FINISH_OUTER;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.START_OUTER;

/**
 * TODO(BSFishy): document this
 */
@Autonomous(name = "Spaghetti Blue Outer Power Shot", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiPowerShot extends SpaghettiAutonomousBlue {
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

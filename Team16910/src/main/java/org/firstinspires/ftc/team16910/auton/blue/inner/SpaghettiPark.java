package org.firstinspires.ftc.team16910.auton.blue.inner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.auton.blue.SpaghettiAutonomousBlue;
import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Blue.FINISH_INNER;
import static org.firstinspires.ftc.team16910.auton.Position.Blue.START_INNER;

/**
 * TODO(BSFishy): document this
 */
@Autonomous(name = "Spaghetti Blue Inner Park", group = SpaghettiAutonomousBlue.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiPark extends SpaghettiAutonomousBlue {
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

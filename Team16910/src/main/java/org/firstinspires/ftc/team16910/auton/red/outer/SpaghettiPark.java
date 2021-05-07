package org.firstinspires.ftc.team16910.auton.red.outer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.auton.red.SpaghettiAutonomousRed;
import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Red.FINISH_OUTER;
import static org.firstinspires.ftc.team16910.auton.Position.Red.START_OUTER;

/**
 * TODO(BSFishy): document this
 */
@Autonomous(name = "Spaghetti Red Outer Park", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiPark extends SpaghettiAutonomousRed {
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

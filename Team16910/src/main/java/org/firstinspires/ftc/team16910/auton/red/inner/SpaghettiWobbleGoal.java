package org.firstinspires.ftc.team16910.auton.red.inner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team16910.auton.red.SpaghettiAutonomousRed;
import org.firstinspires.ftc.team16910.telop.Spaghetti;

import static org.firstinspires.ftc.team16910.auton.Position.Red.FINISH_INNER;
import static org.firstinspires.ftc.team16910.auton.Position.Red.START_INNER;

/**
 * TODO(BSFishy): document this
 */
@Autonomous(name = "Spaghetti Red Inner Wobble Goal", group = SpaghettiAutonomousRed.GROUP, preselectTeleOp = Spaghetti.NAME)
public class SpaghettiWobbleGoal extends SpaghettiAutonomousRed {
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

package org.firstinspires.ftc.team16910.util;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Locale;

/**
 * TODO(BSFishy): document this
 *
 * @author Matt Provost &lt;mattprovost6@gmail.com&gt;
 */
public class Pose2dUtil {
    /**
     * TODO(BSFishy): document this
     *
     * @param input the previous non-rotated pose
     * @param angle the angle to rotate the pose by
     * @return the new rotated angle
     */
    public static Pose2d rotated(Pose2d input, double angle) {
        double newX = input.getX() * Math.cos(angle) - input.getY() * Math.sin(angle);
        double newY = input.getX() * Math.sin(angle) + input.getY() * Math.cos(angle);
        double newHeading = input.getHeading() + angle;

        return new Pose2d(newX, newY, newHeading);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param input the previous non-rotated pose
     * @param angle the angle to rotate the pose by
     * @return the new rotated angle
     */
    public static Pose2d rotateAround(Pose2d input, double angle) {
        double newX = input.getX() * Math.cos(angle) - input.getY() * Math.sin(angle);
        double newY = input.getX() * Math.sin(angle) + input.getY() * Math.cos(angle);
        double newHeading = input.getHeading() - angle;

        return new Pose2d(newX, newY, newHeading);
    }
}

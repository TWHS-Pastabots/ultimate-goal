package org.firstinspires.ftc.team16910.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

/**
 * TODO(BSFishy): document this
 */
@Config
public class Position {
    public static double DEFAULT_TANGENT = 0;

    private Pose2d pose;
    private double tangent;

    /**
     * TODO(BSFishy): document this
     */
    public Position() {
        this(new Pose2d(), DEFAULT_TANGENT);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param pose the pose to use
     */
    public Position(Pose2d pose) {
        this(pose, DEFAULT_TANGENT);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param vector the vector to use as a pose
     */
    public Position(Vector2d vector) {
        this(new Pose2d(vector, 0), DEFAULT_TANGENT);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param tangent the tangent to use
     */
    public Position(double tangent) {
        this(new Pose2d(), tangent);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param pose    the pose to use
     * @param tangent the tangent to use
     */
    public Position(Pose2d pose, double tangent) {
        this.pose = pose;
        this.tangent = tangent;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current pose
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param pose the new pose
     */
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @return the current tangent
     */
    public double getTangent() {
        return tangent;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param tangent the new tangent
     */
    public void setTangent(double tangent) {
        this.tangent = tangent;
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param degrees the new tangent in degrees
     */
    public void setTangentDegrees(double degrees) {
        this.tangent = Math.toRadians(degrees);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param builder the trajectory builder to add on to
     * @return the new trajectory builder
     */
    public TrajectoryBuilder toSpline(TrajectoryBuilder builder) {
        return builder.splineToLinearHeading(pose, tangent);
    }

    /**
     * TODO(BSFishy): document this
     *
     * @param builder the trajectory builder to add on to
     * @return the new trajectory builder
     */
    public TrajectoryBuilder toLine(TrajectoryBuilder builder) {
        return builder.lineToLinearHeading(pose);
    }
}

package frc.robot.utils;

/*
 * From https://github.com/Bankst
 *
 * Messages in FRC Discord:
 * https://discord.com/channels/176186766946992128/368993897495527424/1085237369994739823
 * https://discord.com/channels/176186766946992128/368993897495527424/1085237999345872926
 * https://discord.com/channels/176186766946992128/368993897495527424/1085238166207873074
 */

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.AutoConstants;

import java.util.ArrayList;
import java.util.List;

public class PathFlipper {

    public static boolean shouldFlip() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Red;
    }

    public static PathPlannerTrajectory flipPath(PathPlannerTrajectory trajectory) {
        List<Trajectory.State> newStates = new ArrayList<>();

        for (Trajectory.State s : trajectory.getStates()) {
            PathPlannerTrajectory.PathPlannerState state = (PathPlannerTrajectory.PathPlannerState) s;

            newStates.add(flipPath(state));
        }

        return new PathPlannerTrajectory(
                newStates,
                trajectory.getMarkers(),
                trajectory.getStartStopEvent(),
                trajectory.getEndStopEvent(),
                trajectory.fromGUI);
    }

    public static PathPlannerTrajectory.PathPlannerState flipPath(PathPlannerTrajectory.PathPlannerState state) {
        PathPlannerTrajectory.PathPlannerState newState = new PathPlannerTrajectory.PathPlannerState();
        newState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
        newState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
        newState.curvatureRadPerMeter = -state.curvatureRadPerMeter;
        newState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
        newState.holonomicRotation = Rotation2d.fromRadians(Math.PI - state.holonomicRotation.getRadians());
        newState.poseMeters = new Pose2d(AutoConstants.kFieldLengthMeters - state.poseMeters.getX(),
                state.poseMeters.getY(),
                Rotation2d.fromRadians(Math.PI - state.poseMeters.getRotation().getRadians()));
        newState.timeSeconds = state.timeSeconds;
        newState.velocityMetersPerSecond = state.velocityMetersPerSecond;
        return newState;
    }

    public static CommandBase getFullAuto(SwerveAutoBuilder autoBuilder, PathPlannerTrajectory trajectory) {
        boolean shouldFlip = shouldFlip();
        PathPlannerTrajectory newTraj = trajectory;

        if (shouldFlip) {
            newTraj = flipPath(trajectory);
        }

        return autoBuilder.fullAuto(newTraj);
    }

}

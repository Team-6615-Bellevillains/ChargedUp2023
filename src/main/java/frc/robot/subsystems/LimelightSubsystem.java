// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LimelightConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new Limelight. */
    private PhotonCamera camera;
    private PhotonTrackedTarget bestTarget;
    private PhotonPoseEstimator photonPoseEstimator;

    public LimelightSubsystem() {
        camera = new PhotonCamera(LimelightConstants.kCameraName);

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are
            // on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator = new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, LimelightConstants.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
            // we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
     *         targets used to create
     *         the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public double calculateRange(double cameraHeight, double targetHeight, double cameraPitch) {
        return PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, cameraPitch,
                Units.degreesToRadians(getBestTarget().getYaw()));
    }

    @Override
    public void periodic() {
        // Gets latest pipeline frm the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // Check if the camera sees any targets, if it loses track, the subsystem will
        // expose a null target and tracking commands should halt.
        if (result.hasTargets()) {

            // Expose the target with the best tracking information available for use with
            // autonomous commands.
            this.bestTarget = result.getBestTarget();
            // Transform3d cameTransform = this.bestTarget.getBestCameraToTarget();
            // SmartDashboard.putNumber("Current AprilTag Target",
            // this.bestTarget.getFiducialId());
            // SmartDashboard.putBoolean("Has Targets", result.hasTargets());
            // SmartDashboard.putNumber("Y-distance to april tag", cameTransform.getY());
        } else {
            // Remove tracking for target if we can't see it anymore
            this.bestTarget = null;
        }

    }
}

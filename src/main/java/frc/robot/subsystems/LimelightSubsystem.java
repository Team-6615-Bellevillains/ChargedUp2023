// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  private PhotonCamera camera;
  private PhotonTrackedTarget target;

  public LimelightSubsystem() {
    camera = new PhotonCamera("asdoija");
  }

  public PhotonTrackedTarget getBestTarget() {
    return target;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Gets latest pipeline frm the camera
    var result = camera.getLatestResult();
    SmartDashboard.putNumber("PhotonTS", Timer.getFPGATimestamp());

    // check to see if there is a target
    if (result.hasTargets()) {

      // Get the best target
      this.target = result.getBestTarget();

      // Get Target info
      double yaw = target.getYaw();
      SmartDashboard.putNumber("Yaw", yaw);
      double pitch = target.getPitch();
      Transform3d camToTarget = target.getBestCameraToTarget();

    }

  }
}

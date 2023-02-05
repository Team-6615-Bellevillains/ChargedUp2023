// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  private PhotonCamera camera;
  private PhotonTrackedTarget bestTarget;

  public LimelightSubsystem() {
    camera = new PhotonCamera("asdoija");
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
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

    } else {
      // Remove tracking for target if we can't see it anymore
      this.bestTarget = null;
    }

  }
}

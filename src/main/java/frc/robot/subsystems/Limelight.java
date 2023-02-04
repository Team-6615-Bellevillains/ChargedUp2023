// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private PhotonCamera camera;

  public Limelight() 
  {
    
    camera = new PhotonCamera("mrcrabs6615");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //Gets latest pipeline frm the camera
    var result = camera.getLatestResult();
    
    //check to see if there is a target
    if (result.hasTargets())
    {
      
      //Get the best target
      var target = result.getBestTarget();

      //Get Target info
      var yaw = target.getYaw();
      var pitch = target.getPitch();
      var camToTarget = target.getBestCameraToTarget();

    }

  }
}

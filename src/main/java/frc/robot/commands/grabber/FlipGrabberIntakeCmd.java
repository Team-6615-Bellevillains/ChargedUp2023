// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.GrabberSubsystem;

public class FlipGrabberIntakeCmd extends CommandBase {
  private GrabberSubsystem grabberSubsystem;
  private PIDController flipPIDController;
  private boolean upDirection;

  public FlipGrabberIntakeCmd(GrabberSubsystem grabberSubsystem, boolean upDirection) {
    this.grabberSubsystem = grabberSubsystem;

    flipPIDController = new PIDController(GrabberConstants.kPFlip, GrabberConstants.kIFlip, GrabberConstants.kDFlip);

    this.upDirection = upDirection; //True will raise the grabber up
    


    addRequirements(grabberSubsystem);
  }

  @Override
  public void initialize() {
    if(upDirection == true)
    {
      flipPIDController.setSetpoint(GrabberConstants.grabberInSetpoint);
    }
    else
    {
      flipPIDController.setSetpoint(GrabberConstants.grabberOutSetpoint);
    }
    
    flipPIDController.reset();
  }

  @Override
  public void execute() {
    //Need to test which direction
    double speed = flipPIDController.calculate(grabberSubsystem.getFlipEncoderPosition());
    grabberSubsystem.setFlipMotorSpeed(speed);

  }

  @Override
  public void end(boolean interrupted) {
    grabberSubsystem.setFlipMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return flipPIDController.atSetpoint();
  }
}

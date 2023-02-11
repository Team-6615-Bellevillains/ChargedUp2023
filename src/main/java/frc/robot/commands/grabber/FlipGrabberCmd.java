// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.GrabberSubsystem;

public class FlipGrabberCmd extends CommandBase {
  /** Creates a new setFlipPosition. */
  private GrabberSubsystem grabberSubsystem;
  private PIDController flipPIDController;

  public FlipGrabberCmd(GrabberSubsystem grabberSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabberSubsystem = grabberSubsystem;

    flipPIDController = new PIDController(GrabberConstants.kPFlip, GrabberConstants.kIFlip, GrabberConstants.kDFlip);
    flipPIDController.setSetpoint(setpoint);

    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PID Flipper Works!");
    flipPIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = flipPIDController.calculate(grabberSubsystem.getFlipEncoderPosition());
    grabberSubsystem.setFlipMotorSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Flip finished!");
    grabberSubsystem.setFlipMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flipPIDController.atSetpoint();
  }
}

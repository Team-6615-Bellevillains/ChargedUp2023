// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class vElevatorSetHeight extends CommandBase {
  /** Creates a new vElevatorSetHeight. */
  private final PIDController vPIDController; // v = vertical
  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;
  public vElevatorSetHeight(ElevatorSubsystem elevatorSubsystem, double setpoint) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    this.vPIDController = new PIDController(ElevatorConstants.kPVertical, ElevatorConstants.kIVertical, ElevatorConstants.kDVertical);
    this.vPIDController.setSetpoint(setpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("vPIDController has been reset");
    vPIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double speed = vPIDController.calculate(elevatorSubsystem.vElevatorEncoderPosition());
    elevatorSubsystem.vElevatorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevatorSubsystem.vElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vPIDController.atSetpoint();
  }
}

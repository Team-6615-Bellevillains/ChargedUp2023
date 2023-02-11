// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class hElevatorSetLength extends CommandBase {
  /** Creates a new hElevatorSetHeight. */
  private final PIDController hPIDController; // v = vertical
  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;
  public hElevatorSetLength(ElevatorSubsystem elevatorSubsystem, double setpoint) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.setpoint = setpoint;
    this.hPIDController = new PIDController(ElevatorConstants.kPHorizontal, ElevatorConstants.kIHorizontal, ElevatorConstants.kDHorizontal);
    this.hPIDController.setSetpoint(setpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("hPIDController has been reset");
    hPIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double speed = hPIDController.calculate(elevatorSubsystem.hElevatorEncoderPosition());
    elevatorSubsystem.hElevatorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevatorSubsystem.hElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hPIDController.atSetpoint();
  }
}

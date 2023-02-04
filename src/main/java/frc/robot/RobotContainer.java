// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ArmConstants;

public class RobotContainer {

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.xboxControllerPort);
  private final CommandXboxController xboxController2 = new CommandXboxController(OIConstants.xboxControllerPort2);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -xboxController.getLeftY(),
        () -> -xboxController.getLeftX(),
        () -> -xboxController.getRightX(),
        () -> xboxController.leftBumper().getAsBoolean()));

    configureBindings();
  }


  private void configureBindings() {
    xboxController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    xboxController2.a().whenPressed( new ArmCommand(armSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

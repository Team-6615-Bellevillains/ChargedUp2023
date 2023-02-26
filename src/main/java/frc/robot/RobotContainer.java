// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCmd;
import frc.robot.commands.AlignToMidRungCmd;
import frc.robot.commands.drive.StraightenRobotCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  //private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
 // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();


  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.xboxControllerPort);

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
    xboxController.rightBumper().whileTrue(new StraightenRobotCmd(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return new AlignToMidRungCmd(limelight, swerveSubsystem);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grabber.ShootPieceCmd;
import frc.robot.commands.grabber.SuckObjectCmd;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final HorizontalElevatorSubsystem horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
  private final VerticalElevatorSubsystem verticalElevatorSubsystem = new VerticalElevatorSubsystem();


  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.leftBumper().getAsBoolean()));

    horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));
    verticalElevatorSubsystem.setDefaultCommand(new ManualVerticalElevatorController(verticalElevatorSubsystem, () -> -operatorController.getRightY())); // TODO: Test

    configureBindings();
  }

  private void configureBindings() {
    driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    operatorController.leftBumper().whileTrue(new SuckObjectCmd(grabberSubsystem));
    operatorController.rightBumper().whileTrue(new ShootPieceCmd(grabberSubsystem));

    operatorController.y().whileTrue(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem));
    operatorController.x().whileTrue(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));

    operatorController.rightTrigger(0.1).whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalLowHeight)); // TODO: Test
    operatorController.a().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalMidHeight));
    operatorController.b().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight));
    operatorController.leftTrigger(0.1).whileTrue(new InstantCommand(verticalElevatorSubsystem::lowerElevator));
  }

  public Command getAutonomousCommand() {
    return new AlignToAprilTagCubeCmd(limelight, swerveSubsystem);
  }
}

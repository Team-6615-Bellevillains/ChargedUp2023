// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.drive.StraightenRobotCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.elevator.ManualHorizontalElevatorController;
import frc.robot.commands.elevator.ManualVerticalElevatorController;
import frc.robot.commands.grabber.ManualGrabberFlipInCmd;
import frc.robot.commands.grabber.ManualGrabberFlipOutCmd;
import frc.robot.commands.grabber.ShootPieceCmd;
import frc.robot.commands.grabber.SuckObjectCmd;
import frc.robot.commands.operation.ScoreCubeLowCmd;
import frc.robot.commands.operation.ScoreCubeMidCmd;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final HorizontalElevatorSubsystem horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
  private final VerticalElevatorSubsystem verticalElevatorSubsystem = new VerticalElevatorSubsystem();
  private SendableChooser<Command> m_chooser;

  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.leftBumper().getAsBoolean()));

    horizontalElevatorSubsystem.setDefaultCommand(new ManualHorizontalElevatorController(horizontalElevatorSubsystem, () -> -operatorController.getLeftY()));
    verticalElevatorSubsystem.setDefaultCommand(new ManualVerticalElevatorController(verticalElevatorSubsystem, () -> -operatorController.getRightY()));

    //Adds a smartdashboard widget that will allow us to select the autonomous we want to use. 
    m_chooser = new SendableChooser<>();
    //Default Autonomous that will be run if no other auto is selected
    m_chooser.setDefaultOption("AlignToAprilTagCubeCmd", new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem));
    m_chooser.addOption("ScoreCubeLowCmd", new ScoreCubeLowCmd(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem)); 
    SmartDashboard.putData(m_chooser); 

    configureBindings();
  }

  private void configureBindings() {
    driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    driverController.rightBumper().whileTrue(new StraightenRobotCmd(swerveSubsystem));

    operatorController.a().whileTrue(new ManualGrabberFlipOutCmd(grabberSubsystem));
    operatorController.b().whileTrue(new ManualGrabberFlipInCmd(grabberSubsystem));
    operatorController.leftBumper().whileTrue(new SuckObjectCmd(grabberSubsystem));
    operatorController.rightBumper().whileTrue(new ShootPieceCmd(grabberSubsystem));
    operatorController.y().whileTrue(new ScoreCubeMidCmd(horizontalElevatorSubsystem, verticalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

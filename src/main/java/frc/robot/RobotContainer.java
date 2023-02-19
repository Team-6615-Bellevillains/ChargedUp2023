// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.StraightenRobotCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  private final Joystick xboxController = new Joystick(OIConstants.xboxControllerPort);
  private final JoystickButton leftBumper = new JoystickButton(xboxController, OIConstants.leftBumper);
  private final JoystickButton rightBumper = new JoystickButton(xboxController, OIConstants.rightBumper);
  private final JoystickButton yButton = new JoystickButton(xboxController, OIConstants.yButton);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -xboxController.getRawAxis(OIConstants.kLeftYAxis),
        () -> -xboxController.getRawAxis(OIConstants.kLeftXAxis),
        () -> -xboxController.getRawAxis(OIConstants.kRightXAxis),
        () -> leftBumper.getAsBoolean()));

    configureBindings();
  }


  private void configureBindings() {
    yButton.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    rightBumper.whileTrue(new StraightenRobotCmd(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return new AlignToAprilTag(limelight, swerveSubsystem);
  }
}

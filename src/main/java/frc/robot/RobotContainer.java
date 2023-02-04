// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.StraightenRobotCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick ps4Controller = new Joystick(OIConstants.ps4ControllerPort);
  private final JoystickButton leftBumper = new JoystickButton(ps4Controller, OIConstants.leftBumper);
  private final JoystickButton rightBumper = new JoystickButton(ps4Controller, OIConstants.rightBumper);
  private final JoystickButton triangleButton = new JoystickButton(ps4Controller, OIConstants.triangle);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -ps4Controller.getRawAxis(OIConstants.kLeftYAxis),
        () -> -ps4Controller.getRawAxis(OIConstants.kLeftXAxis),
        () -> -ps4Controller.getRawAxis(OIConstants.kRightXAxis),
        () -> leftBumper.getAsBoolean()));

    configureBindings();
  }

  private void configureBindings() {
    triangleButton.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    rightBumper.whileTrue(new StraightenRobotCmd(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

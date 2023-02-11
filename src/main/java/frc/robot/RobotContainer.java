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
import frc.robot.commands.grabber.ClampGrabberCmd;
import frc.robot.commands.grabber.EjectObjectCmd;
import frc.robot.commands.grabber.OpenGrabberCmd;
import frc.robot.commands.grabber.SuckObjectCmd;
import frc.robot.commands.grabber.FlipGrabberCmd;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  private final Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final JoystickButton driverLeftBumper = new JoystickButton(driverController, OIConstants.leftBumper);
  private final JoystickButton driverRightBumper = new JoystickButton(driverController, OIConstants.rightBumper);
  private final JoystickButton driverTriangleButton = new JoystickButton(driverController, OIConstants.triangle);

  private final Joystick operatorController = new Joystick(OIConstants.kOperatorControllerPort);
  private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, OIConstants.leftBumper);
  private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, OIConstants.rightBumper);
  private final JoystickButton operatorShareButton = new JoystickButton(operatorController, OIConstants.shareButton);
  private final JoystickButton operatorOptionsButton = new JoystickButton(operatorController,
      OIConstants.optionsButton);

  private final SuckObjectCmd suckObjectCmd = new SuckObjectCmd(grabberSubsystem);
  private final EjectObjectCmd ejectObjectCmd = new EjectObjectCmd(grabberSubsystem);
  private final OpenGrabberCmd openGrabberCmd = new OpenGrabberCmd(grabberSubsystem);
  private final ClampGrabberCmd clampGrabberCmd = new ClampGrabberCmd(grabberSubsystem);
  private final FlipGrabberCmd flipGrabberCmd = new FlipGrabberCmd(grabberSubsystem, 2009);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getRawAxis(OIConstants.kLeftYAxis),
        () -> -driverController.getRawAxis(OIConstants.kLeftXAxis),
        () -> -driverController.getRawAxis(OIConstants.kRightXAxis),
        () -> driverLeftBumper.getAsBoolean()));

    configureBindings();
  }

  private void configureBindings() {

    driverTriangleButton.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    driverRightBumper.whileTrue(new StraightenRobotCmd(swerveSubsystem));

    operatorLeftBumper.onTrue(suckObjectCmd);
    operatorRightBumper.onTrue(ejectObjectCmd);
    operatorShareButton.onTrue(openGrabberCmd);
    operatorOptionsButton.onTrue(clampGrabberCmd);

    // Simulatator Cmd Sends
    SmartDashboard.putData("Open Grabber", openGrabberCmd);
    SmartDashboard.putData("Clamp Grabber", clampGrabberCmd);
    SmartDashboard.putData("Flip Grabber", flipGrabberCmd);

  }

  public Command getAutonomousCommand() {
    return new AlignToAprilTag(limelight, swerveSubsystem);
  }
}

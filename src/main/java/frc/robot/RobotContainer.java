// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.StraightenRobotCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();

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

        SmartDashboard.putData("Swerve Joystick Cmd", new SwerveJoystickCmd(
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
        //1. Sets the configuration of the trajectory config with the max speed and acceleration the robot could go. Then sets the configuration to the kinematics of swerve drive.
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kAutoMaxSpeedMetersPerSecond, 
        Constants.AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.DriveConstants.kDriveKinematics);

         // 2. Generate trajectory
         Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(0, 5),
                  new Translation2d(0, 8),
                  new Translation2d(5,10)),
          new Pose2d(0, 10, new Rotation2d(170)),
          trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
 
        

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));

        
              
  }
}
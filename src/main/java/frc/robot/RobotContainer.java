// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grabber.*;
import frc.robot.commands.operation.ScoreHighCmd;
//import frc.robot.commands.operation.ScoreCubeLowCmd;
//import frc.robot.commands.operation.ScoreCubeMidCmd;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

  private final HorizontalElevatorSubsystem horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
  private final VerticalElevatorSubsystem verticalElevatorSubsystem = new VerticalElevatorSubsystem();
  private SendableChooser<Command> m_chooser;

  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  
  private SwerveAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap = new HashMap<>();
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.leftBumper().getAsBoolean()));

    horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));
    verticalElevatorSubsystem.setDefaultCommand(new ManualVerticalElevatorController(verticalElevatorSubsystem, () -> -operatorController.getRightY())); // TODO: Test
    grabberSubsystem.setDefaultCommand(new GrabberJoystickControlCmd(grabberSubsystem, () -> -operatorController.getLeftY()));


    PathPlannerTrajectory testPath = PathPlanner.loadPath("New Path", new PathConstraints(0.35, 1));

    eventMap.put("marker1", new PrintCommand("Passed Marker 1"));

    autoBuilder = new SwerveAutoBuilder(
    swerveSubsystem::getPose, // Pose2d supplier
    swerveSubsystem::resetPoseEstimator, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(Constants.AutoConstants.kPTrackingDriveX, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    (SwerveModuleState[] desiredStates) -> swerveSubsystem.setModuleStates(desiredStates, true), // Module states consumer used to output to the drive subsytem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
);
    

    //Adds a smartdashboard widget that will allow us to select the autonomous we want to use. 
    m_chooser = new SendableChooser<>();
    //Default Autonomous that will be run if no other auto is selected
    m_chooser.setDefaultOption("AlignToAprilTagCubeCmd", new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem));
    m_chooser.addOption("ScoreHighCmd", new ScoreHighCmd(horizontalElevatorSubsystem, grabberSubsystem, verticalElevatorSubsystem, rollerSubsystem));
    //m_chooser.addOption("ScoreCubeLowCmd", new ScoreCubeLowCmd(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem)); 
    m_chooser.addOption("Path Tester", autoBuilder.fullAuto(testPath));
    SmartDashboard.putData(m_chooser); 

    configureBindings();
  }

  private void configureBindings() {
    driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    operatorController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
    operatorController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
    operatorController.leftTrigger(0.1).whileTrue(new SuckObjectCmd(rollerSubsystem));
    operatorController.rightTrigger(0.1).whileTrue(new ShootPieceCmd(rollerSubsystem));

    operatorController.start().whileTrue(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem));
    operatorController.back().whileTrue(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

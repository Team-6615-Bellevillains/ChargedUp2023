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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.elevator.*;
import frc.robot.commands.grabber.*;
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
  private final Joystick buttonBox = new Joystick(OIConstants.kButtonBoxPort);
  private final JoystickButton vertHighButton = new JoystickButton(buttonBox, 1);
  private final JoystickButton vertMidButton = new JoystickButton(buttonBox, 2);
  private final JoystickButton vertLowButton = new JoystickButton(buttonBox, 3);

  private final JoystickButton scoreCubeHighButton = new JoystickButton(buttonBox, 8);
  
  private SwerveAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap = new HashMap<>();

  private Command generateScoreHighCmd() {
    return (new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight))
            .andThen(Commands.runOnce(horizontalElevatorSubsystem::removeDefaultCommand))
            .andThen(Commands.parallel(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem), new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeSetpoint)))
            .andThen(new AutoShootPieceCmd(rollerSubsystem))
            .andThen(Commands.parallel(new HorizontalElevatorInCmd(horizontalElevatorSubsystem), new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint)))
            .andThen(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, Units.inchesToMeters(.5)))
            .andThen(Commands.runOnce(() -> horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem))))
            .andThen(Commands.runOnce(() -> verticalElevatorSubsystem.setVerticalElevatorVoltage(0)));
  }

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.leftBumper().getAsBoolean()));

    setMechanismDefaultCommands();

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


    Command alignToApriltagCubeCmd = new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem);
    Command scoreHighCmd = generateScoreHighCmd();

    //Adds a smartdashboard widget that will allow us to select the autonomous we want to use. 
    m_chooser = new SendableChooser<>();
    //Default Autonomous that will be run if no other auto is selected
    m_chooser.setDefaultOption("AlignToAprilTagCubeCmd",alignToApriltagCubeCmd);
    m_chooser.addOption("ScoreHighCmd", scoreHighCmd);
//    m_chooser.addOption("AlignAndScoreHigh", alignToApriltagCubeCmd.andThen(scoreHighCmd));

    //m_chooser.addOption("ScoreCubeLowCmd", new ScoreCubeLowCmd(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem)); 
    m_chooser.addOption("Path Tester", autoBuilder.fullAuto(testPath));
    SmartDashboard.putData(m_chooser);

    configureBindings();
  }

  public void setMechanismDefaultCommands() {
    horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));
    verticalElevatorSubsystem.setDefaultCommand(new ManualVerticalElevatorController(verticalElevatorSubsystem, () -> -operatorController.getRightY())); // TODO: Test
    grabberSubsystem.setDefaultCommand(new GrabberJoystickControlCmd(grabberSubsystem, () -> -operatorController.getLeftY()));
  }

  public void removeMechanismDefaultCommands() {
    horizontalElevatorSubsystem.removeDefaultCommand();
    verticalElevatorSubsystem.removeDefaultCommand();
    grabberSubsystem.removeDefaultCommand();
  }

  private void configureBindings() {
    driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    driverController.start().onTrue(new InstantCommand(horizontalElevatorSubsystem::resetHorizontalElevatorEncoder));

    driverController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
    driverController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
    driverController.leftTrigger(0.4).whileTrue(new SuckObjectCmd(rollerSubsystem));
    driverController.rightTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem));

    operatorController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
    operatorController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
    operatorController.leftTrigger(0.4).whileTrue(new SuckObjectCmd(rollerSubsystem));
    operatorController.rightTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem));

    operatorController.start().whileTrue(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem));
    operatorController.back().whileTrue(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));

    vertHighButton.whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight));
    scoreCubeHighButton.whileTrue(generateScoreHighCmd());

//    operatorController.a().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight));
//    operatorController.x().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, Units.inchesToMeters(2)));
//    operatorController.b().whileTrue(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeSetpoint));
//    operatorController.y().whileTrue(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint));

  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

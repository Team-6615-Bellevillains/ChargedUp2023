// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.AlignToDoubleSubstation;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.CrossWheelsCmd;
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
  private final JoystickButton doubleSubstationElevatorButton = new JoystickButton(buttonBox, 6);


  private final JoystickButton scoreCubeMidButton = new JoystickButton(buttonBox, 7);
  private final JoystickButton setDefaultCommandsButton = new JoystickButton(buttonBox, 5); 
  private final JoystickButton forceInButton = new JoystickButton(buttonBox, 4); 

  private SwerveAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap = new HashMap<>();

  private Command generateScoreHighCmd() {
    return Commands.runOnce(() -> SmartDashboard.putString("Score High", "running"))
            .andThen(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 1)))
            .andThen(Commands.runOnce(horizontalElevatorSubsystem::removeDefaultCommand))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 2)))
            .andThen(Commands.parallel(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem), new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeHighSetpoint)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 3)))
            .andThen(new AutoShootPieceCmd(rollerSubsystem))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 4)))
            .andThen(Commands.parallel(new HorizontalElevatorInCmd(horizontalElevatorSubsystem), new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 5)))
            .andThen(Commands.runOnce(() -> grabberSubsystem.setMotorVoltage(0), grabberSubsystem))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 6)))
            .andThen(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalLowHeight))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 7)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 8)))
            .andThen(Commands.runOnce(() -> verticalElevatorSubsystem.setVerticalElevatorVoltage(0), verticalElevatorSubsystem).andThen(Commands.runOnce(verticalElevatorSubsystem::resetVerticalElevatorEncoder, verticalElevatorSubsystem)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 9)))
            .andThen(Commands.runOnce(() -> horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem))))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 10)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putString("Score High", "done")))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 0)));
  }

  private Command generateScoreCubeMidCmd() {
    return Commands.runOnce(() -> SmartDashboard.putString("Score Cube Mid", "running"))
            .andThen(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeMidSetpoint))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 1)))
            .andThen(new AutoShootPieceCmd(rollerSubsystem))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 2)))
            .andThen(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 3)))
            .andThen(Commands.runOnce(() -> grabberSubsystem.setMotorVoltage(0), grabberSubsystem))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 4)))
            .andThen(Commands.runOnce(() -> SmartDashboard.putString("Score Cube Mid", "done")))
            .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 0)));
  }

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> true));

    setMechanismDefaultCommands();

    PathPlannerTrajectory testPath = PathPlanner.loadPath("Inside Cube ID8 Copy Copy", new PathConstraints(0.75, 1));
    PathPlannerTrajectory middlePath = PathPlanner.loadPath("Middle Cube ID7", new PathConstraints(0.8, 1));
    PathPlannerTrajectory rightPath = PathPlanner.loadPath("Outside Cube ID6", new PathConstraints(0.75, 1));
    eventMap.put("intake", new AutoSuckPieceCmd(rollerSubsystem));
    eventMap.put("grabberdown", new GrabberToSetpointCmd(grabberSubsystem, Constants.GrabberConstants.grabberIntakeSetpoint));
    eventMap.put("grabberin", new GrabberToSetpointCmd(grabberSubsystem, Constants.GrabberConstants.grabberInSetpoint));


    autoBuilder = new SwerveAutoBuilder(
    swerveSubsystem::getPose, // Pose2d supplier
    swerveSubsystem::resetPoseEstimator, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(1.2, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.3, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    (SwerveModuleState[] desiredStates) -> swerveSubsystem.setModuleStates(desiredStates, true), // Module states consumer used to output to the drive subsytem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
);

    Supplier<CommandBase> pathPlannerCommand = () -> autoBuilder.fullAuto(testPath);
    Supplier<CommandBase> middlePathCommand = () -> autoBuilder.fullAuto(middlePath);
    Supplier<CommandBase> rightPathCommand = () -> autoBuilder.fullAuto(rightPath);

    
    Command alignToApriltagCubeCmd = new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem);
    Command alignToDoubleSubstation = new AlignToDoubleSubstation(limelightSubsystem, swerveSubsystem);
    Command autoSuckPieceCmd = new AutoSuckPieceCmd(rollerSubsystem);

    //Adds a smartdashboard widget that will allow us to select the autonomous we want to use. 
    m_chooser = new SendableChooser<>();
    //Default Autonomous that will be run if no other auto is selected

    m_chooser.setDefaultOption("AlignToAprilTagCubeCmd",alignToApriltagCubeCmd);
    m_chooser.addOption("ScoreHighCmd", generateScoreHighCmd());
    m_chooser.addOption("Double Sub", alignToDoubleSubstation);
    m_chooser.addOption("AutoSuck", autoSuckPieceCmd);
    m_chooser.addOption("Middle Path", autoBuilder.fullAuto(middlePath));
    m_chooser.addOption("Right Path", autoBuilder.fullAuto(rightPath));


    
//    m_chooser.addOption("AlignAndScoreHigh", alignToApriltagCubeCmd.andThen(scoreHighCmd));

    //m_chooser.addOption("ScoreCubeLowCmd", new ScoreCubeLowCmd(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem)); 
    
    m_chooser.addOption("Path Tester", pathPlannerCommand.get());
    m_chooser.addOption("6 / 3 LOADING ZONE",
            new SequentialCommandGroup(
                    generateScoreHighCmd(),
                    Commands.runOnce(horizontalElevatorSubsystem::removeDefaultCommand),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(-2)),
                    pathPlannerCommand.get(),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0)),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem)))));
    m_chooser.addOption("2 / 7 MIDDLE BALANCE",
            new SequentialCommandGroup(
                    generateScoreHighCmd(),
                    Commands.runOnce(horizontalElevatorSubsystem::removeDefaultCommand),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(-2)),
                    middlePathCommand.get(),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0)),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem)))));
    m_chooser.addOption("1 / 8 ",
            new SequentialCommandGroup(
                    generateScoreHighCmd(),
                    Commands.runOnce(horizontalElevatorSubsystem::removeDefaultCommand),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(-2)),
                    rightPathCommand.get(),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0)),
                    Commands.runOnce(() -> horizontalElevatorSubsystem.setDefaultCommand(new HorizontalElevatorInCmd(horizontalElevatorSubsystem)))));

    m_chooser.addOption("Score Cube Mid", generateScoreCubeMidCmd());

    SmartDashboard.putData(m_chooser);

    configureBindings();
  }

  public void setMechanismDefaultCommands() {
    horizontalElevatorSubsystem.setDefaultCommand(new HoldHorizontalElevatorInCmd(horizontalElevatorSubsystem));
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
    driverController.a().whileTrue(new CrossWheelsCmd(swerveSubsystem));

    // driverController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
    // driverController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
    driverController.leftTrigger(0.4).whileTrue(new SuckObjectCmd(rollerSubsystem));
    driverController.rightTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem));
//    driverController.x().whileTrue(new CrossWheelsCmd(swerveSubsystem));
    operatorController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
    operatorController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
    operatorController.leftTrigger(0.4).whileTrue(new SuckObjectCmd(rollerSubsystem));
    operatorController.rightTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem));

    operatorController.start().whileTrue(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem));
    operatorController.back().whileTrue(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));

    vertHighButton.whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight));
    scoreCubeHighButton.whileTrue(generateScoreHighCmd());
    vertMidButton.whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, Units.inchesToMeters(10)));
    // setHorizontalToOutButton.onTrue(Commands.runOnce(() -> horizontalElevatorSubsystem.set, null));
    scoreCubeMidButton.whileTrue(generateScoreCubeMidCmd());
    doubleSubstationElevatorButton.whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, Units.inchesToMeters(26)));

    operatorController.a().onTrue(Commands.runOnce(() -> verticalElevatorSubsystem.setVerticalElevatorVoltage(0), verticalElevatorSubsystem).andThen(Commands.runOnce(verticalElevatorSubsystem::resetVerticalElevatorEncoder, verticalElevatorSubsystem)));
    operatorController.b().onTrue(Commands.runOnce(horizontalElevatorSubsystem::resetHorizontalElevatorEncoder));
    operatorController.x().onTrue(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(1)));
    operatorController.x().onFalse(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(2)));
    setDefaultCommandsButton.onTrue(Commands.runOnce(this::setMechanismDefaultCommands));
//    operatorController.y().onTrue(new AutoShootPieceCmd(rollerSubsystem));

//    operatorController.a().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalHighHeight));
//    operatorController.x().whileTrue(new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, Units.inchesToMeters(2)));
//    operatorController.b().whileTrue(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeSetpoint));
//    operatorController.y().whileTrue(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint));

  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

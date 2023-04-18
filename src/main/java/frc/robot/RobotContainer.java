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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.WaitUntilConditionForTimeCmd;
import frc.robot.commands.drive.VeloAutoBalanceCmd;
import frc.robot.commands.elevator.HoldHorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.ManualVerticalElevatorController;
import frc.robot.commands.grabber.AutoShootPieceCmd;
import frc.robot.commands.grabber.ClampGrabberCmd;
import frc.robot.commands.grabber.GrabberJoystickControlCmd;
import frc.robot.commands.grabber.GrabberToSetpointCmd;
import frc.robot.commands.grabber.OpenGrabberCmd;
import frc.robot.commands.grabber.ReEnableCompressorCmd;
import frc.robot.commands.grabber.ShootPieceCmd;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.commands.drive.CrossWheelsCmd;
import frc.robot.commands.drive.SwerveJoystickCmd;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    // private final LimelightSubsystem limelightSubsystem = new
    // LimelightSubsystem();
    private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
    private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
    private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();

    private final HorizontalElevatorSubsystem horizontalElevatorSubsystem = new HorizontalElevatorSubsystem();
    private final VerticalElevatorSubsystem verticalElevatorSubsystem = new VerticalElevatorSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(
            OIConstants.kOperatorControllerPort);
    private final Joystick buttonBox = new Joystick(OIConstants.kButtonBoxPort);

    private final JoystickButton scoreCubeMidButton = new JoystickButton(buttonBox, 1);
    private final JoystickButton scoreCubeHighButton = new JoystickButton(buttonBox, 2);
    private final JoystickButton reEnableCompressorButton = new JoystickButton(buttonBox, 4);
    private final JoystickButton setDefaultCommandsButton = new JoystickButton(buttonBox, 5);
    private final JoystickButton setGrabberEncoderToLowButton = new JoystickButton(buttonBox, 8);

    private SwerveAutoBuilder autoBuilder;
    private SwerveAutoBuilder heavyRotationCorrectionAutoBuilder;
    private HashMap<String, Command> eventMap = new HashMap<>();
    private final SendableChooser<Command> autonChooser = new SendableChooser<>(); // Adds a smartdashboard widget that
                                                                                   // will allow us to select the
                                                                                   // autonomous we want to use.

    private Command generateScoreHighCmd() {
        return Commands.runOnce(() -> SmartDashboard.putString("Score High", "running"))
                .andThen(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberShootCubeHighSetpoint))
                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 1)))
                .andThen(new AutoShootPieceCmd(rollerSubsystem, 0.5, 0.20))
                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 2)))
                .andThen(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberInSetpoint))
                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 3)))
                .andThen(Commands.runOnce(() -> SmartDashboard.putString("Score High", "done")))
                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 0)));
    }

    private Command generateScoreCubeMidCmd() {
        return Commands.runOnce(() -> SmartDashboard.putString("Score Cube Mid", "running"))
                .andThen(new GrabberToSetpointCmd(grabberSubsystem, GrabberConstants.grabberDepositCubeMidSetpoint))
                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("stage", 1)))
                .andThen(new AutoShootPieceCmd(rollerSubsystem, 0.5, 0.10))
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

        PathPlannerTrajectory nonSubMobilityPath = PathPlanner.loadPath("Non Sub Mobility",
                new PathConstraints(0.75, 1));
        PathPlannerTrajectory subMobilityPath = PathPlanner.loadPath("Sub Mobility", new PathConstraints(0.75, 1));
        PathPlannerTrajectory balancePath = PathPlanner.loadPath("Get on charge", new PathConstraints(1, 1));

        autoBuilder = new SwerveAutoBuilder(
                swerveSubsystem::getPose, // Pose2d supplier
                swerveSubsystem::resetPoseEstimator, // Pose2d consumer, used to reset odometry at the beginning of auto
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDConstants(1.2, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y PID controllers)
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                 // rotation controller)
                (SwerveModuleState[] desiredStates) -> swerveSubsystem.setModuleStates(desiredStates, true), // Module
                                                                                                             // states
                                                                                                             // consumer
                                                                                                             // used to
                                                                                                             // output
                                                                                                             // to the
                                                                                                             // drive
                                                                                                             // subsytem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following
                                // commands
        );

        heavyRotationCorrectionAutoBuilder = new SwerveAutoBuilder(
                swerveSubsystem::getPose, // Pose2d supplier
                swerveSubsystem::resetPoseEstimator, // Pose2d consumer, used to reset odometry at the beginning of auto
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDConstants(1.2, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y PID controllers)
                new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                 // rotation controller)
                (SwerveModuleState[] desiredStates) -> swerveSubsystem.setModuleStates(desiredStates, true), // Module
                                                                                                             // states
                                                                                                             // consumer
                                                                                                             // used to
                                                                                                             // output
                                                                                                             // to the
                                                                                                             // drive
                                                                                                             // subsytem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following
                                // commands
        );

        Supplier<CommandBase> nonSubMobilityPathCommand = () -> autoBuilder.fullAuto(nonSubMobilityPath);
        Supplier<CommandBase> subMobilityPathCommand = () -> autoBuilder.fullAuto(subMobilityPath);
        Supplier<CommandBase> balancePathCommand = () -> heavyRotationCorrectionAutoBuilder.fullAuto(balancePath);

        autonChooser.addOption("Score High", generateScoreHighCmd());
        autonChooser.addOption("[Non Sub] Score High + Mobility",
                new SequentialCommandGroup(
                        generateScoreHighCmd(),
                        nonSubMobilityPathCommand.get()));
        autonChooser.addOption("[Sub] Score High and Mobility",
                new SequentialCommandGroup(
                        generateScoreHighCmd(),
                        subMobilityPathCommand.get()));
        autonChooser.addOption("[Timed] 1 Score High and Balance",
                new SequentialCommandGroup(
                        Commands.runOnce(() -> SmartDashboard.putNumber("Crazy Stage", 1)),
                        generateScoreHighCmd(),
                        Commands.runOnce(() -> SmartDashboard.putNumber("Crazy Stage", 2)),
                        new ParallelDeadlineGroup(
                                new WaitUntilConditionForTimeCmd(() -> swerveSubsystem.getRoll() > 14, 1),
                                balancePathCommand.get(), Commands.runOnce(() -> grabberSubsystem.setMotorVoltage(0.2)))
                                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("Crazy Stage", 3)))
                                .andThen(new VeloAutoBalanceCmd(swerveSubsystem))
                                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("Crazy Stage", 4)))
                                .andThen(new CrossWheelsCmd(swerveSubsystem))
                                .andThen(Commands.runOnce(() -> SmartDashboard.putNumber("Crazy Stage", 5)))
                                .andThen(Commands.runOnce(() -> grabberSubsystem.setMotorVoltage(0)))

                ));

        SmartDashboard.putData(autonChooser);

        configureBindings();
    }

    public void setMechanismDefaultCommands() {
        horizontalElevatorSubsystem.setDefaultCommand(new HoldHorizontalElevatorInCmd(horizontalElevatorSubsystem));
        verticalElevatorSubsystem.setDefaultCommand(
                new ManualVerticalElevatorController(verticalElevatorSubsystem, () -> -operatorController.getRightY()));
        grabberSubsystem.setDefaultCommand(
                new GrabberJoystickControlCmd(grabberSubsystem, () -> -operatorController.getLeftY()));
    }

    private void configureBindings() {
        driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        driverController.a().whileTrue(new CrossWheelsCmd(swerveSubsystem));
        driverController.x().whileTrue(new SequentialCommandGroup(
                // new AutoBackupToTippedCmd(swerveSubsystem),
                new VeloAutoBalanceCmd(swerveSubsystem),
                new CrossWheelsCmd(swerveSubsystem)));

        driverController.leftTrigger(0.4).whileTrue(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(1)));
        driverController.leftTrigger(0.4).onFalse(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(2)));
        driverController.rightTrigger(0.4).whileTrue(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(3)));
        driverController.rightTrigger(0.4).onFalse(Commands.runOnce(() -> swerveSubsystem.setSpeedMultiplier(2)));

        operatorController.leftBumper().whileTrue(new ClampGrabberCmd(pneumaticsSubsystem));
        operatorController.rightBumper().whileTrue(new OpenGrabberCmd(pneumaticsSubsystem));
        operatorController.leftTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem, 0.10));
        operatorController.rightTrigger(0.4).whileTrue(new ShootPieceCmd(rollerSubsystem));

        operatorController.start().whileTrue(new HorizontalElevatorOutCmd(horizontalElevatorSubsystem));
        operatorController.back().whileTrue(new HorizontalElevatorInCmd(horizontalElevatorSubsystem));

        scoreCubeHighButton.whileTrue(generateScoreHighCmd());
        scoreCubeMidButton.whileTrue(generateScoreCubeMidCmd());

        setDefaultCommandsButton.onTrue(Commands.runOnce(this::setMechanismDefaultCommands));

        reEnableCompressorButton.onTrue(new ReEnableCompressorCmd(pneumaticsSubsystem));

        setGrabberEncoderToLowButton.onTrue(Commands.runOnce(grabberSubsystem::resetEncoderToLow));

    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}

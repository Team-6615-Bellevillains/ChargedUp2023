package frc.robot.commands.operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.VerticalElevatorLowCmd;
import frc.robot.commands.elevator.VerticalElevatorMidCmd;
import frc.robot.commands.grabber.FlipGrabberInCmd;
import frc.robot.commands.grabber.ShootPieceCmd;
import frc.robot.subsystems.*;

public class ScoreCubeMidCmd extends CommandBase {

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;
    private VerticalElevatorSubsystem verticalElevatorSubsystem;
    private GrabberSubsystem grabberSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private VerticalElevatorMidCmd verticalElevatorMidCmd = new VerticalElevatorMidCmd(verticalElevatorSubsystem);
    private HorizontalElevatorOutCmd horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(horizontalElevatorSubsystem);

    private FlipGrabberInCmd flipGrabberInCmd = new FlipGrabberInCmd(grabberSubsystem);
    private ShootPieceCmd shootPieceCmd = new ShootPieceCmd(grabberSubsystem);

    private HorizontalElevatorInCmd horizontalElevatorInCmd = new HorizontalElevatorInCmd(horizontalElevatorSubsystem);
    private VerticalElevatorLowCmd verticalElevatorLowCmd = new VerticalElevatorLowCmd(verticalElevatorSubsystem);

    private AlignToAprilTagCubeCmd alignToAprilTagCubeCmd = new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem);

    private Command routine;

    public ScoreCubeMidCmd(
            HorizontalElevatorSubsystem horizontalElevatorSubsystem, VerticalElevatorSubsystem verticalElevatorSubsystem, GrabberSubsystem grabberSubsystem,
            SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {
        routine = alignToAprilTagCubeCmd
                .andThen(verticalElevatorMidCmd)
                .andThen(horizontalElevatorOutCmd)
                .andThen(Commands.parallel(new WaitCommand(0.5), shootPieceCmd))
                .andThen(flipGrabberInCmd)
                .andThen(horizontalElevatorInCmd)
                .andThen(verticalElevatorLowCmd);

        routine.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            routine.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return routine.isFinished();
    }

}

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

    private final VerticalElevatorMidCmd verticalElevatorMidCmd;
    private final HorizontalElevatorOutCmd horizontalElevatorOutCmd;

    private final FlipGrabberInCmd flipGrabberInCmd;
    private final ShootPieceCmd shootPieceCmd;

    private final HorizontalElevatorInCmd horizontalElevatorInCmd;
    private final VerticalElevatorLowCmd verticalElevatorLowCmd;

    private final AlignToAprilTagCubeCmd alignToAprilTagCubeCmd;

    private final Command routine;

    public ScoreCubeMidCmd(
            HorizontalElevatorSubsystem horizontalElevatorSubsystem, VerticalElevatorSubsystem verticalElevatorSubsystem, GrabberSubsystem grabberSubsystem,
            SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(horizontalElevatorSubsystem);
        horizontalElevatorInCmd = new HorizontalElevatorInCmd(horizontalElevatorSubsystem);

        verticalElevatorMidCmd = new VerticalElevatorMidCmd(verticalElevatorSubsystem);
        verticalElevatorLowCmd = new VerticalElevatorLowCmd(verticalElevatorSubsystem);

        shootPieceCmd = new ShootPieceCmd(grabberSubsystem);
        flipGrabberInCmd = new FlipGrabberInCmd(grabberSubsystem);

        alignToAprilTagCubeCmd = new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem);

        routine = alignToAprilTagCubeCmd
                .andThen(verticalElevatorMidCmd)
                .andThen(horizontalElevatorOutCmd)
                .andThen(Commands.parallel(new WaitCommand(0.5), shootPieceCmd))
                .andThen(flipGrabberInCmd)
                .andThen(horizontalElevatorInCmd)
                .andThen(verticalElevatorLowCmd);

        addRequirements(horizontalElevatorSubsystem, verticalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
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

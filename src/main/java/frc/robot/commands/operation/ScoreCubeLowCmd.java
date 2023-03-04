package frc.robot.commands.operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToAprilTagCubeCmd;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.grabber.FlipGrabberIntakeCmd;
import frc.robot.commands.grabber.ShootPieceCmd;
import frc.robot.subsystems.*;

public class ScoreCubeLowCmd extends CommandBase {

    private final HorizontalElevatorOutCmd horizontalElevatorOutCmd;

    private final FlipGrabberIntakeCmd flipGrabberInCmd;
    private final ShootPieceCmd shootPieceCmd;

    private final HorizontalElevatorInCmd horizontalElevatorInCmd;

    private final AlignToAprilTagCubeCmd alignToAprilTagCubeCmd;

    private final Command routine;

    public ScoreCubeLowCmd(
            HorizontalElevatorSubsystem horizontalElevatorSubsystem, GrabberSubsystem grabberSubsystem,
            SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(horizontalElevatorSubsystem);
        horizontalElevatorInCmd = new HorizontalElevatorInCmd(horizontalElevatorSubsystem);


        shootPieceCmd = new ShootPieceCmd(grabberSubsystem);
        flipGrabberInCmd = new FlipGrabberIntakeCmd(grabberSubsystem, true);

        alignToAprilTagCubeCmd = new AlignToAprilTagCubeCmd(limelightSubsystem, swerveSubsystem);

        routine = alignToAprilTagCubeCmd
                .andThen(horizontalElevatorOutCmd)
                .andThen(Commands.parallel(new WaitCommand(0.5), shootPieceCmd))
                .andThen(flipGrabberInCmd)
                .andThen(horizontalElevatorInCmd);
    

        addRequirements(horizontalElevatorSubsystem, grabberSubsystem, swerveSubsystem, limelightSubsystem);
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

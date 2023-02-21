package frc.robot.commands.operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.MoveDistanceCmd;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.VerticalElevatorLowCmd;
import frc.robot.commands.elevator.VerticalElevatorMidCmd;
import frc.robot.commands.grabber.ClampGrabberCmd;
import frc.robot.commands.grabber.FlipGrabberInCmd;
import frc.robot.commands.grabber.OpenGrabberCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreHighCmd extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;
    private GrabberSubsystem grabberSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private VerticalElevatorMidCmd verticalElevatorMidCmd = new VerticalElevatorMidCmd(elevatorSubsystem);
    private HorizontalElevatorOutCmd horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(elevatorSubsystem);

    private OpenGrabberCmd openGrabberCmd = new OpenGrabberCmd(grabberSubsystem);
    private ClampGrabberCmd clampGrabberCmd = new ClampGrabberCmd(grabberSubsystem);
    private FlipGrabberInCmd flipGrabberInCmd = new FlipGrabberInCmd(grabberSubsystem);

    private HorizontalElevatorInCmd horizontalElevatorInCmd = new HorizontalElevatorInCmd(elevatorSubsystem);
    private VerticalElevatorLowCmd verticalElevatorLowCmd = new VerticalElevatorLowCmd(elevatorSubsystem);

    // TODO: Find Distances
    private MoveDistanceCmd moveDistanceCmd = new MoveDistanceCmd(swerveSubsystem, .5, 0);

    private Command routine;

    public ScoreHighCmd(
            ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(elevatorSubsystem, grabberSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {
        routine = moveDistanceCmd
                .andThen(Commands.parallel(verticalElevatorMidCmd, horizontalElevatorOutCmd))
                .andThen(openGrabberCmd)
                .andThen(new WaitCommand(0.5))
                .andThen(clampGrabberCmd)
                .andThen(Commands.parallel(flipGrabberInCmd, horizontalElevatorInCmd, verticalElevatorLowCmd));

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

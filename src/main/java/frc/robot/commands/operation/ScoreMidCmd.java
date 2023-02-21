package frc.robot.commands.operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.VerticalElevatorLowCmd;
import frc.robot.commands.elevator.VerticalElevatorMidCmd;
import frc.robot.commands.grabber.ClampGrabberCmd;
import frc.robot.commands.grabber.FlipGrabberInCmd;
import frc.robot.commands.grabber.OpenGrabberCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreMidCmd extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;
    private GrabberSubsystem grabberSubsystem;

    private VerticalElevatorMidCmd verticalElevatorMidCmd = new VerticalElevatorMidCmd(elevatorSubsystem);
    private HorizontalElevatorOutCmd horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(elevatorSubsystem);

    private OpenGrabberCmd openGrabberCmd = new OpenGrabberCmd(grabberSubsystem);
    private ClampGrabberCmd clampGrabberCmd = new ClampGrabberCmd(grabberSubsystem);
    private FlipGrabberInCmd flipGrabberInCmd = new FlipGrabberInCmd(grabberSubsystem);

    private HorizontalElevatorInCmd horizontalElevatorInCmd = new HorizontalElevatorInCmd(elevatorSubsystem);
    private VerticalElevatorLowCmd verticalElevatorLowCmd = new VerticalElevatorLowCmd(elevatorSubsystem);

    private Command routine;

    public ScoreMidCmd(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(elevatorSubsystem, grabberSubsystem);
    }

    @Override
    public void initialize() {
        routine = Commands.parallel(verticalElevatorMidCmd, horizontalElevatorOutCmd)
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

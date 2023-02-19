package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ArmInCmd;
import frc.robot.commands.elevator.ArmOutCmd;
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
    private ArmOutCmd armOutCmd = new ArmOutCmd(elevatorSubsystem);

    private OpenGrabberCmd openGrabberCmd = new OpenGrabberCmd(grabberSubsystem);
    private ClampGrabberCmd clampGrabberCmd = new ClampGrabberCmd(grabberSubsystem);
    private FlipGrabberInCmd flipGrabberInCmd = new FlipGrabberInCmd(grabberSubsystem);

    private ArmInCmd armInCmd = new ArmInCmd(elevatorSubsystem);
    private VerticalElevatorLowCmd verticalElevatorLowCmd = new VerticalElevatorLowCmd(elevatorSubsystem);

    public ScoreMidCmd(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(elevatorSubsystem, grabberSubsystem);
    }

    @Override
    public void initialize() {
        Commands.parallel(verticalElevatorMidCmd, armOutCmd)
                .andThen(openGrabberCmd)
                .andThen(new WaitCommand(0.5))
                .andThen(clampGrabberCmd)
                .andThen(Commands.parallel(flipGrabberInCmd, armInCmd, verticalElevatorLowCmd)).schedule();
    }

}

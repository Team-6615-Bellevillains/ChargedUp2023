package frc.robot.commands.operation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.elevator.HorizontalElevatorInCmd;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.VerticalElevatorToSetpointCmd;
import frc.robot.commands.grabber.AutoShootPieceCmd;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class ScoreHighCmd extends CommandBase {
    // TODO: Move command initialization to constructor (otherwise subsystems are null), remove then unnecessary instance variables for subsystems

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;
    private VerticalElevatorSubsystem verticalElevatorSubsystem;
    private GrabberSubsystem grabberSubsystem;
    private RollerSubsystem rollerSubsystem;

    private Command routine;

    public ScoreHighCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem, GrabberSubsystem grabberSubsystem, VerticalElevatorSubsystem verticalElevatorSubsystem, RollerSubsystem rollerSubsystem
            ) {

        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.rollerSubsystem = rollerSubsystem;

        addRequirements(horizontalElevatorSubsystem, grabberSubsystem, verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        VerticalElevatorToSetpointCmd midHeight = new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, 0);
        HorizontalElevatorOutCmd horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(horizontalElevatorSubsystem);
        AutoShootPieceCmd shootPieceCmd = new AutoShootPieceCmd(rollerSubsystem);
        //the grabber setpoint cmd

        routine = midHeight.andThen(horizontalElevatorOutCmd).andThen(shootPieceCmd);
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

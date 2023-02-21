package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class HorizontalElevatorOutCmd extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    public HorizontalElevatorOutCmd(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class VerticalElevatorHighCmd extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    public VerticalElevatorHighCmd(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

}
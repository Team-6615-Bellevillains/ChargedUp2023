package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class VerticalElevatorLowCmd extends CommandBase {

    private final VerticalElevatorToSetpointCmd verticalElevatorToSetpointCmd;

    public VerticalElevatorLowCmd(ElevatorSubsystem elevatorSubsystem) {
        this.verticalElevatorToSetpointCmd = new VerticalElevatorToSetpointCmd(elevatorSubsystem, 0);

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        verticalElevatorToSetpointCmd.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorToSetpointCmd.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return verticalElevatorToSetpointCmd.isFinished();
    }

}
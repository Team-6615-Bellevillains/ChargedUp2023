package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class HorizontalElevatorInCmd extends CommandBase {

    private final HorizontalElevatorToSetpointCmd horizontalElevatorToSetpointCmd;

    public HorizontalElevatorInCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem) {
        this.horizontalElevatorToSetpointCmd = new HorizontalElevatorToSetpointCmd(horizontalElevatorSubsystem, ElevatorConstants.horizontalInLength);

        addRequirements(horizontalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        horizontalElevatorToSetpointCmd.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        horizontalElevatorToSetpointCmd.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return horizontalElevatorToSetpointCmd.isFinished();
    }

}
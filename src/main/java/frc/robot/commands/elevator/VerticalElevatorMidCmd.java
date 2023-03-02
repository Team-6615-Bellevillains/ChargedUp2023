package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class VerticalElevatorMidCmd extends CommandBase {

    private final VerticalElevatorToSetpointCmd verticalElevatorToSetpointCmd;

    public VerticalElevatorMidCmd(VerticalElevatorSubsystem verticalElevatorSubsystem) {
        this.verticalElevatorToSetpointCmd = new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, ElevatorConstants.verticalMidHeight);

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        verticalElevatorToSetpointCmd.initialize();
    }

    @Override
    public void execute() {
        verticalElevatorToSetpointCmd.execute();
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
package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class HoldHorizontalElevatorInCmd extends CommandBase {

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;
    private boolean holding = false;

    public HoldHorizontalElevatorInCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;

        addRequirements(horizontalElevatorSubsystem);
    }
    
    // TODO: Remove, shouldn't be needed
    @Override
    public void initialize() {
        holding = false;
        horizontalElevatorSubsystem.setHorizontalElevatorCurrentLimitState(false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Horizontal Holding", holding);
        horizontalElevatorSubsystem.setHorizontalElevatorSpeed(-0.4);
        if (horizontalElevatorSubsystem.getHorizontalElevatorStatorCurrent() <= ElevatorConstants.kHorizontalMotorHoldingStatorCurrentThreshold) {
            holding = true;
            horizontalElevatorSubsystem.setHorizontalElevatorCurrentLimitState(true);
        }

        if (holding) {
            horizontalElevatorSubsystem.resetHorizontalElevatorEncoder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        holding = false;
        horizontalElevatorSubsystem.setHorizontalElevatorCurrentLimitState(false);

        SmartDashboard.putBoolean("Horizontal Holding", false);

    }

}

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class HorizontalElevatorOutCmd extends CommandBase {

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;



    public HorizontalElevatorOutCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;

        addRequirements(horizontalElevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (horizontalElevatorSubsystem.getHorizontalElevatorPosition() < ElevatorConstants.kHorizontalElevatorOutThreshold) {
            horizontalElevatorSubsystem.setHorizontalElevatorVoltage(horizontalElevatorSubsystem.calculateFeedforward(ElevatorConstants.kHorizontalElevatorFFInput));
        } else {
            horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
    }

}
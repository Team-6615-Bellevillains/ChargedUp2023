package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(horizontalElevatorSubsystem.calculateFeedforward(ElevatorConstants.kHorizontalElevatorFFInput));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Stop Hori TS", Timer.getFPGATimestamp());

        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return horizontalElevatorSubsystem.getHorizontalElevatorPosition() >= ElevatorConstants.kHorizontalElevatorOutThreshold;
    }

}
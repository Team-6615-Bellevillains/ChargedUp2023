package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class HorizontalElevatorInCmd extends CommandBase {

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;

    private SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(1.5, 1);


    public HorizontalElevatorInCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;

        addRequirements(horizontalElevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (horizontalElevatorSubsystem.getHorizontalElevatorPosition() > 1) {
            horizontalElevatorSubsystem.setHorizontalElevatorVoltage(simpleMotorFeedforward.calculate(-2));
        } else {
            horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
    }

}
package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.utils.CustomElevatorFeedforward;

import java.util.function.Supplier;

public class ManualVerticalElevatorController extends CommandBase {

    private VerticalElevatorSubsystem verticalElevatorSubsystem;
    private Supplier<Double> controllerPowerOutput;

    public ManualVerticalElevatorController(VerticalElevatorSubsystem verticalElevatorSubsystem, Supplier<Double> controllerPowerOutput) {
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;
        this.controllerPowerOutput = controllerPowerOutput;

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void execute() {
        double power = CustomElevatorFeedforward.feedforward(controllerPowerOutput.get());
        SmartDashboard.putNumber("Power to vertical", power);
        verticalElevatorSubsystem.setVerticalElevatorVoltage(power);
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.setVerticalElevatorVoltage(CustomElevatorFeedforward.feedforward(0));
    }

}

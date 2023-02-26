package frc.robot.commands.elevator;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

import java.util.function.Supplier;

public class ManualHorizontalElevatorController extends CommandBase {

    private HorizontalElevatorSubsystem horizontalElevatorSubsystem;
    private Supplier<Double> controllerPowerOutput;
    private SlewRateLimiter powerOutputLimiter;

    public ManualHorizontalElevatorController(HorizontalElevatorSubsystem horizontalElevatorSubsystem, Supplier<Double> controllerPowerOutput) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;
        this.controllerPowerOutput = controllerPowerOutput;
        this.powerOutputLimiter = new SlewRateLimiter(3);

        addRequirements(horizontalElevatorSubsystem);
    }

    @Override
    public void execute() {
        double power = controllerPowerOutput.get()*12/2;
        SmartDashboard.putNumber("Power to horizontal", power);
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(power);
    }

    @Override
    public void end(boolean interrupted) {
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0);
    }

}

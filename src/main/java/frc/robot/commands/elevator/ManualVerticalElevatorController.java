package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;

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
        SmartDashboard.putNumber("TS (1)", Timer.getFPGATimestamp());
        double velocity = (MathUtil.applyDeadband(controllerPowerOutput.get(), OIConstants.kDeadband) / 5);
        SmartDashboard.putNumber("Velo Desired (in/s)", Units.metersToInches(velocity));
        SmartDashboard.putNumber("Velo True (in/s)", Units.metersToInches(verticalElevatorSubsystem.getVerticalElevatorVelocity()));
        SmartDashboard.putNumber("TS (2)", Timer.getFPGATimestamp());
        verticalElevatorSubsystem.setVerticalElevatorVoltage(verticalElevatorSubsystem.calculateFeedforward(velocity));
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.stopElevator();
    }

}

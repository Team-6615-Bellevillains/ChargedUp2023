package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;

import java.util.function.Supplier;

public class ManualVerticalElevatorController extends CommandBase {

    private VerticalElevatorSubsystem verticalElevatorSubsystem;
    private Supplier<Double> controllerPowerOutput;

    public ManualVerticalElevatorController(VerticalElevatorSubsystem verticalElevatorSubsystem,
            Supplier<Double> controllerPowerOutput) {
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;
        this.controllerPowerOutput = controllerPowerOutput;

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void execute() {
        double controllerPower = MathUtil.applyDeadband(controllerPowerOutput.get(),
                OIConstants.kOperatorControllerRightYDeadband);
        double velocity = (controllerPower / 2);

        SmartDashboard.putNumber("Velo Desired (in per s)", Units.metersToInches(velocity));
        SmartDashboard.putNumber("Velo Desired (m per s)", velocity);

        if (controllerPower == 0
                && verticalElevatorSubsystem.getVerticalElevatorPosition() <= ElevatorConstants.verticalRestThreshold) {
            verticalElevatorSubsystem.setVerticalElevatorVoltage(0);
            verticalElevatorSubsystem.resetVerticalElevatorEncoder();
        } else {
            verticalElevatorSubsystem
                    .setVerticalElevatorVoltage(verticalElevatorSubsystem.calculateFeedforward(velocity));
        }

    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.stopElevator();
    }

}

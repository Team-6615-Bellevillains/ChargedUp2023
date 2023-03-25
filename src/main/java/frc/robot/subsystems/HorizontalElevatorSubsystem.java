package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class HorizontalElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kSHorizontalElevator, ElevatorConstants.kVHorizontalElevator);
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPHorizontalElevator, ElevatorConstants.kIHorizontalElevator, ElevatorConstants.kDHorizontalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityHorizontalElevator, ElevatorConstants.kMaxAccelerationHorizontalElevator));


    public HorizontalElevatorSubsystem() {
        hElevatorMotor.setInverted(true);
        hElevatorMotor.setSensorPhase(true);
        hElevatorMotor.configPeakCurrentLimit(0);
        hElevatorMotor.configContinuousCurrentLimit(ElevatorConstants.kHorizontalMotorActiveHoldingSupplyCurrent);

        resetHorizontalElevatorEncoder();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Horizontal Position", getHorizontalElevatorPosition());
        SmartDashboard.putNumber("Horizontal Supply Current", hElevatorMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Horizontal Stator Current", hElevatorMotor.getStatorCurrent());
    }

    public double calculateFeedforward(double velocity) {
        return feedforward.calculate(velocity);
    }

    double lastVoltage = 0;

    public void setHorizontalElevatorVoltage(double voltage) {
        lastVoltage = voltage;
        SmartDashboard.putNumber("Hori voltage", voltage);
        hElevatorMotor.setVoltage(voltage);
    }

    public void setHorizontalElevatorSpeed(double speed) {
        hElevatorMotor.set(speed);
    }


    public double getHorizontalElevatorPosition() {
        if (hElevatorMotor.getSelectedSensorPosition() < 0) {
            hElevatorMotor.setSelectedSensorPosition(0);
        }
        return hElevatorMotor.getSelectedSensorPosition() * ElevatorConstants.horizontalRotationsToDistance
                / ElevatorConstants.horizontalEncoderPulsesPerRevolution;
    }

    public double getHorizontalElevatorVelocity() {
        return hElevatorMotor.getSelectedSensorVelocity() * (ElevatorConstants.horizontalRotationsToDistance
                / ElevatorConstants.horizontalEncoderPulsesPerRevolution) / 60;
    }

    public double getHorizontalElevatorStatorCurrent() {
        return hElevatorMotor.getStatorCurrent();
    }

    public void setHorizontalElevatorCurrentLimitState(boolean enabled) {
        hElevatorMotor.enableCurrentLimit(enabled);
    }

    public void resetHorizontalElevatorEncoder() {
        hElevatorMotor.setSelectedSensorPosition(0);
    }

    public double getHorizontalElevatorRawEncoder() 
    {
        return hElevatorMotor.getSelectedSensorPosition();
    }

}
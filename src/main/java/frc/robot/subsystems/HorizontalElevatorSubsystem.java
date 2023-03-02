package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import java.lang.reflect.Field;

public class HorizontalElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX hElevatorMotor;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kSHorizontalElevator, ElevatorConstants.kVHorizontalElevator);
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPHorizontalElevator, ElevatorConstants.kIHorizontalElevator, ElevatorConstants.kDHorizontalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityHorizontalElevator, ElevatorConstants.kMaxAccelerationHorizontalElevator));


    public HorizontalElevatorSubsystem() {
        this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);
        this.hElevatorMotor.setInverted(true);
        this.hElevatorMotor.setSensorPhase(true);

        resetHorizontalElevatorEncoder();
    }

    public double calculateFeedforward(double velocity) {
        return feedforward.calculate(velocity);
    }


    public void setHorizontalElevatorVoltage(double voltage) {
        hElevatorMotor.setVoltage(voltage);
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

    public void resetHorizontalElevatorEncoder() {
        hElevatorMotor.setSelectedSensorPosition(0);
    }

}
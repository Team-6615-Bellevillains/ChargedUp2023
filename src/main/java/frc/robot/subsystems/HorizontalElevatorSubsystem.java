package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class HorizontalElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX hElevatorMotor;

    public HorizontalElevatorSubsystem() {
        this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);
        this.hElevatorMotor.setInverted(true);

        resetHorizontalElevatorEncoder();
    }

    public void setHorizontalElevatorVoltage(double voltage) {
        hElevatorMotor.setVoltage(voltage);
    }

    public double getHorizontalElevatorPosition() {
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

    public double getHorizontalElevatorRawEncoder() 
    {
        return hElevatorMotor.getSelectedSensorPosition();
    }

    public void periodic()
    {
        
        SmartDashboard.putNumber("Encoder Raw Ticks", getHorizontalElevatorRawEncoder());
    }

}
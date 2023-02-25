package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalMotorA;
    private final WPI_TalonSRX verticalMotorB;

    public VerticalElevatorSubsystem() {
        // The A motor has an encoder, the B motor does not.
        this.verticalMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort);
        this.verticalMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);

        this.verticalMotorA.setInverted(true);

        resetVerticalElevatorEncoder();
    }

    public void setVerticalElevatorVoltage(double voltage) {
        verticalMotorA.setVoltage(voltage);
        verticalMotorB.setVoltage(voltage);
    }

    public double getVerticalElevatorPosition() {
        return verticalMotorA.getSelectedSensorPosition() * ElevatorConstants.verticalRotationsToDistance
                / ElevatorConstants.verticalEncoderPulsesPerRevolution;
    }

    public double getVerticalElevatorVelocity() {
        return verticalMotorA.getSelectedSensorVelocity() * (ElevatorConstants.verticalRotationsToDistance
                / ElevatorConstants.verticalEncoderPulsesPerRevolution) / 60;
    }

    public void resetVerticalElevatorEncoder() {
        verticalMotorA.setSelectedSensorPosition(0);
    }

}
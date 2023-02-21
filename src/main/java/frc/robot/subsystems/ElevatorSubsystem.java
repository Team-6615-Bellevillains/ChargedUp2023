package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalMotorA;
    private final WPI_TalonSRX verticalMotorB;
    private final WPI_TalonSRX hElevatorMotor;

    private final Encoder horizontalEncoder;

    public ElevatorSubsystem() {
        // The A motor has an encoder, the B motor does not.
        this.verticalMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort);
        this.verticalMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);

        this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);

        this.horizontalEncoder = new Encoder(ElevatorConstants.horizonalEncoderPortA,
                ElevatorConstants.horizonalEncoderPortB);
        this.horizontalEncoder.setDistancePerPulse(ElevatorConstants.horizontalRotationsToDistance
                / ElevatorConstants.horizontalEncoderPulsesPerRevolution);
    }

    public void setVerticalElevatorSpeed(double speed) {
        verticalMotorA.set(speed);
        verticalMotorB.set(speed);
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

    public void setHorizontalElevatorSpeed(double speed) {
        hElevatorMotor.set(speed);
    }

    public double getHorizontalElevatorPosition() {
        return horizontalEncoder.getDistance();
    }

    public double getHorizontalElevatorVelocity() {
        return horizontalEncoder.getRate();
    }

    public void resetHorizontalElevatorEncoder() {
        horizontalEncoder.reset();
    }

}
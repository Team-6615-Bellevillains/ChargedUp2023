package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class VerticalElevatorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX verticalMotorA;
    private final WPI_TalonSRX verticalMotorB;

    private final Encoder verticalElevatorEncoder;


    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private double lastUpdatedTS = Timer.getFPGATimestamp();

    private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kSVerticalElevator, ElevatorConstants.kGVerticalElevator, ElevatorConstants.kVVerticalElevator);

    public VerticalElevatorSubsystem() {
        // The A motor has an encoder, the B motor does not.
        this.verticalMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort);
        this.verticalMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);

        this.verticalMotorA.setInverted(true);

        this.verticalElevatorEncoder = new Encoder(ElevatorConstants.verticalMotorAEncoderAPort, ElevatorConstants.verticalMotorAEncoderBPort);
        this.verticalElevatorEncoder.setReverseDirection(true);

        resetVerticalElevatorEncoder();
    }

    public double calculateFeedforward(double velocity) {
        return elevatorFeedforward.calculate(velocity);
    }

    public void stopElevator() {
        setVerticalElevatorVoltage(getVerticalElevatorPosition() <= ElevatorConstants.verticalRestThreshold ? 0 : calculateFeedforward(0));
    }

    public void lowerElevator() {
        setVerticalElevatorVoltage(getVerticalElevatorPosition() <= ElevatorConstants.verticalRestThreshold ? 0 : ElevatorConstants.kVerticalSlowFallVoltage);
    }

    public void setVerticalElevatorVoltage(double voltage) {
        SmartDashboard.putNumber("vert voltage", voltage);
        verticalMotorA.setVoltage(voltage);
        verticalMotorB.setVoltage(voltage);
    }

    public double getVerticalElevatorPosition() {
        return verticalElevatorEncoder.get() * ElevatorConstants.verticalRotationsToDistance
                / ElevatorConstants.verticalEncoderPulsesPerRevolution;
    }

    public double getVerticalElevatorVelocity() {
        return verticalElevatorEncoder.getRate() * (ElevatorConstants.verticalRotationsToDistance
                / ElevatorConstants.verticalEncoderPulsesPerRevolution);
    }

    public void resetVerticalElevatorEncoder() {
        verticalElevatorEncoder.reset();
    }

}
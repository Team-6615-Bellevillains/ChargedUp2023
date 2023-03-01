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

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private double lastUpdatedTS = Timer.getFPGATimestamp();

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ElevatorConstants.kSHorizontalElevator, ElevatorConstants.kVHorizontalElevator);
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPHorizontalElevator, ElevatorConstants.kIHorizontalElevator, ElevatorConstants.kDHorizontalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityHorizontalElevator, ElevatorConstants.kMaxAccelerationHorizontalElevator));


    public HorizontalElevatorSubsystem() {
        this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.horizontalMotorPort);
        this.hElevatorMotor.setInverted(true);
        this.hElevatorMotor.setSensorPhase(true);

        tuningTable.getEntry("kSHorizontalElevator").setDouble(ElevatorConstants.kSHorizontalElevator);
        tuningTable.getEntry("kVHorizontalElevator").setDouble(ElevatorConstants.kVHorizontalElevator);

        tuningTable.getEntry("kPHorizontalElevator").setDouble(ElevatorConstants.kPHorizontalElevator);
        tuningTable.getEntry("kIHorizontalElevator").setDouble(ElevatorConstants.kIHorizontalElevator);
        tuningTable.getEntry("kDHorizontalElevator").setDouble(ElevatorConstants.kDHorizontalElevator);
        tuningTable.getEntry("kMaxVelocityHorizontalElevator").setDouble(ElevatorConstants.kMaxVelocityHorizontalElevator);
        tuningTable.getEntry("kMaxAccelerationHorizontalElevator").setDouble(ElevatorConstants.kMaxAccelerationHorizontalElevator);

        resetHorizontalElevatorEncoder();
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public ProfiledPIDController getProfiledPIDController() {
        return profiledPIDController;
    }

    @Override
    public void periodic() {
        double nowTime = Timer.getFPGATimestamp();
        if (nowTime - lastUpdatedTS >= 5) {
            double tableKP = tuningTable.getValue("kPHorizontalElevator").getDouble();
            double tableKI = tuningTable.getValue("kIHorizontalElevator").getDouble();
            double tableKD = tuningTable.getValue("kDHorizontalElevator").getDouble();
            double tableMaxVelocity = tuningTable.getValue("kMaxVelocityHorizontalElevator").getDouble();
            double tableMaxAcceleration = tuningTable.getValue("kMaxAccelerationHorizontalElevator").getDouble();

            TrapezoidProfile.Constraints profiledPIDConstraints;
            try {
                Field constraintsField = ProfiledPIDController.class.getDeclaredField("m_constraints");
                constraintsField.setAccessible(true);
                profiledPIDConstraints = (TrapezoidProfile.Constraints) constraintsField.get(profiledPIDController);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new RuntimeException(e);
            }

            double tableKS = tuningTable.getValue("kSHorizontalElevator").getDouble();
            double tableKV = tuningTable.getValue("kVHorizontalElevator").getDouble();


            if (profiledPIDController.getP() != tableKP
                    || profiledPIDController.getI() != tableKI
                    || profiledPIDController.getD() != tableKD
                    || profiledPIDConstraints.maxVelocity != tableMaxVelocity
                    || profiledPIDConstraints.maxAcceleration != tableMaxAcceleration) {
                profiledPIDController.setP(tableKP);
                profiledPIDController.setI(tableKI);
                profiledPIDController.setD(tableKD);
                profiledPIDController.setConstraints(new TrapezoidProfile.Constraints(tableMaxVelocity, tableMaxAcceleration));
                profiledPIDController.reset(getHorizontalElevatorPosition());
            }

            if (feedforward.ks != tableKS || feedforward.kv != tableKV) {
                feedforward = new SimpleMotorFeedforward(tableKS, tableKV);
            }

            lastUpdatedTS = nowTime;
        }

        SmartDashboard.putNumber("Horizontal elevator kS", feedforward.ks);
        SmartDashboard.putNumber("Horizontal elevator kV", feedforward.kv);

        SmartDashboard.putNumber("Horizontal elevator kP", profiledPIDController.getP());
        SmartDashboard.putNumber("Horizontal elevator kI", profiledPIDController.getI());
        SmartDashboard.putNumber("Horizontal elevator kD", profiledPIDController.getD());
        SmartDashboard.putNumber("Horizontal elevator Position", getHorizontalElevatorPosition());
        SmartDashboard.putNumber("Horizontal elevator Ticks", hElevatorMotor.getSelectedSensorPosition());
    }

    public void setHorizontalElevatorVoltage(double voltage) {
        SmartDashboard.putNumber("hori voltage", voltage);
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
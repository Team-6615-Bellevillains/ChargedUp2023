package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final WPI_TalonSRX steerMotor;

    private final RelativeEncoder driveEncoder;

    private final Encoder steerEncoder;

    private final ProfiledPIDController steerPIDController;
    private final SimpleMotorFeedforward steerFeedforward;

    private final double absoluteEncoderOffsetCounts;

    private final int idx;
    private final double trueOffsetForRotation;

    public SwerveModule(int idx, int driverMotorID, int steerMotorID, int steerEncoderAPort, int steerEncoderBPort, boolean isDriveMotorReversed,
            double absoluteEncoderOffsetCounts) {

        this.idx = idx;

        this.absoluteEncoderOffsetCounts = absoluteEncoderOffsetCounts;

        this.driveMotor = new CANSparkMax(driverMotorID, MotorType.kBrushless);
        this.steerMotor = new WPI_TalonSRX(steerMotorID);

        // NEO plugged into Spark MAX
        this.driveEncoder = this.driveMotor.getEncoder();
        // Lamprey2 plugged into TalonSRX
        this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        this.steerMotor.configFeedbackNotContinuous(true, 0);

        this.driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
        this.driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter / 60);

        this.steerEncoder = new Encoder(steerEncoderAPort, steerEncoderBPort);
        this.steerEncoder.setDistancePerPulse(SwerveModuleConstants.kSteerEncoderRot2Rad);
        this.steerEncoder.reset();
        this.trueOffsetForRotation = getRotationFromAbsolute().getRadians();

        this.driveMotor.setInverted(isDriveMotorReversed);

        this.steerPIDController = new ProfiledPIDController(SwerveModuleConstants.kPTurning,
                SwerveModuleConstants.kITurning,
                SwerveModuleConstants.kDTurning, new TrapezoidProfile.Constraints(4 * 2 * Math.PI, 16 * 2 * Math.PI));
        this.steerPIDController.enableContinuousInput(0, 2 * Math.PI);

        this.steerFeedforward = new SimpleMotorFeedforward(SwerveModuleConstants.kSRotation, SwerveModuleConstants.kVRotation, SwerveModuleConstants.kARotation);

        this.driveEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDistance(),
                Rotation2d.fromRadians(getSwerveEncoder()));
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    private String appendIdx(String input) {
        return String.format("[%s] %s", idx, input);
    }

    public Rotation2d getRotationFromAbsolute() {
        /*
         * Magnets that are read on absolute encoders are read as 0 on a random M_point
         * when assembled. We want this to be at the zero point of the wheels
         * (henceforth Z_point), so we must apply an offset.
         */
        double offsetCounts = this.steerMotor.getSelectedSensorPosition() - absoluteEncoderOffsetCounts;
        /*
         * Because Z_point is calculated with M_point - offset, it will jump from
         * positive to negative when M_point goes from its max (1023) to its min (0).
         * (e.g. M_point = 1023, offset = 150, Z_point = 873 ->
         * M_point = 0, offset = 150, Z_point = -150).
         * To fix this jump, we add the maxmimum total counts (max + 1 -> 1024).
         * (e.g. M_point = 0, offset = 150, Z_point = -150 -> 1024 + -150 = 874)
         */
        if (offsetCounts < 0)
            offsetCounts = SwerveModuleConstants.maximumTotalCounts + offsetCounts;

        // Convert to radians using the formula ((amount/maxAmount) * 2 * π)
        double radians = (offsetCounts / SwerveModuleConstants.maximumTotalCounts) * 2 * Math.PI;

        // Convert to Rotation2d for use with other swerve drive methods
        return Rotation2d.fromRadians(radians);
    }

    public double getSwerveEncoder() {
        double encoderRadians = Math.IEEEremainder(steerEncoder.getDistance() + trueOffsetForRotation, 2 * Math.PI);
        if (encoderRadians < 0) {
            encoderRadians += 2 * Math.PI;
        }
        return encoderRadians;
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getSwerveEncoder()));
        driveMotor.set(state.speedMetersPerSecond /
                DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SmartDashboard.putNumber(appendIdx("State"), getSwerveEncoder());
        SmartDashboard.putNumber(appendIdx("Setpoint"), state.angle.getRadians());
        double steerPIDOut = steerPIDController.calculate(getSwerveEncoder(),
                state.angle.getRadians());
        SmartDashboard.putNumber(appendIdx("Steer PID Out"), steerPIDOut);
        double feedforward = steerFeedforward.calculate(steerPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber(appendIdx("Steer Feedforward"), steerPIDOut);
        steerMotor.setVoltage(steerPIDOut + feedforward);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

}

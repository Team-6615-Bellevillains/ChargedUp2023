// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.utils.TunableArmFeedforward;

public class GrabberSubsystem extends SubsystemBase {
    private CANSparkMax flipMotor;
    private RelativeEncoder flipEncoder;

    private TunableArmFeedforward grabberFeedforward = new TunableArmFeedforward("grabber", GrabberConstants.kSGrabber,
            GrabberConstants.kGGrabber, GrabberConstants.kVGrabber, GrabberConstants.kAGrabber);

    public GrabberSubsystem() {
        flipMotor = new CANSparkMax(GrabberConstants.kFlipMotorPort, MotorType.kBrushless);
        flipEncoder = flipMotor.getEncoder();
        flipEncoder.setPositionConversionFactor(GrabberConstants.kGrabberPositionConversionFactor);
        flipEncoder.setVelocityConversionFactor(GrabberConstants.kGrabberPositionConversionFactor / 60.0);
        flipEncoder.setPosition(GrabberConstants.kGrabberHighestPositionDegrees);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flip Encoder Position", flipEncoder.getPosition());
        SmartDashboard.putNumber("Flip Encoder Velocity", flipEncoder.getVelocity());
        SmartDashboard.putNumber("Flip Encoder Velocity (rads per second)", getFlipEncoderVelocityInRadsPerSec());
    }

    private double latestVoltage = 0;

    public double getLatestVoltage() {
        return latestVoltage;
    }

    public double calculateFeedforward(double positionRadians, double velocity) {
        return grabberFeedforward.getController().calculate(positionRadians, velocity);
    }

    public void setMotorVoltage(double voltage) {
        latestVoltage = voltage;
        SmartDashboard.putNumber("grabber voltage", voltage);

        SmartDashboard.putNumber("[GRAB] Velocity True (rads per second)", getFlipEncoderVelocityInRadsPerSec());
        flipMotor.setVoltage(voltage);
    }

    public void resetEncoderToHigh() {
        flipEncoder.setPosition(GrabberConstants.kGrabberHighestPositionDegrees);
    }

    public void resetEncoderToLow() {
        flipEncoder.setPosition(GrabberConstants.kGrabberLowestPositionDegrees);
    }

    public double getFlipEncoderPositionInDegrees() {
        // if (flipEncoder.getPosition() >=
        // GrabberConstants.kGrabberHighestPositionDegrees) {
        // resetEncoderToHigh();
        // }
        return flipEncoder.getPosition();
    }

    public double getFlipEncoderPositionInRads() {
        return Units.degreesToRadians(getFlipEncoderPositionInDegrees());
    }

    public double getFlipEncoderVelocityInDegsPerSec() {
        return flipEncoder.getVelocity();
    }

    public double getFlipEncoderVelocityInRadsPerSec() {
        return Units.degreesToRadians(getFlipEncoderVelocityInDegsPerSec());
    }

}

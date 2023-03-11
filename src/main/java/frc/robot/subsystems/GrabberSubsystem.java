// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  /** Creates a new GrabberSubsystem. */
  private Compressor compressor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  private CANSparkMax flipMotor;
  private RelativeEncoder flipEncoder;

  public GrabberSubsystem() {
    // Compressor and Solenoids
//    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
//    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kLeftSolenoidForwardChannel,
//        GrabberConstants.kLeftSolenoidReverseChannel);
//    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kRightSolenoidForwardChannel,
//        GrabberConstants.kRightSolenoidReverseChannel);

    // flipMotor
    flipMotor = new CANSparkMax(GrabberConstants.kFlipMotorPort, MotorType.kBrushless);
    flipEncoder = flipMotor.getEncoder();
    flipEncoder.setPositionConversionFactor(GrabberConstants.kGrabberPositionConversionFactor);
    flipEncoder.setVelocityConversionFactor(GrabberConstants.kGrabberPositionConversionFactor/60.0);
    flipEncoder.setPosition(GrabberConstants.kGrabberHighestPositionDegrees);
  }

  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Compressor Pressure", compressor.getCurrent());
    SmartDashboard.putNumber("Flip Encoder Position", flipEncoder.getPosition());
    SmartDashboard.putNumber("Flip Encoder Velocity", flipEncoder.getVelocity());
    SmartDashboard.putNumber("Flip Encoder Velocity (rads per second)", getFlipEncoderVelocityInRadsPerSec());
  }

//  public void setCompressorState(boolean on) {
//    if (on) {
//      compressor.enableDigital();
//    } else {
//      compressor.disable();
//    }
//  }

//  public void setSolenoidStates(DoubleSolenoid.Value state) {
//    leftSolenoid.set(state);
//    rightSolenoid.set(state);
//  }

  private double latestVoltage = 0;

  public double getLatestVoltage() {
    return latestVoltage;
  }

  public void setMotorVoltage(double voltage) {
    latestVoltage = voltage;
    SmartDashboard.putNumber("grabber voltage", voltage);
    flipMotor.setVoltage(voltage);
  }

  public double getFlipEncoderPositionInDegrees() {
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

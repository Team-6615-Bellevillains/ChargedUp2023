// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  /** Creates a new GrabberSubsystem. */
  private Compressor compressor;
  private Solenoid solenoid;
  private CANSparkMax flipMotor;
  private RelativeEncoder flipEncoder;

  public GrabberSubsystem() {
    // Compressor and Solenoids
//    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kSolenoidChannel);

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

  public void setSolenoidState(boolean on) {
    solenoid.set(on);
  }

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

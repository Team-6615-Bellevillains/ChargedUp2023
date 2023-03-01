// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {

  /** Creates a new GrabberSubsystem. */
  private Compressor compressor;
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  private CANSparkMax leftMotorRoller;
  private CANSparkMax rightMotorRoller;
  private WPI_TalonSRX flipMotor;

  public GrabberSubsystem() {
    // Find out ports later!!
    // Compressor and Solenoids
//    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
//    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kLeftSolenoidForwardChannel,
//        GrabberConstants.kLeftSolenoidReverseChannel);
//    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kRightSolenoidForwardChannel,
//        GrabberConstants.kRightSolenoidReverseChannel);

    // Roller Motors
    leftMotorRoller = new CANSparkMax(GrabberConstants.kLeftRollerMotorPort, MotorType.kBrushless);
    rightMotorRoller = new CANSparkMax(GrabberConstants.kRightRollerMotorPort, MotorType.kBrushless);

    leftMotorRoller.setInverted(true);

    // flipMotor
    flipMotor = new WPI_TalonSRX(GrabberConstants.kFlipMotorPort);
    flipMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    flipMotor.setSensorPhase(true);

    // Sets flipMotor's thresholds to prevent mechanism from breaking
    flipMotor.configReverseSoftLimitThreshold(GrabberConstants.kFlipReverseThreshold, 10);
    flipMotor.configForwardSoftLimitThreshold(GrabberConstants.kFlipReverseThreshold, 10);

    // Enables or Disables flipMotor's thresholds
    flipMotor.configReverseSoftLimitEnable(false);
    flipMotor.configForwardSoftLimitEnable(false);
  }

  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Compressor Pressure", compressor.getCurrent());
    SmartDashboard.putNumber("Flip Encoder Position", getFlipEncoderPosition());
  }

  public void setRollerSpeeds(double speed) {
    leftMotorRoller.set(speed);
    rightMotorRoller.set(speed);
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

  public void setFlipMotorSpeed(double speed) {
    flipMotor.set(speed);
  }

  public double getFlipEncoderPosition() {
    if (flipMotor.getSelectedSensorPosition() > Units.degreesToRadians(118)) {
      flipMotor.setSelectedSensorPosition(0);
    }
    return flipMotor.getSelectedSensorPosition() * GrabberConstants.flipRotationsToRadians/GrabberConstants.flipPulsesPerRevolution;
  }

  public void resetFlipEncoder() {
    flipMotor.setSelectedSensorPosition(0, 0, 10);
  }

}

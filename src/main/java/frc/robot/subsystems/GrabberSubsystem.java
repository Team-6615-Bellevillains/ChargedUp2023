// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

  private PIDController flipPIDController = new PIDController(GrabberConstants.kPFlip, GrabberConstants.kIFlip, GrabberConstants.kDFlip);

  public GrabberSubsystem() 
  {
    //Find out ports later!!
    //Compressor and Solenoids
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kLeftSolenoidForwardChannel, GrabberConstants.kLeftSolenoidReverseChannel);
    rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kRightSolenoidForwardChannel, GrabberConstants.kRightSolenoidReverseChannel);
    
    //Roller Motors
    leftMotorRoller = new CANSparkMax(GrabberConstants.kLeftRollerMotorPort, MotorType.kBrushless);
    rightMotorRoller = new CANSparkMax(GrabberConstants.kRightRollerMotorPort, MotorType.kBrushless);

    leftMotorRoller.setInverted(true);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Compressor Pressure", compressor.getCurrent()); 
  }

  public void setRollerSpeeds(double speed) {
    leftMotorRoller.set(speed);
    rightMotorRoller.set(speed);
  }

  public void setCompressorState(boolean on) {
    if (on) {
      compressor.enableDigital();
    } 
    else {
      compressor.disable();
    }
  }

  public void setSolenoidStates(DoubleSolenoid.Value state) {
    leftSolenoid.set(state);
    rightSolenoid.set(state);
  }

  
}

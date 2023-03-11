// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class PneumaticsSubsystem extends SubsystemBase {

  /** Creates a new GrabberSubsystem. */
  private Compressor compressor;
  private Solenoid solenoid;

  public PneumaticsSubsystem() {
//    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kSolenoidChannel);
  }

  @Override
  public void periodic() {
//    SmartDashboard.putNumber("Compressor Pressure", compressor.getCurrent());
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

}

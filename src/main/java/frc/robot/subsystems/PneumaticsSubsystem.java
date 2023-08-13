// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class PneumaticsSubsystem extends SubsystemBase {

    private Compressor compressor;
    private Solenoid solenoid;

    private boolean hasFilledSystem = false;

    public PneumaticsSubsystem() {
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kSolenoidChannel);
        setSolenoidState(false);
    }

    @Override
    public void periodic() {

        boolean isSystemFull = compressor.getPressureSwitchValue();

        SmartDashboard.putBoolean("Compressor/isEnabled", compressor.isEnabled());
        SmartDashboard.putBoolean("Compressor/isSystemFull", isSystemFull);

        if (!hasFilledSystem) {
            if (isSystemFull) {
                hasFilledSystem = true;
                compressor.disable();
            } else {
                if (!compressor.isEnabled()) {
                    compressor.enableDigital();
                }
            }
        }
    }

    public void reEnableCompressor() {
        this.hasFilledSystem = false;
    }

    public void setCompressorState(boolean on) {
        if (on) {
            // compressor.enableDigital();
        } else {
            // compressor.disable();
        }
    }

    public void setSolenoidState(boolean on) {
        solenoid.set(on);
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class HoldGrabberToShootingPosition extends CommandBase {
    // TODO: Fix Broken CMD
    private GrabberSubsystem grabberSubsystem;

    public HoldGrabberToShootingPosition(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        if (grabberSubsystem.getFlipEncoderPosition() <= 50) {
            grabberSubsystem.setMotorPercentage(-.1);
        } else {
            grabberSubsystem.setMotorPercentage(.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setMotorVoltage(-1.5);
    }

    @Override
    public boolean isFinished() {
        return grabberSubsystem.getFlipEncoderPosition() <= 33;
    }

}

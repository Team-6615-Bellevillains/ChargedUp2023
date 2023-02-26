// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

public class ManualGrabberFlipOutCmd extends CommandBase {
    private GrabberSubsystem grabberSubsystem;

    public ManualGrabberFlipOutCmd(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setFlipMotorSpeed(.4);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setFlipMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(grabberSubsystem.getFlipEncoderPosition() - Constants.GrabberConstants.grabberIntakeSetpoint) < Units.degreesToRadians(10);
    }

}

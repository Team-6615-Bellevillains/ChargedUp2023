// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GrabberSubsystem;

import java.util.ArrayList;
import java.util.function.Supplier;

public class GrabberJoystickControlCmd extends CommandBase {
    private GrabberSubsystem grabberSubsystem;
    private Supplier<Double> joystickPercentageFunction;

    public GrabberJoystickControlCmd(GrabberSubsystem grabberSubsystem, Supplier<Double> joystickPercentageFunction) {
        this.grabberSubsystem = grabberSubsystem;
        this.joystickPercentageFunction = joystickPercentageFunction;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("manual percentage", joystickPercentageFunction.get()/3);
        grabberSubsystem.setMotorPercentage(joystickPercentageFunction.get()/3);
//        System.out.println(String.format("(%s,%s)", this.grabberSubsystem.getFlipEncoderPosition(), grabberSubsystem.getLatestVoltage())); // plot position vs voltage
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setMotorPercentage(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(grabberSubsystem.getFlipEncoderPosition() - Constants.GrabberConstants.grabberInSetpoint) < Units.degreesToRadians(10);
    }

}

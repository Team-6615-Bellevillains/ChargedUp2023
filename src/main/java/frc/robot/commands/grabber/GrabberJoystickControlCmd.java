// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GrabberSubsystem;

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
        double joystickPower = MathUtil.applyDeadband(joystickPercentageFunction.get(), OIConstants.kOperatorControllerLeftYDeadband) * 1.5;

        SmartDashboard.putNumber("[GRAB] Velocity Desired (rads per second)", joystickPower);

        if (joystickPower == 0 && grabberSubsystem.getFlipEncoderPositionInRads() >= GrabberConstants.grabberRestThreshold) {
            grabberSubsystem.setMotorVoltage(0);
            grabberSubsystem.resetEncoderToHigh();
        } else {
            grabberSubsystem.setMotorVoltage(grabberSubsystem.calculateFeedforward(grabberSubsystem.getFlipEncoderPositionInRads(), joystickPower));
        }

    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setMotorVoltage(grabberSubsystem.calculateFeedforward(grabberSubsystem.getFlipEncoderPositionInRads(), 0));
    }


}

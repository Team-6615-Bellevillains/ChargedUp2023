// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GrabberSubsystem;

import java.util.function.Supplier;

public class GrabberJoystickControlCmd extends CommandBase {
    private GrabberSubsystem grabberSubsystem;
    private Supplier<Double> joystickPercentageFunction;
    private ArmFeedforward grabberFeedforward = new ArmFeedforward(GrabberConstants.kSGrabber,GrabberConstants.kGGrabber,GrabberConstants.kVGrabber, GrabberConstants.kAGrabber);


    public GrabberJoystickControlCmd(GrabberSubsystem grabberSubsystem, Supplier<Double> joystickPercentageFunction) {
        this.grabberSubsystem = grabberSubsystem;
        this.joystickPercentageFunction = joystickPercentageFunction;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        double joystickPower = MathUtil.applyDeadband(joystickPercentageFunction.get(), OIConstants.kDefaultJoystickDeadband) / 2;
        grabberSubsystem.setMotorVoltage(grabberFeedforward.calculate(grabberSubsystem.getFlipEncoderPositionInRads(), joystickPower));
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setMotorVoltage(grabberFeedforward.calculate(grabberSubsystem.getFlipEncoderPositionInRads(), 0));
    }


}

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ClampGrabberCmd extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    public ClampGrabberCmd(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setSolenoidState(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setSolenoidState(DoubleSolenoid.Value.kOff);
    }

}
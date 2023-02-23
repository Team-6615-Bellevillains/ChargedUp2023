package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class OpenGrabberCmd extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    public OpenGrabberCmd(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        
        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setSolenoidStates(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setSolenoidStates(DoubleSolenoid.Value.kOff);
    }
    
}
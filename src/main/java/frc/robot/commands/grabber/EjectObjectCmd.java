package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class EjectObjectCmd extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    public EjectObjectCmd(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setRollerSpeeds(.5);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setRollerSpeeds(0);
    }
    
}
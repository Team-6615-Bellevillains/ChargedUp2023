package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class OpenGrabberCmd extends CommandBase {

    private PneumaticsSubsystem pneumaticsSubsystem;

    public OpenGrabberCmd(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void execute() {
        pneumaticsSubsystem.setSolenoidState(false);
    }

}
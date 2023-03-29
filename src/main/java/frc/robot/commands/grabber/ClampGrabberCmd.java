package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ClampGrabberCmd extends CommandBase {

    private PneumaticsSubsystem pneumaticsSubsystem;
    private boolean finished;

    public ClampGrabberCmd(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;

        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        this.finished = false;
    }

    @Override
    public void execute() {
        pneumaticsSubsystem.setSolenoidState(false);
        this.finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ReEnableCompressorCmd extends CommandBase {

    private PneumaticsSubsystem pneumaticsSubsystem;

    public ReEnableCompressorCmd(PneumaticsSubsystem pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;

        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.reEnableCompressor();
    }

}

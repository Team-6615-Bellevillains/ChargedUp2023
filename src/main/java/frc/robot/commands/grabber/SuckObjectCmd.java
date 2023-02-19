package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class SuckObjectCmd extends CommandBase {

    private GrabberSubsystem grabberSubsystem;
    private double durationMS, initialTS;

    public SuckObjectCmd(GrabberSubsystem grabberSubsystem) {
        this(grabberSubsystem, 0);
    }

    public SuckObjectCmd(GrabberSubsystem grabberSubsystem, double durationMS) {
        this.grabberSubsystem = grabberSubsystem;
        this.durationMS = durationMS;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        this.initialTS = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        grabberSubsystem.setRollerSpeeds(-0.5);
    }

    @Override
    public boolean isFinished() {
        return durationMS > 0 ? (Timer.getFPGATimestamp() - initialTS >= durationMS) : false;
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setRollerSpeeds(0);
    }

}
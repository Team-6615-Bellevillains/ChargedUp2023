package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class ShootPieceCmd extends CommandBase {

    private GrabberSubsystem grabberSubsystem;

    public ShootPieceCmd(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;

        addRequirements(grabberSubsystem);
    }

    @Override
    public void execute() {
        grabberSubsystem.setRollerSpeeds(.2);
    }

    @Override
    public void end(boolean interrupted) {
        grabberSubsystem.setRollerSpeeds(0);
    }

}
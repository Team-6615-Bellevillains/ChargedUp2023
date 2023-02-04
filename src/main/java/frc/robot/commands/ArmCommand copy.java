package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;



public class ArmCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmCommand (ArmSubsystem armSub){
        this.armSubsystem = armSub;

        addRequirements(armSub);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (armSubsystem.armEncoderDistance() < ArmConstants.armEncoderTopValue){
            armSubsystem.armUp();
        } else {
            armSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

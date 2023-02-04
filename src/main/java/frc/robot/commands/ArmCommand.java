package frc.robot.commands;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Constants;



public class ArmCommand extends CommandBase {

    private ArmSubsystem armSybsystem;

    public ArmCommand (ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (armSubsystem.getDistance < ArmConstants.armEncoderTopValue){
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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;



public class ElevatorUp extends CommandBase {

    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorUp (ElevatorSubsystem elevatorSub){
        this.elevatorSubsystem = elevatorSub;

        addRequirements(elevatorSub);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (elevatorSubsystem.vElevatorEncoderDistance() < ElevatorConstants.vElevatorEncoderTopValue){
            elevatorSubsystem.vElevatorUp(0.25);
        } else {
            elevatorSubsystem.stopVElevator();
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

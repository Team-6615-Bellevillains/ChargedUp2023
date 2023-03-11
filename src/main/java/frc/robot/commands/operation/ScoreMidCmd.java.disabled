package frc.robot.commands.operation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.elevator.HorizontalElevatorOutCmd;
import frc.robot.commands.elevator.VerticalElevatorToSetpointCmd;
import frc.robot.commands.grabber.HoldGrabberToShootingPosition;
import frc.robot.subsystems.HorizontalElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class ScoreMidCmd extends CommandBase {
    private VerticalElevatorToSetpointCmd verticalElevatorToSetpointCmd;
    private HorizontalElevatorOutCmd horizontalElevatorOutCmd;
    private HoldGrabberToShootingPosition holdGrabberToShootingPosition;

    private Command routine;

    public ScoreMidCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem, VerticalElevatorSubsystem verticalElevatorSubsystem, GrabberSubsystem grabberSubsystem) {
        verticalElevatorToSetpointCmd = new VerticalElevatorToSetpointCmd(verticalElevatorSubsystem, 2);
        horizontalElevatorOutCmd = new HorizontalElevatorOutCmd(horizontalElevatorSubsystem);
        holdGrabberToShootingPosition = new HoldGrabberToShootingPosition(grabberSubsystem);

        addRequirements(horizontalElevatorSubsystem, grabberSubsystem);
    }

    @Override
    public void initialize() {
        routine = holdGrabberToShootingPosition
                .andThen(horizontalElevatorOutCmd);

        routine.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Ending score mid", Timer.getFPGATimestamp());

        routine.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return routine.isFinished();
    }

}

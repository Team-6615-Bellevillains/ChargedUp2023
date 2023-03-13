package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {

    private WPI_TalonSRX leftMotorRoller;
    private WPI_TalonSRX rightMotorRoller;

    public RollerSubsystem() {
        leftMotorRoller = new WPI_TalonSRX(Constants.GrabberConstants.kLeftRollerMotorPort);
        rightMotorRoller = new WPI_TalonSRX(Constants.GrabberConstants.kRightRollerMotorPort);

        leftMotorRoller.setInverted(true);
    }

    public void setRollerSpeedPercentage(double speedPercentage) {
        leftMotorRoller.set(speedPercentage);
        rightMotorRoller.set(speedPercentage);
    }

}

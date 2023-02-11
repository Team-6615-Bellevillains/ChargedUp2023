package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

private final WPI_TalonSRX vElevatorMotor1;
private final WPI_TalonSRX vElevatorMotor2;
private final WPI_TalonSRX hElevatorMotor;

public ElevatorSubsystem() {

    this.vElevatorMotor1 = new WPI_TalonSRX(ElevatorConstants.vElevatorMotor1); //only one encoder for the two motors
    this.vElevatorMotor2 = new WPI_TalonSRX(ElevatorConstants.vElevatorMotor2);
    this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.hElevatorMotor);

    this.vElevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    this.hElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
}

//VERITCAL
public double vElevatorEncoderPosition()
{
 return vElevatorMotor1.getSelectedSensorPosition();   
}

public void vElevatorEncoderReset()
{
    vElevatorMotor1.setSelectedSensorPosition(0);
}

public void vElevatorSpeed(double speed)
{
    vElevatorMotor1.set(speed);
    vElevatorMotor2.set(speed);
}
/*public void vElevatorDown(){

    vElevatorMotor1.set(.25);
    vElevatorMotor2.set(.25);
}

public void vElevatorUp(double speed) {
    vElevatorMotor1.set(speed);
    vElevatorMotor2.set(speed);
}

public void stopVElevator(){
    vElevatorMotor1.set(0);
    vElevatorMotor2.set(0);
}*/

//HORRIZONTAL

public double hElevatorEncoderPosition()
{
 return hElevatorMotor.getSelectedSensorPosition();   
}

public void hElevatorEncoderReset()
{
    hElevatorMotor.setSelectedSensorPosition(0);
}

public void hElevatorSpeed(double speed)
{
    hElevatorMotor.set(speed);
}

/*public void hElevatorOut(){
    hElevatorMotor.set(.25);
}

public void hElevatorIn(){
    hElevatorMotor.set(-.25);
}

public void stopHElevator(){
    hElevatorMotor.set(0);
}*/

}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase{

private final WPI_TalonSRX vElevatorMotor1;
private final WPI_TalonSRX vElevatorMotor2;
private final Encoder vElevatorEncoder;
private final WPI_TalonSRX hElevatorMotor;
private final Encoder hElevatorEncoder;
private final PIDController vPIDController; // v = vertical
private final PIDController hPIDController; // h = horizontal

public ElevatorSubsystem(WPI_TalonSRX vElevatorMotor, Encoder vElevatorEncoder, WPI_TalonSRX hElevatorMotor, Encoder hElevatorEncoder, PIDController vPIDController, PIDController hPidController) {

    this.vElevatorMotor1 = new WPI_TalonSRX(ElevatorConstants.vElevatorMotor1);
    this.vElevatorMotor2 = new WPI_TalonSRX(ElevatorConstants.vElevatorMotor2);
    this.hElevatorMotor = new WPI_TalonSRX(ElevatorConstants.hElevatorMotor);

    this.vElevatorEncoder = new Encoder(ElevatorConstants.vElevatorEncoderPort1,ElevatorConstants.vElevatorEncoderPort2);
    this.vElevatorEncoder.reset();
    
    this.hElevatorEncoder = new Encoder(ElevatorConstants.hElevatorEncoderPort1,ElevatorConstants.hElevatorEncoderPort2);
    this.hElevatorEncoder.reset();
   // this.elevatorEncoder.setDistancePerPulse(ElevatorConstants.elevatorEncoderRateInInches); don't know yet
   this.vPIDController = new PIDController(ElevatorConstants.kPVertical, ElevatorConstants.kIVertical, ElevatorConstants.kDVertical);
   this.hPIDController = new PIDController(ElevatorConstants.kPHorizontal, ElevatorConstants.kIHorizontal, ElevatorConstants.kDHorizontal);

   this.vElevatorEncoder.setDistancePerPulse(ElevatorConstants.vElevatorEncoderDPR);


}

//VERITCAL
public double vElevatorEncoderDistance(){
    return vElevatorEncoder.getDistance();
}

public void vElevatorDown(){

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
}

//HORRIZONTAL
public double hElevatorEncoderDistance(){
    return hElevatorEncoder.getDistance();
}

public void hElevatorOut(){
    hElevatorMotor.set(.25);
}

public void hElevatorIn(){
    hElevatorMotor.set(-.25);
}

public void stopHElevator(){
    hElevatorMotor.set(0);
    
}

}
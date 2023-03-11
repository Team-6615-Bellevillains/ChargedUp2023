package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class TunablePIDController {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunablePIDController> controllers = new ArrayList<>();

    private final String identifier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();
    private double lastMeasurement;

    private PIDController pidController;

    public TunablePIDController(String identifier, double Kp, double Ki, double Kd) {
        pidController = new PIDController(Kp, Ki, Kd);

        this.identifier = identifier;

        tuningTable.getEntry(appendIdentifier("Kp")).setDouble(Kp);
        tuningTable.getEntry(appendIdentifier("Ki")).setDouble(Ki);
        tuningTable.getEntry(appendIdentifier("Kd")).setDouble(Kd);

        controllers.add(this);
    }

    public PIDController getController() {
        return pidController;
    }

    public static void updateControllersIfOutdated() {
        for (TunablePIDController controller : controllers) {
            controller.updateConstantsIfOutdated();
        }
    }

    public double calculateAndUpdateLastMeasurement(double currentMeasurement) {
        lastMeasurement = currentMeasurement;
        return pidController.calculate(currentMeasurement);
    }

    public String appendIdentifier(String input) {
        return "pid" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval) return;

        double tableKP = tuningTable.getValue(appendIdentifier("Kp")).getDouble();
        double tableKI = tuningTable.getValue(appendIdentifier("Ki")).getDouble();
        double tableKD = tuningTable.getValue(appendIdentifier("Kd")).getDouble();

        if (pidController.getP() != tableKP
                || pidController.getI() != tableKI
                || pidController.getD() != tableKD) {
            pidController.setP(tableKP);
            pidController.setI(tableKI);
            pidController.setD(tableKD);
            pidController.reset();
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }
}

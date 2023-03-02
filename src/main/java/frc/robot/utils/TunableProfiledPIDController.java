package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.lang.reflect.Field;
import java.util.ArrayList;

public class TunableProfiledPIDController extends ProfiledPIDController {

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private final static double updateInterval = 3;
    private final static ArrayList<TunableProfiledPIDController> controllers = new ArrayList<>();

    private final Field m_constraintsField;
    private final String identifier;

    private double lastUpdatedTS = Timer.getFPGATimestamp();
    private double lastMeasurement;


    public TunableProfiledPIDController(String identifier, double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        super(Kp, Ki, Kd, constraints);

        this.identifier = identifier;

        tuningTable.getEntry(appendIdentifier("Kp")).setDouble(Kp);
        tuningTable.getEntry(appendIdentifier("Ki")).setDouble(Ki);
        tuningTable.getEntry(appendIdentifier("Kd")).setDouble(Kd);
        tuningTable.getEntry(appendIdentifier("kMaxVelo")).setDouble(constraints.maxVelocity);
        tuningTable.getEntry(appendIdentifier("kMaxAccel")).setDouble(constraints.maxAcceleration);

        try {
            m_constraintsField = ProfiledPIDController.class.getDeclaredField("m_constraints");
            m_constraintsField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        controllers.add(this);
    }

    public static void updateControllersIfOutdated() {
        for (TunableProfiledPIDController controller : controllers) {
            controller.updateConstantsIfOutdated();
        }
    }

    @Override
    public double calculate(double currentMeasurement) {
        lastMeasurement = currentMeasurement;
        return super.calculate(currentMeasurement);
    }

    public String appendIdentifier(String input) {
        return "pid" + input + identifier;
    }

    public void updateConstantsIfOutdated() {
        if (Timer.getFPGATimestamp() - lastUpdatedTS < updateInterval) return;

        double tableKP = tuningTable.getValue(appendIdentifier("Kp")).getDouble();
        double tableKI = tuningTable.getValue(appendIdentifier("Ki")).getDouble();
        double tableKD = tuningTable.getValue(appendIdentifier("Kd")).getDouble();
        double tableMaxVelocity = tuningTable.getValue(appendIdentifier("kMaxVelo")).getDouble();
        double tableMaxAcceleration = tuningTable.getValue(appendIdentifier("kMaxAccel")).getDouble();

        TrapezoidProfile.Constraints currConstraints = getConstraints();

        if (super.getP() != tableKP
                || super.getI() != tableKI
                || super.getD() != tableKD
                || currConstraints.maxVelocity != tableMaxVelocity
                || currConstraints.maxAcceleration != tableMaxAcceleration) {
            super.setP(tableKP);
            super.setI(tableKI);
            super.setD(tableKD);
            super.setConstraints(new TrapezoidProfile.Constraints(tableMaxVelocity, tableMaxAcceleration));
            super.reset(lastMeasurement);
        }

        lastUpdatedTS = Timer.getFPGATimestamp();
    }

    public TrapezoidProfile.Constraints getConstraints() {
        try {
            return (TrapezoidProfile.Constraints) m_constraintsField.get(this);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }
}

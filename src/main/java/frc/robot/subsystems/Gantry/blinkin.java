package frc.robot.subsystems.Gantry;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class blinkin extends SubsystemBase {

    private static blinkin instance = null;

    private final Spark blinkin;

    private blinkin() {
        blinkin = new Spark(2);
    }

    public static blinkin getInstance() {
        if (instance == null) {
            instance = new blinkin();
        }
        return instance;
    }

    public Command lights(Double value) {
        return run(() -> blinkin.set(value));
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}

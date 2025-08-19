package frc.robot.subsystems.Gantry;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Gantry.Elevator;


public class Hinge extends SubsystemBase{
    private int angle = 0;

    private final Elevator elevator = new Elevator();
    //private final RobotContainer robotContainer = new RobotContainer();

    final TalonFX m_Hinge = new TalonFX(15, "*");
    // create a Motion Magic Expo request, voltage output
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    // set target position to 100 rotations


    public Hinge(){

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        

        // set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

        m_Hinge.getConfigurator().apply(talonFXConfigs);
        

    }

    public void setTargetAngle(int Targetangle) {
        angle=Targetangle;
    }

    public Command Setpoints(int value){
        //System.out.println("setpoint set");
        //System.out.println(value);

        return run(() -> m_Hinge.setControl(m_request.withPosition(Units.degreesToRotations(value)*81)));
    }

    @Override
    public void periodic() {
         
        if (
            (((elevator.getElevatorHeight())[1] < 275) && ((elevator.getElevatorHeight())[1] > 5)) && !(((elevator.getElevatorHeight())[1] < 126.5) && ((elevator.getElevatorHeight())[1] > 120.5)) && !(((elevator.getElevatorHeight())[1] < 47.5) && ((elevator.getElevatorHeight())[1] > 42.5))
           ) 
           {
            m_Hinge.setControl(m_request.withPosition(Units.degreesToRotations(20)*81));
            //System.out.println("at safety");
            //System.out.println(angle);

        } else {
            m_Hinge.setControl(m_request.withPosition(Units.degreesToRotations(angle)*81));
            
            //System.out.println(angle);
        }
            
    }

    @Override
    public void simulationPeriodic() {
    }
}

    




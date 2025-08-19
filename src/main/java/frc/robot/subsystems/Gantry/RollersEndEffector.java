package frc.robot.subsystems.Gantry;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.*;

import java.util.concurrent.TimeUnit;

import org.dyn4j.collision.narrowphase.FallbackCondition;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.*;
import com.revrobotics.spark.*;

import au.grapplerobotics.LaserCan;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollersEndEffector extends SubsystemBase{
    
    final TalonFX m_RollerEndEffector = new TalonFX(22,"*");
    //final Spark blinkin = new Spark(2);
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    final MotionMagicExpoVoltage m_requestP = new MotionMagicExpoVoltage(0);

    // DigitalInput limitSwitch = new DigitalInput(0);
    
    //private Rev2mDistanceSensor distOnBoard;
    
    public boolean ejecting = false;
    //public boolean intaking = false;

    //private Spark blinkin;
    // set target position to 100 rotations
    DigitalInput m_endeffector1 = new DigitalInput(1);
    DigitalInput m_endeffector2 = new DigitalInput(2);



    public RollersEndEffector(){

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.33; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 8000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        m_RollerEndEffector.getConfigurator().apply(talonFXConfigs);
          // Initializes a DigitalInput on DIO 0
        
        //final Spark blinkin = new Spark(2);
        
        
    }

    public Command rollerendeffector(Double value){  

        // if the intake is enabled and either sensor detects a coral, set mode to ejection.
        if ((value != 0) && (!m_endeffector1.get() || !m_endeffector2.get())) {
            ejecting = true;
            return run(() -> m_RollerEndEffector.setControl(m_request.withVelocity(value)));
        
        // if the intake is disabled or there are no coral detected, set mode to neutral.
        } else {
            ejecting = false;
            return run(() -> m_RollerEndEffector.setControl(m_request.withVelocity(value))); 
        }

    }

    @Override
    public void periodic() {

        if (ejecting == false) {
            // if there is a coral between the first and second sensor, stop.
            if ((!m_endeffector1.get()) && (m_endeffector2.get())) {
                m_RollerEndEffector.setControl(m_request.withVelocity(0));
            }

            // if there is a coral after the second sensor, wind it back in.
            if (!m_endeffector2.get()) {
                m_RollerEndEffector.setControl(m_request.withVelocity(1));
            }
        } else {
            // if the mode is set to ejecting but neither sensor detects a coral, then turn it off.
            if ((m_endeffector1.get()) && (m_endeffector2.get())) {
                ejecting = false;
                m_RollerEndEffector.setControl(m_request.withVelocity(0));
            }
        }

    }
                    

    @Override
    public void simulationPeriodic() {
        
    }

    



}
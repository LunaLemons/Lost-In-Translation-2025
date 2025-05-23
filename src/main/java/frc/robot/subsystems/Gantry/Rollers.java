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

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import java.util.concurrent.TimeUnit;

import org.dyn4j.collision.narrowphase.FallbackCondition;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.*;
import com.revrobotics.spark.*;


// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase{
    
    final TalonFX m_Roller = new TalonFX(16,"*");
    //final Spark blinkin = new Spark(2);
    // create a Motion Magic Velocity request, voltage output
    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    final MotionMagicExpoVoltage m_requestP = new MotionMagicExpoVoltage(0);

    // DigitalInput limitSwitch = new DigitalInput(0);
    
    //private Rev2mDistanceSensor distOnBoard;
    
    public boolean sensor = true;
    private LaserCan lc;
    //private Spark blinkin;
    // set target position to 100 rotations


    public Rollers(){

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

        m_Roller.getConfigurator().apply(talonFXConfigs);
        lc = new LaserCan(19);
        //final Spark blinkin = new Spark(2);
        
        
    }

    public Command roller(Double value){
        LaserCan.Measurement measurement = lc.getMeasurement();
        

        if (value < 0) {
            if (value > -15) {
                System.out.println("not error");
                if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                    if (measurement.distance_mm > 20) { // If sensor false
                        sensor = false;
                        return run(() -> m_Roller.setControl(m_request.withVelocity(value)));
                        
                    } else {
                        return run(() -> m_Roller.setControl(m_request.withVelocity(value)));
                    }
                } else {
                    return run(() -> m_Roller.setControl(m_request.withVelocity(value))); 
                }
            } else {
                return run(() -> m_Roller.setControl(m_request.withVelocity(value)));
                
            }


            



        } else {
            System.out.println("error");
            return run(() -> m_Roller.setControl(m_request.withVelocity(value)));
        }
 
        // this doesn't work
    }

    @Override
    public void periodic() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        } else {
            //System.out.println("lasercan sad :(");
            //System.out.println(measurement.distance_mm);
            //System.out.println(sensor);
        }
        //System.out.println(measurement.distance_mm);
        //System.out.println(sensor);

        if (sensor == false) {
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm > 20) {

            //blinkin.set(0.25);
            sensor = false;
        
           } else {

            //blinkin.set(0.05);

            m_Roller.setControl(m_request.withVelocity(0));
           }
            }
            }
        }

    @Override
    public void simulationPeriodic() {
        
    }

    



}
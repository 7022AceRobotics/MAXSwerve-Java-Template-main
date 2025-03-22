// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubystem. */
  private final SparkMax m_shooting_motor;
  private final AnalogPotentiometer m_sensor;
  private final SparkMaxConfig m_config;
  private double sensor_values = 0;
    
    public ShooterSubsystem() {
      m_shooting_motor = new SparkMax(11, MotorType.kBrushless);
      m_sensor  = new AnalogPotentiometer(1);
      m_config = new SparkMaxConfig();
  
      m_config.smartCurrentLimit(20);
  
      m_shooting_motor.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler ru
    }
  
    public void shoot(double speed){
      // if (!(sensor_values > 0.4 && m_sensor.get() < 0.2)){
        m_shooting_motor.set(-speed);
        // sensor_values = m_sensor.get();
    //}
      }
    
  

  public void stop(){
    m_shooting_motor.set(0);
  }
}

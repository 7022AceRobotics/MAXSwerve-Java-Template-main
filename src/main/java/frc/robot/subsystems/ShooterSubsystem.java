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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubystem. */
  private final SparkMax m_shooting_motor;
  private final AnalogPotentiometer m_sensor;
  private final SparkMaxConfig m_config;
  private double sensor_average = 0;
  private double sensor_iters = 0;
    
    public ShooterSubsystem() {
      m_shooting_motor = new SparkMax(11, MotorType.kBrushless);
      m_sensor  = new AnalogPotentiometer(0);
      m_config = new SparkMaxConfig();
  
      m_config.smartCurrentLimit(20, 1000, 500);
  
      m_shooting_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler ru
      SmartDashboard.putNumber("Get sensor val",m_sensor.get());
    }
  
    public void shoot(double speed){
      sensor_iters += 1;
      sensor_average += m_shooting_motor.getBusVoltage();
      if (m_shooting_motor.getBusVoltage() >= sensor_average/sensor_iters - 0.75){
        m_shooting_motor.set(-speed);
    }
      }
    
  

  public void stop(){
    m_shooting_motor.set(0);
    sensor_iters = 0;
      sensor_average = 0;
  }

  @Logged(name="current")
  public double getCurrent(){
    return m_shooting_motor.getOutputCurrent();
  }
  @Logged(name="voltage")
  public double getVoltage(){
    return m_shooting_motor.getBusVoltage();
  }
  @Logged(name="velo")
  public double getVelocity(){
    return m_shooting_motor.getEncoder().getVelocity();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.NeoMotorConstants;

public class AlgaeCollectorSubsystem extends SubsystemBase {
    private final SparkMax m_pull_motor;
    private final SparkMax m_pivot_motor;
    //private final SparkClosedLoopController algae_pid_controller;
    //private final RelativeEncoder algae_encoder;
    private final AnalogPotentiometer sensor;
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    
  /** Creates a new AlgaeSubsystem. */
  public AlgaeCollectorSubsystem() { 
    m_pull_motor = new SparkMax(Constants.AlgaeConstants.kAlgaePullMotorPort, MotorType.kBrushless);
    m_pivot_motor = new SparkMax(Constants.AlgaeConstants.kAlgaePivotMotorPort, MotorType.kBrushless);
    sensor = new AnalogPotentiometer(AlgaeConstants.kAlgaeLightSensorPort);
    m_controller = m_pivot_motor.getClosedLoopController();
    // CHANGE TO ABSOLUTE ENCODER
    m_encoder = m_pivot_motor.getEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit((int) NeoMotorConstants.kSmartCurrentLimitConstant);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(AlgaeConstants.kP)
        .i(AlgaeConstants.kI)
        .d(AlgaeConstants.kD)
        .outputRange(AlgaeConstants.kMinOutput, AlgaeConstants.kMaxOutput);
    config.closedLoop.maxMotion
        .maxVelocity(AlgaeConstants.kMaxVel)
        .maxAcceleration(AlgaeConstants.kMaxAccel)
        .allowedClosedLoopError(AlgaeConstants.kAllowedErr);

    m_pivot_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(0);
    
  }

  public void SetAlgaePosition(double targetposition){
    m_controller.setReference(targetposition, SparkBase.ControlType.kMAXMotionPositionControl);    
  }
  public void PullSetSpeed(double speed){
     m_pull_motor.set(speed);
  }
  public double AlgaeSensor(){
    return sensor.get();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

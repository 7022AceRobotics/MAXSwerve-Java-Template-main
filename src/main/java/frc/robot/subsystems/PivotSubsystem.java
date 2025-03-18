// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final SparkMax m_pivot_motor;
  private final SparkClosedLoopController m_controller;
  private final RelativeEncoder m_encoder;
  private final SparkBaseConfig m_config;

  public final ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
  //public final SimpleWidget pos = tab.add("Pos", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1))
  public final SimpleWidget pos = tab.add("Pos", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget kP = tab.add("Kp Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget kI = tab.add("Ki Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget kD = tab.add("Kd Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget min = tab.add("min Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget max = tab.add("max Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget speed = tab.add("speed Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget accel = tab.add("accel Piv", 0).withWidget(BuiltInWidgets.kTextView);
  public final SimpleWidget err = tab.add("err Piv", 0).withWidget(BuiltInWidgets.kTextView);


  public PivotSubsystem() {
    this.m_pivot_motor = new SparkMax(12, MotorType.kBrushless);
    this.m_controller = m_pivot_motor.getClosedLoopController();
    this.m_encoder = m_pivot_motor.getEncoder();
    this.m_config = new SparkMaxConfig();

    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-1, 1);
    m_config.closedLoop.maxMotion
        .maxVelocity(5000)
        .maxAcceleration(2500)
        .allowedClosedLoopError(0.05);

    m_pivot_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //m_controller.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl); 

    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void goToPosition(double position){
    m_controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
  }
}

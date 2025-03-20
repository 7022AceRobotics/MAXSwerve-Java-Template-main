// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final SparkMax m_pivot_motor;
  private final SparkClosedLoopController m_controller;
  private final RelativeEncoder m_encoder;
  private SparkBaseConfig m_config;
  
    public PivotSubsystem() {
      this.m_pivot_motor = new SparkMax(PivotConstants.PivotMotorPort, MotorType.kBrushless);
      this.m_controller = m_pivot_motor.getClosedLoopController();
      this.m_encoder = m_pivot_motor.getEncoder();
      this.m_config = new SparkMaxConfig();

      m_config.smartCurrentLimit((int) NeoMotorConstants.kSmartCurrentLimitConstant);
  
      m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(PivotConstants.kP)
          .i(PivotConstants.kI)
          .d(PivotConstants.kD)
          .outputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput)
          .p(1, ClosedLoopSlot.kSlot1)
          .i(0, ClosedLoopSlot.kSlot1)
          .d(0, ClosedLoopSlot.kSlot1)
          .velocityFF(0, ClosedLoopSlot.kSlot1);
      m_config.closedLoop.maxMotion
          .maxVelocity(PivotConstants.kMaxVel)
          .maxAcceleration(PivotConstants.kMaxAccel)
          .allowedClosedLoopError(PivotConstants.kAllowedErr);
  
      m_pivot_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      m_encoder.setPosition(0);
      SmartDashboard.putNumber("POS2", 0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("encoder val", -m_encoder.getPosition() / PivotConstants.kGearRatio * 360);
    }
  
    public void goToPosition(double position){
        m_controller.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }
  
    public double rotationPerDegree(double degree){
      // Converts negative to positive becuase neo is facing opposite direction, and I don't want to use the rev hardware client/
      return -degree / 360 * PivotConstants.kGearRatio;
    }
  
    public void changePIDValues(){

      // FUNCTION USED FOR TESTING.
  
      m_config = new SparkMaxConfig();
      m_config.smartCurrentLimit((int) NeoMotorConstants.kSmartCurrentLimitConstant);
      m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("kP", 1))
        .i(SmartDashboard.getNumber("ki", 0))
        .d(SmartDashboard.getNumber("kD", 0))
        .outputRange(-1, 1)
        .p(1, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(0, ClosedLoopSlot.kSlot1);
    m_config.closedLoop.maxMotion
        .maxVelocity(1000)
        .maxAcceleration(500)
        .allowedClosedLoopError(0.05);

    m_pivot_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder.setPosition(0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

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

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
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
  public final RelativeEncoder m_encoder;
  private SparkBaseConfig m_config;
  private ArmFeedforward ff_val = new ArmFeedforward(0.01,0.09,2.92,0);

  @Logged(name = "TargetPosition")
  private double m_targetPosition;
  
    public PivotSubsystem() {
      this.m_pivot_motor = new SparkMax(PivotConstants.PivotMotorPort, MotorType.kBrushless);
      this.m_controller = m_pivot_motor.getClosedLoopController();
      this.m_encoder = m_pivot_motor.getEncoder();
      this.m_config = new SparkMaxConfig();

      //m_config.smartCurrentLimit(20);
      m_config.smartCurrentLimit(40, 5700, 4000);
  
      m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(PivotConstants.kP)
          .i(PivotConstants.kI)
          .d(PivotConstants.kD)
          .outputRange(-0.3, 0.3)
          .p(1, ClosedLoopSlot.kSlot1)
          .i(0, ClosedLoopSlot.kSlot1)
          .d(0, ClosedLoopSlot.kSlot1)
          .velocityFF(0.1, ClosedLoopSlot.kSlot1);

      m_config.closedLoop.maxMotion
          .maxVelocity(PivotConstants.kMaxVel)
          .maxAcceleration(PivotConstants.kMaxAccel)
          .allowedClosedLoopError(PivotConstants.kAllowedErr);
  
      m_pivot_motor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      m_encoder.setPosition(0);
      SmartDashboard.putNumber("PIV POS", 0);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("encoder val", -m_encoder.getPosition());
    }
  
    public void goToPosition(double position){
      SmartDashboard.putNumber("BBBBBBBBBBBB", 5);
      m_targetPosition = position;
      m_controller.setReference(
        position, 
        SparkBase.ControlType.kMAXMotionPositionControl);
        // ClosedLoopSlot.kSlot0, 
        // ff_val.calculate(
        //   Units.degreesToRadians(position - 90), 
        //   Units.degreesToRadians(
        //     degreePerRotation(
        //       m_encoder.getVelocity()
        //       ))));
    }

    public void JoystickPivotPosition(Supplier<Double> targetSpeed){
    m_targetPosition = Math.min(0, m_targetPosition + MathUtil.applyDeadband(targetSpeed.get(), 0.1)*0.1);
    m_controller.setReference(m_targetPosition, ControlType.kPosition);
  }
  
    public double rotationPerDegree(double degree){
      // Converts negative to positive becuase neo is facing opposite direction, and I don't want to use the rev hardware client/
      return -degree / 360 * PivotConstants.kGearRatio;
    }

    public double degreePerRotation(double rotations){
      // Converts negative to positive becuase neo is facing opposite direction, and I don't want to use the rev hardware client/
      return -rotations / PivotConstants.kGearRatio * 360;
    }
  
    public void changePIDValues(){

      // FUNCTION USED FOR TESTING.
  
      m_config = new SparkMaxConfig();
      m_config.smartCurrentLimit((int) NeoMotorConstants.kSmartCurrentLimitConstant);
      m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1.5)
        .i(0)
        .d(0)
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

  @Logged(name = "currentPosition")
  public double getPosition() {
    return m_encoder.getPosition();
  }

  @Logged(name = "getCurrent")
  public double getCurrent() {
    return m_pivot_motor.getOutputCurrent();
  }
  @Logged(name = "getVel")
  public double getVelocity() {
    return degreePerRotation(m_encoder.getVelocity());
  }
}

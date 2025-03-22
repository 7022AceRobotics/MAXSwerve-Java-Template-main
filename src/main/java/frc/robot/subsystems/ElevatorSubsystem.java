// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevator_motor;
    private SparkClosedLoopController elevator_pidController;
    public RelativeEncoder elevator_encoder;
    private SparkBaseConfig config;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    SmartDashboard.putNumber("ELE POS", 0);
    elevator_motor = new SparkMax(Constants.ElevatorConstants.ElevatorMotorPort, MotorType.kBrushless);
    elevator_pidController = elevator_motor.getClosedLoopController();
    elevator_encoder = elevator_motor.getEncoder();
    config = new SparkMaxConfig();

    // config.closedLoop
    //     .p(Constants.ElevatorConstants.kP)
    //     .i(Constants.ElevatorConstants.kI)
    //     .d(Constants.ElevatorConstants.kD)
    //     .outputRange(Constants.ElevatorConstants.kMinOutput, Constants.ElevatorConstants.kMaxOutput);
    // config.closedLoop.maxMotion
    //     .maxVelocity(Constants.ElevatorConstants.maxVel)
    //     .maxAcceleration(Constants.ElevatorConstants.maxAccel)
    //     .allowedClosedLoopError(Constants.ElevatorConstants.allowedErr);
    // elevator_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);    

    // config.closedLoopRampRate(0.05);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ElevatorConstants.kP)
        .i(ElevatorConstants.kI)
        .d(ElevatorConstants.kD)
        .outputRange(-1, 1)
        .p(1, ClosedLoopSlot.kSlot1)
          .i(0, ClosedLoopSlot.kSlot1)
          .d(0, ClosedLoopSlot.kSlot1)
          .velocityFF(0, ClosedLoopSlot.kSlot1);
    config.closedLoop.maxMotion
        .maxVelocity(NeoMotorConstants.kFreeSpeedRpm)
        .maxAcceleration(ElevatorConstants.maxAccel)
        .allowedClosedLoopError(ElevatorConstants.allowedErr);
    //elevator_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);    
    
    elevator_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevator_encoder.setPosition(0);
  }

  @Logged(name = "TargetPosition")
  private double m_targetPosition;

  @Logged(name = "currentPosition")
  public double getPosition() {
    return elevator_encoder.getPosition();
  }

  @Logged(name = "getCurrent")
  public double getCurrent() {
    return elevator_motor.getOutputCurrent();
  }

  public void SetElevatorPosition(double targetposition) {
    m_targetPosition = targetposition;
    elevator_pidController.setReference(targetposition, ControlType.kPosition);   
    // elevator_pidController.setReference(targetposition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.1, ArbFFUnits.kPercentOut);
    // elevator_pidController.setReference(0.05, ControlType.kDutyCycle);
    //SmartDashboard.putNumber("AAAAAAAAAAAAAAA", 5); 
  }
  
  public void resetElevatorPID(){
    config = new SparkMaxConfig();

    config.smartCurrentLimit(50);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(SmartDashboard.getNumber("kp ele", 1))
    .i(SmartDashboard.getNumber("ki ele", 0))
    .d(SmartDashboard.getNumber("kd ele", 0))
    .outputRange(-1, 1)
    .p(1, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(0, ClosedLoopSlot.kSlot1);
    config.closedLoop.maxMotion
    .maxVelocity(NeoMotorConstants.kFreeSpeedRpm)
    .maxAcceleration(5000)
    .allowedClosedLoopError(0.05);
    
    elevator_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double metersToRotations(double meters){
    return meters * ElevatorConstants.kElevatorGearRatio / (ElevatorConstants.kTeethOnSprocket * ElevatorConstants.pitch);
  }

  public double rotationsToMeters(double rotations){
    return rotations / ElevatorConstants.kElevatorGearRatio * (ElevatorConstants.kTeethOnSprocket * ElevatorConstants.pitch);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder ele", rotationsToMeters(elevator_encoder.getPosition()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevator_motor;
    private SparkClosedLoopController elevator_pidController;
    private RelativeEncoder elevator_encoder;
    private SparkBaseConfig config;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
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

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(SmartDashboard.getNumber("Kp Ele", 1))
        .i(SmartDashboard.getNumber("Ki Ele", 0))
        .d(SmartDashboard.getNumber("Kd Ele", 0))
        .outputRange(-1, 1)
        .p(1, ClosedLoopSlot.kSlot1)
          .i(0, ClosedLoopSlot.kSlot1)
          .d(0, ClosedLoopSlot.kSlot1)
          .velocityFF(0, ClosedLoopSlot.kSlot1);;
    config.closedLoop.maxMotion
        .maxVelocity(1000)
        .maxAcceleration(500)
        .allowedClosedLoopError(0.05);
    //elevator_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);    
    
    elevator_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevator_encoder.setPosition(0);
      SmartDashboard.putNumber("speed ele", 0);
      SmartDashboard.putNumber("kp ele", 0);
      SmartDashboard.putNumber("ki ele", 0);
      SmartDashboard.putNumber("kd ele", 0);
      SmartDashboard.putNumber("accel ele", 0);
      SmartDashboard.putNumber("Stage0 Ele", 0);
      SmartDashboard.putNumber("Stage1", 0);
      SmartDashboard.putNumber("Stage2 Ele", 0);
      SmartDashboard.putNumber("Stage3 Ele", 0);
  }

  public void SetElevatorPosition(double targetposition){
    elevator_pidController.setReference(targetposition, ControlType.kMAXMotionPositionControl);    
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
     
    elevator_encoder.setPosition(0);
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

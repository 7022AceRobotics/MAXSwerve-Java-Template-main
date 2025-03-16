// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax elevator_motor;
    private SparkClosedLoopController elevator_pidController;
    private RelativeEncoder elevator_encoder;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevator_motor = new SparkMax(Constants.ElevatorConstants.ElevatorMotorPort, MotorType.kBrushless);
    elevator_pidController = elevator_motor.getClosedLoopController();
    elevator_encoder = elevator_motor.getEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

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

    config.closedLoop
        .p(SmartDashboard.getNumber("Kp Ele", 0))
        .i(SmartDashboard.getNumber("Ki Ele", 0))
        .d(SmartDashboard.getNumber("Kd Ele", 0))
        .outputRange(SmartDashboard.getNumber("min Ele", 0), SmartDashboard.getNumber("min Ele", 0));
    config.closedLoop.maxMotion
        .maxVelocity(SmartDashboard.getNumber("speed Ele", 0))
        .maxAcceleration(SmartDashboard.getNumber("accel Ele", 0))
        .allowedClosedLoopError(SmartDashboard.getNumber("err Ele", 0));
    elevator_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);    
    
  }

  public void SetElevatorPosition(double targetposition){
    elevator_pidController.setReference(targetposition, SparkBase.ControlType.kMAXMotionPositionControl);    
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

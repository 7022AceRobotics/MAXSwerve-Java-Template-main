// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgeacollectorSubsystem extends SubsystemBase {
    private SparkMax Algea_Pull_motor;
    private SparkMax Algea_Pivot_motor;
    private SparkClosedLoopController Algea_pidController;
    private RelativeEncoder Algea_encoder;
    private final AnalogPotentiometer sensor;
  /** Creates a new AlgeaSubsystem. */
  public AlgeacollectorSubsystem() { 
    Algea_Pull_motor = new SparkMax(Constants.AlgeaConstants.AlgeaPullMotorPort, MotorType.kBrushless);
    Algea_Pivot_motor = new SparkMax(Constants.AlgeaConstants.AlgeaPivotMotorPort, MotorType.kBrushless);


    Algea_pidController = Algea_Pivot_motor.getClosedLoopController();
    // CHANGE TO ABSOLUTE ENCODER
    Algea_encoder = Algea_Pivot_motor.getEncoder();
    
    sensor = new AnalogPotentiometer(3);
    SparkMaxConfig config = new SparkMaxConfig();

    // config.closedLoop
    //     .p(Constants.AlgeaConstants.kP)
    //     .i(Constants.AlgeaConstants.kI)
    //     .d(Constants.AlgeaConstants.kD)
    //     .outputRange(Constants.AlgeaConstants.kMinOutput, Constants.AlgeaConstants.kMaxOutput);
    // config.closedLoop.maxMotion
    //     .maxVelocity(Constants.AlgeaConstants.maxVel)
    //     .maxAcceleration(Constants.AlgeaConstants.maxAccel)
    //     .allowedClosedLoopError(Constants.AlgeaConstants.allowedErr);
    // Algea_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);
    config.closedLoop
        .p(SmartDashboard.getNumber("Kp Alg", 0))
        .i(SmartDashboard.getNumber("Ki Alg", 0))
        .d(SmartDashboard.getNumber("Kd Alg", 0))
        .outputRange(SmartDashboard.getNumber("min Alg", 0), SmartDashboard.getNumber("min Alg", 0));
    config.closedLoop.maxMotion
        .maxVelocity(SmartDashboard.getNumber("speed Alg", 0))
        .maxAcceleration(SmartDashboard.getNumber("accel Alg", 0))
        .allowedClosedLoopError(SmartDashboard.getNumber("err Alg", 0));
    Algea_pidController.setReference(0, SparkBase.ControlType.kMAXMotionPositionControl);
  }

  public void SetAlgeaPosition(double targetposition){
    Algea_pidController.setReference(targetposition, SparkBase.ControlType.kMAXMotionPositionControl);    
  }
  public void PullSetSpeed(double speed){
     Algea_Pull_motor.set(speed);
  }
  public double Algeasensor(){
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

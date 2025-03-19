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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RudolphTheReindeer extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLEDBuffer m_led_buffer = new AddressableLEDBuffer(60);

  public RudolphTheReindeer() { 
    m_led.setLength(m_led_buffer.getLength());
    m_led.setData(m_led_buffer);
    m_led.start();
  }


  @Override

  public void periodic() {
    // This method will be called once per scheduler run
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RudolphTheReindeer extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(0);
  private AddressableLED m_led2 = new AddressableLED(1);

  private AddressableLEDBuffer m_led_buffer = new AddressableLEDBuffer(60);

  public RudolphTheReindeer() { 
    m_led.setLength(m_led_buffer.getLength());
    m_led.setData(m_led_buffer);
    m_led.start();

    m_led2.setLength(m_led_buffer.getLength());
    m_led2.setData(m_led_buffer);
    m_led2.start();
  }


  @Override

  public void periodic() {
    // This method will be called once per scheduler run
    // LEDPattern red = LEDPattern.solid(Color.kRed);
    // red.applyTo(m_led_buffer);
    // m_led.setData(m_led_buffer);
  }

  public void setRed(){
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setGreen(){
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    green.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setBlue(){
    LEDPattern blue = LEDPattern.solid(Color.kBlue);
    blue.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setYellow(){
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    yellow.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setOrange(){
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    orange.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setPurple(){
    LEDPattern purple = LEDPattern.solid(Color.kPurple);
    purple.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setBlink(Color colour){
      LEDPattern blinking = LEDPattern.solid(colour);
      blinking.blink(Seconds.of(0.1), Seconds.of(0.1));
      blinking.applyTo(m_led_buffer);
      m_led.setData(m_led_buffer);
      m_led2.setData(m_led_buffer);
  }

  public void setScroll(Color... colours){
    LEDPattern scroll = LEDPattern.gradient(GradientType.kDiscontinuous, colours);
    scroll = scroll.reversed();
    Distance ledSpacing = Meters.of(1/120);

    scroll = scroll.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
    scroll.applyTo(m_led_buffer);
    m_led.setData(m_led_buffer);
    m_led2.setData(m_led_buffer);
  }

  public void setBennies(){
    setScroll(Color.kWhite, Color.kCadetBlue, Color.kGold);
  }

  public void setRedBlink(){
    setBlink(Color.kRed);
  }

  public void setOrangeBlink(){
    setBlink(Color.kOrange);
  }

  public void setYellowBlink(){
    setBlink(Color.kYellow);
  }

  public void setGreenBlink(){
    setBlink(Color.kGreen);
  }

  public void setBlueBlink(){
    setBlink(Color.kBlue);
  }

  public void setPurpleBlink(){
    setBlink(Color.kPurple);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

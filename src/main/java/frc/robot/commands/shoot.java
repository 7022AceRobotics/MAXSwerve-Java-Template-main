// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class shoot extends Command {
  /** Creates a new shoot. */
  private final ShooterSubsystem m_shooting_subsystem;
  private final Supplier<Double> m_axis;
  private final Supplier<Double> m_axis2;
  public shoot(ShooterSubsystem m_shooting_subsystem, Supplier<Double> m_axis, Supplier<Double> m_axis_2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooting_subsystem = m_shooting_subsystem;
    this.m_axis = m_axis;
    this.m_axis2 =  m_axis_2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_axis.get() > 0.05){
    m_shooting_subsystem.shoot(m_axis.get());
    }
    else if (m_axis2.get() > 0.05){
      m_shooting_subsystem.shoot(m_axis2.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooting_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

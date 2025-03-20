// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveElevatorTo extends Command {
  /** Creates a new shoot. */
  private final ElevatorSubsystem m_elevator_subsystem;
  private final double m_position;
  private final ShooterSubsystem m_shooter_subsystem;
  public moveElevatorTo(ElevatorSubsystem m_elevator_subsystem, double m_position, ShooterSubsystem m_shooter_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator_subsystem = m_elevator_subsystem;
    this.m_position = m_position; // note that position is calculated in meters.
    this.m_shooter_subsystem = m_shooter_subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator_subsystem.SetElevatorPosition(m_elevator_subsystem.metersToRotations(ElevatorConstants.kL3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

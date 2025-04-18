// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgaeCollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeShoot extends Command {

  private final AlgaeCollectorSubsystem m_algae_collector_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeShoot(AlgaeCollectorSubsystem m_algae_collector_subsystem) {
    this.m_algae_collector_subsystem = m_algae_collector_subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algae_collector_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_algae_collector_subsystem.PullSetSpeed(-0.01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algae_collector_subsystem.PullSetSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlgeacollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgeaShoot extends Command {

  AlgeacollectorSubsystem AlgeacollectorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgeaShoot(AlgeacollectorSubsystem AlgeacollectorSubsystem) {
    this.AlgeacollectorSubsystem = AlgeacollectorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(AlgeacollectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AlgeacollectorSubsystem.PullSetSpeed(-0.01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AlgeacollectorSubsystem.PullSetSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

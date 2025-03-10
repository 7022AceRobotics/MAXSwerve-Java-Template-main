// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgeacollectorSubsystem;
import frc.robot.subsystems.AlgeacollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class AlgeaSuck extends Command {

  AlgeacollectorSubsystem AlgeacollectorSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgeaSuck(AlgeacollectorSubsystem AlgeacollectorSubsystem) {
    AlgeacollectorSubsystem = AlgeacollectorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(AlgeacollectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AlgeacollectorSubsystem.SetAlgeaPosition(Constants.AlgeaConstants.OutPosistion);
    AlgeacollectorSubsystem.PullSetSpeed(100);
    if(AlgeacollectorSubsystem.Algeasensor()>0.4){
      AlgeacollectorSubsystem.SetAlgeaPosition(Constants.AlgeaConstants.InPosistion);
    }
    AlgeacollectorSubsystem.PullSetSpeed(0);
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

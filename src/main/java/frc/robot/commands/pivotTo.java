// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShuffleboardEntries;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pivotTo extends Command {
  private final PivotSubsystem m_pivot_subsystem;
  private final double position;
    
    public pivotTo(PivotSubsystem m_pivot_subsystem, double position) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.m_pivot_subsystem = m_pivot_subsystem;
      this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  public void execute(){
    m_pivot_subsystem.goToPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
    m_pivot_subsystem.goToPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

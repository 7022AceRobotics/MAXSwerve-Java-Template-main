// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class resetPID extends InstantCommand {
  private final PivotSubsystem m_pivot_subsystem;
  private final ElevatorSubsystem m_elevator_subsystem;
  private final DriveSubsystem m_drive_subsystem;
  private final String subsystem;
  public resetPID(PivotSubsystem m_pivot_subsystem, ElevatorSubsystem m_elevator_subsystem, DriveSubsystem m_drive_subsystem, String subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivot_subsystem = m_pivot_subsystem;
    this.m_elevator_subsystem = m_elevator_subsystem;
    this.m_drive_subsystem = m_drive_subsystem;
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subsystem == "Pivot"){
    m_pivot_subsystem.changePIDValues();
    }
    else if (subsystem == "Elevator"){

      m_elevator_subsystem.resetElevatorPID();
    }
    else if (subsystem == "Drive"){
      m_drive_subsystem.resetPID();
    }
    
  }
}

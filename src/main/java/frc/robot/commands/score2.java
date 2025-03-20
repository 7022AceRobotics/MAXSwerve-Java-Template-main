// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class score2 extends SequentialCommandGroup {
  /** Creates a new score2. */
  // private final ElevatorSubsystem m_elevator_subsystem;
  // private final PivotSubsystem m_pivot_subsystem;
  // private final DriveSubsystem m_drive_subsystem;
  // private final ShooterSubsystem m_shooting_subsystem;
  // private final int m_position;
  private double m_elevator_position;
    private double m_pivot_position;
      

		public score2(ElevatorSubsystem m_elevator_subsystem, PivotSubsystem m_pivot_subsystem, DriveSubsystem m_drive_subsystem, ShooterSubsystem m_shooting_subsystem, int m_position) {
          // Use addRequirements() here to declare subsystem dependencies.
          // this.m_elevator_subsystem = m_elevator_subsystem;
          // this.m_pivot_subsystem = m_pivot_subsystem;
          // this.m_drive_subsystem = m_drive_subsystem;
          // this.m_shooting_subsystem = m_shooting_subsystem;
          // this.m_position = m_position;
      
          this.m_elevator_position = ElevatorConstants.kL0; // note that position is calculated in meters.
          this.m_pivot_position = PivotConstants.kL0; // note that position is calculated in degrees.
      
          switch (m_position){
            case 1:
              this.m_elevator_position = ElevatorConstants.kL1; // note that position is calculated in meters.
            this.m_pivot_position = PivotConstants.kL1; // note that position is calculated in degrees.
            break;
          case 2:
            this.m_elevator_position = ElevatorConstants.kL2; // note that position is calculated in meters.
            this.m_pivot_position = PivotConstants.kL2; // note that position is calculated in degrees.
            break;
          case 3:
            this.m_elevator_position = ElevatorConstants.kL3; // note that position is calculated in meters.
            this.m_pivot_position = PivotConstants.kL3; // note that position is calculated in degrees.
            break;
          case 4:
            this.m_elevator_position = ElevatorConstants.kL4; // note that position is calculated in meters.
            this.m_pivot_position = PivotConstants.kL4; // note that position is calculated in degrees.
            break;
          }

          addCommands(
            new SequentialCommandGroup(
            new moveElevatorTo(m_elevator_subsystem, m_elevator_position, m_shooting_subsystem),
            new pivotTo(m_pivot_subsystem, m_pivot_position),
            //new moveElevatorTo(m_elevator_subsystem, m_elevator_position, m_shooting_subsystem),
          new shoot(m_shooting_subsystem)));
      }
    }
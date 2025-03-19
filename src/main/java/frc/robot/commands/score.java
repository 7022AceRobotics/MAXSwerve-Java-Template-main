// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class score extends SequentialCommandGroup {
  /** Creates a new shoot. */
  private final elevatorSubsystem m_elevator_subsystem;
  private final PivotSubsystem m_pivot_subsystem;
  private final DriveSubsystem m_drive_subsystem;
  private final ShooterSubsystem m_shooting_subsystem
  private final double m_elevator_position;
  private final double m_pivot_position;

  public score(elevatorSubsystem m_elevator_subsystem, PivotSubsystem m_elevator_subsystem, DriveSubsystem m_drive_subsystem, ShooterSubsystem m_shooting_subsystem, double m_position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator_subsystem = m_elevator_subsystem;
    this.m_pivot_subsystem = m_pivot_subsystem;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ENSURE HEADING IS EITHER GOING TOWARDS AT OR AT AT.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new ParallelCommandGroup(
      new PivotTo(m_pivot_subsystem, m_pivot_position),
      new moveElevatorTo(m_elevator_subsystem, m_elevator_position)
    ).
    new shoot()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooting_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double distanceBetweenCenters(double Hx, double Hy, double Sx, double Sy) {
        return Math.sqrt(Math.pow(Hx - Sx, 2) + Math.pow(Hy - Sy, 2));
    }

    public double calculateDistance(double Hr, double Ss, double Hx, double Hy, double Sx, double Sy) {
        double centerDistance = distanceBetweenCenters(double Hx, double Hy, double Sx, double Sy);

        double hexagonEdge = Hr;
        double squareEdge = Ss / 2;

        if (centerDistance < hexagonEdge + squareEdge) {
            return 0;
        }
        return centerDistance - (hexagonEdge + squareEdge);
    }
}

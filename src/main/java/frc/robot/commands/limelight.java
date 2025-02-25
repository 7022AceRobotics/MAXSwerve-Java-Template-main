// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;

 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class limelight extends Command {
  /** Creates a new limelight. */
  private final LimelightHelpers m_limelight_subsystem;
  private final DriveSubsystem m_drive_subsystem;
  private DoubleLogEntry m_DoubleLog;
  private DataLog log;
  private Pose2d pose;
  //private SwerveDrivePoseEstimator m_swerve_drive_pose_estimator;
  private Timer timer;

  private double[] botpose;
  private double time;

  public limelight(LimelightHelpers limelight_subsystem, DriveSubsystem drive_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_limelight_subsystem = limelight_subsystem;
    this.m_drive_subsystem = drive_subsystem;
    //this.m_swerve_drive_pose_estimator = m_drive_subsystem.m_swerve_drive_pose_estimator;
    

    addRequirements(limelight_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_swerve_drive_pose_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_drive_subsystem.getHeading(), m_drive_subsystem.getSwerveModulePositions(), new Pose2d());
    time = timer.getFPGATimestamp();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    botpose = m_limelight_subsystem.update();

    log = DataLogManager.getLog();
    m_DoubleLog = new DoubleLogEntry(log, "/my/double");

    if (botpose != null){
      SmartDashboard.putNumber("Pose1: ", botpose[0]);
      SmartDashboard.putNumber("Pose2: ", botpose[1]);
      SmartDashboard.putNumber("Pose3: ", botpose[2]);
      SmartDashboard.putNumber("Pose4: ", botpose[3]);
      SmartDashboard.putNumber("Pose5: ", botpose[4]);
      SmartDashboard.putNumber("Pose6: ", botpose[5]);
      m_DoubleLog.append(botpose[2]);

      pose = new Pose2d(botpose[0], botpose[2], new Rotation2d(botpose[4]));

      m_drive_subsystem.updateSwerveDrive(m_drive_subsystem.getHeading(), m_drive_subsystem.getSwerveModulePositions());
      m_drive_subsystem.updateVisionSwerveDrive(pose, timer.getFPGATimestamp() - time);

      this.m_drive_subsystem.resetOdometry(m_drive_subsystem.getPoseEstimate());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

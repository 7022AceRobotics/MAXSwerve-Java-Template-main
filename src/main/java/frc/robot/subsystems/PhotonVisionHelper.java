// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PublicKey;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionHelper extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("MicrosoftCam");
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private DriveSubsystem m_drive_subsystem;
  private SwerveDrivePoseEstimator m_swerve_drive_pose_estimator;

  /** Creates a new PhotonVisionHelper. */
  public PhotonVisionHelper(DriveSubsystem m_drive_subsystem) {
    this.m_drive_subsystem = m_drive_subsystem;
    this.m_swerve_drive_pose_estimator = m_drive_subsystem.m_swerve_drive_pose_estimator;
  }

  @Override
  public void periodic() {
    camera.setPipelineIndex(2);
    var result = camera.getAllUnreadResults();
    if (!result.isEmpty()){
      var result2 = result.get(result.size() - 1);
      PhotonTrackedTarget target = result2.getBestTarget();
      Transform3d april_tag_pos = target.getBestCameraToTarget();
      m_swerve_drive_pose_estimator.addVisionMeasurement(
        m_swerve_drive_pose_estimator.getEstimatedPosition().transformBy(
        new Transform2d(
          april_tag_pos.getMeasureX(), 
          april_tag_pos.getMeasureY(), 
          april_tag_pos.getRotation().toRotation2d())), 
          result2.getTimestampSeconds());
      
      SmartDashboard.putNumber("result", 0);
    }

    this.m_drive_subsystem.resetOdometry(m_drive_subsystem.getPoseEstimate());
    // This method will be called once per scheduler run
  }

  public int getID() {
    var result = camera.getLatestResult();
    if (result.hasTargets()){
      return result.getBestTarget().fiducialId;
  }
  else{
      return -1;
  }
}

}

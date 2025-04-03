// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionHelper extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("MicrosoftCam");
  private AprilTagFieldLayout map = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private DriveSubsystem m_drive_subsystem;
  private SwerveDrivePoseEstimator m_swerve_drive_pose_estimator;
  private PhotonPoseEstimator m_photon_pose_estimator;

  private Optional<EstimatedRobotPose> estimated_pose;

  /** Creates a new PhotonVisionHelper. */
  public PhotonVisionHelper(DriveSubsystem m_drive_subsystem) {
    this.m_drive_subsystem = m_drive_subsystem;
    this.m_swerve_drive_pose_estimator = m_drive_subsystem.m_swerve_drive_pose_estimator;
    this.m_photon_pose_estimator = m_drive_subsystem.m_photon_pose_estimator;
  }

  @Override

  public void periodic() {
    camera.setPipelineIndex(2);
    var result = camera.getAllUnreadResults();
    if (!result.isEmpty()){
      //System.out.println("before result get size");
      var result2 = result.get(result.size() - 1);
      if (result2.hasTargets()){
        //System.out.println("before getbest targ");
        PhotonTrackedTarget target = result2.getBestTarget();
        //System.out.println("before best csm");
        Transform3d april_tag_pos = target.getBestCameraToTarget();
 //System.out.println("before heading");
        m_photon_pose_estimator.addHeadingData(result2.getTimestampSeconds(), m_drive_subsystem.getHeading());

        System.out.println("after heading");
        estimated_pose = m_photon_pose_estimator.update(result2);


        
        if (estimated_pose.isPresent()){
        m_swerve_drive_pose_estimator.addVisionMeasurement(
          estimated_pose.get().estimatedPose.toPose2d(),
          estimated_pose.get().timestampSeconds);


          SmartDashboard.putNumber("X100", estimated_pose.get().estimatedPose.getX());
          SmartDashboard.putNumber("Y100", estimated_pose.get().estimatedPose.getY());
          SmartDashboard.putNumber("R100", Units.radiansToDegrees(estimated_pose.get().estimatedPose.getRotation().getAngle()));
        }
      }

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

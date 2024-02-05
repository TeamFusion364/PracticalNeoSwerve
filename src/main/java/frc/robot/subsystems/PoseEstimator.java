// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConfig;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  public SwerveDrivePoseEstimator sEstimator;
  public Pose2d visionPose = new Pose2d();
  
  public PoseEstimator() {
    sEstimator = new SwerveDrivePoseEstimator(
      SwerveConfig.swerveKinematics,
      new Rotation2d(),
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      },
      new Pose2d(),
      Constants.PoseEstimator.stateStdDevs,
      Constants.PoseEstimator.VisionStdDevs
    );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

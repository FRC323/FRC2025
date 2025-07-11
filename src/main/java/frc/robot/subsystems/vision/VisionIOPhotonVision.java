// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final PhotonPoseEstimator poseEstimator;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    poseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void setPipeline(int pipeline) {
    camera.setPipelineIndex(pipeline);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                result.getBestTarget().getFiducialId(),
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation =
            new TargetObservation(0, new Rotation2d(), new Rotation2d());
      }

      Optional<EstimatedRobotPose> poseEst = poseEstimator.update(result);

      if (poseEst.isPresent()) {
        EstimatedRobotPose estimatedPose = poseEst.get();
        Pose3d robotPose = estimatedPose.estimatedPose;

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        double averageTagDistance = totalTagDistance / result.targets.size();

        // Add tag IDs
        for (var target : result.targets) {
          tagIds.add((short) target.fiducialId);
        }

        // calculate ambiguity
        var ambiguity = 0.0;
        if (result.multitagResult.isPresent()) {
          ambiguity = result.multitagResult.get().estimatedPose.ambiguity;
        } else if (!result.targets.isEmpty()) {
          var target = result.targets.get(0);
          ambiguity = target.poseAmbiguity;
        }

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                ambiguity, // Ambiguity
                result.targets.size(), // Tag count
                averageTagDistance, // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      }

      // NOTE: below is template code. -ats
      // I updated with above. To me, using PhotonPoseEstimator just makes more sense
      // in our situation, as we don't have a lot of tags in view at home field.
      // PhotonPoseEstimator defaults to check for multitag results so no loss there
      // just allows us to fall back on low ambiguity for our pratice field.

      // Add pose observation
      //   if (result.multitagResult.isPresent()) {
      //     var multitagResult = result.multitagResult.get();

      //     // Calculate robot pose
      //     Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      //     Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      //     Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(),
      // fieldToRobot.getRotation());

      //     // Calculate average tag distance
      //     double totalTagDistance = 0.0;
      //     for (var target : result.targets) {
      //       totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      //     }

      //     // Add tag IDs
      //     tagIds.addAll(multitagResult.fiducialIDsUsed);

      //     // Add observation
      //     poseObservations.add(
      //         new PoseObservation(
      //             result.getTimestampSeconds(), // Timestamp
      //             robotPose, // 3D pose estimate
      //             multitagResult.estimatedPose.ambiguity, // Ambiguity
      //             multitagResult.fiducialIDsUsed.size(), // Tag count
      //             totalTagDistance / result.targets.size(), // Average tag distance
      //             PoseObservationType.PHOTONVISION)); // Observation type
      //   }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}

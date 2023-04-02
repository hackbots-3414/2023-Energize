package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
 
 public class VisionWrapper {
     private PhotonCamera photonCamera;
     private PhotonPoseEstimator photonPoseEstimator;
     private MedianFilter xDistanceFilter = new MedianFilter(1);
     private MedianFilter yDistanceFilter = new MedianFilter(1);
 
     public VisionWrapper() {
         photonCamera = new PhotonCamera("Front_Camera");
 
         try {
             AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
             photonPoseEstimator =
                     new PhotonPoseEstimator(
                             fieldLayout, 
                             PoseStrategy.MULTI_TAG_PNP, 
                             photonCamera, 
                             new Transform3d(
                                new Translation3d(0, 0, 0), 
                                new Rotation3d(0, 0, 0)));

             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

         } catch (IOException e) {
             DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
             photonPoseEstimator = null;
         }
     }

     public List getTargets() {
        return photonCamera.getLatestResult().targets;
     }

     public Matrix getStandardD() {
        PhotonTrackedTarget target = photonCamera.getLatestResult().getBestTarget();
        Transform3d distance = target.getBestCameraToTarget();

        Matrix values = new Matrix<>(Nat.N3(), Nat.N1());
        values.set(0, 0, Math.pow(xDistanceFilter.calculate(distance.getX()), 2)*0.01);
        values.set(1, 0, Math.pow(yDistanceFilter.calculate(distance.getY()), 2)*0.01);
        values.set(2, 0, Math.pow(distance.getRotation().getAngle(), 2)*1.0);
        return values;
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
      *     the estimate
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         if (photonPoseEstimator == null) {
             return Optional.empty();
         }
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }
 }
/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
 import java.io.IOException;
import java.lang.annotation.Target;
import java.util.List;
import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class VisionWrapper {
     private PhotonCamera photonCamera;
     private PhotonPoseEstimator photonPoseEstimator;
 
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
        values.set(0, 0, Math.pow(distance.getX(), 2)*0.02);
        values.set(1, 0, Math.pow(distance.getY(), 2));
        values.set(2, 0, Math.pow(distance.getRotation().getAngle(), 2)*0.02);
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
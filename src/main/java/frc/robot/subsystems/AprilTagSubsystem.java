package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPipelineResult result;
    DrivebaseSubsystem drivebaseSubsystem;
    AprilTagFieldLayout aprilTagFieldLayout;

    private static final double reprojectionErrorThresholdLow = 1.8;
    private static final double reprojectionErrorThresholdHigh = 5.0;
    Transform2d cameraOffsetTransform = new Transform2d(-0.41, 0.0, Rotation2d.fromDegrees(180));

    private static final int RED_AMP_TAG_ID = 5;
    private static final int BLUE_AMP_TAG_ID = 6;

    Transform2d AMP_TO_CAMERA_TRANSFORM = new Transform2d(0.64,-0.127,new Rotation2d(0));


    public AprilTagSubsystem(DrivebaseSubsystem drivebaseSubsystem){
        this.drivebaseSubsystem = drivebaseSubsystem;
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load april tag layout");
        }
    }

    public boolean seesAprilTag(){
        return result.hasTargets();
    }

    private boolean isSaneMeasurement(PNPResult estimatedPose) {
        // if (estimatedPose.bestReprojErr > reprojectionErrorThresholdLow &&
        //     estimatedPose.bestReprojErr < reprojectionErrorThresholdHigh ) {
        //         return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 10);
        // }
        // return false;
        return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 12);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult  res = camera.getLatestResult();
        
        if (res.hasTargets()) {
            // PhotonTrackedTarget bestTarget = res.getBestTarget();
            double imageCaptureTime = res.getTimestampSeconds();
            var estimatedPose = res.getMultiTagResult().estimatedPose;
            
            if (estimatedPose.isPresent && isSaneMeasurement(estimatedPose)) {
                    Pose2d adjustedPose = new Pose3d(estimatedPose.best.getTranslation(),
                        estimatedPose.best.getRotation()).toPose2d().transformBy(cameraOffsetTransform);

                    drivebaseSubsystem.addVisionMeasurement(adjustedPose, imageCaptureTime);
                    SmartDashboard.putBoolean("Is Using Vision", true);
                    // System.out.println("adding measurement " + adjustedPose + ", error: " + estimatedPose.bestReprojErr);
            } else {
                SmartDashboard.putBoolean("Is Using Vision", false);
            }
            // } else if (bestTarget != null) {
            //     Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
            //     //camToTargetTrans.plus( new Transform3d( cameraOffsetTransform.getTranslation(), new Rotation2d());
            //     camToTargetTrans = camToTargetTrans.plus(cameraOffsetTransform3d);
            //     Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
            //     if (tagPose.isPresent()) {
            //         Pose2d pose = tagPose.get().transformBy(camToTargetTrans).toPose2d();
            //         System.out.println("single vision mesaurement from tag " + bestTarget.getFiducialId() + ": " + pose.toString());
            //         drivebaseSubsystem.addVisionMeasurement(pose, imageCaptureTime);
            //     }
            // }
        }
    }

    public Pose2d getAmpPose() {
        int aprilTagID = 0;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            aprilTagID = (alliance.get() == DriverStation.Alliance.Red) ? RED_AMP_TAG_ID : BLUE_AMP_TAG_ID;
        }

        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(aprilTagID);
        if (tagPose.isPresent()) {
            Pose2d tagPose2d = tagPose.get().toPose2d();
            tagPose2d = tagPose2d.transformBy(AMP_TO_CAMERA_TRANSFORM);
            System.out.println("transformed Tag Pose: " + tagPose2d);
            return tagPose2d;
        }
        return null;
    }
}

package frc.robot.subsystems.apriltags;

import java.io.IOException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera rearCamera;
    private PhotonCamera rightSideCamera;
    private final PhotonPoseEstimator rearCameraPhotonEstimator;
    private final PhotonPoseEstimator sideCameraPhotonEstimator;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    // private PhotonPipelineResult result;
    DrivebaseSubsystem drivebaseSubsystem;
    public AprilTagFieldLayout aprilTagFieldLayout;

    Transform3d robotToRearCameraTransform = new Transform3d(
        -0.4, // x
        -0.07, // y
        0.2286, // z
        new Rotation3d(0,Math.toRadians(-34),Math.toRadians(179)));

    Transform3d robotToSideCameraTransform = new Transform3d(
        0.06, // x
        -0.55, // y
        0.2286, // z
        new Rotation3d(0,Math.toRadians(-29),Math.toRadians(-90)));

    private static final int RED_AMP_TAG_ID = 5;
    private static final int BLUE_AMP_TAG_ID = 6;

    Transform2d AMP_TO_CAMERA_TRANSFORM = new Transform2d(0.64, -0.127, new Rotation2d(0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, Math.toRadians(5));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(3));
    private double lastEstTimestamp = 0;

    private static AprilTagSubsystem mInstance = null;

    public static AprilTagSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new AprilTagSubsystem(DrivebaseSubsystem.getInstance());
        }
        return mInstance;
    }

    private AprilTagSubsystem(DrivebaseSubsystem drivebaseSubsystem) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        rearCamera = new PhotonCamera("AprilTagCamera");
        rightSideCamera = new PhotonCamera("AprilTagCameraSide");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load april tag layout");
        }


        rearCameraPhotonEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rearCamera, robotToRearCameraTransform);
        rearCameraPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        sideCameraPhotonEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightSideCamera, robotToSideCameraTransform);
        sideCameraPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(rearCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToRearCameraTransform);

            cameraSim.enableDrawWireframe(true);
        }

        processingThread.start();
    }

    // public boolean seesAprilTag() {
    //     return result.hasTargets();
    // }

    // private boolean isSaneMeasurement(PNPResult estimatedPose) {
    //     // if (estimatedPose.bestReprojErr > reprojectionErrorThresholdLow &&
    //     // estimatedPose.bestReprojErr < reprojectionErrorThresholdHigh ) {
    //     // return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 10);
    //     // }
    //     // return false;
    //     return (estimatedPose.best.getX() < 4.0) || (estimatedPose.best.getX() > 12);
    // }

    static private double lastRear2TagReading = 0;
    Thread processingThread = new Thread("AprilTag Thread") {
        @Override
        public void run() {
            System.out.println("AprilTag Processing Thread started");
            
            while (true) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                var rearVisionEst = getEstimatedGlobalPose(rearCameraPhotonEstimator, rearCamera);
                rearVisionEst.ifPresent(
                        est -> {
                            Pose2d estPose = est.estimatedPose.toPose2d();
                            // Logger.recordOutput("Pose/Vision/Rear", estPose);

                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs(estPose, rearCameraPhotonEstimator);

                            if (est.targetsUsed.size() >= 2) {
                                lastRear2TagReading = est.timestampSeconds;
                            }
        
                            drivebaseSubsystem.addVisionMeasurement(
                                    estPose, est.timestampSeconds, estStdDevs);
                        });

                // if (rearVisionEst.isEmpty() || (rearVisionEst.isPresent() && rearVisionEst.get().targetsUsed.size() < 2 )) {
                //     var sideVisionEst = getEstimatedGlobalPose(sideCameraPhotonEstimator, rightSideCamera);
                //     sideVisionEst.ifPresent(
                //             est -> {
                //                 if (est.timestampSeconds - lastRear2TagReading > 0.25) {
                //                     Pose2d estPose = est.estimatedPose.toPose2d();
                //                     // Logger.recordOutput("Pose/Vision/Side", estPose);
                //                     // Change our trust in the measurement based on the tags we can see
                //                     // var estStdDevs = getEstimationStdDevs(estPose, sideCameraPhotonEstimator);
                
                //                     // drivebaseSubsystem.addVisionMeasurement(
                //                     //     estPose, est.timestampSeconds, estStdDevs);
                //                 }
                //             });
                // }
            }
        };
    };

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        Pose2d currentPose = drivebaseSubsystem.getCurrentPose();
        if (currentPose == null) {
            return;
        }
        visionSim.update(currentPose);

        var debugField = getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(currentPose);
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

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator estimator, PhotonCamera camera) {
        var visionEst = estimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedRearGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPoseEstimator estimator) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = rearCamera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AprilTagSubsystem extends SubsystemBase {
    private PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    // private PhotonPipelineResult result;
    DrivebaseSubsystem drivebaseSubsystem;
    public AprilTagFieldLayout aprilTagFieldLayout;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("VisionPose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    Transform3d robotToCameraTransform = new Transform3d(
        -0.6, // x
        0.015, // y
        0.585, // z
        new Rotation3d(0,Math.toRadians(-30),Math.toRadians(-180)));
//double roll, double pitch, double yaw
    private static final int RED_AMP_TAG_ID = 5;
    private static final int BLUE_AMP_TAG_ID = 6;

    Transform2d AMP_TO_CAMERA_TRANSFORM = new Transform2d(0.64, -0.127, new Rotation2d(0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 99);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 99);
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
        camera = new PhotonCamera("AprilTagCamera");

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Failed to load april tag layout");
        }


        photonEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


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
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCameraTransform);

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
                var visionEst = getEstimatedGlobalPose();
                visionEst.ifPresent(
                        est -> {
                            var estPose = est.estimatedPose.toPose2d();

                            fieldTypePub.set("Field2d");
                            fieldPub.set(new double[] {
                                    estPose.getX(),
                                    estPose.getY(),
                                    estPose.getRotation().getDegrees()
                            });
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs(estPose);
        
                            drivebaseSubsystem.addVisionMeasurement(
                                    est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
                // PhotonPipelineResult res = camera.getLatestResult();

                // if (res.hasTargets()) {
                //     // PhotonTrackedTarget bestTarget = res.getBestTarget();
                //     double imageCaptureTime = res.getTimestampSeconds();
                //     var estimatedPose = res.getMultiTagResult().estimatedPose;

                //     if (estimatedPose.isPresent && isSaneMeasurement(estimatedPose)) {
                //         Pose2d adjustedPose = new Pose3d(estimatedPose.best.getTranslation(),
                //                 estimatedPose.best.getRotation()).toPose2d().transformBy(cameraOffsetTransform);

                //         drivebaseSubsystem.addVisionMeasurement(adjustedPose, imageCaptureTime);
                //         SmartDashboard.putBoolean("Is Using Vision", true);
                //         // System.out.println("adding measurement " + adjustedPose + ", error: " +
                //         // estimatedPose.bestReprojErr);
                //     } else {
                //         SmartDashboard.putBoolean("Is Using Vision", false);
                //     }
                //     // } else if (bestTarget != null) {
                //     // Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
                //     // //camToTargetTrans.plus( new Transform3d(
                //     // cameraOffsetTransform.getTranslation(), new Rotation2d());
                //     // camToTargetTrans = camToTargetTrans.plus(cameraOffsetTransform3d);
                //     // Optional<Pose3d> tagPose =
                //     // aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
                //     // if (tagPose.isPresent()) {
                //     // Pose2d pose = tagPose.get().transformBy(camToTargetTrans).toPose2d();
                //     // System.out.println("single vision mesaurement from tag " +
                //     // bestTarget.getFiducialId() + ": " + pose.toString());
                //     // drivebaseSubsystem.addVisionMeasurement(pose, imageCaptureTime);
                //     // }
                //     // }
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
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update();
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
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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

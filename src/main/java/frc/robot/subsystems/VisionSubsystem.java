// Copyright (c) 2025
// Based on PhotonVision and AprilTag alignment examples.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlignConstants;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * VisionSubsystem
 *
 * <p>- Keeps alignment command intact - Uses PhotonPoseEstimator per Photon docs/javadocs:
 * PhotonPoseEstimator(AprilTagFieldLayout, PoseStrategy, Transform3d) - Loops over
 * camera.getAllUnreadResults() and calls poseEstimator.update(result) - Stores latest Estimated
 * pose and timestamp; attempts to apply it to your SwerveSubsystem by reflection
 */
public class VisionSubsystem extends SubsystemBase {
  private static final String CAMERA_NAME = "spatulas_eye";
  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);

  private PhotonCameraSim cameraSim = null;
  private VisionSystemSim visionSim = null;
  private boolean simEnabled = false;
  private AprilTagFieldLayout fieldLayout = null;
  // State for alignment
  private boolean hasTarget = false;
  private double targetYawDeg = 0.0;
  private double targetDistanceMeters = 0.0;

  private final SwerveSubsystem drivebase;

  // Gains for alignment
  private static final double TRANSLATION_KP = 2.0;
  private static final double ROTATION_KP = 1.75;
  private static final double MAX_LINEAR_SPEED_MPS = 2.65; // 1.65
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 3.5; // 3.15

  private static final double TAG_STANDOFF_METERS = 0.4375; // .45
  private static final double POSITION_TOLERANCE_METERS = 0.05;
  private static final double ANGLE_TOLERANCE_DEGREES = 2.75; // 3
  private static final int REQUIRED_STABLE_CYCLES = 5;

  private int alignedStableCounter = 0;

  private final Set<Integer> allowedTagIDs =
      new HashSet<>(Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));

  // Fields for PhotonPoseEstimator usage
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator; // may be null if fieldLayout unavailable

  // last estimated pose & timestamp
  private Optional<Pose2d> lastEstimatedPose = Optional.empty();
  private double lastEstimatedTimestamp = 0.0;

  public VisionSubsystem(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;

    // Load field layout
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    } catch (Exception ex) {
      System.err.println("[Vision] Could not load AprilTagFieldLayout: " + ex.getMessage());
      fieldLayout = null;
    }

    // Robot -> camera transform (meters / radians)
    robotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(15), Units.inchesToMeters(-8), Units.inchesToMeters(11)),
            new Rotation3d(0.0, 0.0, 0.0));

    // Create photon pose estimator per javadoc signature:
    // PhotonPoseEstimator(AprilTagFieldLayout fieldTags, PoseStrategy strategy,
    // Transform3d
    // robotToCamera)
    if (fieldLayout != null) {
      poseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

      // BEST fallback for accuracy
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } else {
      // if layout not available, don't create estimator
      // set a dummy (null) and guard usage below
      System.err.println("[Vision] fieldLayout null -> poseEstimator disabled");
      throw new IllegalStateException(
          "[VisionSubsystem] AprilTagFieldLayout required for pose estimator");
    }

    // Simulation setup
    setupSimIfNeeded();
  }

  private List<PhotonPipelineResult> setupSimIfNeeded() {
    if (!RobotBase.isSimulation()) {
      simEnabled = false;
      List<PhotonPipelineResult> result = camera.getAllUnreadResults();
      return result;
    }

    simEnabled = true;

    // Sim camera properties
    SimCameraProperties simProps = new SimCameraProperties();
    simProps.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    simProps.setFPS(60);
    simProps.setAvgLatencyMs(25);

    cameraSim = new PhotonCameraSim(camera, simProps);
    visionSim = new VisionSystemSim("photonvision_sim");

    // Attach camera to sim using same robotToCamera transform
    visionSim.addCamera(cameraSim, robotToCamera);

    if (fieldLayout != null) {
      visionSim.addAprilTags(fieldLayout);
    }

    cameraSim.enableDrawWireframe(true);
    SmartDashboard.putData("Vision Field", visionSim.getDebugField());

    // Return any unread results (likely empty) to satisfy return type
    return camera.getAllUnreadResults();
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getTargetYawDeg() {
    return targetYawDeg;
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // Update simulation if needed
    if (simEnabled && visionSim != null) {
      Pose2d pose2d = drivebase.getPose();
      if (pose2d != null) visionSim.update(pose2d);
    }

    // Process latest vision results

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) {
        continue;
      }
      Optional<EstimatedRobotPose> maybeEst = poseEstimator.update(result);
      targetYawDeg = result.getBestTarget().getYaw();
      maybeEst.ifPresent(
          est -> {
            Pose2d est2d = est.estimatedPose.toPose2d();
            lastEstimatedPose = Optional.of(est2d);
            lastEstimatedTimestamp = est.timestampSeconds;

            SmartDashboard.putBoolean("Vision/HasPose", true);
            SmartDashboard.putNumber("Vision/PoseX", est2d.getX());
            SmartDashboard.putNumber("Vision/PoseY", est2d.getY());
            SmartDashboard.putNumber("Vision/PoseRotDeg", est2d.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/Timestamp", est.timestampSeconds);


          });
    }
  }

  // Returns the last vision-estimated Pose2d (field-relative) if available
  public Optional<Pose2d> getEstimatedPose() {
    return lastEstimatedPose;
  }

  // Returns the timestamp (seconds) of the last estimate, or 0.0 if none.
  public double getEstimatedPoseTimestamp() {
    return lastEstimatedPose.isPresent() ? lastEstimatedTimestamp : 0.0;
  }

  public Command autoAlignCommand() {

    return this.run(
        () -> {

          // Auto-turn if vision sees target
          double FORWARD = 0.0;
          double STRAFE = 0.0;
          double TURN = 0.0;

          if (hasTarget()) {

            // proportional turn (degrees -> radians) with clamping
            double yawErrorRad = Units.degreesToRadians(getTargetYawDeg());
            TURN = -AlignConstants.AIM_KP * yawErrorRad;
            TURN =
                Math.max(
                    -Constants.Swerve.kMaxAngularSpeed,
                    Math.min(Constants.Swerve.kMaxAngularSpeed, TURN));
          }
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  FORWARD, STRAFE, TURN, drivebase.getPose().getRotation());
          drivebase.drive(speeds);

          // Respond
          SmartDashboard.putNumber("Vision/TargetYawDeg", getTargetYawDeg());
          SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
        });
  }
}

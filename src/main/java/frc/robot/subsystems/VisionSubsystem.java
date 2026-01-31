// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
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
 * <p>Keeps alignment command intact - Uses PhotonPoseEstimator per Photon docs/javadocs:
 * PhotonPoseEstimator(AprilTagFieldLayout, PoseStrategy, Transform3d) - Loops over
 * camera.getAllUnreadResults() and calls poseEstimator.update(result) - Stores latest Estimated
 * pose and timestamp; attempts to apply it to your SwerveSubsystem by reflection
 */
public class VisionSubsystem extends SubsystemBase {

  private static final String CAMERA_NAME = "spatulas_eye";
  private static final String REAR_CAMERA_NAME = "spatulas_eye_back";

  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
  private final PhotonCamera rearCamera = new PhotonCamera(REAR_CAMERA_NAME);

  private PhotonCameraSim cameraSim = null;
  private VisionSystemSim visionSim = null;
  private boolean simEnabled = false;
  private AprilTagFieldLayout fieldLayout = null;

  private final SwerveSubsystem drivebase;

  // Gains for alignment (I will eventually move this to Constants, or not since this is the Kitbot,
  // but I will most likely do it on our main robot)
  private static final double TRANSLATION_KP = 2.0;
  private static final double ROTATION_KP = 1.75;
  public static final double MAX_LINEAR_SPEED_MPS = 4.35; // 2.65
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 5; // 3.75
  private static final double MIN_LINEAR_SPEED_MPS = 0.45;
  private static final double MIN_ANGULAR_SPEED_RAD_PER_SEC = 0.35;

  private static final double TAG_STANDOFF_METERS = 0.4375; // .45
  private static final double POSITION_TOLERANCE_METERS = 0.05;
  private static final double ANGLE_TOLERANCE_DEGREES = 2.75; // 3
  private static final int REQUIRED_STABLE_CYCLES = 5;
  private static final double AGGRESSIVE_DISTANCE_METERS = 0.15;
  private static final double FAR_GAIN_MULTIPLIER = 1.8;

  private double lastVisionFuseTime = 0.0;
  private static final double MIN_VISION_FUSE_PERIOD = 0.06; // ~16 Hz

  private int alignedStableCounter = 0;

  // For the April Tags we want to align and drive to
  private final Set<Integer> allowedTagIDs =
      new HashSet<>(Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));

  // Fields for PhotonPoseEstimator usage
  private final Transform3d robotToCamera;
  private final Transform3d robotToRearCamera;
  private final PhotonPoseEstimator frontPoseEstimator;
  private final PhotonPoseEstimator rearPoseEstimator;
  // May be null if fieldLayout is unavailable

  // Last estimated pose & timestamp
  private Optional<Pose2d> lastEstimatedPose = Optional.empty();
  private double lastEstimatedTimestamp = 0.0;

  public VisionSubsystem(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;

    // Load field layout
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    } catch (Exception ex) {
      System.err.println("[Vision] Could not load AprilTagFieldLayout: " + ex.getMessage());
      fieldLayout = null;
    }

    /**
     * Robot to camera transform offset
     *
     * <p>Don't make the same mistake I did, we use FREEDOM units here. When setting an offset in
     * rotation, the unit is in Radians. When setting an offset in length/width, the unit is in
     * Meters. Reference off the WPILib Coordinate System for positive/negative values for the
     * offset.
     */
    robotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0.5), Units.inchesToMeters(-13), Units.inchesToMeters(11)),
            new Rotation3d(0, Units.degreesToRadians(-30), Math.PI));

    robotToRearCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(24), Units.inchesToMeters(0), Units.inchesToMeters(11)),
            new Rotation3d(0, Units.degreesToRadians(15), 0));

    // Create photon pose estimator
    if (fieldLayout != null) {
      frontPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

      frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      rearPoseEstimator =
          new PhotonPoseEstimator(
              fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToRearCamera);

      rearPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    } else {
      // If the layout is not available, don't create estimator
      // Set it to be null so the robot doesn't explode
      System.err.println("[Vision] fieldLayout null -> poseEstimator disabled");
      throw new IllegalStateException(
          "[VisionSubsystem] AprilTagFieldLayout required for pose estimator");
    }

    // Simulation setup
    setupSimIfNeeded();
  }

  private void setupSimIfNeeded() {
    if (!RobotBase.isSimulation()) {
      simEnabled = false;
      return;
    }

    simEnabled = true;

    // Sim camera properties
    SimCameraProperties simProps = new SimCameraProperties();
    simProps.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    simProps.setFPS(60);
    simProps.setAvgLatencyMs(25);

    PhotonCameraSim rearCameraSim = new PhotonCameraSim(rearCamera, simProps);

    visionSim.addCamera(cameraSim, robotToCamera);
    visionSim.addCamera(rearCameraSim, robotToRearCamera);
    visionSim = new VisionSystemSim("photonvision_sim");

    // Attach camera to sim using same robotToCamera transform
    visionSim.addCamera(cameraSim, robotToCamera);

    if (fieldLayout != null) {
      visionSim.addAprilTags(fieldLayout);
    }

    cameraSim.enableDrawWireframe(true);
    SmartDashboard.putData("Vision Field", visionSim.getDebugField());
  }

  private boolean isVisionMeasurementTrusted(EstimatedRobotPose est, PhotonCamera cam) {

    int tagCount = est.targetsUsed.size();

    double avgDistance =
        est.targetsUsed.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(999.0);

    // Reject garbage
    if (tagCount == 0) return false;

    // Reject far poses
    if (avgDistance > 4.5) return false;

    // Reject single-tag far shots
    if (tagCount == 1 && avgDistance > 2.5) return false;

    // Reject rear camera while spinning fast
    if (cam == rearCamera && Math.abs(drivebase.getRobotVelocity().omegaRadiansPerSecond) > 2.5) {
      return false;
    }

    return true;
  }

  private void processCamera(PhotonCamera cam, PhotonPoseEstimator estimator) {

    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    results.add(cam.getLatestResult());

    for (PhotonPipelineResult result : results) {
      if (!result.hasTargets()) continue;

      Optional<EstimatedRobotPose> maybeEst = estimator.update(result);
      maybeEst.ifPresent(
          est -> {
            Pose2d est2d = est.estimatedPose.toPose2d();

            lastEstimatedPose = Optional.of(est2d);
            lastEstimatedTimestamp = est.timestampSeconds;

            double now = Timer.getFPGATimestamp();

            if (now - lastVisionFuseTime < MIN_VISION_FUSE_PERIOD) {
              return;
            }

            if (!isVisionMeasurementTrusted(est, cam)) {
              return;
            }

            lastVisionFuseTime = now;
            drivebase.addVisionMeasurement(est2d, est.timestampSeconds);

            SmartDashboard.putBoolean("Vision/HasPose", true);
            SmartDashboard.putNumber("Vision/PoseX", est2d.getX());
            SmartDashboard.putNumber("Vision/PoseY", est2d.getY());
            SmartDashboard.putNumber("Vision/PoseRotDeg", est2d.getRotation().getDegrees());
          });
    }
  }

  @Override
  public void periodic() {
    // Update simulation if needed
    if (simEnabled && visionSim != null) {
      Pose2d pose2d = drivebase.getPose();
      if (pose2d != null) visionSim.update(pose2d);
    }

    // Process latest vision results
    // Include latest if none unread. It's deprecated, but it's a great fallback
    processCamera(camera, frontPoseEstimator);
    processCamera(rearCamera, rearPoseEstimator);
  }

  // Returns the last vision-estimated Pose2d (field-relative) if available
  public Optional<Pose2d> getEstimatedPose() {
    return lastEstimatedPose;
  }

  // Returns the timestamp (seconds) of the last estimate, or 0.0 if none.
  public double getEstimatedPoseTimestamp() {
    return lastEstimatedPose.isPresent() ? lastEstimatedTimestamp : 0.0;
  }

  // Was more applicable for REEFSCAPE, in review for removal
  private Optional<AprilTag> getNearestAllowedTag(Pose2d robotPose) {
    if (fieldLayout == null) return Optional.empty();

    double closestDistance = Double.MAX_VALUE;
    AprilTag closestTag = null;

    for (AprilTag tag : fieldLayout.getTags()) {
      if (!allowedTagIDs.contains(tag.ID)) continue;

      double distance =
          tag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTag = tag;
      }
    }

    return Optional.ofNullable(closestTag);
  }

  // Computes the geometric center of the scoring area using alliance-specific AprilTags
  private Optional<Translation2d> getScoringCenter() {
    if (fieldLayout == null) return Optional.empty();

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    // Red HUB side
    Set<Integer> redScoringTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    // Blue HUB side
    Set<Integer> blueScoringTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    Set<Integer> scoringTagIDs = (alliance == Alliance.Blue) ? blueScoringTags : redScoringTags;

    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;

    for (AprilTag tag : fieldLayout.getTags()) {
      if (!scoringTagIDs.contains(tag.ID)) continue;

      Translation2d pos = tag.pose.toPose2d().getTranslation();
      sumX += pos.getX();
      sumY += pos.getY();
      count++;
    }

    if (count == 0) return Optional.empty();

    return Optional.of(new Translation2d(sumX / count, sumY / count));
  }

  // Was more applicable for REEFSCAPE, in review for removal
  public Command driveAndAlignToNearestTag() {
    alignedStableCounter = 0;

    return this.run(
            () -> {
              Pose2d robotPose = drivebase.getPose();
              @SuppressWarnings("unused")
              PhotonPipelineResult result = camera.getLatestResult();
              Optional<AprilTag> maybeTag = getNearestAllowedTag(robotPose);

              if (maybeTag.isEmpty()) {
                drivebase.drive(new ChassisSpeeds(0, 0, 0));
                return;
              }

              AprilTag tag = maybeTag.get();
              Pose2d tagPose = tag.pose.toPose2d();

              // Rotate 180° so FRONT faces the tag (instead of back)
              Rotation2d desiredHeading = tagPose.getRotation().plus(Rotation2d.fromDegrees(180));

              // Offset now goes *forward* from tag
              Translation2d offset =
                  new Translation2d(TAG_STANDOFF_METERS, 0).rotateBy(tagPose.getRotation());
              Pose2d targetPose = new Pose2d(tagPose.getTranslation().plus(offset), desiredHeading);

              // Compute errors
              Translation2d error = targetPose.getTranslation().minus(robotPose.getTranslation());
              double dx = error.getX();
              double dy = error.getY();

              double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
              headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

              // PID control
              double vx = TRANSLATION_KP * dx;
              double vy = TRANSLATION_KP * dy;
              double omega = ROTATION_KP * headingError;

              // Distance-based gain boost
              double distanceError = Math.hypot(dx, dy);

              if (distanceError > AGGRESSIVE_DISTANCE_METERS) {
                vx *= FAR_GAIN_MULTIPLIER;
                vy *= FAR_GAIN_MULTIPLIER;
              }

              // Min Speed Snap
              if (Math.abs(dx) > POSITION_TOLERANCE_METERS && Math.abs(vx) < MIN_LINEAR_SPEED_MPS) {
                vx = Math.copySign(MIN_LINEAR_SPEED_MPS, vx);
              }

              if (Math.abs(dy) > POSITION_TOLERANCE_METERS && Math.abs(vy) < MIN_LINEAR_SPEED_MPS) {
                vy = Math.copySign(MIN_LINEAR_SPEED_MPS, vy);
              }

              if (Math.abs(Math.toDegrees(headingError)) > ANGLE_TOLERANCE_DEGREES
                  && Math.abs(omega) < MIN_ANGULAR_SPEED_RAD_PER_SEC) {
                omega = Math.copySign(MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
              }

              vx = Math.max(-MAX_LINEAR_SPEED_MPS, Math.min(MAX_LINEAR_SPEED_MPS, vx));
              vy = Math.max(-MAX_LINEAR_SPEED_MPS, Math.min(MAX_LINEAR_SPEED_MPS, vy));
              omega =
                  Math.max(
                      -MAX_ANGULAR_SPEED_RAD_PER_SEC,
                      Math.min(MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

              boolean posAligned =
                  Math.abs(dx) < POSITION_TOLERANCE_METERS
                      && Math.abs(dy) < POSITION_TOLERANCE_METERS;
              boolean rotAligned = Math.abs(Math.toDegrees(headingError)) < ANGLE_TOLERANCE_DEGREES;

              if (posAligned && rotAligned) {
                alignedStableCounter = Math.min(alignedStableCounter + 1, REQUIRED_STABLE_CYCLES);
                drivebase.drive(new ChassisSpeeds(0, 0, 0));
              } else {
                alignedStableCounter = 0;
                ChassisSpeeds speeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation());
                drivebase.drive(speeds);
              }

              SmartDashboard.putNumber("Vision/TargetTagID", tag.ID);
              SmartDashboard.putBoolean(
                  "Vision/Aligned", alignedStableCounter >= REQUIRED_STABLE_CYCLES);
              SmartDashboard.putNumber("Vision/HeadingErrorDeg", Math.toDegrees(headingError));
            })
        .finallyDo(interrupted -> drivebase.drive(new ChassisSpeeds(0, 0, 0))) // Stop when done
        .until(() -> alignedStableCounter >= REQUIRED_STABLE_CYCLES);
  }

  public Command rotateToAllianceTagWhileDriving(
      DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {

    return this.run(
        () -> {
          Pose2d robotPose = drivebase.getPose();
          Optional<Translation2d> maybeCenter = getScoringCenter();

          double vx = vxSupplier.getAsDouble();
          double vy = vySupplier.getAsDouble();

          // If we can’t compute center, just drive normally
          if (maybeCenter.isEmpty()) {
            drivebase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, robotPose.getRotation()));
            return;
          }

          Translation2d center = maybeCenter.get();

          // Vector from robot to the scoring center
          Translation2d toCenter = center.minus(robotPose.getTranslation());

          // Shooter faces the center
          Rotation2d desiredHeading =
              new Rotation2d(Math.atan2(toCenter.getY(), toCenter.getX()))
                  .plus(Rotation2d.fromDegrees(180)); // Plus 180 for back of robot to face center

          double headingError = desiredHeading.minus(robotPose.getRotation()).getRadians();
          headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

          // Strafing compensation (keeps shots centered while moving, mostly)
          double strafeComp = vy * 1;
          if (Math.abs(vx) < 0.01 && Math.abs(vy) < 0.01) {
            strafeComp = 0.0;
          }

          double omega = (ROTATION_KP * headingError + strafeComp) * 1.8;

          // Minimum rotation snap
          if (Math.abs(Math.toDegrees(headingError)) > ANGLE_TOLERANCE_DEGREES
              && Math.abs(omega) < MIN_ANGULAR_SPEED_RAD_PER_SEC) {
            omega = Math.copySign(MIN_ANGULAR_SPEED_RAD_PER_SEC, omega);
          }

          omega =
              Math.max(
                  -MAX_ANGULAR_SPEED_RAD_PER_SEC, Math.min(MAX_ANGULAR_SPEED_RAD_PER_SEC, omega));

          // The robot has the taste of freedom now
          drivebase.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robotPose.getRotation()));

          // Mainly for debugging purposes if this robot rotates to Timbuktu
          SmartDashboard.putNumber("Vision/ScoringCenterErrorDeg", Math.toDegrees(headingError));
        });
  }
}

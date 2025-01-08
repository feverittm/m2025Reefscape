package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.config.SmartMotionConfigAccessor;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Camera
{
    private PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private PhotonPoseEstimator photonPoseEstimator;

    private List<PhotonPipelineResult> results;

    public Camera(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);

        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

        this.results = null;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        this.results = this.camera.getAllUnreadResults();
        SmartDashboard.putNumber("result size", this.results.size());
        if (!this.results.isEmpty())
        {
            for (PhotonPipelineResult result: results)
            {
                
                Optional<EstimatedRobotPose> estimatedRobotPose = this.photonPoseEstimator.update(result);
                if (result.hasTargets())
                {
                    SmartDashboard.putNumber("has result", 1);
                    SmartDashboard.putNumber("estimated x", estimatedRobotPose.orElseThrow().estimatedPose.toPose2d().getX());
                    SmartDashboard.putNumber("estimated Y", estimatedRobotPose.orElseThrow().estimatedPose.toPose2d().getY());
                    SmartDashboard.putNumber("estimated Theta", estimatedRobotPose.orElseThrow().estimatedPose.toPose2d().getRotation().getDegrees());
                } else
                {
                    SmartDashboard.putNumber("has result", 0);
                }
                if (estimatedRobotPose.isPresent())
                {
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.orElseThrow().estimatedPose.toPose2d(), result.getTimestampSeconds());
                }
            }
        } 
    }

    public int getTagId()
    {
        if (!this.results.isEmpty())
        {
            var target = results.get(0).getBestTarget();
            if (target != null)
            {
                return target.getFiducialId();
            }
        }
        return 0;
    }

    public boolean hasTarget()
    {
        if (!this.results.isEmpty())
        {
            if (results.get(0).getBestTarget() != null)
            {
                return true;
            }
        }
        return false;
    }

    public Translation2d robot_to_tag(Drivebase drivebase)
    {
        if (!this.results.isEmpty())
        {
            PhotonTrackedTarget target = results.get(0).getBestTarget();
            if (target != null)
            {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent())
                {
                    return PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.getDistanceToPose(drivebase.getPose(), tagPose.orElseThrow().toPose2d()), Rotation2d.fromDegrees(-target.getYaw()));
                }
            }
        }
        return new Translation2d();
    }

    public Pose2d get_tag_pose2d()
    {
        if (!this.results.isEmpty())
        {
            PhotonTrackedTarget target = results.get(0).getBestTarget();
            if (target != null)
            {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent())
                {
                    return tagPose.orElseThrow().toPose2d();
                }
            }
        }
        return new Pose2d();
    }
}
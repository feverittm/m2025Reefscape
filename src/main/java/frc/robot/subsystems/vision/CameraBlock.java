package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraBlock 
{
    List<Camera> cameraList;

    public CameraBlock(List<Camera> cameraList)
    {
        this.cameraList = cameraList;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        //for (Camera camera: this.cameraList)
        //{
        //    camera.update(poseEstimator);
        //}
        cameraList.get(0).update(poseEstimator);
    }

    public void troubleShoot()
    {
        SmartDashboard.putNumber("Tag ID", this.cameraList.get(0).getTagId());
    }
}

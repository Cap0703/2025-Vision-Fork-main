package frc.robot.Subsystems;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    static final Set<Integer> redTargets = new HashSet<>(Arrays.asList(15, 5, 3, 2, 1, 6, 7, 8, 9, 10, 11));
    static final Set<Integer> blueTargets = new HashSet<>(Arrays.asList(4, 14, 16, 12, 13, 17, 18, 19, 20, 21, 22));
    //static final Set<Integer> shootingTargets = new HashSet<>(Arrays.asList(3,4,7,8));
    public static AprilTagFieldLayout loadAprilTagFieldLayout(String resourceFile) { 
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile); 
        InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) { 
            ObjectMapper mapper = new ObjectMapper(); return mapper.readValue(isr, AprilTagFieldLayout.class); } 
            catch (IOException e) { 
                throw new UncheckedIOException(e); } } 
            public AprilTagFieldLayout aprilTagFieldLayout = loadAprilTagFieldLayout("/edu/wpi/first/apriltag/2025-reefscape.json");
   
    public enum DetectedAlliance {RED,BLUE,NONE};

    public DetectedAlliance getAllianceStatus() {
        var result = getCamResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        var redTargetCount = 0;
        var blueTargetCount = 0;

        for (PhotonTrackedTarget target : targets) {
            if (redTargets.contains(target.getFiducialId())) {
                redTargetCount += 1;
            }
           if (blueTargets.contains(target.getFiducialId())) {
                blueTargetCount += 1;
            }
        }

        if (redTargetCount > blueTargetCount && redTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.RED;
        } else if (blueTargetCount > redTargetCount && blueTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.BLUE;
        } else return DetectedAlliance.NONE;
    }

    public Pose3d get3dPose() {
        var result = getCamResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Optional<Pose3d> optionalPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

            Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), optionalPose.get(), VisionConstants.cameraToRobot);
            return(cameraRobotPose);
        } else { 
            return null;
        }
  
    }

    public PhotonPipelineResult getCamResult(){
        List<PhotonPipelineResult> results = camera.getAllUnreadResults(); 
        if (results.isEmpty()) { 
            return new PhotonPipelineResult(); // Return an empty result if no results are available 
            } 
        return results.get(results.size() - 1);
    }

    public boolean hasTarget() {
        var result = getCamResult();
        return result.hasTargets();
    }

    public double getCamTimeStamp() {
        var imageCaptureTime = getCamResult().getTimestampSeconds();
        return imageCaptureTime;
    }

    /*public boolean isNearShooter(){
        var result = camera.getLatestResult();
         if (result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();

            if (shootingTargets.contains(target.getFiducialId())) {
                return true;
            } else { 
                return false;
            }

        } else { return false; }
    }
   */
    @Override
    public void periodic() {
    }
}

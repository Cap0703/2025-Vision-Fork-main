package frc.robot.Subsystems;

// Importing necessary libraries for working with the camera, photon vision, JSON parsing, and various utilities.
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

// The Vision class extends SubsystemBase, meaning it is a subsystem for a robot, handling vision processing tasks.
public class Vision extends SubsystemBase {

    // Creates an instance of the PhotonCamera class to interface with the camera.
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    // Sets of fiducial IDs representing targets for different alliances (Red and Blue).
    static final Set<Integer> redTargets = new HashSet<>(Arrays.asList(15, 5, 3, 2, 1, 6, 7, 8, 9, 10, 11));
    static final Set<Integer> blueTargets = new HashSet<>(Arrays.asList(4, 14, 16, 12, 13, 17, 18, 19, 20, 21, 22));

    // A method that loads an AprilTag field layout from a given JSON file (providing tag locations on the field).
    public static AprilTagFieldLayout loadAprilTagFieldLayout(String resourceFile) { 
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile); 
        InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) { 
            ObjectMapper mapper = new ObjectMapper(); 
            return mapper.readValue(isr, AprilTagFieldLayout.class); 
        } catch (IOException e) { 
            throw new UncheckedIOException(e); 
        } 
    }

    // Loads the field layout for the 2025 'reefscape' competition field from a JSON file.
    public AprilTagFieldLayout aprilTagFieldLayout = loadAprilTagFieldLayout("/edu/wpi/first/apriltag/2025-reefscape.json");

    // Enum to represent the alliance that is detected (RED, BLUE, or NONE).
    public enum DetectedAlliance {RED, BLUE, NONE};

    // Method to determine the current alliance based on the number of red and blue targets detected.
    public DetectedAlliance getAllianceStatus() {
        var result = getCamResult(); // Get the latest camera result.
        List<PhotonTrackedTarget> targets = result.getTargets(); // Get a list of targets from the result.
        var redTargetCount = 0;
        var blueTargetCount = 0;

        // Loop through all targets and count how many belong to each alliance (red or blue).
        for (PhotonTrackedTarget target : targets) {
            if (redTargets.contains(target.getFiducialId())) {
                redTargetCount += 1;
            }
            if (blueTargets.contains(target.getFiducialId())) {
                blueTargetCount += 1;
            }
        }

        // Compare the counts and return the alliance with the most targets detected, above a threshold.
        if (redTargetCount > blueTargetCount && redTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.RED;
        } else if (blueTargetCount > redTargetCount && blueTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.BLUE;
        } else {
            return DetectedAlliance.NONE; // No alliance detected.
        }
    }

    // Method to estimate the 3D pose (position and orientation) of the robot relative to the field.
    public Pose3d get3dPose() {
        var result = getCamResult(); // Get the latest camera result.
        if (result.hasTargets()) { // If targets are detected:
            PhotonTrackedTarget target = result.getBestTarget(); // Get the best (most likely) target.
            Optional<Pose3d> optionalPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()); // Get the pose of the target.

            // Use the camera's transformation data and field layout to estimate the robot's position.
            Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), optionalPose.get(), VisionConstants.cameraToRobot);
            return cameraRobotPose; // Return the estimated robot pose.
        } else { 
            return null; // If no targets are detected, return null (no pose).
        }
    }

    // Method to get the latest camera result.
    public PhotonPipelineResult getCamResult() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults(); // Get all unread results from the camera.
        if (results.isEmpty()) { 
            return new PhotonPipelineResult(); // If no results are available, return an empty result.
        } 
        return results.get(results.size() - 1); // Return the most recent result.
    }

    // Method to check if there is at least one target detected.
    public boolean hasTarget() {
        var result = getCamResult(); // Get the latest camera result.
        return result.hasTargets(); // Return true if targets are detected, otherwise false.
    }

    // Method to get the timestamp of the most recent camera result (time of image capture).
    public double getCamTimeStamp() {
        var imageCaptureTime = getCamResult().getTimestampSeconds(); // Get the timestamp of the result.
        return imageCaptureTime; // Return the timestamp.
    }

    // Periodic method that is called periodically to update subsystem state (currently empty).
    @Override
    public void periodic() {
    }
}

package frc.robot.Commands.swerve;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Vision.DetectedAlliance;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.VisionConstants;

public class VisionAlign extends Command {
    Vision vision;
    Pose3d visionPose;
    Pose3d targetPose;
    double alignmentThreshold = 0.2;  // Adjust this value as needed for distance tolerance

    public VisionAlign(Vision vision) {
        this.vision = vision;
    }

    Pose3d cameraRobotPose;

    @Override
    public void initialize() {
        // Initialize pose or any required values here if needed.
    }

    @Override
    public void execute() {
        // Ensure target exists
        PhotonTrackedTarget target = vision.getCamResult().getBestTarget();
        if (target != null) {
            // Retrieve AprilTag pose and estimate robot pose
            if (vision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose3d tagPose = vision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
                cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(),
                        tagPose,
                        VisionConstants.cameraToRobot
                );
            }
        }

        // Update vision pose
        visionPose = vision.get3dPose();

        // Display detected alliance
        SmartDashboard.putString("Detected Alliance", vision.getAllianceStatus() == DetectedAlliance.BLUE ? "BLUE"
                : vision.getAllianceStatus() == DetectedAlliance.RED ? "RED" : "None");

        // Display alignment info on the dashboard
        
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup or reset logic if necessary.
    }

}

package frc.robot.Commands.swerve;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Vision;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionAlign extends Command {
    private final Vision vision; // Reference to the Vision subsystem
    private final SwerveSubsystem swerveSubsystem; // Reference to the Swerve subsystem for driving

    private Pose3d cameraRobotPose; // The pose of the robot relative to the field, based on target detection
    private final double alignmentThreshold = 0.2;  // Threshold for distance tolerance
    private final double angleThreshold = 2.0;      // Angle tolerance in degrees to be considered aligned

    // Constructor that takes the Vision and Swerve subsystems
    public VisionAlign(Vision vision, SwerveSubsystem swerveSubsystem) {
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void initialize() {
        // Optionally, initialize values, or reset the odometry if needed
    }

    @Override
    public void execute() {
        // Ensure that the target exists
        PhotonTrackedTarget target = vision.getCamResult().getBestTarget();
        if (target != null) {
            // Retrieve the pose of the target if it's available in the field layout
            if (vision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose3d tagPose = vision.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
                
                // Estimate the robot's pose using the camera's data and the target's pose
                cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(), // Camera to target transform
                        tagPose, // Target pose on the field
                        VisionConstants.cameraToRobot // Robot to camera transform
                );
            }

            // Calculate the distance from the robot to the target by computing the Euclidean distance
            double distance = cameraRobotPose.getTranslation().getDistance(target.getBestCameraToTarget().getTranslation());
            SmartDashboard.putNumber("Distance to Target (meters)", distance);
            SmartDashboard.putString("Target ID", "ID: " + target.getFiducialId());

            // Calculate the angle to the target and rotate the robot to face it
            double targetAngle = cameraRobotPose.getRotation().getDegrees(); // Get the angle to target from the pose
            Rotation2d desiredRotation = Rotation2d.fromDegrees(targetAngle);

            // Get the robot's current pose to calculate how much to rotate
            double currentAngle = vision.get3dPose().getRotation().getDegrees();
            double angleDifference = targetAngle - currentAngle;

            // Control logic for alignment
            boolean aligned = Math.abs(angleDifference) <= angleThreshold; // Check if aligned in angle
            boolean withinDistance = distance <= alignmentThreshold; // Check if close enough to the target

            if (!aligned || !withinDistance) {
                // If not aligned or too far, we need to adjust both rotation and position
                double rotationSpeed = aligned ? 0 : (angleDifference > 0 ? 0.3 : -0.3); // Adjust rotation speed
                double forwardSpeed = withinDistance ? 0 : (distance > 1.0 ? 0.5 : 0.2); // Adjust forward speed

                // Send the drive commands to the swerve system
                swerveSubsystem.drive(forwardSpeed, 0.0, rotationSpeed); // Move forward/backward and rotate
            } else {
                // If we are aligned and within distance, stop the robot
                swerveSubsystem.drive(0.0, 0.0, 0.0); // Stop the robot
            }
        } else {
            // No target detected, stop the robot
            swerveSubsystem.drive(0.0, 0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerveSubsystem.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // The command finishes when the robot is both aligned and within the distance threshold
        return vision.get3dPose() != null && 
               Math.abs(cameraRobotPose.getRotation().getDegrees() - vision.get3dPose().getRotation().getDegrees()) <= angleThreshold &&
               cameraRobotPose.getTranslation().getDistance(vision.get3dPose().getTranslation()) <= alignmentThreshold;
    }
}

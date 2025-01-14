package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final double targetDistance = 0.5;  // Target distance to maintain from the AprilTag in meters
    private final double alignmentTolerance = 0.05; // Horizontal alignment tolerance (meters)
    private final double distanceTolerance = 0.1;  // Distance tolerance (meters)

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0)); // Stop any previous movement
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            Pose2d aprilTagPose = vision.get2dPose(); // Assuming Vision.get2dPose() returns a Pose2d

            if (aprilTagPose != null) {
                double tagX = aprilTagPose.getX();  // Horizontal position of the AprilTag
                double tagY = aprilTagPose.getY();  // Forward/backward distance to the AprilTag

                // Compute alignment error along X (horizontal) and Y (distance)
                double offsetX = tagX; // Offset for alignment
                double offsetY = tagY - targetDistance; // Offset for maintaining distance

                // Determine the alignment and distance correction speeds
                double alignmentSpeed = Math.abs(offsetX) > alignmentTolerance ? Math.signum(offsetX) * 0.5 : 0;
                double distanceSpeed = Math.abs(offsetY) > distanceTolerance ? Math.signum(offsetY) * 0.3 : 0;

                // Create and send the swerve request
                SwerveRequest swerveRequest = new SwerveRequest.FieldCentric()
                        .withVelocityX(distanceSpeed) // Forward/backward (Y-axis in field-centric terms)
                        .withVelocityY(alignmentSpeed); // Lateral (X-axis in field-centric terms)

                drivetrain.setControl(swerveRequest);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Terminate if aligned and at the correct distance
        if (vision.hasTarget()) {
            Pose2d aprilTagPose = vision.get2dPose();
            if (aprilTagPose != null) {
                double tagX = aprilTagPose.getX();
                double tagY = aprilTagPose.getY();
                return Math.abs(tagX) < alignmentTolerance &&
                       Math.abs(tagY - targetDistance) < distanceTolerance;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0));
    }

    @Override
    public boolean runsWhenDisabled() {
        return false; // Do not run if the robot is disabled
    }
}

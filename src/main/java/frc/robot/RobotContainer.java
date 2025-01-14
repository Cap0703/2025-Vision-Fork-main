// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.autos.BlueLeftCoral;
import frc.robot.commands.autos.MiddleCoral;
import frc.robot.commands.autos.Test;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final Vision vision = new Vision();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);


            
        

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();    

    /* Path follower */
    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


    private final AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivetrain, vision);
    private final SwerveRequest.FieldCentricFacingAngle autoAlign = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);


    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();

    public RobotContainer() {
        configureBindings();
        initializeAutoChooser();

        faceAngle.HeadingController.setP(3); // Proportional gain
        faceAngle.HeadingController.setI(0.0); // Integral gain
        faceAngle.HeadingController.setD(0.01); // Derivative gain
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left) .
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> faceAngle
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(vision.getDegreesToGamePiece())
        ));
        joystick.x().whileTrue(drivetrain.applyRequest(() -> faceAngle
            .withVelocityX(0)
            .withVelocityY(0)
            .withTargetDirection(vision.getDegreesToGamePiece())
        ));


        

        //joystick.b().whileTrue((drivetrain.applyRequest(()-> faceDirection.withTargetDirection(vision.getDegreesToGamePiece()))));
        //joystick.b().whileTrue(drivetrain.applyRequest(() -> faceDirection.withTargetDirection(test)));
        //WILL NEED TO PID THE COMMAND ABOVE

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.2 * MaxSpeed).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.2 * MaxSpeed))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.2 * MaxSpeed).withVelocityY(0))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.2 * MaxSpeed))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void initializeAutoChooser(){
        autoChooser.setDefaultOption("1 Middle Coral",new MiddleCoral().middleCoral());
        autoChooser.addOption("Test", new Test().test());
        autoChooser.addOption("Blue Coral Left 3", new BlueLeftCoral().blueLeftCoral());
        // each auto
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
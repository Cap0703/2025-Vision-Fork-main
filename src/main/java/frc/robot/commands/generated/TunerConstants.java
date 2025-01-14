package frc.robot.commands.generated;

import frc.robot.Constants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(48).withKI(0).withKD(0.2)  
        .withKS(0).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(10.82);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 5.901;
    private static final double kSteerGearRatio = 12.8;
    private static final Distance kWheelRadius = Inches.of(4);

    private static final int kPigeonId = 10;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            Constants.CanId.Front_Left_Turn, Constants.CanId.Front_Left_Drive, Constants.CanId.Front_Left_CANcoder, Constants.SwerveConstants.kFrontLeftEncoderOffset,
            Constants.SwerveConstants.kFrontLeftXPos, Constants.SwerveConstants.kFrontLeftYPos, Constants.SwerveConstants.kInvertLeftSide, Constants.SwerveConstants.kFrontLeftSteerMotorInverted, Constants.SwerveConstants.kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            Constants.CanId.Front_Right_Turn, Constants.CanId.Front_Right_Drive, Constants.CanId.Front_Right_CANcoder, Constants.SwerveConstants.kFrontRightEncoderOffset,
            Constants.SwerveConstants.kFrontRightXPos, Constants.SwerveConstants.kFrontRightYPos, Constants.SwerveConstants.kInvertRightSide, Constants.SwerveConstants.kFrontRightSteerMotorInverted, Constants.SwerveConstants.kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            Constants.CanId.Back_Left_Turn, Constants.CanId.Back_Left_Drive, Constants.CanId.Back_Left_CANcoder, Constants.SwerveConstants.kBackLeftEncoderOffset,
            Constants.SwerveConstants.kBackLeftXPos, Constants.SwerveConstants.kBackLeftYPos, Constants.SwerveConstants.kInvertLeftSide, Constants.SwerveConstants.kBackLeftSteerMotorInverted, Constants.SwerveConstants.kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            Constants.CanId.Back_Right_Turn, Constants.CanId.Back_Right_Drive, Constants.CanId.Back_Right_CANcoder, Constants.SwerveConstants.kBackRightEncoderOffset,
            Constants.SwerveConstants.kBackRightXPos, Constants.SwerveConstants.kBackRightYPos, Constants.SwerveConstants.kInvertRightSide, Constants.SwerveConstants.kBackRightSteerMotorInverted, Constants.SwerveConstants.kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program,.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }


    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }

        private final SwerveModule<TalonFX, TalonFX, CANcoder> frontLeft = getModule(0);
        private final SwerveModule<TalonFX, TalonFX, CANcoder> frontRight = getModule(1);
        private final SwerveModule<TalonFX, TalonFX, CANcoder> backLeft = getModule(2);
        private final SwerveModule<TalonFX, TalonFX, CANcoder>backRight = getModule(3);

        private final TalonFX frontLeftDrive = frontLeft.getDriveMotor();
        private final TalonFX frontRightDrive = frontRight.getDriveMotor();
        private final TalonFX backLeftDrive = backLeft.getDriveMotor();
        private final TalonFX backRightDrive = backRight.getDriveMotor();

        // private final TalonFX frontLeftSteer = frontLeft.getSteerMotor();
        // private final TalonFX frontRightSteer = frontLeft.getSteerMotor();
        // private final TalonFX backLeftSteer = backLeft.getSteerMotor();
        // private final TalonFX backRightSteer = backRight.getSteerMotor();

        private final CANcoder frontLeftEncoder = (CANcoder) frontLeft.getEncoder();
        private final CANcoder frontRightEncoder = (CANcoder) frontRight.getEncoder();
        private final CANcoder backLeftEncoder = (CANcoder) backLeft.getEncoder();
        private final CANcoder backRightEncoder = (CANcoder) backRight.getEncoder();

        private final Pigeon2 pigeon = new Pigeon2(CanId.Pigeon, "Canivore");

        private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            SwerveConstants.kDriveKinematics,
            pigeon.getRotation2d(), 
            new SwerveModulePosition[]{
                new SwerveModulePosition(frontLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(frontLeftEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(frontRightDrive.getPosition().getValueAsDouble(), new Rotation2d(frontRightEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(backLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(backLeftEncoder.getAbsolutePosition().getValue())),
                new SwerveModulePosition(backRightDrive.getPosition().getValueAsDouble(), new Rotation2d(backRightEncoder.getAbsolutePosition().getValue()))
            }, new Pose2d(0, 0, new Rotation2d()) // Initial pose
            );

        public SwerveDriveOdometry getOdometer(){
            return odometer;
        }

        public Pose2d getPose2d(){
            return odometer.getPoseMeters();
        }

        public void resetOdometer(Pose2d pose) {
            odometer.resetPosition(
                pigeon.getRotation2d(),
                new SwerveModulePosition[]{
                    new SwerveModulePosition(frontLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(frontLeftEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(frontRightDrive.getPosition().getValueAsDouble(), new Rotation2d(frontRightEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(backLeftDrive.getPosition().getValueAsDouble(), new Rotation2d(backLeftEncoder.getAbsolutePosition().getValue())),
                    new SwerveModulePosition(backRightDrive.getPosition().getValueAsDouble(), new Rotation2d(backRightEncoder.getAbsolutePosition().getValue()))
                },
                pose
            );
        }

        public ChassisSpeeds getRobotRelativeSpeeds() {
            return getState().Speeds; 
        }
    }
}
package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    
    public static final class CanId { 

        public static final int Front_Left_Drive = 2;
        public static final int Front_Right_Drive = 3;
        public static final int Back_Left_Drive = 4;
        public static final int Back_Right_Drive = 5;
        // drive motors - order - start top left in clockwise rotation

        public static final int Front_Left_Turn = 6;
        public static final int Front_Right_Turn = 7;
        public static final int Back_Left_Turn = 8;
        public static final int Back_Right_Turn = 9;
        // turn motors - order - start top left in clockwise rotation

        public static final int Pigeon = 10;

        public static final int Front_Left_CANcoder = 11;
        public static final int Front_Right_CANcoder = 12;
        public static final int Back_Left_CANcoder = 13;
        public static final int Back_Right_CANcoder = 14;
    }

    public static final class InputConstants {
        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class SwerveConstants {
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        //Front Left
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.427734375);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final Distance kFrontLeftXPos = Inches.of(12.375);
        public static final Distance kFrontLeftYPos = Inches.of(12.375);

        //Front Right
        public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.444580078125);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final Distance kFrontRightXPos = Inches.of(12.375);
        public static final Distance kFrontRightYPos = Inches.of(-12.375);

        //Back Left
        public static final Angle kBackLeftEncoderOffset = Rotations.of(0.3447265625);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;
    
        public static final Distance kBackLeftXPos = Inches.of(-12.375);
        public static final Distance kBackLeftYPos = Inches.of(12.375);

        //Back Right
        public static final Angle kBackRightEncoderOffset = Rotations.of(-0.191162109375);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        public static final Distance kBackRightXPos = Inches.of(-12.375);
        public static final Distance kBackRightYPos = Inches.of(-12.375);
    }
}

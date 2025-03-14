package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public class DrivetrainConstants {
        public static final double kMaxSpeed = 5.41;
        public static final double kMaxAngularRate = kMaxSpeed * 39.37 / 20.75 * Math.PI;
    }
    public class OperatorConstants {
        public static final int kL4 = 1;
        public static final int kL3 = 2;
        public static final int kL2 = 3;
        public static final int kL1 = 4;
        public static final int kAutoAlignLeft = 9;
        public static final int kAutoAlignRight = 8;
        public static final int k0degrees = 10;
        public static final int k60degrees = 12;
        public static final int k120degrees = 6;
        public static final int k180degrees = 5;
        public static final int k240degrees = 7;
        public static final int k300degrees = 11;
    }
    public class KnuckleConstants {
        public static final int kMotorID = 5;
        // public static final double kCurrentThreshold = 75;
        public static final double kHighSpeed = 1.;
        public static final double kLowSpeed = 0.05;
    }
    public class ClimberConstants {
        public static final int kMotorOneID = 8;
        public static final int kMotorTwoID = 9;
        public static final double k90DegreesRotations = 86.19395446777344;
    }
    public class ChuteConstants {
        public static final int kMotorID = 8;
        public static final double kCurrentThreshold = 30;
        public static final int kBeamBreakPort = 0;
    }
    public class HopperConstants {
        public static final int kBeltMotorID = 7;
        public static final int kWheelMotorID = 6;
        public static final int kBeamBreakPort = 1;
    }
    public class AlgaeScorerConstants{
        public static final int kMotorID = 4;
        public static final double kCurrentThreshold = 42.5;
        
    }
    public class ElevatorConstants{
        public static final int kTopMotorID = 1;
        public static final int kBottomMotorID = 2;
        // public static final int kDownLimitPort = 0;
        // public static final int kUpLimitPort = 0;
        public static final double kP = 0.;
        public static final double kI = 0.00;
        public static final double kD = 0.;
        public static final double kPDynamic = 0.;
        public static final double kIDynamic = 0.0;
        public static final double kDDynamic = 0.;
        public static final double kElevL1 = 0.64;
        public static final double kElevL2 = 0.105; 
        public static final double kElevL3 = 0.39;
        public static final double kElevL4 = 0.925;
        public static final double kElevTran = 0.38;
    }
    public class ArmConstants{
        public static final int kMotorID = 3;
        public static final int kEncoderPort = 0;
        public static final double kP = 0.18;
        public static final double kI = 0.016;
        public static final double kD = 0;
        public static final double kTransferAngle = 0;
        public static final double kPDynamic = 0.006;
        public static final double kIDynamic = 0.0;
        public static final double kDDynamic = 0.00035;
        public static final double kEncoderOffset = 0.2515;
        public static final double kArmL1 = -0.08;
        public static final double kArmL2 = 0.1849;
        public static final double kArmL3 = 0.1849;
        public static final double kArmL4 = 0.155;
        public static final double kArmTran = -0.24;
    }
    public class ReefPoses {
        public static final Pose2d kRED0_1 = new Pose2d(11.19, 4.25, Rotation2d.fromDegrees(0));
        public static final Pose2d kRED2_3 = new Pose2d(11.78289794921875 , 6.251760005950928 , Rotation2d.fromDegrees(-60));
        public static final Pose2d kRED4_5 = new Pose2d(14.53, 6.17, Rotation2d.fromDegrees(-120));
        public static final Pose2d kRED6_7 = new Pose2d(15, 4, Rotation2d.fromDegrees(180));
        public static final Pose2d kRED8_9 = new Pose2d(14.234933853149414, 2.207791328430176, Rotation2d.fromDegrees(120));
        public static final Pose2d kRED10_11 = new Pose2d(12.001603126525879, 2.264570951461792, Rotation2d.fromDegrees(60));

        public static final Pose2d kREDSOURCERIGHT_center = new Pose2d(16.17, 7.1, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_bargeWall = new Pose2d(16.5233268737793, 6.850026588439941, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_operatorWall_shifted = new Pose2d(16.50611686706543, 6.901570796966553, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_operatorWall = new Pose2d(16.87, 6.516, Rotation2d.fromDegrees(-128));
        
        public static final Pose2d kREDSOURCELEFT_center = new Pose2d(16.42, 1, Rotation2d.fromDegrees(128));
        public static final Pose2d kREDSOURCELEFT_bargeWall = new Pose2d(15.72, 0.85, Rotation2d.fromDegrees(128));
        public static final Pose2d kREDSOURCELEFT_operatorWall = new Pose2d(16.8, 1.52, Rotation2d.fromDegrees(128));

        public static final Pose2d kBLUE0_1 = new Pose2d(6.35, 4.06, Rotation2d.fromDegrees(180)); 
        public static final Pose2d kBLUE2_3 = new Pose2d(5.69, 1.93, Rotation2d.fromDegrees(120));
        public static final Pose2d kBLUE4_5 = new Pose2d(3.23, 1.91, Rotation2d.fromDegrees(60));
        public static final Pose2d kBLUE6_7 = new Pose2d(2.74, 4.02, Rotation2d.fromDegrees(0));
        public static final Pose2d kBLUE8_9 = new Pose2d(3.59, 5.56, Rotation2d.fromDegrees(-120));
        public static final Pose2d kBLUE10_11 = new Pose2d(5.41, 5.69, Rotation2d.fromDegrees(-60));

        public static final Pose2d kBLUESOURCERIGHT_center = new Pose2d(1.35, 0.95, Rotation2d.fromDegrees(52));
        public static final Pose2d kBLUESOURCERIGHT_bargeWall = new Pose2d(1.76, 0.73, Rotation2d.fromDegrees(52));
        public static final Pose2d kBLUESOURCERIGHT_operatorWall = new Pose2d(0.73, 1.43, Rotation2d.fromDegrees(52));

        public static final Pose2d kBLUESOURCELEFT_center = new Pose2d(1.34, 7.01, Rotation2d.fromDegrees(-52));
        public static final Pose2d kBLUESOURCELEFT_operatorWall = new Pose2d(1.76, 7.36, Rotation2d.fromDegrees(-52));
        public static final Pose2d kBLUESOURCELEFT_bargeWall = new Pose2d(1.61, 7.49, Rotation2d.fromDegrees(-52));

        public static final PathConstraints K_CONSTRAINTS_Fastest = new PathConstraints(5.41, 5., 3*Math.PI, 3*Math.PI);
    }
}

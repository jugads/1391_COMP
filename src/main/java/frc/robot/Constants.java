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
        public static final int kL4 = 8;
        public static final int kL3 = 9;
        public static final int kL2 = 11;
        public static final int kL1 = 6;
        public static final int kAutoAlignLeft = 10;
        public static final int kAutoAlignRight = 2;
        public static final int kAL3 = 7;
        public static final int kAL2 = 12;
        public static final int kNET = 4;
        public static final int kProcs = 3;
        public static final int kT = 5;
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
        public static final double kElevL1 = 0.57;
        public static final double kElevL2 = 0.16; 
        public static final double kElevL3 = 0.49;
        public static final double kElevL4 = 0.94;
        public static final double kElevTran = 0.38; //0.3775
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
        public static final double kEncoderOffset = 0.86;
        public static final double kArmL1 = -0.055;
        public static final double kArmL2 = 0.175;
        public static final double kArmL3 = 0.15;
        public static final double kArmL4 = 0.1475;
        public static final double kArmTran = -0.24;
    }
    public class ReefPoses {
        public static final Pose2d kRED0_1 = new Pose2d(11.19, 4.25, Rotation2d.fromDegrees(0));
        public static final Pose2d kRED2_3 = new Pose2d(11.92 , 6.41, Rotation2d.fromDegrees(-60));
        public static final Pose2d kRED4_5 = new Pose2d(14.46, 6.51, Rotation2d.fromDegrees(-120));
        public static final Pose2d kRED6_7 = new Pose2d(15.47, 4, Rotation2d.fromDegrees(180));
        public static final Pose2d kRED8_9 = new Pose2d(14.44, 1.67, Rotation2d.fromDegrees(120));
        public static final Pose2d kRED10_11 = new Pose2d(11.99, 1.85, Rotation2d.fromDegrees(60));
        public static final Pose2d kRED10_11_ALGAE = new Pose2d(12.257259368896484, 2.6072371006011963, Rotation2d.fromDegrees(60));
        public static final Pose2d kREDBarge = new Pose2d(10.1, 3.2843942642211914, Rotation2d.fromDegrees(-180));

        public static final Pose2d kREDSOURCERIGHT_center = new Pose2d(16.42, 7.06, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_bargeWall = new Pose2d(15.93, 7.46, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_operatorWall_shifted = new Pose2d(16.50611686706543, 6.901570796966553, Rotation2d.fromDegrees(-128));
        public static final Pose2d kREDSOURCERIGHT_operatorWall = new Pose2d(16.87, 6.516, Rotation2d.fromDegrees(-128));
        
        public static final Pose2d kREDSOURCELEFT_center = new Pose2d(16.42, 0.97, Rotation2d.fromDegrees(128));
        public static final Pose2d kREDSOURCELEFT_bargeWall = new Pose2d(15.86, 0.55, Rotation2d.fromDegrees(128));
        public static final Pose2d kREDSOURCELEFT_operatorWall = new Pose2d(16.61, 1.3, Rotation2d.fromDegrees(128));

        public static final Pose2d kBLUE0_1 = new Pose2d(6.35, 4.06, Rotation2d.fromDegrees(180)); 
        public static final Pose2d kBLUE2_3 = new Pose2d(5.68, 1.72, Rotation2d.fromDegrees(120));
        public static final Pose2d kBLUE4_5 = new Pose2d(3.1, 1.75, Rotation2d.fromDegrees(60));
        public static final Pose2d kBLUE6_7 = new Pose2d(2.1, 4.08, Rotation2d.fromDegrees(0));
        public static final Pose2d kBLUE8_9 = new Pose2d(3.24, 6.34, Rotation2d.fromDegrees(-60));
        public static final Pose2d kBLUE10_11 = new Pose2d(5.49, 6.29, Rotation2d.fromDegrees(-120));
        public static final Pose2d kBLUE10_11_ALGAE = new Pose2d(5.3, 5.61, Rotation2d.fromDegrees(-120));
        public static final Pose2d kBLUEBarge = new Pose2d(7.259877681732178, 5.142665386199951, Rotation2d.fromDegrees(0));
        
        public static final Pose2d kBLUESOURCERIGHT_center = new Pose2d(1.12, 0.93, Rotation2d.fromDegrees(52));
        public static final Pose2d kBLUESOURCERIGHT_bargeWall = new Pose2d(1.57, 0.54, Rotation2d.fromDegrees(52));
        public static final Pose2d kBLUESOURCERIGHT_operatorWall = new Pose2d(0.87, 1.32, Rotation2d.fromDegrees(52));

        public static final Pose2d kBLUESOURCELEFT_center = new Pose2d(1.29, 7.17, Rotation2d.fromDegrees(-52));
        public static final Pose2d kBLUESOURCELEFT_operatorWall = new Pose2d(0.85, 6.67, Rotation2d.fromDegrees(-52));
        public static final Pose2d kBLUESOURCELEFT_bargeWall = new Pose2d(1.64, 7.42, Rotation2d.fromDegrees(-52));

        public static final PathConstraints K_CONSTRAINTS_Fastest = new PathConstraints(5.41, 6., 3*Math.PI, 3*Math.PI);
        public static final PathConstraints K_CONSTRAINTS_Barging = new PathConstraints(2, 2., 3*Math.PI, 3*Math.PI);
    }

    public class AlignmentPoses {
        public static final Pose2d[] kAliRED0_1 = new Pose2d[]{new Pose2d(11.65, 3.77, Rotation2d.fromDegrees(0)), new Pose2d(11.65, 4.23, Rotation2d.fromDegrees(0))};
        public static final Pose2d[] kAliRED2_3 = new Pose2d[]{new Pose2d(12.44, 5.3, Rotation2d.fromDegrees(-60)), new Pose2d(12.09, 5.08, Rotation2d.fromDegrees(-60))};
        public static final Pose2d[] kAliRED4_5 = new Pose2d[]{new Pose2d(13.56, 5.37, Rotation2d.fromDegrees(-120)), new Pose2d(13.96, 5.14, Rotation2d.fromDegrees(-120))};
        public static final Pose2d[] kAliRED6_7 = new Pose2d[]{new Pose2d(14.49, 4.25, Rotation2d.fromDegrees(180)), new Pose2d(14.49, 3.86, Rotation2d.fromDegrees(180))};
        public static final Pose2d[] kAliRED8_9 = new Pose2d[]{new Pose2d(13.70, 2.57, Rotation2d.fromDegrees(60)), new Pose2d(13.99, 2.74, Rotation2d.fromDegrees(60))};
        public static final Pose2d[] kAliRED10_11 = new Pose2d[]{new Pose2d(12.16, 2.72, Rotation2d.fromDegrees(120)), new Pose2d(12.46, 2.55, Rotation2d.fromDegrees(120))};

        public static final Pose2d[] kAliBLUE6_7 = new Pose2d[]{new Pose2d(2.91, 3.85, Rotation2d.fromDegrees(0)), new Pose2d(2.91, 4.18, Rotation2d.fromDegrees(0))};
        public static final Pose2d[] kAliBLUE2_3 = new Pose2d[]{new Pose2d(5.11, 2.56, Rotation2d.fromDegrees(120)), new Pose2d(5.39, 2.75, Rotation2d.fromDegrees(120))};
        public static final Pose2d[] kAliBLUE4_5 = new Pose2d[]{new Pose2d(3.53, 2.7, Rotation2d.fromDegrees(60)), new Pose2d(3.82, 2.54, Rotation2d.fromDegrees(60))};
        public static final Pose2d[] kAliBLUE0_1 = new Pose2d[]{new Pose2d(6.07, 3.85, Rotation2d.fromDegrees(180)), new Pose2d(6.07, 4.18, Rotation2d.fromDegrees(180))};
        public static final Pose2d[] kAliBLUE8_9 = new Pose2d[]{new Pose2d(3.83, 5.47, Rotation2d.fromDegrees(-60)), new Pose2d(3.57, 5.33, Rotation2d.fromDegrees(-60))};
        public static final Pose2d[] kAliBLUE10_11 = new Pose2d[]{new Pose2d(5.43, 5.31, Rotation2d.fromDegrees(-120)), new Pose2d(5.14, 5.47, Rotation2d.fromDegrees(-120))};
    }
}

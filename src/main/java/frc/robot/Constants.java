package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class controllerPorts {
        public static final int driveController = 0;
        public static final int operatorController = 1;
    }

    public static class motorPorts {
        public static final int frontElevatorMotor = 11;
        public static final int backElevatorMotor = 12;

        public static final int indexMotor = 21;
        public static final int photoEye = 19;

        public static final int pivotMotor = 20;
    }   

    public static class elevatorConstants {
        //PID Values
        public static final double p = 0.3;
        public static final double i = 0;
        public static final double d = 0;

        //Elevator encoder positions
        public static final double intake = 0;
        public static final double levelTwo = 4.315;
        public static final double levelThree = 10.315;
        public static final double levelFour = 21.4;

        public static final int currentLimit = 40;
        
        public static final double elevatorTolerance = 0.4; //5 ticks should be pretty small I think

        public static enum ElevatorHeight {
            INTAKE,
            LEVELTWO,
            LEVELTHREE,
            LEVELFOUR
        }
    }

    public static class pivotConstants {
        //PID Values
        public static final double p = 1;
        public static final double i = 0;
        public static final double d = 0;

        public static final double algae = 0.027;
        public static final double coralscore = 0.425;
        public static final double coralintake = 0.516;
        public static final double climb = 0;

        public static final int currentLimit = 40;

        public static enum PivotPosition {
            ALGAE,
            CORALSCORE,
            CORALINTAKE,
            CLIMB
        }
    }

    public class indexConstants {
        public static final double indexSpeed = 0.3;
        public static final int currentLimit = 40;
        public static final double distanceTolerance = 0.1;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 5;

    public static final int kPigeonCanId = 18;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }






}

package frc.robot;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.math.twodim.physics.DCMotorTransmission;
import com.spartronics4915.lib.math.twodim.physics.DifferentialDrive;
import com.spartronics4915.lib.util.Units;

public class Constants
{
    // Motor IDs
    public static final int kDriveLeftMasterId = 3;
    public static final int kDriveLeftFollowerId = 4;
    public static final int kDriveRightMasterId = 1;
    public static final int kDriveRightFollowerId = 2;

    // LIDAR
    public static final double kLIDARBinWidth = 0.01;
    public static final double kLIDARTargetRadius = Units.inchesToMeters(10.5);
    public static final Pose2d kVehicleToLIDARTransform = new Pose2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(-14), Rotation2d.fromDegrees(180));

    // Physical constants
    public static final double kDriveTrackScrubFactor = 1.063; // determine with auto mode
    public static final double kDriveTrackWidth = Units.inchesToMeters(23.75 * kDriveTrackScrubFactor);
    public static final double kDriveWheelDiameter = Units.inchesToMeters(6);
    public static final double kDriveWheelRadius = kDriveWheelDiameter / 2.0;

    public static final double kRobotCenterToForward = Units.inchesToMeters(16.125); // meters
    public static final double kRobotCenterToSide = Units.inchesToMeters(13.75); // meters
    public static final double kRobotLinearInertia = 27.93; // kg (robot's mass)
    public static final double kRobotAngularInertia = 1.7419; // kg m^2 (use the moi auto mode)
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)

    public static final double kDriveRightVIntercept = 0.6815; // V
    public static final double kDriveRightKv = 0.2194; // V per rad/s
    public static final double kDriveRightKa = 0.0340; // V per rad/s^2

    public static final double kDriveLeftVIntercept = 0.6995; // V
    public static final double kDriveLeftKv = 0.2066; // V per rad/s
    public static final double kDriveLeftKa = 0.0107; // V per rad/s^2

    public static final double kDriveVelocityKp = 7.8;
    public static final double kDriveBeta = 5.0; // Inverse meters squared
    public static final double kDriveZeta = 0.4; // Unitless dampening coefficient

    public static final double kNativeUnitsPerRev = 1440.0; // PPR (1440) = CPR (360) * 4 (because quadrature)

    // Offset in meters
    public static final Pose2d kCameraToVehicleTransform = new Pose2d(-0.390525, 0, new Rotation2d());
    public static final double kOdometryCovariance = 0.000001; // TODO: Tune me

    public static final DifferentialDrive kDifferentialDrive = new DifferentialDrive(
        kRobotLinearInertia, kRobotAngularInertia, kRobotAngularDrag,
        kDriveWheelRadius, kDriveTrackWidth / 2,
        new DCMotorTransmission(
            kDriveWheelRadius, kRobotLinearInertia, kDriveLeftVIntercept, kDriveLeftKv, kDriveLeftKa
        ),
        new DCMotorTransmission(
            kDriveWheelRadius, kRobotLinearInertia, kDriveRightVIntercept, kDriveRightKv, kDriveRightKa
        )
    );
}

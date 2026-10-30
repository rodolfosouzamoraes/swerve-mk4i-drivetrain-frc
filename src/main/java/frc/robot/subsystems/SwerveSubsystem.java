package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.FL_DRIVE_ID, SwerveConstants.FL_ANGLE_ID, SwerveConstants.FL_CANCODER_ID, true);
    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.FR_DRIVE_ID, SwerveConstants.FR_ANGLE_ID, SwerveConstants.FR_CANCODER_ID, true);
    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.BL_DRIVE_ID, SwerveConstants.BL_ANGLE_ID, SwerveConstants.BL_CANCODER_ID, true);
    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.BR_DRIVE_ID, SwerveConstants.BR_ANGLE_ID, SwerveConstants.BR_CANCODER_ID, true);

    private final Pigeon2 gyro = new Pigeon2(SwerveConstants.PIGEON_ID);
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        zeroHeading();
        odometry = new SwerveDriveOdometry(
            SwerveConstants.KINEMATICS, getRotation2d(), getModulePositions());
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

        // Discretize corrige o desvio lateral (skew) quando o robô translada e gira ao mesmo tempo
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
    }
}
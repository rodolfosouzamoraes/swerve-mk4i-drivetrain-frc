package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class SwerveConstants {
        public static final int PIGEON_ID = 13; // Substitua pelo ID do seu Pigeon 2

        // Dimensões do chassi (distância entre o centro das rodas em metros)
        public static final double TRACK_WIDTH =0.655; 
        public static final double WHEEL_BASE = 0.74;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),   // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),  // Front Right
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),  // Back Left
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)  // Back Right
        );

        // Especificações físicas MK4i L2
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;

        // Fatores de conversão para o Encoder Interno do SparkMax
        public static final double DRIVE_POSITION_CONVERSION = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
        public static final double ANGLE_POSITION_CONVERSION = (2 * Math.PI) / ANGLE_GEAR_RATIO; // Em radianos

        // Velocidades máximas (L2 com NEO atinge ~4.5 m/s teóricos)
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 2.0;

        // IDs e Offsets (Substitua pelos seus IDs e Offsets em Radianos se necessário, 
        // ou deixe 0.0 caso já tenha gravado o MagnetOffset direto no Tuner X)
        public static final int FL_DRIVE_ID = 8, FL_ANGLE_ID = 7, FL_CANCODER_ID = 12;
        public static final int FR_DRIVE_ID = 2, FR_ANGLE_ID = 1, FR_CANCODER_ID = 9;
        public static final int BL_DRIVE_ID = 4, BL_ANGLE_ID = 3, BL_CANCODER_ID = 10;
        public static final int BR_DRIVE_ID = 6, BR_ANGLE_ID = 5, BR_CANCODER_ID = 11;
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController anglePID;
    private final CANcoder absoluteEncoder;

    public SwerveModule(int driveMotorId, int angleMotorId, int canCoderId, boolean driveInverted) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        angleMotor = new SparkMax(angleMotorId, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(canCoderId);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();

        // Configuração do Motor de Tração (Drive)
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.inverted(driveInverted)
                   .idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(40);
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION)
                           .velocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                              .pid(0.1, 0.0, 0.0);
        
        // Novo padrão REVLib 2026 para FeedForward de velocidade
        driveConfig.closedLoop.feedForward.kV(12.0 / SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configuração do Motor de Direção (Angle) - MK4i exige inversão no giro!
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.inverted(true)
                   .idleMode(IdleMode.kBrake)
                   .smartCurrentLimit(20);
        angleConfig.encoder.positionConversionFactor(SwerveConstants.ANGLE_POSITION_CONVERSION);
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                              .pid(0.5, 0.0, 0.0)
                              .positionWrappingEnabled(true)
                              .positionWrappingInputRange(-Math.PI, Math.PI);
        
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetToAbsolute();
    }

    public void resetToAbsolute() {
        // Espera até 100ms por um pacote fresco do CANcoder antes de ler
        double absolutePositionRot = absoluteEncoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble();
        double absolutePositionRad = absolutePositionRot * 2.0 * Math.PI;
        angleEncoder.setPosition(absolutePositionRad);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(angleEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(angleEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(angleEncoder.getPosition());
        
        desiredState.optimize(currentRotation);
        desiredState.cosineScale(currentRotation);

        drivePID.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        anglePID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
    }
}
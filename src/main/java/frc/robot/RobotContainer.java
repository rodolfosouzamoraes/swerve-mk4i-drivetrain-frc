package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        // Comando padrão de movimentação
        swerve.setDefaultCommand(
            new RunCommand(() -> {
                // Inversão dos eixos do Xbox + Deadband de 10%
                double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(), 0.1) 
                                * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
                double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(), 0.1) 
                                * SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
                double rot = -MathUtil.applyDeadband(driverController.getRightX(), 0.1) 
                             * SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

                swerve.drive(xSpeed, ySpeed, rot, true); // true = Field-Oriented
            }, swerve)
        );

        configureBindings();
    }

    private void configureBindings() {
        // Botão Y ou Start para zerar a frente do giroscópio durante a partida
        driverController.y().onTrue(Commands.runOnce(() -> swerve.zeroHeading(), swerve));
    }
}
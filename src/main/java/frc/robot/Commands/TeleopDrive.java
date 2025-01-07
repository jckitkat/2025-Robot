package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TeleopDrive extends Command{

    private final Supplier<SwerveRequest> request;
    private final CommandSwerveDrivetrain drivetrain;
    
    public TeleopDrive(Supplier<SwerveRequest> request, CommandSwerveDrivetrain drivetrain) {
        this.request = request;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setControl(request.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}

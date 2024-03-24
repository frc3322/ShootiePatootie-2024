package frc.robot.Commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.pathNameConstants;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoCommands {
    private final DriveSubsystem driveSubsystem;
    private final Intake intake;
    private final Shooter shooter;
    private final Climbers climbers;

    public AutoCommands(DriveSubsystem driveSubsystem, Intake intake, Shooter shooter, Climbers climbers) {
        this.driveSubsystem = driveSubsystem;
        this.intake = intake;
        this.shooter = shooter;
        this.climbers = climbers;
    }

    /*◇─◇──◇─◇
    🐑Commands🐑
    ◇─◇──◇─◇*/

    public SequentialCommandGroup driveForward(){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathNameConstants.goForth);
        return new SequentialCommandGroup(driveSubsystem.followAutonPath(path));
    }
}

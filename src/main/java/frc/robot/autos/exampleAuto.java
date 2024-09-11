// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.ArrayList;
// import java.util.List;

// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.intake_shoot;

// public class exampleAuto extends SequentialCommandGroup {
//     /*
//     final VictorSPX leftshooter = new VictorSPX(29);
//     final VictorSPX rightshooter = new VictorSPX(28);
//     final VictorSPX leftintake = new VictorSPX(24); // 23
//     final VictorSPX rightintake = new VictorSPX(23); // 24
//      */
//     /*
//     String trajectoryJSON =  "output/simple.wpilib.json";
//     Trajectory trajectory = new Trajectory();

//     @Override
//         public void robotInit() {
//             try {
//                 Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//                 trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
//             } catch (IOException ex) {
//                 DriverStation.reportError("trajectoryJSON error", ex.getStackTrace());
//             }
//         } */
//         /*
//         ArrayList<PathPlannerTrajectory> auto1Paths = 
//             PathPlanner.load

//         Command autoTest =
//         new SequentialCommandGroup(
//             new FollowPathWithEvents(auto1P)
//         ) 
//         double pretime = Timer.getFPGATimestamp();
//         double nowtime = Timer.getFPGATimestamp();   */

//     //public exampleAuto(Swerve s_Swerve){
//         /*
//             while (nowtime - pretime < 3.0) {
//                 rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
//                 leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
//                 rightintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 leftintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 nowtime = Timer.getFPGATimestamp();
//             }
//             while((nowtime - pretime > 3.0) && (nowtime - pretime < 4.5)){
//                 rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
//                 leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
//                 rightintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 leftintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 nowtime = Timer.getFPGATimestamp();
//             }
//             while (nowtime - pretime > 4.5 && nowtime - pretime < 6.0) {
//                 rightshooter.set(VictorSPXControlMode.PercentOutput, -1);
//                 leftshooter.set(VictorSPXControlMode.PercentOutput, 1);
//                 rightintake.set(VictorSPXControlMode.PercentOutput, -1);
//                 leftintake.set(VictorSPXControlMode.PercentOutput,  1);
//                 nowtime = Timer.getFPGATimestamp();
//             }
//             while (nowtime - pretime > 6.0) {
//                 rightshooter.set(VictorSPXControlMode.PercentOutput, 0);
//                 leftshooter.set(VictorSPXControlMode.PercentOutput, 0);
//                 rightintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 leftintake.set(VictorSPXControlMode.PercentOutput, 0);
//                 nowtime = Timer.getFPGATimestamp();
//                 break;
//             }*/

//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                 .setKinematics(Constants.Swerve.swerveKinematics);

//         String trajectoryJSON =  "Path/simple.wpilib.json";
//         Trajectory trajectory = new Trajectory();

//         try {
//             Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//             trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//         } catch (IOException ex) {
//             DriverStation.reportError("trajectoryJSON error", ex.getStackTrace());
//         }

//         // An example trajectory to follow.  All units in meters.
//         Trajectory exampleTrajectory =
//             TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making an 's' curve path
//                 List.of(new Translation2d(-2, 0)), //new Translation2d(-6, 1), new Translation2d(-3, -1)
//                 // End 3 meters straight ahead of where we started, facing forward
//                 new Pose2d(-4, 0, new Rotation2d(0)),
//                 config);        var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         SwerveControllerCommand swerveControllerCommand =
//             new SwerveControllerCommand(
//                 exampleTrajectory, // exampleTrajectory
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//         addCommands(
//             new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
//             swerveControllerCommand
//         );
//     }
// }
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonCameraSim;
import org.photonvision.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.vision.estimation.CameraProperties;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.auto.AutoOptions;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.util.LogUtil;
import frc.robot.vision.estimation.VisionEstimation;
import frc.robot.vision.estimation.VisionEstimation.PNPResults;

public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();

    private OCXboxController controller = new OCXboxController(0);

    private final AutoOptions autoOptions = new AutoOptions(drivetrain);

    private final Field2d field;
    private AprilTagFieldLayout tagLayout;
    private Alliance lastAlliance = DriverStation.Alliance.Blue;

    private NetworkTableInstance instance;
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonCamera camera3;
    private final List<PhotonCamera> cameras;
    private List<PhotonPipelineResult> lastResults;
    private final VisionSystemSim visionSim;

    private boolean correcting = false;
    
    public RobotContainer() {
        autoOptions.submit();
        
        instance = NetworkTableInstance.getDefault();

        camera1 = new PhotonCamera(instance, "front-left");
        camera2 = new PhotonCamera(instance, "front-right");
        camera3 = new PhotonCamera(instance, "back");
        cameras = List.of(camera1, camera2, camera3);
        PhotonCamera.setVersionCheckEnabled(false);
        lastResults = new ArrayList<>(cameras.size());
        cameras.forEach(c -> lastResults.add(new PhotonPipelineResult()));

        
        visionSim = new VisionSystemSim("main");
        // var testprop = new CameraProperties();
        // try{testprop = new CameraProperties("config.json", 640, 480);} catch(Exception e){e.printStackTrace();}
        visionSim.addCamera(
            new PhotonCameraSim(
                camera1,
                // testprop
                CameraProperties.LL2_640_480()
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    -Math.toRadians(18),
                    Math.toRadians(30)
                )
            )
        );

        visionSim.addCamera(
            new PhotonCameraSim(
                camera2,
                CameraProperties.LL2_640_480()
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(-10),
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    -Math.toRadians(18),
                    Math.toRadians(-30)
                )
            )
        );   

        visionSim.addCamera(
            new PhotonCameraSim(
                camera3,
                CameraProperties.LL2_640_480()
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(-10),
                    0,
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    -Math.toRadians(18),
                    Math.PI
                )
            )
        );      

        try {
            // tagLayout = new AprilTagFieldLayout("2022-taglayout.json");
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e){
            e.printStackTrace();
        };

        visionSim.addVisionTargets(tagLayout);
        field = visionSim.getDebugField();
    }

    public Command getAutoCommand() {
        return autoOptions.getSelected();
    }

    public void disable() {
        drivetrain.stop();
    }
    public void setAllBrake(boolean is) {
        drivetrain.setBrakeOn(is);
    }

    public void init() {
        controller = new OCXboxController(0);
        configureDriverBinds(controller);
    }

    public void periodic() {
        var alliance = DriverStation.getAlliance();
        if(alliance != lastAlliance) {
            tagLayout.setOrigin(alliance == DriverStation.Alliance.Blue ?
                OriginPosition.kBlueAllianceWallRightSide
                :
                OriginPosition.kRedAllianceWallRightSide
            );
            lastAlliance = alliance;

            visionSim.removeVisionTargets("apriltags");
            visionSim.addVisionTargets(tagLayout);
        }
    }

    public void configureDriverBinds(OCXboxController controller) {
        drivetrain.setDefaultCommand(
            new RunCommand(()->{
                drivetrain.drive(
                    controller.getForward() * drivetrain.getMaxLinearVelocityMeters(),
                    controller.getStrafe() * drivetrain.getMaxLinearVelocityMeters(),
                    controller.getTurn() * drivetrain.getMaxAngularVelocityRadians(),
                    true
                );
            }, drivetrain)
            .beforeStarting(()->controller.resetLimiters())
        );

        // push-to-change driving "speed"
        controller.rightBumper()
            .onTrue(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedMax)))
            .onFalse(runOnce(()->controller.setDriveSpeed(OCXboxController.kSpeedDefault)));
        
        controller.x()
            .whileTrue(run(()->{
                visionSim.getCameraSim(camera1.getName()).ifPresent(camsim -> {
                    var dist = camsim.prop.getDistCoeffs();
                    camsim.prop.setDistortionCoeffs(dist.plus(0.001));
                });
            }));
        controller.b()
            .whileTrue(run(()->{
                visionSim.getCameraSim(camera1.getName()).ifPresent(camsim -> {
                    var dist = camsim.prop.getDistCoeffs();
                    camsim.prop.setDistortionCoeffs(dist.plus(-0.001));
                });
            }));

        controller.y()
            .whileTrue(run(()->{
                visionSim.getCameraSim(camera1.getName()).ifPresent(camsim -> {
                    visionSim.adjustCamera(camsim,
                        new Transform3d(new Translation3d(), new Rotation3d(0, -0.01, 0))
                            .plus(visionSim.getRobotToCamera(camsim).get())
                    );
                });
                
            }));
        controller.a()
            .whileTrue(run(()->{
                visionSim.getCameraSim(camera1.getName()).ifPresent(camsim -> {
                    visionSim.adjustCamera(camsim,
                        new Transform3d(new Translation3d(), new Rotation3d(0, 0.01, 0))
                            .plus(visionSim.getRobotToCamera(camsim).get())
                    );
                });
            }));
        
        controller.rightTrigger(0.2)
            .onTrue(runOnce(()-> correcting = true))
            .onFalse(runOnce(() -> correcting = false));

        controller.leftTrigger(0.2)
            .onTrue(runOnce(()->{
                var noise = new Transform2d(
                    new Translation2d(Math.random()*2-1, Math.random()*2-1),
                    new Rotation2d(Math.random()*4-2)
                );
                drivetrain.resetNoisyOdometry(drivetrain.getPose().plus(noise));
            }));
    }

    public void log() {
        drivetrain.log();

        field.getObject("Swerve Modules").setPoses(drivetrain.getModulePoses());
        
        Trajectory logTrajectory = drivetrain.getLogTrajectory();
        if(logTrajectory == null) logTrajectory = new Trajectory();
        field.getObject("Trajectory").setTrajectory(logTrajectory);
    }

    //----- Simulation
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPerfPose());
        field.getObject("Noisy Robot").setPose(drivetrain.getPerfPose());

        SmartDashboard.putNumberArray("RobotPose", LogUtil.toPoseArray2d(drivetrain.getPose()));

        final List<TargetCorner> corners = new ArrayList<TargetCorner>();
        final List<AprilTag> foundTags = new ArrayList<AprilTag>();

        for(int i = 0; i < this.cameras.size(); i++) {
            final PhotonCamera camera = this.cameras.get(i);
            final PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets()) continue;

            corners.clear();
            foundTags.clear();

            final List<PhotonTrackedTarget> targets = result.getTargets();

            targets.stream().forEach( target -> {
                final int fiducialId = target.getFiducialId();

                corners.addAll(target.getDetectedCorners());
                foundTags.add(new AprilTag(
                    fiducialId,
                    this.tagLayout.getTagPose(fiducialId).get()
                ));
            });

            if (targets.size() > 1) {
                var cameraSim = visionSim.getCameraSim(camera.getName()).get();
                var robotToCamera = visionSim.getRobotToCamera(cameraSim).get();
                final CameraProperties cameraProp = cameraSim.prop;
                final PNPResults pnpResults = VisionEstimation.estimateCamPosePNP(cameraProp, corners, foundTags);
                final Transform3d robotToCameraPose = robotToCamera;

                SmartDashboard.putNumber(camera.getName() + "/multi/ambiguity", pnpResults.ambiguity);
                SmartDashboard.putNumber(camera.getName() + "/multi/bestErr", pnpResults.bestReprojErr);
                SmartDashboard.putNumber(camera.getName() + "/multi/altErr", pnpResults.altReprojErr);

                if(pnpResults.bestReprojErr < 0.15) {
                    final Pose3d pose = new Pose3d()
                        .plus(pnpResults.best)
                        .plus(robotToCameraPose.inverse());
                    drivetrain.addVisionMeasurement(pose.toPose2d(), result.getLatencyMillis() / 1000.0);
                }
            }
        }
    }

    public double getCurrentDraw(){
        double sum = 0;
        sum += drivetrain.getCurrentDraw();
        return sum;
    }
}

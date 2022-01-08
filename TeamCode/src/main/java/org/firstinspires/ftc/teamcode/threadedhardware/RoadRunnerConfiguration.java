package org.firstinspires.ftc.teamcode.threadedhardware;/*package threadedhardware;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */

import androidx.annotation.NonNull;

//Like all the other RoadRunner stuff, CHANGE THESE IMPORT STATEMENTS to match your programs.
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;

import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive.axes;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.Autonomous.NewRoadRunner.SampleMecanumDrive.getVelocityConstraint;

@Config
public class RoadRunnerConfiguration extends MecanumDrive implements Configuration {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.04;
    public static double inchMult = 86, offset = 0.135;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryFollower follower;

    private List<LynxModule> allHubs;

    public ThreadedMotor backLeft, frontLeft, frontRight, backRight, spinner, slides, ingest, preIngest;
    private List<ThreadedMotor> motors;

    public ThreadedServo dropper, flipdown;

    public ThreadedDigitalSensor limit;

    public ThreadedIMU imu; //A gyroscope in the expansion hubs (inertial measurement unit)

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    //Separated constructor and "Configure" method.
    public RoadRunnerConfiguration(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
    }

    public void Configure(HardwareMap hwMap){

        hardware.clear();

        frontLeft = new ThreadedMotor(hwMap, "front_left_motor");
        backLeft = new ThreadedMotor(hwMap, "back_left_motor");
        backRight = new ThreadedMotor(hwMap, "back_right_motor");
        frontRight = new ThreadedMotor(hwMap, "front_right_motor");
        imu = new ThreadedIMU(hwMap);
        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, axes, AxesSigns.NPN);

        for (ThreadedMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setBulkCachingManual(true);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(MOTOR_VELO_PID);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        motors.get(0).reverse(true);
        motors.get(1).reverse(true);

        // TODO: if desired, use setLocalizer() to change the localization method.

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public void setBulkCachingManual(boolean manual){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                System.out.println("Clearing");
                module.clearBulkCache();
                //module.getBulkData();
            }
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    //Need to fix these at some point
    public void setMode(DcMotor.RunMode runMode) {
        for (ThreadedMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (ThreadedMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) { //Removed DcMotor.RunMode runMode
        for (ThreadedMotor motor : motors) {
            motor.setPID(coefficients);
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (ThreadedHardware motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.get()[1]));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (ThreadedHardware motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.get()[0]));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        motors.get(0).setPower(v);
        motors.get(1).setPower(v1);
        motors.get(2).setPower(v2);
        motors.get(3).setPower(v3);
    }

    //Fix later
    @Override
    public double getRawExternalHeading() {
        return imu.get()[0];
    }
}
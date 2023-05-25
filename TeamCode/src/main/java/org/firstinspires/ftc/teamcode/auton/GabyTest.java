package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.AssetsTrajectoryManager;

@Autonomous
public class GabyTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive s = new SampleMecanumDrive(hardwareMap);
        TrajectoryBuilder t = s.trajectoryBuilder(new Pose2d(0 , 0), 180);
        Trajectory t1 = AssetsTrajectoryManager.load("traj1");
        s.followTrajectory(t1);

    }
}

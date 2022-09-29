package org.firstinspires.ftc.teamcode;

import android.hardware.SensorManager;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;



    public class theGyroscope extends LinearOpMode {
        IntegratingGyroscope gyro;
        gyro = hardwareMap.get;
        ModernRoboticsI2cGyro modernRoboticsI2cGyro;
        ElapsedTime timer = new ElapsedTime();

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");



        public void runOpMode(){

            boolean lastResetState = false;
            boolean curResetState = false;
        }


    }






   /* SensorManager = (SensorManager) HardwareMap.get(type.SENSOR_SERVICE);*/

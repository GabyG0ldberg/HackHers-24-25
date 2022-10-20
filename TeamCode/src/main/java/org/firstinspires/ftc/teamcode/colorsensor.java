package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameInternal;

@Autonomous
public class colorsensor extends LinearOpMode {
    private HackHers_Lib everything;

    ColorSensor CS;

    @Override
    public void runOpMode(){

        super.waitForStart();

        everything..red();





        CS= hardwareMap.get(ColorSensor.class, "CS");





    }

    }




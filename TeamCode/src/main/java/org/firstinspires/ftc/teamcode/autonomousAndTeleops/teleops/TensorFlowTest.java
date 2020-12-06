package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.vision.TensorFlow;

@TeleOp(name = "TensorFlowTest", group = "TeleOp")
// Replace SkeletonOpMode with name of OpMode


public class TensorFlowTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Initialization
        TensorFlow myTensorFlow = new TensorFlow(hardwareMap);
        //Declare local variables
        //Initialization code: example: map hardware

        //Wait for game to start
        waitForStart();

        //Sequential instruction for Autonomous
        //or while loop with update telemetry for driver control

        while (opModeIsActive()){

            int numOfRings = myTensorFlow.getNumberOfRings();

            while (numOfRings == -1){
                numOfRings = myTensorFlow.getNumberOfRings();
                telemetry.update();
            }

            telemetry.addData("Num:", numOfRings);
            telemetry.update();
        }

        myTensorFlow.stop();



    }
}


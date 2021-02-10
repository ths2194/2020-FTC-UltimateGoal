package org.firstinspires.ftc.teamcode.autonomousAndTeleops.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.robotClasses.Pathfinding.Field;
import org.firstinspires.ftc.teamcode.robot.robotClasses.Pathfinding.PathFinding;
import org.firstinspires.ftc.teamcode.robot.vision.Vuforia;
import org.firstinspires.ftc.teamcode.robot.vision.VuforiaFeedback;

@TeleOp(name = "InitializeFieldTest", group = "TeleOp")
// Replace SkeletonOpMode with name of OpMode


public class InitializeFieldTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        //Initialization
        //Declare local variables
        //Initialization code: example: map hardware

        Field field = new Field();
//        field.visualizeStartField();
        PathFinding pathFinding = new PathFinding();
        pathFinding.calculatePath(-20,20,60,60);

        //Wait for game to start
        waitForStart();

        //Sequential instruction for Autonomous
        //or while loop with update telemetry for driver control

        while (opModeIsActive()){
            telemetry.addData("Read:", field.isOpenWithGridAt(0,0));
            telemetry.update();
        }

//        myTensorFlow.stop();



    }
}


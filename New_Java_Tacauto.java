package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.List;

@Autonomous(name = "New_Java_Tacauto")

public class New_Java_Tacauto extends Taco_Super_class {

    double Shooter_Speed;

    @Override
    public void runOpMode() {
        List<Recognition> recognitions; //stuff for vision
        Right_Front = hardwareMap.get(DcMotor.class, "Right_Front");
        Right_Rear = hardwareMap.get(DcMotor.class, "Right_Rear");
        Left_Front = hardwareMap.get(DcMotor.class, "Left_Front");
        Left_Rear = hardwareMap.get(DcMotor.class, "Left_Rear");
        arm = hardwareMap.get(DcMotor.class, "arm");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wheels = hardwareMap.get(DcMotor.class, "wheels");
        indexer = hardwareMap.get(Servo.class, "indexer");
        pincher = hardwareMap.get(Servo.class, "pincher");

        initialization(true);

        telemetry.addData(">", "Wait To Start!!!!!!!!!!!!!!!!!!");
        telemetry.update();
        Shooter_Speed = 0.65;
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels.setDirection(DcMotorSimple.Direction.REVERSE);
        androidSoundPool = new AndroidSoundPool(); //stuff for sound
        vuforiaUltimateGoal = new VuforiaCurrentGame(); //stuff for vision
        tfodUltimateGoal = new TfodCurrentGame(); //stuff for vision
        Zone = 1; //Stuff for vision

        /* stuff for vision below */
        vuforiaUltimateGoal.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodUltimateGoal.activate();
        // Enable following block to zoom in on target.
        tfodUltimateGoal.setZoom(2.5, 16 / 9);
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        androidSoundPool.play("RawRes:ss_roger_roger");//sound stuff

        waitForStart();


    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

public abstract class Taco_Super_class extends LinearOpMode {


    public AndroidSoundPool androidSoundPool;
    public DcMotor Right_Front;
    public DcMotor Right_Rear;
    public DcMotor Left_Front;
    public DcMotor Left_Rear;
    public DcMotor arm;
    public DcMotor shooter;
    public DcMotor wheels;
    public DcMotor intake2;

    public Servo ring_smacking_device_of_ring_smackingness;
    public Servo indexer;
    public Servo pincher;


    public static double shoot_time = 0;
    public static double old_time = 0;
    public static double new_time = 0;
    public static double kP = 0.00078;
    public static double kI = 0.0;                              // these will be used in the PID methods
    public static double kD = 0.0;

    public VuforiaCurrentGame vuforiaUltimateGoal;
    public TfodCurrentGame tfodUltimateGoal;

    double frontlefttarget = 0;
    double frontrighttarget = 0;
    double backrighttarget = 0;
    double backlefttarget = 0;
    double shootertarget = 0;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    double Shooter_Speed;
    double Rounds;
    double Zone;
    Recognition recognition;
    public ElapsedTime tacotimer = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime waittimer = new ElapsedTime();

    double tacotimers;
    double shooterplace;
    double shooterdif;

    double dfore;
    double dright;
    double dturn;


    public void initialization(boolean autonomous) {
        initarm();
        initindexer();
        initwheels();
        initpincher();

        if (autonomous) {
            telemetry.addLine("imu init");
            telemetry.update();
            initiate_imu();

            telemetry.addLine("drive Init");
            telemetry.update();
            //initDrive


            Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        } else {
            Left_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Left_Front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Right_Rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }
        telemetry.addLine("Finished init - GO PRAISE JESUS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        telemetry.update();
    }

    public void initiate_imu() {

        telemetry.addLine("Initiating IMU");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";


        while (opModeIsActive()) {

        }
    }


    // --------------------------------Motors--------------------------------------
    public void initarm() {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void initwheels() {
        wheels = hardwareMap.dcMotor.get("wheels");
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //--------------------------------------Servos-----------------------------------
    //public void initarm_servo(){arm_servo = hardwareMap.get(Servo.class, "arm_servo");}

    public void initindexer() {
        indexer = hardwareMap.get(Servo.class, "indexer");
    }

    public void initpincher() {
        pincher = hardwareMap.get(Servo.class, "pincher");
    }

    //----------------------------Drive Methods---------------------------------------------
    public void DriveOff() {
        Left_Rear.setPower(0);
        Right_Rear.setPower(0);
        Left_Front.setPower(0);
        Right_Front.setPower(0);
    }



    public void driveForwardEncoders(double distance, double power) {
        backlefttarget = (Left_Rear.getCurrentPosition() + distance);
        backrighttarget = (Right_Rear.getCurrentPosition() + distance);
        frontlefttarget = (Left_Front.getCurrentPosition() + distance);
        frontrighttarget = (Right_Front.getCurrentPosition() + distance);

        while (Left_Rear.getCurrentPosition() < backlefttarget
                /*&& leftFront.getCurrentPosition() < frontlefttarget
                && rightBack.getCurrentPosition() < backrighttarget
                && rightFront.getCurrentPosition() < frontrighttarget*/ && opModeIsActive()) {


            Left_Rear.setPower(power);
            Left_Front.setPower(power);
            Right_Rear.setPower(power);
            Right_Front.setPower(power);

            telemetry.addData("Left_Rear", Left_Rear.getCurrentPosition());
            telemetry.addData("Left_Front", Left_Front.getCurrentPosition());
            telemetry.addData("Right_Rear", Right_Rear.getCurrentPosition());
            telemetry.addData("Right_Front", Right_Front.getCurrentPosition());

            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }


    public void driveBackwardEncoders(double distance, double power) {

        backlefttarget = (Left_Rear.getCurrentPosition() - distance);
        backrighttarget = (Right_Rear.getCurrentPosition() - distance);
        frontlefttarget = (Left_Front.getCurrentPosition() - distance);
        frontrighttarget = (Right_Front.getCurrentPosition() - distance);

        while (Left_Rear.getCurrentPosition() > backlefttarget
               /* && leftFront.getCurrentPosition() > frontlefttarget
                && rightBack.getCurrentPosition() > backrighttarget
                && rightFront.getCurrentPosition() > frontrighttarget*/ && opModeIsActive()) {
            Left_Rear.setPower(-power);
            Left_Front.setPower(-power);
            Right_Rear.setPower(-power);
            Right_Front.setPower(-power);

            telemetry.addData("Left_Rear", Left_Rear.getCurrentPosition());
            telemetry.addData("Left_Front", Left_Front.getCurrentPosition());
            telemetry.addData("Right_Rear", Right_Rear.getCurrentPosition());
            telemetry.addData("Right_Front", Right_Front.getCurrentPosition());
            telemetry.update();
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void driveleft(double distance, double power) {
        Left_Front.setPower(-power);
        Left_Rear.setPower(power);
        Right_Front.setPower(power);
        Right_Rear.setPower(-power);

        backlefttarget = (Left_Rear.getCurrentPosition() + distance);
        backrighttarget = (Right_Rear.getCurrentPosition() - distance);
        frontlefttarget = (Left_Front.getCurrentPosition() - distance);
        frontrighttarget = (Right_Front.getCurrentPosition() + distance);

        while (/*leftFront.getCurrentPosition() > frontlefttarget
               &&*/ Left_Rear.getCurrentPosition() < backlefttarget
               /* && rightFront.getCurrentPosition() < frontrighttarget
                && rightBack.getCurrentPosition() > backrighttarget*/ && opModeIsActive()) {
        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void driveright(double distance, double power) {
        Left_Front.setPower(power);
        Left_Rear.setPower(-power);
        Right_Front.setPower(-power);
        Right_Rear.setPower(power);

        backlefttarget = (Left_Rear.getCurrentPosition() - distance);
        backrighttarget = (Right_Rear.getCurrentPosition() + distance);
        frontlefttarget = (Left_Front.getCurrentPosition() + distance);
        frontrighttarget = (Right_Front.getCurrentPosition() - distance);

        while (//leftFront.getCurrentPosition() < frontlefttarget
            /*&&*/ Left_Rear.getCurrentPosition() > backlefttarget
                /* && rightFront.getCurrentPosition() > frontrighttarget
                 *//*&&*//* rightBack.getCurrentPosition() < backrighttarget*/ && opModeIsActive()) {

        }
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void rotateLeft_Encoders(double distance, double power) {
        Left_Rear.setPower(-power);
        Left_Front.setPower(-power);
        Right_Rear.setPower(power);
        Right_Front.setPower(power);

        backlefttarget = (Left_Rear.getCurrentPosition() - distance);
        backrighttarget = (Right_Rear.getCurrentPosition() + distance);
        frontlefttarget = (Left_Front.getCurrentPosition() - distance);
        frontrighttarget = (Right_Front.getCurrentPosition() + distance);

        while (Left_Front.getCurrentPosition() < frontlefttarget
                && Right_Front.getCurrentPosition() > frontrighttarget
                && opModeIsActive()) {

        }

        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();

    }

    public void rotateRight(double distance, double power) {
        Left_Rear.setPower(power);
        Left_Front.setPower(power);
        Right_Rear.setPower(-power);
        Right_Front.setPower(-power);


        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveOff();
    }

    public void Stop_And_Reset_Encoders() {
        Right_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        Right_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();
        Right_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    public void Run_To_Position() {
        Right_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Motor_Power() {
        Right_Front.setPower(0.8);
        Right_Rear.setPower(0.8);
        Left_Front.setPower(0.8);
        Left_Rear.setPower(0.8);
    }

    //--------------------------------- imu stuff i think----------------------------




    //______________________pid stuff__________________________________________________________

   /* PIDController pid = new PIDController(new PIDCoefficients(kP, kI, kD));
    public double IMUstraightDouble(double targetAngle) {
        double currentAngle = getAngle();
        pid.setCoeffs(new PIDCoefficients(kP, kI, kD));
        return pid.update(currentAngle - targetAngle);
    }
    public double distancePIDDouble(double power) {
        //double shooter = getAngle();
        pid.setCoeffs(new PIDCoefficients(kP, kI, kD));
        return pid.update(-shootertarget);
    }*/


    // -------------------------other stuff--------------------------------------------------
    public void Motor_Power_33() {
        Right_Front.setPower(0.33);
        Right_Rear.setPower(0.33);
        Left_Front.setPower(0.33);
        Left_Rear.setPower(0.33);
    }

    public void displayInfo(double i) {
        if (opModeIsActive()) {
            // Display label info.
            telemetry.addData("Zone", Zone);
            telemetry.update();
        }
    }

    public void Object2() {

    }



    public void Motor_Power_75() {
        Right_Front.setPower(0.75);
        Right_Rear.setPower(0.75);
        Left_Front.setPower(0.75);
        Left_Rear.setPower(0.75);
    }

    private void Shoot1() {
        shooter.setPower(Shooter_Speed);
        sleep(2000);
        shooter.setPower(Shooter_Speed);
        indexer.setPosition(0.3);
        shooter.setPower(Shooter_Speed);
        sleep(1000);
        shooter.setPower(Shooter_Speed);
        indexer.setPosition(0.51);
    }

    public void Shoot2() {
        timer.reset();
        shoot(false,false, true);
        while (timer.seconds()<.5){
            shoot(false,false, true);
            shoot(false,false, true);
        }
        while (timer.seconds()<1.75){
            shoot(false,false, true);
            shoot(false,false, true);
            indexer.setPosition(0);
        }
        shoot(false,false, true);
        shoot(false,false, true);
        while (timer.seconds()<2.5){
            shoot(false,false, true);
            shoot(false,false, true);
            indexer.setPosition(0.65);
        }
        shoot(false,false, true);
        while (timer.seconds()<3.25){
            shoot(false,false, true);
            shoot(false,false, true);
            indexer.setPosition(0);
        }
        shoot(false,false, true);
        while (timer.seconds()<4){
            shoot(false,false,true);
            shoot(false,false, true);
            indexer.setPosition(0.65);
        }
        shoot(false,false, true);
    }

    public void powerDriveTrain(double leftPower, double rightPower) {
        double max = 1.0;
        max = Math.max(max, Math.abs(leftPower));
        max = Math.max(max, Math.abs(rightPower));
        leftPower /= max;
        rightPower /= max;

        Left_Rear.setPower(leftPower);
        Left_Front.setPower(leftPower);
        Right_Rear.setPower(rightPower);
        Right_Front.setPower(rightPower);

    }

    public double shoot(boolean powershots, boolean normalshots, boolean auto){

        if(powershots){
            shooter.setPower(7.5*(1/(shooterdif)));
        }
        else if(normalshots){
            shooter.setPower(9*(1/(shooterdif)));
        }
        else if(auto){
            shooter.setPower(12*(1/(shooterdif)));
        }
        else{
            shooter.setPower(0);
        }
        if((tacotimer.seconds()-tacotimers)>.01) {

            shooterdif = shooter.getCurrentPosition() - shooterplace;

            shooterplace = shooter.getCurrentPosition();

            tacotimers = tacotimer.seconds();
        }
        return shooterdif;
    }


    public int fect(){
        return(-Left_Front.getCurrentPosition()/4 - Left_Rear.getCurrentPosition()/4 + Right_Front.getCurrentPosition()/4 + Right_Rear.getCurrentPosition()/4);
    }
    public int rect(){
        return(-Left_Front.getCurrentPosition()/4 + Left_Rear.getCurrentPosition()/4 - Right_Front.getCurrentPosition()/4 + Right_Rear.getCurrentPosition()/4);
    }
    public int tect(){
        return(-Left_Front.getCurrentPosition()/4 - Left_Rear.getCurrentPosition()/4 - Right_Front.getCurrentPosition()/4 - Right_Rear.getCurrentPosition()/4);
    }

    public void teledrive(double forward,double right,double turnc){
        Left_Rear.setPower(-forward + right - turnc);
        Left_Front.setPower(-forward - right - turnc);
        Right_Rear.setPower(forward + right - turnc);
        Right_Front.setPower(forward - right - turnc);

    }

    /*public void goal(double fore,double right,double turn){
        dfore = (fore-fect())*.2;
        dright = (right-rect())*.3;
        dturn = (turn-tect())*.1;
        telemetry.addData("dfore",dfore);
        telemetry.addData("dright",dright);
        telemetry.addData("dturn",dturn);
        telemetry.update();
        teledrive(dfore,dright,dturn);
        Left_Rear.setPower(dfore + dright - dturn);
        Left_Front.setPower(dfore - dright - dturn);
        Right_Rear.setPower(-dfore + dright - dturn);
        Right_Front.setPower(-dfore - dright - dturn);
    }*/

    //Find stuff
    public double foregoal(double goal){
        telemetry.addData("forwards delta",(goal-fect()));
        return (goal-fect())*kP;

    }
    public double rightgoal(double goal){
        telemetry.addData("rightwards delta",(goal-rect()));
        return (goal-rect())*kP;
    }
    public double turngoal(double goal){
        telemetry.addData("turnwards delta",(goal-tect()));
        return (goal-tect())*kP;
    }
    public boolean nextstep(double forego, double rightgo, double turngo){
        if (!((Math.abs(rect()-rightgo)<80) && (Math.abs(fect()-forego)<80) && (Math.abs(tect()-turngo)<80))){
            timer.reset();
        }
        if (timer.seconds()>.001 && ((Math.abs(rect()-rightgo)<80) && (Math.abs(fect()-forego)<80) && (Math.abs(tect()-turngo)<80))){
            return false;
        }
        else{
            return true;
        }
    }
    public void drivetele(){
        telemetry.addData("forward encoders",fect());
        telemetry.addData("right encoders",rect());
        telemetry.addData("turn encoders",tect());
        telemetry.update();

    }
}

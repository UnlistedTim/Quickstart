package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

//test


@TeleOp(name="TeleopStateWood", group="A")
@Config
@Disabled
public class TeleopStateWood extends LinearOpMode {


    Pose2D pose;


    private DcMotorEx Intake, flyBot, flyTop, turretSpin, leftFront, rightFront, leftBack, rightBack;

    private Servo Hood, Blocker, Tripod;

    private DigitalChannel beamBreaker;

    private Limelight3A Limelight;

    public GoBildaPinpointDriver Pinpoint;

    BooleanConfidenceChecker checker = new BooleanConfidenceChecker();




    public base rbg= new base();

    double startHeading, startX, startY;

    boolean allianceRed = false;



    // Pose2D pose;


    public boolean lift = false;

    public boolean BBState = true,BBState0=true,BBState1=true;

    public boolean prevBBState = true,prevBBState2;




    // lift pos 0.07 lift angle 40

    double Tyaverage = 0;

    public static double intakeMaxVel = 3000;

    // private PIDController controller;
    public final int max_vel = 1800;

    public int ball_count = 0;

    public boolean debounce = false;

    public boolean debouncearr[] = {false,false,false};

    LLResult result;

    public double hoodLastPos = 0.0,targetx=144,targety=144;

    public double hoodPos = 0, currentime=0,previoustime=0;

    int i = 0;

    int breakInARow = 1;

    int openInARow = 1;

    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //    double[] Tydata = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//    double[] Tyempty = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    int intake = 0, outtake = 1, spinstatus = 2, spinfix = 3, shootbreak = 4;



    boolean[] flag = new boolean[]{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

    boolean recevieinfo = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean limeValid = false,pinponit_nav=true;
    boolean outtakestate=false;
    boolean drive = true, present = false;
    int  shoot_count = 0;
    int id = 1,intakecount=0,withball=0;
    int target_id = 24;


    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    double  startime=0;
    double Tx = 100;
    public static double Tx_offset = 0;

    double rawIntakeCurrent;

    double filteredIntakeCurrent;

    double Ty = 0.0, dist = 0.0;

    public static double  shootingIntakeVel = 2500;

    boolean red = true;


    public int turretPos = 0;

    public double fieldRelativeAngle = 0;

    public double robotRelativeTurretAngle = 0;
    public int turretTarget=0;
    public double turnPower=0 ;
    public double flyCurrentVel = 0;


    public enum State {
        DEBUG,
        IDLE,
        INTAKE,
        OUTTAKE,
        MANUALOUTTAKE;
    }

    private enum ShootState {
        PRE_SHOOT, SHOOT, DONE
    }
    ShootState shootState = ShootState.PRE_SHOOT;

 //    State state = State.DEBUG;
 State state = State.IDLE;

    boolean shooting = false, intakecheck=false;

    // --- Timers ---
    private ElapsedTime deltaT = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();


    @SuppressLint("SuspiciousIndentation")
    @Override

    public void runOpMode() {

        //push
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        initalize();
        getAutoVars();


        waitForStart();
        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));

        afterstart();
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //Bulk reading for faster loop times

//        Intake.setVelocity(rbg.intakeVel);
//        telemetry.clearAll();
        while (opModeIsActive()) { //Main While loop


           BBState= beamBreaker.getState();

            switch (state) {
                case DEBUG:

                  //  intakecount++;
                    if(BBState0&&BBState1) {
                        intakecheck=false;
                        if(!BBState){
                            startime=timer.milliseconds();intakecheck=true;
                        }
                    }

                    if(intakecheck&&timer.milliseconds()-startime>400) {
                        Intake.setVelocity(0);
                        sleep(500000000);

                    }
                    BBState1=BBState0;
                    BBState0=BBState;


                    break;
                case IDLE:
                    if (stoptimers(300,intake )){
                        intakeStart();
                        rbg.limelocked=false;
                        state = State.INTAKE;
                       // flyprepower(0.3);
                        stoptimers(0,intake);
                        break;
                    }

                case INTAKE:
                    if ( gamepad2.leftBumperWasPressed() || (stoptimers(800,intake) && beamBreakCount())){
                        state = State.OUTTAKE;
                        outtakestate=true;
                        break;
                    }

                    break;
                case OUTTAKE:

                   if(pinponit_nav) flywheelPP();
                      else  flywheel();

                    if (gamepad2.rightBumperWasPressed() || shooting){
                        if(shoot()) outtakestate=false;
                    }
                    if(!outtakestate || gamepad2.leftBumperWasPressed()) //||gamepad2.leftBumperWasPressed()
                    {
                       outtakedone();
                        state= State.IDLE;

                    }

                    break;

                case MANUALOUTTAKE:
                    break;
            }





            if (gamepad1.psWasPressed()){
                if (!lift){
                    Tripod.setPosition(rbg.tripodPark);
                    drive = false;
                    lift = true;
                }

                else{
                    Tripod.setPosition(rbg.tripodIdle);
                    lift = false;
                    drive = true;
                }

            }

            statusupdate();//caputure all the hardware reading info.
            if (drive) mecanumRobotDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            else stopDriveMotors();

            turntable();



        }
    }

    public void statusupdate()

    {
        turretPos=turretSpin.getCurrentPosition();
        Pinpoint.update();
        pose = Pinpoint.getPosition();
        if(outtakestate) {
          if(!pinponit_nav) {
              result = Limelight.getLatestResult();
              limeValid = result.isValid();
              if(limeValid)  {
                  Tx=result.getTx();
                  Ty=result.getTy();
            }

          }
//            dashboardTelemetry.addData("Field relative Angle",Math.toDegrees(fieldRelativeAngle));
//            dashboardTelemetry.addData("Robot relative Turret angle,",Math.toDegrees(robotRelativeTurretAngle));

            flyCurrentVel=flyBot.getVelocity();
            rawIntakeCurrent= Intake.getCurrent(CurrentUnit.MILLIAMPS);
           // filteredIntakeCurrent = rbg.intakeCurrentFilter.update(rawIntakeCurrent);

        }




    }

    public void  turntable()

    {
        if(pinponit_nav)  turnPower= rbg.turretturnPP(outtakestate,pose,turretPos);
       else  turnPower= rbg.turretturn(outtakestate,limeValid ,turretTarget,turretPos,Tx, Tx_offset);

       turretSpin.setPower(turnPower);

    }








    public void flyprepower(double power) {
        flyBot.setPower(power);
        flyTop.setPower(power);
    }
    public void stopDriveMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void afterstart() {

        deltaT.reset();
        timer.reset();
        outtakeTimer.reset();
        runtime.reset();

    }


    public void intakeStart(){
        flyTop.setPower(rbg.intakeflypower1);
        flyBot.setPower(rbg.intakeflypower1);
        checker = new BooleanConfidenceChecker();
        // intakeCurrentFilter = new MedianFilter(10);

        Intake.setVelocity(rbg.intakeVel);
    }


    public void outtakedone()


    {
        drive = true;
        shooting = false;
        ball_count = 0;
        limeValid=false;
        rbg.limelocked=false;
        stoptimers(0, intake);
        outtakestate=false;
        Blocker.setPosition(rbg.blockClose);
    }

    public void initalize() {

        Hw_init();
        rbg.init();
        Blocker.setPosition(rbg.blockClose);
        Tripod.setPosition(rbg.tripodIdle);

        if (red) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");
        telemetry.addLine("Blue Alliance Selected");
        //  telemetry.addData(" Patter Green ", pattern_id - 20);
        telemetry.addLine("*******************************************");
        configinfo();
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                red = false;
                recevieinfo = true;
                telemetry.addLine("Blue Selected");
            }
            if (gamepad1.circle) {
                red = true;
                recevieinfo = true;
                telemetry.addLine("Red  Selected");
            }

            if (recevieinfo) {
                configinfo();
                telemetry.update();
            }
            if (gamepad1.right_bumper) break;
        }
        if (red) {
            Tx_offset = 0;
            target_id = 24;
           rbg.targetGoalX=rbg.redGoalX;
           rbg.targetGoalY=rbg.redGoalY;

            Limelight.pipelineSwitch(6);
        } else {
            Tx_offset = 0;
            target_id = 20;
            targetx=0;
            Limelight.pipelineSwitch(7);
            rbg.targetGoalX=rbg.blueGoalX;
            rbg.targetGoalY=rbg.blueGoalY;
        }

        telemetry.clear();

        if (red) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");
        telemetry.addLine("Blue Alliance Selected");
        //  telemetry.addData(" Patter Green ", pattern_id - 20);
        telemetry.update();


        Limelight.start();





    }

    public void resetIntakeVars(){
        breakInARow = 0;
        ball_count = 0;
        debounce = false;
        i = 0;
        boolean debouncearr[] =  {false,false,false};
        prevBBState = true;

    }



    public boolean beamBreakCount(){
        BBState = beamBreaker.getState();
        if (!BBState && !debounce){
            ball_count++;
            if (ball_count == 3){
                Intake.setVelocity(100);
                resetIntakeVars();
                // state = State.OUTTAKE;
                return true;
            }
            debounce = true;
        }

        if (checker.update(!BBState)) {
            Intake.setVelocity(100);

            resetIntakeVars();
            state = State.OUTTAKE;
            return true;

        }

//

        if (BBState && prevBBState) openInARow++;
        else openInARow = 0;

        if (debounce && openInARow > 10){
            debounce = false;
            openInARow = 0;

        }
        prevBBState = BBState;


        return false;
    }

    void configinfo() {
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");
        telemetry.addLine("Drive Right Bumper Confrim ");


    }

//
//    public void manualturn(double x) {
//        if (x > 0.4) {//clockwise
//            turetturndir = -1;
//            flag[dected]=false;
//         //   turretSpin.setPower(-0.3);
//        //flag[manualturn]=true;
//            turettarget=turrretclock+10;
//        }
//
//        if (x < -0.4) {//couterclockwise
//            turetturndir = 1;
//           // turretSpin.setPower(0.3);
//          //  flag[manualturn]=true;
//            turettarget=turretcounterclock-10;
//            flag[dected]=false;
//        }
//
//
//    }
//
//
//    public void manualturnadj(double x) {
//        turretPos=turretSpin.getCurrentPosition();
//        double turnp= -0.35*x;
//
//
//
//        if(turretPos>turretcounterclock-30&& turnp>0)  {turnp=-0.3;sleep(50);}
//        if(turretPos<(turrretclock+30)&& turnp<0)  {turnp=-0.3;sleep(50);}
//
//        turretSpin.setPower(turnp);
//
//    }


//    public void ATy()
//
//    {
//        if(flag[Tylock]||!limeValid) return;
//        if(Ty<-15 || Ty>11) return;
//
//
//        double sum=0,count=0;
//        Tyaverage=0;
//        for(int i=0;i<10;i++)
//
//
//        {   if(i<9)
//            Tydata[i]=Tydata[i+1];
//            else Tydata[i]=Ty;
//            if(Ty>-15 && Ty<11) count++;
//            sum=sum+Tydata[i];
//
//
//        }
//        Tyaverage=sum/count;
//
//
//
//    }

    public void mecanumFieldDrive(double y, double x, double rx){
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;

        double botHeading = 0;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

    }

    public class BooleanConfidenceChecker {

        private static final int WINDOW_SIZE = 100;   // total samples
        private static final double TRUE_THRESHOLD = 0.90; // 95%

        private final boolean[] window = new boolean[WINDOW_SIZE];
        private int index = 0;
        private int trueCount = 0;
        private boolean filled = false;

        public boolean update(boolean input) {

            if (filled) {
                if (window[index]) trueCount--;
            }
            window[index] = input;
            if (input) trueCount++;

            index++;
            if (index >= WINDOW_SIZE) {
                index = 0;
                filled = true;
            }

            if (!filled) return false;
            double trueRate = (double) trueCount / WINDOW_SIZE;

            return trueRate >= TRUE_THRESHOLD;
        }
    }



    public void mecanumRobotDrive(double y, double x, double rx){

//        Pinpoint.update();
//        pose = Pinpoint.getPosition();


//        dashboardTelemetry.addData("Pinpoint x", pose.getX(DistanceUnit.INCH));
//        dashboardTelemetry.addData("Pinpoint Y", pose.getY(DistanceUnit.INCH));
//        dashboardTelemetry.update();

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double botHeading = pose.getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

//        telemetry.addData("Angle", botHeading);
//        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
//        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));



//        telemetry.update();
    }



    public void flywheel() {

        double flypower;

        flypower=rbg.flyspeed(flyCurrentVel,Ty);
        flyBot.setPower(flypower);
        flyTop.setPower(flypower);
        hoodPos=rbg.flyhood(Ty);
        if(hoodPos>0 &&Math.abs(hoodPos-hoodLastPos)>0.01){

            Hood.setPosition(hoodPos);
            hoodLastPos=hoodPos;
        }


    }

    public void flywheelPP() {

        double flypower;

//        flypower=rbg.flyspeedPP(flyCurrentVel,dist);
//        flyBot.setPower(flypower);
//        flyTop.setPower(flypower);
        hoodPos=rbg.flyhoodPP();
        if(hoodPos>0 &&Math.abs(hoodPos-hoodLastPos)>0.01){

            Hood.setPosition(hoodPos);
            hoodLastPos=hoodPos;
        }


    }


    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");

        beamBreaker = hardwareMap.get(DigitalChannel.class, "beamBreaker");


        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");

        configurePinpoint();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo

        Hood = hardwareMap.get(Servo.class, "Hood");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");

        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setVelocity(0);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyBot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        flyBot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyBot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyBot.setDirection(DcMotorSimple.Direction.REVERSE);
        flyTop.setDirection(DcMotorSimple.Direction.FORWARD);
        Hood.setDirection(Servo.Direction.REVERSE);
        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretSpin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Blocker.setPosition(rbg.blockClose);
        Tripod.setPosition(rbg.tripodIdle);


    }


    public boolean stoptimers(double period, int i) {

        if (period == 0) {
            stoptime[i] = runtime.milliseconds();
            return false;
        }
        return runtime.milliseconds() - stoptime[i] > period;
    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        Pinpoint.setOffsets(3.15, -5, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        Pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        Pinpoint.resetPosAndIMU();
    }

    public void getAutoVars(){


        try {
            startHeading = (double) blackboard.get("Heading");
        }
        catch (NullPointerException e){
            startHeading = 3*Math.PI/2;
        }

        try {
            startX = (double) blackboard.get("X");
        }

        catch (NullPointerException e){
            startX = rbg.REDXOFFSET;
        }

        try {
            startY = (double) blackboard.get("Y");
        }

        catch (NullPointerException e){
           // startY = rbg.REDYOFFSET;
        }

        try{
            allianceRed = (boolean) blackboard.get("Color");
        }
        catch (NullPointerException e){
            allianceRed = true;
        }




    }


    public boolean shoot(){

        if (!shooting){
            shooting = true;
            Intake.setVelocity(shootingIntakeVel);  //shootingIntakeVel
            rbg.Txgap=30;
            shootState = ShootState.PRE_SHOOT;


        }
        switch (shootState){
            case PRE_SHOOT:
                if(rbg.flyspeedgap <= 40&& rbg.Txgap < 1){  // rbg.Txgap < 1
                    drive = false;
                    Blocker.setPosition(rbg.blockOpen);
                    stoptimers(0, outtake);
                    shootState = ShootState.SHOOT;
                    break;
                }
                break;
            case SHOOT:

                if ((filteredIntakeCurrent < 700 && stoptimers(500,outtake)) ){  //TODO OLD 700 vel
                   // shootState = ShootState.DONE;
                    return true;
                }
                break;
            case DONE:

                return true;


        }
        return false;
    }



}





















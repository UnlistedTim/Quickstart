package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;


@TeleOp(name="TeleopState", group="A")
@Config
public class TeleopStateH extends LinearOpMode  {

    Pose2D pose;
    private DcMotorEx intakeLeft,intakeRight, flyBot, flyTop,  leftFront, rightFront, leftBack, rightBack;
    private CRServo turretLeft, turretRight;
    private Servo Hood, Blocker, Tripod,Led;
    private DigitalChannel botBB,topBB,midBB;
    private Limelight3A Limelight;
    private Follower follower;
    public GoBildaPinpointDriver Pinpoint;


    public baseh rbg= new baseh();

    double startHeading, startX, startY;



    public boolean lift = false;

    public double colorFactor = 1;


    public boolean topbb=true,botbb=true,midbb=true,intakefirst=false;






    public double intakespeed=0;

    public int ball_count = 0,Tgap=0;

    public boolean debounce = false;



    LLResult result;

    public double hoodLastPos = 0.0,targetx=144,targety=144,intakepower=0;

    public double hoodPos = 0, currentime=0,previoustime=0,flypower=0.3;



    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



    int intake = 0, outtake = 1;

    private final Pose RHPStartPose = new Pose(130, 10, Math.toRadians(0)); //
    private final Pose BHPStartPose = new Pose(-10, 10, Math.toRadians(180)); //
    private final Pose RHPShootPose = new Pose(72, 26, Math.toRadians(0)); //
    private final Pose BHPShootPose = new Pose(72, 26, Math.toRadians(180));
    private final Pose BlueParkPose = new Pose(14, 22.5, Math.toRadians(0)); //
    private final Pose RedParkPose = new Pose(14, 22.5, Math.toRadians(0)); //





    boolean recevieinfo = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
  //  Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean limeValid = false,pinpoint_nav=true;
    boolean outtakestate=false,intakestate=true,manual=false;
    boolean drive = true, present = false, fieldCentric = true;

    int target_id = 24;


    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    double  startime=0, intaketargetvel=1500;
    double Tx = 100,manualTy=-8.5;//todo
    public static double Tx_offset = 0;



    double Ty = 0.0, dist = 0.0;



    boolean red = true;


    public int turretPos = 0;




    public ElapsedTime looptimer = new ElapsedTime();



    public double turnPower=0 ;
    public double flyCurrentVel = 0;


    public enum State {

        IDLE,
        INTAKE,
        OUTTAKE,
        MANUALOUTTAKE,

        PARK;
    }

    private enum ShootState {
        START,PRE_SHOOT, UNLOCK,SHOOT, DONE
    }
    ShootState shootState = ShootState.START;

 //    State state = State.DEBUG;
 State state = State.IDLE;
    boolean shooting = false;

    // --- Timers ---
    private ElapsedTime deltaT = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();



    @SuppressLint("SuspiciousIndentation")
    @Override

    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        initalize();


        Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.RADIANS, startHeading));
        pose = Pinpoint.getPosition();
        waitForStart();

        afterstart();

//        looptimer.reset();
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //Bulk reading for faster loop times


        while (opModeIsActive()) { //Main While loop




            switch (state) {
//                case DEBUG:
//
//
//                    break;
                case IDLE:
                    if (stoptimers(300,intake )){
                        intakeStart();
                        state = State.INTAKE;
                        stoptimers(0,intake);
                        break;
                    }

                case INTAKE:

                     if ( gamepad2.leftBumperWasPressed() || rbg.beamscanintake(topbb,midbb,botbb)){//(stoptimers(500,intake)

                            state = State.OUTTAKE;
                            Led.setPosition(rbg.ledred);
                            gamepad1.rumble(500);
                             intaketargetvel=0;
                            outtakestate=true;
                            intakestate=false;
                            shootState= ShootState.START;
                    }

                    break;
                case OUTTAKE:

                    if (gamepad2.rightBumperWasPressed() || shooting){
                        if(shoot()) outtakestate=false;
                    }
                    if(!outtakestate || gamepad2.leftBumperWasPressed()) //||gamepad2.leftBumperWasPressed()
                    {
                        outtakedone();
                        state= State.IDLE;
                    }
                    break;


                case PARK:

                    if (!lift){

                        Tripod.setPosition(rbg.tripodPark);
                        lift=true;
                    }

                    if (lift && gamepad1.psWasPressed()){
                        Tripod.setPosition(rbg.tripodIdle);
                        state = State.IDLE;
                        lift = false;
                        drive = true;
                    }



                    break;
            }


            if(gamepad2.shareWasPressed()){

               manual=!manual;
               pinpoint_nav = false;
//               telemetry.addData("pinpoint-nav",pinpoint_nav);
//               telemetry.update();
            }

            if(gamepad2.rightStickButtonWasPressed()) {
                turretLeft.setPower(0);
                turretRight.setPower(0);
                stopDriveMotors();
                sleep(100);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sleep(50);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Tgap = 0;

            }


            if (gamepad1.psWasPressed()){

                state = State.PARK;
                drive = false;
                outtakestate = false;
                lift = false;
                flypower = 0;
                intaketargetvel = 0;


            }
            if(gamepad2.psWasPressed()){

               pinpoint_nav=!pinpoint_nav;
//               telemetry.addData("pinpoint-nav",pinpoint_nav);
//               telemetry.update();
            }

//




            if (drive) {
                if (fieldCentric) mecanumRobotDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                else mecanumRobotCentricDrive(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }

            else stopDriveMotors();
            statusupdate();//caputure all the hardware reading info.
            flywheel();

            if (gamepad1.crossWasPressed()) fieldCentric = !fieldCentric;


            turntable(gamepad2.right_stick_x);




        }
    }




    void calibrate()

    {
        stopDriveMotors();
        sleep(150);
        if(red) {
            Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, rbg.rcalxoffset, rbg.calyoffset, AngleUnit.RADIANS, rbg.calheading));
        }


        else {
            Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, rbg.bcalxoffset, rbg.calyoffset, AngleUnit.RADIANS, rbg.calheading));

        }


    }

    public void statusupdate()

    {
       turretPos=leftFront.getCurrentPosition()+Tgap;
        Pinpoint.update();
        pose = Pinpoint.getPosition();
        intakespeed=intakeLeft.getVelocity();
        Intake(intaketargetvel);
      //  dist = calcDist(pose.getX(DistanceUnit.INCH),pose.getY(DistanceUnit.INCH),rbg.redGoalX,rbg.redGoalY);
        topbb=topBB.getState();
        botbb=botBB.getState();
        midbb=midBB.getState();
        if(outtakestate) {
         // if(!pinpoint_nav) {
              result = Limelight.getLatestResult();
              limeValid = result.isValid();
              if(limeValid)  {
                  Tx=result.getTx();
                  Ty=result.getTy();

                  if(red){
                      if(Ty<-10.5) rbg.txoffset = -2;

                      else rbg.txoffset = 0;
                  } else {

                      if(Ty<-10.5) rbg.txoffset =2.0;

                      else rbg.txoffset = 0;

                  }
                  if(pinpoint_nav) Tx_offset=rbg.txoffset;

                 // fardis= Ty < -10.5;// simpify
           }

              if (manual) {Ty = manualTy;}

         // }
          flyCurrentVel=flyBot.getVelocity();

        }



    }

    public void  turntable( double x)

    {

       if(Math.abs(x)>0.1){
            turnPower = -0.8*x;
         }

       else {

           turnPower = rbg.turret(outtakestate&&!manual, pose, turretPos, red, pinpoint_nav, limeValid, Tx, shooting);

       }
        if (turretPos > rbg.turretCcwlim- 300 &&  turnPower  > 0) {
           turnPower= -0.5;//
            rbg.limelocked = false;
        }
        if (turretPos < (rbg.turretCwlim + 300) && turnPower< 0) {
           turnPower= 0.5;//
            rbg.limelocked = false;
        }

           turretLeft.setPower(turnPower);
           turretRight.setPower(turnPower);


    }




    public void stopDriveMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void afterstart() {

        timer.reset();
        outtakeTimer.reset();
        runtime.reset();
        Blocker.setPosition(rbg.blockClose);
        Tripod.setPosition(rbg.tripodIdle);
        Led.setPosition(rbg.ledgreen);
        Limelight.start();
    }


    public void intakeStart(){
       // Blocker.setPosition(rbg.blockClose);
       flypower=rbg.intakeflypower1;
       intakefirst=true;
       // checker = new BooleanConfidenceChecker();
        // intakeCurrentFilter = new MedianFilter(10);
        Blocker.setPosition(rbg.blockClose);
      //  Intake(rbg.intakeVel);
        intaketargetvel=rbg.intakeVel;
        rbg.limelocked=false;
        rbg.beamscancount=0;
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
        Led.setPosition(rbg.ledgreen);

    }

    public void initalize() {
      //  follower = Constants.createFollower(hardwareMap);
        Hw_init();
        rbg.init();
        getAutoVars();

       // buildPaths();

        if (red) telemetry.addLine("Red  Selected");
        else telemetry.addLine("Blue  Selected");
        configinfo();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross) {
                red = false;
                recevieinfo = true;
              //  telemetry.addLine("Blue Selected");

            }
            if (gamepad1.circle) {
                red = true;
                recevieinfo = true;
               // telemetry.addLine("Red  Selected");
            }

            if (recevieinfo) {

                break;

               // telemetry.update();
            }

        }


       // telemetry.clear();

        if (red) telemetry.addLine("Red Alliance Selected");
        else telemetry.addLine("Blue Alliance Selected");

        telemetry.update();
        if (red) {
            Tx_offset = 0;
            target_id = 24;
            colorFactor = 1.0;
            rbg.targetGoalX=rbg.redGoalX;
              if(startY==0)  { startY=startY+rbg.rfyoffset;}
            if(startX==0){ startX=startX+rbg.rfxoffset;}
            Limelight.pipelineSwitch(6);//6 for highlight red , 2 for low light red
        } else {
            Tx_offset = 0;
            target_id = 20;
            colorFactor = -1.0;
           if(startY==0) startY=startY+rbg.bfyoffset;
           if(startX==0) startX=startX+rbg.bfxoffset;
            rbg.targetGoalX=rbg.blueGoalX;
            Limelight.pipelineSwitch(7);
        }
        if(startHeading==0) startHeading=1.5*Math.PI;
        Limelight.start();

    }

//    private void buildPaths() {
//
//        RHPshoot = follower.pathBuilder()
//                .addPath(new BezierLine(RHPStartPose, RHPShootPose))
//                .setLinearHeadingInterpolation(RHPShootPose.getHeading(), RHPShootPose.getHeading())
//                .build();
//
//        BHPshoot = follower.pathBuilder()
//                .addPath(new BezierLine(BHPStartPose, BHPShootPose))
//                .setLinearHeadingInterpolation(BHPStartPose.getHeading(), BHPShootPose.getHeading())
//                .build();
//
//    }







    void configinfo() {
        telemetry.addLine("Driver Cross select Blue side");
        telemetry.addLine("Driver Circle select  Red  side");
        telemetry.addLine("Gunner Right Bumper Confrim afters election");
        telemetry.addLine("*******************************************");
        telemetry.update();

    }



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

        y*= colorFactor;
        x*=colorFactor;


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

     //   telemetry.addData("Angle", pose.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
//        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));



      //  telemetry.update();
    }


    public void mecanumRobotCentricDrive(double y, double x, double rx){


        // Rotate the movement direction counter to the bot's rotation
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        //   telemetry.addData("Angle", pose.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
//        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));



        //  telemetry.update();
    }

public void turretspin()


{


}

    public void flywheel() {


        if(outtakestate) {
            if (pinpoint_nav&&!shooting) {
                flypower = rbg.flyspeedPP(flyCurrentVel);

            } else {
                flypower = rbg.flyspeed(flyCurrentVel, Ty);
               // hoodPos = rbg.flyhood(Ty);
            }

//            if (hoodPos > 0 && Math.abs(hoodPos - hoodLastPos) > 0.01) {
//                Hood.setPosition(hoodPos);
//                hoodLastPos = hoodPos;
//            }

        }
        else

        {
            if(intakefirst&&!topbb) {flypower=rbg.intakeflypower2;intakefirst=false;}
            if(manual)   {flypower = rbg.flyspeed(flyCurrentVel, manualTy);}

        }

        flyBot.setPower(flypower);
        flyTop.setPower(flypower);

    }

//    public void flywheelPP() {
//
//        double flypower=rbg.flyspeedPP(flyCurrentVel,dist);;
//        flyBot.setPower(flypower);
//        flyTop.setPower(flypower);
//        hoodPos=rbg.flyhoodPP(dist);
//        if(hoodPos>0 &&Math.abs(hoodPos-hoodLastPos)>0.01){
//            Hood.setPosition(hoodPos);
//            hoodLastPos=hoodPos;
//        }
//
//    }


    public void Hw_init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeLeft = hardwareMap.get(DcMotorEx.class,"intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class,"intakeRight");

      //  turretPosition=hardwareMap.get(DcMotorEx.class, "leftFront");

        flyBot = hardwareMap.get(DcMotorEx.class, "flyBot");
        flyTop = hardwareMap.get(DcMotorEx.class, "flyTop");
      //  turretSpin = hardwareMap.get(DcMotorEx.class, "turretSpin");
        botBB = hardwareMap.get(DigitalChannel.class, "botBB");
        topBB = hardwareMap.get(DigitalChannel.class, "topBB");
        midBB = hardwareMap.get(DigitalChannel.class, "midBB");

        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        configurePinpoint();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//todo

        Hood = hardwareMap.get(Servo.class, "Hood");
        turretLeft = hardwareMap.get(CRServo.class,"turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        Blocker = hardwareMap.get(Servo.class, "Blocker");
        Tripod = hardwareMap.get(Servo.class, "Tripod");
        Led = hardwareMap.get(Servo.class, "LED");
        Limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
        Pinpoint.setOffsets(-3.15, -4.9, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

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
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

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





    public void Intake(double targetvel)

    {

        double power=  rbg.intakePID.calculate(intakespeed,targetvel)+ rbg.intakef*targetvel;
      //  power= Range.clip(power,-0.8,0.8);
        if(targetvel==0) power=0;
        intakepower=power;
        intakeLeft.setPower(power);
        intakeRight.setPower(power);


    }



    public void getAutoVars(){


        try {
            startHeading = (double) blackboard.get("Heading");
        }
        catch (NullPointerException e){
            startHeading = 0;
        }

        try {
            startX = (double) blackboard.get("X");
        }

        catch (NullPointerException e){

                startX = 0;

        }

        try {
            startY = (double) blackboard.get("Y");
        }

        catch (NullPointerException e){
            startY =0;
        }

        try{
            red = (boolean) blackboard.get("COLOR");
        }
        catch (NullPointerException e){
            red = true;
        }

        try{
            Tgap = (int) blackboard.get("T");
        }
        catch (NullPointerException e){
            Tgap= 0;
        }


    }


    public boolean shoot(){
        switch (shootState){
            case START:
                drive = false;
                intaketargetvel=rbg.outtakVel;
                Hood.setPosition(rbg.hoodposition(pinpoint_nav,Ty));
                shooting = true;
                rbg.Txgap=30;
                if(manual) rbg.Txgap=1;
                rbg.beamscancount=0;
                shootState = ShootState.PRE_SHOOT;
                break;
            case PRE_SHOOT:
                if(rbg.flyspeedgap <= 40&& rbg.Txgap < 1.5){  // rbg.Txgap < 1
//                    drive = false;
                    Blocker.setPosition(rbg.blockOpen);
                    stoptimers(0, outtake);
                    shootState = ShootState.SHOOT;  //Unlock

                }
                break;



            case SHOOT:

                if (rbg.beamscanouttake(topbb,midbb,botbb )){//todo
                    stoptimers(0, outtake);
                    shootState = ShootState.DONE;
                    drive=true;
                }
                break;
            case DONE:
                if(stoptimers(200, outtake)) return true;
                break;


        }
        return false;
    }



}





















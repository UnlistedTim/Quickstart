
package org.firstinspires.ftc.teamcode;



import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class base {


    public final double ticksPerDegree = 957.0/180.0;

    public double redGoalX = 144;
    public double redGoalY = 144;
    public double blueGoalX = 0;
    public double blueGoalY = 144;
    public double targetGoalY = 144;
    public double targetGoalX = 144;
    public double Pi=Math.PI,intakeflypower1=0.3,intakeflypower2=0.6;







    public final double YOFFSET = 7.25; //TODO

    public final double HEADINGOFFSET = Math.PI/2; // TODO


    public final double REDXOFFSET = 144-7.5; //TODO

    public final double BLUEXOFFSET = 7.5; // TODO


    public final double  ledgreen=0.5, ledred=0.28;
    private Timer beamtimer;



    public boolean limelocked=false ,intakefirst=false,intakelast=false,beamcheck=false;
    InterpLUT Flylut = new InterpLUT();

    InterpLUT FlylutPP = new InterpLUT();
    InterpLUT Hoodlut = new InterpLUT();
    InterpLUT HoodlutPP = new InterpLUT();
    //public static double turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;//
    public static double turretkP = 0.025, turretkI = 0.0, turretkD = 0.001;//
    public static double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;
    public double flyspeedgap=500,Txgap=50,  turnMax=0.4,dist=50;


    public double targetVel=0;
    public double blockClose = 0.35, blockOpen = 0.46;
    public double tripodIdle = 1.0, tripodPark = 0.35;
    public int step=0,beamoncount=0,ballcount=0,beamoffcount=0,beamoncount1=0,step1=0,beamtimecount=0;


    //double turretPower=0;
    PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);
    double Tx_offset=0;
    int turretCwlim=-750;
    int turretCcwlim=750;
    public MedianFilter intakeCurrentFilter = new MedianFilter(10);

    public double intakeVel = 2500,outtakVel=2200;





    public class MedianFilter {
        private final double[] window;
        private int index = 0;
        private boolean filled = false;

        public MedianFilter(int size) {
            window = new double[size];
        }

        public double update(double value) {
            window[index] = value;
            index = (index + 1) % window.length;

            if (index == 0) {
                filled = true;
            }

            double[] temp = filled
                    ? window.clone()
                    : Arrays.copyOf(window, index);

            Arrays.sort(temp);

            return temp[temp.length / 2];
        }
    }


    public void init()

    {
        beamtimer=new Timer();
        flyPID.setPID(flyp, flyi, flyd);
        turretPID.setPID(turretkP, turretkI, turretkD);
        Flylut.add(-13.5,1720); //far
        Flylut.add(-12.7,1680); //far
        Flylut.add(-12.2,1630); //far
        Flylut.add(-11.85,1610); //far
        Flylut.add(-9.27,1400); //close
        Flylut.add(-2.71,1240); //close
        Flylut.add(6.28,1060); //close
        Flylut.add(11 , 1000); // close
        Hoodlut.add(-13.5,0.78);   //far
        Hoodlut.add(-12.7,0.75);   //far
        Hoodlut.add(-12.2,0.75);   //far
        Hoodlut.add(-11.8,0.75);   //far
        Hoodlut.add(-9.27,0.65);
        Hoodlut.add(-2.71,0.48);
        Hoodlut.add(6.28,0.2);
        Hoodlut.add(11,0.18);
        Flylut.createLUT();
        Hoodlut.createLUT();
        FlylutPP.add(0,800);
        FlylutPP.add(49.01,1080);
        FlylutPP.add(69.01,1180);
        FlylutPP.add(89.49,1290);
        FlylutPP.add(109.18,1420);
        FlylutPP.add(129.20,1540); //far
        FlylutPP.add(144.66,1630); //far
        FlylutPP.add(162.00,1720); //far
        HoodlutPP.add(0,0.15);
        HoodlutPP.add(49.01,0.15);
        HoodlutPP.add(69.01,0.48);
        HoodlutPP.add(89.49,0.55);
        HoodlutPP.add(109.18,0.65);
        HoodlutPP.add(129.20,0.8);//far
        HoodlutPP.add(144.66,0.8);//far
        HoodlutPP.add(162.00,0.8); //far
        FlylutPP.createLUT();
        HoodlutPP.createLUT();
   }

      public boolean beamintakecheck(boolean topon ,boolean boton)

      {
      switch (step) {


          case 0:
                    if (!topon)   {
                        step = 1;
                    }

                    break;
              case 1:
                    if (!boton) {
                        beamtimer.resetTimer();
                        beamoncount=0;
                        step = 2;
                         }
                    break;
              case 2:
                    if(beamtimer.getElapsedTime()>500)
                    {
                        step=0;
                        beamoncount=0;
                        return true;
                    }
                    if(boton) beamoncount++; else beamoncount=0;

                    if(beamoncount>2) step=1;
                    break;
                  }

            return false;
      }
    public boolean beamintakecheck1(boolean topon,boolean boton)

    {
        if(ballcount>1){
            beamtimecount=0;
            beamoffcount=0;
            beamoncount1=0;
            ballcount=0;
            beamcheck=false;
            return true;
        }
       //if(ballcount==0&&!topon) ballcount=1;
        if(!beamcheck&&!topon&!boton) {
            beamtimer.resetTimer();
            beamcheck=true;
            beamtimecount=0;
        }
        if(beamcheck) {
         if(boton)  beamtimecount++; else beamtimecount=0;
         if(beamtimecount>2) beamcheck=false;
        if(beamtimer.getElapsedTime()>600) {
                beamtimecount=0;
                beamoffcount=0;
                beamoncount1=0;
                ballcount=0;
                beamcheck=false;
                return true;
            }
        }



        switch (step1) {
            case 0:
                if (!boton)   {
                    beamoffcount++;
                }
                else beamoffcount=0;
                if(beamoffcount>3) {step1=1;}
                break;

            case 1:

                if (beamoncount1>2){
                    beamcheck=false;
                    if(!boton)  {ballcount++; step1=0;}
                }

               if(boton) beamoncount1++; else {beamoncount1=0;}
               break;



        }

        return false;
    }
    public boolean beamouttakecheck(boolean topon ,boolean boton)

    {
        if(ballcount>1&& beamoncount>15) {ballcount=0;beamoncount=0;return true;}
        if(beamoncount>2&&!topon) {ballcount++;}
        if (topon) beamoncount++; else beamoncount=0;

        switch (step) {


            case 0:
                if (boton)   step = 1;
                break;
            case 1:
                if (topon) {
                    beamtimer.resetTimer();
                    step = 2;
                }
                break;
            case 2:
                if(beamtimer.getElapsedTime()>500)
                {
                    step=0;
                    return true;
                }
                if(!topon||!boton)  step=1;
                break;
        }

        return false;
    }



    public double  turretturn(boolean outtake , boolean valid,int target, int turretPos, double tx, double offset){
        double turretPower;
        if (outtake)  {
            if(valid) {
                turretPower = turretPID.calculate(tx, offset);
                limelocked=true;
                Txgap=Math.abs(tx-offset);
                if (turretPos > turretCcwlim- 10 &&  turretPower  > 0) {
                    turretPower= -0.3;
//                    target = 0;
                    limelocked = false;
                }
                if (turretPos < (turretCwlim + 10) && turretPower< 0) {
                    turretPower= 0.3;
//                    target = 0;
                    limelocked = false;
                }

                turretPower = Range.clip(turretPower,-turnMax,turnMax);
                return  turretPower ;
            }

            if (limelocked) {
                turretPower=0;
                return turretPower;
            }


        }

        turretPower = turretPID.calculate( turretPos*0.35,  target*0.35);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);

        return turretPower;


    }

    public double  turretturnPP(boolean outtake , Pose2D p, int turretTicks){

        double dx = targetGoalX- p.getX(DistanceUnit.INCH);
        double dy = targetGoalY -p.getY(DistanceUnit.INCH);

        double targetAngle= Math.atan2(dy, dx) - (p.getHeading(AngleUnit.RADIANS) -Pi);
        if(Math.abs(targetAngle) >Pi) targetAngle=-Math.signum(targetAngle)*(2*Pi-Math.abs(targetAngle));
        dist=Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));


        //dist = rbg.calcDist(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), targetx, targety);

        double turretPower;
        if (outtake)  {
            turretPower = turretPID.calculate(turretTicks, Math.toDegrees(targetAngle) * ticksPerDegree);
            Txgap=Math.abs( (Math.toDegrees(targetAngle)  - (turretTicks / ticksPerDegree)));

            if (turretTicks> turretCcwlim- 10 &&  turretPower  > 0) {
                turretPower= -0.3;


            }
            if (turretTicks < (turretCwlim + 10) && turretPower< 0) {
                turretPower= 0.3;


            }



        }



       else  turretPower = turretPID.calculate( turretTicks*0.35,  0);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);
        return turretPower;



    }

    public double flyhood(double Ty) {

        double hoodLutGet;
        if (Ty < 11 && Ty > -13.5) {


            hoodLutGet = Hoodlut.get(Ty);
        }

        else hoodLutGet=0;

        return hoodLutGet;


    }

    public double flyhoodPP() {

        double hoodLutGet;
        if (dist < 162 && dist >= 0) {


            hoodLutGet = HoodlutPP.get(dist);
        }

        else hoodLutGet=0.8;

        return hoodLutGet;


    }

    public double calcTurretAngle(double robotAngle, double targetAngle, double minAngle, double maxAngle) {
        double desired = targetAngle - (robotAngle -Math.PI);
        desired = Math.atan2(Math.sin(desired), Math.cos(desired));
        if (desired < minAngle) return minAngle;
        if (desired > maxAngle) return maxAngle;
        return desired;
    }

    public double calcDist(double x0, double y0, double x1, double y1){
        return Math.sqrt (Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));

    }

    public double calcAbsAngle(double x0, double y0, double x1, double y1){
        double dx = x1 - x0;
        double dy = y1 - y0;
        return Math.atan2(dy, dx);

    }

    public double getVel(double ty){
        return Flylut.get(ty);
    }

    public double  flyspeed(double currentVel,double ty) {


        if (ty < 11 && ty > -13.5) {

            targetVel = Flylut.get(ty);// Tx offset

//            Tx_offset=0;

        }
        else targetVel = 1600;



        targetVel = Math.round(targetVel / 0.001) * 0.001;

        flyspeedgap=Math.abs(currentVel-targetVel);


        //   double power = flyPID.calculate(flyCurrentVel, targetVel) + flyf * targetVel;

        return(flyPID.calculate(currentVel, targetVel) + flyf * targetVel);


    }

    public double  flyspeedPP(double currentVel) {


        if (dist < 162 && dist >= 0) {

            targetVel = FlylutPP.get(dist);// Tx offset


        }
        else targetVel = 1500;



        targetVel = Math.round(targetVel / 0.001) * 0.001;

        flyspeedgap=Math.abs(currentVel-targetVel);


        //   double power = flyPID.calculate(flyCurrentVel, targetVel) + flyf * targetVel;

        return(flyPID.calculate(currentVel, targetVel) + flyf * targetVel);


    }



}

//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
//import com.arcrobotics.ftclib.util.InterpLUT;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//public class bass {
//
//
//
//    public void limeAutoAimer(boolean lim , int target, int turretPos, Limelight3A limelight){
//
////        int turretPos=turretSpin.getCurrentPosition();
//        int id;
//
//        if (lim)  {
//
//            LLResult result = limelight.getLatestResult();
//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                id = fiducial.getFiducialId(); // The ID number of the fiducial
//            }
//
//            limeValid = result.isValid();
//
//            if (limeValid && ((id == target_id)) ) {
//
//                //telemetry.addLine("detected");
//                //   flag[autoaim]=true;
//                // flag[limedetect]=true;
//                Tx = result.getTx();
//                Ty = result.getTy();
//                if (red && Ty < 10 && Ty> -10.5) Tx_offset = -1.5;
//
//                if (!red && Ty < 10 && Ty> -10.5) Tx_offset = 1.5;
//
//                else if (Ty < -10.5 && red) Tx_offset = -3;
//
//                else if (Ty < -10.5 && !red) Tx_offset = 2.5;
//                flag[detected]=true;
//
//                //turettarget=turretPos;
//                pid_power = turretPID.calculate(Tx, Tx_offset);
//                if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;flag[detected]=false;}
//                if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;flag[detected]=false;}
//                turretSpin.setPower(pid_power);
//
//                return;
//
//
//            }
//            else if (flag[detected]) {
//                turretSpin.setPower(0);
//                Tx=10;
//                return;
//            }
//
//
//
//        }
//        // non limalinment, contrl by encoder/angle
//
//        //      if(Math.abs(turretPos-target)<12) {pid_power=0; if (target!=0) turettarget=0;}// if reach the limit, turn to the middle
//
//        double turnpower = turretPID.calculate( turretPos*0.35,  target*0.35);
//
//        pid_power = Range.clip(turnpower, -MAX_TURN_POWER, MAX_TURN_POWER);
//
//        if(turretPos>turretcounterclock-10&& pid_power>0)  {pid_power=-0.2;turettarget=0;}
//        if(turretPos<(turrretclock+10)&& pid_power<0)  {pid_power=0.2;turettarget=0;}
//
//        turretSpin.setPower(pid_power);
//
//
//    }
//
//}


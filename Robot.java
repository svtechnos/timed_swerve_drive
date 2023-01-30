// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final double sped=0.1; 
  private static final double tremp=0.04;
  private static final double dremp=0.1;
  private static final double fremp=0.2;
  //even motors(ID) are turn
  private static final int mDeviceID[] = {1,2,3,4,5,6,7,8};
  private static final int tDeviceID[] = {21,23,24,22};
  private static final double tOffset[] = {93.78, 174.1,307.89,299.8};
  private Joystick joystick;
  private int i;
  private double dPower;
  private double dAngle;
  private double dPowerMin = 0.06;
  private double tF = 1;
  private double dF = 0.5;
  private double jXt = 0.006;
  private double mT = 0.4;
  private double mtPower;
  private double turn_in_progress = 5;
  private int counter;
  private Timer clock;
  private WPI_TalonSRX t[] = new WPI_TalonSRX[4];
  private CANSparkMax m[] = new CANSparkMax[8];
  private RelativeEncoder e[] = new RelativeEncoder[8];


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    clock = new Timer();
    joystick=new Joystick(0);
    for(int j=0;j<4;j++){
      t[j]=new WPI_TalonSRX(tDeviceID[j]);
      t[j].configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
    }
    for(i=0;i<8;i++){
      m[i] = new CANSparkMax(mDeviceID[i], MotorType.kBrushless);
      m[i].restoreFactoryDefaults();
      if(i%2==0){m[i].setOpenLoopRampRate(dremp);}
      else{m[i].setOpenLoopRampRate(tremp);}
      m[i].setIdleMode(IdleMode.kBrake);
      e[i] = m[i].getEncoder();
    }
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  public void clear_Motors(){
    for(i=0;i<8;i++){
      m[i].setOpenLoopRampRate(fremp);
      m[i].stopMotor();
      m[i].setOpenLoopRampRate(dremp);
    }
  }
  public void test_forward(){
    for(i=0;i<8;i+=2){
      m[i].set(sped);
    }
  }
  public void test_encoder(){
    for(i=0;i<4;i++){
      System.out.println("Encoder "+(i+1)+":"+(((t[i].getSelectedSensorPosition()*360/4096)+tOffset[i])%360));
      
    }
  }
  public void test_Motors(){
    if(counter < 16){
      if(counter%2!=0){
        m[counter/2].set(sped);
        System.out.println("e_1 pos: "+e[counter/2].getPosition());
      } else {clear_Motors();}
      if(clock.get()>=1){
        counter++;
        clear_Motors();
        clock.reset();
        clock.start();
      }
    }
  }
  public double deltaMod(double t, double c){
    double d;
    d = Math.abs(t-c);
    if(d<4){
      return 0;
    }
    if(d>90){
      d=180-d;
      if(t>c){return 1*d;}
      else{return -1*d;}
    }else{
      if(t>c){return -1*d;}
      else{return 1*d;}
    }
  }
  public void test_joystick(){
    double jX;
    double jY;
    double cAngle;
    jX = joystick.getX();
    jY = joystick.getY()*-1;
    for(i=1;i<8;i+=2){
      cAngle=(((t[(i-1)/2].getSelectedSensorPosition())*360/4096)+(tOffset[(i-1)/2]))%360;
      dPower = Math.sqrt(((jX*jX)+(jY*jY))/2);
      if(dPower<dPowerMin){
        dAngle = cAngle;
        dPower = 0;
      }else{
        if(Math.abs(jX)<jXt){
          if(jY>0){
            dAngle=90;
          }else{
            dAngle=270;
          }
        }else{
          dAngle=(Math.atan(Math.abs(jY)/Math.abs(jX)))*360/(2*3.1415);
          if(jX<=0 &&jY>=0){dAngle=180-dAngle;}
          else if(jX<0 &&jY<0){dAngle+=180;}
          else if(jX>0 &&jY<0){dAngle=360-dAngle;}
        }
      }
      if(dAngle>180){
        dAngle-=180;
        dPower*=-1;
      }
      double dir;
      dir= deltaMod(dAngle, cAngle);
      int cc,dd;
      cc=(int)cAngle;
      dd=(int)dAngle;
      System.out.print("cAngle is: "+cc);
      System.out.println(" dAngle is: "+dd);
      mtPower=tF*dir/180;
      if(Math.abs(mtPower)>mT) mtPower=mT*mtPower/Math.abs(mtPower);
      m[i].set(mtPower);
      if(Math.abs(dir)<turn_in_progress){m[i-1].set(dF*dPower);}
    }
  }

  @Override
  public void teleopInit() {
    counter = 0;
    clock.reset();
    clock.start();
    clear_Motors();
  }
  @Override
  public void teleopPeriodic() {test_joystick();}
  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}
}
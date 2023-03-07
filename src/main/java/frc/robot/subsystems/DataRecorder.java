/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataRecorder extends SubsystemBase {

  public class datapoint{
    // first entry is timestamp in milliseconds
    public static final int RunTimestamp = 0;
    public static final int Drive_X = 1;
    public static final int Drive_Y = 2;
    public static final int Drive_Z = 3;
    public static final int GyroAngle = 4;
    public static final int ArmPosition = 5;
    public static final int ExtensionPosition = 6;
    public static final int WristPosition = 7;
    public static final int IntakeSpeed = 8;
    public static final int unused9 = 9;
    public static final int frontLeftDistance = 10;
    public static final int frontLeftVelocity = 11;
    public static final int frontRightDistance = 12;
    public static final int frontRightVelocity = 13;    
    public static final int rearLeftDistance = 14;
    public static final int rearLeftVelocity = 15;
    public static final int rearRightDistance = 16;
    public static final int rearRightVelocity = 17;   
    }

  
  private double[] blankvalues = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  private double[] datavalues = blankvalues; // same number of datapoints from  list above

  //private FileInputStream in = null;
 // private FileOutputStream outFile = null;
     private BufferedWriter outBuffer = null;
     private FileWriter outFile = null;

//private final Servo m_Servo = new Servo(0);
  
  public DataRecorder() {
    String test = SmartDashboard.getString("RecordfileName", "file.csv");
    SmartDashboard.putString("RecordfileName", test);
    SmartDashboard.putBoolean("RecordingOn", false); // default to not recording
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    //SmartDashboard.putNumberArray("recordedValues", datavalues);

    boolean recordingOn = SmartDashboard.getBoolean("RecordingOn", false);

    if (recordingOn && outBuffer==null){ startRecording(); }
    if (!recordingOn && outBuffer!=null){ endRecording(); }

    //SmartDashboard.putString("DataValues" )
    if (recordingOn && outBuffer!=null) {
      //datavalues[0] = System.currentTimeMillis();
      datavalues[datapoint.Drive_X] = SmartDashboard.getNumber("dataRecorder." + datapoint.Drive_X, 0);
      datavalues[datapoint.Drive_Y] = SmartDashboard.getNumber("dataRecorder." + datapoint.Drive_Y, 0);
      datavalues[datapoint.Drive_Z] = SmartDashboard.getNumber("dataRecorder." + datapoint.Drive_Z, 0);
      datavalues[datapoint.GyroAngle] = SmartDashboard.getNumber("dataRecorder." + datapoint.GyroAngle, 0);
      datavalues[datapoint.ArmPosition] = SmartDashboard.getNumber("dataRecorder." + datapoint.ArmPosition, 0);
      datavalues[datapoint.ExtensionPosition] = SmartDashboard.getNumber("dataRecorder." + datapoint.ExtensionPosition, 0);
      datavalues[datapoint.WristPosition] = SmartDashboard.getNumber("dataRecorder." + datapoint.WristPosition, 0);
      datavalues[datapoint.IntakeSpeed] = SmartDashboard.getNumber("dataRecorder." + datapoint.IntakeSpeed, 0);
      datavalues[datapoint.unused9] = SmartDashboard.getNumber("dataRecorder." + datapoint.unused9, 0);

      datavalues[datapoint.frontLeftDistance] = SmartDashboard.getNumber("dataRecorder." + datapoint.frontLeftDistance, 0);
      datavalues[datapoint.frontLeftVelocity] = SmartDashboard.getNumber("dataRecorder." + datapoint.frontLeftVelocity, 0);
      datavalues[datapoint.frontRightDistance] = SmartDashboard.getNumber("dataRecorder." + datapoint.frontRightDistance, 0);
      datavalues[datapoint.frontRightVelocity] = SmartDashboard.getNumber("dataRecorder." + datapoint.frontRightVelocity, 0);
      datavalues[datapoint.rearLeftDistance] = SmartDashboard.getNumber("dataRecorder." + datapoint.rearLeftDistance, 0);
      datavalues[datapoint.rearLeftVelocity] = SmartDashboard.getNumber("dataRecorder." + datapoint.rearLeftVelocity, 0);
      datavalues[datapoint.rearRightDistance] = SmartDashboard.getNumber("dataRecorder." + datapoint.rearRightDistance, 0);
      datavalues[datapoint.rearRightVelocity] = SmartDashboard.getNumber("dataRecorder." + datapoint.rearRightVelocity, 0);

      writeValuesToFile(); 
    }
    
    }
  
  private void writeValuesToFile(){
     if (outBuffer==null) {return;}
     
     // String stringdatavalues = datavalues.toString();
     //StringBuilder line = new StringBuilder();
     String line = "";
     for (int i=0; i < datavalues.length; i++) {
        //line.append(datavalues[i]);
        line += datavalues[i];
        if (i != datavalues.length - 1) { line +=",";} // line.append(','); }     
     }
     //line.append("\n");
     //line +="\n";
     try {
      outBuffer.write(line);
      outBuffer.write("\n");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // SmartDashboard.putString("Data Values", line.toString());
  }
  

  public void startRecording(){
   
      //outFile = new FileOutputStream("output.txt"); 
      String fileNameString = SmartDashboard.getString("RecordfileName", "file.csv");
      File file = new File("/home/lvuser/" + fileNameString);
      try {
        if (file.exists()) {
        outFile = new FileWriter(file,false); 
        }
        else {
          outFile = new FileWriter(file);
        }
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

      file.setWritable(true);
      file.setReadable(true);

      outBuffer = new BufferedWriter(outFile);

      datavalues = blankvalues;
      writeValuesToFile();
  }
  
   public void endRecording(){

    if (outFile != null)
    { 
      try {
        outBuffer.flush();
        outFile.flush();
        outBuffer.close();
        outFile.close();
        outBuffer = null;
        outFile = null;
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

  }

  public void recordValue(Integer ix, double valueToRecord){
//    if (outFile==null) {return;}
    datavalues[ix] = valueToRecord;
  }

  // LoadFile method reads all lines of files and returns LIST of doubles
  public List<double[]> LoadFile(String fileName) {

    List<double[]> lines = new ArrayList<double[]>();
    String[] strRow;
    double[] row;
    // String errors = "";
    Integer rowNum = 0;

    // string xx = Filesystem.getDeployDirectory().getAbsolutePath()
    try (BufferedReader br = new BufferedReader(new FileReader("/home/lvuser/deploy/" + fileName))) {
      for(String line; (line = br.readLine()) != null; ) {
        rowNum += 1;
        //System.out.println(line);
        //SmartDashboard.putString("fileLine", line);
        strRow = line.split(",");
        row = new double[strRow.length];
        // if (strRow.length < 9){errors += "line " + rowNum + " columns=" + strRow.length; }
        for (int i=0; i<strRow.length; i++) {
          row[i] = Double.parseDouble(strRow[i]); //(double)test2; //Double.parseDouble(strDatapoint.toString());
        }
        lines.add(row);
      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    //SmartDashboard.putString("fileLine", "SUCCESS!");
    return lines;
  }
}


package frc.robot.util;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Toolkit.CT_LIDARSensor;

import java.util.ArrayList;

public class LidarManager implements Runnable {

  private CT_LIDARSensor m_Lidar = new CT_LIDARSensor(Port.kOnboard);
  private ArrayList<Double> m_Last5Dist = new ArrayList<Double>();
  private double m_TotalDist = 0;
  private double m_Average = 0;

  public LidarManager() {
  }

  @Override
  public void run() {
    while (true)
      try {
        Thread.sleep(20);
        m_Lidar.measureDistanceOnce();
        addLidarValue();
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
        
        //System.out.println(getLidarAverage());
    }

    public void addLidarValue() {
        // add value to array here
        double m_Dist = m_Lidar.getDistanceInches();
        if(m_Last5Dist.size() == 5) {
          m_TotalDist -= m_Last5Dist.get(0);
          m_TotalDist += m_Dist;
          m_Last5Dist.remove(0);
          m_Last5Dist.add(Double.valueOf(m_Dist));
        } else {
          m_Last5Dist.add(Double.valueOf(m_Dist));
          m_TotalDist += m_Dist;
        }
      }

      public void clearLidarAverage() {
        // clear average values
        m_Last5Dist.clear();
        m_TotalDist = 0;
        m_Average = 0;
      }

      public double getLidarAverage() {
        // return average
        
        m_Average = m_TotalDist / m_Last5Dist.size();
        return m_Average;
      }

      public CT_LIDARSensor getLidar() {
        return m_Lidar;
      }
}
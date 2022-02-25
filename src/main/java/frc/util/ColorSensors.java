import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensors
{
    private final I2C.Port port;

    private final ColorMatch m_colorMatcher = new ColorMatch();

    ColorSensor colorSensor = new ColorSensor(Port.kOnboard);

    public ColorSensor(I2C.Port port)
    {
        this.port = port;
    }


    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);



    enum Colors
    {
        RED,
        BLUE,
        NONE;
    }

    //Colors red = Colors.RED;

    //Colors blue = Colors.BLUE;

    //Colors white = Colors.NONE;

    m_colorMatcher.addColorMatch(blue);
    m_colorMatcher.addColorMatch(red);

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor); 



//Testing enums

public Colors getValue()
{

    if (match.color == blue) 
    {
        return BLUE;
    } 
    
    else if (match.color == red) 
    {
        return RED;

      } 
    else 
    {
        return NONE;
    }
}


switch(red)
{
    case RED:
    System.out.println("RED");
    case BLUE:
    System.out.println("BLUE");
    case NONE;
    System.out.println("NONE");
}
}

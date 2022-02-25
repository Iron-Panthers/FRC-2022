import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import javax.lang.model.util.ElementScanner6;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensors
{
    private final I2C.Port port;

    private final ColorMatch m_colorMatcher = new ColorMatch();

    private ColorSensorV3 colorsensor3;


    public ColorSensors(I2C.Port port, ColorSensorV3 colorsensor3)
    {
        this.port = port;
        this.colorsensor3 = colorsensor3;
    }


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



    public Colors getValue()
    {
   
        if(colorsensor3.getProximity()<=240)
        {
            Color detectedColor = colorsensor3.getColor();
    
            if(detectedColor )
            {
                return Colors.RED;
            }

            else if(detectedColor )
            {
                return Colors.BLUE;
            }
            else
            {
                return Colors.NONE; 
            }

        }
    }
}

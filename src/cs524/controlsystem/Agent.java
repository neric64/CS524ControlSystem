package cs524.controlsystem;

//=============================================================================================================================================================
/**
 * Defines an arbitrary physical agent (assumed to be a boat in CS 524 Task 3) in two-dimensional space. It starts at a position with an azimuth and speed. From
 * there, only putative rudder can be controlled, which then changes the azimuth per update. The new position is based on the change in position from the old
 * position with respect to the current speed and azimuth.
 * 
 * Do not change this file.
 * 
 * @author Dan Tappan [22.04.14]
 */
public class Agent implements I_Updateable
{
   /** the plus/minus band for accepting that the current azimuth has reached the target azimuth */
   private static final double EPSILON = 0.1;

   /** the positive and negative limit to azimuth delta */
   private static final double AZIMUTH_DELTA_LIMIT = 5;

   /** for noninstantaneous deflection, the current azimuth updates toward the target azimuth by this value once per update */
   private static final double AZIMUTH_DELTA_DELTA = 0.1;

   /** the degrees in a circle */
   private static final double CIRCLE_DEGREES = 360;

   /** the x position of the agent */
   private double _x;

   /** the y position of the agent */
   private double _y;

   /** the current speed of the agent */
   private final double _speed;

   /** the current azimuth of the agent */
   private double _azimuth;

   /** the current signed angle on [-AZIMUTH_DELTA_LIMIT,+AZIMUTH_DELTA_LIMIT] by which the azimuth changes per update */
   private double _azimuthDelta;

   /** the desired signed angle on [-AZIMUTH_DELTA_LIMIT,+AZIMUTH_DELTA_LIMIT] by which the azimuth changes per update */
   private double _azimuthDeltaTarget;

   /** whether deflection to a new azimuth delta is instantaneous, otherwise incremental */
   private final boolean _isDeflectionInstantaneous;

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * @param x - the <i>x</i> position of the agent
    * @param y - the <i>y</i> position of the agent
    * @param speed - the current speed of the agent
    * @param azimuth - the current azimuth of the agent
    * @param isDeflectionInstantaneous - whether deflection to a new azimuth delta is instantaneous, otherwise incremental
    */
   public Agent(final double x, final double y, final double speed, final double azimuth, final boolean isDeflectionInstantaneous)
   {
      Assert.nonnegative(speed);
      Assert.range(azimuth, 0, 360);

      _x = x;
      _y = y;

      _speed = speed;

      _azimuth = azimuth;
      _azimuthDelta = 0;
      _azimuthDeltaTarget = 0;
      _isDeflectionInstantaneous = isDeflectionInstantaneous;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the current azimuth of the agent.
    * 
    * @return the azimuth
    */
   public double getAzimuth()
   {
      return _azimuth;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the current signed angle on [-AZIMUTH_DELTA_LIMIT,+AZIMUTH_DELTA_LIMIT] by which the azimuth changes per update.
    * 
    * @return the angle
    */
   public double getAzimuthDelta()
   {
      return _azimuthDelta;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the desired signed angle on [-AZIMUTH_DELTA_LIMIT,+AZIMUTH_DELTA_LIMIT] by which the azimuth changes per update.
    * 
    * @return the angle
    */
   public double getAzimuthDeltaTarget()
   {
      return _azimuthDeltaTarget;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the current speed of the agent.
    * 
    * @return the speed
    */
   public double getSpeed()
   {
      return _speed;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the state of the agent in string comma-separated-values form.
    * 
    * @return the state
    */
   public String getStateGnuplot()
   {
      // xxx add vector barbs
      
      return (_x + " " + _y);
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the state of the agent in string comma-separated-values form.
    * 
    * @return the state
    */
   public String getStateCSV()
   {
      return (_x + "," + _y + "," + _speed + "," + _azimuth + "," + _azimuthDelta + "," + _azimuthDeltaTarget + "," + _isDeflectionInstantaneous);
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the <i>x</i> position of the agent.
    * 
    * @return the position
    */
   public double getX()
   {
      return _x;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the <i>y</i> position of the agent.
    * 
    * @return the position
    */
   public double getY()
   {
      return _y;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets whether deflection to a new azimuth delta is instantaneous, otherwise incremental.
    * 
    * @return the result
    */
   public boolean isDeflectionInstantaneous()
   {
      return _isDeflectionInstantaneous;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Determines whether the agent is within a certain distance of a point.
    * 
    * @param x - the <i>x</i> position of the point
    * @param y - the <i>y</i> position of the point
    * @param distance - the distance threshold
    * 
    * @return the result
    */
   public boolean isProximate(final double x, final double y, final double distance)
   {
      Assert.positive(distance);

      double distanceX = (_x - x);
      double distanceY = (_y - y);

      double distanceActual = Math.sqrt((distanceX * distanceX) + (distanceY * distanceY));

      boolean isProximate = (distanceActual <= distance);

      return isProximate;
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Sets the desired signed angle on [-AZIMUTH_DELTA_LIMIT,+AZIMUTH_DELTA_LIMIT] by which the azimuth changes per update. If the deflection mode is
    * instantaneous, then this angle is achieved immediately; otherwise, the current angle updates toward the desired angle by <code>AZIMUTH_DELTA_DELTA</code>.
    * 
    * @param azimuthDelta - the angle
    */
   public void setAzimuthDeltaTarget(final double azimuthDelta)
   {
      Assert.range(azimuthDelta, -AZIMUTH_DELTA_LIMIT, AZIMUTH_DELTA_LIMIT);

      _azimuthDeltaTarget = azimuthDelta;

      if (_isDeflectionInstantaneous)
      {
         _azimuthDelta = _azimuthDeltaTarget;
      }
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Gets the state of the agent in string form.
    */
   @Override
   public String toString()
   {
      return ("Agent{x=" + _x + " y=" + _y + " speed=" + _speed + " azimuth=" + _azimuth + " azimuthDelta=" + _azimuthDelta + " azimuthDeltaTarget="
            + _azimuthDeltaTarget + " isDeflectionInstantaneous=" + _isDeflectionInstantaneous + "}");
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * {@inheritDoc}
    */
   @Override
   public void update_()
   {
      updateAzimuth();

      updatePosition();
   }
   
   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Updates the azimuth of the agent by one delta toward the target.
    */
   private void updateAzimuth()
   {
      if (!_isDeflectionInstantaneous)
      {
         if (_azimuthDelta < (_azimuthDeltaTarget - EPSILON))
         {
            _azimuthDelta += AZIMUTH_DELTA_DELTA;
         }
         else if (_azimuthDelta > (_azimuthDeltaTarget + EPSILON))
         {
            _azimuthDelta -= AZIMUTH_DELTA_DELTA;
         }
         else
         {
            _azimuthDelta = _azimuthDeltaTarget;
         }
      }

      Assert.range(_azimuthDelta, -AZIMUTH_DELTA_LIMIT, AZIMUTH_DELTA_LIMIT);

      _azimuth += _azimuthDelta;

      if (_azimuth < 0)
      {
         _azimuth = (CIRCLE_DEGREES + _azimuth);
      }

      _azimuth %= CIRCLE_DEGREES;

      // System.err.println(_azimuth + " " + _azimuthDelta);
   }

   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Updates the position of the agent based on the current speed and azimuth.
    */
   private void updatePosition()
   {
      double azimuthRadians = Math.toRadians(90 - _azimuth);

      double deltaX = (Math.cos(azimuthRadians) * _speed);
      double deltaY = (Math.sin(azimuthRadians) * _speed);

      _x += deltaX;
      _y += deltaY;

      // System.err.println(_x + " " + _y + " " + _azimuth +" "+ _azimuthDelta);
   }
}

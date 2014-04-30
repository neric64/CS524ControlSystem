package cs524.controlsystem;

//=============================================================================================================================================================
/**
 * Defines to contract for any agent whose state can be updated.
 * 
 * Do not change this file.
 * 
 * @author Dan Tappan [22.04.14]
 */
public interface I_Updateable
{
   // ---------------------------------------------------------------------------------------------------------------------------------------------------------
   /**
    * Updates the current azimuth and position of the agent.
    */
   public void update_();
}

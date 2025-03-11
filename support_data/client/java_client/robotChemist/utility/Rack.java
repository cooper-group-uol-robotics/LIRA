package robotChemist.utility;

import robotChemist.exceptions.GraspPositionException;

import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Workpiece;

/***
 * A class that represents a rack. It wraps {@link roboticsAPI.geometricModel.Workpiece} and provides utility
 * methods to retrieve different frames for rack and vial handling.
 * @author stoic-roboticist
 *
 */
public class Rack 
{
	private Workpiece rack;
	private int numVials;
	
	/***
	 * Constructs a rack object from the given Workpiece. The constructed rack
	 * is not attached to a frame.
	 * @param rack - the {@link Workpiece} representing the rack stored in the Template data.
	 */
	public Rack(Workpiece rack)
	{
		this.rack = rack;
		this.numVials = rack.getAllFrames().size() - 1; 
	}
	
	/***
	 * Constructs a rack object from the given Workpiece and rack holder frame. The constructed
	 * rack is attached to the given frame.
	 * @param rack - the {@link Workpiece} representing the rack stored in the Template data.
	 * @param rackHolderFrame - the object frame representing the rack location in space.
	 */
	public Rack(Workpiece rack, ObjectFrame rackLocationFrame)
	{
		this.rack = rack;
		this.rack.attachTo(rackLocationFrame);
		System.out.println(this.rack.getFrame("GraspFrame").toString());
		this.numVials = rack.getAllFrames().size() - 1; 
	}
	
	/***
	 * Gets the rack grasp frame.
	 * @return the rack grasp frame as specified by the Template Workpiece data.
	 */
	public ObjectFrame getGraspFrame()
	{
		return rack.getFrame("GraspFrame");
	}
	
	/***
	 * Gets the grasp frame for the given vial index.
	 * @param index - the index of the vial to be grasped.
	 * @return the vial grasp frame as specified by the Template Workpiece data.
	 * @throws GraspPositionException if the given vial index is incorrect
	 */
	public ObjectFrame getVialGraspFrame (int index) throws GraspPositionException
	{
		if (numVials == 0 || index < 0 || index > numVials)
		{
			throw new GraspPositionException("Incorrect Index"); 
		}
		else
		{
			return rack.getFrame(String.format("GraspFrame/Vial1GraspFrame/Vial%d", index));
		}
	}
	
	/***
	 * 
	 * @return the {@link Workpiece} wrapped by with the rack object.
	 */
	public Workpiece getWorkpiece()
	{
		return this.rack;
	}
	
	/***
	 * Change the rack location in space to the new given frame.
	 * @param newFrame - the object frame representing the new rack location in space.
	 */
	public void moveToFrame(ObjectFrame newFrame)
	{
		this.rack.detach();
		this.rack.attachTo(newFrame);
	}
			
}

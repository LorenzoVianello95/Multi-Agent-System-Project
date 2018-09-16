/*
 * Simple structure for a data packet.
 * 
 * @author dario albani
 * @mail dario.albani@istc.cnr.it
 */


package sim.app.firecontrol;

import java.util.LinkedHashSet;
import java.util.Set;
import sim.util.Int2D;

import java.lang.NullPointerException;

public class DataPacket{

	public class Header{
		public int id;
		public int timestamp;
		public int r;

		public Header(UAV uav,int step, int r){
			//TODO
			this.id= uav.id;
			this.timestamp= step;
			this.r=r;
			//System.err.println("TODO: You have to define the header. Maybe a timestamp and an ID?");
		}
	};

	public class Payload{
		public double x; //x position in the world
		public double y; //y position in the world
		public double z; //z position in the world
		public Set<Integer> rIDs;// elenco degli uavs che si sono gia passati questo messaggio
		public Int2D taskPos;
		public Int2D prevPos;
		public int mode;
		public Set<WorldCell> knownCells; 

		public Payload(UAV uav){
			//TODO
			try{
				this.x=uav.x;
				this.y=uav.y;
				this.z=uav.z;
				this.rIDs=new LinkedHashSet<>();
				this.taskPos= uav.myTask.centroid;
				this.prevPos=uav.prevTask;
				this.mode=uav.mode;
				this.knownCells= uav.knownCells;
				}catch(NullPointerException e){}
			
			
			//System.err.println("TODO: You have to define the payload. What are you going to share?");
		}
	};

	public Header header;
	public Payload payload;

	//TODO
	//define the data packet according to your payload and your header.
	//please, note that if you do not define a good header you could have problem 
	//with duplicates messages
	public DataPacket(UAV uav,int step,int r){
		this.header = new Header(uav,step,r);
		this.payload = new Payload(uav);
	}
}

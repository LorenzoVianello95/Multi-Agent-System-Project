/**
 * A single UAV running over the simulation. 
 * This class implements the class Steppable, the latter requires the implementation 
 * of one crucial method: step(SimState).
 * Please refer to Mason documentation for further details about the step method and how the simulation
 * loop is working.
 * 
 * @author dario albani
 * @mail albani@dis.uniroma1.it
 * @thanks Sean Luke
 */


/**
*TODO:
*-droni vanno subito a task piu vicino, e si dividono le celle intorno, se il numero di droni è troppo alto il drone va in centro e ricerca 
*nuovo task con meno uav
*/
package sim.app.firecontrol;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.lang.ArrayIndexOutOfBoundsException;
import java.lang.Math;
import java.lang.NullPointerException;

import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Double3D;
import sim.util.Double2D;
import sim.util.Int3D;
import sim.util.Int2D;

public class UAV implements Steppable{
	private static final long serialVersionUID = 1L;

	// Agent's variable
	public int id; //unique ID
	public double x; //x position in the world
	public double y; //y position in the world
	public double z; //z position in the world
	public Double3D target; //UAV target
	public AgentAction action; //last action executed by the UAV
	public static double communicationRange = 30; //communication range for the UAVs

	// Agent's local knowledge 
	public Set<WorldCell> knownCells; 
	public LinkedList<Int2D> avoidTasks;
	public LinkedList<Int2D> targetCells;
	public LinkedList<Int2D> targetCells2;
	public Task myTask;
	public Int2D prevTask;
	
	// variabile che serve a trattare il drone come una macchina a stati
	public int mode=0;
        public double dep=2;
        
        public boolean tgt= false;
	
	// Agent's settings - static because they have to be the same for all the 
	// UAV in the simulation. If you change it once, you change it for all the UAV.
	public static double linearvelocity = 0.04;
        //con velocità 0.02 è buona profondità 1

	//used to count the steps needed to extinguish a fire in a location
	public static int stepToExtinguish = 10;
	//used to remember when first started to extinguish at current location
	private int startedToExtinguishAt = -1;

	public UAV(int id, Double3D myPosition){
		//set agent's id
		this.id = id;
		//set agent's position
		this.x = myPosition.x;
		this.y = myPosition.y;
		this.z = myPosition.z;
		//at the beginning agents have no action
		this.action = null;
		//at the beginning agents have no known cells 
		this.knownCells = new LinkedHashSet<>();
		this.targetCells= new LinkedList<>();
		this.avoidTasks= new LinkedList<>();
		this.targetCells2= new LinkedList<>();
	}

	// DO NOT REMOVE
	// Getters and setters are used to display information in the inspectors
	public int getId(){
		return this.id;
	}

	public void setId(int id){
		this.id = id;
	}

	public double getX(){
		return this.x;
	}

	public void setX(double x){
		this.x = x;
	}

	public double getY(){
		return this.y;
	}

	public void setY(double y){
		this.y = y;
	}

	public double getZ(){
		return this.z;
	}

	public void setZ(double z){
		this.z = z;
	}

	/**
	 *  Do one step.
	 *  Core of the simulation.   
	 */
	public void step(SimState state){
		Ignite ignite = (Ignite)state;

		//select the next action for the agent
		AgentAction a = nextAction(ignite);
                
		
		switch(a){	
		case SELECT_TASK:
			// ------------------------------------------------------------------------
			// this is where your task allocation logic has to go. 
			// be careful, agents have their own knowledge about already explored cells, take that 
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your task allocation strategy
			//System.err.println("TODO: and now? Use one of methods for tasks assignment!");

			selectTask(ignite); //<- change the signature if needed

			this.action = a;
			break;

		case SELECT_CELL:
			// ------------------------------------------------------------------------
			// this is where your random walk or intra-task allocation logic has to go. 
			// be careful, agents have their own knowledge about already explored cells, take that 
			// in consideration if you want to implement an efficient strategy.
			// TODO Implement here your random walk or intra-task allocation strategy
			//System.err.println("TODO: and now? Use random walk or task assignment!");
			
			//System.err.println(this.id+ "mode"+ this.mode);
			selectCell(); //<- change the signature if needed


			
		case MOVE:
			move(state);
			break;

		case EXTINGUISH:
			//if true set the cell to be normal and foamed
			if(extinguish(ignite)){
				//retrieve discrete location of this
				Int3D dLoc = ignite.air.discretize(new Double3D(this.x, this.y, this.z));
				//extinguish the fire
				((WorldCell)ignite.forest.field[dLoc.x][dLoc.y]).extinguish(ignite);
                                ignite.cellsEstinguesh++;
				this.target=null;
			}

			this.action = a;
			break;

		default:	
			System.exit(-1);
		}
	}

	/**
	 * What to do next?
	 * TODO Feel free to modify this at your own will in case you have a better 
	 * strategy
	 */ 
	private AgentAction nextAction(Ignite ignite){
		//if I do not have a task I need to take one
                
		if(this.myTask == null){
			return AgentAction.SELECT_TASK;
		}
		//else, if I have a task but I do not have target I need to take one
		else if(this.target == null){
			//DataPacket p= new DataPacket(this,(int) ignite.schedule.getSteps());
			//sendData(p,ignite);
			return AgentAction.SELECT_CELL;
			
		}
		//else, if I have a target and task I need to move toward the target
		//check if I am over the target and in that case execute the right action;
		//if not, continue to move toward the target
		else if(this.target.equals(ignite.air.discretize(new Double3D(x, y, z)))){
		
			//if on fire then extinguish, otherwise move on
			//System.err.println("_____"+x+","+y);
			WorldCell cell = (WorldCell)ignite.forest.field[(int) x][(int) y];
			//store the knowledge for efficient selection
                        if(cell.type.equals(CellType.FIRE) || cell.type.equals(CellType.EXTINGUISHED) || cell.type.equals(CellType.WATER)){
                            this.knownCells.add(cell);                      
                        }
                        if(cell.type.equals(CellType.NORMAL) && Math.random() < 0.3){
                            this.knownCells.add(cell); 
                        }

			//nearCellAdd(new Int2D(cell.x,cell.y),ignite);	

                        if(this.myTask.centroid.x>=this.myTask.centroid.y){
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato dx
                                                                                nearCellAddDX(new Int2D(cell.x,cell.y),ignite);
										}
							else{
							//lato alto
                                                                 nearCellAddUP(new Int2D(cell.x,cell.y),ignite);
								}
									}

			else {
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato basso
                                                                                nearCellAddDWN(new Int2D(cell.x,cell.y),ignite);
													}
							else{
							//lato sx
                                                                nearCellAddSX(new Int2D(cell.x,cell.y),ignite);
								}
									}

			if(this.mode==0 ){ //|| this.myTask.isExausted()
				this.myTask = null;
				this.target = null;
				System.err.println("annullare");
				this.targetCells= new LinkedList<>();
				this.targetCells.addLast(new Int2D((int)x,(int)y));
				this.targetCells2= new LinkedList<>();
				this.targetCells2.addLast(new Int2D((int)x,(int)y));
				this.mode=1;
				}

			//TODO maybe, you can share the knowledge about the just extinguished cell here!
			else if(this.myTask != null && mode!=0){
					try{
						int[] taskDisp= retrieveAgents(ignite);
						//System.err.println("kk"+taskDisp[0]+taskDisp[1]+taskDisp[2]);
						if(taskDisp[ignite.tasks.indexOf(this.myTask)]>2){
								System.err.println("xxxxx");
								this.mode=0;
								this.prevTask=this.myTask.centroid;
								this.myTask = null;
								this.target = null;
								//DataPacket p20= new DataPacket(this,(int) ignite.schedule.getSteps(),20);
								//sendData(p20,ignite);
										}
				 		}catch(ArrayIndexOutOfBoundsException e){}
					}

			if(cell.type.equals(CellType.FIRE))
				{
				try{
				//this.myTask.notifyNewFire(new WorldCell((int)this.x,(int)this.y,CellType.FIRE));
				}catch(NullPointerException e){System.err.println(cell.type);}
				return AgentAction.EXTINGUISH;
				}
			else
				return AgentAction.SELECT_CELL;

		}else{
			return AgentAction.MOVE;
		}		
	}

	/**
	 * Take the centroid of the fire and its expected radius and extract the new
	 * task for the agent.
	 */
	private void selectTask(Ignite ignite) {
		//remember to set the new task at the end of the procedure
		//Questo approccio ha il vantaggio che mette abbastanza bene i droni, ma non sempre funziona
		// probabilmente funziona meglio con tanti uav
		Task taskCloser=null;
		//System.err.println("ggg");
		if(this.mode==0)
				{
					System.err.println("rid");
					Int2D midC= new Int2D(30,30);
					taskCloser=new Task(midC,1);
				}
		else if(this.mode==1 )
		{
		int[] taskDisp= retrieveAgents(ignite);
		//System.err.println("kk"+taskDisp[0]+taskDisp[1]+taskDisp[2]);
                int together=0;
                for(Object obj : ignite.UAVs){
                    UAV other = (UAV) obj;
                    Double2D myLoc = new Double2D(this.x,this.y);
                    if(myLoc.distance(new Double2D(other.x, other.y ))<= 3){    // 3 è la distanza massima di cui si possono distaniare in centro
                        together=together+1;
                    }
                        //System.err.println(together+"this: x  "+this.x+"  y  "+this.y+"  z  "+this.z+"this: x  "+other.x+"  y  "+other.y+"  z  "+other.z);
                }
                if(together==ignite.numUAVs || tgt){
                   
                    DataPacket p= new DataPacket(this,(int) ignite.schedule.getSteps(),5);
                    p.payload.rIDs.add(this.id);
                    sendData(p,ignite);
                    int indexT=0;
                    int valueT=taskDisp[0];
                    taskCloser=ignite.tasks.get(0);

                    for(int i=0; i<taskDisp.length; i++){
                            if (taskDisp[i]<valueT && prevTask!=ignite.tasks.get(i).centroid && !ignite.tasks.get(i).isExausted() && !this.avoidTasks.contains(ignite.tasks.get(i).centroid)){
                                            indexT=i;
                                            valueT=taskDisp[i];
                                            taskCloser=ignite.tasks.get(i);					
                                            }

                            }
                    }
		}

		
		try{
			this.myTask = taskCloser;
			//this.target = new Double3D(taskCloser.centroid.x, taskCloser.centroid.y, z);
			if(taskCloser.centroid.x>=taskCloser.centroid.y){
							if(taskCloser.centroid.y>=60-taskCloser.centroid.x){
											//lato dx
								this.target = new Double3D(taskCloser.centroid.x-((int)this.myTask.radius), taskCloser.centroid.y, z);
										}
							else{
							//lato alto
								this.target = new Double3D(taskCloser.centroid.x, taskCloser.centroid.y+(int)this.myTask.radius, z);
								}
									}

			else {
							if(taskCloser.centroid.y>=60-taskCloser.centroid.x){
											//lato basso
								this.target = new Double3D(taskCloser.centroid.x, taskCloser.centroid.y-((int)this.myTask.radius), z);
													}
							else{
							//lato sx
								this.target = new Double3D(taskCloser.centroid.x+(int)this.myTask.radius, taskCloser.centroid.y, z);
								}
									}

			System.err.println(this.id+ "mode"+ this.mode);
			System.err.println("_______________________");
			if(this.mode!=0){
					this.targetCells2= new LinkedList<>();
					//buildQueue(new Int2D(taskCloser.centroid.x, taskCloser.centroid.y),ignite);
					DataPacket p= new DataPacket(this,(int) ignite.schedule.getSteps(),100);
					p.payload.rIDs.add(this.id);
					sendData(p,ignite);
					
					//System.err.println(this.targetCells2);
					System.err.println("_______________________");
					
					}
		}catch(NullPointerException e){
			//System.err.println("Something is null, have you forgetten to implement some part?");
		}
		//System.err.println("%%%%%%%");
		//DataPacket p= new DataPacket(this,(int) ignite.schedule.getSteps());
		//sendData(p,ignite);
	}

	/**
	 * Take the centroid of the fire and its expected radius and select the next 
	 * cell that requires closer inspection or/and foam. 
	 */
	private void selectCell() {
		//remember to set the new target at the end of the procedure
		boolean done=false;
/*
		WorldCell newTarget=null;
		while(!done){
			try{newTarget = this.targetCells.getFirst();}catch(NoSuchElementException e){done=true;}
			//if(this.mode==1){done=true;}
			if((newTarget.x + newTarget.y)%2==0 && this.mode==1){
									done=true;
									}
			else if((newTarget.x + newTarget.y)%2==1 && this.mode==2){
									done=true;
									}
			try{this.targetCells.removeFirst();}catch(NoSuchElementException e){done=true;}
			}
		
		this.target = new Double3D(newTarget.x, newTarget.y, z);
*/


		Int2D newTarget=null;
		while(!done){
                    	try{
                            newTarget = this.targetCells.getFirst();
                        }
                        catch(NoSuchElementException e){
                            // TODO:  Se il mio task non è esaurito lo rivisita alrimenti mode= 0 quindi selezion un altro task
                            // nella selezione dei task il task scelto non deve essere esaurito
                               // if(this.myTask.isExausted()){System.err.println("task"+this.myTask.centroid+"esaurito");}
                            /*
                            if(this.myTask.centroid.x>=this.myTask.centroid.y){
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato dx
								newTarget = new Int2D(this.myTask.centroid.x-((int)this.myTask.radius+1), this.myTask.centroid.y);
										}
							else{
							//lato alto
								newTarget = new Int2D(this.myTask.centroid.x, this.myTask.centroid.y+(int)this.myTask.radius+1);
								}
									}

                                                        else {
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato basso
								newTarget= new Int2D(this.myTask.centroid.x, this.myTask.centroid.y-((int)this.myTask.radius+1));
													}
							else{
							//lato sx
								newTarget = new Int2D(this.myTask.centroid.x+(int)this.myTask.radius+1, this.myTask.centroid.y);
								}
									} 
                            */
                            done=true;
                        }
                        if(this.mode==1 && myTask==null){done=true;}
                        else if(this.myTask.centroid.x>=this.myTask.centroid.y){
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato dx
                                                                                if(this.mode==1 && newTarget!=null && myTask!=null && newTarget.getY()>=this.myTask.centroid.y){
                                                                                                        done=true;
                                                                                                }

                                                                                else if(this.mode==2 && newTarget!=null && myTask!=null && newTarget.getY()<=this.myTask.centroid.y){
                                                                                        		done=true;
                                                                                               	}
										}
							else{
							//lato alto
                                                                                if(this.mode==1 && newTarget!=null && myTask!=null && newTarget.getX()>=this.myTask.centroid.x){
                                                                                                        done=true;
                                                                                                }

                                                                                else if(this.mode==2 && newTarget!=null && myTask!=null && newTarget.getX()<=this.myTask.centroid.x){
                                                                                        		done=true;
                                                                                               	}
								}
									}

			else {
							if(this.myTask.centroid.y>=60-this.myTask.centroid.x){
											//lato basso

                                                                                if(this.mode==1 && newTarget!=null && myTask!=null && newTarget.getX()>=this.myTask.centroid.x){
                                                                                                        done=true;
                                                                                                }

                                                                                else if(this.mode==2 && newTarget!=null && myTask!=null && newTarget.getX()<=this.myTask.centroid.x){
                                                                                        		done=true;
                                                                                               	}
													}
							else{
							//lato sx
                                                                                if(this.mode==1 && newTarget!=null && myTask!=null && newTarget.getY()>=this.myTask.centroid.y){
                                                                                                        done=true;
                                                                                                }

                                                                                else if(this.mode==2 && newTarget!=null && myTask!=null && newTarget.getY()<=this.myTask.centroid.y){
                                                                                        		done=true;
                                                                                               	}
								}
									}
			
			try{this.targetCells.removeFirst();}catch(NoSuchElementException e){done=true;}
			}


/*
		Int2D newTarget=null;
		while(!done){
			try{newTarget = this.targetCells2.getFirst();}catch(NoSuchElementException e){done=true;}
			//if(this.mode==1){done=true;}
			if(newTarget!=null && (newTarget.getX()<this.myTask.centroid.x)  && this.mode==1 && !this.avoidCells.contains(newTarget)){
									done=true;
									}
			else if(newTarget!=null && (newTarget.getX()>=this.myTask.centroid.x) && this.mode==2 && !this.avoidCells.contains(newTarget)){
									done=true;
									}
			try{this.targetCells2.removeFirst();}catch(NoSuchElementException e){done=true;}
			}
*/
	
		try{
		this.target = new Double3D(newTarget.getX(), newTarget.getY(), z);
		}catch(NullPointerException e){
                            if(this.myTask!=null && this.myTask.isExausted() ){ //|| this.myTask.isExausted()
				this.myTask = null;
				//this.target = null;
				System.err.println("annullare");
				this.avoidTasks= new LinkedList<>();
				//this.targetCells.addLast(new Int2D((int)x,(int)y));
				//this.targetCells2= new LinkedList<>();
				//this.targetCells2.addLast(new Int2D((int)x,(int)y));
				this.mode=0;
				}
			//System.err.println(this.targetCells2+"hh"+this.mode);
		}

		// TODO		
		//the cell selection should be inside myTask area.
		//System.err.println("TODO: implement here your strategy for exploration");

	}
	
	/**
	 * Move the agent toward the target position
	 * The agent moves at a fixed given velocity
	 * @see this.linearvelocity
	 */
	public void move(SimState state){
		Ignite ignite = (Ignite) state;
		
		// retrieve the location of this 
		Double3D location = ignite.air.getObjectLocationAsDouble3D(this);
		double myx = location.x;
		double myy = location.y;
		double myz = location.z;
try{
		// compute the distance w.r.t. the target
		// the z axis is only used when entering or leaving an area
		double xdistance = this.target.x - myx;
		double ydistance = this.target.y - myy;

		if(xdistance < 0)
			myx -= Math.min(Math.abs(xdistance), linearvelocity);
		else
			myx += Math.min(xdistance, linearvelocity);

		if(ydistance < 0){ 
			myy -= Math.min(Math.abs(ydistance), linearvelocity); 
		}
		else{	
			myy += Math.min(ydistance, linearvelocity); 
		}
	}catch(NullPointerException e){}
		// update position in the simulation
		ignite.air.setObjectLocation(this, new Double3D(myx, myy, myz));
		// update position local position
		this.x = myx;
		this.y = myy;
		this.z = myz;
	}

	/**
	 * Start to extinguish the fire at current location.
	 * @return true if enough time has passed and the fire is gone, false otherwise
	 * @see this.stepToExtinguish
	 * @see this.startedToExtinguishAt
	 */
	private boolean extinguish(Ignite ignite){
		if(startedToExtinguishAt==-1){
			this.startedToExtinguishAt = (int) ignite.schedule.getSteps();
		}
		//enough time has passed, the fire is gone
		if(ignite.schedule.getSteps() - startedToExtinguishAt == stepToExtinguish){
			startedToExtinguishAt = -1;
			return true;
		}		
		return false;
	}

	/**
	 * COMMUNICATION
	 * Check if the input location is within communication range
	 */
	public boolean isInCommunicationRange(Double3D otherLoc){
		Double3D myLoc = new Double3D(x,y,z);
		return myLoc.distance(otherLoc) <= UAV.communicationRange;
	}

	/**
	 * COMMUNICATION
	 * Send a message to the team
	 */
	public void sendData(DataPacket packet,Ignite ignite){
		//TODO
		for(Object obj : ignite.UAVs){ 
			UAV other = (UAV) obj;
			if(isInCommunicationRange(new Double3D(other.x, other.y, other.z)))
				other.receiveData(packet,ignite);
			}
		
	}

	/**
	 * COMMUNICATION
	 * Receive a message from the team
	 */
	public void receiveData(DataPacket packet,Ignite ignite){
	try{
		//TODO
               int r=packet.header.r;
               int otherId= packet.header.id;
               
               if(r==5 && otherId!=this.id && !packet.payload.rIDs.contains(this.id)){
				packet.payload.rIDs.add(this.id);
                                System.err.println("è vicino l'uav:"+this.id);
                                tgt=true;
				sendData(packet,ignite);
				}
		
                Double3D otherLoc = new Double3D(packet.payload.x,packet.payload.y,packet.payload.z);

		Int2D taskPos= packet.payload.taskPos;
		Int2D prevPos= packet.payload.prevPos;
		int modeOth= packet.payload.mode;
		int t= packet.header.timestamp;
		//System.err.println(t+"___"+ignite.schedule.getSteps());

		Set<WorldCell> kCells= packet.payload.knownCells;
		Object[] kC=kCells.toArray();
		//System.err.println(otherId+"___"+taskPos+"___out"+this.id);

		//messaggio che comunica il preTask ma non molto efficente..
		if(r==20 && otherId!=this.id && !packet.payload.rIDs.contains(this.id) && packet.payload.prevPos!=null){
				this.prevTask=prevPos;
				packet.payload.rIDs.add(this.id);
				sendData(packet,ignite);		
				}
                
                if(r==10 && otherId!=this.id && !packet.payload.rIDs.contains(this.id) && taskPos!=null && this.myTask!=null){
				packet.payload.rIDs.add(this.id);
				System.err.println("qq"+this.id+ " mode "+ this.mode+ " target "+this.myTask.centroid);
				if(taskPos==this.myTask.centroid && modeOth == this.mode){
							if(this.mode==1){this.mode++;}
							else if(this.mode==2){this.mode--;}
						System.err.println("qq"+this.id+ " mode "+ this.mode+ " target "+this.myTask.centroid);
							}
				else if(taskPos==this.myTask.centroid && modeOth <= this.mode){
							this.mode++;
							if(this.mode>2){this.mode=0;}
						System.err.println("qq"+this.id+ " mode "+ this.mode+ " target "+this.myTask.centroid);
							}
    
				packet.payload.rIDs.add(this.id);
				sendData(packet,ignite);

				for(int i=0; i< kC.length ; i++){
							if(!this.knownCells.contains((WorldCell)kC[i])){
											this.knownCells.add((WorldCell)kC[i]);
											}
							}		
				}

		
		if(r==100 && otherId!=this.id && !packet.payload.rIDs.contains(this.id)){
				//System.err.println(t+"___"+ignite.schedule.getSteps());
				//System.err.println("from "+otherId+" to "+this.id+" of mode: "+this.mode+"__"+packet.payload.rIDs);
				//System.err.println(this.id+"mode:"+this.mode);
                    
				packet.payload.rIDs.add(this.id);
                                System.err.println("from "+otherId+" of mode "+modeOth+" to "+this.id+" of mode: "+this.mode+"task"+this.myTask.centroid+"__"+packet.payload.rIDs);
				sendData(packet,ignite);
                                if(modeOth==2){
                                                this.avoidTasks.add(taskPos);
                                                }
				if(taskPos==this.myTask.centroid && modeOth <= this.mode){
							this.mode++;
							//System.err.println(this.id+"mode:"+this.mode);
							if(this.mode>(ignite.numUAVs/3)){
								this.mode=0;
								this.myTask = null;
								this.target = null;
								this.prevTask=this.myTask.centroid;
								//DataPacket p20= new DataPacket(this,(int) ignite.schedule.getSteps(),20);
								//sendData(p20,ignite);
									}
							DataPacket p= new DataPacket(this,(int) ignite.schedule.getSteps(),100);
							p.payload.rIDs.add(this.id);
							sendData(p,ignite);
								}
                                else if(taskPos!=this.myTask.centroid && modeOth==2){
                                                        this.avoidTasks.add(taskPos);
                                                        }
				//sendData(packet,ignite);

				for(int i=0; i< kC.length ; i++){
							if(!this.knownCells.contains((WorldCell)kC[i])){
											this.knownCells.add((WorldCell)kC[i]);
											}
							}
				}

	}catch(NullPointerException e){//System.err.println("--"+this.id+ " mode "+ this.mode+ "task null");
                                        int r=packet.header.r;
					int otherId= packet.header.id;
                                        int modeOth= packet.payload.mode;
                                        Int2D taskPos= packet.payload.taskPos;
                                        if(r==100){System.err.println("from "+otherId+" of mode "+modeOth+" to "+this.id+" of mode: "+this.mode+"task null__"+packet.payload.rIDs);}
					if(otherId!=this.id && !packet.payload.rIDs.contains(this.id)){
                                                                        if(modeOth==2){
                                                                                        this.avoidTasks.add(taskPos);
                                                                                        }                                                
									packet.payload.rIDs.add(this.id);
                                                                        //System.err.println("from "+otherId+" of mode "+modeOth+" to "+this.id+" of mode: "+this.mode+"task null__"+packet.payload.rIDs);
									sendData(packet,ignite);
									//System.err.println("rispedito");
									}
					}		
		//hint for a possible flooding strategy: 
		//if: (neverReceived(packet) && packet.origin != this) -> sendData(packet)
	}

	/**
	 * COMMUNICATION
	 * Retrieve the status of all the agents in the communication range.
	 * @return an array of size Ignite.tasks().size+1 where at position i you have 
	 * the number of agents enrolled in task i (i.e. Ignite.tasks().get(i)). 
	 * 
	 * HINT: you can easily assume that the number of uncommitted agents is equal to:
	 * Ignite.numUAVs - sum of all i in the returned array
	 */
	public int[] retrieveAgents(Ignite ignite){
		int[] status = new int[ignite.tasks.size()];
		//System.err.println(status.length);
		for(Object obj : ignite.UAVs){ //count also this uav
			UAV other = (UAV) obj;

			if(isInCommunicationRange(new Double3D(other.x, other.y, other.z))){
				Task task = other.myTask;
				if(task != null && task.centroid.x!=30 && task.centroid.y!=30)
					{
					//System.err.println(ignite.tasks.indexOf(task));
					status[ignite.tasks.indexOf(task)]++;
					}
			}
		}
		
		return status;
	}
	

	public void buildQueue(Int2D cell,Ignite ignite){
		this.targetCells2.addLast(cell);
		nearCellAdd(cell,ignite);
		Int2D mom= null;
		
		while(this.targetCells2.size()<1000){
			mom=this.targetCells.getFirst();
			this.targetCells.removeFirst();
			this.targetCells2.addLast(mom);
			nearCellAdd(mom,ignite);
			}
		
	}



	public void nearCellAdd(Int2D c,Ignite ignite){
	int x=c.getX();
	int y=c.getY();
	WorldCell cell=(WorldCell)ignite.forest.field[(int) x][(int) y];
			double j=x+1;
			double g=x-1;
			double h=y+1;
			double i=y-1;


			// aggiungo all insieme da visitare l'elemento alla destra di quello visitato solo se però la cella in cui
			// mi trovo e quindi quello che è la visuale,non è acqua
			try{			
				WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
				Int2D dxi= new Int2D((int)j,(int)y);
				if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(dxi);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento alla destra in basso di quello visitato
			try{			
				WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
				Int2D dxdwni= new Int2D((int)j,(int)i);
				if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(dxdwni);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento in basso a quello visitato
			try{
				WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
				Int2D dwni= new Int2D((int)x,(int)i);
				if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(dwni);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento alla sinistra in basso di quello visitato
			try{			
				WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
				Int2D sxdwni= new Int2D((int)g,(int)i);
				if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(sxdwni);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento alla sinistra di quello visitato
			try{
				WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
				Int2D sxi= new Int2D((int)g,(int)y);
				if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(sxi);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento alla simistra in alto di quello visitato
			try{			
				WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
				Int2D sxupi= new Int2D((int)g,(int)h);
				if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(sxupi);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento in alto di quello visitato
			try{
				WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
				Int2D upi= new Int2D((int)x,(int)h);
				if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(upi);
						}
				}catch(ArrayIndexOutOfBoundsException e){}
			// aggiungo all insieme da visitare l'elemento alla destra in alto di quello visitato
			try{			
				WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
				Int2D dxupi= new Int2D((int)j,(int)h);
				if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) && !cell.type.equals(CellType.WATER)) {
					this.targetCells.addLast(dxupi);
						}
				}catch(ArrayIndexOutOfBoundsException e){}	

	}
	

        
       
	public void nearCellAddDX(Int2D c,Ignite ignite){
	int x=c.getX();
	int y=c.getY();
	WorldCell cell=(WorldCell)ignite.forest.field[(int) x][(int) y];
			double j=x+1;
			double g=x-1;
			double h=y-1;
			double i=y+1;
                                         
                        if((int)cell.y==(int) this.myTask.centroid.y && cell.x<= this.myTask.centroid.x ){
                                                                try{			
                                                                    WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                    Int2D dxi= new Int2D((int)j,(int)y);
                                                                    if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) && dxi.distance(this.myTask.centroid)>this.myTask.radius-1) {
                                                                            this.targetCells.addLast(dxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                             
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) ) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}  
                                                                
                                                                try{
                                                                    WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                                                    Int2D upi= new Int2D((int)x,(int)h);
                                                                    if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) ) {
                                                                            this.targetCells.addLast(upi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                
                                                                try{
                                                                    WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                                                    Int2D dwni= new Int2D((int)x,(int)i);
                                                                    if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) ) {
                                                                            this.targetCells.addLast(dwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                try{			
                                                                    WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                                    Int2D sxupi= new Int2D((int)g,(int)h);
                                                                    if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                                            this.targetCells.addLast(sxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                
                                                 		try{			
                                                                    WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                                    Int2D sxdwni= new Int2D((int)g,(int)i);
                                                                    if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                                            this.targetCells.addLast(sxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                
                                                                try{
                                                                    WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                                                    Int2D sxi= new Int2D((int)g,(int)y);
                                                                    if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) && sxi.distance(this.myTask.centroid)<this.myTask.radius+1 ) {
                                                                            this.targetCells.addLast(sxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                   
                            }
                        else if (cell.type.equals(CellType.WATER)){
                                if(cell.y< this.myTask.centroid.y){
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                     
                                                        }
                                else if(cell.y> this.myTask.centroid.y){
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                    
                                                        }
                            }
                        else if (cell.type.equals(CellType.NORMAL) || cell.type.equals(CellType.EXTINGUISHED)){
                                if(cell.x<= this.myTask.centroid.x){
                                                                try{			
                                                                    WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                    Int2D dxi= new Int2D((int)j,(int)y);
                                                                    if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(dxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                    
                                }
                                if(cell.x>= this.myTask.centroid.x){
                                    if(cell.y> this.myTask.centroid.y){
                                                                try{
                                                                    WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                                                    Int2D upi= new Int2D((int)x,(int)h);
                                                                    if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(upi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                     
                                    }
                                    if(cell.y< this.myTask.centroid.y){
                                                                try{
                                                                    WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                                                    Int2D dwni= new Int2D((int)x,(int)i);
                                                                    if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(dwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                    
                                    }                                    
                                }                                
                            }
                        else if (cell.type.equals(CellType.FIRE)){
                                if(cell.y> this.myTask.centroid.y){ 
                                    if(cell.x<= this.myTask.centroid.x+1){
                                                                try{			
                                                                    WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                                    Int2D sxupi= new Int2D((int)g,(int)h);
                                                                    if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                                            this.targetCells.addLast(sxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                        
                                                                try{
                                                                    WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                                                    Int2D sxi= new Int2D((int)g,(int)y);
                                                                    if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) ) {
                                                                            this.targetCells.addLast(sxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                    
                                                 		try{			
                                                                    WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                                    Int2D sxdwni= new Int2D((int)g,(int)i);
                                                                    if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                                            this.targetCells.addLast(sxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                                                                     
                                                                try{
                                                                    WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                                                    Int2D dwni= new Int2D((int)x,(int)i);
                                                                    if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni)) {
                                                                            this.targetCells.addLast(dwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                 
                                                                                              
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) && dxdwni.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                    }
                                    if(cell.x> this.myTask.centroid.x){
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                        
                                                                try{			
                                                                    WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                    Int2D dxi= new Int2D((int)j,(int)y);
                                                                    if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                                            this.targetCells.addLast(dxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                
                                    }
                                }
                                if(cell.y< this.myTask.centroid.y){
                                    if(cell.x<= this.myTask.centroid.x+1){
                                                 		try{			
                                                                    WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                                    Int2D sxdwni= new Int2D((int)g,(int)i);
                                                                    if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                                            this.targetCells.addLast(sxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                        
                                                                try{
                                                                    WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                                                    Int2D sxi= new Int2D((int)g,(int)y);
                                                                    if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) ) {
                                                                            this.targetCells.addLast(sxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                    
                                                                try{			
                                                                    WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                                    Int2D sxupi= new Int2D((int)g,(int)h);
                                                                    if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                                            this.targetCells.addLast(sxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                                                             
                                                                try{
                                                                    WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                                                    Int2D upi= new Int2D((int)x,(int)h);
                                                                    if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) ) {
                                                                            this.targetCells.addLast(upi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                                                                      
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) && dxupi.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                                  
                                                        }
                                    if(cell.x> this.myTask.centroid.x){
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                try{			
                                                                    WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                    Int2D dxi= new Int2D((int)j,(int)y);
                                                                    if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                                            this.targetCells.addLast(dxi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}  
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) ) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                
                                    }
                                }
                            }
	

	}
        
	public void nearCellAddSX(Int2D c,Ignite ignite){
	int x=c.getX();
	int y=c.getY();
	WorldCell cell=(WorldCell)ignite.forest.field[(int) x][(int) y];
			double j=x+1;
			double g=x-1;
			double h=y-1;
			double i=y+1;
                                         
                        if((int)cell.y== (int)this.myTask.centroid.y && cell.x>= this.myTask.centroid.x){
                                
                                try{			
                                        WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                        Int2D sxupi= new Int2D((int)g,(int)h);
                                        if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                this.targetCells.addLast(sxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}

                                try{			
                                        WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                        Int2D sxdwni= new Int2D((int)g,(int)i);
                                        if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                this.targetCells.addLast(sxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{
                                        WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                        Int2D upi= new Int2D((int)x,(int)h);
                                        if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) ) {
                                                this.targetCells.addLast(upi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                

                                try{
                                        WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                        Int2D dwni= new Int2D((int)x,(int)i);
                                        if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) ) {
                                                this.targetCells.addLast(dwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                 
                                try{			
                                        WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                        Int2D dxupi= new Int2D((int)j,(int)h);
                                        if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                this.targetCells.addLast(dxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}    
                                
                                try{			
                                        WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                        Int2D dxdwni= new Int2D((int)j,(int)i);
                                        if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                this.targetCells.addLast(dxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                                try{			
                                        WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                        Int2D dxi= new Int2D((int)j,(int)y);
                                        if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) && dxi.distance(this.myTask.centroid)<this.myTask.radius+1 ) {
                                                this.targetCells.addLast(dxi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                        }
                        else if (cell.type.equals(CellType.WATER)){
                                if((int)cell.y> (int)this.myTask.centroid.y){
                                        try{			
                                                WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                Int2D sxdwni= new Int2D((int)g,(int)i);
                                                if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni)) {
                                                        this.targetCells.addLast(sxdwni);
                                                                }
                                                }catch(ArrayIndexOutOfBoundsException e){}                                  
                                }
                                else if((int)cell.y< (int)this.myTask.centroid.y){
                                        try{			
                                                WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                Int2D sxupi= new Int2D((int)g,(int)h);
                                                if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi)) {
                                                        this.targetCells.addLast(sxupi);
                                                                }
                                                }catch(ArrayIndexOutOfBoundsException e){}                                
                                }
                        }
                        else if (cell.type.equals(CellType.NORMAL) || cell.type.equals(CellType.EXTINGUISHED)){
                                if((int)cell.x>= (int)this.myTask.centroid.x){
                                        try{
                                                WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                                Int2D sxi= new Int2D((int)g,(int)y);
                                                if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) && !cell.type.equals(CellType.WATER)) {
                                                        this.targetCells.addLast(sxi);
                                                                }
                                                }catch(ArrayIndexOutOfBoundsException e){}                                
                                }
                                if((int)cell.x<= (int)this.myTask.centroid.x){
                                    if((int)cell.y> (int)this.myTask.centroid.y){
                                        try{
                                                WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                                Int2D upi= new Int2D((int)x,(int)h);
                                                if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
                                                        this.targetCells.addLast(upi);
                                                                }
                                                }catch(ArrayIndexOutOfBoundsException e){}                                    
                                    }
                                    if((int)cell.y< (int)this.myTask.centroid.y){
                                        try{
                                                WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                                Int2D dwni= new Int2D((int)x,(int)i);
                                                if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && !cell.type.equals(CellType.WATER)) {
                                                        this.targetCells.addLast(dwni);
                                                                }
                                                }catch(ArrayIndexOutOfBoundsException e){}                                    
                                    }
                                }
                        }
                        else if (cell.type.equals(CellType.FIRE)){
                            if((int)cell.x>= (int)this.myTask.centroid.x+1){
                                if((int)cell.y> (int)this.myTask.centroid.y){
                                                 		try{			
                                                                    WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                                    Int2D sxdwni= new Int2D((int)g,(int)i);
                                                                    if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) && sxdwni.distance(this.myTask.centroid)>=this.myTask.radius-2) {
                                                                            this.targetCells.addLast(sxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                                                                     
                                                                try{
                                                                    WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                                                    Int2D dwni= new Int2D((int)x,(int)i);
                                                                    if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(dwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                 
                                                                                              
                                                                try{			
                                                                    WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                    Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                    if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(dxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}    
                                                                try{			
                                                                        WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                        Int2D dxi= new Int2D((int)j,(int)y);
                                                                        if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                                                this.targetCells.addLast(dxi);
                                                                                        }
                                                                        }catch(ArrayIndexOutOfBoundsException e){} 
                                                                try{			
                                                                        WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                        Int2D dxupi= new Int2D((int)j,(int)h);
                                                                        if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                                                this.targetCells.addLast(dxupi);
                                                                                        }
                                                                        }catch(ArrayIndexOutOfBoundsException e){}                                                                 
                                }
                                if((int)cell.y< (int)this.myTask.centroid.y){                                                                
                                                                try{			
                                                                    WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                                    Int2D sxupi= new Int2D((int)g,(int)h);
                                                                    if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) && sxupi.distance(this.myTask.centroid)>=this.myTask.radius-2) {
                                                                            this.targetCells.addLast(sxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                                                             
                                                                try{
                                                                    WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                                                    Int2D upi= new Int2D((int)x,(int)h);
                                                                    if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(upi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                                                                      
                                                                try{			
                                                                    WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                                                    Int2D dxupi= new Int2D((int)j,(int)h);
                                                                    if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) && !cell.type.equals(CellType.WATER)) {
                                                                            this.targetCells.addLast(dxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){} 
                                                                try{			
                                                                        WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                                                        Int2D dxi= new Int2D((int)j,(int)y);
                                                                        if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi)  ) {
                                                                                this.targetCells.addLast(dxi);
                                                                                        }
                                                                        }catch(ArrayIndexOutOfBoundsException e){}                                                                 
                                                                try{			
                                                                        WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                                                        Int2D dxdwni= new Int2D((int)j,(int)i);
                                                                        if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) ) {
                                                                                this.targetCells.addLast(dxdwni);
                                                                                        }
                                                                        }catch(ArrayIndexOutOfBoundsException e){}                                                                 
                                }
                            }
                            if((int)cell.x< (int)this.myTask.centroid.x){
                                                                try{			
                                                                    WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                                                    Int2D sxupi= new Int2D((int)g,(int)h);
                                                                    if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) && sxupi.distance(this.myTask.centroid)>=this.myTask.radius-2) {
                                                                            this.targetCells.addLast(sxupi);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}
                                                                try{
                                                                        WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                                                        Int2D sxi= new Int2D((int)g,(int)y);
                                                                        if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) && !cell.type.equals(CellType.WATER)) {
                                                                                this.targetCells.addLast(sxi);
                                                                                        }
                                                                        }catch(ArrayIndexOutOfBoundsException e){}                                                                
                                                 		try{			
                                                                    WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                                                    Int2D sxdwni= new Int2D((int)g,(int)i);
                                                                    if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) && sxdwni.distance(this.myTask.centroid)>=this.myTask.radius-2) {
                                                                            this.targetCells.addLast(sxdwni);
                                                                                    }
                                                                    }catch(ArrayIndexOutOfBoundsException e){}                                                                
                            }
                        }
	

	}
	        
        public void nearCellAddUP(Int2D c,Ignite ignite){
	int x=c.getX();
	int y=c.getY();
	WorldCell cell=(WorldCell)ignite.forest.field[(int) x][(int) y];
			double j=x+1;
			double g=x-1;
			double h=y-1;
			double i=y+1;
                                         
                        if((int)cell.x== (int)this.myTask.centroid.x && cell.y>= this.myTask.centroid.y){
                                try{			
                                        WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                        Int2D sxupi= new Int2D((int)g,(int)h);
                                        if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi)) {
                                                this.targetCells.addLast(sxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{			
                                        WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                        Int2D dxupi= new Int2D((int)j,(int)h);
                                        if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                this.targetCells.addLast(dxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{
                                        WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                        Int2D sxi= new Int2D((int)g,(int)y);
                                        if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi)) {
                                                this.targetCells.addLast(sxi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){} 
                                try{			
                                        WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                        Int2D dxi= new Int2D((int)j,(int)y);
                                        if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi)) {
                                                this.targetCells.addLast(dxi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}  
                                try{			
                                        WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                        Int2D sxdwni= new Int2D((int)g,(int)i);
                                        if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni)) {
                                                this.targetCells.addLast(sxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{			
                                        WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                        Int2D dxdwni= new Int2D((int)j,(int)i);
                                        if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                this.targetCells.addLast(dxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                                try{
                                        WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                        Int2D dwni= new Int2D((int)x,(int)i);
                                        if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && dwni.distance(this.myTask.centroid)<this.myTask.radius+1) {
                                                this.targetCells.addLast(dwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}  

                        }
                        
                        if (cell.type.equals(CellType.WATER)){
                            if(cell.x> this.myTask.centroid.x){
                                try{			
                                        WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                        Int2D dxupi= new Int2D((int)j,(int)h);
                                        if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                this.targetCells.addLast(dxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                            }
                            else if(cell.x< this.myTask.centroid.x){
                                try{			
                                        WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                        Int2D sxupi= new Int2D((int)g,(int)h);
                                        if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi)) {
                                                this.targetCells.addLast(sxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                            
                            }
                        }
                        else if (cell.type.equals(CellType.NORMAL) || cell.type.equals(CellType.EXTINGUISHED)){
                            if(cell.y>= this.myTask.centroid.y){
                                try{
                                        WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                        Int2D upi= new Int2D((int)x,(int)h);
                                        if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
                                                this.targetCells.addLast(upi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                            }
                            if(cell.y<= this.myTask.centroid.y){
                                if(cell.x> this.myTask.centroid.x){
                                    try{
                                            WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                            Int2D sxi= new Int2D((int)g,(int)y);
                                            if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi)) {
                                                    this.targetCells.addLast(sxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                }
                                if(cell.x< this.myTask.centroid.x){
                                    try{			
                                            WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                            Int2D dxi= new Int2D((int)j,(int)y);
                                            if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi)) {
                                                    this.targetCells.addLast(dxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                }
                            }
                        }
                        else if (cell.type.equals(CellType.FIRE)){
                            if(cell.y>= this.myTask.centroid.y+1){
                                if(cell.x> this.myTask.centroid.x){
                                    try{			
                                            WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                            Int2D sxdwni= new Int2D((int)g,(int)i);
                                            if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni)) {
                                                    this.targetCells.addLast(sxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}    
                                    try{
                                            WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                            Int2D dwni= new Int2D((int)x,(int)i);
                                            if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) ) {
                                                    this.targetCells.addLast(dwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                  
                                    try{			
                                            WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                            Int2D dxdwni= new Int2D((int)j,(int)i);
                                            if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) ) {
                                                    this.targetCells.addLast(dxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){} 
                                    try{			
                                            WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                            Int2D dxi= new Int2D((int)j,(int)y);
                                            if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                    this.targetCells.addLast(dxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}
                                    try{			
                                            WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                            Int2D dxupi= new Int2D((int)j,(int)h);
                                            if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) && dxupi.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                    this.targetCells.addLast(dxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                            
                                }
                                else if(cell.x< this.myTask.centroid.x){  
                                    try{			
                                            WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                            Int2D dxdwni= new Int2D((int)j,(int)i);
                                            if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) ) {
                                                    this.targetCells.addLast(dxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                 
                                    try{
                                            WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                            Int2D dwni= new Int2D((int)x,(int)i);
                                            if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) ) {
                                                    this.targetCells.addLast(dwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                  
                                    try{			
                                            WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                            Int2D sxdwni= new Int2D((int)g,(int)i);
                                            if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                    this.targetCells.addLast(sxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}
                                    try{
                                            WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                            Int2D sxi= new Int2D((int)g,(int)y);
                                            if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi)) {
                                                    this.targetCells.addLast(sxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){} 

                                    try{			
                                            WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                            Int2D sxupi= new Int2D((int)g,(int)h);
                                            if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) && sxupi.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                    this.targetCells.addLast(sxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                           
                                }
                            }
                            else if(cell.y< this.myTask.centroid.y){
                                    try{			
                                            WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                            Int2D sxupi= new Int2D((int)g,(int)h);
                                            if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi)) {
                                                    this.targetCells.addLast(sxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}  
                                    try{
                                            WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                            Int2D upi= new Int2D((int)x,(int)h);
                                            if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && !cell.type.equals(CellType.WATER)) {
                                                    this.targetCells.addLast(upi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}  
                                    try{			
                                            WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                            Int2D dxupi= new Int2D((int)j,(int)h);
                                            if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi)) {
                                                    this.targetCells.addLast(dxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                 
                            }
                        }
	

	}
        
        public void nearCellAddDWN(Int2D c,Ignite ignite){
	int x=c.getX();
	int y=c.getY();
	WorldCell cell=(WorldCell)ignite.forest.field[(int) x][(int) y];
			double j=x+1;
			double g=x-1;
			double h=y-1;
			double i=y+1;
                                         
                        if((int)cell.x== (int)this.myTask.centroid.x && cell.y<= this.myTask.centroid.y){
                                try{			
                                        WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                        Int2D dxdwni= new Int2D((int)j,(int)i);
                                        if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                this.targetCells.addLast(dxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{			
                                        WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                        Int2D sxdwni= new Int2D((int)g,(int)i);
                                        if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                this.targetCells.addLast(sxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{			
                                        WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                        Int2D dxi= new Int2D((int)j,(int)y);
                                        if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                this.targetCells.addLast(dxi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){} 
                                try{
                                        WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                        Int2D sxi= new Int2D((int)g,(int)y);
                                        if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) ) {
                                                this.targetCells.addLast(sxi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}   
                                try{			
                                        WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                        Int2D dxupi= new Int2D((int)j,(int)h);
                                        if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                this.targetCells.addLast(dxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}
                                try{			
                                        WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                        Int2D sxupi= new Int2D((int)g,(int)h);
                                        if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                this.targetCells.addLast(sxupi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                                try{
                                        WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                        Int2D upi= new Int2D((int)x,(int)h);
                                        if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) && upi.distance(this.myTask.centroid)<this.myTask.radius+1 ) {
                                                this.targetCells.addLast(upi);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}

                        }
                        else if (cell.type.equals(CellType.WATER)){
                            if(cell.x> this.myTask.centroid.x){
                                try{			
                                        WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                        Int2D dxdwni= new Int2D((int)j,(int)i);
                                        if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                this.targetCells.addLast(dxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                                
                            }    
                            else if(cell.x< this.myTask.centroid.x){
                                try{			
                                        WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                        Int2D sxdwni= new Int2D((int)g,(int)i);
                                        if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni)) {
                                                this.targetCells.addLast(sxdwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                            
                            }
                        }
                        else if (cell.type.equals(CellType.NORMAL) || cell.type.equals(CellType.EXTINGUISHED)){
                            if(cell.y<= this.myTask.centroid.y){
                                try{
                                        WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                        Int2D dwni= new Int2D((int)x,(int)i);
                                        if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) && !cell.type.equals(CellType.WATER)) {
                                                this.targetCells.addLast(dwni);
                                                        }
                                        }catch(ArrayIndexOutOfBoundsException e){}                            
                            }
                            if(cell.y>= this.myTask.centroid.y){
                                if(cell.x> this.myTask.centroid.x){
                                    try{
                                            WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                            Int2D sxi= new Int2D((int)g,(int)y);
                                            if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) && !cell.type.equals(CellType.WATER)) {
                                                    this.targetCells.addLast(sxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                               
                                }
                                if(cell.x< this.myTask.centroid.x){
                                    try{			
                                            WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                            Int2D dxi= new Int2D((int)j,(int)y);
                                            if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) && !cell.type.equals(CellType.WATER)) {
                                                    this.targetCells.addLast(dxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                    
                                }
                            }
                        }
                        else if (cell.type.equals(CellType.FIRE)){
                            if(cell.y<= this.myTask.centroid.y+1){
                                if(cell.x> this.myTask.centroid.x){
                                    try{			
                                            WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                            Int2D sxupi= new Int2D((int)g,(int)h);
                                            if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                    this.targetCells.addLast(sxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                    try{
                                            WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                            Int2D upi= new Int2D((int)x,(int)h);
                                            if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) ) {
                                                    this.targetCells.addLast(upi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                    try{			
                                            WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                            Int2D dxupi= new Int2D((int)j,(int)h);
                                            if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                    this.targetCells.addLast(dxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}
                                    try{			
                                            WorldCell dx=(WorldCell)ignite.forest.field[(int) j][(int) y];
                                            Int2D dxi= new Int2D((int)j,(int)y);
                                            if(!this.knownCells.contains(dx) && !this.targetCells.contains(dxi) ) {
                                                    this.targetCells.addLast(dxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){} 
                                    try{			
                                            WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                            Int2D dxdwni= new Int2D((int)j,(int)i);
                                            if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni) && dxdwni.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                    this.targetCells.addLast(dxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                            
                                }
                                else if(cell.x< this.myTask.centroid.x){
                                    try{			
                                            WorldCell dxup=(WorldCell)ignite.forest.field[(int) j][(int) h];
                                            Int2D dxupi= new Int2D((int)j,(int)h);
                                            if(!this.knownCells.contains(dxup) && !this.targetCells.contains(dxupi) ) {
                                                    this.targetCells.addLast(dxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                    try{
                                            WorldCell up=(WorldCell)ignite.forest.field[(int) x][(int) h];
                                            Int2D upi= new Int2D((int)x,(int)h);
                                            if(!this.knownCells.contains(up) && !this.targetCells.contains(upi) ) {
                                                    this.targetCells.addLast(upi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                
                                    try{			
                                            WorldCell sxup=(WorldCell)ignite.forest.field[(int) g][(int) h];
                                            Int2D sxupi= new Int2D((int)g,(int)h);
                                            if(!this.knownCells.contains(sxup) && !this.targetCells.contains(sxupi) ) {
                                                    this.targetCells.addLast(sxupi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}
                                    try{
                                            WorldCell sx=(WorldCell)ignite.forest.field[(int) g][(int) y];
                                            Int2D sxi= new Int2D((int)g,(int)y);
                                            if(!this.knownCells.contains(sx) && !this.targetCells.contains(sxi) ) {
                                                    this.targetCells.addLast(sxi);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}
                                    try{			
                                            WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                            Int2D sxdwni= new Int2D((int)g,(int)i);
                                            if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) && sxdwni.distance(this.myTask.centroid)>=this.myTask.radius-this.dep) {
                                                    this.targetCells.addLast(sxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                            
                                }
                            }
                            if(cell.y> this.myTask.centroid.y){
                                    try{			
                                            WorldCell sxdwn=(WorldCell)ignite.forest.field[(int) g][(int) i];
                                            Int2D sxdwni= new Int2D((int)g,(int)i);
                                            if(!this.knownCells.contains(sxdwn) && !this.targetCells.contains(sxdwni) ) {
                                                    this.targetCells.addLast(sxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}   
                                    try{
                                            WorldCell dwn=(WorldCell)ignite.forest.field[(int) x][(int) i];
                                            Int2D dwni= new Int2D((int)x,(int)i);
                                            if(!this.knownCells.contains(dwn) && !this.targetCells.contains(dwni) ) {
                                                    this.targetCells.addLast(dwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}  
                                    try{			
                                            WorldCell dxdwn=(WorldCell)ignite.forest.field[(int) j][(int) i];
                                            Int2D dxdwni= new Int2D((int)j,(int)i);
                                            if(!this.knownCells.contains(dxdwn) && !this.targetCells.contains(dxdwni)) {
                                                    this.targetCells.addLast(dxdwni);
                                                            }
                                            }catch(ArrayIndexOutOfBoundsException e){}                                    
                            }
                        }
	

	}

	@Override
	public boolean equals(Object obj){
		UAV uav = (UAV) obj;
		return uav.id == this.id;
	}
	
	@Override
	public String toString(){ 
		return id+"UAV-"+x+","+y+","+z+"-"+action;
	} 	
}



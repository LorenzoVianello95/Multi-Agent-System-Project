package sim.app.firecontrol;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import sim.engine.SimState;
import sim.engine.Steppable;

public class FireController implements Steppable{
	private static final long serialVersionUID = 1L;
	
	/**
	 * This will check for termination conditions and writes out a file on mason root directory.
	 * TODO: fill the file with information about your simulation according to what you would like to show
	 * in your report
	 */
	@Override
	public void step(SimState state) {
		//create a .txt file where we can store simulation informations
		if(Ignite.cellsOnFire == 0){
			Ignite ign = (Ignite) state;		
			for(Task t: ign.tasks){
			if(t.isExausted()){System.out.println("taskesaurito");}				
			}

			//String fileName = System.getProperty("user.dir") + "/" + System.currentTimeMillis() + ".txt";
			String fileName = System.getProperty("user.dir") + "/" + "ProjectMAS" + ".txt";

			try {
				FileWriter fw = new FileWriter(new File(fileName),true);
				BufferedWriter bwr = new BufferedWriter(fw);		
				//bwr.append("Cells on water:"+Ignite.cellsOnWater+"\n");
				bwr.append("Cells Estinguesh:"+Ignite.cellsEstinguesh +"\n");
				bwr.append("Cells Burned:"+Ignite.cellsBurned +"\n");
				bwr.append("Total number of cells:"+"3600\n");
				bwr.append("______________________________\n");
				bwr.flush();
				bwr.close();
			} catch (IOException e) {
				System.err.println("Exception in FireControll.step() " + e.toString());
				e.printStackTrace();
			}
			
			//kill the current job of the simulation
			state.kill();
		}
	}

}

import java.io.*;
import java.util.*;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

class node implements Comparable<node>{
		public double x;
		public double y;
		public double g,h;
		public ArrayList<node> neighbour_list;
		public boolean visited;
		public boolean isGoal = false, isStart;	
		public node father;
		public int compareTo(node other_node){
			return Double.compare(this.g + this.h,other_node.g + other_node.h);
		}
		public double distFrom(double lat2, double lng2) {
			double lat1 = this.y;
			double lng1 = this.x;
			double earthRadius = 6371000; //meters
			double dLat = Math.toRadians(lat2-lat1);
			double dLng = Math.toRadians(lng2-lng1);
			double a = Math.sin(dLat/2) * Math.sin(dLat/2) +
					   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
					   Math.sin(dLng/2) * Math.sin(dLng/2);
			double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
			double dist = (earthRadius * c);

			return dist;
		}
		public void init() {
			this.g = Double.POSITIVE_INFINITY;
			this.visited = false;
			this.father = null;
			this.isStart = false;
		}
		public void heuristic(node goal){
			this.h = this.distFrom(goal.y,goal.x);
		}
		
	}

class coords{	// this is used only for path
	Double x=null,y=null;
	Integer id = null;
	public double distFrom(double lat2, double lng2) {
			double lat1 = this.y;
			double lng1 = this.x;
			double earthRadius = 6371000; //meters
			double dLat = Math.toRadians(lat2-lat1);
			double dLng = Math.toRadians(lng2-lng1);
			double a = Math.sin(dLat/2) * Math.sin(dLat/2) +
					   Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2)) *
					   Math.sin(dLng/2) * Math.sin(dLng/2);
			double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
			double dist = (earthRadius * c);

			return dist;
		}
}

class string_pair{
	String a=null,b=null;
	public string_pair(String in1,String in2){
		a = in1;
		b = in2;
	}
}
	
public class taxi{
	
	public static node closest(double x, double y,HashMap<String,node> map){
		node min=null, point;
		double min_dist=Double.MAX_VALUE;
		double dist;
		for (HashMap.Entry<String,node> entry : map.entrySet()) { // find closest point to client
				point = entry.getValue();
				dist = point.distFrom(y,x);
				if( dist < min_dist ){
					min_dist = dist;
					min = point;
				}
		}
		return min;
	}
	
	public static Stack<string_pair> create_kml(BufferedWriter kml,int limit){
		try{
			kml.write( "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://earth.google.com/kml/2.1\">\n<Document>\n<name>Routes, A* limit: "+limit+"</name>\n" );
			Stack<string_pair> c = new Stack<string_pair>();
			
			c.push(new string_pair("red","ff0000ff"));
			c.push(new string_pair("orange","501478FF"));
			c.push(new string_pair("pink","50781EF0"));
			c.push(new string_pair("purple","50FF78B4"));
			c.push(new string_pair("blue","50FA7800"));
			c.push(new string_pair("yellow","5014F0FA"));
			c.push(new string_pair("light_brown","50143CB4"));
			c.push(new string_pair("black","50000000"));
			c.push(new string_pair("light_blue","50F0FF14"));
			c.push(new string_pair("magenta","50FF78F0"));
			c.push(new string_pair("grey","50787878"));
			return c;
			
		} catch (FileNotFoundException e) {
            e.printStackTrace(); return null;
        } catch (IOException e) {
            e.printStackTrace(); return null;
		}
	
	}
	
	public static void kml_continue(Stack<String> routes,int id,Stack<string_pair> colors,BufferedWriter kml){
		try{
			string_pair color = colors.pop();
			kml.write( "<Style id=\""+color.a+"\">\n<LineStyle>\n<color>"+color.b+"</color>\n<width>4</width>\n</LineStyle>\n</Style>\n" );
			
			kml.write( "<Placemark>\n<name>Taxi "+id+"</name>\n<styleUrl>#"+color.a+"</styleUrl>\n<LineString>\n<altitudeMode>relative</altitudeMode>\n<coordinates>\n" );
			while(!routes.isEmpty()) kml.write(routes.pop() + "\n");
			kml.write("</coordinates>\n");
			kml.write("</LineString>\n</Placemark>\n");
		} catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
		}
	}

	public static void main(String []args){
		try{
			String line_full = "";
			String cvsSplitBy = ",";
			HashMap<String,node> map = new HashMap<String, node >();										  // hashmap will have the nodes of the map
			TreeSet<node> openset = new TreeSet<node>();													  // treemap will be the openset for A*
			Stack<String> path = new Stack<String>();														  // this stack will have the path for the nearest taxi 
			BufferedReader in=null;
			
			coords taxi = new coords(), best = new coords(), client = new coords(), client_depart = new coords();
			node closest_to_taxi = new node();
			double min_dist_best=Double.MAX_VALUE;
			
			BufferedWriter kml = new BufferedWriter(new FileWriter(new File("limit"+args[0]+".kml")));
			Stack<string_pair> colors = create_kml(kml,Integer.parseInt(args[0])); 							  // for kml file creation
			BufferedWriter steps_file = new BufferedWriter(new FileWriter(new File("steps"+args[0]+".out"))); // in steps file is number of algorithm steps and max openset size for each taxi 
			
			
			try { // read nodes here

				in = new BufferedReader(new FileReader("nodes.csv"));
				
				line_full = in.readLine(); 						 		// must ignore first line
				
				line_full = in.readLine(); 						 		// read second line here to keep track of previous node each time
				
				String[] line = line_full.split(cvsSplitBy);	 
				String key = line[0] + "#" + line[1] ;
				
				node point = new node(); 				 		 		// create node		
				int previous_id;
				node previous_point;
				
				point.x = Double.parseDouble(line[0]);
				point.y = Double.parseDouble(line[1]);
						
				previous_id = Integer.parseInt(line[2]);	// save current id to compare with id in next line
				previous_point = point;
				
				point.neighbour_list = new ArrayList<node>();		
				map.put(key,point);
				
				while ((line_full = in.readLine()) != null) {	 		// read third to last line

					line = line_full.split(cvsSplitBy); 				// use comma as separator
					
					key = line[0] + "#" + line[1] ; 		 			// insert to map, key is in this format : <stringX>#<stringY>
					
					int id = Integer.parseInt(line[2]);
					
					if ( !map.containsKey(key) ){						// check if this key is already in use
						point = new node(); 				 			// if not, this is the first so create node			
						point.x = Double.parseDouble(line[0]);
						point.y = Double.parseDouble(line[1]);	
						point.neighbour_list = new ArrayList<node>(); 	// create ArrayList of neighbours for this node
						map.put(key,point);
					}
					else{									 			// key is in use, no need to create new node
						point = map.get(key);
					}
					
					if( previous_id == id ){			 			// if we are in the same road just connect previous with current
						previous_point.neighbour_list.add(point);
						point.neighbour_list.add(previous_point);
					}
					previous_id = id;						 	// save current id to compare with id in next line
					previous_point = point;						// save previous node
				}
				
			} catch (ClassCastException e) {
				e.printStackTrace();			
			} catch(NullPointerException e) {
				e.printStackTrace();
			} catch (NoSuchElementException e) {
				e.printStackTrace();
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
				if (in != null) {
					try {
						in.close();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
			
			try { /* A* here */
				node start,current,goal; 									// goal is client, start is taxi
				double dist;
				String key;
				node point;
				
				/* read client */{
					in = new BufferedReader(new FileReader("client.csv"));
					line_full = in.readLine(); 						 						// must ignore first line
					line_full = in.readLine();
					String[] line = line_full.split(cvsSplitBy);
					goal = closest(client.x = Double.parseDouble(line[0]), client.y = Double.parseDouble(line[1]),map); // client's departure point
					client_depart.x = goal.x;
					client_depart.y = goal.y;
					goal.isGoal = true;
					
					for (HashMap.Entry<String,node> entry : map.entrySet()) { 				// find heuristic value once
						key = entry.getKey();
						point = entry.getValue();
						point.heuristic(goal);
					}
				}
				
				/* read line of taxis and find start */
				
				in = new BufferedReader(new FileReader("taxis.csv"));
				line_full = in.readLine(); 						 							// must ignore first line
				
				while( ( line_full = in.readLine() ) != null){
					int steps = 0;															// count number of algorithm's steps for each taxi
					int max_openset_size = Integer.MIN_VALUE;								// find actual maximum openset size for each taxi 
					for (HashMap.Entry<String,node> entry : map.entrySet()) { 				// initiallize data every time
						key = entry.getKey();
						point = entry.getValue();
						point.init();
					}
					openset.clear();
					
					String[] line = line_full.split(cvsSplitBy);
					taxi.x = Double.parseDouble(line[0]);
					taxi.y = Double.parseDouble(line[1]);
					taxi.id = Integer.parseInt(line[2]);
					
					start = closest(taxi.x,taxi.y,map);
					start.isStart = true;
					start.g = 0;
					
					openset.add(start);
					while( !openset.isEmpty() ) {
						steps++;															// count steps
						current = openset.pollFirst(); 										// get and remove 
						if (current.isGoal){
							node end = current;
							if(current.g < min_dist_best){ 									// we only keep the taxi with the shortest path
								best.id = taxi.id;
								best.x = taxi.x;
								best.y = taxi.y;
								closest_to_taxi = start;
								min_dist_best = current.g;
								
								/* reconstruct path */
								
								path.clear(); 												// remove previous elements
								path.push(client.x + "," + client.y + ",0");				// add client's location
								path.push(goal.x + "," + goal.y + ",0");					// and client's departure location
								while(!current.isStart){
									path.push(current.father.x + "," + current.father.y + ",0");
									current = current.father;
								}
								path.push(closest_to_taxi.x + "," + closest_to_taxi.y + ",0");				// add taxi's closest existing node
								path.push(taxi.x + "," +taxi.y + ",0");										// this is taxi's actual location
							}
							
							/* print all taxi routes for kml file */
							
								Stack<String> routes = new Stack<String>();
								routes.push(client.x + "," + client.y + ",0");				// add client's location
								routes.push(goal.x + "," + goal.y + ",0");					// and client's departure location
								while(!end.isStart){
										routes.push(end.father.x + "," + end.father.y + ",0");
										end = end.father;
								}
								routes.push(start.x + "," + start.y + ",0");				// add taxi's closest existing node
								routes.push(taxi.x + "," +taxi.y + ",0");					// this is taxi's actual location
								
								/* write to kml and pop colors */
								
								kml_continue(routes,taxi.id,colors,kml);
								
								/* print number of steps for each taxi */
								
								steps_file.write("Taxi " + taxi.id + ", steps: " + steps + ", max openset size: " + max_openset_size + "\n");
								
							break;															// stop loop, next check taxi
						}
						current.visited = true;
						
						/* for each non visited neighbour node: add it to the open set */
						
						for( int i = 0; i < current.neighbour_list.size() ; i++ ){
							node neighbour = current.neighbour_list.get(i);
							if( !neighbour.visited ){
								dist = current.g + current.distFrom(neighbour.y,neighbour.x);
								if( dist < neighbour.g ){
									openset.remove(neighbour);
									neighbour.father = current;
									neighbour.g = dist;
									openset.add(neighbour);
								}
								
								if( openset.size() > Double.parseDouble(args[0]) ) openset.pollLast(); 	// if openset's size exceeded given limit, remove last node
								if( openset.size() > max_openset_size ) max_openset_size = openset.size();	// find actual maximum openset size
							}
						}					
					}
				}
				
				/* close steps file */
				
				steps_file.close();
				
				/* we found the closest point to the taxi, therefore add their distance to the final distance of the path */
				/* add the client's distance to the goal (client's departure point) too */
				
				System.out.println("Shortest distance: " + ( min_dist_best + closest_to_taxi.distFrom(best.y,best.x) + client.distFrom(client_depart.y,client_depart.x) ) + " meters" ); 
				System.out.println("Call taxi with coordinates " + "X: " + best.x +", Y: "+ best.y + " and Id: " + best.id);
				
				/* print results into a file too */
				
					BufferedWriter output = new BufferedWriter(new FileWriter(new File("results_limit"+args[0]+".out")));
					
					output.write("Shortest distance: " + ( min_dist_best + closest_to_taxi.distFrom(best.y,best.x) ) + " meters" + "\n");
					output.write("Call taxi with coordinates " + "X: " + best.x +", Y: "+ best.y + " and Id: " + best.id + "\n");
					output.close();
				
			} catch (ClassCastException e) {
				e.printStackTrace();			
			} catch(NullPointerException e) {
				e.printStackTrace();
			} catch (NoSuchElementException e) {
				e.printStackTrace();
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			} finally {
				if (in != null) {
					try {
						in.close();
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
			}
			
			/* print path to kml file */
			
			try {
				kml.write( "<Style id=\"green\">\n<LineStyle>\n<color>ff009900</color>\n<width>4</width>\n</LineStyle>\n</Style>\n" );
			
				kml.write( "<Placemark>\n<name>Taxi "+best.id+"</name>\n<styleUrl>#green</styleUrl>\n<LineString>\n<altitudeMode>relative</altitudeMode>\n<coordinates>\n" );
				
				while( !path.isEmpty() ) kml.write(path.pop() + "\n");
				
				kml.write("</coordinates>\n");
				
				/* end kml file */ 
				
				kml.write("</LineString>\n</Placemark>\n</Document>\n</kml>\n");
				
				kml.close();
				
			} catch (IOException e) {
					e.printStackTrace();
			}		
		}catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
		}  
	}
}
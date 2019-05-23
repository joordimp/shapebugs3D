/*
  Copyright 2006 by Sean Luke and George Mason University


  Licensed under the Academic Free License version 3.0
  See the file "LICENSE" for more information
*/

/*
 * PONE EN MARCHA LA SIMULACIÖN
 * 
 */
package com.jemop.apps.shapeBugs3D;
import sim.engine.*;
import sim.util.*;
import sim.field.continuous.*;



public class ShapeBugs3D extends SimState
    {
    private static final long serialVersionUID = 1;
    
    public Continuous3D environment;
    public static final double widthX = 100;
    public static final double widthY = 100;
    public static final double height = 100;
    public static final double DIAMETER = 2;
    
    public int numBugs = 500;
    public double randomness = 1.0;
    public double neighborhood = 4;
    public double maxNeighborhood = 20;
    public double repulsiveRadius = 8;
    
    public static final int cHeight = 5;
    public static final int cWidthX = 5;
    public static final int cWidthY = 5;
    
    //ShapeMode = Cómo se elige la forma (0: por grid, 1: por ecuación)
    //particleOutsideMode = Cómo se comporta una partícula que está fuera de la forma (0: encontrar vecino en forma, 1: eliminar partícula)
    public int shapeMode=1, particleOutsideMode=1;
    public int form = 0;
    public int[][][] mS1 = {{{0,0,0,0,0},
    						{0,0,0,0,0},
    						{0,0,0,0,0},
    						{0,0,0,0,0},
    						{0,0,0,0,0}},
					    			{{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,1,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0}},
				    		{{0,0,0,0,0},
							{0,0,1,0,0},
							{0,1,1,1,0},
							{0,0,1,0,0},
							{0,0,0,0,0}},
					    			{{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,1,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0}},
					    	{{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0}}};
    
    public int[][][] mS2 = {{{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0}},
					    			{{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0}},
							{{0,0,0,0,0},
							{0,0,0,0,0},
							{0,1,1,1,0},
							{0,0,0,0,0},
							{0,0,0,0,0}},
					    			{{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0}},
					    	{{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0},
							{0,0,0,0,0}}};
							
    
    public double jump = 0.7;  // how far do we move in a timestep?
    
    
   
    public double getRandomness() { return randomness; }
    public void setRandomness(double val) { if (val >= 0.0) randomness = val; }
    public int getNumBugs() { return numBugs; }
    public void setNumBugs(int val) { if (val >= 1) numBugs = val; }
    public double getNeighborhood() { return neighborhood; }
    public void setNeighborhood(double val) { if (val > 0) neighborhood = val; }
    
    public void setForm(int val){
    	form = val;
    	//System.out.println("Cambiado form a "+form);
    }
    
    // 2a forma de expresar el patron, mediante condiciones
    public boolean inequation(double x, double y, double z)
    {
    	//return Math.pow(x-50,2)+Math.pow(y-50,2)+Math.pow(z-50, 2)<400;
    	boolean ineq;
    	
    	switch (form)
    	{
    	default:
    	case 0://CONO 
    		//ineq=Math.pow(x-50,2)+Math.pow(y-50,2)<2*(z-20) && z>20 && z<80; break;
    		
    		//ESPIRAL
    		//ineq= Math.pow(x-50-30*Math.cos(z/8),2)+Math.pow(y-50-30*Math.sin(z/8),2)<144; break;
    
    		//TUERCA
    		ineq= Math.pow(x-50,2)+Math.pow(y-50,2)<900 && Math.pow(x-50,2)+Math.pow(y-50,2)>400 && z>30 && z<50; break;
    	
    	case 1: //CONO MAS PEQUEÑO
    			//ineq=Math.pow(x-50,2)+Math.pow(y-50,2)<(z-20) && z>20 && z<80; break;
    		
    		//ESPIRAL CORTADA
    		//ineq= Math.pow(x-50-30*Math.cos(z/8),2)+Math.pow(y-50-30*Math.sin(z/8),2)<144 && z<30; break;
    		
    		//MEDIA TUERCA
    		ineq= Math.pow(x-50,2)+Math.pow(y-50,2)<900 && Math.pow(x-50,2)+Math.pow(y-50,2)>400 && z>30 && z<50 && x>50; break;
    	
    	case 2: //CONO
    			//ineq=Math.pow(x-50,2)+Math.pow(y-50,2)<4*(z-20) && z>20 && z<80; break;
    	
    		//ESPIRAL ENTERA
    		//ineq= Math.pow(x-50-30*Math.cos(z/8),2)+Math.pow(y-50-30*Math.sin(z/8),2)<144; break;
    		
    		//TUERCA ENTERA
			ineq= Math.pow(x-50,2)+Math.pow(y-50,2)<900 && Math.pow(x-50,2)+Math.pow(y-50,2)>400 && z>30 && z<50; break;
    	}
    	
    	return ineq;
    }
        
    //Mira si una determinada localización está dentro de la figura (expresada en cualquiera de las 2 formas)
    public boolean isLocationInsideShape(final Double3D location)
    {	
    	if (shapeMode==0){
    		int coordX = (int) location.x/20;
    		int coordY = (int) location.y/20;
    		int coordZ = (int) location.z/20;

    		int[][][] mShape;

    		switch(form)
    		{
    		case 0: mShape=mS2; break;
    		case 1: mShape=mS1; break;
    		default: mShape=mS1;

    		}

    		return coordX>0 && coordX < cWidthX && coordY>0 && coordY<cWidthY && coordZ>0 && coordZ<cHeight 
    				&& mShape[coordX][coordY][coordZ]==1;
    	}
    	else
    	{
    		return inequation(location.x,location.y,location.z);
    	}
    }
    
    //Mira si la posiciónd e dos agentes coincide
    boolean conflict(final Double3D a, final Double3D b )
    {
    if( ( ( a.x > b.x && a.x < b.x+DIAMETER ) ||
            ( a.x+DIAMETER > b.x && a.x+DIAMETER < b.x+DIAMETER ) ) &&
            ( ( a.y > b.y && a.y < b.y+DIAMETER ) ||
            ( a.y+DIAMETER > b.y && a.y+DIAMETER < b.y+DIAMETER ) ) &&
            ( ( a.z > b.z && a.z < b.z+DIAMETER ) ||
            ( a.z+DIAMETER > b.z && a.z+DIAMETER < b.z+DIAMETER ) ))
        {
        return true;
        }
    return false;
    }
    
    //Mira si una determinada posición está dentro de la figura y no ocupada por otro agente
    boolean acceptablePosition( final Double3D location, Bug3D agent, boolean haveToKeepInShape)
    {
    	//if( location.x < DIAMETER/2 || location.x > (XMAX-XMIN)/*environment.getXSize()*/-DIAMETER/2 ||
    	//		location.y < DIAMETER/2 || location.y > (YMAX-YMIN)/*environment.getYSize()*/-DIAMETER/2 )
    	//	return false;
    	
    	if (!(location.x>=0 && location.x < widthX && location.y>=0 && location.y<widthY && location.z>=0 && location.z<height) 
    			|| (haveToKeepInShape && !isLocationInsideShape(location)))
    			return false;
    	
    	Bag misteriousObjects = environment.getNeighborsWithinDistance( location, /*Strict*/Math.max( 2*DIAMETER, 2*DIAMETER) );
    	if( misteriousObjects != null )
    	{
    		for( int i = 0 ; i < misteriousObjects.numObjs ; i++ )
    		{
    			if( misteriousObjects.objs[i] != null && (agent==null || misteriousObjects.objs[i] != agent) )
    			{
    				Object ta = (Bug3D) (misteriousObjects.objs[i]);
    				if( conflict(location, environment.getObjectLocation(ta) ) )
    					return false;
    			}
    		}
    	}
    	return true;
    }
    
    
    /** Creates a ShapeBugs3D simulation with the given random number seed. */
    public ShapeBugs3D(long seed)
        {
        super(seed);
        }
    
    public void start()
        {
        super.start();
       
        // set up the environment field.  It looks like a discretization
        // of about neighborhood / 1.5 is close to optimal for us.  Hmph,
        // that's 16 hash lookups! I would have guessed that 
        // neighborhood * 2 (which is about 4 lookups on average)
        // would be optimal.  Go figure.
        environment = new Continuous3D(neighborhood/1.5,widthX,widthY,height);
        
        // crear los agentes e iniciarlos
        for(int x=0;x<numBugs;x++)
            {
        	
            Double3D location;
            
            do
            	location = new Double3D(random.nextDouble()*widthX,random.nextDouble()*widthY, random.nextDouble() * height);	
            while(!acceptablePosition(location,null, true));
            
            Bug3D newBug = new Bug3D(location,random.nextBoolean());
            //Bug3D newBug = new Bug3D(location,true);
            
            environment.setObjectLocation(newBug, location);
            newBug.environment = environment;
            newBug.theSwarm = this;

            System.out.println("Crea bug en pos "+location.x+","+location.y+","+location.z);
            Stoppable stop=schedule.scheduleRepeating(newBug);
            newBug.setStop(stop);
            }
        
        System.out.println("Termina ShapeBugs start");
        }

    public static void main(String[] args)
        {
        doLoop(ShapeBugs3D.class, args);
        System.exit(0);
        }    
    }

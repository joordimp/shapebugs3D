
/*
 * 
 * Autor: Jorge Moreno Palenzuela
 * 
 * Descripción: Programa interno de los agentes de la simulación
 * 
 */


package com.jemop.apps.shapeBugs3D;
import sim.engine.*;
import sim.field.continuous.*;
import sim.portrayal3d.simple.SpherePortrayal3D;
import sim.util.*;
import ec.util.*;

import javax.media.j3d.*;

public class Bug3D extends SpherePortrayal3D implements Steppable
    {
    private static final long serialVersionUID = 1;
    
    public int numSteps=0;
    
    public Double3D realPos = new Double3D(0,0,0);		//Real localization of the robot
    public Double3D perceivedPos = new Double3D(0,0,0);	//Localization where the robot believes it is
  
    public Continuous3D environment;
    
    public ShapeBugs3D theSwarm;
    
    public boolean lost = true, inShape=true;
    
    Stoppable stopObject;
    
    
    public Bug3D(Double3D position, boolean isSeed) {
    	super(ShapeBugs3D.DIAMETER);
    	realPos = position;
    	inShape=true;
    	
    	if (isSeed)
    	{
    		perceivedPos=position;
    		lost=false;	
    	}
    }
    
    public void setStop(Stoppable stop){	//para eliminar al agente de la simulación
    	stopObject=stop;
    }
    
    public Bag getNeighbors() 	//Obtiene los vecinos cercanos
    {
    	if (environment!=null)
    		return environment.getNeighborsExactlyWithinDistance(realPos, theSwarm.neighborhood, true);
    	else
    		System.out.println("Llama a getNeighbors sin environment");
    	return null;
    }
    
    public Double3D getPerceivedPosition() { return perceivedPos; }
    public Double3D getRealPosition() { return realPos; }
    public boolean isLost() { return lost; }
        
    //Intenta obtener la localización fijándose en sus vecinos
    public void tryToLocalizate(Bag b, Continuous3D environment)
    {
    	if (b!=null && b.numObjs > 3)
    	{
	    	Bug3D[] neighbors = new Bug3D[3];
	    	int numNeigborsWithPosition=0;
	    	
	    	for (int i=0; i<b.numObjs && numNeigborsWithPosition<3; i++)
	    	{
	    		Bug3D neighbor = (Bug3D) b.objs[i];
	    		if (!neighbor.isLost()){
	    			neighbors[numNeigborsWithPosition]=neighbor;
	    			numNeigborsWithPosition++;
	    		}
	    	}
	    	
	    	if (numNeigborsWithPosition==3)
	    	{
	    		perceivedPos=calculatePosition(neighbors, environment);
	    		lost=false;
	    	}
    	}
    }
    
    
    /*
     * Obtiene el valor del gradiente en un punto dado.
     * 
     * 
     * min sum abs(sqrt(xi-x)^2)+(yi-y)^2)-di)
     * ~=
     * min abs(sqrt(xi-x)^2)+(yi-y)^2)-d1)+abs(sqrt(x2-x)^2)+(y2-y)^2)-d2)+abs(sqrt(x3-x)^2)+(y3-y)^2)-d3)
     * ~=
     * min (sqrt(xi-x)^2)+(yi-y)^2)-d1)^2+(sqrt(x2-x)^2)+(y2-y)^2)-d2)^2+abs(sqrt(x3-x)^2)+(y3-y)^2)-d3)^2
     * 
     * 
     */
    
    public Double3D evGradient(Double3D[] n, Double3D actual, double[] dist)
    {
    	double x=actual.x;
    	double y=actual.y;
    	double z=actual.z;
    	
    	double gX=0;
    	double gY=0;
    	double gZ=0;
    	
    	for(int i=0; i<n.length; i++)
    	{
    		//gX += -2*(n[i].x-x)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2));
        	//gY += -2*(n[i].y-y)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2));
        	
    		gX += -2*(n[i].x-x)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2));
    		gY += -2*(n[i].y-y)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2));
    		gZ += -2*(n[i].z-z)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)+Math.pow(n[i].z-z,2));
    		//gZ += -2*(n[i].z-z)*(Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2)-dist[i]))/Math.sqrt(Math.pow(n[i].x-x,2)+Math.pow(n[i].y-y,2));
    	}
    	
    	return new Double3D(gX,gY,gZ);
    }
    
    //Calcula la posición mediante el método del gradiente
    public Double3D calculatePosition(Bug3D[] neighbors, Continuous3D environment)
    {
    	
    	Double3D currentPos;
    	double maxError = 0.1;
    	double beta=0.5;
    	int maxIterations=500;
    	
    	Double3D[] neighborsPos = new Double3D[neighbors.length];
    	double dist[] = new double[neighbors.length];
    	double dX, dY, dZ;
    	for (int i=0; i<dist.length; i++)
    	{
    		neighborsPos[i]=neighbors[i].getPerceivedPosition();
    		dX = neighbors[i].getRealPosition().x-realPos.x;
    		dY = neighbors[i].getRealPosition().y-realPos.y;
    		dZ = neighbors[i].getRealPosition().z-realPos.z;
			
    		dist[i]=Math.sqrt(Math.pow(dX,2)+Math.pow(dY,2)+Math.pow(dZ,2));
    	}
    	
    	
    	
    	if (lost)
    		currentPos = new Double3D(realPos.x+(Math.random()-0.5)*4,realPos.y+(Math.random()-0.5)*4,realPos.z+(Math.random()-0.5)*4);
    	else
    		currentPos = perceivedPos;
 
    	
    	
    	Double3D gradient = evGradient(neighborsPos, currentPos, dist);
    	int it=0;
    	
    	while(it<maxIterations && Math.abs(gradient.x)<maxError && Math.abs(gradient.y)<maxError && Math.abs(gradient.z)<maxError)
    	{
    		currentPos = new Double3D(currentPos.x-beta*gradient.x, currentPos.y-beta*gradient.y,currentPos.z-beta*gradient.z);
    		gradient = evGradient(neighborsPos, currentPos, dist);
    		it++;
    	}
    	
    	System.out.println("Encontrada posición de partícula");
    	//TODO: contemplar que no encuentre
    	return currentPos;
    }
    
    //Genera un vector de movimiento aleatorio
    public Double3D randomMove(MersenneTwisterFast r)
    {
    	int tries=0;
    	Double3D vRandom;
    	double x,y,z;
    	
    	do{
    		 x = r.nextDouble() * 2 - 1.0;
    		 y = r.nextDouble() * 2 - 1.0;
    		 z = r.nextDouble() * 2 - 1.0;
    		 tries++;
    		 vRandom=new Double3D(x,y,z);
    	} while (tries<20 && !theSwarm.acceptablePosition(new Double3D(perceivedPos.x+vRandom.x, perceivedPos.y+vRandom.y, perceivedPos.z+vRandom.z),this,inShape));
    	//No permitimos que salga del contenedor Si ya esta en el
		
    		
    	if (tries>=20)
    		vRandom=new Double3D(0,0,0);
   
    return vRandom;
    }

    //Genera un vector de movimiento a partir de las fuerzas que ejercen los vecinos
    public Double3D calculateMovementVector(Bag b, Continuous3D environment, double repulsionRadius)
    {
    	double mX= 0.0;
    	double mY= 0.0;
    	double mZ= 0.0;
    	
    	if (b!=null && b.numObjs > 0)
    	{
    		double distance;
    		for (int i=0; i<b.numObjs; i++)
	    	{
    			Bug3D neighbor = (Bug3D) b.objs[i];
    			
    			if (neighbor==this)
    				continue;
    			//double dX = environment.tdx(neighbor.getPerceivedPosition().x,perceivedPos.x);
    			//double dY = environment.tdy(neighbor.getPerceivedPosition().y,perceivedPos.y);
    			double dX=neighbor.getPerceivedPosition().x-perceivedPos.x;
    			double dY=neighbor.getPerceivedPosition().y-perceivedPos.y;
    			double dZ=neighbor.getPerceivedPosition().z-perceivedPos.z;
    			distance=Math.sqrt(dX*dX+dY*dY+dZ*dZ);
    			
    			if (distance<repulsionRadius)
    			{
    				mX-=dX*(repulsionRadius-distance)/distance;
    				mY-=dY*(repulsionRadius-distance)/distance;
    				mZ-=dZ*(repulsionRadius-distance)/distance;
    				//System.out.println("se calcula el vector:"+mX+","+mY+"\ndistancia:"+distance);
    			}
	    	}
    	}
    	
    	return new Double3D(mX,mY,mZ);
    }
    
    
    
    /*
     * Método principal del programa interno, se ejecuta una vez en cada paso de la simulación.
     * Controla los cálculos y movimientos que realiza el agente.
     */
    public void step(SimState state)
        {
    	//System.out.println("Empieza step");
    	 	
        final ShapeBugs3D swarm = (ShapeBugs3D)state;
        realPos = swarm.environment.getObjectLocation(this);
        //System.out.println("Continua step, realpos="+realPos.x+","+realPos.y);
        
        numSteps++;
    	
        //para cambiar de forma durante la simulación
    	if (numSteps==500)
    		swarm.setForm(1);
    	else if (numSteps==700)
    		swarm.setForm(2);
        
        Double3D vMove = new Double3D(0,0,0);
        
        if (lost)	//El agente no conoce su posición
        {
        	System.out.println("Partícula perdida");
        	Bag b = getNeighbors();
        	tryToLocalizate(b,swarm.environment);
        	vMove = randomMove(swarm.random);
        } 
        else
        {
        	inShape = swarm.isLocationInsideShape(perceivedPos);	//Comprueba si el agente está dentro de la figura
        	
        	if (!inShape)
        	{
	        	//Manejar si en algun mommento esta fuera de la forma 
        		Bag b = getNeighbors();
        		if (swarm.particleOutsideMode==0){	//Encontrar vecino dentro de forma
        			int it=1, itMax=3;
        			boolean foundNeighborInside = false;

        			while(!foundNeighborInside && it<itMax)
        			{
        				if (b!=null && b.numObjs>1)
        				{
        					for (int i=0; i<b.numObjs && !foundNeighborInside; i++)
        					{
        						Bug3D n = (Bug3D) b.objs[i];
        						if (!n.isLost() && theSwarm.isLocationInsideShape(n.perceivedPos))
        						{
        							System.out.println("Encuentra vecino");
        							vMove = new Double3D((n.perceivedPos.x-perceivedPos.x)/2,(n.perceivedPos.y-perceivedPos.y)/2,(n.perceivedPos.z-perceivedPos.z)/2);
        							foundNeighborInside=true;
        						}
        					}
        				}

        				it++;

        				if (!foundNeighborInside && it<itMax)
        					environment.getNeighborsExactlyWithinDistance(realPos, swarm.maxNeighborhood,false);
        			}

        			if (!foundNeighborInside)
        				vMove=randomMove(swarm.random);   
        		}	
        		else	//ParticleOutsideMode=1: Eliminar partícula
        		{
        			environment.remove(this);
        			stopObject.stop();
        			return;
        		}
	       }
        	else	//El agente está dentro y no está perdido
        	{
        		Bag b = getNeighbors();
        		if (b.numObjs>1){
        			vMove = calculateMovementVector(b, swarm.environment, swarm.repulsiveRadius);
        			if (!swarm.acceptablePosition(new Double3D(vMove.x+perceivedPos.x, vMove.y+perceivedPos.y, vMove.z+perceivedPos.z), this, true))
        				vMove = randomMove(swarm.random);
        		}
        		else
        			vMove = randomMove(swarm.random);
        	}
        }
      
        //Maneja el movimiento final del agente, tras los cálculos anteriores, y actualiza las posiciones
        double dx = vMove.x;
        double dy = vMove.y;
        double dz = vMove.z;
        
        perceivedPos = new Double3D(perceivedPos.x+dx,perceivedPos.y+dy,perceivedPos.z+dz); 
        realPos = new Double3D(realPos.x+dx,realPos.y+dy,realPos.z+dz);
        swarm.environment.setObjectLocation(this, realPos);
        }
 
    }

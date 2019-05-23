/*
  Copyright 2006 by Sean Luke and George Mason University
  Licensed under the Academic Free License version 3.0
  See the file "LICENSE" for more information
*/


/*
 * 
 * MANEJA EL ASPECTO VISUAL (GUI) DE LA SIMULACIÖN
 * 
 */
package com.jemop.apps.shapeBugs3D;

import sim.engine.*;
import sim.display3d.*;
import sim.display.*;
import sim.portrayal3d.continuous.ContinuousPortrayal3D;
import sim.portrayal3d.simple.*;
import java.awt.*;
import javax.swing.*;

public class ShapeBugs3DWithUI extends GUIState {
	public Display3D display;
	public JFrame displayFrame;

	ContinuousPortrayal3D particlesPortrayal = new ContinuousPortrayal3D();
	WireFrameBoxPortrayal3D wireFramePortrayal;

	public static void main(String[] args) {
		new ShapeBugs3DWithUI().createController();
	}

	public ShapeBugs3DWithUI() {
		super(new ShapeBugs3D(System.currentTimeMillis()));
	}

	public ShapeBugs3DWithUI(SimState state) {
		super(state);
	}

	public static String getName() {
		return "3D ShapeBugs";
	}

	public void quit() {
		super.quit();

		if (displayFrame != null)
			displayFrame.dispose();
		displayFrame = null; // let gc
		display = null; // let gc
	}

	public void start() {
		super.start();
		// set up our portrayals
		setupPortrayals();
	}

	public void load(SimState state) {
		super.load(state);
		setupPortrayals();
	}

	public void setupPortrayals() {
		// display.destroySceneGraph();

		particlesPortrayal.setField(((ShapeBugs3D) state).environment);
		particlesPortrayal.setPortrayalForAll(new SpherePortrayal3D(Color.red));

		// reschedule the displayer
		display.reset();

		// redraw the display
		display.createSceneGraph();
	}

	public void init(Controller c) {
		super.init(c);
		display = new Display3D(600, 600, this);

		wireFramePortrayal = new WireFrameBoxPortrayal3D(-0.5, -0.5, -0.5, ShapeBugs3D.widthX, ShapeBugs3D.widthY,
				ShapeBugs3D.height, Color.blue);

		// attach the portrayals
		display.attach(wireFramePortrayal, "Wire Frame");
		display.attach(particlesPortrayal, "Particles");

		display.translate(-ShapeBugs3D.widthX / 2.0, -ShapeBugs3D.widthY / 2.0, -ShapeBugs3D.height / 2.0);

		display.scale(1.0 / ShapeBugs3D.widthX);

		display.setBackdrop(Color.white);
		displayFrame = display.createFrame();
		c.registerFrame(displayFrame); // register the frame so it appears in
										// the "Display" list
		displayFrame.setVisible(true);

	}
}

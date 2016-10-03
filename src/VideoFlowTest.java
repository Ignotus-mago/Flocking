import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.awt.geom.Point2D;
import java.io.PrintWriter;
import java.util.ArrayList;
import net.paulhertz.geom.GeomUtils;
import net.paulhertz.geom.Matrix3;
import net.paulhertz.util.RandUtil;
import com.ignofactory.steering.*;
import net.paulhertz.aifile.*;
import controlP5.*;
import processing.core.*;
import processing.video.*;
import processing.core.PApplet;


public class VideoFlowTest extends PApplet {
	int displayWidth;
	int displayHeight;
	int videoWidth;
	int videoHeight;
	ControlP5 controlP5;
	ControlGroup settings;
	Tab settingsTab;
	// public IgnoCodeLib igno;
	public OpticalFlower optical;
	public ImageUtil imageUtil;
	public PGraphics pg;
	VideoCallbackINF videoResponder;
	public boolean isVideoReady = false;
	public boolean isShowVideo = false;
	String[] videoDevices;

	
	public void setup() {
		// for "best" results, make displayWidth/displayHeight == videoWidth/videoHeight
		// other proportions will distort video but can be useful
		size(1280, 720);
		smooth();
		frameRate = 15;
		displayWidth = width;
		displayHeight = height;
		videoWidth = 640;
		videoHeight = 360;
		imageUtil = new ImageUtil(this);
		controlP5 = new ControlP5(this);       // initialize ControlP5
		offscreenSetup();
		loadPanel();
		// setupVideo(videoWidth, videoHeight, 30, 8, 0.25f, "FaceTime HD Camera");
	}

	/**
	 * Method required for Eclipse IDE and standalone Java.
	 * @param args
	 */
	public static void main(String[] args) {
		PApplet.main(new String[] { "--present", "VideoFlowTest" });
	}
		

	public void draw() {
		if (!isShowVideo) {
			if (width == pg.width && height == pg.height) background(pg);
			else {
				pg.resize(width, height);
				background(pg);
				println("Resized pg.");
				println("width = "+ width +", height = "+ height);
				println("pg.width = "+ pg.width +", pg.height = "+ pg.height);
			}
		}
		if (isVideoReady) {
			optical.flow();
			if (!(optical.isFlagflow())) {
				println("draw vector lines from parent");
				this.drawVectorLines();
			}
		}
	}
	
	
	// key presses TODO
	public void keyPressed() {
		if (key == ' ') {
			if (controlP5.isVisible()) {
				controlP5.hide();
			}
			else {
				controlP5.show();
			}
		}
		else if (key == 'i' || key == 'I') {
			if (!isVideoReady) return;
			isShowVideo = !isShowVideo;
			if (isShowVideo) {
				optical.showImage();
			}
			else {
				optical.hideImage();
			}
		}
	}


	/******** OFFSCREEN BUFFER SETUP ********/

	public void offscreenSetup() {
		pg = createGraphics(width, height);    // offscreen graphics 
		pg.beginDraw();
		pg.smooth();
		pg.background(192, 192, 192, 127);
		pg.endDraw();
	}
	
	
	/******** VIDEO DEVICE SETUP ********/
	
	/**
	 * @param w          width of video capture
	 * @param h          height of video capture
	 * @param fps        frames per second of video capture, 15 or 30 will do fine
	 * @param grid       edge of a grid cell subdividing video 
	 * @param timespan   time window in decimal seconds to track motion flow (prediction interval)
	 * @param device     name of the video capture device, will vary with the gear you are using
	 */
	public void setupVideo(int w, int h, int fps, int grid, float timespan, String device) {
		if (null == optical) {
			optical = new OpticalFlower(this, w, h, fps, grid, timespan, device);
		}
		else {
			optical.setVideoParams(w, h, fps, grid, timespan, device);
		}
		isVideoReady = optical.init();
		if (!isVideoReady) {
			println("***** Video failed to initialize. Please check that you are using an available device and settings. *****");
			println("***** Select device and settings from the following list: *****");
			printDevices();
		}
		else {
			optical.setFlowColor(color(233, 220, 199, 127));
			optical.setImageFlowColor(color(144, 110, 233, 255));
			videoResponder = new VideoResponder();
			optical.setResponder(videoResponder);
		}
	}
	
	/**
	 * @return   a list of attached video devices and information about each
	 */
	public String[] getVideoDevices() {
		if (null == videoDevices) {
			videoDevices = Capture.list();
		}
		return videoDevices;
	}
	
	/**
	 * Prints available video device information to the console.
	 */
	public void printDevices() {
		PApplet.println("Available Video Devices:");
		String[] devices = getVideoDevices();
		for (String device : devices) {
			PApplet.println("-- "+ device);
		}
	}
	
	
	/************* Control Panel *************/

	// TODO 
	/**
	 * Sets up the control panel.
	 */
	public void loadPanel() {
		int panelBack = this.color(123, 123, 144, 255);
		int settingsWidth = 270;
		int panelHeight = 280;
		int startPos = 20;
		int yPos = startPos + 4;
		int step = 18;
		int widgetH = 14;
		int menuH = widgetH + 2;
		int labelW = 144;
		settings = controlP5.addGroup("Global Settings", 4, yPos, settingsWidth);
		settings.setBackgroundColor(panelBack);
		settings.setBackgroundHeight(panelHeight);
		settings.setBarHeight(widgetH);
		settings.setMoveable(true);
		// add widgets
		// drop down list for video devices
		DropdownList dd1 = controlP5.addDropdownList("devicelist", 8, startPos-20, 240, widgetH + 2);
		dd1.setBarHeight(menuH);
		dd1.setItemHeight(menuH);
		String[] devices = getVideoDevices();
		dd1.setHeight(devices.length * menuH + menuH + 2);
		for (int i = 0; i < devices.length; i++) {
			dd1.addItem(devices[i], i);
		}
		dd1.setOpen(false);
		dd1.setGroup(settings);
		// finish
		settings.moveTo("global");
		settingsTab = controlP5.getTab("default");
		settingsTab.activateEvent(true);
		settingsTab.setLabel("Flock Settings");
		settingsTab.setId(1);
	}
	

	/**
	 * TODO be sure controls correctly reflect the initial settings
	 * Called by control panel checkbox "setUseFactors," locks or unlocks 
	 * alignment and cohesion number boxes.
	 * @param evt
	 */
	public void controlEvent(ControlEvent evt) {
		if (evt.isController() && evt.getController().getName().equals("devicelist")) {
			int index = (int) evt.getController().getValue();
			String[] devices = getVideoDevices();
			// name=Built-in iSight,size=320x240,fps=30
			String device = devices[index];
			//println("selected device is "+ device +", index = "+ index);
			String[] info = device.split(",");
			//for (int i = 0; i < info.length; i++) {
			//	println(info[i]);
			//}
			String deviceName = info[0].substring(info[0].indexOf("=") + 1);
			println("device name = "+ deviceName);
			String dimensions = info[1].substring(info[1].indexOf("=") + 1);
			//println("dimensions = "+ dimensions);
			String[] coords = dimensions.split("x");
			int w = Integer.valueOf(coords[0]);
			int h = Integer.valueOf(coords[1]);
			videoWidth = w;
			videoHeight = h;
			println("device width = "+ w);
			println("device height = "+ h);
			String fps = info[2].substring(info[2].indexOf("=") + 1);
			int framerate = Integer.valueOf(fps);
			println("device fps = "+ fps);
			int grid = w/40;
			println("device grid = "+ grid);
			println("device horizontal scale = "+ width/(float) videoWidth);
			println("device vertical scale = "+ height/(float) videoHeight);
			float predictionSeconds = 0.25f * 30.0f/framerate;
			setupVideo(w, h, framerate, grid, predictionSeconds, deviceName);		
		}
		// debugging
		/**/
		if (evt.isGroup()) {
			println(evt.getGroup().getValue() +" from group "+ evt.getGroup() +" with name "+ evt.getGroup().getName());
		}
		else if (evt.isController()) {
			println(evt.getController().getValue() +" from controller "+ evt.getController() +" with name "+ evt.getController().getName());
		}
		/**/
	}

	
	/************* VideoResponder Class *************/

	boolean skipAction = true;
	class VideoResponder implements VideoCallbackINF {
		float actionThreshold = 0.5f;
		float sepMax = 233;
		float sepMin = 3.0f;
		int evtTimer;
		int evtDebounce = 120;
		
		@Override
		public void videoCallback(Capture video) {
			// fill the background
			background(pg);
			// get the video image, give it an alpha channel and draw it on our display
			PImage img = imageUtil.loadImageAlpha(video.get(), 127);
			image(img, 0, 0, width, height);
			// println("videoCallback");
		}

		@Override
		public void vectorCallback(Capture video) {
			drawVectorLines(pg);
		}
		
		@Override
		public void actionCallback(Capture video) {
			if (skipAction) return;
	
		}
		
		public void markGrid(int x, int y, int fillColor) {
			// find the grid
			ellipseMode(PApplet.CENTER);
			noStroke();
			fill(fillColor);
			ellipse(x, y, 20, 20);
		}
		
	}
	
	public void drawVectorLines() {
		pushStyle();
		stroke(color(55, 55, 89, 192));
		strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		if (null == vec || 0 >= vec.length || null == vec[0]) {
			println("vec array is empty");
			return;
		}
		float flowScale = optical.getFlowScale();
		// TODO scale flow lines when drawing window has different proportions from video window
		float hs = (width/(float) videoWidth);
		float vs = (height/(float) videoHeight);
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
//			float a = PApplet.sqrt(u * u + v * v);
//			if(a >= 2.0) line(x0, y0, x0 + u * flowScale, y0 + v * flowScale);
				line(x0, y0 * vs, x0 + u * hs, y0 * vs + v * vs);
		}
		popStyle();
	}
	
	public void drawVectorLines(PGraphics offscreen) {
		offscreen.beginDraw();
		offscreen.background(192, 192, 192, 127);
		offscreen.pushStyle();
		offscreen.stroke(color(55, 55, 89, 255));
		offscreen.strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		if (null == vec || 0 >= vec.length || null == vec[0]) {
			println("vec array is empty");
			return;
		}
		float flowScale = optical.getFlowScale();
		// TODO scale flow lines when drawing window has different proportions from video window
		float hs = (width/(float) videoWidth);
		float vs = (height/(float) videoHeight);
		int rowCount = optical.getGw();
		int columnCount = optical.getGh();
		int g1 = optical.getGs();
		int g2 = g1/2;
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
			if (i % rowCount == 0 || i % rowCount == rowCount - 1
					|| i < rowCount || i > vec.length - rowCount) {
				offscreen.stroke(color(233, 0, 89, 255));
				offscreen.fill(color(233, 0, 89, 255));
				// offscreen.ellipse(x0 * hs, y0 * vs, 5, 5);
				offscreen.ellipse(x0, y0, 5, 5);
			}
			else {
				offscreen.stroke(color(55, 55, 89, 255));
			}
			float a = PApplet.sqrt(u * u + v * v);
			if(a >= 2.0) {
				offscreen.line(x0, y0, x0 + u * flowScale, y0 + v * flowScale);
				// offscreen.line(x0, y0, x0 + u * hs, y0 + v * vs);
				// offscreen.line(x0 * hs - g1, y0 * vs, x0 * hs + u * hs - g1, y0 * vs + v * vs);
			}
		}
		offscreen.popStyle();
		offscreen.endDraw();
	}


	public void drawVectorEllipses() {
		pushStyle();
		noStroke();
		fill(color(233, 55, 89, 127));
		strokeWeight(1);
		float[] vx = optical.getVxcoords();
		float[] vy = optical.getVycoords();
		PVector[] vec = optical.getFlowList();
		float flowScale = optical.getFlowScale();
		for (int i = 0; i < vec.length; i++) {
			float u = vec[i].x;
			float v = vec[i].y;
			float x0 = vx[i];
			float y0 = vy[i];
			float a = PApplet.sqrt(u * u + v * v);
			if(a >= 2.0) {
				ellipse(x0, y0, a, a);
			}
		}
		popStyle();
	}


}

package scene;

import kha.Font;
import kha.Key;
import kha.FontStyle;
import kha.Loader;
import kha.Color;

import komponent.ds.Matrix;
import komponent.GameObject;
import komponent.Scene;
import komponent.components.misc.Camera;
import komponent.input.Keyboard;
import komponent.input.Input;
import komponent.input.Mouse;
import komponent.utils.Painter;
import komponent.components.graphic.Image;
import komponent.components.graphic.Text;

import components.ShipController;
import components.CameraController;

class MatrixTest extends Scene
{
	
	public var shipControl:Bool = true;
	
	var font:Font;
	
	override public function begin()
	{
		font = Loader.the.loadFont("Arimo", new FontStyle(false, false, false), 20);
		
		var camera = new GameObject("Camera", 0, 0);
		camera.addComponent(Camera);
		camera.addComponent(CameraController);
		
		var ship = new GameObject("Ship", 0, 0);
		ship.addComponent(ShipController);
		ship.transform.localLayer = -1;
		
		var name = new GameObject("ShipName", 0, -60);
		name.addComponent(Text).set("Ship", font);
		name.transform.localScaleX = name.transform.localScaleY = 2;
		name.transform.attachTo(ship);
		
		var rect = new GameObject("Rectangle", 0, 0);
		rect.addComponent(Image).load("Rect");
		rect.transform.localScaleX = rect.transform.localScaleY = 0.75;
		
		
		Keyboard.define("move_left", ["a"], [Key.LEFT]);
		Keyboard.define("move_right", ["d"], [Key.RIGHT]);
		Keyboard.define("move_up", ["w"], [Key.UP]);
		Keyboard.define("move_down", ["s"], [Key.DOWN]);
		Keyboard.define("rotate_left", ["q"]);
		Keyboard.define("rotate_right", ["e"]);
		Keyboard.define("zoom_in", ["r"]);
		Keyboard.define("zoom_out", ["f"]);
		Keyboard.define("switch_control", [" "]);
		
		Input.defineAxis("horizontal", [KEYBOARD("move_left", -1), KEYBOARD("move_right", 1)]);
		Input.defineAxis("vertical", [KEYBOARD("move_up", -1), KEYBOARD("move_down", 1)]);
		Input.defineAxis("rotation", [KEYBOARD("rotate_left", -1), KEYBOARD("rotate_right", 1)]);
		Input.defineAxis("zoom", [KEYBOARD("zoom_in", 1), KEYBOARD("zoom_out", -1)]);
	}
	
	override public function update()
	{
		super.update();
		if (Keyboard.pressed("switch_control"))
			shipControl = !shipControl;
	}
	
	override public function render()
	{
		super.render();
		
		Painter.matrix = Matrix.identity();
		Painter.set(Color.White, 1.0, font);
		Painter.drawString("Movement: wasd", 6, 4);
		Painter.drawString("Rotation: q/e", 6, 20);
		Painter.drawString("Zoom: r/f", 6, 36);
		Painter.matrix = null;
		
		Painter.matrix = Matrix.scale(2, 2);
		if (shipControl)
		{
			Painter.color = Color.Red;
			Painter.drawString("Ship", 208, 4);
			Painter.color = Color.White;
			Painter.drawString("Camera", 248, 4);
		}
		else
		{
			Painter.color = Color.Red;
			Painter.drawString("Camera", 248, 4);
			Painter.color = Color.White;
			Painter.drawString("Ship", 208, 4);
		}
		Painter.matrix = Matrix.identity();
		
		Painter.drawString("Switch(Space)", 500, 44);
	}
	
}
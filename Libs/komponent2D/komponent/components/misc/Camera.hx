package komponent.components.misc;

import komponent.components.Transform;
import komponent.utils.Screen;
import komponent.utils.Misc;
import komponent.ds.Matrix;

class Camera extends Component
{
	
	public var x(get, set):Float;
	public var y(get, set):Float;
	
	public var scaleX(default, set):Float = 1;
	public var scaleY(default, set):Float = 1;
	
	public var zoom(default, set):Float = 1;
	
	public var fullScaleX(default, null):Float = 1;
	public var fullScaleY(default, null):Float = 1;
	
	public var matrix(get, null):Matrix;
	
	override public function added()
	{
		if (Screen.camera != null)
			trace("Only one Camera is currently supported.");
		else
			Screen.camera = this;
		
		Screen.cameras.push(this);
	}
	
	override public function removed()
	{
		if (Screen.camera == this)
			Screen.camera = null;
		Screen.cameras.remove(this);
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		scaleX = data.scaleX.parse(1.0);
		scaleY = data.scaleY.parse(1.0);
		zoom = data.zoom.parse(1.0);
	}
	
	private inline function get_x():Float { return transform.x; }
	private inline function set_x(value:Float):Float { return transform.localX = value; }
	
	private inline function get_y():Float { return transform.y; }
	private inline function set_y(value:Float):Float { return transform.localY = value; }
	
	private inline function set_scaleX(value:Float):Float
	{
		scaleX = value;
		fullScaleX = scaleX * zoom;
		matrix = null;
		return scaleX;
	}
	
	private inline function set_scaleY(value:Float):Float
	{
		scaleY = value;
		fullScaleY = scaleY * zoom;
		matrix = null;
		return scaleY;
	}
	
	private inline function set_zoom(value:Float):Float
	{
		zoom = value;
		fullScaleX = scaleX * zoom;
		fullScaleY = scaleY * zoom;
		matrix = null;
		return zoom;
	}
	
	private function onTransformChange(_)
	{
		matrix = null;
	}
	
	private function onSceneChanged(_)
	{
		removed();
	}
	
	private inline function get_matrix():Matrix
	{
		if (matrix == null)
		{
			matrix = Matrix.scale(fullScaleX, fullScaleY) *
					 Matrix.rotation(transform.rotation * Misc.toRad) *
					 Matrix.translation(x, y);
		}
		return matrix;
	}
}
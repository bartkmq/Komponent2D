package komponent.components.misc;

import komponent.components.Transform;
import komponent.utils.Screen;

class Camera extends Component
{
	
	public var x(get, set):Float;
	public var y(get, set):Float;
	
	public var scaleX(default, set):Float = 1;
	public var scaleY(default, set):Float = 1;
	
	public var zoom(default, set):Float = 1;
	
	public var fullScaleX(default, null):Float = 1;
	public var fullScaleY(default, null):Float = 1;
	
	private var _transform:Transform;
	
	override public function added()
	{
		if (Screen.camera != null)
			trace("Only one Camera is currently supported.");
		else
		{
			Screen.camera = this;
			_transform = transform;
		}
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
	
	private inline function get_x():Float { return _transform.x; }
	private inline function set_x(value:Float):Float { return _transform.localX = value; }
	
	private inline function get_y():Float { return _transform.y; }
	private inline function set_y(value:Float):Float { return _transform.localY = value; }
	
	private inline function set_scaleX(value:Float):Float
	{
		scaleX = value;
		fullScaleX = scaleX * zoom;
		return scaleX;
	}
	
	private inline function set_scaleY(value:Float):Float
	{
		scaleY = value;
		fullScaleY = scaleY * zoom;
		return scaleY;
	}
	
	private inline function set_zoom(value:Float):Float
	{
		zoom = value;
		fullScaleX = scaleX * zoom;
		fullScaleY = scaleY * zoom;
		return zoom;
	}
}
package komponent.components.misc;

import komponent.components.Transform;
import komponent.utils.Screen;

class Camera extends Component
{
	
	public var x(get, set):Float;
	public var y(get, set):Float;
	//public var scaleX(get, set):Float;
	//public var scaleY(get, set):Float;
	
	private var _transform:Transform;
	
	public function new()
	{
		
	}
	
	override public function added()
	{
		if (Screen.camera != null)
			trace("Only one Camera is supported.");
		else
		{
			Screen.camera = this;
			_transform = transform;
		}
	}
	
	override public function removed()
	{
		if (Screen.camera == this)
			Screen.camera = null;
	}
	
	private inline function get_x():Float { return _transform.x; }
	private inline function set_x(value:Float):Float { return _transform.x = value; }
	
	private inline function get_y():Float { return _transform.y; }
	private inline function set_y(value:Float):Float { return _transform.y = value; }
	
	//private inline function get_scaleX():Float { return _transform.scaleX; }
	//private inline function set_scaleX(value:Float):Float { return _transform.scaleX = value; }
	
	//private inline function get_scaleY():Float { return _transform.scaleY; }
	//private inline function set_scaleY(value:Float):Float { return _transform.scaleY = value; }
}
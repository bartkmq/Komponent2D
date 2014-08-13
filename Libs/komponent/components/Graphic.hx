package komponent.components;

import kha.Rectangle;

using komponent.utils.Parser;

class Graphic extends Component
{
	
	public var visible:Bool;
	public var bounds(get, null):Rectangle;
	public var layer(default, set):Null<Int>;

	public function new() 
	{
		visible = true;
		bounds = new Rectangle(0, 0, 0, 0);
	}
	
	override public function added()
	{
		layer = 0;
	}
	/**
	 * Called when the graphic should render.
	 */
	public function render()
	{
		
	}
	
	override public function loadConfig(data:Dynamic)
	{
		visible = data.parse(true);
		layer = data.parse(0);
	}
	
	private inline function get_bounds():Rectangle
	{
		trace("get_Bounds() not overridden!");
		return bounds;
	}
	
	private inline function set_layer(value:Int):Int
	{
		if (layer == null)
		{
			layer = value;
			scene.addGraphic(this);
		}
		else
		{
			if (value != layer)
			{
				layer = value;
				scene.removeGraphic(this);
				scene.addGraphic(this);
			}
		}
		return value;
	}
}
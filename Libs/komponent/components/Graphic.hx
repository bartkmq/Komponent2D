package komponent.components;

using komponent.utils.Parser;

class Graphic extends Component
{
	
	public var visible:Bool;
	public var layer(default, set):Null<Int>;

	public function new() 
	{
		visible = true;
	}
	
	override public function added()
	{
		layer = 0;
	}
	
	override public function loadConfig(data:Dynamic)
	{
		visible = data.parse(true);
		layer = data.parse(0);
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
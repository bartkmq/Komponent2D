package komponent.components;

using komponent.utils.Parser;

class Graphic extends Component
{
	
	public var visible:Bool;

	public function new() 
	{
		visible = true;
	}
	
	override public function loadConfig(data:Dynamic)
	{
		visible = data.parse(true);
	}
}
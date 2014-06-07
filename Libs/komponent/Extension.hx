package komponent;

@:keepSub
class Extension
{
	/**
	* Base class for all extensions.
	* A Extension is like a component for a scene.
	*/
	
	public var scene:Scene;
	public var engine(get, never):Engine;
	
	/**
	* Called when this extension is added to a scene.
	*/
	public function added():Void { }
	
	/**
	* Called when this extension should update.
	*/	
	public function update():Void { }
	
	/**
	* Called when this extension should render.
	*/
	public function render():Void { }
	
	/**
	* Called after render() when debugging
	*/
	public function debugDraw():Void { }
	
	/**
	* Called when this extension is removed from a scene
	*/	
	public function removed():Void { }
	
	private inline function get_engine():Engine { return scene.engine; }	
}
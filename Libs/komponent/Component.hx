package komponent;

import komponent.components.Transform;
import komponent.components.Graphic;
import komponent.components.Collider;

class Component
{
	/**
	* Base class for all components.
	* Use this to add functionality to a GameObject.
	* The Constructor of a Component should not need any arguments.
	*/
	
	public var gameObject:GameObject;
	public var scene(get, null):Scene;
	public var engine(get, null):Engine;
	
	private var transform(get, null):Transform;
	private var graphic(get, null):Graphic;
	private var collider(get, null):Collider;
	
	/**
	* Called when this component is added to a GameObject.
	*/
	public function added():Void { }
	
	/**
	* Called when this component should update.
	*/	
	public function update():Void { }
	
	/**
	* Called when this component should render.
	*/
	public function render():Void { }
	
	/**
	* Called after render() when debugging.
	*/
	public function debugDraw():Void { }
	
	/**
	* Called when this component is removed from a GameObject.
	*/	
	public function removed():Void { }
	
	/**
	 * Creates a component instance from a config file.
	 */
	public function loadConfig(data:Dynamic):Void { }
	
	/**
	 * Checks if the GameObject has a specific component type .
	 * @param	componentType	The type of component to search for.
	 * @return	True if a matching component was found, or false if none was found.
	 */
	private inline function hasComponent(componentType:Class<Component>):Bool
	{
		return gameObject.hasComponent(componentType);
	}
	
	/**
	 * Gets a Component from a GameObject.
	 * @param	componentType	The type of component to search for.
	 * @return	The first Component that matches, or null if none was found.
	 */
	private inline function getComponent<T:Component>(componentType:Class<T>):T
	{
		return gameObject.getComponent(componentType);
	}
	
	/**
	 * Adds a Component to the GameObject.
	 * @param	component	Component instance to add.
	 * @return	The added component.
	 */
	private inline function addComponent<T:Component>(componentType:Class<T>):T
	{
		return gameObject.addComponent(componentType);
	}
		
	/**
	 * Removes a Component from the GameObject.
	 * @param	component	Component instance to remove.
	 * @return	The added 	component.
	 */
	private inline function removeComponent<T:Component>(component:T):T
	{
		return gameObject.removeComponent(component);
	}
	
	/**
	 * Finds a GameObject by its name.
	 * @param	name	The name to search for.
	 * @return	The found GameObject or null if none was found.
	 */
	private inline function getGameObjectByName(name:String):GameObject
	{
		return scene.getGameObjectByName(name);
	}
	
	/**
	 * Finds all GameObjects by name.
	 * @param	name	The name to search for.
	 * @return	The found GameObjects or null if none were found.
	 */
	private inline function getGameObjectsByName(name:String):Iterator<GameObject>
	{
		return scene.getGameObjectsByName(name);
	}
	
	/**
	 * Finds a GameObject that has the specified component type.
	 * @param	componentType	The name to search for.
	 * @return	The found GameObject or null if none was found.
	 */
	private inline function getGameObjectByComponent(component:Class<Component>):GameObject
	{
		return scene.getGameObjectByComponent(component);
	}
	
	/**
	 * Finds all GameObjects that have the specified component type.
	 * @param	componentType	The name to search for.
	 * @return	The found GameObjects or null if none were found.
	 */
	private inline function getGameObjectsByComponent(component:Class<Component>):Array<GameObject>
	{
		return getGameObjectsByComponent(component);
	}
	
	/**
	 * Calls a function on every component in the GameObject.
	 * @param	functionName	Name of the function to call.
	 * @param	message			Dynamic object to pass to the function.
	 */
	public inline function sendMessage(functionName:String, message:Dynamic = null)
	{
		gameObject.sendMessage(functionName, message);
	}
	
	private inline function toString():String
	{
		return Type.getClassName(Type.getClass(this)).split(".").pop();
	}
	
	private inline function get_scene():Scene { return gameObject.scene; }
	
	private inline function get_engine():Engine { return gameObject.scene.engine; }
	
	private inline function get_transform():Transform { return gameObject.transform; }
	
	private inline function get_collider():Collider {	return gameObject.collider; }
	
	private inline function get_graphic():Graphic {	return gameObject.graphic; }
}
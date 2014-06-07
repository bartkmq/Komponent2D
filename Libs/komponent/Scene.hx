package komponent;

import komponent.extension.Nape;
import komponent.utils.Misc;
import komponent.utils.Painter;
import komponent.utils.Screen;

using komponent.utils.Misc;

@:keepSub
class Scene
{
	/**
	* Base class for all scenes.
	* 
	* Dispatches:
	* -> onGameObjectAdded(gameObject) when a GameObject is added to this scene.
	* -> onGameObjectRemoved(gameObject) when a GameObject is removed from this scene.
	* -> onQuit() when the game is quit
	*/
	
	public var engine:Engine;
	
	public var gameObjects:Array<GameObject>;
	public var names:Map<String, List<GameObject>>;
	
	public var extensions:Array<Extension>;

	/**
	* Constructor. Should have no arguments.
	*/
	public function new() 
	{
		gameObjects = [];
		names = new Map();
		extensions = [];
	}
	
	/**
	* Called when this scene is added to the engine.
	*/
	public function begin():Void { }
	
	/**
	* Called when this scene gains focus.
	*/
	public function focusGained():Void { }
	
	/**
	* Called when this scene looses focus.
	*/
	public function focusLost():Void { }
	
	/**
	* Called when this scene is removed from the engine.
	*/	
	public function end():Void { }
	
	/**
	* Called when the game is quit.
	*/	
	public function quit():Void
	{
		sendMessage("onQuit");
	}
	
	/**
	* Called when this scene should update.
	*/	
	public function update():Void
	{
		for (gameObject in gameObjects)
		{
			if (gameObject.active)
				gameObject.update();
		}
		for (extension in extensions)
			extension.update();
	}
	
	/**
	* Called when this scene should render.
	*/
	public function render():Void
	{
		if (Screen.color != null)
		{
			Painter.color = Screen.color;
			Painter.clear();
		}
		for (gameObject in gameObjects)
		{
			if (gameObject.active)
				gameObject.render();
		}
		for (extension in extensions)
		{
			extension.render();
			if (engine.debug)
				extension.debugDraw();
		}
	}
	
	public inline function remove(gameObject:GameObject):GameObject
	{
		gameObjects.remove(gameObject);
		removeName(gameObject.name, gameObject);
		sendMessage("onGameObjectRemoved", gameObject);
		return gameObject;
	}
	
	/**
	 * Finds a GameObject by its name.
	 * @param	name	The name to search for.
	 * @return	The found GameObject or null if none was found.
	 */
	public inline function getGameObjectByName(name:String):GameObject
	{
		var objects = names[name];
		if (objects != null)
			return objects.first();
		else
			return null;
	}
	
	/**
	 * Finds all GameObjects by name.
	 * @param	name	The name to search for.
	 * @return	The found GameObjects or null if none were found.
	 */
	public inline function getGameObjectsByName(name:String):Iterator<GameObject>
	{
		var objects = names[name];
		if (objects != null)
			return objects.iterator();
		else
			return null;
	}
	
	/**
	 * Finds a GameObject that has the specified component type.
	 * @param	componentType	The name to search for.
	 * @return	The found GameObject or null if none was found.
	 */
	public function getGameObjectByComponent(componentType:Class<Component>):GameObject
	{
		for (gameObject in gameObjects)
		{
			if (gameObject.hasComponent(componentType))
				return gameObject;
		}
		return null;
	}
	
	/**
	 * Finds all GameObjects that have the specified component type.
	 * @param	componentType	The name to search for.
	 * @return	The found GameObjects or null if none were found.
	 */
	public inline function getGameObjectsByComponent(componentType:Class<Component>):Array<GameObject>
	{
		var objects:Array<GameObject> = [];
		for (gameObject in gameObjects)
		{
			if (gameObject.hasComponent(componentType))
				objects.push(gameObject);
		}
		return objects;
	}
	
	/**
	 * Returns a existing Extension or creates a new one.
	 * @param	extension	The Extension type to return.
	 * @return	The Extension instance.
	 */
	public function getExtension<T:Extension>(extensionType:Class<T>):T
	{
		for (extension in extensions)
		{
			if (Std.is(extension, extensionType))
				return cast extension;
		}
		var newExtension:T = Type.createInstance(extensionType, []);
		extensions.push(newExtension);
		return newExtension;
	}
	
	/**
	 * Calls a function on all the components in the gameObjects in the scene.
	 * @param	functionName The function to call.
	 * @param	The Dynamic Object to pass to the function.
	 */
	public inline function sendMessage(functionName:String, message:Dynamic = null)
	{
		for (gameObject in gameObjects)
			gameObject.sendMessage(functionName, message);
		for (extension in extensions)
			Misc.callFunction(extension, functionName, message);
	}
	
	public function loadPrefab(data:Dynamic)
	{
		for (objectData in cast(data.gameObjects, Array<Dynamic>))
		{
			GameObject.loadPrefab(objectData);
		}
		for (extension in cast(data.extensions, Array<Dynamic>))
		{
			getExtension(Type.resolveClass(extension));
		}
	}
	
	@:noCompletion
	@:access(komponent.GameObject.components)
	@:allow(komponent.GameObject)
	private inline function add(gameObject:GameObject):Void
	{
		gameObjects.push(gameObject);
		gameObject.scene = this;
		addName(gameObject.name, gameObject);
		sendMessage("onGameObjectAdded", gameObject);
	}
	
	private inline function addName(name:String, object:GameObject):Void
	{
		if (names[name] == null)
		{
			var objectList = new List<GameObject>();
			objectList.add(object);
			names[name] = objectList;
		}
		else
			names[name].add(object);
	}
	
	private inline function removeName(name:String, object:GameObject):Void
	{
		if (names[name].length == 1)
			names[name] = null;
		else
			names[name].remove(object);
	}
}
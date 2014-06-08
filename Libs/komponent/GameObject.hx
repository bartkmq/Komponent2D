package komponent;

import komponent.components.Transform;
import komponent.components.Graphic;
import komponent.components.Collider;
import komponent.utils.Misc;

using komponent.utils.Parser;

@:final
class GameObject
{
	/**
	* Base class for all entities.
	* This class can't be extended.
	* To add functionality add components.
	* Every GameObject has a Transform Component.
	* 
	* Dispatches:
	* -> onComponentAdded(component) when a Component is added to this GameObject
	* -> onComponentRemoved(component) when a Component is removed from this GameObject
	* -> onActivated() when this GameObject starts to update/render (again)
	* -> onDeactivated() when this GameObject is disabled and won't update/render anymore till activated again
	*/
	
	public var transform(default, null):Transform;
	public var graphic(get, never):Graphic;
	public var collider(get, never):Collider;
	
	public var scene:Scene;
	public var active:Bool;
	public var name(default, null):String;
	
	private var components:Array<Component>;
	
	/**
	 * Constructor. Can be used to place the GameObject and assign a name.
	 * @param	name		Name to assign to the Entity.
	 * @param	x			X position to place the Entity.
	 * @param	y			Y position to place the Entity.
	 */
	public function new(name:String, x:Float = 0, y:Float = 0) 
	{
		this.name = name;
		active = true;
		components = [];
		Misc.engine.currentScene.add(this);
		
		transform = addComponent(Transform);
		transform.setPos(x, y);
	}
	
	public inline function update()
	{
		if (active)
		{
			for (component in components)
			{
				component.update();
			}
		}
	}
	
	public inline function render()
	{
		for (component in components)
			component.render();
	}
	
	public inline function debugDraw()
	{
		for (component in components)
			component.debugDraw();
	}
	
	/**
	 * Checks if the GameObject has a specific component type .
	 * @param	componentType	The type of component to search for.
	 * @return	True if a matching component was found, or false if none was found.
	 */
	public function hasComponent<T:Component>(componentType:Class<T>):Bool
	{	
		if (Std.is(transform, componentType))
			return cast transform;
		for (component in components)
		{
			if (Std.is(component, componentType))
				return true;
		}
		return false;
	}
	
	/**
	 * Gets a Component from a GameObject.
	 * @param	componentType	The type of component to search for.
	 * @return	The first Component that matches, or null if none was found.
	 */
	public function getComponent<T:Component>(componentType:Class<T>):T
	{
		if (Std.is(transform, componentType))
			return cast transform;
		for (component in components)
		{
			if (Std.is(component, componentType))
				return cast component;
		}
		return null;
	}
	
	/**
	 * Adds a Component to the GameObject.
	 * @param	component	Component type to add.
	 * @return	The added component instance or null.
	 */
	public inline function addComponent<T:Component>(componentType:Class<T>):T
	{	
		//if (!hasComponent(componentType)) TODO: Fix Subclasses
		//{
		var component = Type.createInstance(componentType, []);
		components.push(component);
		component.gameObject = this;
		component.added();
		sendMessage("onComponentAdded", component);
		return component;
		//}
		//return null;
	}
	
	/**
	 * Removes a Component from the GameObject.
	 * @param	component	Component instance to remove.
	 * @return	The added component.
	 */
	public inline function removeComponent<T:Component>(component:T):T
	{
		for (localComponent in components)
		{
			if (localComponent == component)
			{
				components.remove(component);
				component.removed();
				sendMessage("onComponentRemoved", component);
				break;
			}
		}
		return component;
	}
	
	/**
	 * Calls a function on every component in the GameObject.
	 * @param	functionName	Name of the function to call.
	 * @param	message			Dynamic object to pass to the function.
	 */
	public inline function sendMessage(functionName:String, message:Dynamic = null):Void
	{		
		for (component in components)
			Misc.callFunction(component, functionName, message);
	}
		
	/**
	 * Converts the GameObject to a string.
	 * @return	String of the form: GameObject:Name[ComponentType, ComponentType, ...]
	 */
	public inline function toString():String
	{
		var string = 'GameObject:$name[';
		string += transform;
		for (i in 0...components.length)
		{
			string += ", ";
			string += components[i];
		}
		return string += "]";
	}
	
	@:access(komponent.components.Transform)
	public static inline function loadPrefab(data:Dynamic):GameObject
	{
		var name:String = data.name.parse("");
		var active:Null<Bool> = data.active.parse(true);
		
		var object:GameObject = Type.createEmptyInstance(GameObject);
		object.components = [];
		object.active = active;
		object.name = name;
		Misc.engine.currentScene.add(object);
		
		var tempComponents:Array<Component> = [];
		var tempComponentDatas:Array<Dynamic> = [];
		for (componentData in cast(data.components, Array<Dynamic>))
		{
			if (componentData.type == null) continue;
			var component:Component = null;
			try
			{
				component = Type.createInstance(Type.resolveClass(componentData.type), []);
			}
			catch (error:Dynamic)
			{
				trace("Error creating an instance of " + componentData.type);
				continue;
			}
			
			component.gameObject = object;
			object.components.push(component);
			if (Std.is(component, Transform))
			{
				component.added();
				component.loadConfig(componentData);
				object.transform = cast component;
			}
			else
			{
				tempComponents.push(component);
				tempComponentDatas.push(componentData);
			}
		}
		
		if (!object.hasComponent(Transform))
			object.transform = new Transform();
		
		for (i in 0...tempComponents.length)
		{
			var tempComponent = tempComponents.pop();
			tempComponent.added();
			object.sendMessage("onComponentAdded", tempComponent);
			tempComponent.loadConfig(tempComponentDatas.pop());
		}
		
		return object;
	}
	
	private inline function set_active(value:Bool):Bool
	{
		if (value != active)
		{
			active = value;
			if (value)
				sendMessage("onActivated");
			else
				sendMessage("onDeactivated");
		}
		return value;
	}
	
	private inline function get_graphic():Graphic { return getComponent(Graphic); }
	
	private inline function get_collider():Collider { return getComponent(Collider); }
}
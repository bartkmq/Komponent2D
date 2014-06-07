package komponent.utils;

import kha.Key;
import kha.Button;

using komponent.utils.Parser;

@:allow(komponent.Engine)
class Input //TODO: Fix pressed()
{
	
	/**
	 * Contains the string of the last keys pressed
	 */
	public static var keyString:String = "";

	/**
	 * If the left button mouse is held down
	 */
	public static var mouseDown:Bool;
	/**
	 * If the left button mouse is up
	 */
	public static var mouseUp:Bool;
	/**
	 * If the left button mouse was recently pressed
	 */
	public static var mousePressed:Bool;
	/**
	 * If the left button mouse was recently released
	 */
	public static var mouseReleased:Bool;

	/**
	 * If the right button mouse is held down
	 */
	public static var rightMouseDown:Bool;
	/**
	 * If the right button mouse is up
	 */
	public static var rightMouseUp:Bool;
	/**
	 * If the right button mouse was recently pressed
	 */
	public static var rightMousePressed:Bool;
	/**
	 * If the right button mouse was recently released
	 */
	public static var rightMouseReleased:Bool;

	/**
	 * If the middle button mouse is held down
	 */
	public static var middleMouseDown:Bool;
	/**
	 * If the middle button mouse is up
	 */
	public static var middleMouseUp:Bool;
	/**
	 * If the middle button mouse was recently pressed
	 */
	public static var middleMousePressed:Bool;
	/**
	 * If the middle button mouse was recently released
	 */
	public static var middleMouseReleased:Bool;

	/**
	 * If the mouse wheel has moved
	 */
	public static var mouseWheel:Bool;
	
	/**
	 * If the mouse wheel was moved this frame, this was the delta.
	 */
	public static var mouseWheelDelta:Int;

	/**
	 * X position of the mouse on the screen.
	 */
	public static var mouseX:Int;

	/**
	 * Y position of the mouse on the screen.
	 */
	public static var mouseY:Int;
	
	/**
	 * Defines a new input.
	 * @param	name		String to map the input to.
	 * @param	chars		The keys to use for the Input.
	 * @param	buttons		The buttons to use for the Input.
	 * @param	modifiers	The modifiers to use for the Input.
	 */
	public static inline function define(name:String, ?chars:Array<String> = null, ?buttons:Array<Button> = null, ?modifiers:Array<Key> = null)
	{
		_controls[name] = true;
		_controlsChars[name] = chars;
		_controlsButtons[name] = buttons;
		_controlsModifiers[name] = modifiers;
	}
	
	/**
	 * If the input or key is held down.
	 * @param	input		An input name to check for.
	 * @return	True or false.
	 */
	public static function check(input:String):Bool
	{
		// if Input doesn't exists return false
		if (_controls[input] != true)
			return false;
		
		var chars = _controlsChars[input];
		if (chars != null)
		{
			for (char in chars)
			{
				if (_pressedChars.indexOf(char) == -1)
					return false;
			}
		}
		
		var buttons = _controlsButtons[input];
		if (buttons != null)
		{
			for (button in buttons)
			{
				if (_pressedButtons.indexOf(button) == -1)
					return false;
			}
		}
		
		var modifiers = _controlsModifiers[input];
		if (modifiers != null)
		{
			for (modifier in modifiers)
			{
				if (_pressedModifiers.indexOf(modifier) == -1)
					return false;
			}
		}
		return true;
	}
	
	/**
	 * If the input or key was pressed this frame.
	 * @param	input		An input name or key to check for.
	 * @return	True or false.
	 */
	public static inline function pressed(input:String):Bool
	{
		// if Input doesn't exists return false
		if (_controls[input] != true)
			return false;
			
		var chars = _controlsChars[input];
		if (chars != null)
		{
			for (char in chars)
			{
				if (_pressedChars.indexOf(char) == -1)
					return false;
			}
		}
		
		var buttons = _controlsButtons[input];
		if (buttons != null)
		{
			for (button in buttons)
			{
				if (_pressedButtons.indexOf(button) == -1)
					return false;
			}
		}
		
		var modifiers = _controlsModifiers[input];
		if (modifiers != null)
		{
			for (modifier in modifiers)
			{
				if (_pressedModifiers.indexOf(modifier) == -1)
					return false;
			}
		}
		return true;
	}
	
	/**
	 * If the input or key was released this frame.
	 * @param	input		An input name or key to check for.
	 * @return	True or false.
	 */
	public static inline function released(input:String):Bool
	{
		// if Input doesn't exists return false
		if (_controls[input] != true)
			return false;
		
		var chars = _controlsChars[input];
		if (chars != null)
		{
			for (char in chars)
			{
				if (_releasedChars.indexOf(char) == -1)
					return false;
			}
		}
		
		var buttons = _controlsButtons[input];
		if (buttons != null)
		{
			for (button in buttons)
			{
				if (_pressedButtons.indexOf(button) == -1)
					return false;
			}
		}
		
		var modifiers = _controlsModifiers[input];
		if (modifiers != null)
		{
			for (modifier in modifiers)
			{
				if (_releasedModifiers.indexOf(modifier) == -1)
					return false;
			}
		}
		return true;
	}
	
	public static inline function update()
	{
		mousePressed = false;
		mouseReleased = false;
		
		rightMousePressed = false;
		rightMouseReleased = false;
		
		mouseWheel = false;
		mouseWheelDelta = 0;
		
		_releasedChars = [];
		_releasedButtons = [];
		_releasedModifiers = [];
	}
	
	public static inline function loadConfig(data:Dynamic)
	{
		var definitions:Array<Dynamic> = data.definitions;
		if (definitions != null)
		{
			for (definition in definitions)
			{
				var name:String = definition.name;
				if (name == null)
					trace("Input Definition is missing a name: " + definition);
				var chars:Array<String> = definition.chars;
				
				var buttons:Array<Button> = [];
				if (definition.buttons != null)
				{
					for (dynamicButton in cast(definition.buttons, Array<Dynamic>))
					{
						var button = dynamicButton.parseButton(null);
						if  (button != null)
							buttons.push(button);
					}
					if (buttons.length == 0) buttons = null;
				}
				
				var modifiers:Array<Key> = [];
				if (definition.modifiers != null)
				{
					for (dynamicModifier in cast(definition.modifiers, Array<Dynamic>))
					{
						var modifier = dynamicModifier.parseKey(null);
						if  (modifier != null)
							modifiers.push(modifier);
					}
					if (modifiers.length == 0) modifiers = null;
				}
					
				if (chars != null || buttons != null || modifiers != null)
					define(name, chars, buttons, modifiers);
			}
		}
	}
	
	private static inline function onButtonDown(button:Button):Void
	{
		if (_pressedButtons.indexOf(button) == -1)
			_pressedButtons.push(button);
		_releasedButtons.remove(button);
	}
	
	private static inline function onButtonUp(button:Button):Void
	{
		_pressedButtons.remove(button);
		_releasedButtons.push(button);
	}
	
	private static inline function onKeyDown(key:Key, char:String):Void
	{
		if (_pressedChars.indexOf(char) == -1)
			_pressedChars.push(char);
		_releasedChars.remove(char);
		
		_pressedModifiers.push(key);
		_releasedModifiers.remove(key);
	}
	private static inline function onKeyUp(key: Key,char:String):Void
	{
		_pressedChars.remove(char);
		_releasedChars.push(char);
		
		_pressedModifiers.remove(key);
		_releasedModifiers.push(key);
	}
	
	private static inline function onMouseDown(x:Int, y:Int):Void
	{
		mouseDown = true;
		mouseUp = false;
		mousePressed = true;
	}
	
	private static inline function onMouseUp(x:Int, y:Int):Void
	{
		mouseDown = false;
		mouseUp = true;
		mouseReleased = true;
	}
	
	private static inline function onRightMouseDown(x:Int, y:Int):Void
	{
		rightMouseDown = true;
		rightMouseUp = false;
		rightMousePressed = true;
	}
	
	private static inline function onRightMouseUp(x:Int, y:Int):Void
	{
		rightMouseDown = false;
		rightMouseUp = true;
		rightMouseReleased = true;
	}
	
	private static inline function onMouseMove(x:Int, y:Int):Void
	{
		mouseX = x;
		mouseY = y;
	}
	private static inline function onMouseWheel(delta:Int):Void
	{
		mouseWheelDelta = delta;
		mouseWheel = true;
	}
	
	// if the Input exists
	private static var _controls:Map<String, Bool> = new Map();
	
	// maps control name to keys/buttons
	private static var _controlsChars:Map<String, Array<String>> = new Map();
	private static var _controlsModifiers:Map<String, Array<Key>> = new Map();
	private static var _controlsButtons:Map<String, Array<Button>> = new Map();
	
	private static var _pressedChars:Array<String> = [];
	private static var _pressedModifiers:Array<Key> = [];
	private static var _pressedButtons:Array<Button> = [];
	
	private static var _releasedChars:Array<String> = [];
	private static var _releasedModifiers:Array<Key> = [];
	private static var _releasedButtons:Array<Button> = [];
}
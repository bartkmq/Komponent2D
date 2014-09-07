package komponent.input;

import kha.input.Mouse in KhaMouse;

import komponent.utils.Screen;

enum MouseButton
{
	LEFT;
	RIGHT;
	// if the wheel was pressed
	MIDDLE;
	// if the wheel was moved
	WHEEL;
}

@:allow(komponent.input.Input)
class Mouse
{
	/**
	 * If the left button mouse is held down
	 */
	public static var leftButtonDown:Bool;
	/**
	 * If the left button mouse is up
	 */
	public static var leftButtonUp:Bool;
	/**
	 * If the left button mouse was recently pressed
	 */
	public static var leftButtonPressed:Bool;
	/**
	 * If the left button mouse was recently released
	 */
	public static var leftButtonReleased:Bool;

	/**
	 * If the right button mouse is held down
	 */
	public static var rightButtonDown:Bool;
	/**
	 * If the right button mouse is up
	 */
	public static var rightButtonUp:Bool;
	/**
	 * If the right button mouse was recently pressed
	 */
	public static var rightButtonPressed:Bool;
	/**
	 * If the right button mouse was recently released
	 */
	public static var rightButtonReleased:Bool;

	/**
	 * If the middle button mouse is held down
	 */
	public static var middleButtonDown:Bool;
	/**
	 * If the middle button mouse is up
	 */
	public static var middleButtonUp:Bool;
	/**
	 * If the middle button mouse was recently pressed
	 */
	public static var middleButtonPressed:Bool;
	/**
	 * If the middle button mouse was recently released
	 */
	public static var middleButtonReleased:Bool;

	/**
	 * If the mouse wheel has moved
	 */
	public static var wheel:Bool;
	
	/**
	 * If the mouse wheel was moved this frame, this was the delta.
	 */
	public static var wheelDelta:Int;

	/**
	 * X position of the mouse on the screen.
	 */
	public static var x:Int;

	/**
	 * Y position of the mouse on the screen.
	 */
	public static var y:Int;
	
	/**
	 * The touch x-axis coord in the scene.
	 */
	public static var sceneX(get, never):Float;
	private static inline function get_sceneX():Float { return (x - Screen.camera.x - Screen.halfWidth) / Screen.camera.fullScaleX + Screen.halfWidth; }

	/**
	 * The touch y-axis coord in the scene.
	 */
	public static var sceneY(get, never):Float;
	private static inline function get_sceneY():Float { return (y - Screen.camera.x - Screen.halfHeight) / Screen.camera.fullScaleY + Screen.halfHeight; }
	
	public static function check(mouseButton:MouseButton):Bool
	{
		switch (mouseButton)
		{
			case LEFT: return leftButtonDown;
			case RIGHT: return rightButtonDown;
			case MIDDLE: return middleButtonDown;
			case WHEEL: return wheel;
		}
	}
	
	private static function init()
	{
		KhaMouse.get().notify(onMouseDown, onMouseUp, onMouseMove, onMouseWheel);
	}
	
	private static function update()
	{
		leftButtonPressed = false;
		leftButtonReleased = false;
		
		rightButtonPressed = false;
		rightButtonReleased = false;
		
		middleButtonPressed = false;
		middleButtonReleased = false;
		
		wheel = false;
		wheelDelta = 0;
	}
	
	private static inline function onMouseDown(button:Int, x:Int, y:Int):Void
	{
		if (button == 0)
		{
			leftButtonDown = true;
			leftButtonUp = false;
			leftButtonPressed = true;
		}
		else if (button == 1)
		{
			rightButtonDown = true;
			rightButtonUp = false;
			rightButtonPressed = true;
		}
	}
	private static inline function onMouseUp(button:Int, x:Int, y:Int):Void
	{	
		if (button == 0)
		{
			leftButtonDown = false;
			leftButtonUp = true;
			leftButtonReleased = true;
		}
		else if (button == 1)
		{
			rightButtonDown = false;
			rightButtonUp = true;
			rightButtonReleased = true;
		}
	}
	
	private static inline function onMouseMove(x:Int, y:Int):Void
	{
		Mouse.x = x;
		Mouse.y = y;
	}
	private static inline function onMouseWheel(delta:Int):Void
	{
		wheelDelta = delta;
		wheel = true;
	}
	
}
package komponent.input;

import kha.input.Gamepad in KhaGamepad;

import komponent.input.Gamepad;
import komponent.utils.Time;

enum ButtonStatus
{
	PRESSED;
	RELEASED;
	ON;
	OFF;
}

/**
 * Interface to acess Gamepad input.
 */
@:allow(komponent.Engine)
class Gamepad
{
	/**
	 * The ID of this gamepad.
	 */
	public var id:Int;
	
	/**
	 * A map of buttons and their states
	 */
	public var buttons:Map<Int, ButtonStatus>;
	/**
	 * Each axis contained in an array.
	 */
	public var axis(null, default):Array<Float>;
	
	/**
	 * If the gamepad should try to automatically try to reconnect.
	 */
	public var reconnect:Bool;

	/**
	 * Determines the gamepads's deadZone. Anything under this value will be considered 0 to prevent jitter.
	 */
	public static inline var deadZone:Float = 0.15;
	 
	private var khaGamepad:KhaGamepad;
	private static var GAMEPADS:Array<Gamepad> = [];

	/**
	 * Creates and initializes a new Gamepad.
	 */
	public function new(id:Int)
	{
		this.id = id;
		buttons = new Map();
		axis = new Array();
		reconnect = true;
		connected = true;
		GAMEPADS.push(this);
	}

	/**
	 * Updates all gamepads.
	 */
	private static function update()
	{	
		for (gamepad in GAMEPADS)
			gamepad.updateGamepad();
	}
	
	/**
	 * Updates the gamepads's state.
	 */
	public function updateGamepad()
	{
		if (reconnect && !connected)
		{
			connected = true;
		}
			
		for (button in buttons.keys())
		{
			switch (buttons[button])
			{
				case PRESSED:
					buttons.set(button, ON);
				case RELEASED:
					buttons.set(button, OFF);
				default:
			}
		}
	}

	/**
	 * If the gamepad button was pressed this frame.
	 * Omit argument to check for any button.
	 * @param  button The button index to check.
	 */
	public function pressed(?button:Int):Bool
	{
		if (button == null)
		{
			for (k in buttons.keys())
			{
				if (buttons.get(k) == PRESSED) return true;
			}
		}
		else if (buttons.exists(button))
		{
			return buttons.get(button) == PRESSED;
		}
		return false;
	}

	/**
	 * If the gamepad  button was released this frame.
	 * Omit argument to check for any button.
	 * @param  button The button index to check.
	 */
	public function released(?button:Int):Bool
	{
		if (button == null)
		{
			for (k in buttons.keys())
			{
				if (buttons.get(k) == RELEASED) return true;
			}
		}
		else if (buttons.exists(button))
		{
			return buttons.get(button) == RELEASED;
		}
		return false;
	}

	/**
	 * If the gamepad  button is held down.
	 * Omit argument to check for any button.
	 * @param  button The button index to check.
	 */
	public function check(?button:Int):Bool
	{
		if (button == null)
		{
			for (k in buttons.keys())
			{
				var b = buttons.get(k);
				if (b != OFF && b != RELEASED) return true;
			}
		}
		else if (buttons.exists(button))
		{
			var b = buttons.get(button);
			return b != OFF && b != RELEASED;
		}
		return false;
	}

	/**
	 * Returns the axis value (from 0 to 1)
	 * @param  a The axis index to retrieve starting at 0
	 */
	public inline function getAxis(a:Int):Float
	{
		if (a < 0 || a >= axis.length) return 0;
		else return (Math.abs(axis[a]) < deadZone) ? 0 : axis[a];
	}
	
	/**
	 * Returns the button value (from 0 to 1)
	 * @param  button The buttons id to retrieve starting at 0
	 */
	public inline function getButton(button:Int):Float
	{
		return 0;//buttons[button].getParameters()[0];
		// TODO: implement
	}
	
	private function onButtonDown(buttonID:Int, value:Float)
	{
		buttons[buttonID] = value > 0 ? PRESSED : RELEASED;
	}
	
	private function onAxiChange(axiID:Int, value:Float)
	{
		axis[axiID] = value;
	}

	/**
	 * If the gamepad is currently connected.
	 */
	public var connected(get, null):Bool;
	private function get_connected():Bool { return khaGamepad != null; }
	private function set_connected(value:Bool):Bool
	{
		if (value && khaGamepad == null)
		{
			khaGamepad = KhaGamepad.get(id);
			khaGamepad.notify(onAxiChange, onButtonDown);
		}
		else
		{
			khaGamepad.remove(onAxiChange, onButtonDown);
			khaGamepad = null;
		}
		return value;
	}
}

package komponent.components.graphic;

import komponent.utils.Random;
import komponent.utils.Screen;
import komponent.utils.Time;

using komponent.utils.Parser;

class Animation extends Image
{
	
	/**
	 * If the animation has stopped.
	 */
	public var complete:Bool;

	/**
	 * Animation speed factor, alter this to speed up/slow down all animations.
	 */
	public var rate:Float;
	
	override public function render()
	{	
		if (_anim != null && !complete)
		{
			_timer += _anim.frameRate * Time.elapsed * rate;
			if (_timer >= 1)
			{
				while (_timer >= 1)
				{
					_timer--;
					_index += reverse ? -1 : 1;
					
					if ((reverse && _index == -1) || (!reverse && _index == _anim.frameCount))
					{
						if (_anim.loop)
						{
							_index = reverse ? _anim.frameCount - 1 : 0;
						}
						else
						{
							_index = reverse ? 0 : _anim.frameCount - 1;
							complete = true;
							break;
						}
					}
				}
				_frame = Std.int(_anim.frames[_index]);
				
				var sx = _frameWidth * _frame;
				var sy = Std.int(sx / width) * _frameHeight;
				var sx = sx % width;
				if (flipX) sx = (Math.floor(width) - _frameWidth) - sx;
				if (flipY) sy = (Math.floor(height) - _frameHeight) - sy; // TODO: Test
				
				setSourceRectangle(sx, sy, _frameWidth, _frameHeight);
			}
		}
		super.render();
	}
	
	override public function loadConfig(data:Dynamic):Void
	{
		super.loadConfig(data);
		
		var file = data.file;
		var frameWidth:Null<Int> = data.frameWidth;
		var frameHeight:Null<Int> = data.frameHeight;
		if (file != null && frameWidth != null && frameHeight != null)
			loadSpritemap(file, frameWidth, frameHeight);
		
		var animations:Array<Dynamic> = data.animations;
		if (animations != null)
			for (animation in animations)
			{
				var name:String = animation.name;
				var frames:Array<Int> = animation.frames;
				var frameRate:Int = animation.frameRate.parse(0);
				var loop:Bool = animation.parse(false);
				if (name != null && frames != null && frames.length > 0)
					add(name, frames, frameRate, loop);
			}
		
		rate = data.rate.parse(1.0);
		var playAnim:String = data.play;
		if (playAnim != null)
			play(playAnim);
	}
	
	/**
	 * Loads a Spritemap.
	 * @param	source			Source image.
	 * @param	frameWidth		Frame width.
	 * @param	frameHeight		Frame height.
	 */
	public inline function loadSpritemap(image:String, frameWidth:Int, frameHeight:Int):Void
	{
		load(image);
		
		complete = true;
		rate = 1;
		_anims = new Map();
		_timer = _frame = 0;
		
		setSourceRectangle(0, 0, frameWidth, frameHeight);
		
		_frameWidth = frameWidth;
		_frameHeight = frameHeight;
		
		if (width % _frameWidth != 0 || height % _frameHeight != 0)
			trace("Source image width and height should be multiples of the frame width and height.");
		
		_columns = Math.ceil(width / _frameWidth);
		_rows = Math.ceil(height / _frameHeight);
		_frameCount = _columns * _rows;
	}
	
	/**
	 * Add an Animation.
	 * @param	name		Name of the animation.
	 * @param	frames		Array of frame indices to animate through.
	 * @param	frameRate	Animation speed (in frames per second, 0 defaults to assigned frame rate)
	 * @param	loop		If the animation should loop
	 * @return	A new AnimationData object for the animation.
	 */
	public inline function add(name:String, frames:Array<Int>, frameRate:Float = 0, loop:Bool = true):AnimationData
	{
		if (_anims.get(name) != null)
			throw "Cannot have multiple animations with the same name";

		for (i in 0...frames.length)
		{
			frames[i] %= _frameCount;
			if (frames[i] < 0) frames[i] += _frameCount;
		}
		var anim = new AnimationData(name, frames, frameRate, loop);
		_anims.set(name, anim);
		anim.parent = this;
		return anim;
	}
	
	/**
	 * Plays an animation previous defined by add().
	 * @param	name		Name of the animation to play.
	 * @param	reset		If the animation should force-restart if it is already playing.
	 * @param	reverse		If the animation should be played backward.
	 * @return	Anim object representing the played animation.
	 */
	public inline function play(name:String = "", reset:Bool = false, reverse:Bool = false):AnimationData
	{
		if (!reset && _anim != null && _anim.name == name)
		{
			return _anim;
		}

		if (!_anims.exists(name))
		{
			stop(reset);
			return null;
		}

		_anim = _anims.get(name);
		this.reverse = reverse;
		restart();

		return _anim;
	}
	
	/**
	 * Plays a new ad hoc animation.
	 * @param	frames		Array of frame indices to animate through.
	 * @param	frameRate	Animation speed (in frames per second, 0 defaults to assigned frame rate)
	 * @param	loop		If the animation should loop
	 * @param	reset		When the supplied frames are currently playing, should the animation be force-restarted
	 * @param	reverse		If the animation should be played backward.
	 * @return	Anim object representing the played animation.
	 */
	public inline function playFrames(frames:Array<Int>, frameRate:Float = 0, loop:Bool = true, reset:Bool = false, reverse:Bool = false):AnimationData
	{
		if(frames == null || frames.length == 0)
		{
			stop(reset);		
			return null;
		}

		if(reset == false && _anim != null && _anim.frames == frames)
			return _anim;

		return playAnimation(new AnimationData(null, frames, frameRate, loop), reset, reverse);
	}

	/**
	 * Plays or restarts the supplied Animation.
	 * @param	animation	The Animation object to play
	 * @param	reset		When the supplied animation is currently playing, should it be force-restarted
	 * @param	reverse		If the animation should be played backward.
	 * @return	Anim object representing the played animation.
	 */
 	public inline function playAnimation(anim:AnimationData, reset:Bool = false, reverse:Bool = false):AnimationData
	{
		if(anim == null)
			throw "No animation supplied";

		if(reset == false && _anim == anim)
			return anim;

		_anim = anim;
		this.reverse = reverse;
		restart();

		return anim;
	}

	/**
	 * Resets the animation to play from the beginning.
	 */
	public inline function restart()
	{
		_timer = _index = reverse ? _anim.frames.length - 1 : 0;
		_frame = _anim.frames[_index];
		complete = false;
	}
	
	/**
	 * Immediately stops the currently playing animation.
	 * @param	reset		If true, resets the animation to the first frame.
	 */
	public inline function stop(reset:Bool = false)
	{
		_anim = null;

		if(reset)
			_frame = _index = reverse ? _anim.frames.length - 1 : 0;

		complete = true;
	}
	
	/**
	 * Gets the frame index based on the column and row of the source image.
	 * @param	column		Frame column.
	 * @param	row			Frame row.
	 * @return	Frame index.
	 */
	public inline function getFrame(column:Int = 0, row:Int = 0):Int
	{
		return (row % _rows) * _columns + (column % _columns);
	}
	
	/**
	 * Sets the current display frame based on the column and row of the source image.
	 * When you set the frame, any animations playing will be stopped to force the frame.
	 * @param	column		Frame column.
	 * @param	row			Frame row.
	 */
	public inline function setFrame(column:Int = 0, row:Int = 0)
	{
		_anim = null;
		var frame:Int = getFrame(column, row);
		if (_frame == frame) return;
		_frame = frame;
	}
	
	/**
	 * Assigns the Spritemap to a random frame.
	 */
	public inline function randFrame()
	{
		frame = Random.int(0, _frameCount);
	}
	
	/**
	 * Sets the frame to the frame index of an animation.
	 * @param	name	Animation to draw the frame frame.
	 * @param	index	Index of the frame of the animation to set to.
	 */
	public inline function setAnimFrame(name:String, index:Int)
	{
		var frames:Array<Int> = _anims.get(name).frames;
		index = index % frames.length;
		if (index < 0) index += frames.length;
		frame = frames[index];
	}
	
	/**
	 * Sets the current frame index. When you set this, any
	 * animations playing will be stopped to force the frame.
	 */
	public var frame(get, set):Int;
	private inline function get_frame():Int { return _frame; }
	private inline function set_frame(value:Int):Int
	{
		_anim = null;
		value %= _frameCount;
		if (value < 0) value = _frameCount + value;
		if (_frame == value) return _frame;
		_frame = value;
		return _frame;
	}

	/**
	 * Current index of the playing animation.
	 */
	public var index(get, set):Int;
	private inline function get_index():Int { return _anim != null ? _index : 0; }
	private inline function set_index(value:Int):Int
	{
		if (_anim == null) return 0;
		value %= _anim.frameCount;
		if (_index == value) return _index;
		_index = value;
		_frame = _anim.frames[_index];
		return _index;
	}
	
	/**
	 * If the animation is played in reverse.
	 */
	public var reverse:Bool;

	/**
	 * The amount of frames in the Spritemap.
	 */
	public var frameCount(get, null):Int;
	private inline function get_frameCount():Int { return _frameCount; }

	/**
	 * Columns in the Spritemap.
	 */
	public var columns(get, null):Int;
	private inline function get_columns():Int { return _columns; }

	/**
	 * Rows in the Spritemap.
	 */
	public var rows(get, null):Int;
	private inline function get_rows():Int { return _rows; }
	
	/**
	 * The currently playing animation.
	 */
	public var currentAnim(get, null):String;
	private inline function get_currentAnim():String { return (_anim != null) ? _anim.name : ""; }
	
	private var _frameWidth:Int;
	private var _frameHeight:Int;
	private var _columns:Int;
	private var _rows:Int;
	private var _frameCount:Int;
	private var _anims:Map<String, AnimationData>;
	private var _anim:AnimationData;
	private var _index:Int;
	private var _frame:Int;
	private var _timer:Float;
}

/**
 * Template used by Animation to define animations. Don't create
 * these yourself, instead you can fetch them with Animation's add().
 */
class AnimationData
{
	/**
	 * Constructor.
	 * @param	name		Animation name.
	 * @param	frames		Array of frame indices to animate.
	 * @param	frameRate	Animation speed.
	 * @param	loop		If the animation should loop.
	 */
	public function new(name:String, frames:Array<Int>, frameRate:Float = 0, loop:Bool = true, parent:Animation = null)
	{
        this.name       = name;
        this.frames     = frames;
        this.frameRate  = (frameRate == 0 ? 60 : frameRate);
        this.loop       = loop;
        this.frameCount = frames.length;
        this.parent 	= parent;
	}

	/**
	 * Plays the animation.
	 * @param	reset		If the animation should force-restart if it is already playing.
	 */
	public inline function play(reset:Bool = false, reverse:Bool = false)
	{
		if(name == null)
			_parent.playAnimation(this, reset, reverse);
		else
			_parent.play(name, reset, reverse);
	}

	public var parent(null, set):Animation;
	private inline function set_parent(value:Animation):Animation {
		_parent = value;
		return _parent;
	}

	/**
	 * Name of the animation.
	 */
	public var name(default, null):String;

	/**
	 * Array of frame indices to animate.
	 */
	public var frames(default, null):Array<Int>;

	/**
	 * Animation speed.
	 */
	public var frameRate(default, null):Float;

	/**
	 * Amount of frames in the animation.
	 */
	public var frameCount(default, null):Int;

	/**
	 * If the animation loops.
	 */
	public var loop(default, null):Bool;

	private var _parent:Animation;
}
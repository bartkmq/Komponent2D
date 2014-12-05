package komponent;

import yaml.Parser;
import yaml.Yaml;

import kha.Sys;
import kha.Image;
import kha.Configuration;
import kha.Game;
import kha.Loader;
import kha.Scaler;
import kha.LoadingScreen;
import kha.Scheduler;
import kha.Framebuffer;

#if debug
import CompileTime;
#end

import komponent.utils.Misc;
import komponent.utils.Time;
import komponent.utils.Painter;
import komponent.utils.Screen;
import komponent.input.Input;

using komponent.utils.Misc;

@:keep
class Engine extends Game
{
	
	public var currentScene(default, set):Scene;
	
	public var fps(get, never):Float;
	public var paused:Bool;
	
	// Debug draw
	public var debug:Bool;
	
	// Config
	public var config:Dynamic;
	
	public var backbuffer:Image;
	
	/**
	 * Constructor. Can be used to set the name of this game and to load Rooms.
	 * @param	name			Name to assign to the game.
	 * @param	loadRooms		Rooms that should be loaded before the engine starts
	 * @param	scene			Scene that will be created. Should have no constructor arguments. Default: Scene.
	 * @param	loadingScreen	Loading Screen that is displayed while loading the rooms. Should have no constructor arguments. Default: LoadingScreen.
	 */
	public function new(name:String = "", loadRooms:Array<String> = null, scene:Class<Scene> = null, loadingScreen:Class<Game> = null)
	{
		super(name);
		_loadRooms = loadRooms;
		paused = false;
		
		#if debug
		debug = true;
		#else
		debug = false;
		#end
		
		if (scene == null)
			_startScene = Scene;
		else
			_startScene = scene;
		
		if (loadingScreen == null)
			_loadingScreen = LoadingScreen;
		else
			_loadingScreen = loadingScreen;
			
		#if debug
		CompileTime.importPackage("komponent");
		#end
	}
	
	override public function init():Void
	{	
		Configuration.setScreen(Type.createInstance(_loadingScreen, []));
		_loadingScreen = null;

		#if debug
		Time.start("loading");
		#end
		for (room in _loadRooms)
			Loader.the.loadRoom(room, roomLoaded);
	}
	
	private inline function roomLoaded():Void
	{
		_loadRooms.pop();
		
		if (_loadRooms.length == 0)
		{
			_loadRooms = null;
			#if debug
			trace('Rooms loaded in ${Misc.round(Time.stop("loading"), 2)}ms');
			#end
			initEngine();
		}
	}
	
	private function initEngine():Void
	{
		Misc.engine = this;
		
		// init Input
		Input.init();
		
		if (_config != null)
		{
			config = Yaml.parse(Loader.the.getBlob(_config).toString(), Parser.options().useObjects());
			currentScene = Type.createInstance(config.Engine.scene.type.loadClass(_startScene), []);
			currentScene.loadPrefab(config.Engine.scene);
			
			name = config.Engine.name.loadDefault("");
			paused = config.Engine.paused.loadDefault(false);
			debug = config.Engine.debug.loadDefault(#if debug true #else false #end);
		}
		else
		{
			currentScene = Type.createInstance(_startScene, []);
		}		
		_startScene = null;
		
		// create backbuffer for rendering and init Painter
		backbuffer = Image.createRenderTarget(width, height);
		Painter.backbuffer = backbuffer;
		
		Configuration.setScreen(this);
	}
	
	override public function update():Void
	{
		Time.start("updating");
		
		currentScene.update();
		Input.update();
		Time.frames++;
		
		Time.stop("updating");
	}
	
	override public function render(framebuffer:Framebuffer):Void
	{
		Time.start("rendering");
		Painter.set(Screen.color, 1);
		Painter.g2.begin();
		currentScene.render();
		Painter.g2.end();
		
		startRender(framebuffer);
		Scaler.scale(backbuffer, framebuffer, Sys.screenRotation);
		endRender(framebuffer);
		
		Time.stop("rendering");
	}
	
	override public function onClose():Void
	{
		currentScene.quit();
	}
	
	public static function fromConfig(configfile:String, loadRooms:Array<String> = null, loadingScreen:Class<Game> = null):Engine
	{
		var engine:Engine = new Engine("", loadRooms, null, loadingScreen);
		engine._config = configfile;
		return engine;
	}
	
	private inline function set_currentScene(value:Scene):Scene
	{
		if (currentScene != null)
		{
			currentScene.end();
		}
		if (currentScene == value)
			return value;
			
		currentScene = value;
		currentScene.engine = this;
		currentScene.begin();
		return value;
	}
	
	private inline function get_fps():Float { return 1 / Time.elapsed; }
	
	private var _startScene:Class<Scene>;
	private var _loadRooms:Array<String>;
	private var _loadingScreen:Class<Game>;
	
	private var _config:String;
}
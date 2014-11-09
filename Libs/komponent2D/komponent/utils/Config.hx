package komponent.utils;

import kha.Loader;

import yaml.Yaml;
import yaml.Parser.ParserOptions;
import yaml.Renderer.RenderOptions;

class Config
{
	
	public static function load(filename:String, strict:Bool = false, validation:Bool = true):Dynamic
	{
		if (filename == null || filename.length == 0)
		{
			trace("Filename must contain atleast one character.");
			return null;
		}
		else
		{
			var blob = Loader.the.getBlob(filename).toString();
			
			var parserOptions = new ParserOptions();
			parserOptions.useObjects();
			parserOptions.strict = strict;
			parserOptions.validation = validation;
			return Yaml.parse(blob, parserOptions);
		}
	}
	
	public static function render(config:Dynamic):String
	{
		return Yaml.render(config);
	}
	
}
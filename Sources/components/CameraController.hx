package components;

import komponent.Component;
import komponent.utils.Screen;
import komponent.input.Input;

import scene.MatrixTest;

class CameraController extends Component
{
	
	var movementSpeed:Float = 10;
	var rotatationSpeed:Float = 3;
	var scalingSpeed:Float = 0.1;

	override public function update()
	{
		if (!cast(scene, MatrixTest).shipControl)
		{
			var camera = Screen.camera;
			transform.localX += Input.getAxis("horizontal") * movementSpeed;
			transform.localY += Input.getAxis("vertical") * movementSpeed;
			transform.localRotation += Input.getAxis("rotation") * rotatationSpeed;
			camera.zoom += Input.getAxis("zoom") * scalingSpeed;
		}
	}
	
}
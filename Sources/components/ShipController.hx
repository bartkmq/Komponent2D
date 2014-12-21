package components;

import komponent.Component;
import komponent.components.graphic.Image;
import komponent.input.Input;

import scene.MatrixTest;

class ShipController extends Component
{
	
	var movementSpeed:Float = 10;
	var rotatationSpeed:Float = 3;
	var scalingSpeed:Float = 0.1;
	
	override public function added()
	{
		var image = addComponent(Image);
		image.load("playerShip1_red");
	}
	
	override public function update()
	{
		if (cast(scene, MatrixTest).shipControl)
		{
			transform.localX += Input.getAxis("horizontal") * movementSpeed;
			transform.localY += Input.getAxis("vertical") * movementSpeed;
			transform.localRotation += Input.getAxis("rotation") * rotatationSpeed;
			transform.localScaleX += Input.getAxis("zoom") * scalingSpeed;
			transform.localScaleY += Input.getAxis("zoom") * scalingSpeed;
		}
	}
	
}
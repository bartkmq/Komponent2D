package komponent.ds;

using komponent.utils.Misc;

/**
 * A Signal<M> can bind one or more callback functions that will be
 * called with messages of type M.
 * Typically M is some type of event that will be passed to the callback.
 */ 
class Signal<M>
{
	public var callbacks:Array<(M->Void)>;

	public function new()
	{
		callbacks = [];
	}

	public function add(callback:M->Void)
	{
		// add callback if it's not already added
		if (callbacks.indexOf(callback) == -1)
		{
			callbacks.push(callback);
		}
	}

	public function remove(callback:M->Void)
	{
		callbacks.remove(callback);
	}

	public function clear()
	{
		callbacks.clear();
	}

	public function dispatch(message:M)
	{
		for (callback in callbacks) callback(message);
	}
}

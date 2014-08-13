package komponent.ds;

using komponent.utils.Misc;

/**
 * A Signal<M, R> can bind one or more callback functions that will be
 * called with messages of type M and return type R.
 *
 * Typically M is some type of event that will be passed to the callback.
 * R can be Void or some value that needs to be returned to the handler
 * (e.g. a Bool value specifying whether a collision took place.)
 */ 
class Signal<M, R>
{
	public var callbacks:Array<(M->R)>;

	public function new()
	{
		callbacks = [];
	}

	public function add(callback:M->R)
	{
		// add callback if it's not already added
		if (callbacks.indexOf(callback) == -1)
		{
			callbacks.push(callback);
		}
	}

	public function remove(callback:M->R)
	{
		callbacks.remove(callback);
	}

	public function clear()
	{
		callbacks.clear();
	}

	public function dispatch(message:M):Array<R>
	{
		return [for (callback in callbacks) callback(message)];
	}
}

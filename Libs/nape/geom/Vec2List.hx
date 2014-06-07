package nape.geom;
import zpp_nape.Const;
import zpp_nape.constraint.PivotJoint;
import zpp_nape.ID;
import zpp_nape.constraint.Constraint;
import zpp_nape.constraint.WeldJoint;
import zpp_nape.constraint.UserConstraint;
import zpp_nape.constraint.LineJoint;
import zpp_nape.constraint.DistanceJoint;
import zpp_nape.constraint.LinearJoint;
import zpp_nape.constraint.MotorJoint;
import zpp_nape.constraint.AngleJoint;
import zpp_nape.phys.Interactor;
import zpp_nape.phys.FeatureMix;
import zpp_nape.phys.Material;
import zpp_nape.constraint.PulleyJoint;
import zpp_nape.phys.FluidProperties;
import zpp_nape.phys.Compound;
import zpp_nape.callbacks.OptionType;
import zpp_nape.phys.Body;
import zpp_nape.callbacks.CbSetPair;
import zpp_nape.callbacks.CbType;
import zpp_nape.callbacks.Callback;
import zpp_nape.callbacks.CbSet;
import zpp_nape.callbacks.Listener;
import zpp_nape.geom.GeomPoly;
import zpp_nape.geom.Mat23;
import zpp_nape.geom.ConvexRayResult;
import zpp_nape.geom.Cutter;
import zpp_nape.geom.Vec2;
import zpp_nape.geom.Ray;
import zpp_nape.geom.Convex;
import zpp_nape.geom.MatMath;
import zpp_nape.geom.Triangular;
import zpp_nape.geom.PartitionedPoly;
import zpp_nape.geom.Simplify;
import zpp_nape.geom.AABB;
import zpp_nape.geom.SweepDistance;
import zpp_nape.geom.Simple;
import zpp_nape.geom.VecMath;
import zpp_nape.geom.Monotone;
import zpp_nape.geom.Vec3;
import zpp_nape.geom.MatMN;
import zpp_nape.geom.PolyIter;
import zpp_nape.geom.MarchingSquares;
import zpp_nape.geom.Geom;
import zpp_nape.shape.Circle;
import zpp_nape.geom.Collide;
import zpp_nape.shape.Shape;
import zpp_nape.shape.Edge;
import zpp_nape.space.Broadphase;
import zpp_nape.shape.Polygon;
import zpp_nape.space.SweepPhase;
import zpp_nape.space.DynAABBPhase;
import zpp_nape.dynamics.Contact;
import zpp_nape.space.Space;
import zpp_nape.dynamics.InteractionGroup;
import zpp_nape.dynamics.InteractionFilter;
import zpp_nape.dynamics.SpaceArbiterList;
import zpp_nape.dynamics.Arbiter;
import zpp_nape.util.Array2;
import zpp_nape.util.Lists;
import zpp_nape.util.Flags;
import zpp_nape.util.Queue;
import zpp_nape.util.Debug;
import zpp_nape.util.RBTree;
import zpp_nape.util.FastHash;
import zpp_nape.util.WrapLists;
import zpp_nape.util.Pool;
import zpp_nape.util.Names;
import zpp_nape.util.Math;
import zpp_nape.util.UserData;
import zpp_nape.util.DisjointSetForest;
import nape.TArray;
import nape.Config;
import zpp_nape.util.Circular;
import nape.constraint.PivotJoint;
import nape.constraint.WeldJoint;
import nape.constraint.UserConstraint;
import nape.constraint.Constraint;
import nape.constraint.DistanceJoint;
import nape.constraint.LineJoint;
import nape.constraint.LinearJoint;
import nape.constraint.ConstraintList;
import nape.constraint.AngleJoint;
import nape.constraint.MotorJoint;
import nape.constraint.ConstraintIterator;
import nape.phys.GravMassMode;
import nape.phys.BodyList;
import nape.phys.Interactor;
import nape.phys.InertiaMode;
import nape.phys.InteractorList;
import nape.constraint.PulleyJoint;
import nape.phys.MassMode;
import nape.phys.Material;
import nape.phys.InteractorIterator;
import nape.phys.FluidProperties;
import nape.phys.BodyIterator;
import nape.phys.Compound;
import nape.phys.CompoundList;
import nape.phys.BodyType;
import nape.phys.CompoundIterator;
import nape.callbacks.InteractionListener;
import nape.callbacks.OptionType;
import nape.callbacks.PreListener;
import nape.callbacks.BodyListener;
import nape.callbacks.ListenerIterator;
import nape.callbacks.CbType;
import nape.callbacks.ListenerType;
import nape.callbacks.PreFlag;
import nape.callbacks.CbEvent;
import nape.callbacks.InteractionType;
import nape.callbacks.PreCallback;
import nape.callbacks.InteractionCallback;
import nape.phys.Body;
import nape.callbacks.ListenerList;
import nape.callbacks.BodyCallback;
import nape.callbacks.CbTypeList;
import nape.callbacks.Callback;
import nape.callbacks.ConstraintListener;
import nape.callbacks.CbTypeIterator;
import nape.callbacks.ConstraintCallback;
import nape.callbacks.Listener;
import nape.geom.Mat23;
import nape.geom.ConvexResultIterator;
import nape.geom.GeomPoly;
import nape.geom.Ray;
import nape.geom.GeomPolyIterator;
import nape.geom.Vec2Iterator;
import nape.geom.RayResult;
import nape.geom.Winding;
import nape.geom.RayResultIterator;
import nape.geom.AABB;
import nape.geom.IsoFunction;
import nape.geom.GeomVertexIterator;
import nape.geom.ConvexResult;
import nape.geom.GeomPolyList;
import nape.geom.Vec2;
import nape.geom.RayResultList;
import nape.geom.Vec3;
import nape.geom.MatMN;
import nape.geom.MarchingSquares;
import nape.geom.ConvexResultList;
import nape.shape.Circle;
import nape.shape.ValidationResult;
import nape.geom.Geom;
import nape.shape.ShapeIterator;
import nape.shape.Polygon;
import nape.shape.Edge;
import nape.shape.EdgeList;
import nape.shape.Shape;
import nape.shape.EdgeIterator;
import nape.shape.ShapeList;
import nape.shape.ShapeType;
import nape.space.Broadphase;
import nape.dynamics.Contact;
import nape.dynamics.InteractionGroupList;
import nape.dynamics.Arbiter;
import nape.dynamics.InteractionGroup;
import nape.dynamics.ContactIterator;
import nape.dynamics.InteractionFilter;
import nape.dynamics.ArbiterList;
import nape.space.Space;
import nape.dynamics.ArbiterIterator;
import nape.dynamics.InteractionGroupIterator;
import nape.dynamics.FluidArbiter;
import nape.dynamics.ContactList;
import nape.dynamics.ArbiterType;
import nape.dynamics.CollisionArbiter;
import nape.util.Debug;
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Nape list of Vec2 type objects
 * <br/><br/>
 * Internally this list is at present implemented as a linked list with
 * object pooled nodes and iterators with various fast paths made for
 * standard access patterns (For instance accessing successive elements
 * runs in constant time when using random access functions)
 * <br/><br/>
 * Iteration of this list can be done in various ways, but the preferred
 * way on all targets, is through use of the foreach function:
 * <pre>
 * list.foreach(function (obj) {
 * });
 * </pre>
 * This method is inlined so that in haxe no closure will need to be created.
 * <br/><br/>
 * In AS3, a closure would need to be created in general, so for performance
 * reasons you 'may' choose to use iteration as follows:
 * <pre>
 * for (var i:int = 0; i < list.length; i++) {
 *     var obj:Vec2 = list.at(i);
 * }
 * </pre>
 * <br/>
 * NOTE: It is 'not' safe to modify a list whilst iterating over it.
 * If you wish to remove elements during an iteration you should use the
 * filter method, for example:
 * <pre>
 * list.filter(function (obj) {
 *     // operate on object.
 *     // ...
 *     return (false if you want object to be removed);
 * });
 * </pre>
 * <br/><br/>
 * In AS3, if you wish to avoid a closure generation, you can perform such
 * an iteration in a safe manner as follows:
 * <pre>
 * var i:int = 0;
 * while (i < list.length) {
 *     var obj:Vec2 = list.at(i);
 *     // operate on object.
 *     // ...
 *     if (should remove obj) {
 *         list.remove(obj);
 *         continue;
 *     }
 *     else i++;
 * }
 * </pre>
 * Or if you are always clearing the list entirely you could write:
 * <pre>
 * while (!list.empty()) {
 *     var obj:Vec2 = list.pop();
 *     // operate on object.
 *     // ...
 * }
 * </pre>
 */
#if!true@:final #end
#if nape_swc@:keep #end
class Vec2List{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Vec2List=null;
    /**
     * Length of list.
     */
    #if nape_swc@:isVar #end
    public var length(get_length,never):Int;
    #if false inline function get_length(){
        zpp_inner.valmod();
        if(zpp_inner.zip_length){
            zpp_inner.zip_length=false;
            if(false){
                zpp_inner.user_length=0;
                {
                    var cx_ite=zpp_inner.inner.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        if(true)zpp_inner.user_length++;
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else zpp_inner.user_length=zpp_inner.inner.length;
        }
        return zpp_inner.user_length;
    }
    #else inline function get_length(){
        return zpp_gl();
    }
    /**
     * @private
     */
    public function zpp_gl(){
        zpp_inner.valmod();
        if(zpp_inner.zip_length){
            zpp_inner.zip_length=false;
            if(false){
                zpp_inner.user_length=0;
                {
                    var cx_ite=zpp_inner.inner.begin();
                    while(cx_ite!=null){
                        var i=cx_ite.elem();
                        if(true)zpp_inner.user_length++;
                        cx_ite=cx_ite.next;
                    }
                };
            }
            else zpp_inner.user_length=zpp_inner.inner.length;
        }
        return zpp_inner.user_length;
    }
    /**
     * @private
     */
    public function zpp_vm(){
        zpp_inner.valmod();
    }
    #end
    /**
     * Convert standard Array to Nape list.
     *
     * @param array The array to be converted
     * @return An equivalent Nape list.
     * @throws If array argument is null.
     * @throws If array contains elements of type other than Vec2
     */
    #if nape_swc@:keep #end
    public static function fromArray(array:Array<Vec2>){
        #if(!NAPE_RELEASE_BUILD)
        if(array==null){
            throw "Error: Cannot convert null Array to Nape list";
        }
        #end
        var ret=new Vec2List();
        for(i in array){
            #if flash9#if(!NAPE_RELEASE_BUILD)
            if(!#if flash untyped __is__(i,Vec2)#else Std.is(i,Vec2)#end)throw "Error: Array contains non "+"Vec2"+" types.";
            #end
            #end
            ret.push(i);
        }
        return ret;
    }
    #if flash9 /**
     * Convert flash.Vector to Nape list.
     *
     * @param vector The vector to be converted
     * @return An equivalent Nape list.
     * @throws # If vector argument is null.
     */
    #if nape_swc@:keep #end
    public static function fromVector(vector:flash.Vector<Vec2>){
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot convert null Vector to Nape list";
        }
        #end
        var ret=new Vec2List();
        for(i in vector)ret.push(i);
        return ret;
    }
    #end
    /**
     * Check if element is already in the list
     *
     * @param obj The object to test.
     * @return True if object is in the list.
     */
    #if nape_swc@:keep #end
    public function has(obj:Vec2):Bool{
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        return zpp_inner.inner.has(obj.zpp_inner);
    }
    /**
     * Random access to elements of list by index.
     * <br/><br/>
     * Under normal circumstances, accessing succesive elements via this
     * method will occur in constant time.
     *
     * @param index The index of the element in list to access.
     * @returns The element at the given index.
     * @throws # If index is out of bounds.
     */
    #if nape_swc@:keep #end
    public function at(index:Int):Vec2{
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(index<0||index>=length)throw "Error: Index out of bounds";
        #end
        if(zpp_inner.reverse_flag)index=(length-1-index);
        if(!false){
            if(index<zpp_inner.at_index||zpp_inner.at_ite==null){
                zpp_inner.at_index=index;
                zpp_inner.at_ite=zpp_inner.inner.iterator_at(index);
            }
            else{
                while(zpp_inner.at_index!=index){
                    zpp_inner.at_index++;
                    zpp_inner.at_ite=zpp_inner.at_ite.next;
                }
            }
        }
        else{
            if(index<zpp_inner.at_index||zpp_inner.at_ite==null){
                zpp_inner.at_index=0;
                zpp_inner.at_ite=zpp_inner.inner.begin();
                while(true){
                    var x=zpp_inner.at_ite.elem();
                    if(true)break;
                    zpp_inner.at_ite=zpp_inner.at_ite.next;
                }
            }
            while(zpp_inner.at_index!=index){
                zpp_inner.at_index++;
                zpp_inner.at_ite=zpp_inner.at_ite.next;
                while(true){
                    var x=zpp_inner.at_ite.elem();
                    if(true)break;
                    zpp_inner.at_ite=zpp_inner.at_ite.next;
                }
            }
        }
        return zpp_inner.at_ite.elem().wrapper();
    }
    /**
     * Push element to back of list.
     * <br/><br/>
     * When the order of objects is not important, it is best to use the
     * add() method instead.
     *
     * @param obj The object to insert.
     * @returns True if object was successively inserted.
     * @throws # If list is immutable.
     */
    #if nape_swc@:keep #end
    public function push(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        var cont=if(zpp_inner.adder!=null)zpp_inner.adder(obj)else true;
        if(cont){
            if(zpp_inner.reverse_flag)zpp_inner.inner.add(obj.zpp_inner);
            else{
                if(zpp_inner.push_ite==null)zpp_inner.push_ite=empty()?null:zpp_inner.inner.iterator_at(length-1);
                zpp_inner.push_ite=zpp_inner.inner.insert(zpp_inner.push_ite,obj.zpp_inner);
            }
            zpp_inner.invalidate();
            if(zpp_inner.post_adder!=null)zpp_inner.post_adder(obj);
        }
        return cont;
    }
    /**
     * Push element to front of list.
     * <br/><br/>
     * When the order of objects is not important, it is best to use the
     * add() method instead.
     *
     * @param obj The object to insert.
     * @returns True if object was successively inserted.
     * @throws # If list is immutable.
     */
    #if nape_swc@:keep #end
    public function unshift(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        var cont=if(zpp_inner.adder!=null)zpp_inner.adder(obj)else true;
        if(cont){
            if(zpp_inner.reverse_flag){
                if(zpp_inner.push_ite==null)zpp_inner.push_ite=empty()?null:zpp_inner.inner.iterator_at(length-1);
                zpp_inner.push_ite=zpp_inner.inner.insert(zpp_inner.push_ite,obj.zpp_inner);
            }
            else zpp_inner.inner.add(obj.zpp_inner);
            zpp_inner.invalidate();
            if(zpp_inner.post_adder!=null)zpp_inner.post_adder(obj);
        }
        return cont;
    }
    /**
     * Pop element from back of list.
     * <br/><br/>
     * If you are wanting to clear a list, whilst operating on its elements,
     * consider use of the filter method instead.
     *
     * @returns The element removed from list.
     * @throws # If list is immutable.
     * @throws # If the list is empty.
     */
    #if nape_swc@:keep #end
    public function pop():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if(!NAPE_RELEASE_BUILD)
        if(empty())throw "Error: Cannot remove from empty list";
        #end
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        var ret=null;
        if(zpp_inner.reverse_flag){
            ret=zpp_inner.inner.front();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)zpp_inner.inner.pop();
        }
        else{
            if(zpp_inner.at_ite!=null&&zpp_inner.at_ite.next==null)zpp_inner.at_ite=null;
            var ite=length==1?null:zpp_inner.inner.iterator_at(length-2);
            ret=ite==null?zpp_inner.inner.front():ite.next.elem();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)zpp_inner.inner.erase(ite);
        }
        zpp_inner.invalidate();
        var retx=ret.wrapper();
        return retx;
    }
    /**
     * Pop element from front of list.
     * <br/><br/>
     * If you are wanting to clear a list, whilst operating on its elements,
     * consider use of the filter method instead.
     *
     * @returns The element removed from list.
     * @throws # If list is immutable.
     * @throws # If the list is empty.
     */
    #if nape_swc@:keep #end
    public function shift():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if(!NAPE_RELEASE_BUILD)
        if(empty())throw "Error: Cannot remove from empty list";
        #end
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        var ret=null;
        if(zpp_inner.reverse_flag){
            if(zpp_inner.at_ite!=null&&zpp_inner.at_ite.next==null)zpp_inner.at_ite=null;
            var ite=length==1?null:zpp_inner.inner.iterator_at(length-2);
            ret=ite==null?zpp_inner.inner.front():ite.next.elem();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)zpp_inner.inner.erase(ite);
        }
        else{
            ret=zpp_inner.inner.front();
            var retx=ret.wrapper();
            if(zpp_inner.subber!=null)zpp_inner.subber(retx);
            if(!zpp_inner.dontremove)zpp_inner.inner.pop();
        }
        zpp_inner.invalidate();
        var retx=ret.wrapper();
        return retx;
    }
    /**
     * Insert element into list in most effecient way.
     * <br/><br/>
     * This method will defer to either the push or unshift function
     * depending on which is most effecient in the context.
     * <br/><br/>
     * If order of elements is not important then you should always use
     * this function to insert elements.
     *
     * @param obj The object to insert.
     * @return True if object was successfuly inserted.
     * @throws # If list is immutable
     */
    #if nape_swc@:keep #end
    public  function add(obj:Vec2):Bool{
        return if(zpp_inner.reverse_flag)push(obj)else unshift(obj);
    }
    /**
     * Remove element from list.
     * <br/><br/>
     * This is a linear time operation.
     *
     * @param obj The object to remove
     * @return True if object was removed from list.
     * @throws # If list is immutable
     */
    #if nape_swc@:keep #end
    public function remove(obj:Vec2):Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        zpp_inner.modify_test();
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        var ret;
        {
            ret=false;
            {
                var cx_ite=zpp_inner.inner.begin();
                while(cx_ite!=null){
                    var x=cx_ite.elem();
                    {
                        if(x==obj.zpp_inner){
                            ret=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        if(ret){
            if(zpp_inner.subber!=null)zpp_inner.subber(obj);
            if(!zpp_inner.dontremove)zpp_inner.inner.remove(obj.zpp_inner);
            zpp_inner.invalidate();
        }
        return ret;
    }
    /**
     * Clear the list, removing all elements.
     *
     * @throws # If list is iummutable
     */
    #if nape_swc@:keep #end
    public function clear(){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable)throw "Error: "+"Vec2"+"List is immutable";
        #end
        if(zpp_inner.reverse_flag){
            while(!empty())pop();
        }
        else{
            while(!empty())shift();
        }
    }
    /**
     * Test if list is empty or not.
     *
     * @return True if list is empty.
     */
    #if nape_swc@:keep #end
    public  function empty(){
        #if false if(false)return length==0;
        else return zpp_inner.inner.empty();
        #else return length==0;
        #end
    }
    /**
     * Return Haxe iterator for list.
     * <br/><br/>
     * Use of this iterator, whilst stylistically better in Haxe should not
     * be used, in preference for use of the foreach function which will
     * not require allocation of an iterator object.
     * <br/><br/>
     * Equally in AS3, the foreach method should be the preferred way to iterate.
     */
    #if nape_swc@:keep #end
    public  function iterator(){
        #if false zpp_inner.valmod();
        #else zpp_vm();
        #end
        return Vec2Iterator.get(this);
    }
    /**
     * Produce a possibly deep copy of list.
     *
     * @param deep If true, then each element will have its own copy
     *             function called instead of simply having its
     *             reference copied over.
     * @return     The copied list.
     */
    #if nape_swc@:keep #end
    public function copy(deep:Bool=false){
        var ret=new Vec2List();
        for(i in this)ret.push(deep?i.copy():i);
        return ret;
    }
    /**
     * Merge given list into this one.
     * <br/><br/>
     * The result is that this list will have all objects from the argument
     * that were not already in the list inserted. You should make no
     * assumption about the order of these insertions.
     *
     * @param xs The list to merge.
     * @throws # If xs argument is null.
     */
    #if nape_swc@:keep #end
    public function merge(xs:Vec2List):Void{
        #if(!NAPE_RELEASE_BUILD)
        if(xs==null)throw "Error: Cannot merge with null list";
        #end
        for(x in xs){
            if(!has(x))add(x);
        }
    }
    /**
     * Construct a new list.
     */
    public function new(){
        zpp_inner=new ZPP_Vec2List();
        zpp_inner.outer=this;
    }
    /**
     * @private
     */
    @:keep public function toString(){
        var ret="[";
        var fst=true;
        for(i in this){
            if(!fst)ret+=",";
            ret+=(i==null?"NULL":i.toString());
            fst=false;
        }
        return ret+"]";
    }
    /**
     * Iterate over list applying function.
     * <br/><br/>
     * Any exception thrown by the supplied function will be treat as a
     * signal to halt iteration acting as a 'break' statement.
     * <br/><br/>
     * This method should be preferred to using standard haxe iteration
     * as there will be no allocation of an iterator object.
     * <pre>
     * list.foreach(function (obj) {
     *     if (ignore_object(obj)) return; //acts as a 'continue' statement
     *     if (halt_iteration(obj)) throw "": //acts as a 'break' statement
     * });
     * </pre>
     *
     * @param lambda The function to apply to each argument.
     * @return A reference to 'this' list.
     * @throws # If lambda argument is null.
     */
    #if nape_swc@:keep #end
    public  function foreach(lambda:Vec2->Void):Vec2List{
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: Cannot execute null on list elements";
        #end
        var it=iterator();
        while(it.hasNext()){
            try{
                lambda(it.next());
            }
            catch(e:Dynamic){
                {
                    it.zpp_next=Vec2Iterator.zpp_pool;
                    Vec2Iterator.zpp_pool=it;
                    it.zpp_inner=null;
                };
                break;
            }
        }
        return this;
    }
    /**
     * Iterate over list filtering elements.
     * <br/><br/>
     * The given function will be applied to each element, whenever the
     * function returns false, the element will be removed from the list.
     * <br/><br/>
     * Any exception thrown by the supplied function will be treat as a
     * signal to halt iteration acting as a 'break' statement.
     * <br/><br/>
     * This method is to be greatly preferred for filtering logic as
     * it is otherwise unsafe to modify the list during an iteration.
     * <br/><br/>
     * An example of using this method to clean up a list whilst performing
     * actions on the elements.
     * <pre>
     * list.filter(function (obj) {
     *    // perform clean up with obj
     *    return false; // remove from list.
     * });
     * </pre>
     *
     * @param lambda The function to apply to each argument, deciding if
     *               element should be removed.
     * @return A reference to 'this' list.
     * @throws # If lambda argument is null.
     */
    #if nape_swc@:keep #end
    public function filter(lambda:Vec2->Bool):Vec2List{
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: Cannot select elements of list with null";
        #end
        var i=0;
        while(i<length){
            var x=at(i);
            try{
                if(lambda(x))i++;
                else remove(x);
            }
            catch(e:Dynamic){
                break;
            }
        }
        return this;
    }
}

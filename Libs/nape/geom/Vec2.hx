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
import nape.geom.Vec2List;
import nape.geom.RayResultIterator;
import nape.geom.AABB;
import nape.geom.IsoFunction;
import nape.geom.GeomVertexIterator;
import nape.geom.ConvexResult;
import nape.geom.GeomPolyList;
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
 * 2 Dimensional vector.
 * <br/><br/>
 * Note that in many cases of a Vec2 object being returned by a Nape function
 * the Vec2 object will be marked internally as an 'immutable' Vec2. This will
 * always be documented and trying to mutate such a Vec2 will result in an
 * error being thrown.
 * <br/><br/>
 * Vec2 objects can make use of a global object pool, attempting to make use
 * of a disposed Vec2 will also result in an error with the object pool
 * working in a FILO order to increase the likelihood of such misuse being
 * caught.
 * <br/><br/>
 * Additionally Vec2 objects can be created as 'weak'. Passing a weak Vec2 to
 * any Nape function as an argument will result in the automatic disposal of
 * the Vec2 once the method has finished with it. There may be exceptions to
 * this rule which will also be documented; a notable case being the appending
 * of a weak Vec2 to a Nape Vec2List in which case the disposal of the weak
 * Vec2 is performed when that Vec2List is handed to a Nape function instead.
 * <br/><br/>
 * Example:
 * <pre>
 * var vertices = Polygon.box(20, 20, true);
 * var polygon = new Polygon(vertices);
 * </pre>
 * In this example, passing <code>true</code> to the Polygon.box method means
 * that we will be returned a Vec2List containing only 'weak' Vec2s. Upon
 * passing this Vec2List to the Polygon constructor, all of the Vec2s from
 * that list will be automatically disposed.
 */
@:final#if nape_swc@:keep #end
class Vec2{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Vec2=null;
    /**
     * @private
     */
    public var zpp_pool:Vec2=null;
    #if(!NAPE_RELEASE_BUILD)
    /**
     * @private
     */
    public var zpp_disp:Bool;
    #end
    #if NAPE_POOL_STATS 
    /**
     * @private
     */
    static public var POOL_CNT:Int=0;
    /**
     * @private
     */
    static public var POOL_TOT:Int=0;
    /**
     * @private
     */
    static public var POOL_ADD:Int=0;
    /**
     * @private
     */
    static public var POOL_ADDNEW:Int=0;
    /**
     * @private
     */
    static public var POOL_SUB:Int=0;
    #end
    
    /**
     * Allocate a weak Vec2 from global object pool.
     * <br/><br/>
     * This object which will be automaticaly released back to the object pool
     * when supplied as an argument to a Nape function.
     * <br/><br/>
     * Note that <code>Vec2.weak(x, y)</code> is exactly equivalent to <code>
     * Vec2.get(x, y, true)</code>.
     *
     * @param x The x coordinate for the vector. (default 0)
     * @param y The y coordiante for the vector. (default 0)
     * @return  The allocated weak Vec2 with given x/y values.
     */
    public static#if NAPE_NO_INLINE#else inline #end
    function weak(x:Float=0,y:Float=0):Vec2{
        return get(x,y,true);
    }
    /**
     * Allocates a Vec2 from the global object pool.
     * <br/><br/>
     * Note that <code>Vec2.get(x, y, true)</code> is exactly equivalent to
     * <code>Vec2.weak(x, y)</code> and should be preferred.
     *
     * @param x    The x coordinate for the vector. (default 0)
     * @param y    The y coordinate for the vector. (default 0)
     * @param weak If true, then a weak Vec2 will be allocated which will be
     *             automatically released to object pool when passed as an
     *             argument to a Nape function. (default false)
     * @return     The allocated, possibly weak Vec2 with given x/y values.
     */
    public static#if NAPE_NO_INLINE#else inline #end
    function get(x:Float=0,y:Float=0,weak:Bool=false):Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if((x!=x)||(y!=y)){
            throw "Error: Vec2 components cannot be NaN";
        }
        #end
        var ret;
        {
            if(ZPP_PubPool.poolVec2==null){
                ret=new Vec2();
                #if NAPE_POOL_STATS Vec2.POOL_TOT++;
                Vec2.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_PubPool.poolVec2;
                ZPP_PubPool.poolVec2=ret.zpp_pool;
                ret.zpp_pool=null;
                #if(!NAPE_RELEASE_BUILD)
                ret.zpp_disp=false;
                if(ret==ZPP_PubPool.nextVec2)ZPP_PubPool.nextVec2=null;
                #end
                #if NAPE_POOL_STATS Vec2.POOL_CNT--;
                Vec2.POOL_ADD++;
                #end
            }
        };
        if(ret.zpp_inner==null){
            ret.zpp_inner=ZPP_Vec2.get(x,y);
            ret.zpp_inner.outer=ret;
        }
        else{
            ret.setxy(x,y);
        }
        ret.zpp_inner.weak=weak;
        return ret;
    }
    /**
     * Release this Vec2 to global object pool.
     * <br/><br/>
     * Once diposed this Vec2
     * will be accessible to Nape internals for re-allocation and should
     * not be touched (Good practice would be to set any references to this
     * Vec2 to null to help ensure this).
     * <br/><br/>
     * In debug mode, should you attempt to access this Vec2 after disposal
     * and the Vec2 is still in the object pool, you will be given an Error.
     * The object pool operates on a First-In-Last-Out principal in debug
     * mode to help catch these sort of errors.
     * @throws # If this vector has already been disposed.
     * @throws # If this vector is immutable.
     * @throws # If this vector is in use in some other manner, such as being
     *           an element of a Polygon's vertex list.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function dispose():Void{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner._inuse){
            throw "Error: This Vec2 is not disposable";
        }
        #end
        var inner=zpp_inner;
        zpp_inner.outer=null;
        zpp_inner=null;
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("PublicFree(in T: "+"Vec2"+", in obj: "+"this"+")");
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_pool=null;
            if(ZPP_PubPool.nextVec2!=null)ZPP_PubPool.nextVec2.zpp_pool=o;
            else ZPP_PubPool.poolVec2=o;
            ZPP_PubPool.nextVec2=o;
            #end
            #if NAPE_RELEASE_BUILD 
            o.zpp_pool=ZPP_PubPool.poolVec2;
            ZPP_PubPool.poolVec2=o;
            #end
            #if NAPE_POOL_STATS Vec2.POOL_CNT++;
            Vec2.POOL_SUB++;
            #end
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_disp=true;
            #end
        };
        {
            var o=inner;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"inner"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
    }
    /**
     * Construct a new Vec2.
     * <br/><br/>
     * This constructor will obviously not make use of
     * the global object pool: <code>Vec2.get</code> should be used in
     * preference noting that <code>new Vec2(x, y)</code> is semantically
     * equivalent to <code>Vec2.get(x, y)</code>.
     *
     * @param x The x coordinate for the vector. (default 0)
     * @param y The y coordinate for the vector. (default 0)
     * @return  The newly constructed Vec2 object with given x/y values.
     */
    public function new(x:Float=0,y:Float=0){
        #if(!NAPE_RELEASE_BUILD)
        if((x!=x)||(y!=y)){
            throw "Error: Vec2 components cannot be NaN";
        }
        #end
        zpp_inner=ZPP_Vec2.get(x,y);
        zpp_inner.outer=this;
    }
    /**
     * Produce a copy of this Vec2.
     * <br/><br/>
     * The Vec2 will be allocated from the global object pool.
     * <br/><br/>
     * As would be expected, if you produce a copy of an 'immutable' Vec2, then
     * the copy will be 'mutable'.
     *
     * @param weak If true, then a weak Vec2 will be allocated which will be
     *             automatically released to the object pool when passed as an
     *             argument to any Nape function. (default false)
     * @return     The possibly weak copy of this Vec2.
     * @throws # If this vector has been disposed.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function copy(weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        return Vec2.get(x,y,weak);
    }
    #if(flash9||openfl||nme)
    /**
     * Allocate a Vec2 from AS3 Point object.
     * <br/><br/>
     * This Vec2 will be allocated from the global object pool.
     * <br/><br/>
     * This method is only available on <code>flash</code> and
     * <code>openfl||nme</code> targets.
     *
     * @param point The AS3 Point to initialise Vec2 with
     * @param weak  If true, then a weak Vec2 will be allocated which will
     *              be automatically released to the object pool when
     *              pass as an argument to any Nape function.
     *              (default false)
     * @return      The possibly weak Vec2 allocated with same values as
     *              input Point object.
     * @throws # If the point argument is null.
     */
    #if nape_swc@:keep #end
    public static function fromPoint(point:flash.geom.Point,weak:Bool=false):Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(point==null){
            throw "Error: Cannot create Vec2 from null Point object";
        }
        if((point.x!=point.x)||(point.y!=point.y)){
            throw "Error: Error: Vec2 components cannot be NaN";
        }
        #end
        return Vec2.get(point.x,point.y,weak);
    }
    /**
     * Create an AS3 Point object from Vec2.
     * <br/><br/>
     * This method is only available on <code>flash</code> and
     * <code>openfl||nme</code> targets.
     *
     * @param output If supplied, this Point will have its x/y
     *               set instead of creating a new Point.
     * @return    The AS3 Point object representing Vec2.
     * @throws # If this vector has been disposed of.
     */
    #if nape_swc@:keep #end
    public function toPoint(output:flash.geom.Point=null):flash.geom.Point{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        if(output==null)output=new flash.geom.Point();
        output.x=x;
        output.y=y;
        return output;
    }
    #end
    /**
     * Allocate a Vec2 given polar coordinates.
     * <br/><br/>
     * This Vec2 will be allocated from the global object pool.
     * <br/><br/>
     * This method will assign x/y values equal respectively to:
     * <code>length&#42Math.cos(angle)</code>,
     * <code>length&#42Math.sin(angle)</code>
     *
     * @param length The length of the Vec2. This value may be negative.
     * @param angle  The angle of the Vec2 as measured in radians clockwise
     *               from the positive x axis.
     * @param weak   If true, then a weak Vec2 will be allocated which will be
     *               automatically released to the object pool when passed as
     *               an argument to any Nape function. (default false)
     * @return       The possibly weak Vec2 allocated with given polar values.
     */
    #if nape_swc@:keep #end
    public static function fromPolar(length:Float,angle:Float,weak:Bool=false):Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if((length!=length)){
            throw "Error: Vec2::length cannot be NaN";
        }
        if((angle!=angle)){
            throw "Error: Vec2::angle cannot be NaN";
        }
        #end
        return Vec2.get(length*Math.cos(angle),length*Math.sin(angle),weak);
    }
    /**
     * x coordinate of vector.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var x(get_x,set_x):Float;
    inline function get_x():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.x;
    }
    inline function set_x(x:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            zpp_inner.immutable();
            if(this.x!=x){
                #if(!NAPE_RELEASE_BUILD)
                if((x!=x)){
                    throw "Error: Vec2::"+"x"+" cannot be NaN";
                }
                #end
                zpp_inner.x=x;
                zpp_inner.invalidate();
            }
        }
        return get_x();
    }
    /**
     * y coordinate of vector.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var y(get_y,set_y):Float;
    inline function get_y():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.y;
    }
    inline function set_y(y:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            zpp_inner.immutable();
            if(this.y!=y){
                #if(!NAPE_RELEASE_BUILD)
                if((y!=y)){
                    throw "Error: Vec2::"+"y"+" cannot be NaN";
                }
                #end
                zpp_inner.y=y;
                zpp_inner.invalidate();
            }
        }
        return get_y();
    }
    /**
     * Length of this Vec2.
     * <br/><br/>
     * This value can be set and may be set to negative values so that
     * <code>vec.length &#42= -1</code> is a valid - if sub-optimal - way of
     * negating a Vec2.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var length(get_length,set_length):Float;
    inline function get_length():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        return Math.sqrt((this.x*this.x+this.y*this.y));
    }
    inline function set_length(length:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            zpp_inner.immutable();
            #if(!NAPE_RELEASE_BUILD)
            if((length!=length)){
                throw "Error: Vec2::length cannot be NaN";
            }
            if((this.x*this.x+this.y*this.y)==0){
                throw "Error: Cannot set length of a zero vector";
            }
            #end
            {
                var t=(length/Math.sqrt((this.x*this.x+this.y*this.y)));
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"this."+",in s: "+"length/Math.sqrt((this.x*this.x+this.y*this.y))"+")");
                    #end
                };
                this.x*=t;
                this.y*=t;
            };
            zpp_inner.invalidate();
        }
        return get_length();
    }
    /**
     * Compute squared length of this Vec2.
     * <br/><br/>
     * This is exactly the same as performing <code>vec.length &#42
     * vec.length</code> except for being more effecient.
     *
     * @return The squared length of this Vec2.
     * @throws # If this vector has been disposed.
     */
    #if nape_swc@:keep #end
    public function lsq():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        return(this.x*this.x+this.y*this.y);
    }
    /**
     * Set values of this Vec2 to those of the argument.
     * <br/><br/>
     * Note that <code>vec.set(p)</code> is semantically equivalent to
     * <code>vec.setxy(p.x, p.y)</code>.
     *
     * @param vector The Vec2 to set the values of this Vec2 with.
     * @return       A reference to 'this' Vec2.
     * @throws # If this vector, or vector argument  has been disposed.
     * @throws # If this vector is immutable.
     * @throws # If the vector passed as argument is null.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function set(vector:Vec2):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot assign null Vec2";
        }
        #end
        var ret=setxy(vector.x,vector.y);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Compute square distance between two points.
     *
     * @param a The first point Vec2.
     * @param b The second point Vec2.
     * @return Squared distance between points.
     * @throws # If a, b are disposed of or null.
     */
    public static#if NAPE_NO_INLINE#else inline #end
    function dsq(a:Vec2,b:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(a!=null&&a.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(b!=null&&b.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(a==null||b==null)throw "Error: Cannot compute squared distance between null Vec2";
        #end
        var ret=ZPP_VecMath.vec_dsq(a.x,a.y,b.x,b.y);
        ({
            if(({
                a.zpp_inner.weak;
            })){
                a.dispose();
                true;
            }
            else{
                false;
            }
        });
        ({
            if(({
                b.zpp_inner.weak;
            })){
                b.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Compute distance between two points.
     *
     * @param a The first point Vec2.
     * @param b The second point Vec2.
     * @return Distance between points.
     * @throws # If a, b are disposed of or null.
     */
    public static#if NAPE_NO_INLINE#else inline #end
    function distance(a:Vec2,b:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(a!=null&&a.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(b!=null&&b.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(a==null||b==null)throw "Error: Cannot compute squared distance between null Vec2";
        #end
        var ret=Math.sqrt(ZPP_VecMath.vec_dsq(a.x,a.y,b.x,b.y));
        ({
            if(({
                a.zpp_inner.weak;
            })){
                a.dispose();
                true;
            }
            else{
                false;
            }
        });
        ({
            if(({
                b.zpp_inner.weak;
            })){
                b.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Set values of this Vec2 given pair of x/y values.
     *
     * @param x The x value to set this Vec2's x value to.
     * @param y The y value to set this Vec2's y value to.
     * @return  A reference to 'this' Vec2.
     * @throws # If this vector has been disposed.
     * @throws # If this vector is immutable.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function setxy(x:Float,y:Float):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if((x!=x)||(y!=y)){
            throw "Error: Vec2 components cannot be NaN";
        }
        #end
        if(!(this.x==x&&this.y==y)){
            {
                zpp_inner.x=x;
                zpp_inner.y=y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((zpp_inner.x!=zpp_inner.x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(zpp_inner.x)"+") :: "+("vec_set(in n: "+"zpp_inner."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((zpp_inner.y!=zpp_inner.y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(zpp_inner.y)"+") :: "+("vec_set(in n: "+"zpp_inner."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
            };
            zpp_inner.invalidate();
        }
        return this;
    }
    /**
     * Angle of this Vec2.
     *
     * Measured in radians as measured clockwise from the positive x axis.
     * The value will be given in the range -pi to pi.
     * <br/><br/>
     * If the x/y values of this Vec2 are both 0, then the angle value will
     * default to 0.
     * <br/><br/>
     * This value can also be set (to any value) so that <code>vec.angle +=
     * Math.PI</code> is a valid - if sub-optimial - way of negating a Vec2.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var angle(get_angle,set_angle):Float;
    inline function get_angle():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        if(x==y&&x==0){
            return 0.0;
        }
        else{
            return Math.atan2(y,x);
        }
    }
    inline function set_angle(angle:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            zpp_inner.immutable();
            #if(!NAPE_RELEASE_BUILD)
            if((angle!=angle)){
                throw "Error: Vec2::angle cannot be NaN";
            }
            #end
            var l=length;
            setxy(l*Math.cos(angle),l*Math.sin(angle));
        }
        return get_angle();
    }
    /**
     * Rotate Vec2 in-place by given number of radians..
     * <br/><br/>
     * Rotation performed in the clockwise direction.
     * <br/><br/>
     * The Vec2 will be mutated, with it's new x/y values being the result
     * of the rotation.
     *
     * @param angle The number of radians to rotate Vec2 by in the clockwise
     *              direction.
     * @return A reference to 'this' Vec2.
     * @throws # If this vector has been disposed.
     * @throws # If this vector is immutable.
     */
    #if nape_swc@:keep #end
    public function rotate(angle:Float):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if((angle!=angle)){
            throw "Error: Cannot rotate Vec2 by NaN";
        }
        #end
        if((angle%(Math.PI*2))!=0){
            var ax=Math.sin(angle);
            var ay=Math.cos(angle);
            {
                var t=(ay*zpp_inner.x-ax*zpp_inner.y);
                zpp_inner.y=(zpp_inner.x*ax+zpp_inner.y*ay);
                zpp_inner.x=t;
            };
            zpp_inner.invalidate();
        }
        return this;
    }
    /**
     * Reflect given Vec2 in plane whose normal is this Vec2.
     * <br/><br/>
     * @param vec The vector to be reflected.
     * @param weak If true, the returned Vec2 will be automatically released
     *             to object pool when used in another Nape function (default false)
     * @return The reflected Vec2.
     * @throws # If this vector or argument has been disposed.
     * @throws # If this vector has zero length.
     */
    #if nape_swc@:keep #end
    public function reflect(vec:Vec2,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vec!=null&&vec.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(this.length==0){
            throw "Error: Cannot reflect in zero vector";
        }
        #end
        var normal=this.copy(true).normalise();
        var ret=vec.sub(normal.muleq(2*normal.dot(vec)),weak);
        ({
            if(({
                vec.zpp_inner.weak;
            })){
                vec.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Normalise this vector.
     * <br/><br/>
     * Equivalent to setting the length of the vector to 1, and also to the
     * (less-optimal) <code>this.set(this.unit())</code>.
     *
     * @return A reference to 'this' vector.
     * @throws # If this vector has been disposed of or is immutable.
     * @throws # If length of this vector is 0.
     */
    #if nape_swc@:keep #end
    public function normalise():Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if(this.length==0){
            throw "Error: Cannot normalise vector of length 0";
        }
        #end
        {
            var d=(this.x*this.x+this.y*this.y);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    d!=0.0;
                };
                if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"this."+")");
                #end
            };
            var imag=1.0/Math.sqrt(d);
            {
                var t=(imag);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"this."+",in s: "+"imag"+")");
                    #end
                };
                this.x*=t;
                this.y*=t;
            };
        };
        zpp_inner.invalidate();
        return this;
    }
    /**
     * Return normalisation of this vector.
     *
     * @param weak If true then the allocated Vec2 will be automatically
     *             released to the global object pool when used as an argument
     *             to a Nape function. (default false)
     * @return A copy of this vector, normalised.
     * @throws # If this vector has been disposed of.
     * @throws # If length of this vector is 0.
     */
    #if nape_swc@:keep #end
    public function unit(weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(this.length==0){
            throw "Error: Cannot normalise vector of length 0";
        }
        #end
        var scale=1/ZPP_Math.sqrt((this.x*this.x+this.y*this.y));
        return Vec2.get(x*scale,y*scale,weak);
    }
    /**
     * Add another vector to this vector.
     * <br/><br/>
     * Returns a newly allocated vector so that
     * this vector is not modified.
     * <br/><br/>
     * @param vector The vector to add to this vector. This value can not be
     *               null
     * @param weak   If true then the returned vector will be automatically
     *               released to the global object pool when used as an
     *               argument to another Nape function. (default false)
     * @return       The possibly weak vector representing the sum of this
     *               and the input vector.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function add(vector:Vec2,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot add null vectors";
        }
        #end
        var ret=Vec2.get(x+vector.x,y+vector.y,weak);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Add a multiple of a vector to this vector.
     * <br/><br/>
     * This operation is equivalent to:
     * <pre>
     * this.add(vector.mul(scalar, true));
     * </pre>
     * <br/><br/>
     * Returns a newly allocated vector so that
     * this vector is not modified.
     * <br/><br/>
     * @param vector The vector to add to this vector. This value can not be
     *               null
     * @param scalar The scalar multiplier for the vector being added.
     * @param weak   If true then the returned vector will be automatically
     *               released to the global object pool when used as an
     *               argument to another Nape function. (default false)
     * @return       The possibly weak vector representing the sum of this
     *               and the input vector by scalar multiplier.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function addMul(vector:Vec2,scalar:Float,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot add null vectors";
        }
        #end
        var ret=Vec2.get(x+(vector.x*scalar),y+(vector.y*scalar),weak);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Subtract another vector from this vector.
     * <br/><br/>
     * Returns a newly allocated vector so that this vector is not mutated.
     *
     * @param vector The vector to subtract from this vector. This value can
     *               not be null
     * @param weak   If true then the returned vector will be automatically
     *               released to the global object pool when used as an
     *               argument to another Nape function. (default false)
     * @return       The possibly weak vector representing the subtraction of
     *               the input vector from this vector.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function sub(vector:Vec2,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot subtract null vectors";
        }
        #end
        var ret=Vec2.get(x-vector.x,y-vector.y,weak);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Multiply this vector with a number.
     * <br/><br/>
     * Returns a newly allocated vector so that this vector is not mutated.
     *
     * @param scalar The number to multiply this vector with.
     * @param weak   If true then the returned vector will be automatically
     *               released to the global object pool when used as an
     *               argument to another Nape function. (default false)
     * @return       The possibly weak vector representing the multiplication
     *               of this vector and the input number.
     * @throws # If this vector has been disposed.
     */
    #if nape_swc@:keep #end
    public function mul(scalar:Float,weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if((scalar!=scalar)){
            throw "Error: Cannot multiply with NaN";
        }
        #end
        return Vec2.get(x*scalar,y*scalar,weak);
    }
    /**
     * Add another vector into this vector.
     * <br/><br/>
     * This vector is mutated to be the result of the operation.
     * <br/><br/>
     * Semantically equivalent to (the less optimal)
     * <code>vec1.set(vec1.add(vec2))</code>
     *
     * @param vector The vector to add into this vector.
     * @return       A reference to this vector.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If this vector is immutable.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function addeq(vector:Vec2):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot add null vectors";
        }
        #end
        setxy(x+vector.x,y+vector.y);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Subtract another vector from this vector.
     * <br/><br/>
     * This vector is mutated to be the result of the operation.
     * <br/><br/>
     * Semantically equivalent to (the less optimal)
     * <code>vec1.set(vec1.sub(vec2))</code>
     *
     * @param vector The vector to subtract from this vector.
     * @return       A reference to this vector.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If this vector is immutable.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function subeq(vector:Vec2):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot subtract null vectors";
        }
        #end
        setxy(x-vector.x,y-vector.y);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Multiply this vector with a number.
     * <br/><br/>
     * This vector is mutated to be the result of the operation.
     * <br/><br/>
     * Semantically equivalent to (the less optimal)
     * <code>vec.set(vec.mul(scalar))</code>
     *
     * @param scalar The number to multiply this vector with.
     * @return       A reference to this vector.
     * @throws # If this vector has been disposed.
     * @throws # If this vector is immutable.
     */
    #if nape_swc@:keep #end
    public function muleq(scalar:Float):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.immutable();
        #if(!NAPE_RELEASE_BUILD)
        if((scalar!=scalar)){
            throw "Error: Cannot multiply with NaN";
        }
        #end
        return setxy(x*scalar,y*scalar);
    }
    /**
     * Dot product with another vector.
     * <br/><br/>
     * The dot product is equal to the product of the length of each
     * vector, multiplied by the cosine of the smallest angle between them.
     * <br/><br/>
     * If one of the vectors is of length 1. Then the dot product is the
     * length of the projection of the other vector onto it which may be
     * computed (assuming 'this' is of length 1) like:
     * <code>vec1.mul(vec1.dot(vec2))</code>
     *
     * @param vector The vector to compute dot product with.
     * @return       The dot product of this vector and the other.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function dot(vector:Vec2):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot take dot product with null vector";
        }
        #end
        var ret=(x*vector.x+y*vector.y);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Cross product with another vector.
     * <br/><br/>
     * Also known as the perp-dot product, this operation represents
     * the determinant of the matrix formed by having the 2 columns
     * as the two vectors. This is also the z-value of a 3D cross product
     * if you extend the input vectors with a z-value of 0.
     * <br/><br/>
     * Though not technically a cross-product in the way a 3D cross product
     * is, it shares many mathematical similarities.
     * <br/><br/>
     * If one of the vectors is of length 1. Then the cross product is the
     * length of the projection of the other vector onto the
     * right-perpendicular of the unit vector.
     * <br/><br/>
     * The cross and dot product are related like:
     * <code>vec1.cross(vec2) == vec1.perp().dot(vec2)</code>
     * Hence the name 'perp-dot'
     * <br/><br/>
     * Another useful property is that if the cross-product of two vectors
     * is 0, then the vectors are collinear, if positive then the second
     * vector is 'to the right' of the first vector, and if negative then
     * the second vector is 'to the left' of the first vector.
     *
     * @param vector The vector to compute cross product with.
     * @return       The cross product of this vector and the other.
     * @throws # If this vector, or the vector argument has been disposed.
     * @throws # If the vector passed as argument is null.
     */
    #if nape_swc@:keep #end
    public function cross(vector:Vec2):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot take cross product with null vector";
        }
        #end
        var ret=(vector.y*x-vector.x*y);
        ({
            if(({
                vector.zpp_inner.weak;
            })){
                vector.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * The right-perpendicular to this vector.
     * <br/><br/>
     * Computes the result of rotating this vector by 90 degrees clockwise
     * returning a newly allocated vector.
     * <br/><br/>
     * This is semantically equivalent to (the less optimal)
     * <code>vec.copy().rotate(Math.PI/2)</code>
     * <br/><br/>
     * The cross and dot product are related like:
     * <code>vec1.cross(vec2) == vec1.perp().dot(vec2)</code>
     * Hence the name 'perp-dot'
     *
     * @param weak If true then the returned vector will be automatically
     *             released to the global object pool when used as an argument
     *             to another Nape function. (default false)
     * @return     The possibly weakly allocated, right-perpendicular to this
     *             vector.
     * @throws # If this vector has been disposed.
     */
    #if nape_swc@:keep #end
    public function perp(weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        return Vec2.get(-y,x,weak);
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.toString();
    }
}

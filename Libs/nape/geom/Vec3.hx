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
import nape.geom.Vec2;
import nape.geom.RayResultList;
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
 * A 3 dimensional vector object.
 * <br/><br/>
 * In many instances a Vec3 will be accessible from Nape which is marked
 * as immutable, these cases will be documented and modifying such a Vec3
 * will result in an error.
 */
@:final#if nape_swc@:keep #end
class Vec3{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Vec3=null;
    /**
     * @private
     */
    public var zpp_pool:Vec3=null;
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
     * The x component of Vec3.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var x(get_x,set_x):Float;
    inline function get_x():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.x;
    }
    inline function set_x(x:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.immutable){
                throw "Error: Vec3 is immutable";
            }
            #end
            zpp_inner.x=x;
        }
        return get_x();
    }
    /**
     * The y component of Vec3.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var y(get_y,set_y):Float;
    inline function get_y():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.y;
    }
    inline function set_y(y:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.immutable){
                throw "Error: Vec3 is immutable";
            }
            #end
            zpp_inner.y=y;
        }
        return get_y();
    }
    /**
     * The z component of Vec3.
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var z(get_z,set_z):Float;
    inline function get_z():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner.validate();
        return zpp_inner.z;
    }
    inline function set_z(z:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.immutable){
                throw "Error: Vec3 is immutable";
            }
            #end
            zpp_inner.z=z;
        }
        return get_z();
    }
    /**
     * Allocate a Vec3 from the global object pool.
     * <br/><br/>
     * Use of this method should always be preferred to the constructor.
     *
     * @param x The x component of Vec3. (default 0)
     * @param y The y component of Vec3. (default 0)
     * @param z The z component of Vec3. (default 0)
     * @return A Vec3 allocated from global object pool with given components.
     */
    public static function get(x:Float=0,y:Float=0,z:Float=0):Vec3{
        var ret;
        {
            if(ZPP_PubPool.poolVec3==null){
                ret=new Vec3();
                #if NAPE_POOL_STATS Vec3.POOL_TOT++;
                Vec3.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_PubPool.poolVec3;
                ZPP_PubPool.poolVec3=ret.zpp_pool;
                ret.zpp_pool=null;
                #if(!NAPE_RELEASE_BUILD)
                ret.zpp_disp=false;
                if(ret==ZPP_PubPool.nextVec3)ZPP_PubPool.nextVec3=null;
                #end
                #if NAPE_POOL_STATS Vec3.POOL_CNT--;
                Vec3.POOL_ADD++;
                #end
            }
        };
        ret.setxyz(x,y,z);
        ret.zpp_inner.immutable=false;
        ret.zpp_inner._validate=null;
        return ret;
    }
    /**
     * Construct a new Vec3.
     * <br/><br/>
     * This method should not generally be used with preference for the
     * static get method which will make use of the global object pool.
     *
     * @param x The x component of Vec3. (default 0)
     * @param y The y component of Vec3. (default 0)
     * @param z The z component of Vec3. (default 0)
     * @return A newly constructed Vec3 with given components.
     */
    public function new(x:Float=0,y:Float=0,z:Float=0){
        zpp_inner=new ZPP_Vec3();
        zpp_inner.outer=this;
        {
            {
                this.x=x;
                this.y=y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((this.x!=this.x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(this.x)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((this.y!=this.y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(this.y)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
            };
            this.z=z;
        };
    }
    /**
     * Produce a copy of this Vec3.
     *
     * @return The copy of this Vec3.
     * @throws # If this Vec3 has been disposed of.
    public function copy():Vec3{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        return Vec3.get(x,y,z);
    }
    /**
     * Release Vec3 object to global object pool.
     *
     * @throws # If this Vec3 has already been disposed of.
     * @throws # If this Vec3 is immutable.
     */
    public function dispose():Void{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.immutable){
            throw "Error: This Vec3 is not disposable";
        }
        #end
        {
            var o=this;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("PublicFree(in T: "+"Vec3"+", in obj: "+"this"+")");
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_pool=null;
            if(ZPP_PubPool.nextVec3!=null)ZPP_PubPool.nextVec3.zpp_pool=o;
            else ZPP_PubPool.poolVec3=o;
            ZPP_PubPool.nextVec3=o;
            #end
            #if NAPE_RELEASE_BUILD 
            o.zpp_pool=ZPP_PubPool.poolVec3;
            ZPP_PubPool.poolVec3=o;
            #end
            #if NAPE_POOL_STATS Vec3.POOL_CNT++;
            Vec3.POOL_SUB++;
            #end
            #if(!NAPE_RELEASE_BUILD)
            o.zpp_disp=true;
            #end
        };
    }
    /**
     * Length of Vec3.
     * <br/><br/>
     * This value may also be set to any value including negatives, though
     * an error will be thrown if length of the Vec3 is already 0 as such
     * a scaling would be undefined. As well as if this Vec3 has been disposed
     * of, or is immutable.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var length(get_length,set_length):Float;
    inline function get_length():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        return Math.sqrt(((this.x*this.x+this.y*this.y)+this.z*this.z));
    }
    inline function set_length(length:Float):Float{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if((length!=length)){
                throw "Error: Vec3::length cannot be NaN";
            }
            if(((this.x*this.x+this.y*this.y)+this.z*this.z)==0){
                throw "Error: Cannot set length of a zero vector";
            }
            #end
            {
                var t=((length/this.length));
                {
                    var t=(t);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"this."+",in s: "+"t"+")");
                        #end
                    };
                    this.x*=t;
                    this.y*=t;
                };
                this.z*=t;
            };
        }
        return get_length();
    }
    /**
     * Compute squared length of Vec3.
     *
     * @return The squared length of this Vec3.
     * @throws # If the Vec3 has been disposed of.
     */
    #if nape_swc@:keep #end
    public function lsq():Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        return((this.x*this.x+this.y*this.y)+this.z*this.z);
    }
    /**
     * Set values of this Vec3 from another.
     *
     * @param vector The vector to set values from.
     * @return A reference to this Vec3.
     * @throws # If the vector argument is null.
     * @throws # If this, or the vector argument are disposed of.
     * @throws # If this Vec3 is immutable.
     */
    public function set(vector:Vec3):Vec3{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null){
            throw "Error: Cannot assign null Vec3";
        }
        #end
        return setxyz(vector.x,vector.y,vector.z);
    }
    /**
     * Set values of this Vec3 from numbers.
     *
     * @param x The new x component value for this vector.
     * @param y The new y component value for this vector.
     * @param z The new z component value for this vector.
     * @return A reference to this Vec3.
     * @throws # If this Vec3 has been disposed of.
     * @throws # If this Vec3 is immutable.
     */
    public function setxyz(x:Float,y:Float,z:Float):Vec3{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        {
            {
                this.x=x;
                this.y=y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((this.x!=this.x));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(this.x)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((this.y!=this.y));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(this.y)"+") :: "+("vec_set(in n: "+"this."+",in x: "+"x"+",in y: "+"y"+")");
                    #end
                };
            };
            this.z=z;
        };
        return this;
    }
    /**
     * Produce copy of the xy components of Vec3.
     * <br/><br/>
     * This function will return a new Vec2 completely seperate
     * from this Vec3 with values equal to the xy components of
     * this Vec3.
     *
     * @param weak If true, then the allocated Vec2 will be weak
     *             so that when used as an argument to a Nape
     *             function it will be automatically released back
     *             to the global object pool. (default false)
     * @return An allocated Vec2 representing the xy components of
     *         this vector.
     */
    public function xy(weak:Bool=false):Vec2{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        return Vec2.get(x,y,weak);
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(this!=null&&this.zpp_disp)throw "Error: "+"Vec3"+" has been disposed and cannot be used!";
            #end
        };
        return "{ x: "+x+" y: "+y+" z: "+z+" }";
    }
}

package nape.phys;
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
 * FluidProperties providing shared parameters for fluid interaction.
 */
@:final#if nape_swc@:keep #end
class FluidProperties{
    /**
     * @private
     */
    public var zpp_inner:ZPP_FluidProperties=null;
    /**
     * Dynamic object for user to store additional data.
     * <br/><br/>
     * This object cannot be set, only its dynamically created
     * properties may be set. In AS3 the type of this property is &#42
     * <br/><br/>
     * This object will be lazily constructed so that until accessed
     * for the first time, will be null internally.
     *
     * @default {}
     */
    #if nape_swc@:isVar #end
    public var userData(get_userData,never):Dynamic<Dynamic>;
    inline function get_userData():Dynamic<Dynamic>{
        if(zpp_inner.userData==null){
            zpp_inner.userData=cast{};
        }
        return zpp_inner.userData;
    }
    /**
     * Set of all active shapes using this object.
     * <br/><br/>
     * Activeness of a shape in the sense that the Shape's Body is inside of a Space.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var shapes(get_shapes,never):ShapeList;
    inline function get_shapes():ShapeList{
        if(zpp_inner.wrap_shapes==null)zpp_inner.wrap_shapes=ZPP_ShapeList.get(zpp_inner.shapes,true);
        return zpp_inner.wrap_shapes;
    }
    /**
     * Construct a new FluidProperties objects.
     *
     * @param density The density of the fluid in g/px/px. (default 1)
     * @param viscosity The viscosity of the fluid for drag computations in kg/px/s
     *                  (default 1)
     * @return The constructed FluidProperties object.
     */
    public function new(density:Float=1,viscosity:Float=1){
        {
            if(ZPP_FluidProperties.zpp_pool==null){
                zpp_inner=new ZPP_FluidProperties();
                #if NAPE_POOL_STATS ZPP_FluidProperties.POOL_TOT++;
                ZPP_FluidProperties.POOL_ADDNEW++;
                #end
            }
            else{
                zpp_inner=ZPP_FluidProperties.zpp_pool;
                ZPP_FluidProperties.zpp_pool=zpp_inner.next;
                zpp_inner.next=null;
                #if NAPE_POOL_STATS ZPP_FluidProperties.POOL_CNT--;
                ZPP_FluidProperties.POOL_ADD++;
                #end
            }
            zpp_inner.alloc();
        };
        zpp_inner.outer=this;
        this.density=density;
        this.viscosity=viscosity;
    }
    /**
     * Produce a copy of this FluidProperties object.
     * <br/><br/>
     * The copied object will be identical in all properties with the the
     * copied userData being assigned the same fields as 'this' Shape with the
     * same values copied over by reference for object types.
     *
     * @return The copied FluidProperties.
     */
    #if nape_swc@:keep #end
    public function copy(){
        var ret=new FluidProperties(density,viscosity);
        if(zpp_inner.userData!=null)ret.zpp_inner.userData=Reflect.copy(zpp_inner.userData);
        ret.gravity=this.gravity;
        return ret;
    }
    /**
     * Density of fluid.
     * <br/><br/>
     * This value, like Material density is of g/pixel/pixel.
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var density(get_density,set_density):Float;
    inline function get_density():Float{
        return zpp_inner.density*("density"=="density"?1000:1);
    }
    inline function set_density(density:Float):Float{
        {
            if(density!=this.density){
                #if(!NAPE_RELEASE_BUILD)
                if((density!=density))throw "Error: FluidProperties::"+"density"+" cannot be NaN";
                if("density"!="density"&&density<0)throw "Error: FluidProperties::"+"density"+" ("+density+") must be >= 0";
                #end
                zpp_inner.density=density/("density"=="density"?1000:1);
                zpp_inner.invalidate();
            }
        }
        return get_density();
    }
    /**
     * Viscosity of fluid.
     * <br/><br/>
     * This value is used in drag comutations, the higher the viscosity the
     * more quickly objects will come to rest in the fluid.
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var viscosity(get_viscosity,set_viscosity):Float;
    inline function get_viscosity():Float{
        return zpp_inner.viscosity*("viscosity"=="density"?1000:1);
    }
    inline function set_viscosity(viscosity:Float):Float{
        {
            if(viscosity!=this.viscosity){
                #if(!NAPE_RELEASE_BUILD)
                if((viscosity!=viscosity))throw "Error: FluidProperties::"+"viscosity"+" cannot be NaN";
                if("viscosity"!="density"&&viscosity<0)throw "Error: FluidProperties::"+"viscosity"+" ("+viscosity+") must be >= 0";
                #end
                zpp_inner.viscosity=viscosity/("viscosity"=="density"?1000:1);
                zpp_inner.invalidate();
            }
        }
        return get_viscosity();
    }
    /**
     * Local gravity for buoyancy computations.
     * <br/><br/>
     * When this value is not null, it will be used in place of the Space gravity
     * when performing buoyancy computations.
     */
    #if nape_swc@:isVar #end
    public var gravity(get_gravity,set_gravity):Null<Vec2>;
    inline function get_gravity():Null<Vec2>{
        return zpp_inner.wrap_gravity;
    }
    inline function set_gravity(gravity:Null<Vec2>):Null<Vec2>{
        {
            if(gravity==null){
                if(zpp_inner.wrap_gravity!=null){
                    zpp_inner.wrap_gravity.zpp_inner._inuse=false;
                    zpp_inner.wrap_gravity.dispose();
                    zpp_inner.wrap_gravity=null;
                }
            }
            else{
                {
                    #if(!NAPE_RELEASE_BUILD)
                    if(gravity!=null&&gravity.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                    #end
                };
                if(zpp_inner.wrap_gravity==null)zpp_inner.getgravity();
                this.gravity.set(gravity);
            }
        }
        return get_gravity();
    }
    /**
     * @private
     */
    @:keep public function toString(){
        return "{ density: "+density+" viscosity: "+viscosity+" gravity: "+gravity+" }";
    }
}

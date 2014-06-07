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
 * Material property providing physical attributes to a Shape.
 */
@:final#if nape_swc@:keep #end
class Material{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Material=null;
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
     * Construct a new Material object.
     *
     * @param elasticity The coeffecient of elasticity for material.
     *                   (default 0.0)
     * @param dynamicFriction The coeffecient of dynamic friction for
     *                        material. (default 1.0)
     * @param staticFriction The coeffecient of static friction for
     *                       material. (default 2.0)
     * @param density The density of the shape using this material in units
     *                of g/pixel/pixel. (default 1.0)
     * @param rollingFriction The coeffecient of rolling friction for material
     *                        used in circle friction computations. (default 0.001)
     * @return The constructed Material object.
     */
    public function new(elasticity:Float=0.0,dynamicFriction:Float=1.0,staticFriction:Float=2.0,density:Float=1,rollingFriction:Float=0.001){
        {
            if(ZPP_Material.zpp_pool==null){
                zpp_inner=new ZPP_Material();
                #if NAPE_POOL_STATS ZPP_Material.POOL_TOT++;
                ZPP_Material.POOL_ADDNEW++;
                #end
            }
            else{
                zpp_inner=ZPP_Material.zpp_pool;
                ZPP_Material.zpp_pool=zpp_inner.next;
                zpp_inner.next=null;
                #if NAPE_POOL_STATS ZPP_Material.POOL_CNT--;
                ZPP_Material.POOL_ADD++;
                #end
            }
            zpp_inner.alloc();
        };
        zpp_inner.outer=this;
        this.elasticity=elasticity;
        this.dynamicFriction=dynamicFriction;
        this.staticFriction=staticFriction;
        this.density=density;
        this.rollingFriction=rollingFriction;
    }
    /**
     * Produce a copy of this Material object.
     * <br/><br/>
     * The copied object will be identical in all properties with the the
     * copied userData being assigned the same fields as 'this' Shape with the
     * same values copied over by reference for object types.
     *
     * @return The copied Material.
     */
    #if nape_swc@:keep #end
    public function copy(){
        var ret=new Material(elasticity,dynamicFriction,staticFriction,density,rollingFriction);
        if(zpp_inner.userData!=null)ret.zpp_inner.userData=Reflect.copy(zpp_inner.userData);
        return ret;
    }
    /**
     * Elasticity of material.
     * <br/><br/>
     * This property may take any value. Coeffecients of elasticity are combined
     * by taking their average, and then clamping to the range [0,1]. In this way
     * you may give very large values (even infinites) to this property to bias
     * the result of combining elasticities.
     * <br/><br/>
     * A combined, clamped value of 0 results in no bouncing whatsoever.
     * <br/>
     * A combine, clamped value of 1 results in complete elasticity where if
     * possible, the objects will not lose any energy at all.
     *
     * @default 0.0
     */
    #if nape_swc@:isVar #end
    public var elasticity(get_elasticity,set_elasticity):Float;
    inline function get_elasticity():Float{
        return zpp_inner.elasticity*("elasticity"=="density"?1000:1);
    }
    inline function set_elasticity(elasticity:Float):Float{
        {
            if(elasticity!=this.elasticity){
                #if(!NAPE_RELEASE_BUILD)
                if((elasticity!=elasticity))throw "Error: Material::"+"elasticity"+" cannot be NaN";
                if("elasticity"=="density"&&elasticity<0)throw "Error: Material::density must be positive";
                if("elasticity"!="elasticity"&&elasticity<0)throw "Error: Material::"+"elasticity"+" cannot be negative";
                #end
                zpp_inner.elasticity=elasticity/("elasticity"=="density"?1000:1);
                zpp_inner.invalidate(ZPP_Material.WAKE|ZPP_Material.ARBITERS);
            }
        }
        return get_elasticity();
    }
    /**
     * Coeffecient of dynamic friction for material.
     * <br/><br/>
     * This property may take any zero or positive value. Coeffecients of
     * dynamicFriction are combined by taking the square root of their product.
     * <br/><br/>
     * The higher this value the more quickly objects will slow down from speed
     * when sliding.
     *
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var dynamicFriction(get_dynamicFriction,set_dynamicFriction):Float;
    inline function get_dynamicFriction():Float{
        return zpp_inner.dynamicFriction*("dynamicFriction"=="density"?1000:1);
    }
    inline function set_dynamicFriction(dynamicFriction:Float):Float{
        {
            if(dynamicFriction!=this.dynamicFriction){
                #if(!NAPE_RELEASE_BUILD)
                if((dynamicFriction!=dynamicFriction))throw "Error: Material::"+"dynamicFriction"+" cannot be NaN";
                if("dynamicFriction"=="density"&&dynamicFriction<0)throw "Error: Material::density must be positive";
                if("dynamicFriction"!="elasticity"&&dynamicFriction<0)throw "Error: Material::"+"dynamicFriction"+" cannot be negative";
                #end
                zpp_inner.dynamicFriction=dynamicFriction/("dynamicFriction"=="density"?1000:1);
                zpp_inner.invalidate(ZPP_Material.WAKE|ZPP_Material.ANGDRAG|ZPP_Material.ARBITERS);
            }
        }
        return get_dynamicFriction();
    }
    /**
     * Coeffecient of static friction for material.
     * <br/><br/>
     * This property may take any zero or positive value. Coeffecients of
     * staticFriction are combined by taking the square root of their product.
     * <br/><br/>
     * The higher this value the more quickly objects will come to rest once moving
     * very slowly, and the harder it will be to cause the objcet to begin to slide.
     *
     * @default 2
     */
    #if nape_swc@:isVar #end
    public var staticFriction(get_staticFriction,set_staticFriction):Float;
    inline function get_staticFriction():Float{
        return zpp_inner.staticFriction*("staticFriction"=="density"?1000:1);
    }
    inline function set_staticFriction(staticFriction:Float):Float{
        {
            if(staticFriction!=this.staticFriction){
                #if(!NAPE_RELEASE_BUILD)
                if((staticFriction!=staticFriction))throw "Error: Material::"+"staticFriction"+" cannot be NaN";
                if("staticFriction"=="density"&&staticFriction<0)throw "Error: Material::density must be positive";
                if("staticFriction"!="elasticity"&&staticFriction<0)throw "Error: Material::"+"staticFriction"+" cannot be negative";
                #end
                zpp_inner.staticFriction=staticFriction/("staticFriction"=="density"?1000:1);
                zpp_inner.invalidate(ZPP_Material.WAKE|ZPP_Material.ARBITERS);
            }
        }
        return get_staticFriction();
    }
    /**
     * Density of Shape's using this Material.
     * <br/><br/>
     * This property has units of g/pixel/pixel, not Kg/pixel/pixel for the
     * simple reason that we get more reasonable values like 1 instead of 0.001
     * to attain reasonable mass values for Bodys.
     *
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
                if((density!=density))throw "Error: Material::"+"density"+" cannot be NaN";
                if("density"=="density"&&density<0)throw "Error: Material::density must be positive";
                if("density"!="elasticity"&&density<0)throw "Error: Material::"+"density"+" cannot be negative";
                #end
                zpp_inner.density=density/("density"=="density"?1000:1);
                zpp_inner.invalidate(ZPP_Material.WAKE|ZPP_Material.PROPS);
            }
        }
        return get_density();
    }
    /**
     * Coeffecient of rolling friction for circle interactions.
     * <br/><br/>
     * This property may take any zero or positive value. Coeffecients of
     * staticFriction are combined by taking the square root of their product.
     * <br/><br/>
     * The higher this value, the more quickly a rolling circle - which would
     * otherwise roll forever ignoring drag and numerical issues - will come to
     * rest.
     *
     * @default 0.01
     */
    #if nape_swc@:isVar #end
    public var rollingFriction(get_rollingFriction,set_rollingFriction):Float;
    inline function get_rollingFriction():Float{
        return zpp_inner.rollingFriction*("rollingFriction"=="density"?1000:1);
    }
    inline function set_rollingFriction(rollingFriction:Float):Float{
        {
            if(rollingFriction!=this.rollingFriction){
                #if(!NAPE_RELEASE_BUILD)
                if((rollingFriction!=rollingFriction))throw "Error: Material::"+"rollingFriction"+" cannot be NaN";
                if("rollingFriction"=="density"&&rollingFriction<0)throw "Error: Material::density must be positive";
                if("rollingFriction"!="elasticity"&&rollingFriction<0)throw "Error: Material::"+"rollingFriction"+" cannot be negative";
                #end
                zpp_inner.rollingFriction=rollingFriction/("rollingFriction"=="density"?1000:1);
                zpp_inner.invalidate(ZPP_Material.WAKE|ZPP_Material.ARBITERS);
            }
        }
        return get_rollingFriction();
    }
    /**
     * @private
     */
    @:keep public function toString(){
        return "{ elasticity: "+elasticity+" dynamicFriction: "+dynamicFriction+" staticFriction: "+staticFriction+" density: "+density+" rollingFriction: "+rollingFriction+" }";
    }
    /**
     * Predefined constructor for a wood style Material.
     *
     * @return <code>new Material(0.4,0.2,0.38,0.7,0.005)</code>
     */
    #if nape_swc@:keep #end
    public static function wood(){
        return new Material(0.4,0.2,0.38,0.7,0.005);
    }
    /**
     * Predefined constructor for a steel style Material.
     *
     * @return <code>new Material(0.2,0.57,0.74,7.8,0.001)</code>
     */
    #if nape_swc@:keep #end
    public static function steel(){
        return new Material(0.2,0.57,0.74,7.8,0.001);
    }
    /**
     * Predefined constructor for a ice style Material.
     *
     * @return <code>new Material(0.3,0.03,0.1,0.9,0.0001)</code>
     */
    #if nape_swc@:keep #end
    public static function ice(){
        return new Material(0.3,0.03,0.1,0.9,0.0001);
    }
    /**
     * Predefined constructor for a rubber style Material.
     *
     * @return <code>new Material(0.8,1.0,1.4,1.5,0.01)</code>
     */
    #if nape_swc@:keep #end
    public static function rubber(){
        return new Material(0.8,1.0,1.4,1.5,0.01);
    }
    /**
     * Predefined constructor for a glass style Material.
     *
     * @return <code>new Material(0.4,0.4,0.94,2.6,0.002)</code>
     */
    #if nape_swc@:keep #end
    public static function glass(){
        return new Material(0.4,0.4,0.94,2.6,0.002);
    }
    /**
     * Predefined constructor for a sand style Material.
     *
     * @return <code>new Material(-1.0,0.45,0.6,1.6,16.0)</code>
     */
    #if nape_swc@:keep #end
    public static function sand(){
        return new Material(-1.0,0.45,0.6,1.6,16.0);
    }
}

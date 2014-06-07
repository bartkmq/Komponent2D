package nape.shape;
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
 * Base type for Nape Shape's
 */
#if nape_swc@:keep #end
class Shape extends Interactor{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Shape=null;
    /**
     * Type of shape.
     */
    #if nape_swc@:isVar #end
    public var type(get_type,never):ShapeType;
    inline function get_type():ShapeType{
        return ZPP_Shape.types[zpp_inner.type];
    }
    /**
     * Faster equivalent to <code>type == ShapeType.CIRCLE</code>
     *
     * @return True if shape is a Circle type.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isCircle(){
        return zpp_inner.isCircle();
    }
    /**
     * Faster equivalent to <code>type == ShapeType.POLYGON</code>
     *
     * @return True if shape is a Polygon type.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isPolygon(){
        return zpp_inner.isPolygon();
    }
    /**
     * @private
     */
    function new(){
        #if(!NAPE_RELEASE_BUILD)
        try{
            super();
        }
        catch(e:Dynamic){}
        #end
        #if NAPE_RELEASE_BUILD 
        super();
        #end
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: Shape cannot be instantiated derp!";
        #end
    }
    /**
     * Body this Shape is assigned to.
     * <br/><br/>
     * This value can be set to add Shape to the given Body, and set to null
     * to remove it from its present Body.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var body(get_body,set_body):Null<Body>;
    inline function get_body():Null<Body>{
        return if(zpp_inner.body!=null)zpp_inner.body.outer else null;
    }
    inline function set_body(body:Null<Body>):Null<Body>{
        {
            zpp_inner.immutable_midstep("Shape::body");
            if(this.body!=body){
                if(zpp_inner.body!=null)this.body.shapes.remove(this);
                if(body!=null)body.shapes.add(this);
            }
        }
        return get_body();
    }
    /**
     * Faster equivalent to casting this to Circle type
     */
    #if nape_swc@:isVar #end
    public var castCircle(get_castCircle,never):Null<Circle>;
    inline function get_castCircle():Null<Circle>{
        return if(isCircle())zpp_inner.circle.outer_zn else null;
    }
    /**
     * Faster equivalent to casting this to Polygon type
     */
    #if nape_swc@:isVar #end
    public var castPolygon(get_castPolygon,never):Null<Polygon>;
    inline function get_castPolygon():Null<Polygon>{
        return if(isPolygon())zpp_inner.polygon.outer_zn else null;
    }
    /**
     * World space centre of mass of this Shape.
     * <br/><br/>
     * This value can be accessed even if Shape is not in a Body, but
     * attempting to query the values of it will return an error in debug
     * builds unless the Shape is in a Body.
     * <br/><br/>
     * This Vec2 is immutable.
     */
    #if nape_swc@:isVar #end
    public var worldCOM(get_worldCOM,never):Vec2;
    inline function get_worldCOM():Vec2{
        if(zpp_inner.wrap_worldCOM==null){
            zpp_inner.wrap_worldCOM=Vec2.get(zpp_inner.worldCOMx,zpp_inner.worldCOMy);
            zpp_inner.wrap_worldCOM.zpp_inner._inuse=true;
            zpp_inner.wrap_worldCOM.zpp_inner._immutable=true;
            zpp_inner.wrap_worldCOM.zpp_inner._validate=zpp_inner.getworldCOM;
        }
        return zpp_inner.wrap_worldCOM;
    }
    /**
     * Local space centre of mass of this Shape.
     * <br/><br/>
     * This Vec2 can be set and is equivalent to performing the necessary
     * translation of the Shape in local coordinates, and also equivalent
     * to <code>this.localCOM.set(value)</code>.
     * <br/>
     * Setting this value whilst this shape is part of a static Body that
     * is part of a Space is not permitted.
     */
    #if nape_swc@:isVar #end
    public var localCOM(get_localCOM,set_localCOM):Vec2;
    inline function get_localCOM():Vec2{
        if(zpp_inner.wrap_localCOM==null){
            if(isCircle())zpp_inner.circle.setupLocalCOM();
            else zpp_inner.polygon.setupLocalCOM();
        }
        return zpp_inner.wrap_localCOM;
    }
    inline function set_localCOM(localCOM:Vec2):Vec2{
        {
            zpp_inner.immutable_midstep("Body::localCOM");
            {
                #if(!NAPE_RELEASE_BUILD)
                if(localCOM!=null&&localCOM.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.body!=null&&zpp_inner.body.space!=null&&zpp_inner.body.isStatic())throw "Error: Cannot modify Shape belonging to a static Object once inside a Space";
            if(localCOM==null)throw "Error: Shape::localCOM cannot be null";
            #end
            this.localCOM.set(localCOM);
        }
        return get_localCOM();
    }
    /**
     * Area of the Hhape.
     */
    #if nape_swc@:isVar #end
    public var area(get_area,never):Float;
    inline function get_area():Float{
        zpp_inner.validate_area_inertia();
        return zpp_inner.area;
    }
    /**
     * Non-mass weighted moment of inertia for Shape.
     */
    #if nape_swc@:isVar #end
    public var inertia(get_inertia,never):Float;
    inline function get_inertia():Float{
        zpp_inner.validate_area_inertia();
        return zpp_inner.inertia;
    }
    /**
     * Coeffecient of angular fluid drag for this Shape.
     */
    #if nape_swc@:isVar #end
    public var angDrag(get_angDrag,never):Float;
    inline function get_angDrag():Float{
        zpp_inner.validate_angDrag();
        return zpp_inner.angDrag;
    }
    /**
     * Material used by this shape.
     *
     * @default new Material()
     */
    #if nape_swc@:isVar #end
    public var material(get_material,set_material):Material;
    inline function get_material():Material{
        return zpp_inner.material.wrapper();
    }
    inline function set_material(material:Material):Material{
        {
            zpp_inner.immutable_midstep("Shape::material");
            #if(!NAPE_RELEASE_BUILD)
            if(material==null)throw "Error: Cannot assign null as Shape material";
            #end
            zpp_inner.setMaterial(material.zpp_inner);
        }
        return get_material();
    }
    /**
     * InteractionFilter used by this shape.
     *
     * @default new InteractionFilter()
     */
    #if nape_swc@:isVar #end
    public var filter(get_filter,set_filter):InteractionFilter;
    inline function get_filter():InteractionFilter{
        return zpp_inner.filter.wrapper();
    }
    inline function set_filter(filter:InteractionFilter):InteractionFilter{
        {
            zpp_inner.immutable_midstep("Shape::filter");
            #if(!NAPE_RELEASE_BUILD)
            if(filter==null)throw "Error: Cannot assign null as Shape filter";
            #end
            zpp_inner.setFilter(filter.zpp_inner);
        }
        return get_filter();
    }
    /**
     * FluidProperties used by this shape.
     * <br/><br/>
     * This object provides information for buoyancy and fluid drag computations
     * when this shape is interacting as a fluid.
     *
     * @default new FluidProperties();
     */
    #if nape_swc@:isVar #end
    public var fluidProperties(get_fluidProperties,set_fluidProperties):FluidProperties;
    inline function get_fluidProperties():FluidProperties{
        zpp_inner.immutable_midstep("Shape::fluidProperties");
        if(zpp_inner.fluidProperties==null)zpp_inner.setFluid(new FluidProperties().zpp_inner);
        return zpp_inner.fluidProperties.wrapper();
    }
    inline function set_fluidProperties(fluidProperties:FluidProperties):FluidProperties{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(fluidProperties==null)throw "Error: Cannot assign null as Shape fluidProperties, disable fluids by setting fluidEnabled to false";
            #end
            zpp_inner.setFluid(fluidProperties.zpp_inner);
        }
        return get_fluidProperties();
    }
    /**
     * Whether this shape is able to interact as a fluid.
     * <br/><br/>
     * Unless this field is true, this Shape can never interact as a fluid.
     * <br/><br/>
     * Just because this field is true however, does not mean this shape will always
     * interact as a fluid, the final result is down to the combination of
     * InteractionFilters on the pairing of shapes and sensory interaction
     * takes higher priority.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var fluidEnabled(get_fluidEnabled,set_fluidEnabled):Bool;
    inline function get_fluidEnabled():Bool{
        return zpp_inner.fluidEnabled;
    }
    inline function set_fluidEnabled(fluidEnabled:Bool):Bool{
        {
            zpp_inner.immutable_midstep("Shape::fluidEnabled");
            zpp_inner.fluidEnabled=fluidEnabled;
            if(fluidEnabled&&zpp_inner.fluidProperties==null)this.fluidProperties=new FluidProperties();
            zpp_inner.wake();
        }
        return get_fluidEnabled();
    }
    /**
     * Whether this shape is able to interact as sensor.
     * <br/><br/>
     * Unless this field is true, this Shape can never interact as a sensor.
     * <br/><br/>
     * Just because this field is true however, does not mean this shape will always
     * interact as a sensor, the final result is down to the combination of
     * InteractionFilters on the pairing of shapes. Sensor interaction has highest priority.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var sensorEnabled(get_sensorEnabled,set_sensorEnabled):Bool;
    inline function get_sensorEnabled():Bool{
        return zpp_inner.sensorEnabled;
    }
    inline function set_sensorEnabled(sensorEnabled:Bool):Bool{
        {
            zpp_inner.immutable_midstep("Shape::sensorEnabled");
            zpp_inner.sensorEnabled=sensorEnabled;
            zpp_inner.wake();
        }
        return get_sensorEnabled();
    }
    /**
     * World space bounding box for this shape.
     * <br/><br/>
     * This value can be accessed even if the Shape is not part of a Body,
     * however attempting to query its values would result in an error in
     * debug builds.
     * <br/><br/>
     * This AABB is immutable.
     */
    #if nape_swc@:isVar #end
    public var bounds(get_bounds,never):AABB;
    inline function get_bounds():AABB{
        return zpp_inner.aabb.wrapper();
    }
    /**
     * Translate this shape in its local coordinate system.
     * <br/><br/>
     * This is equivalent to: <code>shape.localCOM.addeq(displacement)</code>
     *
     * @param translation The local translation to apply to Shape.
     * @return A reference to 'this' Shape.
     * @throws # If translation is null or has been disposed of.
     * @throws # If this shape is part of a static body that is inside of a Space.
     */
    #if nape_swc@:keep #end
    public function translate(translation:Vec2){
        zpp_inner.immutable_midstep("Shape::translate()");
        {
            #if(!NAPE_RELEASE_BUILD)
            if(translation!=null&&translation.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.body!=null&&zpp_inner.body.space!=null&&zpp_inner.body.isStatic())throw "Error: Cannot modify Shape belonging to a static Object once inside a Space";
        if(translation==null)throw "Error: Cannot displace Shape by null Vec2";
        #end
        if(translation.lsq()>0){
            if(isCircle())zpp_inner.circle.__translate(translation.x,translation.y);
            else zpp_inner.polygon.__translate(translation.x,translation.y);
        }
        ({
            if(({
                translation.zpp_inner.weak;
            })){
                translation.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Scale this shape in its local coordinate system.
     * <br/><br/>
     * For Circle shapes, scalex and scaley must be exactly equal.
     *
     * @param scalex The x-coordinate scaling to apply to Shape.
     * @param scaley The y-coordinate scaling to apply to Shape.
     * @return A reference to 'this' Shape.
     * @throws # If this shape is part of a static body that is inside of a Space.
     * @throws # If scalex or scaley is 0. Negative values 'are' permitted.
     */
    #if nape_swc@:keep #end
    public function scale(scalex:Float,scaley:Float){
        zpp_inner.immutable_midstep("Shape::scale()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.body!=null&&zpp_inner.body.space!=null&&zpp_inner.body.isStatic())throw "Error: Cannot modify Shape belonging to a static Object once inside a Space";
        if((scalex!=scalex)||(scaley!=scaley))throw "Error: Cannot scale Shape by NaN";
        if(scalex==0||scaley==0)throw "Error: Cannot Scale shape by a factor of 0";
        #end
        if(isCircle()){
            var d=scalex*scalex-scaley*scaley;
            if(d*d<(Config.epsilon*Config.epsilon)){
                zpp_inner.circle.__scale(scalex,scaley);
            }
            else{
                #if(!NAPE_RELEASE_BUILD)
                throw "Error: Cannot perform a non equal scaling on a Circle";
                #end
            }
        }
        else zpp_inner.polygon.__scale(scalex,scaley);
        return this;
    }
    /**
     * Rotate this shape in its local coordinate system.
     *
     * @param angle The number of radians to rotate this Shape by in a clockwise
     *              direction.
     * @return A reference to 'this' Shape.
     * @throws # If this shape is part of a static body that is inside of a Space.
     */
    #if nape_swc@:keep #end
    public function rotate(angle:Float){
        zpp_inner.immutable_midstep("Shape::rotate()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.body!=null&&zpp_inner.body.space!=null&&zpp_inner.body.isStatic())throw "Error: Cannot modify Shape belonging to a static Object once inside a Space";
        if((angle!=angle))throw "Error: Cannot rotate Shape by NaN";
        #end
        var dr=angle%(2*Math.PI);
        if(dr!=0.0){
            var cos=Math.cos(angle);
            var sin=Math.sin(angle);
            if(isCircle())zpp_inner.circle.__rotate(sin,cos);
            else zpp_inner.polygon.__rotate(sin,cos);
        }
        return this;
    }
    /**
     * Apply local transformation matrix to Shape.
     * <br/><br/>
     * For Circle shapes, the matrix must be equiorthogonal.
     *
     * @param matrix The matrix to transform Shape by.
     * @return A reference to 'this' Shape.
     * @throws # If matrix is null or singular.
     * @throws # If shape is a Circle, and matrix is not equiorthogonal.
     * @throws # If this shape is part of a static body that is inside of a Space.
     */
    #if nape_swc@:keep #end
    public function transform(matrix:Mat23){
        zpp_inner.immutable_midstep("Shape::transform()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.body!=null&&zpp_inner.body.space!=null&&zpp_inner.body.isStatic())throw "Error: Cannot modify Shape belonging to a static Object once inside a Space";
        if(matrix==null)throw "Error: Cannot transform Shape by null matrix";
        if(matrix.singular())throw "Error: Cannot transform Shape by a singular matrix";
        #end
        if(isCircle()){
            if(matrix.equiorthogonal()){
                zpp_inner.circle.__transform(matrix);
            }
            else{
                #if(!NAPE_RELEASE_BUILD)
                throw "Error: Cannot transform Circle by a non equiorthogonal matrix";
                #end
            }
        }
        else zpp_inner.polygon.__transform(matrix);
        return this;
    }
    /**
     * Test containment of world-space coordinate in Shape.
     * <br/><br/>
     * This Shape must be part of a Body so that world coordinates are
     * defined.
     *
     * @param point The point to check for containment.
     * @return True if point is contained within the Shape.
     * @throws If point is null or disposed of.
     * @throws If this shape is not part of a Body.
     */
    #if nape_swc@:keep #end
    public function contains(point:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Cannot check null point for containment";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: Shape is not well defined without a Body";
        #end
        ZPP_Geom.validateShape(zpp_inner);
        point.zpp_inner.validate();
        var ret=ZPP_Collide.shapeContains(zpp_inner,point.zpp_inner);
        ({
            if(({
                point.zpp_inner.weak;
            })){
                point.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Produce an exact copy of this Shape.
     * <br/><br/>
     * The copied shape will be identical with the copied Shape's userData
     * object being assigned the same fields as 'this' Shape with the same
     * values copied over by reference for object types.
     *
     * @return A copy of this shape.
     */
    #if nape_swc@:keep #end
    public function copy(){
        return zpp_inner.copy();
    }
    /**
     * @private
     */
    @:keep public override function toString(){
        var ret=isCircle()?"Circle":"Polygon";
        return ret+"#"+id;
    }
}

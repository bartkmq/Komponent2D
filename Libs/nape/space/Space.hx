package nape.space;
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
 * The heart of all Nape simulations.
 */
@:final#if nape_swc@:keep #end
class Space{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Space=null;
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
     * Space gravity.
     * <br/><br/>
     * Units are of pixels/second/second
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var gravity(get_gravity,set_gravity):Vec2;
    inline function get_gravity():Vec2{
        if(zpp_inner.wrap_gravity==null)zpp_inner.getgravity();
        return zpp_inner.wrap_gravity;
    }
    inline function set_gravity(gravity:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(gravity!=null&&gravity.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(gravity==null)throw "Error: Space::gravity cannot be null";
            #end
            this.gravity.set(gravity);
        }
        return get_gravity();
    }
    /**
     * Broadphase type in use.
     */
    #if nape_swc@:isVar #end
    public var broadphase(get_broadphase,never):Broadphase;
    inline function get_broadphase():Broadphase{
        return zpp_inner.bphase.is_sweep?Broadphase.SWEEP_AND_PRUNE:Broadphase.DYNAMIC_AABB_TREE;
    }
    /**
     * Flag controlling sorting of contact points.
     * <br/><br/>
     * If true, then collisions will be resolved in an order defined by their
     * penetration depths. This can be shown to improve stability of the physics
     * as well as making simulations more consistent regardless of which broadphase
     * is used.
     * <br/><br/>
     * Having sorting enabled obviously incurs a cost, and you may consider
     * disabling it if you are having issues with performance (Though things
     * such as number of physics iterations will have much greater bearing on
     * performance than this, especcialy since enabling this may permit you
     * to use less iterations).
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var sortContacts(get_sortContacts,set_sortContacts):Bool;
    inline function get_sortContacts():Bool{
        return zpp_inner.sortcontacts;
    }
    inline function set_sortContacts(sortContacts:Bool):Bool{
        {
            zpp_inner.sortcontacts=sortContacts;
        }
        return get_sortContacts();
    }
    /**
     * Angular drag applied to all bodies in Space.
     * <br/><br/>
     * This represents the fraction of a body's angular velocity which will be
     * removed per second. This value has no unit attached.
     *
     * @default 0.015
     */
    #if nape_swc@:isVar #end
    public var worldAngularDrag(get_worldAngularDrag,set_worldAngularDrag):Float;
    inline function get_worldAngularDrag():Float{
        return zpp_inner.global_ang_drag;
    }
    inline function set_worldAngularDrag(worldAngularDrag:Float):Float{
        {
            var d=worldAngularDrag;
            #if(!NAPE_RELEASE_BUILD)
            if((d!=d))throw "Error: Space::worldAngularDrag cannot be NaN";
            #end
            zpp_inner.global_ang_drag=d;
        }
        return get_worldAngularDrag();
    }
    /**
     * Linear drag applied to all bodies in Space.
     * <br/><br/>
     * This represents the fraction of a body's linear velocity which will be
     * removed per second. This value has no unit attached.
     *
     * @default 0.015
     */
    #if nape_swc@:isVar #end
    public var worldLinearDrag(get_worldLinearDrag,set_worldLinearDrag):Float;
    inline function get_worldLinearDrag():Float{
        return zpp_inner.global_lin_drag;
    }
    inline function set_worldLinearDrag(worldLinearDrag:Float):Float{
        {
            var d=worldLinearDrag;
            #if(!NAPE_RELEASE_BUILD)
            if((d!=d))throw "Error: Space::worldLinearDrag cannot be NaN";
            #end
            zpp_inner.global_lin_drag=d;
        }
        return get_worldLinearDrag();
    }
    /**
     * List of all Compounds directly placed in space.
     * <br/><br/>
     * This list is mutable, and adding an element to this list is one way of
     * adding a Compound to this Space equivalent to: <code>compound.space = space</code>
     * <br/><br/>
     * This list is only those compounds directly placed in the space, any
     * compound that is a child of another compound will not be in this list.
     */
    #if nape_swc@:isVar #end
    public var compounds(get_compounds,never):CompoundList;
    inline function get_compounds():CompoundList{
        return zpp_inner.wrap_compounds;
    }
    /**
     * List of all Bodys directly placed in space.
     * <br/><br/>
     * This list is mutable, and adding an element to this list is one way of
     * adding a Body to this Space equivalent to: <code>body.space = space</code>
     * <br/><br/>
     * This list is only those bodies directly placed in the space, any
     * body that is a child of a Compound will not be in this list.
     */
    #if nape_swc@:isVar #end
    public var bodies(get_bodies,never):BodyList;
    inline function get_bodies():BodyList{
        return zpp_inner.wrap_bodies;
    }
    /**
     * List of all active dynamic Bodies in space.
     * <br/><br/>
     * This list contains all dynamic bodies that are awake regardless of their containment in a Compound.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var liveBodies(get_liveBodies,never):BodyList;
    inline function get_liveBodies():BodyList{
        return zpp_inner.wrap_live;
    }
    /**
     * List of all Constraints directly placed in space.
     * <br/><br/>
     * This list is mutable, and adding an element to this list is one way of
     * adding a Constraint to this Space equivalent to: <code>constraint.space = space</code>
     * <br/><br/>
     * This list is only those bodies directly placed in the space, any
     * constraint that is a child of a Compound will not be in this list.
     */
    #if nape_swc@:isVar #end
    public var constraints(get_constraints,never):ConstraintList;
    inline function get_constraints():ConstraintList{
        return zpp_inner.wrap_constraints;
    }
    /**
     * List of all active Constraints in space.
     * <br/><br/>
     * This list contains all constraints regardless of their containment in a Compound.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var liveConstraints(get_liveConstraints,never):ConstraintList;
    inline function get_liveConstraints():ConstraintList{
        return zpp_inner.wrap_livecon;
    }
    /**
     * Apply given function to all bodies in space.
     * <br/><br/>
     * This method is a way to iterate over 'every' Body in the Space
     * regardless of containment in a Compound.
     *
     * @param lambda The function to apply to each Body.
     * @throws # If lambda is null.
     */
    public function visitBodies(lambda:Body->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Space::visitBodies";
        #end
        for(b in bodies)lambda(b);
        for(c in compounds)c.visitBodies(lambda);
    }
    /**
     * Apply given function to all constraints in space.
     * <br/><br/>
     * This method is a way to iterate over 'every' Constraint in the Space
     * regardless of containment in a Compound.
     *
     * @param lambda The function to apply to each Constraint.
     * @throws # If lambda is null.
     */
    public function visitConstraints(lambda:Constraint->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Space::visitConstraints";
        #end
        for(c in constraints)lambda(c);
        for(c in compounds)c.visitConstraints(lambda);
    }
    /**
     * Apply given function to all compounds in space.
     * <br/><br/>
     * This method is a way to iterate over 'every' Compound in the Space
     * regardless of containment in another Compound.
     *
     * @param lambda The function to apply to each Compound.
     * @throws # If lambda is null.
     */
    public function visitCompounds(lambda:Compound->Void){
        #if(!NAPE_RELEASE_BUILD)
        if(lambda==null)throw "Error: lambda cannot be null for Space::visitCompounds";
        #end
        for(c in compounds){
            lambda(c);
            c.visitCompounds(lambda);
        }
    }
    /**
     * Static, immutable Body for constraint purposes.
     * <br/><br/>
     * This is a completely static, uncollidable, uninteractable Body
     * with no Shapes, that cannot be modified in any way.
     * <br/><br/>
     * Its purpose is to provide a means for attaching Constraints
     * from one Body to the Space itself, for instance pinning a body
     * against a static point in space.
     */
    #if nape_swc@:isVar #end
    public var world(get_world,never):Body;
    inline function get_world():Body{
        return zpp_inner.__static;
    }
    /**
     * List of all active arbiters in Space.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var arbiters(get_arbiters,never):ArbiterList;
    inline function get_arbiters():ArbiterList{
        if(zpp_inner.wrap_arbiters==null){
            var ret=new ZPP_SpaceArbiterList();
            ret.space=zpp_inner;
            zpp_inner.wrap_arbiters=ret;
        }
        return zpp_inner.wrap_arbiters;
    }
    /**
     * List of all Listeners in space.
     * <br/><br/>
     * This list is mutable, and adding an element to this list is one way of
     * adding a Listener to this Space equivalent to: <code>listener.space = space</code>
     */
    #if nape_swc@:isVar #end
    public var listeners(get_listeners,never):ListenerList;
    inline function get_listeners():ListenerList{
        return zpp_inner.wrap_listeners;
    }
    /**
     * Clear the Space of all objects.
     * <br/><br/>
     * Things such as the elapsed simulation time, and time step will too be
     * reset to 0.
     * <br/><br/>
     * Parameters such as gravity, and worldLinearDrag will be untouched by
     * this operation.
     */
    #if nape_swc@:keep #end
    public function clear(){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.midstep)throw "Error: Space::clear() cannot be called during space step()";
        #end
        zpp_inner.clear();
    }
    /**
     * Step simulation forward in time.
     *
     * @param deltaTime The number of seconds to simulate. For 60fps physics
     *                  you would use a value of 1/60.
     * @param velocityIterations The number of iterations to use in resolving
     *                           errors in the velocities of objects. This is
     *                           together with collision detection the most
     *                           expensive phase of a simulation update, as well
     *                           as the most important for stable results.
     *                           (default 10)
     * @param positionIterations The number of iterations to use in resolving
     *                           errors in the positions of objects. This is
     *                           far more lightweight than velocity iterations,
     *                           as well as being less important for the
     *                           stability of results. (default 10)
     * @throws # If deltaTime is not strictly positive.
     * @throws # If either of the number of iterations is not strictly positive.
     */
    #if nape_swc@:keep #end
    public function step(deltaTime:Float,velocityIterations:Int=10,positionIterations:Int=10){
        #if(!NAPE_RELEASE_BUILD)
        if((deltaTime!=deltaTime))throw "Error: deltaTime cannot be NaN";
        if(deltaTime<=0)throw "Error: deltaTime must be strictly positive";
        if(velocityIterations<=0)throw "Error: must use atleast one velocity iteration";
        if(positionIterations<=0)throw "Error: must use atleast one position iteration";
        #end
        zpp_inner.step(deltaTime,velocityIterations,positionIterations);
    }
    /**
     * The time stamp of this Space object.
     * <br/><br/>
     * This is equal to the number of times that space.step(..) has been invoked.
     */
    #if nape_swc@:isVar #end
    public var timeStamp(get_timeStamp,never):Int;
    inline function get_timeStamp():Int{
        return zpp_inner.stamp;
    }
    /**
     * The elapsed simulation time.
     * <br/><br/>
     * This is the total amount of 'time' that has elapsed in the Space simulation.
     */
    #if nape_swc@:isVar #end
    public var elapsedTime(get_elapsedTime,never):Float;
    inline function get_elapsedTime():Float{
        return zpp_inner.time;
    }
    /**
     * Construct a new Space object.
     *
     * @param gravity The gravity of this space. (default &#40;0,0&#41;)
     * @param broadphase The broadphase type to use. (default DYNAMIC_AABB_TREE)
     * @return The constructed Space object.
     * @throws # If gravity is non-null, and has been disposed of.
     */
    public function new(gravity:Vec2=null,broadphase:Broadphase=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(gravity!=null&&gravity.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        zpp_inner=new ZPP_Space(gravity==null?null:gravity.zpp_inner,broadphase);
        zpp_inner.outer=this;
        if(gravity!=null)({
            if(({
                gravity.zpp_inner.weak;
            })){
                gravity.dispose();
                true;
            }
            else{
                false;
            }
        });
    }
    /**
     * Determine the interaction type that would occur between a pair of Shapes.
     * <br/><br/>
     * This function takes into account everything possible, and ignoring the
     * callback system will tell you precisely the type of interaction (if any
     * at all) which will occur between these Shapes.
     * <br/><br/>
     * This function can only work if the Shapes belong to a Body.
     * <br/><br/>
     * This function can only make use of any constraints 'ignore' property
     * to determine if 'null' should be returned if the constraints being used
     * are inside of a Space.
     *
     * @param shape1 The first Shape to test.
     * @param shape2 The second Shape to test.
     * @return The interaction type that will occur between these shapes, or null
     *         if no interaction will occur.
     * @throws # If either shape is null, or is not contained within a body.
     */
    #if nape_swc@:keep #end
    public function interactionType(shape1:Shape,shape2:Shape):Null<InteractionType>{
        #if(!NAPE_RELEASE_BUILD)
        if(shape1==null||shape2==null)throw "Error: Cannot evaluate interaction type for null shapes";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(shape1.body==null||shape2.body==null)throw "Error: Cannot evaluate interaction type for shapes not part of a Body";
        #end
        if(shape1.body.isStatic()&&shape2.body.isStatic())return null;
        if(shape1.body==shape2.body)return null;
        var s1=shape1.zpp_inner;
        var s2=shape2.zpp_inner;
        return switch(zpp_inner.interactionType(s1,s2,s1.body,s2.body)){
            case 0:InteractionType.FLUID;
            case 1:InteractionType.COLLISION;
            case 2:InteractionType.SENSOR;
            default:null;
        }
    }
    /**
     * Evaluate all Shapes under a given Point.
     * <br/><br/>
     * If the filter argument is non-null, then only shapes who's filter
     * agrees to 'collide' will be considered.
     *
     * @param point The point to evaluate shapes.
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the Shapes containing the given point.
     * @throws # If point is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function shapesUnderPoint(point:Vec2,filter:InteractionFilter=null,output:ShapeList=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Error: Cannot evaluate shapes under a null point :)";
        #end
        var ret=zpp_inner.shapesUnderPoint(point.x,point.y,filter==null?null:filter.zpp_inner,output);
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
     * Evaluate all Bodies under a given Point.
     * <br/><br/>
     * If the filter argument is non-null, then only bodies with a shape containing
     * the given point whose filter agrees to 'collide' will be considered.
     *
     * @param point The point to evaluate bodies.
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the Bodies containing the given point.
     * @throws # If point is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function bodiesUnderPoint(point:Vec2,filter:InteractionFilter=null,output:BodyList=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Error: Cannot evaluate objects under a null point :)";
        #end
        var ret=zpp_inner.bodiesUnderPoint(point.x,point.y,filter==null?null:filter.zpp_inner,output);
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
     * Evaluate all Shapes given an AABB.
     * <br/><br/>
     * If the filter argument is non-null, then only shapes who's filter
     * agrees to 'collide' will be considered.
     *
     * @param aabb The bounding box to query shapes by,
     * @param containment If true, then only Shapes entirely contained (Rather
     *                    than simply intersected) will be considered.
     *                    (default false)
     * @param strict If false, then the Shape's bounding box will be used to
     *               classify the Shape, instead of the Shape itself.
     *               (default true)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given AABB.
     * @throws # If AABB is null, or is degenerate.
     */
    #if nape_swc@:keep #end
    public function shapesInAABB(aabb:AABB,containment:Bool=false,strict:Bool=true,filter:InteractionFilter=null,output:ShapeList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(aabb==null)throw "Error: Cannot evaluate shapes in a null AABB :)";
        if(aabb.width==0||aabb.height==0)throw "Error: Cannot evaluate shapes in degenerate AABB :/";
        #end
        return zpp_inner.shapesInAABB(aabb,strict,containment,filter==null?null:filter.zpp_inner,output);
    }
    /**
     * Evaluate all Bodies given an AABB.
     * <br/><br/>
     * If the filter argument is non-null, then only bodies with a shape
     * classified as being part of the AABB, whose filter agrees to collide
     * will be considered.
     *
     * @param aabb The bounding box to query bodies by,
     * @param containment If true, then only Bodies entirely contained (Rather
     *                    than simply intersecting) will be considered.
     *                    (default false)
     * @param strict If false, then the body's shape's bounding box will be used to
     *               classify the shapes of the body, instead of the Shape itself.
     *               (default true)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given AABB.
     * @throws # If AABB is null, or is degenerate.
     */
    #if nape_swc@:keep #end
    public function bodiesInAABB(aabb:AABB,containment:Bool=false,strict:Bool=true,filter:InteractionFilter=null,output:BodyList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(aabb==null)throw "Error: Cannot evaluate objects in a null AABB :)";
        if(aabb.width==0||aabb.height==0)throw "Error: Cannot evaluate objects in degenerate AABB :/";
        #end
        return zpp_inner.bodiesInAABB(aabb,strict,containment,filter==null?null:filter.zpp_inner,output);
    }
    /**
     * Evaluate all Shapes given a circle.
     * <br/><br/>
     * If the filter argument is non-null, then only shapes who's filter
     * agrees to 'collide' will be considered.
     *
     * @param position The position of the centre of the circle.
     * @param radius The radius of the circle.
     * @param containment If true, then only Shapes entirely contained (Rather
     *                    than simply intersected) will be considered.
     *                    (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given circle.
     * @throws # If positions is null or disposed of.
     * @throws # If radius is not strictly positive.
     */
    #if nape_swc@:keep #end
    public function shapesInCircle(position:Vec2,radius:Float,containment:Bool=false,filter:InteractionFilter=null,output:ShapeList=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(position==null)throw "Error: Cannot evaluate shapes at null circle :)";
        if((radius!=radius))throw "Error: Circle radius cannot be NaN";
        if(radius<=0)throw "Error: Circle radius must be strictly positive";
        #end
        var ret=zpp_inner.shapesInCircle(position,radius,containment,filter==null?null:filter.zpp_inner,output);
        ({
            if(({
                position.zpp_inner.weak;
            })){
                position.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Evaluate all Bodies given a circle.
     * <br/><br/>
     * If the filter argument is non-null, then only bodies with a shape
     * classified as being part of the circle, whose filter agrees to collide
     * will be considered.
     *
     * @param position The position of the centre of the circle.
     * @param radius The radius of the circle.
     * @param containment If true, then only Bodies entirely contained (Rather
     *                    than simply intersecting) will be considered. If a
     *                    filter is supplied, only shapes that agree to collide
     *                    will be used in this containment check.
     *                    (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given circle.
     * @throws # If positions is null or disposed of.
     * @throws # If radius is not strictly positive.
     */
    #if nape_swc@:keep #end
    public function bodiesInCircle(position:Vec2,radius:Float,containment:Bool=false,filter:InteractionFilter=null,output:BodyList=null){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(position==null)throw "Error: Cannot evaluate objects at null circle :)";
        if((radius!=radius))throw "Error: Circle radius cannot be NaN";
        if(radius<=0)throw "Error: Circle radius must be strictly positive";
        #end
        var ret=zpp_inner.bodiesInCircle(position,radius,containment,filter==null?null:filter.zpp_inner,output);
        ({
            if(({
                position.zpp_inner.weak;
            })){
                position.dispose();
                true;
            }
            else{
                false;
            }
        });
        return ret;
    }
    /**
     * Evaluate all Shapes given another shape.
     * <br/><br/>
     * If the filter argument is non-null, then only shapes who's filter
     * agrees to 'collide' will be considered. The input shape's own filter
     * is never used in this method. The input shape is considered a purely
     * geometric object.
     * <br/><br/>
     * The input shape must be part of a Body so as to be well defined.
     *
     * @param shape The shape to use in classifying other shapes.
     * @param containment If true, then only Shapes entirely contained (Rather
     *                    than simply intersected) will be considered.
     *                    (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given shape.
     * @throws # If shape is null or not part of a body.
     * @throws # If shape is a polygon, and that polygon is not 'valid'
     */
    #if nape_swc@:keep #end
    public function shapesInShape(shape:Shape,containment:Bool=false,filter:InteractionFilter=null,output:ShapeList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(shape==null)throw "Error: Cannot evaluate shapes in a null shapes :)";
        if(shape.body==null)throw "Error: Query shape needs to be inside a Body to be well defined :)";
        if(shape.isPolygon()){
            var res=shape.zpp_inner.polygon.valid();
            if(res!=ValidationResult.VALID)throw "Error: Polygon query shape is invalid : "+res.toString();
        }
        #end
        return zpp_inner.shapesInShape(shape.zpp_inner,containment,filter==null?null:filter.zpp_inner,output);
    }
    /**
     * Evaluate all Bodies given a shape.
     * <br/><br/>
     * If the filter argument is non-null, then only bodies with a shape
     * classified as being part of the input shape, whose filter agrees to collide
     * will be considered. The input shape is considered a purely geometric
     * <br/><br/>
     * The input shape must be part of a Body so as to be well defined.
     *
     * @param shape The shape to use in classifying other shapes.
     * @param containment If true, then only Bodies entirely contained (Rather
     *                    than simply intersecting) will be considered.
     *                    (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the bodies for given shape.
     * @throws # If shape is null or not part of a body.
     * @throws # If shape is a polygon, and that polygon is not 'valid'
     */
    #if nape_swc@:keep #end
    public function bodiesInShape(shape:Shape,containment:Bool=false,filter:InteractionFilter=null,output:BodyList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(shape==null)throw "Error: Cannot evaluate bodies in a null shapes :)";
        if(shape.body==null)throw "Error: Query shape needs to be inside a Body to be well defined :)";
        if(shape.isPolygon()){
            var res=shape.zpp_inner.polygon.valid();
            if(res!=ValidationResult.VALID)throw "Error: Polygon query shape is invalid : "+res.toString();
        }
        #end
        return zpp_inner.bodiesInShape(shape.zpp_inner,containment,filter==null?null:filter.zpp_inner,output);
    }
    /**
     * Evaluate all Shapes given a Body.
     * <br/><br/>
     * If the filter argument is non-null, then only shapes who's filter
     * agrees to 'collide' will be considered. The input body's shape's own filters
     * are never used in this method. The input body is considered a purely
     * geometric object.
     *
     * @param body The body to use in classifying other shapes.
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the shapes for given body.
     * @throws # If body is null.
     * @throws # If body has a shape that is a polygon, and that polygon is not 'valid'
     */
    #if nape_swc@:keep #end
    public function shapesInBody(body:Body,filter:InteractionFilter=null,output:ShapeList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: Cannot evaluate shapes in null body";
        #end
        var ret=(output==null?new ShapeList():output);
        for(shape in body.shapes){
            var cur=shapesInShape(shape,false,filter,ret);
        }
        return ret;
    }
    /**
     * Evaluate all Bodies given a Body.
     * <br/><br/>
     * If the filter argument is non-null, then only bodies with a shape
     * classified as being part of the input body, whose filter agrees to collide
     * will be considered. The input body is considered a purely geometric
     *
     * @param body The body to use in classifying other bodies.
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output Optional list to append results to instead of creating a new list (default null).
     * @return A list of all the bodies for given body.
     * @throws # If body is null.
     * @throws # If body has a shape that is a polygon, and that polygon is not 'valid'
     */
    #if nape_swc@:keep #end
    public function bodiesInBody(body:Body,filter:InteractionFilter=null,output:BodyList=null){
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: Cannot evaluate shapes in null body";
        #end
        var ret=(output==null?new BodyList():output);
        for(shape in body.shapes){
            var cur=bodiesInShape(shape,false,filter,ret);
        }
        return ret;
    }
    /**
     * Perform a convex cast for soonest collision.
     * <br/><br/>
     * This method will return only the soonest collision result (if any), to find
     * more than this, use the convexMultiCast method. The shape will not be
     * swept further than the time delta provided.
     * Shapes already intersecting
     * the sweep shape at t = 0 are ignored.
     * <br/><br/>
     * If the filter argument is null, then all shapes will be collidable
     * otherwise only those for whose filter agrees to 'collide'.
     *
     * @param shape The Shape to be cast through space. This shape must belong
     *              to a body whose velocity is used to define the sweep.
     * @param deltaTime The amount of time to sweep the shape forward.
     * @param liveSweep If true, then moving objects in the space will have their motion considered during the sweep. Otherwise; like with normal rayCast, objects in the space are considered un-moving for the cast. (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @return The soonest result (if any) of convex intersection.
     * @throws # If shape is null, or not part of a body.
     * @throws # If deltaTime is negative.
     */
    #if nape_swc@:keep #end
    public function convexCast(shape:Shape,deltaTime:Float,liveSweep:Bool=false,filter:InteractionFilter=null):Null<ConvexResult>{
        #if(!NAPE_RELEASE_BUILD)
        if(shape==null)throw "Error: Cannot cast null shape :)";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(shape.body==null)throw "Error: Shape must belong to a body to be cast.";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(deltaTime<0||(deltaTime!=deltaTime))throw "Error: deltaTime must be positive";
        #end
        return zpp_inner.convexCast(shape.zpp_inner,deltaTime,filter,liveSweep);
    }
    /**
     * Perform a convex cast for all collisions in time order.
     * <br/><br/>
     * This method will return all collisions, or an empty list if there are none.
     * The shape will not be
     * swept further than the time delta provided. Shapes already intersecting
     * the sweep shape at t = 0 are ignored.
     * <br/><br/>
     * If the filter argument is null, then all shapes will be collidable
     * otherwise only those for whose filter agrees to 'collide'.
     *
     * @param shape The Shape to be cast through space. This shape must belong
     *              to a body whose velocity is used to define the sweep.
     * @param deltaTime The amount of time to sweep the shape forward.
     * @param liveSweep If true, then moving objects in the space will have their motion considered during the sweep. Otherwise; like with normal rayCast, objects in the space are considered un-moving for the cast. (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output A list to append results to instead of allocating a new one (default null)
     * @return The collision results in time order.
     * @throws # If shape is null, or not part of a body.
     * @throws # If deltaTime is negative.
     */
    #if nape_swc@:keep #end
    public function convexMultiCast(shape:Shape,deltaTime:Float,liveSweep:Bool=false,filter:InteractionFilter=null,output:ConvexResultList):ConvexResultList{
        #if(!NAPE_RELEASE_BUILD)
        if(shape==null)throw "Error: Cannot cast null shape :)";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(shape.body==null)throw "Error: Shape must belong to a body to be cast.";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(deltaTime<0||(deltaTime!=deltaTime))throw "Error: deltaTime must be positive";
        #end
        return zpp_inner.convexMultiCast(shape.zpp_inner,deltaTime,filter,liveSweep,output);
    }
    /**
     * Perform a ray cast for closest result.
     * <br/><br/>
     * This method will return only the closest result (if any), to find more
     * the first result, use the rayMultiCast method. The ray will not be
     * cast beyond its maxDistance.
     * <br/><br/>
     * If the filter argument is null, then all shapes will be intersectable
     * otherwise only those for whose filter agrees to 'collide'.
     *
     * @param ray The ray to cast through space.
     * @param inner If true then inner surfaces of shapes will also be intersected.
     *              otherwise only the outer surfaces. (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @return The closest result (if any) of ray intersection.
     * @throws # If ray is null.
     */
    #if nape_swc@:keep #end
    public function rayCast(ray:Ray,inner:Bool=false,filter:InteractionFilter=null):Null<RayResult>{
        #if(!NAPE_RELEASE_BUILD)
        if(ray==null)throw "Error: Cannot cast null ray :)";
        #end
        return zpp_inner.rayCast(ray,inner,filter);
    }
    /**
     * Perform a ray cast for all valid results.
     * <br/><br/>
     * This method will return all intersections (in distance order) of ray
     * with shapes in the space up to the ray's maxDistance.
     * <br/><br/>
     * If the filter argument is null, then all shapes will be intersectable
     * otherwise only those for whose filter agrees to 'collide'.
     *
     * @param ray The ray to cast through space.
     * @param inner If true then inner surfaces of shapes will also be intersected.
     *              otherwise only the outer surfaces. (default false)
     * @param filter Optional filter to pick and choose shapes, based on whether
     *               the filters agree to collide. (default null)
     * @param output A list to append results to instead of allocating a new one (default null)
     * @return All valid results of ray cast in distance order from closest to furthest.
     * @throws # If ray is null.
     */
    #if nape_swc@:keep #end
    public function rayMultiCast(ray:Ray,inner:Bool=false,filter:InteractionFilter=null,output:RayResultList=null):RayResultList{
        #if(!NAPE_RELEASE_BUILD)
        if(ray==null)throw "Error: Cannot cast null ray :)";
        #end
        return zpp_inner.rayMultiCast(ray,inner,filter,output);
    }
}

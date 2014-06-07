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
 * Class representing a physics Rigid Body.
 */
@:final#if nape_swc@:keep #end
class Body extends Interactor{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Body=null;
    /**
     * Set to disable debug drawing/
     * <br/><br/>
     * When true, this Body will not be drawn during debug draw operations
     * unless specifically given as argument to Debug draw() method.
     * @default true
     */
    public var debugDraw:Bool=true;
    /**
     * Type of body.
     * <br/><br/>
     * This value can be changed even if Body is inside of a Space.
     */
    #if nape_swc@:isVar #end
    public var type(get_type,set_type):BodyType;
    inline function get_type():BodyType{
        return ZPP_Body.types[zpp_inner.type];
    }
    inline function set_type(type:BodyType):BodyType{
        {
            zpp_inner.immutable_midstep("Body::type");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            if(this.type!=type){
                #if(!NAPE_RELEASE_BUILD)
                if(type==null)throw "Error: Cannot use null BodyType";
                #end
                var ntype=type==BodyType.DYNAMIC?ZPP_Flags.id_BodyType_DYNAMIC:type==BodyType.KINEMATIC?ZPP_Flags.id_BodyType_KINEMATIC:ZPP_Flags.id_BodyType_STATIC;
                if(ntype==ZPP_Flags.id_BodyType_STATIC&&zpp_inner.space!=null){
                    {
                        zpp_inner.velx=0;
                        zpp_inner.vely=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((zpp_inner.velx!=zpp_inner.velx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(zpp_inner.velx)"+") :: "+("vec_set(in n: "+"zpp_inner.vel"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((zpp_inner.vely!=zpp_inner.vely));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(zpp_inner.vely)"+") :: "+("vec_set(in n: "+"zpp_inner.vel"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    zpp_inner.angvel=0;
                }
                zpp_inner.invalidate_type();
                if(zpp_inner.space!=null)zpp_inner.space.transmitType(zpp_inner,ntype);
                else zpp_inner.type=ntype;
            }
        }
        return get_type();
    }
    /**
     * Mark object for continuous collisions against other dynamic Bodies
     * <br/><br/>
     * If true, then this Body will undergo continuous collisions with other
     * dynamic Bodies. This flag has no effect for non-dynamic Bodies.
     * <br/><br/>
     * This flag should only be set for very fast, small moving dynamic bodies,
     * and due to the way continuous collisions are resolved it is not wise to
     * enable this for a large group of bodies that interact together as it
     * will lead to visual stalling.
     * <br/>
     * Bullets also do not play well when existing in a group with respect to
     * continuous collisions against kinematic objects and may cause
     * tunnelling against the kinematic.
     * @default false
     */
    #if nape_swc@:isVar #end
    public var isBullet(get_isBullet,set_isBullet):Bool;
    inline function get_isBullet():Bool{
        return zpp_inner.bulletEnabled;
    }
    inline function set_isBullet(isBullet:Bool):Bool{
        {
            zpp_inner.bulletEnabled=isBullet;
        }
        return get_isBullet();
    }
    /**
     * Declare object should never be collided continuously
     * <br/><br/>
     * When performing continuous collisions, Nape will check both Bodies to see
     * if either has opted-out of CCD. If either Body has this flag true, then
     * no CCD will be performed for that pair.
     * @default false
     */
    #if nape_swc@:isVar #end
    public var disableCCD(get_disableCCD,set_disableCCD):Bool;
    inline function get_disableCCD():Bool{
        return zpp_inner.disableCCD;
    }
    inline function set_disableCCD(disableCCD:Bool):Bool{
        {
            zpp_inner.disableCCD=disableCCD;
        }
        return get_disableCCD();
    }
    /**
     * Integrate body forward in time, taking only velocities into account.
     *
     * @param deltaTime The time to integrate body by. This value may be negative to
     *                  integrate back in time.
     * @return A refernce to 'this' Body
     */
    #if nape_swc@:keep #end
    public function integrate(deltaTime:Float){
        #if(!NAPE_RELEASE_BUILD)
        if((deltaTime!=deltaTime))throw "Cannot integrate by NaN time";
        #end
        zpp_inner.immutable_midstep("Body::space");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        if(deltaTime==0)return this;
        var cur=this.zpp_inner;
        cur.sweepTime=0;
        cur.sweep_angvel=cur.angvel;
        cur.sweepIntegrate(deltaTime);
        cur.invalidate_pos();
        cur.invalidate_rot();
        cur.sweepTime=0;
        return this;
    }
    /**
     * Fast equivalent to <code>body.type == BodyType.STATIC</code>
     * @return True if body is Static.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isStatic(){
        return zpp_inner.isStatic();
    }
    /**
     * Fast equivalent to <code>body.type == BodyType.DYNAMIC</code>
     * @return True if body is Dynamic.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isDynamic(){
        return zpp_inner.isDynamic();
    }
    /**
     * Fast equivalent to <code>body.type == BodyType.KINEMATIC</code>
     * @return True if body is Kinematic.
     */
    #if nape_swc@:keep #end
    public#if NAPE_NO_INLINE#else inline #end
    function isKinematic(){
        return zpp_inner.isKinematic();
    }
    /**
     * List of shapes owned by Body.
     * <br/><br/>
     * Appending a Shape to this list is equivalent to <code>shape.body = this</code>
     *
     * @default []
     */
    #if nape_swc@:isVar #end
    public var shapes(get_shapes,never):ShapeList;
    inline function get_shapes():ShapeList{
        return zpp_inner.wrap_shapes;
    }
    /**
     * Compound this Body belongs to.
     * <br/><br/>
     * If this Body belongs to a Compound, then the Compound 'owns' this Body and
     * it is the Compound which would be added/removed from a Space rather than
     * this Body.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var compound(get_compound,set_compound):Null<Compound>;
    inline function get_compound():Null<Compound>{
        return if(zpp_inner.compound==null)null else zpp_inner.compound.outer;
    }
    inline function set_compound(compound:Null<Compound>):Null<Compound>{
        {
            if(this.compound!=compound){
                if(this.compound!=null)this.compound.bodies.remove(this);
                if(compound!=null)compound.bodies.add(this);
            }
        }
        return get_compound();
    }
    /**
     * Space this Body is assigned to.
     * <br/><br/>
     * When this Body is part of a Compound, this value is immutable.
     * <br/>
     * When a Body is part of a Compound it is owned by that Compound and it
     * is the Compound that is added/removed from a Space.
     */
    #if nape_swc@:isVar #end
    public var space(get_space,set_space):Null<Space>;
    inline function get_space():Null<Space>{
        return if(zpp_inner.space==null)null else zpp_inner.space.outer;
    }
    inline function set_space(space:Null<Space>):Null<Space>{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.compound!=null)throw "Error: Cannot set the space of a Body belonging to a Compound, only the root Compound space can be set";
            #end
            zpp_inner.immutable_midstep("Body::space");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            if(this.space!=space){
                if(this.space!=null)zpp_inner.component.woken=false;
                if(this.space!=null)this.space.bodies.remove(this);
                if(space!=null)space.bodies.add(this);
            }
        }
        return get_space();
    }
    /**
     * Set of active arbiters related to this Body.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var arbiters(get_arbiters,never):ArbiterList;
    inline function get_arbiters():ArbiterList{
        if(zpp_inner.wrap_arbiters==null)zpp_inner.wrap_arbiters=ZPP_ArbiterList.get(zpp_inner.arbiters,true);
        return zpp_inner.wrap_arbiters;
    }
    /**
     * Whether this body is sleeping.
     * <br/><br/>
     * This value is immutable, In Nape you do not ever need to manually wake up a Body.
     * It will always be done automatically without error.
     * <br/><br/>
     * To manually put a Body to sleep is against the very nature of Nape API
     * and so is excluded from the core of Nape. If you really want to do this
     * then you should make use of the nape-hacks module.
     */
    #if nape_swc@:isVar #end
    public var isSleeping(get_isSleeping,never):Bool;
    inline function get_isSleeping():Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.space==null)throw "Error: isSleeping makes no sense if the object is not contained within a Space";
        #end
        return zpp_inner.component.sleeping;
    }
    /**
     * Set of constraints using this Body.
     * <br/><br/>
     * This list contains those constraints that are inside of a Space only.
     * <br/><br/>
     * This list is immutable.
     */
    #if nape_swc@:isVar #end
    public var constraints(get_constraints,never):ConstraintList;
    inline function get_constraints():ConstraintList{
        if(zpp_inner.wrap_constraints==null)zpp_inner.wrap_constraints=ZPP_ConstraintList.get(zpp_inner.constraints,true);
        return zpp_inner.wrap_constraints;
    }
    /**
     * Construct a new Body.
     * <br/><br/>
     * @param type The type of Body to create. (default DYNAMIC)
     * @param position The initial position for object. (default &#40;0,0&#41;)
     * @return The newly constructed Body.
     * @throws # If position is non-null, and has been disposed of.
     */
    #if flib@:keep function flibopts_2(){}
    #end
    public function new(type:BodyType=null,position:Vec2=null){
        #if(!NAPE_RELEASE_BUILD)
        try{
            super();
        }
        catch(e:Dynamic){}
        #end
        #if NAPE_RELEASE_BUILD 
        super();
        #end
        zpp_inner=new ZPP_Body();
        zpp_inner.outer=this;
        zpp_inner.outer_i=this;
        zpp_inner_i=zpp_inner;
        if(position!=null){
            {
                #if(!NAPE_RELEASE_BUILD)
                if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            {
                zpp_inner.posx=position.x;
                zpp_inner.posy=position.y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((zpp_inner.posx!=zpp_inner.posx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(zpp_inner.posx)"+") :: "+("vec_set(in n: "+"zpp_inner.pos"+",in x: "+"position.x"+",in y: "+"position.y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((zpp_inner.posy!=zpp_inner.posy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(zpp_inner.posy)"+") :: "+("vec_set(in n: "+"zpp_inner.pos"+",in x: "+"position.x"+",in y: "+"position.y"+")");
                    #end
                };
            };
        }
        else{
            zpp_inner.posx=0;
            zpp_inner.posy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((zpp_inner.posx!=zpp_inner.posx));
                };
                if(!res)throw "assert("+"!assert_isNaN(zpp_inner.posx)"+") :: "+("vec_set(in n: "+"zpp_inner.pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((zpp_inner.posy!=zpp_inner.posy));
                };
                if(!res)throw "assert("+"!assert_isNaN(zpp_inner.posy)"+") :: "+("vec_set(in n: "+"zpp_inner.pos"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        this.type=type==null?BodyType.DYNAMIC:type;
        if(position!=null)({
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
        zpp_inner_i.insert_cbtype(CbType.ANY_BODY.zpp_inner);
    }
    /**
     * Construct an exact copy of this Body.
     * <br/><br/>
     * All properties will be exactly copied, with Shapes also
     * being copied with the copied Body's and Shape's userData
     * objects being assigned the same fields as the existing ones with
     * values copied over by reference for object types.
     *
     * @return A copy of this Body.
     */
    #if nape_swc@:keep #end
    public function copy(){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world cannot be copied";
        #end
        return zpp_inner.copy();
    }
    /**
     * Position of Body's origin in px.
     * <br/><br/>
     * This value can be set and is equivalent to: <code>this.position.set(value)</code>
     * <br/><br/>
     * Attempting to set this value on a static Body that is in a Space will result
     * in a debug build error.
     * <br/><br/>
     * Please note that for kinematic objects, setting this value is equiavalent
     * to 'teleporting' the object, and for normal movement you should be using
     * the kinematic body's velocity.
     *
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var position(get_position,set_position):Vec2;
    inline function get_position():Vec2{
        if(zpp_inner.wrap_pos==null)zpp_inner.setupPosition();
        return zpp_inner.wrap_pos;
    }
    inline function set_position(position:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(position!=null&&position.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(position==null)throw "Error: Body::"+"position"+" cannot be null";
            #end
            this.position.set(position);
        }
        return get_position();
    }
    /**
     * Linear velocity of Body's origin in px/s.
     * <br/><br/>
     * This value can be set and is equivalent to: <code>this.velocity.set(value)</code>
     * <br/><br/>
     * A static body cannot have its velocity set.
     *
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var velocity(get_velocity,set_velocity):Vec2;
    inline function get_velocity():Vec2{
        if(zpp_inner.wrap_vel==null)zpp_inner.setupVelocity();
        return zpp_inner.wrap_vel;
    }
    inline function set_velocity(velocity:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(velocity!=null&&velocity.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(velocity==null)throw "Error: Body::"+"velocity"+" cannot be null";
            #end
            this.velocity.set(velocity);
        }
        return get_velocity();
    }
    /**
     * Set velocities to achieve desired position at end of time step.
     * <br/><br/>
     * This function is a utility to help with animating kinematic bodies.
     * Kinematic bodies should be moved through velocity, but it is often
     * easier to think in terms of position.
     * <br/><br/>
     * This method will set linear and angular velocities so that the target
     * position/rotation is achieved at end of time step.
     *
     * @param targetPosition The target position for Body.
     * @param targetRotation The target rotation for Body.
     * @param deltaTime The time step for next call to space.step().
     * @throws # If targetPosition is null or disposed of.
     * @returns A reference to 'this' Body.
     */
    #if nape_swc@:keep #end
    public function setVelocityFromTarget(targetPosition:Vec2,targetRotation:Float,deltaTime:Float){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(targetPosition!=null&&targetPosition.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(targetPosition==null)throw "Cannot set velocity for null target position";
        if(deltaTime==0)throw "deltaTime cannot be 0 for setVelocityFromTarget";
        #end
        var idt=(1/deltaTime);
        this.velocity.set(targetPosition.sub(this.position,true).muleq(idt));
        this.angularVel=(targetRotation-this.rotation)*idt;
        ({
            if(({
                targetPosition.zpp_inner.weak;
            })){
                targetPosition.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Additional kinematic velocity of Body in px/s.
     * <br/><br/>
     * A bodies 'kinematic' velocity is an added velocity bias used in all physics
     * computations but that will not effect how the Body moves directly.
     * <br/><br/>
     * Even a static body can be given a kinematic velocity, and can be used for
     * such things as giving a body of water a fluid-velocity for fluid drag
     * computations.
     *
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var kinematicVel(get_kinematicVel,set_kinematicVel):Vec2;
    inline function get_kinematicVel():Vec2{
        if(zpp_inner.wrap_kinvel==null)zpp_inner.setupkinvel();
        return zpp_inner.wrap_kinvel;
    }
    inline function set_kinematicVel(kinematicVel:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(kinematicVel!=null&&kinematicVel.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(kinematicVel==null)throw "Error: Body::"+"kinematicVel"+" cannot be null";
            #end
            this.kinematicVel.set(kinematicVel);
        }
        return get_kinematicVel();
    }
    /**
     * Additional surface velocity for Body in px/s.
     * <br/><br/>
     * A bodies 'surface' velocity is an added velocity bias that is rotated to match
     * the angle of the contact surface used in contact physics and will not
     * effect how the Body moves directly.
     * <br/><br/>
     * Even a static body can be given a surface velocity, and can be used for
     * such things as conveyor belts (By setting the x-component of surfaceVel).
     *
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var surfaceVel(get_surfaceVel,set_surfaceVel):Vec2;
    inline function get_surfaceVel():Vec2{
        if(zpp_inner.wrap_svel==null)zpp_inner.setupsvel();
        return zpp_inner.wrap_svel;
    }
    inline function set_surfaceVel(surfaceVel:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(surfaceVel!=null&&surfaceVel.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(surfaceVel==null)throw "Error: Body::"+"surfaceVel"+" cannot be null";
            #end
            this.surfaceVel.set(surfaceVel);
        }
        return get_surfaceVel();
    }
    /**
     * Accumulated force acting on body in px.kg/s/s
     * <br/><br/>
     * This value is not used internally for any physics computations.
     * <br/><br/>
     * You may set this property only on dynamic bodies.
     *
     * @default (0,0)
     */
    #if nape_swc@:isVar #end
    public var force(get_force,set_force):Vec2;
    inline function get_force():Vec2{
        if(zpp_inner.wrap_force==null)zpp_inner.setupForce();
        return zpp_inner.wrap_force;
    }
    inline function set_force(force:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(force!=null&&force.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(force==null)throw "Error: Body::"+"force"+" cannot be null";
            #end
            this.force.set(force);
        }
        return get_force();
    }
    /**
     * This property represents the velocity seen by constraint physics.
     * <br/><br/>
     * You should not need to use this property unless writing your own
     * constraints using the UserConstraint API.
     */
    #if nape_swc@:isVar #end
    public var constraintVelocity(get_constraintVelocity,never):Vec3;
    inline function get_constraintVelocity():Vec3{
        if(zpp_inner.wrapcvel==null)zpp_inner.setup_cvel();
        return zpp_inner.wrapcvel;
    }
    /**
     * Rotation of Body in clockwise rad.
     * <br/><br/>
     * Attempting to set this value on a static Body that is in a Space will result
     * in a debug build error.
     * <br/><br/>
     * Please note that for kinematic objects, setting this value is equiavalent
     * to 'teleporting' the object, and for normal movement you should be using
     * the kinematic body's angularVel.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var rotation(get_rotation,set_rotation):Float;
    inline function get_rotation():Float{
        return zpp_inner.rot;
    }
    inline function set_rotation(rotation:Float):Float{
        {
            zpp_inner.immutable_midstep("Body::rotation");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(isStatic()&&zpp_inner.space!=null)throw "Error: Static objects cannot be rotated once inside a Space";
            #end
            if(this.rotation!=rotation){
                #if(!NAPE_RELEASE_BUILD)
                if((rotation!=rotation))throw "Error: Body::rotation cannot be NaN";
                #end
                zpp_inner.rot=rotation;
                zpp_inner.invalidate_rot();
                zpp_inner.wake();
            }
        }
        return get_rotation();
    }
    /**
     * Angular velocity of Body in clockwise rad/s
     * <br/><br/>
     * A static body cannot have its angular velocity set.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var angularVel(get_angularVel,set_angularVel):Float;
    inline function get_angularVel():Float{
        return zpp_inner.angvel;
    }
    inline function set_angularVel(angularVel:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            if(this.angularVel!=angularVel){
                #if(!NAPE_RELEASE_BUILD)
                if((angularVel!=angularVel))throw "Error: Body::angularVel cannot be NaN";
                #end
                #if(!NAPE_RELEASE_BUILD)
                if(zpp_inner.isStatic())throw "Error: A static object cannot be given a velocity";
                #end
                zpp_inner.angvel=angularVel;
                zpp_inner.invalidate_wake();
            }
        }
        return get_angularVel();
    }
    /**
     * Additional kinematic angular velocity of Body in rad/s.
     * <br/><br/>
     * A bodies 'kinematic' velocity is an added velocity bias used in all physics
     * computations but that will not effect how the Body moves directly.
     * <br/><br/>
     * Even a static body can be given a kinematic velocity, and can be used for
     * such things as giving a body of water a fluid-velocity for fluid drag
     * computations.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var kinAngVel(get_kinAngVel,set_kinAngVel):Float;
    inline function get_kinAngVel():Float{
        return zpp_inner.kinangvel;
    }
    inline function set_kinAngVel(kinAngVel:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            if(this.kinAngVel!=kinAngVel){
                #if(!NAPE_RELEASE_BUILD)
                if((kinAngVel!=kinAngVel))throw "Error: Body::kinAngVel cannot be NaN";
                #end
                zpp_inner.kinangvel=kinAngVel;
                zpp_inner.invalidate_wake();
            }
        }
        return get_kinAngVel();
    }
    /**
     * Accumulated torque acting on body in px.px.kg/s/s
     * <br/><br/>
     * This value is not used internally for any physics computations.
     * <br/><br/>
     * You may set this property only on dynamic bodies.
     *
     * @default 0
     */
    #if nape_swc@:isVar #end
    public var torque(get_torque,set_torque):Float;
    inline function get_torque():Float{
        return zpp_inner.torque;
    }
    inline function set_torque(torque:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(!isDynamic())throw "Error: Non-dynamic body cannot have torque applied.";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((torque!=torque))throw "Error: Body::torque cannot be NaN";
            #end
            if(this.torque!=torque){
                zpp_inner.torque=torque;
                zpp_inner.invalidate_wake();
            }
        }
        return get_torque();
    }
    /**
     * Bounding box of Body in world space.
     * <br/><br/>
     * This value can be accessed even if there are no Shapes in the Body, but
     * attempting to query its values whilst there are no Shapes will result
     * in a debug build error.
     * <br/><br/>
     * This AABB is immutable.
     */
    #if nape_swc@:isVar #end
    public var bounds(get_bounds,never):AABB;
    inline function get_bounds():AABB{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no bounds";
        #end
        return zpp_inner.aabb.wrapper();
    }
    /**
     * Whether dynamic Body is permitted to be moved by physics linearly.
     * <br/><br/>
     * When this field is false, no physics will be able to cause a change in the
     * bodies linear velocity (It can still move, but only if you tell it to like
     * a kinematic body).
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var allowMovement(get_allowMovement,set_allowMovement):Bool;
    inline function get_allowMovement():Bool{
        return!zpp_inner.nomove;
    }
    inline function set_allowMovement(allowMovement:Bool):Bool{
        {
            zpp_inner.immutable_midstep("Body::"+allowMovement);
            if(this.allowMovement!=allowMovement){
                zpp_inner.nomove=!allowMovement;
                zpp_inner.invalidate_mass();
            }
        }
        return get_allowMovement();
    }
    /**
     * Whether dynamic Body is permitted to be rotated by physics.
     * <br/><br/>
     * When this field is false, no physics will be able to cause a change in the
     * bodies angular velocity (It can still rotate, but only if you tell it to like
     * a kinematic body).
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var allowRotation(get_allowRotation,set_allowRotation):Bool;
    inline function get_allowRotation():Bool{
        return!zpp_inner.norotate;
    }
    inline function set_allowRotation(allowRotation:Bool):Bool{
        {
            zpp_inner.immutable_midstep("Body::"+allowRotation);
            if(this.allowRotation!=allowRotation){
                zpp_inner.norotate=!allowRotation;
                zpp_inner.invalidate_inertia();
            }
        }
        return get_allowRotation();
    }
    /**
     * Method of mass computation for Body.
     * <br/><br/>
     * This value will be set implicitly to FIXED when mass property is set.
     * <br/>Setting back to DEFAULT will then set mass implicitly back to the default
     * computed mass.
     *
     * @default MassMode.DEFAULT
     */
    #if nape_swc@:isVar #end
    public var massMode(get_massMode,set_massMode):MassMode;
    inline function get_massMode():MassMode{
        return[MassMode.DEFAULT,MassMode.FIXED][zpp_inner.massMode];
    }
    inline function set_massMode(massMode:MassMode):MassMode{
        {
            zpp_inner.immutable_midstep("Body::massMode");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(massMode==null)throw "Error: cannot use null massMode";
            #end
            zpp_inner.massMode=massMode==MassMode.DEFAULT?ZPP_Flags.id_MassMode_DEFAULT:ZPP_Flags.id_MassMode_FIXED;
            zpp_inner.invalidate_mass();
        }
        return get_massMode();
    }
    /**
     * Mass to be used for a Body in User built constraints.
     * <br/><br/>
     * This value is given as the inverse mass of the Body taking into account
     * Body type (Static and Kinematic Bodies will have constraintMass of 0)
     * as well as properties like allowMovement.
     */
    #if nape_swc@:isVar #end
    public var constraintMass(get_constraintMass,never):Float;
    inline function get_constraintMass():Float{
        if(!zpp_inner.world)zpp_inner.validate_mass();
        return zpp_inner.smass;
    }
    /**
     * Mass of the Body.
     * <br/><br/>
     * This value is computed by default based on the Body's Shape's areas and
     * Material densities.
     * <br/>
     * When massMode is DEFAULT, accessing this value for an empty Body will thus
     * give an error as the value is undefined.
     * <br/><br/>
     * Setting this value will permit you to give a fixed mass to the Body
     * implicitly changing the massMode to MassMode.FIXED
     */
    #if nape_swc@:isVar #end
    public var mass(get_mass,set_mass):Float;
    inline function get_mass():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no mass";
        #end
        zpp_inner.validate_mass();
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.massMode==ZPP_Flags.id_MassMode_DEFAULT&&zpp_inner.shapes.empty())throw "Error: Given current mass mode, Body::mass only makes sense if it contains shapes";
        #end
        return zpp_inner.cmass;
    }
    inline function set_mass(mass:Float):Float{
        {
            zpp_inner.immutable_midstep("Body::mass");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((mass!=mass))throw "Error: Mass cannot be NaN";
            if(mass<=0)throw "Error: Mass must be strictly positive";
            if(mass>=ZPP_Const.POSINF())throw "Error: Mass cannot be infinite, use allowMovement = false instead";
            #end
            zpp_inner.massMode=ZPP_Flags.id_MassMode_FIXED;
            zpp_inner.cmass=mass;
            zpp_inner.invalidate_mass();
        }
        return get_mass();
    }
    /**
     * Method of computing mass as seen by gravity.
     * <br/><br/>
     * This value will be implicitly set by modifying gravMass or gravMassScale properties.
     *
     * @default GravMassMode.DEFAULT
     */
    #if nape_swc@:isVar #end
    public var gravMassMode(get_gravMassMode,set_gravMassMode):GravMassMode;
    inline function get_gravMassMode():GravMassMode{
        return[GravMassMode.DEFAULT,GravMassMode.FIXED,GravMassMode.SCALED][zpp_inner.massMode];
    }
    inline function set_gravMassMode(gravMassMode:GravMassMode):GravMassMode{
        {
            zpp_inner.immutable_midstep("Body::gravMassMode");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(gravMassMode==null)throw "Error: Cannot use null gravMassMode";
            #end
            zpp_inner.gravMassMode=gravMassMode==GravMassMode.SCALED?ZPP_Flags.id_GravMassMode_SCALED:gravMassMode==GravMassMode.DEFAULT?ZPP_Flags.id_GravMassMode_DEFAULT:ZPP_Flags.id_GravMassMode_FIXED;
            zpp_inner.invalidate_gravMass();
        }
        return get_gravMassMode();
    }
    /**
     * Mass used in gravity computations in a Space.
     * <br/><br/>
     * Setting this value will implicitly change the gravMassMode to FIXED.
     * <br/>
     * Set to 0 to disable gravity for this Body.
     */
    #if nape_swc@:isVar #end
    public var gravMass(get_gravMass,set_gravMass):Float;
    inline function get_gravMass():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no gravMass";
        #end
        zpp_inner.validate_gravMass();
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.shapes.empty()){
            if(zpp_inner.massMode==ZPP_Flags.id_MassMode_DEFAULT&&zpp_inner.gravMassMode!=ZPP_Flags.id_GravMassMode_FIXED)throw "Error: Given current mass/gravMass modes; Body::gravMass only makes sense if it contains Shapes";
        }
        #end
        return zpp_inner.gravMass;
    }
    inline function set_gravMass(gravMass:Float):Float{
        {
            zpp_inner.immutable_midstep("Body::gravMass");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((gravMass!=gravMass))throw "Error: gravMass cannot be NaN";
            #end
            zpp_inner.gravMassMode=ZPP_Flags.id_GravMassMode_FIXED;
            zpp_inner.gravMass=gravMass;
            zpp_inner.invalidate_gravMass();
        }
        return get_gravMass();
    }
    /**
     * Mass scale used in computation of gravity for Body in Space.
     * <br/><br/>
     * Setting this value will implicitly change the gravMassMode to SCALED.
     * <br/>
     * When set, the gravMass of Body will be computed as this scaling factor
     * multiplied with the Body's mass.
     */
    #if nape_swc@:isVar #end
    public var gravMassScale(get_gravMassScale,set_gravMassScale):Float;
    inline function get_gravMassScale():Float{
        zpp_inner.validate_gravMassScale();
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.shapes.empty()){
            if(zpp_inner.massMode==ZPP_Flags.id_MassMode_DEFAULT&&zpp_inner.gravMassMode!=ZPP_Flags.id_GravMassMode_SCALED)throw "Error: Given current mass/gravMass modes; Body::gravMassScale only makes sense if it contains Shapes";
        }
        #end
        return zpp_inner.gravMassScale;
    }
    inline function set_gravMassScale(gravMassScale:Float):Float{
        {
            zpp_inner.immutable_midstep("Body::gravMassScale");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((gravMassScale!=gravMassScale))throw "Error: gravMassScale cannot be NaN";
            #end
            zpp_inner.gravMassMode=ZPP_Flags.id_GravMassMode_SCALED;
            zpp_inner.gravMassScale=gravMassScale;
            zpp_inner.invalidate_gravMassScale();
        }
        return get_gravMassScale();
    }
    /**
     * Method of computing Body moment of inertia.
     * <br/><br/>
     * This value will be set implicitly by modifying Body inertia property.
     *
     * @default InertiaMode.DEFAULT
     */
    #if nape_swc@:isVar #end
    public var inertiaMode(get_inertiaMode,set_inertiaMode):InertiaMode;
    inline function get_inertiaMode():InertiaMode{
        return[InertiaMode.DEFAULT,InertiaMode.FIXED][zpp_inner.inertiaMode];
    }
    inline function set_inertiaMode(inertiaMode:InertiaMode):InertiaMode{
        {
            zpp_inner.immutable_midstep("Body::inertiaMode");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if(inertiaMode==null)throw "Error: Cannot use null InertiaMode";
            #end
            zpp_inner.inertiaMode=inertiaMode==InertiaMode.FIXED?ZPP_Flags.id_InertiaMode_FIXED:ZPP_Flags.id_InertiaMode_DEFAULT;
            zpp_inner.invalidate_inertia();
        }
        return get_inertiaMode();
    }
    /**
     * Moment of inertia to be used in user defined Constraints.
     * <br/><br/>
     * This value is equal to the inverse inertia of the Body taking into account
     * Body type (Static and Kinematic bodies will have constraintInertia of 0).
     * As well as properties like allowRotation.
     */
    #if nape_swc@:isVar #end
    public var constraintInertia(get_constraintInertia,never):Float;
    inline function get_constraintInertia():Float{
        if(!zpp_inner.world)zpp_inner.validate_inertia();
        return zpp_inner.sinertia;
    }
    /**
     * Moment of inertia of this Body.
     * <br/><br/>
     * Setting this value will implicitly change the inertiaMode to FIXED.
     */
    #if nape_swc@:isVar #end
    public var inertia(get_inertia,set_inertia):Float;
    inline function get_inertia():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no inertia";
        #end
        zpp_inner.validate_inertia();
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inertiaMode==ZPP_Flags.id_InertiaMode_DEFAULT&&shapes.empty())throw "Error: Given current inertia mode flag, Body::inertia only makes sense if Body contains Shapes";
        #end
        return zpp_inner.cinertia;
    }
    inline function set_inertia(inertia:Float):Float{
        {
            zpp_inner.immutable_midstep("Body::inertia");
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.world)throw "Error: Space::world is immutable";
            #end
            #if(!NAPE_RELEASE_BUILD)
            if((inertia!=inertia))throw "Error: Inertia cannot be NaN";
            if(inertia<=0)throw "Error: Inertia must be strictly positive";
            if(inertia>=ZPP_Const.POSINF())throw "Error: Inertia cannot be infinite, use allowRotation = false instead";
            #end
            zpp_inner.inertiaMode=ZPP_Flags.id_InertiaMode_FIXED;
            zpp_inner.cinertia=inertia;
            zpp_inner.invalidate_inertia();
        }
        return get_inertia();
    }
    /**
     * Compute set of bodies connected via constraints.
     * <br/><br/>
     * Only constraints that are inside of a Space will be considered the
     * same way that the body's constraints list only tracks constraints
     * that are part of a simulation.
     *
     * @param depth Control the depth limit of the graph search. Negative
     *              values indicate an unlimited search. A depth value of
     *              0 would cause only the current Body to be returned.
     *              (default -1)
     * @param output An optional list to append results to, if left as null
     *               then a new list is created.
     * @return A list of the connected bodies up to
     *         the given graph depth.
     */
    #if nape_swc@:keep #end
    public function connectedBodies(depth:Int=-1,output:BodyList=null):BodyList{
        return zpp_inner.connectedBodies(depth,output);
    }
    /**
     * Compute set of bodies interacting with this body.
     *
     * @param type When not equal to null, this parameter controls what sort
     *             of interaction we permit in the search.
     * @param depth Control the depth limit of the graph search. Negative
     *              values indicate an unlimited search. A depth value of
     *              0 would cause only the current Body to be returned.
     *              (default -1)
     * @param output An optional list to append results to, if left as null
     *               then a new list is created.
     * @return A list of the interacting bodies up to
     *         the given graph depth.
     */
    #if nape_swc@:keep #end
    public function interactingBodies(type:InteractionType=null,depth:Int=-1,output:BodyList=null){
        var arbiter_type=if(type==null){
            ZPP_Arbiter.COL|ZPP_Arbiter.SENSOR|ZPP_Arbiter.FLUID;
        }
        else if(type==InteractionType.COLLISION)ZPP_Arbiter.COL else if(type==InteractionType.SENSOR)ZPP_Arbiter.SENSOR else ZPP_Arbiter.FLUID;
        return zpp_inner.interactingBodies(arbiter_type,depth,output);
    }
    /**
     * Determine how much this body is being crushed.
     * <br/><br/>
     * This is an approximate value, computed as:
     * <code>crushFactor = (sum(magnitude(impulse)) - magnitude(sum(impulse))) / mass</code>
     * <br/><br/>
     * In this way, it is a mass and time step invariant value which is 0 when all impulses
     * are acting on body in the same direction, and has maximum value when impulses
     * act in opposing directions 'crushing' the Body.
     *
     * @return A positive value representing an approximation to how much the
     *         body is being crushed.
     * @throws # If body is not in a Space.
     */
    #if nape_swc@:keep #end
    public function crushFactor():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(space==null)throw "Error: Makes no sense to see how much an object not taking part in a simulation is being crushed";
        #end
        var msum=0.0;
        var jsum=Vec2.get();
        for(arb in arbiters){
            var imp3=arb.totalImpulse(this);
            var imp=imp3.xy();
            jsum.addeq(imp);
            msum+=imp.length;
            imp.dispose();
            imp3.dispose();
        }
        for(con in constraints){
            var imp3=con.bodyImpulse(this);
            var imp=imp3.xy();
            jsum.addeq(imp);
            msum+=imp.length;
            imp.dispose();
            imp3.dispose();
        }
        var ret=(msum-jsum.length)/(this.mass*space.zpp_inner.pre_dt);
        jsum.dispose();
        return ret;
    }
    /**
     * Transform a point from Body's local coordinates to world coordinates.
     *
     * @param point The point to transform.
     * @param weak If true the returned Vec2 will be automatically released
     *             back to object pool when used as an argument to a Nape function.
     *             (default false)
     * @return The result of the transformation.
     * @throws # If point is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function localPointToWorld(point:Vec2,weak:Bool=false){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Error: Cannot transform null Vec2";
        #end
        zpp_inner.validate_axis();
        var tempx:Float=0.0;
        var tempy:Float=0.0;
        {
            tempx=(zpp_inner.axisy*point.x-zpp_inner.axisx*point.y);
            tempy=(point.x*zpp_inner.axisx+point.y*zpp_inner.axisy);
        };
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
        return Vec2.get(tempx+zpp_inner.posx,tempy+zpp_inner.posy,weak);
    }
    /**
     * Transform a point from world coordinates to Body's local coordinates.
     *
     * @param point The point to transform.
     * @param weak If true the returned Vec2 will be automatically released
     *             back to object pool when used as an argument to a Nape function.
     *             (default false)
     * @return The result of the transformation.
     * @throws # If point is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function worldPointToLocal(point:Vec2,weak:Bool=false){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Error: Cannot transform null Vec2";
        #end
        zpp_inner.validate_axis();
        var tempx:Float=0.0;
        var tempy:Float=0.0;
        var pointx:Float=0.0;
        var pointy:Float=0.0;
        {
            pointx=point.x-zpp_inner.posx;
            pointy=point.y-zpp_inner.posy;
        };
        {
            tempx=pointx*zpp_inner.axisy+pointy*zpp_inner.axisx;
            tempy=pointy*zpp_inner.axisy-pointx*zpp_inner.axisx;
        };
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
        return Vec2.get(tempx,tempy,weak);
    }
    /**
     * Transform vector from Body's local coordinates into world coordinates.
     * <br/><br/>
     *
     * @param vector The vector to transform.
     * @param weak If true the returned Vec2 will be automatically released
     *             back to object pool when used as an argument to a Nape function.
     *             (default false)
     * @return The result of the transformation.
     * @throws # If vector is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function localVectorToWorld(vector:Vec2,weak:Bool=false){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null)throw "Error: Cannot transform null Vec2";
        #end
        zpp_inner.validate_axis();
        var tempx:Float=0.0;
        var tempy:Float=0.0;
        {
            tempx=(zpp_inner.axisy*vector.x-zpp_inner.axisx*vector.y);
            tempy=(vector.x*zpp_inner.axisx+vector.y*zpp_inner.axisy);
        };
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
        return Vec2.get(tempx,tempy,weak);
    }
    /**
     * Transform vector from world coordinates to Body's local coordinates
     * <br/><br/>
     *
     * @param vector The vector to transform.
     * @param weak If true the returned Vec2 will be automatically released
     *             back to object pool when used as an argument to a Nape function.
     *             (default false)
     * @return The result of the transformation.
     * @throws # If vector is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function worldVectorToLocal(vector:Vec2,weak:Bool=false){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(vector!=null&&vector.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(vector==null)throw "Error: Cannot transform null Vec2";
        #end
        zpp_inner.validate_axis();
        var tempx:Float=0.0;
        var tempy:Float=0.0;
        {
            tempx=vector.x*zpp_inner.axisy+vector.y*zpp_inner.axisx;
            tempy=vector.y*zpp_inner.axisy-vector.x*zpp_inner.axisx;
        };
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
        return Vec2.get(tempx,tempy,weak);
    }
    /**
     * Apply impulse to a point on Body.
     * <br/><br/>
     * If position argument is not given, then body.position is assumed so that impulse
     * is applied at centre of Body.
     *
     * @param impulse The impulse to apply given in world coordinates.
     * @param pos The position to apply impulse given in world coordinates.
     *            (default body.position)
     * @param sleepable This parameter can be set to true, in the case that you
     *                  are constantly applying an impulse which is dependent only
     *                  on the position/velocity of the body meaning that application
     *                  of this impulse does not need to prevent the object from sleeping.
     *                  When true, and the body is sleeping, this method call will not
     *                  apply any impulse.
     *                  (default false).
     * @throws # If impulse is null or disposed of.
     * @throws # If pos is non-null and disposed of.
     * @returns A reference to 'this' Body.
     */
    #if nape_swc@:keep #end
    public function applyImpulse(impulse:Vec2,pos:Vec2=null,sleepable:Bool=false){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(impulse!=null&&impulse.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        {
            #if(!NAPE_RELEASE_BUILD)
            if(pos!=null&&pos.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(impulse==null)throw "Error: Cannot apply null impulse to Body";
        #end
        if(sleepable&&isSleeping){
            ({
                if(({
                    impulse.zpp_inner.weak;
                })){
                    impulse.dispose();
                    true;
                }
                else{
                    false;
                }
            });
            if(pos!=null){
                ({
                    if(({
                        pos.zpp_inner.weak;
                    })){
                        pos.dispose();
                        true;
                    }
                    else{
                        false;
                    }
                });
            }
            return this;
        }
        zpp_inner.validate_mass();
        {
            var t=(zpp_inner.imass);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"zpp_inner.vel"+",in b: "+"impulse."+",in s: "+"zpp_inner.imass"+")");
                #end
            };
            zpp_inner.velx+=impulse.x*t;
            zpp_inner.vely+=impulse.y*t;
        };
        if(pos!=null){
            var rx:Float=0.0;
            var ry:Float=0.0;
            {
                rx=pos.x-zpp_inner.posx;
                ry=pos.y-zpp_inner.posy;
            };
            zpp_inner.validate_inertia();
            zpp_inner.angvel+=(impulse.y*rx-impulse.x*ry)*zpp_inner.iinertia;
            ({
                if(({
                    pos.zpp_inner.weak;
                })){
                    pos.dispose();
                    true;
                }
                else{
                    false;
                }
            });
        }
        if(!sleepable){
            if(isDynamic())zpp_inner.invalidate_wake();
        }
        ({
            if(({
                impulse.zpp_inner.weak;
            })){
                impulse.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Apply a pure angular impulse to Body.
     *
     * @param impulse The angular impulse to apply.
     * @param sleepable This parameter can be set to true, in the case that you
     *                  are constantly applying an impulse which is dependent only
     *                  on the position/velocity of the body meaning that application
     *                  of this impulse does not need to prevent the object from sleeping.
     *                  When true, and the body is sleeping, this method call will not
     *                  apply any impulse.
     *                  (default false).
     * @returns A reference to 'this' Body.
     */
    #if nape_swc@:keep #end
    public function applyAngularImpulse(impulse:Float,sleepable:Bool=false){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        if(sleepable&&isSleeping){
            return this;
        }
        zpp_inner.validate_inertia();
        zpp_inner.angvel+=impulse*zpp_inner.iinertia;
        if(!sleepable){
            if(isDynamic())zpp_inner.invalidate_wake();
        }
        return this;
    }
    /**
     * Translate each shape in local coordinates.
     * <br/><br/>
     * This operation does not effect the Body's position, but the position
     * of the shapes 'inside' of the Body.
     *
     * @param translation The local translation to apply to Shapes.
     * @return A reference to this Body.
     * @throws # If translation is null or has been disposed of.
     * @throws # If this Body is static, and inside of a Space.
     */
    #if nape_swc@:keep #end
    public function translateShapes(translation:Vec2){
        zpp_inner.immutable_midstep("Body::translateShapes()");
        {
            #if(!NAPE_RELEASE_BUILD)
            if(translation!=null&&translation.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(translation==null)throw "Error: Cannot displace by null Vec2";
        #end
        var weak=translation.zpp_inner.weak;
        translation.zpp_inner.weak=false;
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.translate(translation);
                cx_ite=cx_ite.next;
            }
        };
        translation.zpp_inner.weak=weak;
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
     * Rotate each shape in local coordinates.
     * <br/><br/>
     * This operation does not effect the Body's rotation, but rotates
     * each of the shapes 'inside' of the Body.
     *
     * @param angle The angle to rotate shapes by in clockwise radians.
     * @return A reference to this Body.
     * @throws # If this Body is static, and inside of a Space.
     */
    #if nape_swc@:keep #end
    public function rotateShapes(angle:Float){
        zpp_inner.immutable_midstep("Body::rotateShapes()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.rotate(angle);
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Scale each shape in local coordinates.
     * <br/><br/>
     * This operation does not affect the Body itself, but affects each
     * Shape 'inside' of the Body instead.
     *
     * @param scaleX The x-coordinate factor of scaling.
     * @param scaleY The y-coordinate factor of scaling.
     * @return A reference to this Body.
     * @throws # If this Body is static, and inside of a Space.
     * @throws # If Body contains Circle shapes, and scaleX != scaleY
     */
    #if nape_swc@:keep #end
    public function scaleShapes(scaleX:Float,scaleY:Float){
        zpp_inner.immutable_midstep("Body::scaleShapes()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.scale(scaleX,scaleY);
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Transform each shape in local coordiantes.
     * <br/><br/>
     * This operation does not affect the Body itself, but affects each
     * Shape 'inside' of the Body instead.
     *
     * @param matrix The transformation matrix to apply to each Shape.
     * @return A reference to this Body.
     * @throws # If this Body is static, and inside of a Space.
     * @throws # If matrix is null or singular.
     * @throws # If Body contains Circle shapes, and input matrix is
     *           not equiorthogonal.
     */
    #if nape_swc@:keep #end
    public function transformShapes(matrix:Mat23){
        zpp_inner.immutable_midstep("Body::transformShapes()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.transform(matrix);
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Align rigid body so that its origin is also its centre of mass.
     * <br/><br/>
     * This operation will both translate the Shapes inside of the Body,
     * as well as translating the Body itself so that its 'apparent' position
     * has not been modified.
     * <br/><br/>
     * Alignment of Rigid bodies is necessary for dynamic bodies so that
     * they will interact and rotate as expected.
     * <br/><br/>
     * Simple Body's created with a single Polygon.box() or basic Circle
     * will already be aligned.
     *
     * @return A reference to this Body.
     */
    #if nape_swc@:keep #end
    public function align(){
        zpp_inner.immutable_midstep("Body::align()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.shapes.empty())throw "Error: Cannot align empty Body";
        #end
        zpp_inner.validate_localCOM();
        var dx=Vec2.get(-zpp_inner.localCOMx,-zpp_inner.localCOMy);
        translateShapes(dx);
        var dx2=localVectorToWorld(dx);
        position.subeq(dx2);
        if(zpp_inner.pre_posx<ZPP_Const.POSINF()){
            var t=(1.0);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"zpp_inner.pre_pos"+",in b: "+"dx2."+",in s: "+"1.0"+")");
                #end
            };
            zpp_inner.pre_posx-=dx2.x*t;
            zpp_inner.pre_posy-=dx2.y*t;
        };
        dx.dispose();
        dx2.dispose();
        return this;
    }
    /**
     * Rotate body about about given point.
     * <br/><br/>
     * Please note that this method is equivalent to teleporting the body,
     * the same way direct manipulation of position and rotation is.
     *
     * @param centre The centre of rotation in world coordinates.
     * @param angle The angle to rotate body by in clockwise radians.
     * @return A reference to this Body.
     * @throws # If this Body is static, and inside of a Space.
     * @throws # If centre is null or disposed of.
     */
    #if nape_swc@:keep #end
    public function rotate(centre:Vec2,angle:Float){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(centre!=null&&centre.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(centre==null)throw "Error: Cannot rotate about a null Vec2";
        #end
        #if(!NAPE_RELEASE_BUILD)
        if((angle!=angle))throw "Error: Cannot rotate by NaN radians";
        #end
        var weak=centre.zpp_inner.weak;
        centre.zpp_inner.weak=false;
        var del=position.sub(centre);
        del.rotate(angle);
        position=centre.add(del,true);
        del.dispose();
        rotation+=angle;
        centre.zpp_inner.weak=weak;
        ({
            if(({
                centre.zpp_inner.weak;
            })){
                centre.dispose();
                true;
            }
            else{
                false;
            }
        });
        return this;
    }
    /**
     * Set material of all shapes.
     * <br/><br/>
     * Equivalent to: <code>body.shapes.foreach(function (shape) shape.material = material)</code>
     *
     * @param material The material to set Shape's material to.
     * @return A reference to this Body.
     */
    #if nape_swc@:keep #end
    public function setShapeMaterials(material:Material){
        zpp_inner.immutable_midstep("Body::setShapeMaterials()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.material=material;
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Set interaction filter of all shapes.
     * <br/><br/>
     * Equivalent to: <code>body.shapes.foreach(function (shape) shape.filter = filter)</code>
     *
     * @param filter The filter to set Shape's filter to.
     * @return A reference to this Body.
     */
    #if nape_swc@:keep #end
    public function setShapeFilters(filter:InteractionFilter){
        zpp_inner.immutable_midstep("Body::setShapeFilters()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.filter=filter;
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Set fluidProperties of all shapes.
     * <br/><br/>
     * Equivalent to: <code>body.shapes.foreach(function (shape) shape.fluidProperties = fluidProperties)</code>
     *
     * @param fluidProperties The fluidProperties to set Shape's fluidProperties to.
     * @return A reference to this Body.
     */
    #if nape_swc@:keep #end
    public function setShapeFluidProperties(fluidProperties:FluidProperties){
        zpp_inner.immutable_midstep("Body::setShapeFluidProperties()");
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world is immutable";
        #end
        {
            var cx_ite=zpp_inner.shapes.begin();
            while(cx_ite!=null){
                var s=cx_ite.elem();
                s.outer.fluidProperties=fluidProperties;
                cx_ite=cx_ite.next;
            }
        };
        return this;
    }
    /**
     * Local centre of mass of Body.
     * <br/><br/>
     * This value can be accessed even if Body has no shapes, but attempting
     * to query its values will result in a debug build error.
     * <br/><br/>
     * This Vec2 is immutable.
     */
    #if nape_swc@:isVar #end
    public var localCOM(get_localCOM,never):Vec2;
    inline function get_localCOM():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no "+"localCOM";
        #end
        if(zpp_inner.wrap_localCOM==null){
            zpp_inner.wrap_localCOM=Vec2.get(zpp_inner.localCOMx,zpp_inner.localCOMy);
            zpp_inner.wrap_localCOM.zpp_inner._inuse=true;
            zpp_inner.wrap_localCOM.zpp_inner._immutable=true;
            zpp_inner.wrap_localCOM.zpp_inner._validate=zpp_inner.getlocalCOM;
        }
        return zpp_inner.wrap_localCOM;
    }
    /**
     * World centre of mass of Body.
     * <br/><br/>
     * This value can be accessed even if Body has no shapes, but attempting
     * to query its values will result in a debug build error.
     * <br/><br/>
     * This Vec2 is immutable.
     */
    #if nape_swc@:isVar #end
    public var worldCOM(get_worldCOM,never):Vec2;
    inline function get_worldCOM():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.world)throw "Error: Space::world has no "+"worldCOM";
        #end
        if(zpp_inner.wrap_worldCOM==null){
            zpp_inner.wrap_worldCOM=Vec2.get(zpp_inner.worldCOMx,zpp_inner.worldCOMy);
            zpp_inner.wrap_worldCOM.zpp_inner._inuse=true;
            zpp_inner.wrap_worldCOM.zpp_inner._immutable=true;
            zpp_inner.wrap_worldCOM.zpp_inner._validate=zpp_inner.getworldCOM;
        }
        return zpp_inner.wrap_worldCOM;
    }
    /**
     * Evaluate sum effect of all normal contact impulses on Body.
     * <br/><br/>
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @param freshOnly If true, then only 'new' contact points will be considered.
     *             (default false)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function normalImpulse(body:Body=null,freshOnly:Bool=false){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.COL)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().collisionArbiter.normalImpulse(this,freshOnly);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all tangent contact impulses on Body.
     * <br/><br/>
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @param freshOnly If true, then only 'new' contact points will be considered.
     *             (default false)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function tangentImpulse(body:Body=null,freshOnly:Bool=false){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.COL)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().collisionArbiter.tangentImpulse(this,freshOnly);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all contact impulses on Body.
     * <br/><br/>
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @param freshOnly If true, then only 'new' contact points will be considered.
     *             (default false)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function totalContactsImpulse(body:Body=null,freshOnly:Bool=false){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.COL)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().collisionArbiter.totalImpulse(this,freshOnly);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all rolling friction contact impulses on Body.
     * <br/><br/
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @param freshOnly If true, then only 'new' contact points will be considered.
     *             (default false)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function rollingImpulse(body:Body=null,freshOnly:Bool=false){
        var ret=0.0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.COL)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    ret+=arb.wrapper().collisionArbiter.rollingImpulse(this,freshOnly);
                };
            }
        };
        return ret;
    }
    /**
     * Evaluate sum effect of all buoyancy impulses acting on Body.
     * <br/><br/
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function buoyancyImpulse(body:Body=null){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.FLUID)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().fluidArbiter.buoyancyImpulse(this);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all fluid drag impulses acting on Body.
     * <br/><br/
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function dragImpulse(body:Body=null){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.FLUID)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().fluidArbiter.dragImpulse(this);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all fluid impulses acting on Body.
     * <br/><br/
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function totalFluidImpulse(body:Body=null){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var arbs=zpp_inner.arbiters;
            for(oarb in arbiters){
                var arb=oarb.zpp_inner;
                if(arb.type!=ZPP_Arbiter.FLUID)continue;
                if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
                {
                    var imp=arb.wrapper().fluidArbiter.totalImpulse(this);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all constraint impulses on this Body.
     *
     * @return The summed effect of constraint impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function constraintsImpulse(){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        {
            var cx_ite=zpp_inner.constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                {
                    var imp=con.outer.bodyImpulse(this);
                    {
                        var t=(1);
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                #end
                            };
                            retx+=imp.x*t;
                            rety+=imp.y*t;
                        };
                        retz+=imp.z*t;
                    };
                    imp.dispose();
                };
                cx_ite=cx_ite.next;
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Evaluate sum effect of all impulses on Body.
     * <br/><br/
     * If the body argument is non-null, then only impulses between 'this' and
     * the given Body will be considered when evaluating interaction impulses.
     * <br/>
     * Constraint impulses are not effected by the body argument.
     *
     * @param body The Body to restrict consideration of impulses with.
     *             (default null)
     * @param freshOnly If true, then only 'new' contact points will be considered
     *             when evaluating contact impulses.
     *             (default false)
     * @return The summed effect of impulses acting on Body.
     */
    #if nape_swc@:keep #end
    public function totalImpulse(body:Body=null,freshOnly:Bool=false){
        var retx:Float=0;
        var rety:Float=0;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((retx!=retx));
            };
            if(!res)throw "assert("+"!assert_isNaN(retx)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rety!=rety));
            };
            if(!res)throw "assert("+"!assert_isNaN(rety)"+") :: "+("vec_new(in n: "+"ret"+",in x: "+"0"+",in y: "+"0"+")");
            #end
        };
        var retz:Float=0;
        var arbs=zpp_inner.arbiters;
        for(oarb in arbiters){
            var arb=oarb.zpp_inner;
            if(arb.type==ZPP_Arbiter.SENSOR)continue;
            if(body!=null&&arb.b2!=body.zpp_inner&&arb.b1!=body.zpp_inner)continue;
            var imp=arb.wrapper().totalImpulse(this,freshOnly);
            {
                var t=(1);
                {
                    var t=(t);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                        #end
                    };
                    retx+=imp.x*t;
                    rety+=imp.y*t;
                };
                retz+=imp.z*t;
            };
            imp.dispose();
        }
        {
            var cx_ite=zpp_inner.constraints.begin();
            while(cx_ite!=null){
                var con=cx_ite.elem();
                {
                    if(con.active){
                        var imp=con.outer.bodyImpulse(this);
                        {
                            var t=(1);
                            {
                                var t=(t);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"ret"+",in b: "+"imp."+",in s: "+"t"+")");
                                    #end
                                };
                                retx+=imp.x*t;
                                rety+=imp.y*t;
                            };
                            retz+=imp.z*t;
                        };
                        imp.dispose();
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        return Vec3.get(retx,rety,retz);
    }
    /**
     * Determine if point is contained in Body.
     *
     * @param point The point to test containment for in world coordinates.
     * @return True if point is contained.
     * @throws # If point is null or has been disposed.
     */
    #if nape_swc@:keep #end
    public function contains(point:Vec2){
        {
            #if(!NAPE_RELEASE_BUILD)
            if(point!=null&&point.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
            #end
        };
        #if(!NAPE_RELEASE_BUILD)
        if(point==null)throw "Error: Cannot check containment of null point";
        #end
        var wasWeak=point.zpp_inner.weak;
        point.zpp_inner.weak=false;
        var retvar;
        {
            retvar=false;
            {
                var cx_ite=zpp_inner.shapes.begin();
                while(cx_ite!=null){
                    var s=cx_ite.elem();
                    {
                        if(s.outer.contains(point)){
                            retvar=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        point.zpp_inner.weak=wasWeak;
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
        return retvar;
    }
    /**
     * @private
     */
    @:keep public override function toString(){
        return(zpp_inner.world?"(space::world":("("+(isDynamic()?"dynamic":isStatic()?"static":"kinematic")))+")#"+id;
    }
}

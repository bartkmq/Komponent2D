package nape.constraint;
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
 * Base type for all Nape joints and constraints
 */
#if nape_swc@:keep #end
class Constraint{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Constraint;
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
     * Set to disable debug drawing/
     * <br/><br/>
     * When true, this Constraint will not be drawn during debug draw operations
     * unless specifically given as argument to Debug draw() method.
     * @default true
     */
    public var debugDraw:Bool=true;
    /**
     * Compound this Constraints belong to.
     * <br/><br/>
     * If this constraint is in a Space or another Compound and you change
     * its compound, then it will be removed from that Space or Compound.
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
                if(this.compound!=null){
                    this.compound.constraints.remove(this);
                }
                if(compound!=null){
                    compound.constraints.add(this);
                }
            }
        }
        return get_compound();
    }
    /**
     * Space this constraint is inside of.
     * <br/><br/>
     * Whether this constraint is directly in a Space, or part of a Compound
     * which is inside of a space, this value will be equal to that Space.
     * <br/><br/>
     * If this constraint is inside of a Compound, then you cannot modify its
     * Space as the constraint belongs to that Compound.
     *
     * @default null
     */
    #if nape_swc@:isVar #end
    public var space(get_space,set_space):Null<Space>;
    inline function get_space():Null<Space>{
        return if(zpp_inner.space==null)null else zpp_inner.space.outer;
    }
    inline function set_space(space:Null<Space>):Null<Space>{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(zpp_inner.compound!=null){
                throw "Error: Cannot set the space of a Constraint belonging to"+" a Compound, only the root Compound space can be set";
            }
            #end
            if(this.space!=space){
                if(zpp_inner.component!=null)zpp_inner.component.woken=false;
                zpp_inner.clearcache();
                if(zpp_inner.space!=null){
                    zpp_inner.space.outer.constraints.remove(this);
                }
                if(space!=null){
                    space.constraints.add(this);
                }
                else{
                    zpp_inner.space=null;
                }
            }
        }
        return get_space();
    }
    /**
     * Whether this constraint is sleeping or not.
     * <br/><br/>
     * This property is only defined if the constraint is inside of a Space
     * and is active, otherwise an error will be thrown should you access this
     * property.
     * <br/><br/>
     * This value is immutable, In Nape you do not ever need to manually
     * wake up a Constraint. It will always be done automatically without error.
     * <br/><br/>
     * To manually put a Constraint to sleep is against the very nature of Nape
     * API and so is excluded from the core of Nape. If you really want to do this
     * you should make use of the nape-hacks module.
     */
    #if nape_swc@:isVar #end
    public var isSleeping(get_isSleeping,never):Bool;
    inline function get_isSleeping():Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.space==null||!zpp_inner.active){
            throw "Error: isSleeping only makes sense if constraint is"+" active and inside a space";
        }
        #end
        return zpp_inner.component.sleeping;
    }
    /**
     * Whether this constraint is active or not.
     * <br/><br/>
     * Setting a constraint to be no longer active is a useful way of
     * temporarigly disabling a constraint without having to remove it
     * from a Space.
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var active(get_active,set_active):Bool;
    inline function get_active():Bool{
        return zpp_inner.active;
    }
    inline function set_active(active:Bool):Bool{
        {
            if(this.active!=active){
                if(zpp_inner.component!=null)zpp_inner.component.woken=false;
                zpp_inner.clearcache();
                if(active){
                    zpp_inner.active=active;
                    zpp_inner.activate();
                    if(zpp_inner.space!=null){
                        if(zpp_inner.component!=null)zpp_inner.component.sleeping=true;
                        zpp_inner.space.wake_constraint(zpp_inner,true);
                    }
                }
                else{
                    if(zpp_inner.space!=null){
                        zpp_inner.wake();
                        zpp_inner.space.live_constraints.remove(zpp_inner);
                    }
                    zpp_inner.active=active;
                    zpp_inner.deactivate();
                }
            }
        }
        return get_active();
    }
    /**
     * Whether interactions between related Bodys will be ignored.
     * <br/><br/>
     * If true, then the Bodys related to this constraint will not
     * be permitted to interact in anyway, including callbacks.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var ignore(get_ignore,set_ignore):Bool;
    inline function get_ignore():Bool{
        return zpp_inner.ignore;
    }
    inline function set_ignore(ignore:Bool):Bool{
        {
            if(this.ignore!=ignore){
                zpp_inner.ignore=ignore;
                zpp_inner.wake();
            }
        }
        return get_ignore();
    }
    /**
     * Whether constraint is stiff, or elastic.
     * <br/><br/>
     * A stiff constraint has its positional error resolved directly
     * as with contact penetrations. This is generally a more stable
     * way of solving positional errors but has a side-effect that for example
     * changing the pivot point on a constraint used for mouse control will not
     * cause the objects to swing as the positional error is solved without
     * effecting the velocity of the object which may not be wanted.
     * <br/><br/>
     * If false, then the positional error of the constraint will be
     * resolved in an elastic way using changes in velocity.
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var stiff(get_stiff,set_stiff):Bool;
    inline function get_stiff():Bool{
        return zpp_inner.stiff;
    }
    inline function set_stiff(stiff:Bool):Bool{
        {
            if(this.stiff!=stiff){
                zpp_inner.stiff=stiff;
                zpp_inner.wake();
            }
        }
        return get_stiff();
    }
    /**
     * Frequency of elastic properties of constraint.
     * <br/><br/>
     * This property only has an effect when constraint is not stiff.
     * <br/><br/>
     * This value corresponds to in an ideal situation, the number of
     * spring like oscillations the constraint will make per second.
     * <br/><br/>
     * This value must be strictly positive (0 not allowed).
     *
     * @default 10
     */
    #if nape_swc@:isVar #end
    public var frequency(get_frequency,set_frequency):Float;
    inline function get_frequency():Float{
        return zpp_inner.frequency;
    }
    inline function set_frequency(frequency:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((frequency!=frequency)){
                throw "Error: Constraint::Frequency cannot be NaN";
            }
            if(frequency<=0){
                throw "Error: Constraint::Frequency must be >0";
            }
            #end
            if(this.frequency!=frequency){
                zpp_inner.frequency=frequency;
                if(!zpp_inner.stiff){
                    zpp_inner.wake();
                }
            }
        }
        return get_frequency();
    }
    /**
     * Damping ratio of elastic properties of constraint.
     * <br/><br/>
     * This property only has an effect when constraint is not stiff.
     * <br/><br/>
     * This value corresponds to in the ideal situation, the damping
     * ratio of the constraints oscillations with 1 corresponding to
     * a total dampening, and values greater than one being over-dampening.
     * <br/><br/>
     * This value must be zero or positive.
     *
     * @default 1
     */
    #if nape_swc@:isVar #end
    public var damping(get_damping,set_damping):Float;
    inline function get_damping():Float{
        return zpp_inner.damping;
    }
    inline function set_damping(damping:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((damping!=damping)){
                throw "Error: Constraint::Damping cannot be Nan";
            }
            if(damping<0){
                throw "Error: Constraint::Damping must be >=0";
            }
            #end
            if(this.damping!=damping){
                zpp_inner.damping=damping;
                if(!zpp_inner.stiff){
                    zpp_inner.wake();
                }
            }
        }
        return get_damping();
    }
    /**
     * The maximum amount of force this constraint is allowed to use.
     * <br/><br/>
     * This value, whilst still used in a stiff constraint will not work
     * as you might hope for; since a stiff constraint resolves positional
     * error without using impulses, the maxForce will not have any effect
     * on how positional errors are resolved.
     * <br/><br/>
     * This value must be zero or positive.
     *
     * @default infinity
     */
    #if nape_swc@:isVar #end
    public var maxForce(get_maxForce,set_maxForce):Float;
    inline function get_maxForce():Float{
        return zpp_inner.maxForce;
    }
    inline function set_maxForce(maxForce:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((maxForce!=maxForce)){
                throw "Error: Constraint::maxForce cannot be NaN";
            }
            if(maxForce<0){
                throw "Error: Constraint::maxForce must be >=0";
            }
            #end
            if(this.maxForce!=maxForce){
                zpp_inner.maxForce=maxForce;
                zpp_inner.wake();
            }
        }
        return get_maxForce();
    }
    /**
     * The maximum amount of error this constraint is allowed to use.
     * <br/><br/>
     * For stiff constraints, this value only serves to work in conjunction
     * with breakUnderError to permit breaking of the constraint.
     * <br/><br/>
     * For non-stiff constraints, this value will also effect how the constraint
     * behaves when breakUnderError is false by restricting the amount of error
     * that will be resolved; this will not work for stiff constraints.
     *
     * @default infinity
     */
    #if nape_swc@:isVar #end
    public var maxError(get_maxError,set_maxError):Float;
    inline function get_maxError():Float{
        return zpp_inner.maxError;
    }
    inline function set_maxError(maxError:Float):Float{
        {
            #if(!NAPE_RELEASE_BUILD)
            if((maxError!=maxError)){
                throw "Error: Constraint::maxError cannot be NaN";
            }
            if(maxError<0){
                throw "Error: Constraint::maxError must be >=0";
            }
            #end
            if(this.maxError!=maxError){
                zpp_inner.maxError=maxError;
                zpp_inner.wake();
            }
        }
        return get_maxError();
    }
    /**
     * Whether constraint will break once maxForce is reached.
     * <br/><br/>
     * This property effects both stiff and non-stiff constraints, though
     * for the same reasons as those of maxForce, does not make much sense
     * to be used in stiff constraints.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var breakUnderForce(get_breakUnderForce,set_breakUnderForce):Bool;
    inline function get_breakUnderForce():Bool{
        return zpp_inner.breakUnderForce;
    }
    inline function set_breakUnderForce(breakUnderForce:Bool):Bool{
        {
            if(this.breakUnderForce!=breakUnderForce){
                zpp_inner.breakUnderForce=breakUnderForce;
                zpp_inner.wake();
            }
        }
        return get_breakUnderForce();
    }
    /**
     * Whether constraint will break once maxError is reached.
     * <br/><br/>
     * This property effects both stiff and non-stiff constraints.
     *
     * @default false
     */
    #if nape_swc@:isVar #end
    public var breakUnderError(get_breakUnderError,set_breakUnderError):Bool;
    inline function get_breakUnderError():Bool{
        return zpp_inner.breakUnderError;
    }
    inline function set_breakUnderError(breakUnderError:Bool):Bool{
        {
            if(this.breakUnderError!=breakUnderError){
                zpp_inner.breakUnderError=breakUnderError;
                zpp_inner.wake();
            }
        }
        return get_breakUnderError();
    }
    /**
     * Whether constraint will be removed when it breaks.
     * <br/><br/>
     * If true, then when constraint is broken it will be removed from
     * the Space. Otherwise it will simple be made inactive.
     *
     * @default true
     */
    #if nape_swc@:isVar #end
    public var removeOnBreak(get_removeOnBreak,set_removeOnBreak):Bool;
    inline function get_removeOnBreak():Bool{
        return zpp_inner.removeOnBreak;
    }
    inline function set_removeOnBreak(removeOnBreak:Bool):Bool{
        {
            zpp_inner.removeOnBreak=removeOnBreak;
        }
        return get_removeOnBreak();
    }
    /**
     * Return the constraint-space impulse applied in previous step.
     *
     * @return A new MatMN representing the constraint space impulse.
     */
    public function impulse():MatMN{
        return null;
    }
    /**
     * Compute impulse that was applied to the given Body.
     * <br/><br/>
     * This impulse is the actual (mass weighted) change in velocity
     * that occured due to this constraint.
     *
     * @param body The Body to compute impulse for.
     * @return The impulse that was applied to the body in the previous step.
     * @throws # If Body is not related to the Constraint.
     */
    public function bodyImpulse(body:Body):Vec3{
        return null;
    }
    /**
     * Apply given function to all Bodys linked to the constraint.
     * <br/><br/>
     * If a body is duplicated in a constraint then it will only
     * be visited once.
     *
     * @param lambda The function to apply to each Body.
     * @throws # If lambda is null.
     */
    public function visitBodies(lambda:Body->Void):Void{}
    /**
     * Set of CbTypes for this constraints for callbacks.
     * <br/><br/>
     * This value cannot at present be set, but can be modified.
     *
     * @default [CbType.ANY_CONSTRAINT]
     */
    #if nape_swc@:isVar #end
    public var cbTypes(get_cbTypes,never):CbTypeList;
    inline function get_cbTypes():CbTypeList{
        if(zpp_inner.wrap_cbTypes==null){
            zpp_inner.setupcbTypes();
        }
        return zpp_inner.wrap_cbTypes;
    }
    /**
     * @private
     */
    public function new(){
        zpp_inner.insert_cbtype(CbType.ANY_CONSTRAINT.zpp_inner);
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: Constraint cannot be instantiated derp!";
        #end
    }
    /**
     * @private
     */
    @:keep public function toString():String{
        return "{Constraint}";
    }
    /**
     * Produce copy of constraint.
     * <br/><br/>
     * All constraint properties except for internal impulse cache
     * and userData field will be copied.
     *
     * @return The copied Constraint.
     */
    public function copy():Constraint{
        return zpp_inner.copy();
    }
}

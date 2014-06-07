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
 * UserConstraint providing a low-level API for user-defined Constraints.
 * <br/><br/>
 * This API is intended to be powerful enough to model any constraint that
 * Nape can handle, but not so low level as to be completely prohibitive.
 * <br/>
 * For instance, things like soft-constraints are automatically provided
 * by this API.
 * <br/><br/>
 * Working with this API will require mathematical skills. A full manual
 * for this API is provided at: http://napephys.com/help/Constraints.pdf
 * <br/><br/>
 * You may also be interested in the nape-symbolic module that is available
 * on github/haxelib/nape downloads. Which provides a run-time compiled DSL
 * using this API to make prototyping (or creating non-performance critical)
 * user-defined constraints simple without the need for great mathematical
 * skills as well as being much quicker to work with.
 */
#if nape_swc@:keep #end
class UserConstraint extends Constraint{
    /**
     * @private
     */
    public var zpp_inner_zn:ZPP_UserConstraint=null;
    /**
     * Create a Vec2 property for user-constraint.
     * <br/><br/>
     * This method is used in creating a custom constraint, to create a
     * Vec2 property which will be tied to this constraint so that modifications
     * to the Vec2 will have the appropriate side effects on constraint.
     * <pre>
     * //Haxe (Cleanest way without using macros, clearly you must ensure this
     * //      is set at least once, likely in constructor)
     * public var property(default, set_property):Vec2;
     * inline function set_property(property:Vec2) {
     *     if (this.property == null) this.property = __bindVec2();
     *     return this.property.set(property);
     * }
     *
     * //AS3
     * private var _property:Vec2 = __bindVec2();
     * public function get property():Vec2 { return _property; }
     * public function set property(property:Vec2):void {
     *     _property.set(property);
     * }
     * </pre>
     * This bound Vec2 will behave like standard Nape anchor/direction Vec2's.
     * <br/><br/>
     * You should hide this method in your sub-type.
     */
    public function __bindVec2():Vec2{
        var ret=new Vec2();
        ret.zpp_inner._inuse=true;
        ret.zpp_inner._invalidate=zpp_inner_zn.bindVec2_invalidate;
        return ret;
    }
    /**
     * Internal copying of user defined constraint.
     * <br/><br/>
     * This method must be overriden, and defines how your customised constraint
     * is to be copied. Likely by simply calling your constructor with constraint
     * properties as argument.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @return A copy of your constraint.
     * @throws # If not overriden by sub-type.
     */
    public function __copy():UserConstraint{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: UserConstraint::__copy must be overriden";
        #end
        return null;
    }
    /**
     * Internal extra steps in breaking constraint
     * <br/><br/>
     * This method may be optionally overriden, and defines extra steps to
     * be taken when your constraint is broken. This will be called before
     * the constraint is removed or made inactive.
     * <br/><br/>
     * You should hide this method in your sub-type.
     */
    public function __broken():Void{}
    /**
     * Internal validation of constraint.
     * <br/><br/>
     * This method may be optionally overriden, and defines extra validation
     * steps to be made in validating your constraint integrity in terms of
     * property values. (Think things like standard nape errors if a body
     * is not in the same space as constraint).
     * <br/><br/>
     * This method will be called in all build types, not just debug and can also
     * be used to pre-compute values that will remain constant over an entire
     * time step and do not depend on the state of the Body's.
     * <br/><br/>
     * You should hide this method in your sub-type.
     */
    public function __validate():Void{}
    /**
     * Internal debug drawing of constraint.
     * <br/><br/>
     * This method will be called by Nape debug draws when enabled to
     * draw your constraint. You do not need to override this method.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param debug Nape Debug draw to draw constraint to.
     */
    public function __draw(debug:Debug):Void{}
    /**
     * Internal position dependant calculations for constraint.
     * <br/><br/>
     * This method may be overriden to define extra computations that will
     * remain constant as long as a Body's position/rotation is not changed.
     * <br/><br/>
     * You should hide this method in your sub-type.
     */
    public function __prepare():Void{}
    /**
     * Internal positional error function for constraint.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param err The output array to store constraint space positional error.
     * @throws # If this is not a velocity-only constraint, and is not overriden.
     */
    public function __position(err:TArray<Float>):Void{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: UserConstraint::__position must be overriden";
        #end
    }
    /**
     * Internal velocity error function for constraint.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param err The output array to store constraint space velocity error.
     * @throws # If not overriden by sub-type.
     */
    public function __velocity(err:TArray<Float>):Void{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: Userconstraint::__velocity must be overriden";
        #end
    }
    /**
     * Internal effective mass matrix function for constraint.
     * <br/><br/>
     * This array will be of size <code>dimension * (dimension - 1)</code> as a
     * compressed, symmetric matrix:
     * <pre>
     * // Assuming dimension of 3 for constraint:
     * [ eff[0] eff[1] eff[2] ]
     * [ eff[1] eff[3] eff[4] ]
     * [ eff[2] eff[4] eff[5] ]
     * </pre>
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param eff The output array to store constraint space effective mass matrix.
     * @throws # If this method is not overriden.
     */
    public function __eff_mass(eff:TArray<Float>):Void{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: UserConstraint::__eff_mass must be overriden";
        #end
    }
    /**
     * Internal, optional clamping of impulse for constraint.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param jAcc The constraint space impulse to be clamped.
     */
    public function __clamp(jAcc:TArray<Float>):Void{}
    /**
     * Internal application of impulse to body.
     * <br/><br/>
     * You should hide this method in your sub-type.
     *
     * @param imp The constraint space impulse to be applied to bodies.
     * @param body The body to apply impulse to.
     * @param out The Vec3 to store impulse on body to be applied. This
     *            should be in world space.
     */
    public function __impulse(imp:TArray<Float>,body:Body,out:Vec3):Void{
        #if(!NAPE_RELEASE_BUILD)
        throw "Error: UserConstraint::__impulse must be overriden";
        #end
    }
    /**
     * Base constructor for user constraints.
     * <br/><br/>
     * You should never call this function directly, only though use of
     * super(..) in sub-typed UserConstraint.
     *
     * @param dimensions The number of constraint space dimensions.
     * @param velocityOnly If true, then this constraint will be implemented
     *                     as a velocity-only constraint like the MotorJoint.
     * @throws # If dimensions < 1
     */
    #if flib@:keep function flibopts_1(){}
    #end
    public function new(dimensions:Int,velocityOnly:Bool=false){
        #if(!NAPE_RELEASE_BUILD)
        if(dimensions<1){
            throw "Error: Constraint dimension must be at least 1";
        }
        #end
        zpp_inner_zn=new ZPP_UserConstraint(dimensions,velocityOnly);
        zpp_inner=zpp_inner_zn;
        zpp_inner.outer=this;
        zpp_inner_zn.outer_zn=this;
        #if(!NAPE_RELEASE_BUILD)
        try{
            super();
        }
        catch(e:Dynamic){}
        #end
        #if NAPE_RELEASE_BUILD 
        super();
        #end
    }
    /**
     @inheritDoc
     * <br/><br/>
     * For user-defined constraints, this will be a dimensions * 1 MatMN.
     */
    public override function impulse():MatMN{
        var ret=new MatMN(zpp_inner_zn.dim,1);
        for(i in 0...zpp_inner_zn.dim){
            ret.setx(i,0,zpp_inner_zn.jAcc[i]);
        }
        return ret;
    }
    /**
     @inheritDoc
     */
    public override function bodyImpulse(body:Body):Vec3{
        #if(!NAPE_RELEASE_BUILD)
        if(body==null){
            throw "Error: Cannot evaluate impulse on null body";
        }
        var found=false;
        for(b in zpp_inner_zn.bodies){
            if(b.body==body.zpp_inner){
                found=true;
                break;
            }
        }
        if(!found){
            throw "Error: Body is not linked to this constraint";
        }
        #end
        if(!active){
            return Vec3.get();
        }
        else{
            return zpp_inner_zn.bodyImpulse(body.zpp_inner);
        }
    }
    /**
     @inheritDoc
     */
    public override function visitBodies(lambda:Body->Void):Void{
        var i=0;
        var nbodies:Int=cast zpp_inner_zn.bodies.length;
        while(i<nbodies){
            var b=zpp_inner_zn.bodies[i];
            if(b.body!=null){
                var found=false;
                for(j in(i+1)...nbodies){
                    var c=zpp_inner_zn.bodies[j];
                    if(c.body==b.body){
                        found=true;
                        break;
                    }
                }
                if(!found){
                    lambda(b.body.outer);
                }
            }
            i++;
        }
    }
    /**
     * Internal method to invalidate constraint on property changes
     * <br/><br/>
     * This method should be hidden in your sub-type, and called by your
     * constraint's API when a property of the constraint has been changed.
     * <br/>
     * This does not need to be called for Vec2 properties created via
     * the bindVec2 method.
     *
     * @throws # If you call this method in the middle of a space step.
     */
    public function __invalidate():Void{
        zpp_inner.immutable_midstep("UserConstraint::invalidate()");
        if(active&&space!=null){
            zpp_inner.wake();
        }
    }
    /**
     * Internal method to register Body's with constraint.
     * <br/><br/>
     * This method should be hidden in your sub-type, and used to deal with
     * adding/removing objects from the constraint so that a functionally equivalent
     * constraint can be created (Dealing with all necessary side-effects etc).
     * <pre>
     * //Haxe
     * public var body1(default, set_body1):Null&lt;Body&gt;;
     * inline function set_body1(body1:Null&lt;Body&gt;) {
     *     return this.body1 = __registerBody(this.body1, body1);
     * }
     *
     * //AS3
     * private var _body1:Body;
     * public function get body1():Body {
     *     return _body1;
     * }
     * public function set body1(body1:Body):void {
     *     _body1 = __registerBody(_body1, body1);
     * }
     * </pre>
     *
     * @param oldBody The present value of body parameter.
     * @param newBody The new value for body parameter.
     * @return Returns newBody parameter.
     * @throws # If oldBody is not registered with constraint.
     * @throws # If you call this method in the middle of a space step.
     */
    public function __registerBody(oldBody:Null<Body>,newBody:Null<Body>):Null<Body>{
        zpp_inner.immutable_midstep("UserConstraint::registerBody(..)");
        if(oldBody!=newBody){
            if(oldBody!=null){
                if(!zpp_inner_zn.remBody(oldBody.zpp_inner)){
                    #if(!NAPE_RELEASE_BUILD)
                    throw "Error: oldBody is not registered to the cosntraint";
                    #end
                }
                if(active&&space!=null){
                    oldBody.zpp_inner.wake();
                }
            }
            if(newBody!=null){
                zpp_inner_zn.addBody(newBody.zpp_inner);
            }
            zpp_inner.wake();
            if(newBody!=null){
                newBody.zpp_inner.wake();
            }
        }
        return newBody;
    }
}

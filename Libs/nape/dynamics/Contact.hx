package nape.dynamics;
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
 * Contact point for collision interactions
 * <br/><br/>
 * These objects are automatically reused and you should not keep references to them.
 */
@:final#if nape_swc@:keep #end
class Contact{
    /**
     * @private
     */
    public var zpp_inner:ZPP_Contact=null;
    /**
     * Reference to the CollisionArbiter this contact belongs to
     */
    #if nape_swc@:isVar #end
    public var arbiter(get_arbiter,never):CollisionArbiter;
    inline function get_arbiter():CollisionArbiter{
        return if(zpp_inner.arbiter==null)null else zpp_inner.arbiter.outer.collisionArbiter;
    }
    /**
     * Penetration of bodies along normal for this contact.
     * <br/><br/>
     * This value may be negative and corresponds to the penetration (if at all)
     * of the contact point before positional integration and error resolvement
     * took place (correct at time of pre-listeners).
     */
    #if nape_swc@:isVar #end
    public var penetration(get_penetration,never):Float;
    inline function get_penetration():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        return(-zpp_inner.dist);
    }
    /**
     * The world-space position of contact.
     * <br/><br/>
     * This value corresponds to the position
     * of the contact point before positional integration and error resolvement
     * took place (correct at time of pre-listeners).
     */
    #if nape_swc@:isVar #end
    public var position(get_position,never):Vec2;
    inline function get_position():Vec2{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        if(zpp_inner.wrap_position==null)zpp_inner.getposition();
        return zpp_inner.wrap_position;
    }
    /**
     * Whether this contact is newly generated, or persistant from previous step.
     */
    #if nape_swc@:isVar #end
    public var fresh(get_fresh,never):Bool;
    inline function get_fresh():Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        return zpp_inner.fresh;
    }
    /**
     * Evaluate normal reactive impulses for this contact for a given body.
     * <br/><br/>
     * If body argument is null, then the contact normal impulses will be returned instead
     * with no angular impulse derivable, the direction of this impulse will be the direction of the normal.
     * <br/>
     * If body argument is not null, then this will return the actual impulse applied to that specific body
     * for this contact this will include angular impulses due to position of contact point and normal.
     *
     * @param body The Body to query normal impulse for. (default null)
     * @return The impulse applied to the given body, considering normal reactive forces.
     * @throws # If body is non-null, and unrelated to this Contact.
     */
    #if nape_swc@:keep #end
    public function normalImpulse(body:Body=null){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        var colarb=zpp_inner.arbiter.colarb;
        var cin=zpp_inner.inner;
        var jnAcc=cin.jnAcc;
        if(body==null)return Vec3.get(colarb.nx*jnAcc,colarb.ny*jnAcc);
        else{
            #if(!NAPE_RELEASE_BUILD)
            if(body!=colarb.b1.outer&&body!=colarb.b2.outer)throw "Error: Contact does not relate to the given body";
            #end
            if(body==colarb.b1.outer)return Vec3.get(colarb.nx*-jnAcc,colarb.ny*-jnAcc,-(colarb.ny*cin.r1x-colarb.nx*cin.r1y)*jnAcc);
            else return Vec3.get(colarb.nx*jnAcc,colarb.ny*jnAcc,(colarb.ny*cin.r2x-colarb.nx*cin.r2y)*jnAcc);
        }
    }
    /**
     * Evaluate tangent impulses for this contact for a given body.
     * <br/><br/>
     * If body argument is null, then the contact friction impulses is returned with
     * no angular impulse derivable, the direction of this impulse will be against the relative
     * velocity of the first body against the second.
     * <br/>
     * If the body argument is non-null, then the actual impulse applied to that body due to tangent
     * frictino impulses will be returned, including angular effects due to contact position and normal.
     * <br/><br/>
     * These tangent impulses correspond to the forces of static and dynamic friction for this contact.
     *
     * @param body The Body to query tangent impulse for. (default null)
     * @return The impulse applied to the given body, considering standard frictional forces.
     * @throws # If body is non-null, and unrelated to this Contact.
     */
    #if nape_swc@:keep #end
    public function tangentImpulse(body:Body=null){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        var colarb=zpp_inner.arbiter.colarb;
        var cin=zpp_inner.inner;
        var jtAcc=cin.jtAcc;
        if(body==null)return Vec3.get(-colarb.ny*jtAcc,colarb.nx*jtAcc);
        else{
            #if(!NAPE_RELEASE_BUILD)
            if(body!=colarb.b1.outer&&body!=colarb.b2.outer)throw "Error: Contact does not relate to the given body";
            #end
            if(body==colarb.b1.outer)return Vec3.get(colarb.ny*jtAcc,-colarb.nx*jtAcc,-(cin.r1x*colarb.nx+cin.r1y*colarb.ny)*jtAcc);
            else return Vec3.get(-colarb.ny*jtAcc,colarb.nx*jtAcc,(cin.r2x*colarb.nx+cin.r2y*colarb.ny)*jtAcc);
        }
    }
    /**
     * Evaluate rolling friction impulses for this contact for a given body.
     * <br/><br/>
     * If body argument is null, then the rolling impulse of this contact will be returned
     * instead of the angular impulse applied to the specific body as a result of the rolling impulse.
     *
     * @param body The Body to query rolling impulse for. (default null)
     * @return The angular impulse applied to the given body.
     * @throws # If body is non-null, and unrelated to this Contact.
     */
    #if nape_swc@:keep #end
    public function rollingImpulse(body:Body=null){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        var colarb=zpp_inner.arbiter.colarb;
        var jrAcc=zpp_inner.arbiter.colarb.jrAcc;
        if(body==null)return jrAcc;
        else{
            #if(!NAPE_RELEASE_BUILD)
            if(body!=colarb.b1.outer&&body!=colarb.b2.outer)throw "Error: Contact does not relate to the given body";
            #end
            if(body==colarb.b1.outer)return-jrAcc;
            else return jrAcc;
        }
    }
    /**
     * Evaluate total contact impulse for a given body.
     * <br/><br/>
     * If body argument is null, then this will return the sum of normal and tangent contact impulse, and the contact
     * rolling impulse.
     * <br/>
     * When body argument is non-null, this impulse will be the actual change in (mass weighted)
     * velocity that this contact caused to the Body in the previous time step.
     *
     * @param body The Body to query total impulse for. (default null)
     * @return The impulse applied to the given body
     * @throws # If body is non-null, and unrelated to this Contact.
     */
    #if nape_swc@:keep #end
    public function totalImpulse(body:Body=null){
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        var colarb=zpp_inner.arbiter.colarb;
        var cin=zpp_inner.inner;
        var jnAcc=cin.jnAcc;
        var jtAcc=cin.jtAcc;
        var jrAcc=colarb.jrAcc;
        if(body==null){
            return Vec3.get(colarb.nx*jnAcc-colarb.ny*jtAcc,colarb.ny*jnAcc+colarb.nx*jtAcc,jrAcc);
        }
        else{
            #if(!NAPE_RELEASE_BUILD)
            if(body!=colarb.b1.outer&&body!=colarb.b2.outer)throw "Error: Contact does not relate to the given body";
            #end
            var jx:Float=colarb.nx*jnAcc-colarb.ny*jtAcc;
            var jy:Float=colarb.ny*jnAcc+colarb.nx*jtAcc;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jx!=jx));
                };
                if(!res)throw "assert("+"!assert_isNaN(jx)"+") :: "+("vec_new(in n: "+"j"+",in x: "+"colarb.nx*jnAcc-colarb.ny*jtAcc"+",in y: "+"colarb.ny*jnAcc+colarb.nx*jtAcc"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((jy!=jy));
                };
                if(!res)throw "assert("+"!assert_isNaN(jy)"+") :: "+("vec_new(in n: "+"j"+",in x: "+"colarb.nx*jnAcc-colarb.ny*jtAcc"+",in y: "+"colarb.ny*jnAcc+colarb.nx*jtAcc"+")");
                #end
            };
            if(body==colarb.b1.outer)return Vec3.get(-jx,-jy,-(jy*cin.r1x-jx*cin.r1y)-jrAcc);
            else return Vec3.get(jx,jy,(jy*cin.r2x-jx*cin.r2y)+jrAcc);
        }
    }
    /**
     * The specific coeffecient of friction for this contact.
     * <br/><br/>
     * This value is equal either to the static or dynamic friction coeffecient of the arbiter
     * based on the relative velocity at contact point.
     * <br/><br/>
     * This value cannot be set, though you may implicitly set it exactly by modifying
     * the arbiter to have the same static and dynamic friction in the PreListener.
     */
    #if nape_swc@:isVar #end
    public var friction(get_friction,never):Float;
    inline function get_friction():Float{
        #if(!NAPE_RELEASE_BUILD)
        if(zpp_inner.inactiveme())throw "Error: Contact not currently in use";
        #end
        return zpp_inner.inner.friction;
    }
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Contact.internal)throw "Error: Cannot instantiate Contact derp!";
        #end
    }
    /**
     * @private
     */
    @:keep public function toString(){
        if(zpp_inner.arbiter==null||zpp_inner.arbiter.cleared)return "{object-pooled}";
        else return "{Contact}";
    }
}

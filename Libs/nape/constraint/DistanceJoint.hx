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
import nape.constraint.Constraint;
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
 * DistanceJoint limiting the distance between two local anchor points of Bodies.
 * <br/><br/>
 * The equation for this constraint could be written like:
 * <pre>
 * jointMin <= distance(body2.localPointToWorld(anchor2), body1.localPointToWorld(anchor1)) <= jointMax
 * </pre>
 * This joint is not designed to work when <code> jointMin = jointMax = 0 </code> and constraint is
 * stiff. In this instance you should use a PivotJoint instead.
 */
@:final#if nape_swc@:keep #end
class DistanceJoint extends Constraint{
    /**
     * @private
     */
    public var zpp_inner_zn:ZPP_DistanceJoint=null;
    /**
     * First Body in constraint.
     * <br/><br/>
     * This value may be null, but trying to simulate the constraint whilst
     * this body is null will result in an error.
     */
    #if nape_swc@:isVar #end
    public var body1(get_body1,set_body1):Null<Body>;
    inline function get_body1():Null<Body>{
        return if(zpp_inner_zn.b1==null)null else zpp_inner_zn.b1.outer;
    }
    inline function set_body1(body1:Null<Body>):Null<Body>{
        {
            zpp_inner.immutable_midstep("Constraint::"+"body1");
            var inbody1=if(body1==null)null else body1.zpp_inner;
            if(inbody1!=zpp_inner_zn.b1){
                if(zpp_inner_zn.b1!=null){
                    if(space!=null&&zpp_inner_zn.b2!=zpp_inner_zn.b1){
                        {
                            if(zpp_inner_zn.b1!=null)zpp_inner_zn.b1.constraints.remove(this.zpp_inner);
                        };
                    }
                    if(active&&space!=null)zpp_inner_zn.b1.wake();
                }
                zpp_inner_zn.b1=inbody1;
                if(space!=null&&inbody1!=null&&zpp_inner_zn.b2!=inbody1){
                    {
                        if(inbody1!=null)inbody1.constraints.add(this.zpp_inner);
                    };
                }
                if(active&&space!=null){
                    zpp_inner.wake();
                    if(inbody1!=null)inbody1.wake();
                }
            }
        }
        return get_body1();
    }
    /**
     * Second Body in constraint.
     * <br/><br/>
     * This value may be null, but trying to simulate the constraint whilst
     * this body is null will result in an error.
     */
    #if nape_swc@:isVar #end
    public var body2(get_body2,set_body2):Null<Body>;
    inline function get_body2():Null<Body>{
        return if(zpp_inner_zn.b2==null)null else zpp_inner_zn.b2.outer;
    }
    inline function set_body2(body2:Null<Body>):Null<Body>{
        {
            zpp_inner.immutable_midstep("Constraint::"+"body2");
            var inbody2=if(body2==null)null else body2.zpp_inner;
            if(inbody2!=zpp_inner_zn.b2){
                if(zpp_inner_zn.b2!=null){
                    if(space!=null&&zpp_inner_zn.b1!=zpp_inner_zn.b2){
                        {
                            if(zpp_inner_zn.b2!=null)zpp_inner_zn.b2.constraints.remove(this.zpp_inner);
                        };
                    }
                    if(active&&space!=null)zpp_inner_zn.b2.wake();
                }
                zpp_inner_zn.b2=inbody2;
                if(space!=null&&inbody2!=null&&zpp_inner_zn.b1!=inbody2){
                    {
                        if(inbody2!=null)inbody2.constraints.add(this.zpp_inner);
                    };
                }
                if(active&&space!=null){
                    zpp_inner.wake();
                    if(inbody2!=null)inbody2.wake();
                }
            }
        }
        return get_body2();
    }
    /**
     * Anchor point on first Body.
     * <br/><br/>
     * This anchor point is defined in the local coordinate system of body1.
     */
    #if nape_swc@:isVar #end
    public var anchor1(get_anchor1,set_anchor1):Vec2;
    inline function get_anchor1():Vec2{
        if(zpp_inner_zn.wrap_a1==null)zpp_inner_zn.setup_a1();
        return zpp_inner_zn.wrap_a1;
    }
    inline function set_anchor1(anchor1:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(anchor1!=null&&anchor1.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(anchor1==null)throw "Error: Constraint::"+"anchor1"+" cannot be null";
            #end
            this.anchor1.set(anchor1);
        }
        return get_anchor1();
    }
    /**
     * Anchor point on second Body.
     * <br/><br/>
     * This anchor point is defined in the local coordinate system of body2.
     */
    #if nape_swc@:isVar #end
    public var anchor2(get_anchor2,set_anchor2):Vec2;
    inline function get_anchor2():Vec2{
        if(zpp_inner_zn.wrap_a2==null)zpp_inner_zn.setup_a2();
        return zpp_inner_zn.wrap_a2;
    }
    inline function set_anchor2(anchor2:Vec2):Vec2{
        {
            {
                #if(!NAPE_RELEASE_BUILD)
                if(anchor2!=null&&anchor2.zpp_disp)throw "Error: "+"Vec2"+" has been disposed and cannot be used!";
                #end
            };
            #if(!NAPE_RELEASE_BUILD)
            if(anchor2==null)throw "Error: Constraint::"+"anchor2"+" cannot be null";
            #end
            this.anchor2.set(anchor2);
        }
        return get_anchor2();
    }
    /**
     * Lower bound for constraint.
     * <br/><br/>
     * This value must be less than or equal to jointMax, and greater equal to 0.
     */
    #if nape_swc@:isVar #end
    public var jointMin(get_jointMin,set_jointMin):Float;
    inline function get_jointMin():Float{
        return zpp_inner_zn.jointMin;
    }
    inline function set_jointMin(jointMin:Float):Float{
        {
            zpp_inner.immutable_midstep("DistanceJoint::jointMin");
            #if(!NAPE_RELEASE_BUILD)
            if((jointMin!=jointMin)){
                throw "Error: DistanceJoint::jointMin cannot be NaN";
            }
            if(jointMin<0){
                throw "Error: DistanceJoint::jointMin must be >= 0";
            }
            #end
            if(this.jointMin!=jointMin){
                zpp_inner_zn.jointMin=jointMin;
                zpp_inner.wake();
            }
        }
        return get_jointMin();
    }
    /**
     * Upper bound for constraint.
     * <br/><br/>
     * This value must be greater than or equal to jointMin.
     */
    #if nape_swc@:isVar #end
    public var jointMax(get_jointMax,set_jointMax):Float;
    inline function get_jointMax():Float{
        return zpp_inner_zn.jointMax;
    }
    inline function set_jointMax(jointMax:Float):Float{
        {
            zpp_inner.immutable_midstep("DistanceJoint::jointMax");
            #if(!NAPE_RELEASE_BUILD)
            if((jointMax!=jointMax)){
                throw "Error: DistanceJoint::jointMax cannot be NaN";
            }
            if(jointMax<0){
                throw "Error: DistanceJoint::jointMax must be >= 0";
            }
            #end
            if(this.jointMax!=jointMax){
                zpp_inner_zn.jointMax=jointMax;
                zpp_inner.wake();
            }
        }
        return get_jointMax();
    }
    /**
     * Determine if constraint is slack.
     * <br/><br/>
     * This constraint is slack if the positional error is within
     * the bounds of (jointMin, jointMax).
     *
     * @return True if positional error of constraint is between the limits
     *              indicating that the constraint is not doing any work.
     * @throws # If either of the bodies is null.
     */
    public#if NAPE_NO_INLINE#else inline #end
    function isSlack():Bool{
        #if(!NAPE_RELEASE_BUILD)
        if(body1==null||body2==null){
            throw "Error: Cannot compute slack for DistanceJoint if either body is null.";
        }
        #end
        return zpp_inner_zn.slack;
    }
    /**
     * Construct a new DistanceJoint.
     *
     * @param body1 The first body in DistanceJoint.
     * @param body2 The second body in DistanceJoint.
     * @param anchor1 The first local anchor for joint.
     * @param anchor2 The second local anchor for joint.
     * @param jointMin The lower bound for constraint.
     * @param jointMax The upper bound for constraint.
     * @return The constructed DistanceJoint.
     */
    #if flib@:keep function flibopts_0(){}
    #end
    public function new(body1:Null<Body>,body2:Null<Body>,anchor1:Vec2,anchor2:Vec2,jointMin:Float,jointMax:Float){
        zpp_inner_zn=new ZPP_DistanceJoint();
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
        this.body1=body1;
        this.body2=body2;
        this.anchor1=anchor1;
        this.anchor2=anchor2;
        this.jointMin=jointMin;
        this.jointMax=jointMax;
    }
    /**
     * @inheritDoc
     * <br/><br/>
     * For this constraint, the MatMN will be 1x1.
     */
    public override function impulse():MatMN{
        var ret=new MatMN(1,1);
        ret.setx(0,0,zpp_inner_zn.jAcc);
        return ret;
    }
    /**
     * @inheritDoc
     */
    public override function bodyImpulse(body:Body):Vec3{
        #if(!NAPE_RELEASE_BUILD)
        if(body==null){
            throw "Error: Cannot evaluate impulse on null body";
        }
        if(body!=body1&&body!=body2){
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
     * @inheritDoc
     */
    public override function visitBodies(lambda:Body->Void){
        if(body1!=null){
            lambda(body1);
        }
        if(body2!=null&&body2!=body1){
            lambda(body2);
        }
    }
}

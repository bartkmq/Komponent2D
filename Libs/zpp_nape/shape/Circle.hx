package zpp_nape.shape;
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
#if nape_swc@:keep #end
class ZPP_Circle extends ZPP_Shape{
    public var outer_zn:Circle=null;
    public var radius:Float=0.0;
    public function new(){
        super(ZPP_Flags.id_ShapeType_CIRCLE);
        circle=this;
        zip_localCOM=false;
    }
    public function __clear(){}
    public function invalidate_radius(){
        invalidate_area_inertia();
        invalidate_angDrag();
        invalidate_aabb();
        if(body!=null)body.wake();
    }
    private function localCOM_validate(){
        wrap_localCOM.zpp_inner.x=localCOMx;
        wrap_localCOM.zpp_inner.y=localCOMy;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((wrap_localCOM.zpp_inner.x!=wrap_localCOM.zpp_inner.x));
            };
            if(!res)throw "assert("+"!assert_isNaN(wrap_localCOM.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_localCOM.zpp_inner."+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((wrap_localCOM.zpp_inner.y!=wrap_localCOM.zpp_inner.y));
            };
            if(!res)throw "assert("+"!assert_isNaN(wrap_localCOM.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_localCOM.zpp_inner."+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
            #end
        };
    }
    private function localCOM_invalidate(x:ZPP_Vec2){
        {
            localCOMx=x.x;
            localCOMy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((localCOMx!=localCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(localCOMx)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((localCOMy!=localCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(localCOMy)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
        invalidate_localCOM();
        if(body!=null)body.wake();
    }
    #if(!NAPE_RELEASE_BUILD)
    private function localCOM_immutable(){
        if(body!=null&&body.isStatic()&&body.space!=null)throw "Error: Cannot modify localCOM of Circle added to a static Body whilst within a Space";
    }
    #end
    public function setupLocalCOM(){
        var me=this;
        wrap_localCOM=Vec2.get(localCOMx,localCOMy);
        wrap_localCOM.zpp_inner._inuse=true;
        wrap_localCOM.zpp_inner._validate=localCOM_validate;
        wrap_localCOM.zpp_inner._invalidate=localCOM_invalidate;
        #if(!NAPE_RELEASE_BUILD)
        wrap_localCOM.zpp_inner._isimmutable=localCOM_immutable;
        #end
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function __validate_aabb(){
        validate_worldCOM();
        var rx:Float=radius;
        var ry:Float=radius;
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((rx!=rx));
            };
            if(!res)throw "assert("+"!assert_isNaN(rx)"+") :: "+("vec_new(in n: "+"r"+",in x: "+"radius"+",in y: "+"radius"+")");
            #end
        };
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                !((ry!=ry));
            };
            if(!res)throw "assert("+"!assert_isNaN(ry)"+") :: "+("vec_new(in n: "+"r"+",in x: "+"radius"+",in y: "+"radius"+")");
            #end
        };
        {
            aabb.minx=worldCOMx-rx;
            aabb.miny=worldCOMy-ry;
        };
        {
            aabb.maxx=worldCOMx+rx;
            aabb.maxy=worldCOMy+ry;
        };
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function _force_validate_aabb(){
        {
            worldCOMx=body.posx+(body.axisy*localCOMx-body.axisx*localCOMy);
            worldCOMy=body.posy+(localCOMx*body.axisx+localCOMy*body.axisy);
        };
        aabb.minx=worldCOMx-radius;
        aabb.miny=worldCOMy-radius;
        aabb.maxx=worldCOMx+radius;
        aabb.maxy=worldCOMy+radius;
    }
    public function __validate_sweepRadius(){
        sweepCoef=Math.sqrt((localCOMx*localCOMx+localCOMy*localCOMy));
        sweepRadius=sweepCoef+radius;
    }
    public function __validate_area_inertia(){
        var r2=radius*radius;
        area=r2*Math.PI;
        inertia=r2*0.5+(localCOMx*localCOMx+localCOMy*localCOMy);
    }
    public function __validate_angDrag(){
        var lc=(localCOMx*localCOMx+localCOMy*localCOMy);
        var r2=radius*radius;
        var skin=material.dynamicFriction*Config.fluidAngularDragFriction;
        angDrag=(lc+2*r2)*skin+0.5*Config.fluidAngularDrag*(1+Config.fluidVacuumDrag)*lc;
        angDrag/=(2*(lc+0.5*r2));
    }
    public function __scale(sx:Float,sy:Float){
        var factor=((sx<0?-sx:sx)+(sy<0?-sy:sy))/2;
        radius*=factor<0?-factor:factor;
        invalidate_radius();
        if((localCOMx*localCOMx+localCOMy*localCOMy)>0){
            localCOMx*=sx;
            localCOMy*=sy;
            invalidate_localCOM();
        }
    }
    public function __translate(x:Float,y:Float){
        {
            var t=(1.0);
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((t!=t));
                };
                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"localCOM"+",in b: "+""+",in s: "+"1.0"+")");
                #end
            };
            localCOMx+=x*t;
            localCOMy+=y*t;
        };
        invalidate_localCOM();
    }
    public function __rotate(x:Float,y:Float){
        if((localCOMx*localCOMx+localCOMy*localCOMy)>0){
            var tx:Float=0.0;
            var ty:Float=0.0;
            {
                tx=(y*localCOMx-x*localCOMy);
                ty=(localCOMx*x+localCOMy*y);
            };
            {
                localCOMx=tx;
                localCOMy=ty;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMx!=localCOMx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMx)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"tx"+",in y: "+"ty"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((localCOMy!=localCOMy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(localCOMy)"+") :: "+("vec_set(in n: "+"localCOM"+",in x: "+"tx"+",in y: "+"ty"+")");
                    #end
                };
            };
            invalidate_localCOM();
        }
    }
    public function __transform(m:Mat23){
        var det=(m.a*m.d-m.b*m.c);
        if(det<0)det=-det;
        radius*=Math.sqrt(det);
        {
            var t=m.a*localCOMx+m.b*localCOMy+m.tx;
            localCOMy=m.c*localCOMx+m.d*localCOMy+m.ty;
            localCOMx=t;
        };
        invalidate_radius();
        invalidate_localCOM();
    }
    public function __copy(){
        var ret=new Circle(radius).zpp_inner_zn;
        {
            ret.localCOMx=localCOMx;
            ret.localCOMy=localCOMy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.localCOMx!=ret.localCOMx));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.localCOMx)"+") :: "+("vec_set(in n: "+"ret.localCOM"+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((ret.localCOMy!=ret.localCOMy));
                };
                if(!res)throw "assert("+"!assert_isNaN(ret.localCOMy)"+") :: "+("vec_set(in n: "+"ret.localCOM"+",in x: "+"localCOMx"+",in y: "+"localCOMy"+")");
                #end
            };
        };
        ret.zip_localCOM=false;
        return ret;
    }
}

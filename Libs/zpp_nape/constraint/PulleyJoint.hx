package zpp_nape.constraint;
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
class ZPP_PulleyJoint extends ZPP_Constraint{
    public var outer_zn:PulleyJoint=null;
    public var ratio:Float=1.0;
    public var jointMin:Float=0.0;
    public var jointMax:Float=0.0;
    public var slack:Bool=false;
    public var equal:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function is_slack(){
        var slack;
        {
            a1relx=(b1.axisy*a1localx-b1.axisx*a1localy);
            a1rely=(a1localx*b1.axisx+a1localy*b1.axisy);
        };
        {
            a2relx=(b2.axisy*a2localx-b2.axisx*a2localy);
            a2rely=(a2localx*b2.axisx+a2localy*b2.axisy);
        };
        {
            a3relx=(b3.axisy*a3localx-b3.axisx*a3localy);
            a3rely=(a3localx*b3.axisx+a3localy*b3.axisy);
        };
        {
            a4relx=(b4.axisy*a4localx-b4.axisx*a4localy);
            a4rely=(a4localx*b4.axisx+a4localy*b4.axisy);
        };
        var n12x:Float=0.0;
        var n12y:Float=0.0;
        var n34x:Float=0.0;
        var n34y:Float=0.0;
        {
            var t12x:Float=0.0;
            var t12y:Float=0.0;
            var t34x:Float=0.0;
            var t34y:Float=0.0;
            t12x=(b2.posx+a2relx)-(b1.posx+a1relx);
            t12y=(b2.posy+a2rely)-(b1.posy+a1rely);
            t34x=(b4.posx+a4relx)-(b3.posx+a3relx);
            t34y=(b4.posy+a4rely)-(b3.posy+a3rely);
            var C12=ZPP_Math.sqrt((t12x*t12x+t12y*t12y));
            var C34=ZPP_Math.sqrt((t34x*t34x+t34y*t34y));
            if(C12!=0){
                {
                    var t=(1.0/(C12));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t12"+",in s: "+"1.0/(C12)"+",out r: "+"n12"+")");
                        #end
                    };
                    n12x=t12x*t;
                    n12y=t12y*t;
                };
            }
            if(C34!=0){
                {
                    var t=(1.0/(C34));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t34"+",in s: "+"1.0/(C34)"+",out r: "+"n34"+")");
                        #end
                    };
                    n34x=t34x*t;
                    n34y=t34y*t;
                };
                {
                    var t=(ratio);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            else{
                {
                    var t=(ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y)));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y))"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            var C=C12+(ratio*C34);
            if(equal){
                C-=jointMax;
                slack=false;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    {
                        n12x=-n12x;
                        n12y=-n12y;
                    };
                    {
                        n34x=-n34x;
                        n34y=-n34y;
                    };
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    {
                        n12x=0;
                        n12y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12x!=n12x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12y!=n12y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    {
                        n34x=0;
                        n34y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34x!=n34x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34y!=n34y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        return slack;
    }
    public var n12x:Float=0.0;
    public var n12y:Float=0.0;
    public var n34x:Float=0.0;
    public var n34y:Float=0.0;
    public var cx1:Float=0.0;
    public var cx2:Float=0.0;
    public var cx3:Float=0.0;
    public var cx4:Float=0.0;
    public function bodyImpulse(b:ZPP_Body){
        if(stepped){
            var ret=Vec3.get();
            if(b==b1){
                ret.x-=jAcc*n12x;
                ret.y-=jAcc*n12y;
                ret.z-=cx1*jAcc;
            }
            if(b==b2){
                ret.x+=jAcc*n12x;
                ret.y+=jAcc*n12y;
                ret.z+=cx2*jAcc;
            }
            if(b==b3){
                ret.x-=jAcc*n34x;
                ret.y-=jAcc*n34y;
                ret.z-=cx3*jAcc;
            }
            if(b==b4){
                ret.x+=jAcc*n34x;
                ret.y+=jAcc*n34y;
                ret.z+=cx4*jAcc;
            }
            return ret;
        }
        else return Vec3.get(0,0,0);
    }
    public override function activeBodies(){
        {
            if(b1!=null)b1.constraints.add(this);
        };
        if(b2!=b1){
            if(b2!=null)b2.constraints.add(this);
        };
        if(b3!=b1&&b3!=b2){
            if(b3!=null)b3.constraints.add(this);
        };
        if(b4!=b1&&b4!=b2&&b4!=b3){
            if(b4!=null)b4.constraints.add(this);
        };
    }
    public override function inactiveBodies(){
        {
            if(b1!=null)b1.constraints.remove(this);
        };
        if(b2!=b1){
            if(b2!=null)b2.constraints.remove(this);
        };
        if(b3!=b1&&b3!=b2){
            if(b3!=null)b3.constraints.remove(this);
        };
        if(b4!=b1&&b4!=b2&&b4!=b3){
            if(b4!=null)b4.constraints.remove(this);
        };
    }
    public var b1:ZPP_Body=null;
    public var a1localx:Float=0.0;
    public var a1localy:Float=0.0;
    public var a1relx:Float=0.0;
    public var a1rely:Float=0.0;
    private function validate_a1(){
        {
            wrap_a1.zpp_inner.x=a1localx;
            wrap_a1.zpp_inner.y=a1localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a1.zpp_inner.x!=wrap_a1.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a1.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a1.zpp_inner."+",in x: "+"a1localx"+",in y: "+"a1localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a1.zpp_inner.y!=wrap_a1.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a1.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a1.zpp_inner."+",in x: "+"a1localx"+",in y: "+"a1localy"+")");
                #end
            };
        };
    }
    private function invalidate_a1(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a1");
        {
            a1localx=x.x;
            a1localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a1localx!=a1localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a1localx)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a1localy!=a1localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a1localy)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a1(){
        wrap_a1=Vec2.get(a1localx,a1localy);
        wrap_a1.zpp_inner._inuse=true;
        wrap_a1.zpp_inner._validate=validate_a1;
        wrap_a1.zpp_inner._invalidate=invalidate_a1;
    }
    public var wrap_a1:Vec2=null;
    public var b2:ZPP_Body=null;
    public var a2localx:Float=0.0;
    public var a2localy:Float=0.0;
    public var a2relx:Float=0.0;
    public var a2rely:Float=0.0;
    private function validate_a2(){
        {
            wrap_a2.zpp_inner.x=a2localx;
            wrap_a2.zpp_inner.y=a2localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a2.zpp_inner.x!=wrap_a2.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a2.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a2.zpp_inner."+",in x: "+"a2localx"+",in y: "+"a2localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a2.zpp_inner.y!=wrap_a2.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a2.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a2.zpp_inner."+",in x: "+"a2localx"+",in y: "+"a2localy"+")");
                #end
            };
        };
    }
    private function invalidate_a2(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a2");
        {
            a2localx=x.x;
            a2localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a2localx!=a2localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a2localx)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a2localy!=a2localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a2localy)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a2(){
        wrap_a2=Vec2.get(a2localx,a2localy);
        wrap_a2.zpp_inner._inuse=true;
        wrap_a2.zpp_inner._validate=validate_a2;
        wrap_a2.zpp_inner._invalidate=invalidate_a2;
    }
    public var wrap_a2:Vec2=null;
    public var b3:ZPP_Body=null;
    public var a3localx:Float=0.0;
    public var a3localy:Float=0.0;
    public var a3relx:Float=0.0;
    public var a3rely:Float=0.0;
    private function validate_a3(){
        {
            wrap_a3.zpp_inner.x=a3localx;
            wrap_a3.zpp_inner.y=a3localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a3.zpp_inner.x!=wrap_a3.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a3.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a3.zpp_inner."+",in x: "+"a3localx"+",in y: "+"a3localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a3.zpp_inner.y!=wrap_a3.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a3.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a3.zpp_inner."+",in x: "+"a3localx"+",in y: "+"a3localy"+")");
                #end
            };
        };
    }
    private function invalidate_a3(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a3");
        {
            a3localx=x.x;
            a3localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a3localx!=a3localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a3localx)"+") :: "+("vec_set(in n: "+"a3local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a3localy!=a3localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a3localy)"+") :: "+("vec_set(in n: "+"a3local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a3(){
        wrap_a3=Vec2.get(a3localx,a3localy);
        wrap_a3.zpp_inner._inuse=true;
        wrap_a3.zpp_inner._validate=validate_a3;
        wrap_a3.zpp_inner._invalidate=invalidate_a3;
    }
    public var wrap_a3:Vec2=null;
    public var b4:ZPP_Body=null;
    public var a4localx:Float=0.0;
    public var a4localy:Float=0.0;
    public var a4relx:Float=0.0;
    public var a4rely:Float=0.0;
    private function validate_a4(){
        {
            wrap_a4.zpp_inner.x=a4localx;
            wrap_a4.zpp_inner.y=a4localy;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a4.zpp_inner.x!=wrap_a4.zpp_inner.x));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a4.zpp_inner.x)"+") :: "+("vec_set(in n: "+"wrap_a4.zpp_inner."+",in x: "+"a4localx"+",in y: "+"a4localy"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((wrap_a4.zpp_inner.y!=wrap_a4.zpp_inner.y));
                };
                if(!res)throw "assert("+"!assert_isNaN(wrap_a4.zpp_inner.y)"+") :: "+("vec_set(in n: "+"wrap_a4.zpp_inner."+",in x: "+"a4localx"+",in y: "+"a4localy"+")");
                #end
            };
        };
    }
    private function invalidate_a4(x:ZPP_Vec2){
        immutable_midstep("Constraint::"+"a4");
        {
            a4localx=x.x;
            a4localy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a4localx!=a4localx));
                };
                if(!res)throw "assert("+"!assert_isNaN(a4localx)"+") :: "+("vec_set(in n: "+"a4local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((a4localy!=a4localy));
                };
                if(!res)throw "assert("+"!assert_isNaN(a4localy)"+") :: "+("vec_set(in n: "+"a4local"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
         wake();
    }
    public function setup_a4(){
        wrap_a4=Vec2.get(a4localx,a4localy);
        wrap_a4.zpp_inner._inuse=true;
        wrap_a4.zpp_inner._validate=validate_a4;
        wrap_a4.zpp_inner._invalidate=invalidate_a4;
    }
    public var wrap_a4:Vec2=null;
    public var kMass:Float=0.0;
    public var jAcc:Float=0.0;
    public var jMax:Float=0.0;
    public var gamma:Float=0.0;
    public var bias:Float=0.0;
    public var stepped:Bool=false;
    public override function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Constraint{
        var ret=new PulleyJoint(null,null,null,null,outer_zn.anchor1,outer_zn.anchor2,outer_zn.anchor3,outer_zn.anchor4,jointMin,jointMax,ratio);
        copyto(ret);
        {
            if(dict!=null&&b1!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b1.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b1=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b1.id,function(b:Body)ret.zpp_inner_zn.b1=b.zpp_inner));
            }
        };
        {
            if(dict!=null&&b2!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b2.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b2=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b2.id,function(b:Body)ret.zpp_inner_zn.b2=b.zpp_inner));
            }
        };
        {
            if(dict!=null&&b3!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b3.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b3=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b3.id,function(b:Body)ret.zpp_inner_zn.b3=b.zpp_inner));
            }
        };
        {
            if(dict!=null&&b4!=null){
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        todo!=null;
                    };
                    if(!res)throw "assert("+"todo!=null"+") :: "+("dict non-null,but todo is in constraint copy?");
                    #end
                };
                var b:Body=null;
                for(idc in dict){
                    if(idc.id==b4.id){
                        b=idc.bc;
                        break;
                    }
                }
                if(b!=null)ret.zpp_inner_zn.b4=b.zpp_inner;
                else todo.push(ZPP_CopyHelper.todo(b4.id,function(b:Body)ret.zpp_inner_zn.b4=b.zpp_inner));
            }
        };
        return ret;
    }
    public function new(){
        super();
        {
            {
                a1localx=0;
                a1localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1localx!=a1localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1localx)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1localy!=a1localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1localy)"+") :: "+("vec_set(in n: "+"a1local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a1relx=0;
                a1rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1relx!=a1relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1relx)"+") :: "+("vec_set(in n: "+"a1rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a1rely!=a1rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a1rely)"+") :: "+("vec_set(in n: "+"a1rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
        {
            {
                a2localx=0;
                a2localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2localx!=a2localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2localx)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2localy!=a2localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2localy)"+") :: "+("vec_set(in n: "+"a2local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a2relx=0;
                a2rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2relx!=a2relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2relx)"+") :: "+("vec_set(in n: "+"a2rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a2rely!=a2rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a2rely)"+") :: "+("vec_set(in n: "+"a2rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
        {
            {
                a3localx=0;
                a3localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a3localx!=a3localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a3localx)"+") :: "+("vec_set(in n: "+"a3local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a3localy!=a3localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a3localy)"+") :: "+("vec_set(in n: "+"a3local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a3relx=0;
                a3rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a3relx!=a3relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a3relx)"+") :: "+("vec_set(in n: "+"a3rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a3rely!=a3rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a3rely)"+") :: "+("vec_set(in n: "+"a3rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
        {
            {
                a4localx=0;
                a4localy=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a4localx!=a4localx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a4localx)"+") :: "+("vec_set(in n: "+"a4local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a4localy!=a4localy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a4localy)"+") :: "+("vec_set(in n: "+"a4local"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
            {
                a4relx=0;
                a4rely=0;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a4relx!=a4relx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a4relx)"+") :: "+("vec_set(in n: "+"a4rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((a4rely!=a4rely));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(a4rely)"+") :: "+("vec_set(in n: "+"a4rel"+",in x: "+"0"+",in y: "+"0"+")");
                    #end
                };
            };
        };
        {
            n12x=1;
            n12y=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n12x!=n12x));
                };
                if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"1"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n12y!=n12y));
                };
                if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"1"+",in y: "+"0"+")");
                #end
            };
        };
        {
            n34x=1;
            n34y=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n34x!=n34x));
                };
                if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"1"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n34y!=n34y));
                };
                if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"1"+",in y: "+"0"+")");
                #end
            };
        };
        jAcc=0;
        jMax=ZPP_Const.POSINF();
        stepped=false;
        cx1=cx2=cx3=cx4=0;
    }
    public override function validate(){
        if(b1==null||b2==null||b3==null||b4==null)throw "Error: PulleyJoint cannot be simulated with null bodies";
        if(b1==b2||b3==b4)throw "Error: PulleyJoint cannot have body1==body2 or body3==body4";
        if(b1.space!=space||b2.space!=space||b3.space!=space||b4.space!=space)throw "Error: Constraints must have each body within the same space to which the constraint has been assigned";
        if(jointMin>jointMax)throw "Error: PulleyJoint must have jointMin <= jointMax";
        if(!b1.isDynamic()&&!b2.isDynamic())throw "Error: PulleyJoint cannot have both bodies in a linked pair non-dynamic";
        if(!b3.isDynamic()&&!b4.isDynamic())throw "Error: PulleyJoint cannot have both bodies in a linked pair non-dynamic";
    }
    public override function wake_connected(){
        if(b1!=null&&b1.isDynamic())b1.wake();
        if(b2!=null&&b2.isDynamic())b2.wake();
        if(b3!=null&&b3.isDynamic())b3.wake();
        if(b4!=null&&b4.isDynamic())b4.wake();
    }
    public override function forest(){
        if(b1.isDynamic()){
            var xr=({
                if(b1.component==b1.component.parent)b1.component;
                else{
                    var obj=b1.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
        if(b2.isDynamic()){
            var xr=({
                if(b2.component==b2.component.parent)b2.component;
                else{
                    var obj=b2.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
        if(b3.isDynamic()){
            var xr=({
                if(b3.component==b3.component.parent)b3.component;
                else{
                    var obj=b3.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
        if(b4.isDynamic()){
            var xr=({
                if(b4.component==b4.component.parent)b4.component;
                else{
                    var obj=b4.component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            var yr=({
                if(component==component.parent)component;
                else{
                    var obj=component;
                    var stack=null;
                    while(obj!=obj.parent){
                        var nxt=obj.parent;
                        obj.parent=stack;
                        stack=obj;
                        obj=nxt;
                    }
                    while(stack!=null){
                        var nxt=stack.parent;
                        stack.parent=obj;
                        stack=nxt;
                    }
                    obj;
                }
            });
            if(xr!=yr){
                if(xr.rank<yr.rank)xr.parent=yr;
                else if(xr.rank>yr.rank)yr.parent=xr;
                else{
                    yr.parent=xr;
                    xr.rank++;
                }
            }
        };
    }
    public override function pair_exists(id:Int,di:Int){
        return(b1.id==id&&(b2.id==di||b3.id==di||b4.id==di))||(b2.id==id&&(b3.id==di||b4.id==di||b1.id==di))||(b3.id==id&&(b4.id==di||b1.id==di||b2.id==di))||(b4.id==id&&(b1.id==di||b2.id==di||b3.id==di));
    }
    public override function clearcache(){
        jAcc=0;
        pre_dt=-1.0;
    }
    public override function preStep(dt:Float){
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        stepped=true;
        equal=jointMin==jointMax;
        {
            a1relx=(b1.axisy*a1localx-b1.axisx*a1localy);
            a1rely=(a1localx*b1.axisx+a1localy*b1.axisy);
        };
        {
            a2relx=(b2.axisy*a2localx-b2.axisx*a2localy);
            a2rely=(a2localx*b2.axisx+a2localy*b2.axisy);
        };
        {
            a3relx=(b3.axisy*a3localx-b3.axisx*a3localy);
            a3rely=(a3localx*b3.axisx+a3localy*b3.axisy);
        };
        {
            a4relx=(b4.axisy*a4localx-b4.axisx*a4localy);
            a4rely=(a4localx*b4.axisx+a4localy*b4.axisy);
        };
        var C={
            var t12x:Float=0.0;
            var t12y:Float=0.0;
            var t34x:Float=0.0;
            var t34y:Float=0.0;
            t12x=(b2.posx+a2relx)-(b1.posx+a1relx);
            t12y=(b2.posy+a2rely)-(b1.posy+a1rely);
            t34x=(b4.posx+a4relx)-(b3.posx+a3relx);
            t34y=(b4.posy+a4rely)-(b3.posy+a3rely);
            var C12=ZPP_Math.sqrt((t12x*t12x+t12y*t12y));
            var C34=ZPP_Math.sqrt((t34x*t34x+t34y*t34y));
            if(C12!=0){
                {
                    var t=(1.0/(C12));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t12"+",in s: "+"1.0/(C12)"+",out r: "+"n12"+")");
                        #end
                    };
                    n12x=t12x*t;
                    n12y=t12y*t;
                };
            }
            if(C34!=0){
                {
                    var t=(1.0/(C34));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t34"+",in s: "+"1.0/(C34)"+",out r: "+"n34"+")");
                        #end
                    };
                    n34x=t34x*t;
                    n34y=t34y*t;
                };
                {
                    var t=(ratio);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            else{
                {
                    var t=(ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y)));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y))"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            var C=C12+(ratio*C34);
            if(equal){
                C-=jointMax;
                slack=false;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    {
                        n12x=-n12x;
                        n12y=-n12y;
                    };
                    {
                        n34x=-n34x;
                        n34y=-n34y;
                    };
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    {
                        n12x=0;
                        n12y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12x!=n12x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12y!=n12y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    {
                        n34x=0;
                        n34y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34x!=n34x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34y!=n34y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        if(!slack){
            kMass={
                cx1=(n12y*a1relx-n12x*a1rely);
                cx2=(n12y*a2relx-n12x*a2rely);
                cx3=(n34y*a3relx-n34x*a3rely);
                cx4=(n34y*a4relx-n34x*a4rely);
                var ratioSq=ratio*ratio;
                var K=b1.smass+b2.smass+ratioSq*(b3.smass+b4.smass)+b1.sinertia*cx1*cx1+b2.sinertia*cx2*cx2+b3.sinertia*cx3*cx3+b4.sinertia*cx4*cx4;
                if(b1==b4)K-=2*(((n12x*n34x+n12y*n34y)*b1.smass)+(cx1*cx4*b1.sinertia));
                if(b1==b3)K+=2*(((n12x*n34x+n12y*n34y)*b1.smass)+(cx1*cx3*b1.sinertia));
                if(b2==b3)K-=2*(((n12x*n34x+n12y*n34y)*b2.smass)+(cx2*cx3*b2.sinertia));
                if(b2==b4)K+=2*(((n12x*n34x+n12y*n34y)*b2.smass)+(cx2*cx4*b2.sinertia));
                K;
            };
            if(kMass!=0)kMass=1/kMass;
            else jAcc=0;
            if(!stiff){
                if(breakUnderError&&C*C>maxError*maxError)return true;
                var biasCoef;
                kMass*={
                    var omega=2*Math.PI*frequency;
                    gamma=1/(dt*omega*(2*damping+omega*dt));
                    var ig=1/(1+gamma);
                    biasCoef=dt*omega*omega*gamma;
                    gamma*=ig;
                    ig;
                };
                bias=-C*biasCoef;
                {
                    if(bias<-maxError)bias=-maxError;
                    else if(bias>maxError)bias=maxError;
                };
            }
            else{
                bias=0;
                gamma=0;
            }
            jAcc*=dtratio;
            jMax=maxForce*dt;
        }
        return false;
    }
    public override function warmStart(){
        if(!slack){
            if(false){
                {
                    var t=(b1.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n12"+",in s: "+"b1.imass*jAcc"+")");
                        #end
                    };
                    b1.posx-=n12x*t;
                    b1.posy-=n12y*t;
                };
                {
                    var t=(b2.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n12"+",in s: "+"b2.imass*jAcc"+")");
                        #end
                    };
                    b2.posx+=n12x*t;
                    b2.posy+=n12y*t;
                };
                {
                    var t=(b3.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.pos"+",in b: "+"n34"+",in s: "+"b3.imass*jAcc"+")");
                        #end
                    };
                    b3.posx-=n34x*t;
                    b3.posy-=n34y*t;
                };
                {
                    var t=(b4.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.pos"+",in b: "+"n34"+",in s: "+"b4.imass*jAcc"+")");
                        #end
                    };
                    b4.posx+=n34x*t;
                    b4.posy+=n34y*t;
                };
                b1.delta_rot(-cx1*b1.iinertia*jAcc);
                b2.delta_rot(cx2*b2.iinertia*jAcc);
                b3.delta_rot(-cx3*b3.iinertia*jAcc);
                b4.delta_rot(cx4*b4.iinertia*jAcc);
            }
            else{
                {
                    var t=(b1.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n12"+",in s: "+"b1.imass*jAcc"+")");
                        #end
                    };
                    b1.velx-=n12x*t;
                    b1.vely-=n12y*t;
                };
                {
                    var t=(b2.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n12"+",in s: "+"b2.imass*jAcc"+")");
                        #end
                    };
                    b2.velx+=n12x*t;
                    b2.vely+=n12y*t;
                };
                {
                    var t=(b3.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.vel"+",in b: "+"n34"+",in s: "+"b3.imass*jAcc"+")");
                        #end
                    };
                    b3.velx-=n34x*t;
                    b3.vely-=n34y*t;
                };
                {
                    var t=(b4.imass*jAcc);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.vel"+",in b: "+"n34"+",in s: "+"b4.imass*jAcc"+")");
                        #end
                    };
                    b4.velx+=n34x*t;
                    b4.vely+=n34y*t;
                };
                b1.angvel-=cx1*b1.iinertia*jAcc;
                b2.angvel+=cx2*b2.iinertia*jAcc;
                b3.angvel-=cx3*b3.iinertia*jAcc;
                b4.angvel+=cx4*b4.iinertia*jAcc;
            }
        };
    }
    public override function applyImpulseVel(){
        if(slack)return false;
        var E={
            n12x*(b2.velx+b2.kinvelx-b1.velx-b1.kinvelx)+n12y*(b2.vely+b2.kinvely-b1.vely-b1.kinvely)+n34x*(b4.velx+b4.kinvelx-b3.velx-b3.kinvelx)+n34y*(b4.vely+b4.kinvely-b3.vely-b3.kinvely)+(b2.angvel+b2.kinangvel)*cx2-(b1.angvel+b1.kinangvel)*cx1+(b4.angvel+b4.kinangvel)*cx4-(b3.angvel+b3.kinangvel)*cx3;
        };
        var j=kMass*(bias-E)-jAcc*gamma;
        {
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((j!=j));
                };
                if(!res)throw "assert("+"!assert_isNaN(j)"+") :: "+("accum nan");
                #end
            };
            var jOld=jAcc;
            jAcc+=j;
            {
                if(!equal&&jAcc>0)jAcc=0;
                if(breakUnderForce&&jAcc<-jMax)return true;
                if(!stiff){
                    if(jAcc<-jMax)jAcc=-jMax;
                }
            };
            j=jAcc-jOld;
        };
        {
            if(false){
                {
                    var t=(b1.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n12"+",in s: "+"b1.imass*j"+")");
                        #end
                    };
                    b1.posx-=n12x*t;
                    b1.posy-=n12y*t;
                };
                {
                    var t=(b2.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n12"+",in s: "+"b2.imass*j"+")");
                        #end
                    };
                    b2.posx+=n12x*t;
                    b2.posy+=n12y*t;
                };
                {
                    var t=(b3.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.pos"+",in b: "+"n34"+",in s: "+"b3.imass*j"+")");
                        #end
                    };
                    b3.posx-=n34x*t;
                    b3.posy-=n34y*t;
                };
                {
                    var t=(b4.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.pos"+",in b: "+"n34"+",in s: "+"b4.imass*j"+")");
                        #end
                    };
                    b4.posx+=n34x*t;
                    b4.posy+=n34y*t;
                };
                b1.delta_rot(-cx1*b1.iinertia*j);
                b2.delta_rot(cx2*b2.iinertia*j);
                b3.delta_rot(-cx3*b3.iinertia*j);
                b4.delta_rot(cx4*b4.iinertia*j);
            }
            else{
                {
                    var t=(b1.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n12"+",in s: "+"b1.imass*j"+")");
                        #end
                    };
                    b1.velx-=n12x*t;
                    b1.vely-=n12y*t;
                };
                {
                    var t=(b2.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n12"+",in s: "+"b2.imass*j"+")");
                        #end
                    };
                    b2.velx+=n12x*t;
                    b2.vely+=n12y*t;
                };
                {
                    var t=(b3.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.vel"+",in b: "+"n34"+",in s: "+"b3.imass*j"+")");
                        #end
                    };
                    b3.velx-=n34x*t;
                    b3.vely-=n34y*t;
                };
                {
                    var t=(b4.imass*j);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.vel"+",in b: "+"n34"+",in s: "+"b4.imass*j"+")");
                        #end
                    };
                    b4.velx+=n34x*t;
                    b4.vely+=n34y*t;
                };
                b1.angvel-=cx1*b1.iinertia*j;
                b2.angvel+=cx2*b2.iinertia*j;
                b3.angvel-=cx3*b3.iinertia*j;
                b4.angvel+=cx4*b4.iinertia*j;
            }
        };
        return false;
    }
    public override function applyImpulsePos(){
        var E;
        var j;
        var r1x:Float=0.0;
        var r1y:Float=0.0;
        {
            {
                r1x=(b1.axisy*a1localx-b1.axisx*a1localy);
                r1y=(a1localx*b1.axisx+a1localy*b1.axisy);
            };
        };
        var r2x:Float=0.0;
        var r2y:Float=0.0;
        {
            {
                r2x=(b2.axisy*a2localx-b2.axisx*a2localy);
                r2y=(a2localx*b2.axisx+a2localy*b2.axisy);
            };
        };
        var r3x:Float=0.0;
        var r3y:Float=0.0;
        {
            {
                r3x=(b3.axisy*a3localx-b3.axisx*a3localy);
                r3y=(a3localx*b3.axisx+a3localy*b3.axisy);
            };
        };
        var r4x:Float=0.0;
        var r4y:Float=0.0;
        {
            {
                r4x=(b4.axisy*a4localx-b4.axisx*a4localy);
                r4y=(a4localx*b4.axisx+a4localy*b4.axisy);
            };
        };
        var slack;
        var n12x:Float=0.0;
        var n12y:Float=0.0;
        var n34x:Float=0.0;
        var n34y:Float=0.0;
        {
            n12x=this.n12x;
            n12y=this.n12y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n12x!=n12x));
                };
                if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"this.n12x"+",in y: "+"this.n12y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n12y!=n12y));
                };
                if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"this.n12x"+",in y: "+"this.n12y"+")");
                #end
            };
        };
        {
            n34x=this.n34x;
            n34y=this.n34y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n34x!=n34x));
                };
                if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"this.n34x"+",in y: "+"this.n34y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((n34y!=n34y));
                };
                if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"this.n34x"+",in y: "+"this.n34y"+")");
                #end
            };
        };
        E={
            var t12x:Float=0.0;
            var t12y:Float=0.0;
            var t34x:Float=0.0;
            var t34y:Float=0.0;
            t12x=(b2.posx+r2x)-(b1.posx+r1x);
            t12y=(b2.posy+r2y)-(b1.posy+r1y);
            t34x=(b4.posx+r4x)-(b3.posx+r3x);
            t34y=(b4.posy+r4y)-(b3.posy+r3y);
            var C12=ZPP_Math.sqrt((t12x*t12x+t12y*t12y));
            var C34=ZPP_Math.sqrt((t34x*t34x+t34y*t34y));
            if(C12!=0){
                {
                    var t=(1.0/(C12));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t12"+",in s: "+"1.0/(C12)"+",out r: "+"n12"+")");
                        #end
                    };
                    n12x=t12x*t;
                    n12y=t12y*t;
                };
            }
            if(C34!=0){
                {
                    var t=(1.0/(C34));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t34"+",in s: "+"1.0/(C34)"+",out r: "+"n34"+")");
                        #end
                    };
                    n34x=t34x*t;
                    n34y=t34y*t;
                };
                {
                    var t=(ratio);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            else{
                {
                    var t=(ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y)));
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y))"+")");
                        #end
                    };
                    n34x*=t;
                    n34y*=t;
                };
            }
            var C=C12+(ratio*C34);
            if(equal){
                C-=jointMax;
                slack=false;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    {
                        n12x=-n12x;
                        n12y=-n12y;
                    };
                    {
                        n34x=-n34x;
                        n34y=-n34y;
                    };
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    slack=false;
                }
                else{
                    {
                        n12x=0;
                        n12y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12x!=n12x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n12y!=n12y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    {
                        n34x=0;
                        n34y=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34x!=n34x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((n34y!=n34y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        if(!slack){
            if(breakUnderError&&E*E>maxError*maxError)return true;
            if(E*E<Config.constraintLinearSlop*Config.constraintLinearSlop)return false;
            E*=0.5;
            if(E*E>6){
                var k=b1.smass+b2.smass;
                if(k>Config.epsilon){
                    k=0.75/k;
                    j=-E*k;
                    if(equal||j<0){
                        {
                            var t=(j*b1.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n12"+",in s: "+"j*b1.imass"+")");
                                #end
                            };
                            b1.posx-=n12x*t;
                            b1.posy-=n12y*t;
                        };
                        {
                            var t=(j*b2.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n12"+",in s: "+"j*b2.imass"+")");
                                #end
                            };
                            b2.posx+=n12x*t;
                            b2.posy+=n12y*t;
                        };
                        {
                            var t=(j*b3.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.pos"+",in b: "+"n34"+",in s: "+"j*b3.imass"+")");
                                #end
                            };
                            b3.posx-=n34x*t;
                            b3.posy-=n34y*t;
                        };
                        {
                            var t=(j*b4.imass);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.pos"+",in b: "+"n34"+",in s: "+"j*b4.imass"+")");
                                #end
                            };
                            b4.posx+=n34x*t;
                            b4.posy+=n34y*t;
                        };
                        E={
                            var t12x:Float=0.0;
                            var t12y:Float=0.0;
                            var t34x:Float=0.0;
                            var t34y:Float=0.0;
                            t12x=(b2.posx+r2x)-(b1.posx+r1x);
                            t12y=(b2.posy+r2y)-(b1.posy+r1y);
                            t34x=(b4.posx+r4x)-(b3.posx+r3x);
                            t34y=(b4.posy+r4y)-(b3.posy+r3y);
                            var C12=ZPP_Math.sqrt((t12x*t12x+t12y*t12y));
                            var C34=ZPP_Math.sqrt((t34x*t34x+t34y*t34y));
                            if(C12!=0){
                                {
                                    var t=(1.0/(C12));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t12"+",in s: "+"1.0/(C12)"+",out r: "+"n12"+")");
                                        #end
                                    };
                                    n12x=t12x*t;
                                    n12y=t12y*t;
                                };
                            }
                            if(C34!=0){
                                {
                                    var t=(1.0/(C34));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"t34"+",in s: "+"1.0/(C34)"+",out r: "+"n34"+")");
                                        #end
                                    };
                                    n34x=t34x*t;
                                    n34y=t34y*t;
                                };
                                {
                                    var t=(ratio);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio"+")");
                                        #end
                                    };
                                    n34x*=t;
                                    n34y*=t;
                                };
                            }
                            else{
                                {
                                    var t=(ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y)));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n34"+",in s: "+"ratio/ZPP_Math.sqrt((n34x*n34x+n34y*n34y))"+")");
                                        #end
                                    };
                                    n34x*=t;
                                    n34y*=t;
                                };
                            }
                            var C=C12+(ratio*C34);
                            if(equal){
                                C-=jointMax;
                                slack=false;
                            }
                            else{
                                if(C<jointMin){
                                    C=jointMin-C;
                                    {
                                        n12x=-n12x;
                                        n12y=-n12y;
                                    };
                                    {
                                        n34x=-n34x;
                                        n34y=-n34y;
                                    };
                                    slack=false;
                                }
                                else if(C>jointMax){
                                    C-=jointMax;
                                    slack=false;
                                }
                                else{
                                    {
                                        n12x=0;
                                        n12y=0;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((n12x!=n12x));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(n12x)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                                            #end
                                        };
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((n12y!=n12y));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(n12y)"+") :: "+("vec_set(in n: "+"n12"+",in x: "+"0"+",in y: "+"0"+")");
                                            #end
                                        };
                                    };
                                    {
                                        n34x=0;
                                        n34y=0;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((n34x!=n34x));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(n34x)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                                            #end
                                        };
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((n34y!=n34y));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(n34y)"+") :: "+("vec_set(in n: "+"n34"+",in x: "+"0"+",in y: "+"0"+")");
                                            #end
                                        };
                                    };
                                    C=0;
                                    slack=true;
                                }
                            }
                            C;
                        };
                        E*=0.5;
                    }
                }
            }
            var cx1,cx2,cx3,cx4;
            var k={
                cx1=(n12y*r1x-n12x*r1y);
                cx2=(n12y*r2x-n12x*r2y);
                cx3=(n34y*r3x-n34x*r3y);
                cx4=(n34y*r4x-n34x*r4y);
                var ratioSq=ratio*ratio;
                var K=b1.smass+b2.smass+ratioSq*(b3.smass+b4.smass)+b1.sinertia*cx1*cx1+b2.sinertia*cx2*cx2+b3.sinertia*cx3*cx3+b4.sinertia*cx4*cx4;
                if(b1==b4)K-=2*(((n12x*n34x+n12y*n34y)*b1.smass)+(cx1*cx4*b1.sinertia));
                if(b1==b3)K+=2*(((n12x*n34x+n12y*n34y)*b1.smass)+(cx1*cx3*b1.sinertia));
                if(b2==b3)K-=2*(((n12x*n34x+n12y*n34y)*b2.smass)+(cx2*cx3*b2.sinertia));
                if(b2==b4)K+=2*(((n12x*n34x+n12y*n34y)*b2.smass)+(cx2*cx4*b2.sinertia));
                K;
            };
            if(k!=0)k=1/k;
            j=-E*k;
            if(equal||j<0){
                if(true){
                    {
                        var t=(b1.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.pos"+",in b: "+"n12"+",in s: "+"b1.imass*j"+")");
                            #end
                        };
                        b1.posx-=n12x*t;
                        b1.posy-=n12y*t;
                    };
                    {
                        var t=(b2.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.pos"+",in b: "+"n12"+",in s: "+"b2.imass*j"+")");
                            #end
                        };
                        b2.posx+=n12x*t;
                        b2.posy+=n12y*t;
                    };
                    {
                        var t=(b3.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.pos"+",in b: "+"n34"+",in s: "+"b3.imass*j"+")");
                            #end
                        };
                        b3.posx-=n34x*t;
                        b3.posy-=n34y*t;
                    };
                    {
                        var t=(b4.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.pos"+",in b: "+"n34"+",in s: "+"b4.imass*j"+")");
                            #end
                        };
                        b4.posx+=n34x*t;
                        b4.posy+=n34y*t;
                    };
                    b1.delta_rot(-cx1*b1.iinertia*j);
                    b2.delta_rot(cx2*b2.iinertia*j);
                    b3.delta_rot(-cx3*b3.iinertia*j);
                    b4.delta_rot(cx4*b4.iinertia*j);
                }
                else{
                    {
                        var t=(b1.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b1.vel"+",in b: "+"n12"+",in s: "+"b1.imass*j"+")");
                            #end
                        };
                        b1.velx-=n12x*t;
                        b1.vely-=n12y*t;
                    };
                    {
                        var t=(b2.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b2.vel"+",in b: "+"n12"+",in s: "+"b2.imass*j"+")");
                            #end
                        };
                        b2.velx+=n12x*t;
                        b2.vely+=n12y*t;
                    };
                    {
                        var t=(b3.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"b3.vel"+",in b: "+"n34"+",in s: "+"b3.imass*j"+")");
                            #end
                        };
                        b3.velx-=n34x*t;
                        b3.vely-=n34y*t;
                    };
                    {
                        var t=(b4.imass*j);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b4.vel"+",in b: "+"n34"+",in s: "+"b4.imass*j"+")");
                            #end
                        };
                        b4.velx+=n34x*t;
                        b4.vely+=n34y*t;
                    };
                    b1.angvel-=cx1*b1.iinertia*j;
                    b2.angvel+=cx2*b2.iinertia*j;
                    b3.angvel-=cx3*b3.iinertia*j;
                    b4.angvel+=cx4*b4.iinertia*j;
                }
            };
        }
        return false;
    }
    public override function draw(g:Debug){
        #if(flash9||openfl||nme)var me=outer_zn;
        var a1=me.body1.localPointToWorld(me.anchor1);
        var a2=me.body2.localPointToWorld(me.anchor2);
        var a3=me.body3.localPointToWorld(me.anchor3);
        var a4=me.body4.localPointToWorld(me.anchor4);
        var n12=a2.sub(a1);
        var n34=a4.sub(a3);
        var nl12=n12.length;
        var nl34=n34.length;
        drawLink(g,a1,a2,n12,nl12,nl34*ratio,1.0,0xffff00,0x00ffff);
        drawLink(g,a3,a4,n34,nl34,nl12,(1/ratio),0x00ffff,0xff00ff);
        g.drawFilledCircle(a1,2,0xff);
        g.drawFilledCircle(a2,2,0xff0000);
        g.drawFilledCircle(a3,2,0xff00);
        g.drawFilledCircle(a4,2,0xff00ff);
        a1.dispose();
        a2.dispose();
        a3.dispose();
        a4.dispose();
        n12.dispose();
        n34.dispose();
        #end
    }
    #if(flash9||openfl||nme)public function drawLink(g:Debug,a1:Vec2,a2:Vec2,n:Vec2,nl:Float,bias:Float,scale:Float,ca:Int,cb:Int){
        if(nl!=0){
            n.muleq(1/nl);
            var mid=a1.add(a2).muleq(0.5);
            var cmin=(jointMin-bias)*scale;
            if(cmin<0)cmin=0;
            var cmax=(jointMax-bias)*scale;
            if(cmax<0)cmax=0;
            var min1=mid.sub(n.mul(cmin*0.5,true));
            var min2=mid.add(n.mul(cmin*0.5,true));
            var max1=mid.sub(n.mul(cmax*0.5,true));
            var max2=mid.add(n.mul(cmax*0.5,true));
            g.drawLine(min1,min2,ca);
            g.drawLine(max1,min1,cb);
            g.drawLine(max2,min2,cb);
            if(!stiff){
                if(nl>cmax){
                    g.drawSpring(max1,a1,cb);
                    g.drawSpring(max2,a2,cb);
                }
                else if(nl<cmin){
                    g.drawSpring(min1,a1,ca);
                    g.drawSpring(min2,a2,ca);
                }
            }
            mid.dispose();
            min1.dispose();
            min2.dispose();
            max1.dispose();
            max2.dispose();
        }
    }
    #end
}

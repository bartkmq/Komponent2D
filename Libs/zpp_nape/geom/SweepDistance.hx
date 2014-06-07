package zpp_nape.geom;
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
class ZPP_ToiEvent{
    public var next:ZPP_ToiEvent=null;
    static public var zpp_pool:ZPP_ToiEvent=null;
    #if NAPE_POOL_STATS 
    /**
     * @private
     */
    static public var POOL_CNT:Int=0;
    /**
     * @private
     */
    static public var POOL_TOT:Int=0;
    /**
     * @private
     */
    static public var POOL_ADD:Int=0;
    /**
     * @private
     */
    static public var POOL_ADDNEW:Int=0;
    /**
     * @private
     */
    static public var POOL_SUB:Int=0;
    #end
    
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){
        failed=false;
        s1=s2=null;
        arbiter=null;
    }
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){}
    public var toi:Float=0.0;
    public var s1:ZPP_Shape=null;
    public var s2:ZPP_Shape=null;
    public var arbiter:ZPP_ColArbiter=null;
    public var frozen1:Bool=false;
    public var frozen2:Bool=false;
    public var c1:ZPP_Vec2=null;
    public var c2:ZPP_Vec2=null;
    public var axis:ZPP_Vec2=null;
    public var slipped:Bool=false;
    public var failed:Bool=false;
    public var kinematic:Bool=false;
    public function new(){
        c1=new ZPP_Vec2();
        c2=new ZPP_Vec2();
        axis=new ZPP_Vec2();
    }
}
#if nape_swc@:keep #end
class ZPP_SweepDistance{
    static public function dynamicSweep(toi:ZPP_ToiEvent,timeStep:Float,lowerBound:Float,negRadius:Float,userAPI:Bool=false){
        var s1:ZPP_Shape=toi.s1;
        var s2:ZPP_Shape=toi.s2;
        var b1:ZPP_Body=s1.body;
        var b2:ZPP_Body=s2.body;
        var deltax:Float=0.0;
        var deltay:Float=0.0;
        {
            deltax=b2.velx-b1.velx;
            deltay=b2.vely-b1.vely;
        };
        var ang1=b1.angvel;
        if(ang1<0)ang1=-ang1;
        var ang2=b2.angvel;
        if(ang2<0)ang2=-ang2;
        var angBias=(s1.sweepCoef*ang1)+(s2.sweepCoef*ang2);
        if(!userAPI&&!toi.kinematic&&(deltax*deltax+deltay*deltay)<(Config.dynamicSweepLinearThreshold*Config.dynamicSweepLinearThreshold)&&angBias<Config.dynamicSweepAngularThreshold){
            toi.toi=-1;
            toi.failed=true;
            return;
        }
        var c1=toi.c1;
        var c2=toi.c2;
        var axis=toi.axis;
        var curTOI=lowerBound;
        var curIter=0;
        while(true){
            b1.sweepIntegrate(curTOI*timeStep);
            b1.sweepValidate(s1);
            b2.sweepIntegrate(curTOI*timeStep);
            b2.sweepValidate(s2);
            var sep=distance(s1,s2,c1,c2,axis)+negRadius;
            var dot=(deltax*axis.x+deltay*axis.y);
            if(sep<Config.distanceThresholdCCD){
                if(userAPI){
                    break;
                }
                var d1x:Float=0.0;
                var d1y:Float=0.0;
                {
                    d1x=c1.x-b1.posx;
                    d1y=c1.y-b1.posy;
                };
                var d2x:Float=0.0;
                var d2y:Float=0.0;
                {
                    d2x=c2.x-b2.posx;
                    d2y=c2.y-b2.posy;
                };
                var proj=dot-b1.sweep_angvel*(axis.y*d1x-axis.x*d1y)+b2.sweep_angvel*(axis.y*d2x-axis.x*d2y);
                if(proj>0){
                    toi.slipped=true;
                }
                if(proj<=0||sep<Config.distanceThresholdCCD*0.5){
                    break;
                }
            }
            var denom=(angBias-dot)*timeStep;
            if(denom<=0){
                curTOI=-1;
                break;
            }
            var delta=sep/denom;
            if(delta<1e-6)delta=1e-6;
            curTOI+=delta;
            if(curTOI>=1){
                curTOI=1;
                b1.sweepIntegrate(curTOI*timeStep);
                b1.sweepValidate(s1);
                b2.sweepIntegrate(curTOI*timeStep);
                b2.sweepValidate(s2);
                var sep=distance(s1,s2,c1,c2,axis)+negRadius;
                var dot=(deltax*axis.x+deltay*axis.y);
                if(sep<Config.distanceThresholdCCD){
                    if(userAPI){
                        break;
                    }
                    var d1x:Float=0.0;
                    var d1y:Float=0.0;
                    {
                        d1x=c1.x-b1.posx;
                        d1y=c1.y-b1.posy;
                    };
                    var d2x:Float=0.0;
                    var d2y:Float=0.0;
                    {
                        d2x=c2.x-b2.posx;
                        d2y=c2.y-b2.posy;
                    };
                    var proj=dot-b1.sweep_angvel*(axis.y*d1x-axis.x*d1y)+b2.sweep_angvel*(axis.y*d2x-axis.x*d2y);
                    if(proj>0){
                        toi.slipped=true;
                    }
                    if(proj<=0||sep<Config.distanceThresholdCCD*0.5){
                        break;
                    }
                }
                curTOI=-1;
                break;
            }
            if((++curIter)>=40){
                if(sep>negRadius){
                    toi.failed=true;
                }
                break;
            }
        }
        toi.toi=curTOI;
    }
    static public function staticSweep(toi:ZPP_ToiEvent,timeStep:Float,lowerBound:Float,negRadius:Float){
        var s1:ZPP_Shape=toi.s1;
        var s2:ZPP_Shape=toi.s2;
        var b1:ZPP_Body=s1.body;
        var b2:ZPP_Body=s2.body;
        var deltax:Float=0.0;
        var deltay:Float=0.0;
        {
            deltax=-b1.velx;
            deltay=-b1.vely;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((deltax!=deltax));
                };
                if(!res)throw "assert("+"!assert_isNaN(deltax)"+") :: "+("vec_set(in n: "+"delta"+",in x: "+"-b1.velx"+",in y: "+"-b1.vely"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((deltay!=deltay));
                };
                if(!res)throw "assert("+"!assert_isNaN(deltay)"+") :: "+("vec_set(in n: "+"delta"+",in x: "+"-b1.velx"+",in y: "+"-b1.vely"+")");
                #end
            };
        };
        var ang1=b1.sweep_angvel;
        if(ang1<0)ang1=-ang1;
        var angBias=(s1.sweepCoef*ang1);
        var c1=toi.c1;
        var c2=toi.c2;
        var axis=toi.axis;
        var curTOI=lowerBound;
        var curIter=0;
        while(true){
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(curTOI*timeStep!=curTOI*timeStep);
                };
                if(!res)throw "assert("+"!(curTOI*timeStep!=curTOI*timeStep)"+") :: "+(curTOI+" "+timeStep);
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(b1.sweepTime!=b1.sweepTime);
                };
                if(!res)throw "assert("+"!(b1.sweepTime!=b1.sweepTime)"+") :: "+("sweeeeep");
                #end
            };
            b1.sweepIntegrate(curTOI*timeStep);
            b1.sweepValidate(s1);
            var sep=distance(s1,s2,c1,c2,axis)+negRadius;
            var dot=(deltax*axis.x+deltay*axis.y);
            if(sep<Config.distanceThresholdCCD){
                var d1x:Float=0.0;
                var d1y:Float=0.0;
                {
                    d1x=c1.x-b1.posx;
                    d1y=c1.y-b1.posy;
                };
                var proj=dot-b1.sweep_angvel*(axis.y*d1x-axis.x*d1y);
                if(proj>0){
                    toi.slipped=true;
                }
                if(proj<=0||sep<Config.distanceThresholdCCD*0.5){
                    break;
                }
            }
            var denom=(angBias-dot)*timeStep;
            if(denom<=0){
                curTOI=-1;
                break;
            }
            var delta=sep/denom;
            if(delta<1e-6)delta=1e-6;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !(delta!=delta);
                };
                if(!res)throw "assert("+"!(delta!=delta)"+") :: "+(delta+" "+(s1.sweepCoef)+" "+ang1);
                #end
            };
            curTOI+=delta;
            if(curTOI>=1){
                curTOI=1;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !(curTOI*timeStep!=curTOI*timeStep);
                    };
                    if(!res)throw "assert("+"!(curTOI*timeStep!=curTOI*timeStep)"+") :: "+(curTOI+" "+timeStep);
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !(b1.sweepTime!=b1.sweepTime);
                    };
                    if(!res)throw "assert("+"!(b1.sweepTime!=b1.sweepTime)"+") :: "+("sweeeeep");
                    #end
                };
                b1.sweepIntegrate(curTOI*timeStep);
                b1.sweepValidate(s1);
                var sep=distance(s1,s2,c1,c2,axis)+negRadius;
                var dot=(deltax*axis.x+deltay*axis.y);
                if(sep<Config.distanceThresholdCCD){
                    var d1x:Float=0.0;
                    var d1y:Float=0.0;
                    {
                        d1x=c1.x-b1.posx;
                        d1y=c1.y-b1.posy;
                    };
                    var proj=dot-b1.sweep_angvel*(axis.y*d1x-axis.x*d1y);
                    if(proj>0){
                        toi.slipped=true;
                    }
                    if(proj<=0||sep<Config.distanceThresholdCCD*0.5){
                        break;
                    }
                }
                curTOI=-1;
                break;
            }
            if((++curIter)>=40){
                if(sep>negRadius){
                    toi.failed=true;
                }
                break;
            }
        }
        toi.toi=curTOI;
    }
    static public function distanceBody(b1:ZPP_Body,b2:ZPP_Body,w1:ZPP_Vec2,w2:ZPP_Vec2):Float{
        var t1;
        {
            if(ZPP_Vec2.zpp_pool==null){
                t1=new ZPP_Vec2();
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_TOT++;
                ZPP_Vec2.POOL_ADDNEW++;
                #end
            }
            else{
                t1=ZPP_Vec2.zpp_pool;
                ZPP_Vec2.zpp_pool=t1.next;
                t1.next=null;
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT--;
                ZPP_Vec2.POOL_ADD++;
                #end
            }
            t1.alloc();
        };
        var t2;
        {
            if(ZPP_Vec2.zpp_pool==null){
                t2=new ZPP_Vec2();
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_TOT++;
                ZPP_Vec2.POOL_ADDNEW++;
                #end
            }
            else{
                t2=ZPP_Vec2.zpp_pool;
                ZPP_Vec2.zpp_pool=t2.next;
                t2.next=null;
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT--;
                ZPP_Vec2.POOL_ADD++;
                #end
            }
            t2.alloc();
        };
        var ax;
        {
            if(ZPP_Vec2.zpp_pool==null){
                ax=new ZPP_Vec2();
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_TOT++;
                ZPP_Vec2.POOL_ADDNEW++;
                #end
            }
            else{
                ax=ZPP_Vec2.zpp_pool;
                ZPP_Vec2.zpp_pool=ax.next;
                ax.next=null;
                #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT--;
                ZPP_Vec2.POOL_ADD++;
                #end
            }
            ax.alloc();
        };
        var min=ZPP_Const.FMAX;
        {
            var cx_ite=b1.shapes.begin();
            while(cx_ite!=null){
                var s1=cx_ite.elem();
                {
                    {
                        var cx_ite=b2.shapes.begin();
                        while(cx_ite!=null){
                            var s2=cx_ite.elem();
                            {
                                var dist=distance(s1,s2,t1,t2,ax,min);
                                if(dist<min){
                                    min=dist;
                                    {
                                        w1.x=t1.x;
                                        w1.y=t1.y;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((w1.x!=w1.x));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(w1.x)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"t1.x"+",in y: "+"t1.y"+")");
                                            #end
                                        };
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((w1.y!=w1.y));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(w1.y)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"t1.x"+",in y: "+"t1.y"+")");
                                            #end
                                        };
                                    };
                                    {
                                        w2.x=t2.x;
                                        w2.y=t2.y;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((w2.x!=w2.x));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(w2.x)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"t2.x"+",in y: "+"t2.y"+")");
                                            #end
                                        };
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((w2.y!=w2.y));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(w2.y)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"t2.x"+",in y: "+"t2.y"+")");
                                            #end
                                        };
                                    };
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            var o=t1;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"t1"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        {
            var o=t2;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"t2"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        {
            var o=ax;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    o!=null;
                };
                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"ax"+")");
                #end
            };
            o.free();
            o.next=ZPP_Vec2.zpp_pool;
            ZPP_Vec2.zpp_pool=o;
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
            ZPP_Vec2.POOL_SUB++;
            #end
        };
        return min;
    }
    static public#if NAPE_NO_INLINE#else inline #end
    function distance(s1:ZPP_Shape,s2:ZPP_Shape,w1:ZPP_Vec2,w2:ZPP_Vec2,axis:ZPP_Vec2,upperBound:Float=1e100):Float{
        if(s1.isCircle()&&s2.isCircle()){
            var c1=s1.circle;
            var c2=s2.circle;
            var dist;
            {
                var nx:Float=0.0;
                var ny:Float=0.0;
                {
                    nx=c2.worldCOMx-c1.worldCOMx;
                    ny=c2.worldCOMy-c1.worldCOMy;
                };
                var len=ZPP_Math.sqrt((nx*nx+ny*ny));
                dist=len-(c1.radius+c2.radius);
                if(dist<upperBound){
                    if(len==0){
                        nx=1;
                        ny=0;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((nx!=nx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((ny!=ny));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                            #end
                        };
                    };
                    else{
                        var t=(1.0/(len));
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(len)"+")");
                            #end
                        };
                        nx*=t;
                        ny*=t;
                    };
                    {
                        var t=(c1.radius);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                            #end
                        };
                        w1.x=c1.worldCOMx+(nx*t);
                        w1.y=c1.worldCOMy+(ny*t);
                    };
                    {
                        var t=(-c2.radius);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                            #end
                        };
                        w2.x=c2.worldCOMx+(nx*t);
                        w2.y=c2.worldCOMy+(ny*t);
                    };
                    {
                        axis.x=nx;
                        axis.y=ny;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((axis.x!=axis.x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((axis.y!=axis.y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                            #end
                        };
                    };
                }
            };
            return dist;
        }
        else{
            var swapped=false;
            if(s1.isCircle()&&s2.isPolygon()){
                var tmp=s1;
                s1=s2;
                s2=tmp;
                var tmp2=w1;
                w1=w2;
                w2=tmp2;
                swapped=true;
            }
            if(s1.isPolygon()&&s2.isCircle()){
                var poly=s1.polygon;
                var circle=s2.circle;
                var best=-ZPP_Const.FMAX;
                var a0=null;
                {
                    var cx_ite=poly.edges.begin();
                    while(cx_ite!=null){
                        var a=cx_ite.elem();
                        {
                            var dist=(a.gnormx*circle.worldCOMx+a.gnormy*circle.worldCOMy)-a.gprojection-circle.radius;
                            if(dist>upperBound){
                                best=dist;
                                break;
                            }
                            if(dist>0){
                                if(dist>best){
                                    best=dist;
                                    a0=a;
                                }
                            }
                            else if(best<0&&dist>best){
                                best=dist;
                                a0=a;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(best<upperBound){
                    var v0=a0.gp0;
                    var v1=a0.gp1;
                    var dt=(circle.worldCOMy*a0.gnormx-circle.worldCOMx*a0.gnormy);
                    if(dt<=(v0.y*a0.gnormx-v0.x*a0.gnormy)){
                        {
                            var nx:Float=0.0;
                            var ny:Float=0.0;
                            {
                                nx=circle.worldCOMx-v0.x;
                                ny=circle.worldCOMy-v0.y;
                            };
                            var len=ZPP_Math.sqrt((nx*nx+ny*ny));
                            best=len-(0+circle.radius);
                            if(best<upperBound){
                                if(len==0){
                                    nx=1;
                                    ny=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((nx!=nx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((ny!=ny));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                                        #end
                                    };
                                };
                                else{
                                    var t=(1.0/(len));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(len)"+")");
                                        #end
                                    };
                                    nx*=t;
                                    ny*=t;
                                };
                                {
                                    var t=(0);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w1.x=v0.x+(nx*t);
                                    w1.y=v0.y+(ny*t);
                                };
                                {
                                    var t=(-circle.radius);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w2.x=circle.worldCOMx+(nx*t);
                                    w2.y=circle.worldCOMy+(ny*t);
                                };
                                {
                                    axis.x=nx;
                                    axis.y=ny;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((axis.x!=axis.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((axis.y!=axis.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                                        #end
                                    };
                                };
                            }
                        };
                    }
                    else if(dt>=(v1.y*a0.gnormx-v1.x*a0.gnormy)){
                        {
                            var nx:Float=0.0;
                            var ny:Float=0.0;
                            {
                                nx=circle.worldCOMx-v1.x;
                                ny=circle.worldCOMy-v1.y;
                            };
                            var len=ZPP_Math.sqrt((nx*nx+ny*ny));
                            best=len-(0+circle.radius);
                            if(best<upperBound){
                                if(len==0){
                                    nx=1;
                                    ny=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((nx!=nx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((ny!=ny));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"1"+",in y: "+"0"+")");
                                        #end
                                    };
                                };
                                else{
                                    var t=(1.0/(len));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"1.0/(len)"+")");
                                        #end
                                    };
                                    nx*=t;
                                    ny*=t;
                                };
                                {
                                    var t=(0);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w1.x=v1.x+(nx*t);
                                    w1.y=v1.y+(ny*t);
                                };
                                {
                                    var t=(-circle.radius);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w2.x=circle.worldCOMx+(nx*t);
                                    w2.y=circle.worldCOMy+(ny*t);
                                };
                                {
                                    axis.x=nx;
                                    axis.y=ny;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((axis.x!=axis.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((axis.y!=axis.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"nx"+",in y: "+"ny"+")");
                                        #end
                                    };
                                };
                            }
                        };
                    }
                    else{
                        {
                            var t=(-circle.radius);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                #end
                            };
                            w2.x=circle.worldCOMx+(a0.gnormx*t);
                            w2.y=circle.worldCOMy+(a0.gnormy*t);
                        };
                        {
                            var t=(-best);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                #end
                            };
                            w1.x=w2.x+(a0.gnormx*t);
                            w1.y=w2.y+(a0.gnormy*t);
                        };
                        {
                            axis.x=a0.gnormx;
                            axis.y=a0.gnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.x!=axis.x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"a0.gnormx"+",in y: "+"a0.gnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.y!=axis.y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"a0.gnormx"+",in y: "+"a0.gnormy"+")");
                                #end
                            };
                        };
                    }
                }
                if(swapped){
                    axis.x=-axis.x;
                    axis.y=-axis.y;
                };
                return best;
            }
            else{
                var p1=s1.polygon;
                var p2=s2.polygon;
                var best=-ZPP_Const.FMAX;
                var a1=null;
                var a2=null;
                var besti=0;
                {
                    var cx_ite=p1.edges.begin();
                    while(cx_ite!=null){
                        var a=cx_ite.elem();
                        {
                            var min=ZPP_Const.FMAX;
                            {
                                var cx_ite=p2.gverts.begin();
                                while(cx_ite!=null){
                                    var v=cx_ite.elem();
                                    {
                                        var k=(a.gnormx*v.x+a.gnormy*v.y);
                                        if(k<min)min=k;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            min-=a.gprojection;
                            if(min>upperBound){
                                best=min;
                                break;
                            }
                            if(min>0){
                                if(min>best){
                                    best=min;
                                    a1=a;
                                    besti=1;
                                }
                            }
                            else if(best<0&&min>best){
                                best=min;
                                a1=a;
                                besti=1;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(best<upperBound){
                    {
                        var cx_ite=p2.edges.begin();
                        while(cx_ite!=null){
                            var a=cx_ite.elem();
                            {
                                var min=ZPP_Const.FMAX;
                                {
                                    var cx_ite=p1.gverts.begin();
                                    while(cx_ite!=null){
                                        var v=cx_ite.elem();
                                        {
                                            var k=(a.gnormx*v.x+a.gnormy*v.y);
                                            if(k<min)min=k;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                min-=a.gprojection;
                                if(min>upperBound){
                                    best=min;
                                    break;
                                }
                                if(min>0){
                                    if(min>best){
                                        best=min;
                                        a2=a;
                                        besti=2;
                                    }
                                }
                                else if(best<0&&min>best){
                                    best=min;
                                    a2=a;
                                    besti=2;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    if(best<upperBound){
                        var q1,q2;
                        var ax;
                        if(besti==1){
                            q1=p1;
                            q2=p2;
                            ax=a1;
                        }
                        else{
                            q1=p2;
                            q2=p1;
                            ax=a2;
                            var tmp=w1;
                            w1=w2;
                            w2=tmp;
                            swapped=!swapped;
                        }
                        var ay:ZPP_Edge=null;
                        var min=ZPP_Const.FMAX;
                        {
                            var cx_ite=q2.edges.begin();
                            while(cx_ite!=null){
                                var a=cx_ite.elem();
                                {
                                    var k=(ax.gnormx*a.gnormx+ax.gnormy*a.gnormy);
                                    if(k<min){
                                        min=k;
                                        ay=a;
                                    }
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                        if(swapped){
                            axis.x=-ax.gnormx;
                            axis.y=-ax.gnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.x!=axis.x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"-ax.gnormx"+",in y: "+"-ax.gnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.y!=axis.y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"-ax.gnormx"+",in y: "+"-ax.gnormy"+")");
                                #end
                            };
                        };
                        else{
                            axis.x=ax.gnormx;
                            axis.y=ax.gnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.x!=axis.x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.x)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"ax.gnormx"+",in y: "+"ax.gnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((axis.y!=axis.y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(axis.y)"+") :: "+("vec_set(in n: "+"axis."+",in x: "+"ax.gnormx"+",in y: "+"ax.gnormy"+")");
                                #end
                            };
                        };
                        if(best>=0){
                            var v0=ax.gp0;
                            var v1=ax.gp1;
                            var q0=ay.gp0;
                            var q1=ay.gp1;
                            var vx:Float=0.0;
                            var vy:Float=0.0;
                            var qx:Float=0.0;
                            var qy:Float=0.0;
                            {
                                vx=v1.x-v0.x;
                                vy=v1.y-v0.y;
                            };
                            {
                                qx=q1.x-q0.x;
                                qy=q1.y-q0.y;
                            };
                            var vdot=1/(vx*vx+vy*vy);
                            var qdot=1/(qx*qx+qy*qy);
                            var t1=-(vx*(v0.x-q0.x)+vy*(v0.y-q0.y))*vdot;
                            var t2=-(vx*(v0.x-q1.x)+vy*(v0.y-q1.y))*vdot;
                            var s1=-(qx*(q0.x-v0.x)+qy*(q0.y-v0.y))*qdot;
                            var s2=-(qx*(q0.x-v1.x)+qy*(q0.y-v1.y))*qdot;
                            if(t1<0)t1=0;
                            else if(t1>1)t1=1;
                            if(t2<0)t2=0;
                            else if(t2>1)t2=1;
                            if(s1<0)s1=0;
                            else if(s1>1)s1=1;
                            if(s2<0)s2=0;
                            else if(s2>1)s2=1;
                            var f1x:Float=0.0;
                            var f1y:Float=0.0;
                            {
                                var t=(t1);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                    #end
                                };
                                f1x=v0.x+(vx*t);
                                f1y=v0.y+(vy*t);
                            };
                            var f2x:Float=0.0;
                            var f2y:Float=0.0;
                            {
                                var t=(t2);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                    #end
                                };
                                f2x=v0.x+(vx*t);
                                f2y=v0.y+(vy*t);
                            };
                            var g1x:Float=0.0;
                            var g1y:Float=0.0;
                            {
                                var t=(s1);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                    #end
                                };
                                g1x=q0.x+(qx*t);
                                g1y=q0.y+(qy*t);
                            };
                            var g2x:Float=0.0;
                            var g2y:Float=0.0;
                            {
                                var t=(s2);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                    #end
                                };
                                g2x=q0.x+(qx*t);
                                g2y=q0.y+(qy*t);
                            };
                            var d1=ZPP_VecMath.vec_dsq(f1x,f1y,q0.x,q0.y);
                            var d2=ZPP_VecMath.vec_dsq(f2x,f2y,q1.x,q1.y);
                            var e1=ZPP_VecMath.vec_dsq(g1x,g1y,v0.x,v0.y);
                            var e2=ZPP_VecMath.vec_dsq(g2x,g2y,v1.x,v1.y);
                            var minfx:Float=0.0;
                            var minfy:Float=0.0;
                            var minq=null;
                            if(d1<d2){
                                {
                                    minfx=f1x;
                                    minfy=f1y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((minfx!=minfx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(minfx)"+") :: "+("vec_set(in n: "+"minf"+",in x: "+"f1x"+",in y: "+"f1y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((minfy!=minfy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(minfy)"+") :: "+("vec_set(in n: "+"minf"+",in x: "+"f1x"+",in y: "+"f1y"+")");
                                        #end
                                    };
                                };
                                minq=q0;
                            }
                            else{
                                {
                                    minfx=f2x;
                                    minfy=f2y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((minfx!=minfx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(minfx)"+") :: "+("vec_set(in n: "+"minf"+",in x: "+"f2x"+",in y: "+"f2y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((minfy!=minfy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(minfy)"+") :: "+("vec_set(in n: "+"minf"+",in x: "+"f2x"+",in y: "+"f2y"+")");
                                        #end
                                    };
                                };
                                minq=q1;
                                d1=d2;
                            }
                            var mingx:Float=0.0;
                            var mingy:Float=0.0;
                            var minv=null;
                            if(e1<e2){
                                {
                                    mingx=g1x;
                                    mingy=g1y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((mingx!=mingx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(mingx)"+") :: "+("vec_set(in n: "+"ming"+",in x: "+"g1x"+",in y: "+"g1y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((mingy!=mingy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(mingy)"+") :: "+("vec_set(in n: "+"ming"+",in x: "+"g1x"+",in y: "+"g1y"+")");
                                        #end
                                    };
                                };
                                minv=v0;
                            }
                            else{
                                {
                                    mingx=g2x;
                                    mingy=g2y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((mingx!=mingx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(mingx)"+") :: "+("vec_set(in n: "+"ming"+",in x: "+"g2x"+",in y: "+"g2y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((mingy!=mingy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(mingy)"+") :: "+("vec_set(in n: "+"ming"+",in x: "+"g2x"+",in y: "+"g2y"+")");
                                        #end
                                    };
                                };
                                minv=v1;
                                e1=e2;
                            }
                            if(d1<e1){
                                {
                                    w1.x=minfx;
                                    w1.y=minfy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w1.x!=w1.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w1.x)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"minfx"+",in y: "+"minfy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w1.y!=w1.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w1.y)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"minfx"+",in y: "+"minfy"+")");
                                        #end
                                    };
                                };
                                {
                                    w2.x=minq.x;
                                    w2.y=minq.y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.x!=w2.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.x)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"minq.x"+",in y: "+"minq.y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.y!=w2.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.y)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"minq.x"+",in y: "+"minq.y"+")");
                                        #end
                                    };
                                };
                                best=Math.sqrt(d1);
                            }
                            else{
                                {
                                    w2.x=mingx;
                                    w2.y=mingy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.x!=w2.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.x)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"mingx"+",in y: "+"mingy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.y!=w2.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.y)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"mingx"+",in y: "+"mingy"+")");
                                        #end
                                    };
                                };
                                {
                                    w1.x=minv.x;
                                    w1.y=minv.y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w1.x!=w1.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w1.x)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"minv.x"+",in y: "+"minv.y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w1.y!=w1.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w1.y)"+") :: "+("vec_set(in n: "+"w1."+",in x: "+"minv.x"+",in y: "+"minv.y"+")");
                                        #end
                                    };
                                };
                                best=Math.sqrt(e1);
                            }
                            if(best!=0){
                                {
                                    axis.x=w2.x-w1.x;
                                    axis.y=w2.y-w1.y;
                                };
                                {
                                    var t=(1.0/(best));
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"axis."+",in s: "+"1.0/(best)"+")");
                                        #end
                                    };
                                    axis.x*=t;
                                    axis.y*=t;
                                };
                                if(swapped){
                                    axis.x=-axis.x;
                                    axis.y=-axis.y;
                                };
                            }
                            return best;
                        }
                        else{
                            var c0x:Float=0.0;
                            var c0y:Float=0.0;
                            {
                                c0x=ay.gp0.x;
                                c0y=ay.gp0.y;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((c0x!=c0x));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(c0x)"+") :: "+("vec_set(in n: "+"c0"+",in x: "+"ay.gp0.x"+",in y: "+"ay.gp0.y"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((c0y!=c0y));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(c0y)"+") :: "+("vec_set(in n: "+"c0"+",in x: "+"ay.gp0.x"+",in y: "+"ay.gp0.y"+")");
                                    #end
                                };
                            };
                            var c1x:Float=0.0;
                            var c1y:Float=0.0;
                            {
                                c1x=ay.gp1.x;
                                c1y=ay.gp1.y;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((c1x!=c1x));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(c1x)"+") :: "+("vec_set(in n: "+"c1"+",in x: "+"ay.gp1.x"+",in y: "+"ay.gp1.y"+")");
                                    #end
                                };
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((c1y!=c1y));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(c1y)"+") :: "+("vec_set(in n: "+"c1"+",in x: "+"ay.gp1.x"+",in y: "+"ay.gp1.y"+")");
                                    #end
                                };
                            };
                            var dvx:Float=0.0;
                            var dvy:Float=0.0;
                            {
                                dvx=c1x-c0x;
                                dvy=c1y-c0y;
                            };
                            var d0=(ax.gnormy*c0x-ax.gnormx*c0y);
                            var d1=(ax.gnormy*c1x-ax.gnormx*c1y);
                            var den=1/(d1-d0);
                            var t=(-ax.tp1-d0)*den;
                            if(t>Config.epsilon){
                                var t=(t);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c0"+",in b: "+"dv"+",in s: "+"t"+")");
                                    #end
                                };
                                c0x+=dvx*t;
                                c0y+=dvy*t;
                            };
                            var t=(-ax.tp0-d1)*den;
                            if(t<-Config.epsilon){
                                var t=(t);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c1"+",in b: "+"dv"+",in s: "+"t"+")");
                                    #end
                                };
                                c1x+=dvx*t;
                                c1y+=dvy*t;
                            };
                            var c0d=(c0x*ax.gnormx+c0y*ax.gnormy)-ax.gprojection;
                            var c1d=(c1x*ax.gnormx+c1y*ax.gnormy)-ax.gprojection;
                            if(c0d<c1d){
                                {
                                    w2.x=c0x;
                                    w2.y=c0y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.x!=w2.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.x)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"c0x"+",in y: "+"c0y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.y!=w2.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.y)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"c0x"+",in y: "+"c0y"+")");
                                        #end
                                    };
                                };
                                {
                                    var t=(-c0d);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w1.x=w2.x+(ax.gnormx*t);
                                    w1.y=w2.y+(ax.gnormy*t);
                                };
                                return c0d;
                            }
                            else{
                                {
                                    w2.x=c1x;
                                    w2.y=c1y;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.x!=w2.x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.x)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"c1x"+",in y: "+"c1y"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((w2.y!=w2.y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(w2.y)"+") :: "+("vec_set(in n: "+"w2."+",in x: "+"c1x"+",in y: "+"c1y"+")");
                                        #end
                                    };
                                };
                                {
                                    var t=(-c1d);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addmul()");
                                        #end
                                    };
                                    w1.x=w2.x+(ax.gnormx*t);
                                    w1.y=w2.y+(ax.gnormy*t);
                                };
                                return c1d;
                            }
                        }
                    }
                    else return upperBound;
                }
                else return upperBound;
            }
        }
    }
}

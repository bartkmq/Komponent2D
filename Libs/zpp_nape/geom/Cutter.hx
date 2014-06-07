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
class ZPP_CutVert{
    static public var zpp_pool:ZPP_CutVert=null;
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
    
    public var prev:ZPP_CutVert=null;
    public var next:ZPP_CutVert=null;
    public var posx:Float=0.0;
    public var posy:Float=0.0;
    public var vert:ZPP_GeomVert=null;
    public var value:Float=0.0;
    public var positive:Bool=false;
    public var parent:ZPP_CutVert=null;
    public var rank:Int=0;
    public var used:Bool=false;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        vert=null;
        parent=null;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function path(poly:ZPP_GeomVert){
        var ret;
        {
            if(ZPP_CutVert.zpp_pool==null){
                ret=new ZPP_CutVert();
                #if NAPE_POOL_STATS ZPP_CutVert.POOL_TOT++;
                ZPP_CutVert.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_CutVert.zpp_pool;
                ZPP_CutVert.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_CutVert.POOL_CNT--;
                ZPP_CutVert.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.vert=poly;
        ret.parent=ret;
        ret.rank=0;
        ret.used=false;
        return ret;
    }
}
#if nape_swc@:keep #end
class ZPP_CutInt{
    public var next:ZPP_CutInt=null;
    static public var zpp_pool:ZPP_CutInt=null;
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
    
    public var time:Float=0.0;
    public var virtualint:Bool=false;
    public var vertex:Bool=false;
    public var path0:ZPP_CutVert=null;
    public var end:ZPP_GeomVert=null;
    public var start:ZPP_GeomVert=null;
    public var path1:ZPP_CutVert=null;
    public function new(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function alloc(){}
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function free(){
        end=start=null;
        path0=path1=null;
    }
    public static#if NAPE_NO_INLINE#else inline #end
    function get(time:Float,end=null,start=null,path0=null,path1=null,virtualint=false,vertex=false){
        var ret;
        {
            if(ZPP_CutInt.zpp_pool==null){
                ret=new ZPP_CutInt();
                #if NAPE_POOL_STATS ZPP_CutInt.POOL_TOT++;
                ZPP_CutInt.POOL_ADDNEW++;
                #end
            }
            else{
                ret=ZPP_CutInt.zpp_pool;
                ZPP_CutInt.zpp_pool=ret.next;
                ret.next=null;
                #if NAPE_POOL_STATS ZPP_CutInt.POOL_CNT--;
                ZPP_CutInt.POOL_ADD++;
                #end
            }
            ret.alloc();
        };
        ret.virtualint=virtualint;
        ret.end=end;
        ret.start=start;
        ret.path0=path0;
        ret.path1=path1;
        ret.time=time;
        ret.vertex=vertex;
        return ret;
    }
}
#if nape_swc@:keep #end
class ZPP_Cutter{
    static var ints:ZNPList_ZPP_CutInt=null;
    static var paths:ZNPList_ZPP_CutVert=null;
    public static function run(P:ZPP_GeomVert,_start:Vec2,_end:Vec2,bstart:Bool,bend:Bool,output:GeomPolyList){
        var px:Float=0.0;
        var py:Float=0.0;
        {
            px=_start.x;
            py=_start.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((px!=px));
                };
                if(!res)throw "assert("+"!assert_isNaN(px)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"_start.x"+",in y: "+"_start.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((py!=py));
                };
                if(!res)throw "assert("+"!assert_isNaN(py)"+") :: "+("vec_set(in n: "+"p"+",in x: "+"_start.x"+",in y: "+"_start.y"+")");
                #end
            };
        };
        var dx:Float=0.0;
        var dy:Float=0.0;
        {
            dx=_end.x-px;
            dy=_end.y-py;
        };
        var min=bstart?0:ZPP_Const.NEGINF();
        var max=bend?1:ZPP_Const.POSINF();
        var crx=-(py*dx-px*dy);
        var verts:ZPP_CutVert=null;
        var clashes=false;
        var p=P;
        do{
            var c;
            {
                if(ZPP_CutVert.zpp_pool==null){
                    c=new ZPP_CutVert();
                    #if NAPE_POOL_STATS ZPP_CutVert.POOL_TOT++;
                    ZPP_CutVert.POOL_ADDNEW++;
                    #end
                }
                else{
                    c=ZPP_CutVert.zpp_pool;
                    ZPP_CutVert.zpp_pool=c.next;
                    c.next=null;
                    #if NAPE_POOL_STATS ZPP_CutVert.POOL_CNT--;
                    ZPP_CutVert.POOL_ADD++;
                    #end
                }
                c.alloc();
            };
            c.vert=p;
            {
                c.posx=c.vert.x;
                c.posy=c.vert.y;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c.posx!=c.posx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c.posx)"+") :: "+("vec_set(in n: "+"c.pos"+",in x: "+"c.vert.x"+",in y: "+"c.vert.y"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((c.posy!=c.posy));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(c.posy)"+") :: "+("vec_set(in n: "+"c.pos"+",in x: "+"c.vert.x"+",in y: "+"c.vert.y"+")");
                    #end
                };
            };
            c.value=(c.posy*dx-c.posx*dy)+crx;
            c.positive=c.value>0;
            if(c.value==0)clashes=true;
            verts={
                var obj=c;
                if(verts==null)verts=obj.prev=obj.next=obj;
                else{
                    obj.prev=verts;
                    obj.next=verts.next;
                    verts.next.prev=obj;
                    verts.next=obj;
                }
                obj;
            };
            p=p.next;
        }
        while(p!=P);
        if(clashes){
            var start:ZPP_CutVert=null;
            {
                var F=verts;
                var L=verts;
                if(F!=null){
                    var nite=F;
                    do{
                        var p=nite;
                        {
                            {
                                if(p.value!=0.0){
                                    start=p;
                                    break;
                                }
                            };
                        }
                        nite=nite.next;
                    }
                    while(nite!=L);
                }
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    start!=null;
                };
                if(!res)throw "assert("+"start!=null"+") :: "+("all vertices on line but it's simple?");
                #end
            };
            var nx:Float=0.0;
            var ny:Float=0.0;
            {
                nx=dx;
                ny=dy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((nx!=nx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"dx"+",in y: "+"dy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ny!=ny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"dx"+",in y: "+"dy"+")");
                    #end
                };
            };
            {
                {
                    var d=(nx*nx+ny*ny);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            d!=0.0;
                        };
                        if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"n"+")");
                        #end
                    };
                    var imag=ZPP_Math.invsqrt(d);
                    {
                        var t=(imag);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"n"+",in s: "+"imag"+")");
                            #end
                        };
                        nx*=t;
                        ny*=t;
                    };
                };
                {
                    var t=nx;
                    nx=-ny;
                    ny=t;
                };
            };
            var pre:ZPP_CutVert=null;
            var p=start;
            do{
                if(p.value!=0.0&&(pre==null||p==pre.next)){
                    pre=p;
                    p=p.next;
                    continue;
                }
                var prod=pre.value*p.value;
                if(prod==0){
                    p=p.next;
                    continue;
                }
                var a=pre.next;
                var positive=if(prod>0)pre.positive;
                else{
                    var b=a.next;
                    var midx:Float=0.0;
                    var midy:Float=0.0;
                    {
                        midx=a.posx+b.posx;
                        midy=a.posy+b.posy;
                    };
                    {
                        var t=(0.5);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"mid"+",in s: "+"0.5"+")");
                            #end
                        };
                        midx*=t;
                        midy*=t;
                    };
                    ({
                        var x=midx+nx*1e-8;
                        var y=midy+ny*1e-8;
                        var ret=false;
                        {
                            var F=P;
                            var L=P;
                            if(F!=null){
                                var nite=F;
                                do{
                                    var p=nite;
                                    {
                                        {
                                            var q=p.prev;
                                            if((p.y<y&&q.y>=y||q.y<y&&p.y>=y)&&(p.x<=x||q.x<=x)){
                                                if((p.x+(y-p.y)/(q.y-p.y)*(q.x-p.x))<x){
                                                    ret=!ret;
                                                }
                                            }
                                        };
                                    }
                                    nite=nite.next;
                                }
                                while(nite!=L);
                            }
                        };
                        ret;
                    });
                };
                {
                    var F=a;
                    var L=p;
                    if(F!=null){
                        var nite=F;
                        do{
                            var q=nite;
                            {
                                q.positive=positive;
                            }
                            nite=nite.next;
                        }
                        while(nite!=L);
                    }
                };
                pre=p;
                p=p.next;
            }
            while(p!=start);
            do{
                if(p.value!=0.0&&(pre==null||p==pre.next)){
                    pre=p;
                    p=p.next;
                    continue;
                }
                var prod=pre.value*p.value;
                if(prod==0){
                    p=p.next;
                    continue;
                }
                var a=pre.next;
                var positive=if(prod>0)pre.positive;
                else{
                    var b=a.next;
                    var midx:Float=0.0;
                    var midy:Float=0.0;
                    {
                        midx=a.posx+b.posx;
                        midy=a.posy+b.posy;
                    };
                    {
                        var t=(0.5);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"mid"+",in s: "+"0.5"+")");
                            #end
                        };
                        midx*=t;
                        midy*=t;
                    };
                    ({
                        var x=midx+nx*1e-8;
                        var y=midy+ny*1e-8;
                        var ret=false;
                        {
                            var F=P;
                            var L=P;
                            if(F!=null){
                                var nite=F;
                                do{
                                    var p=nite;
                                    {
                                        {
                                            var q=p.prev;
                                            if((p.y<y&&q.y>=y||q.y<y&&p.y>=y)&&(p.x<=x||q.x<=x)){
                                                if((p.x+(y-p.y)/(q.y-p.y)*(q.x-p.x))<x){
                                                    ret=!ret;
                                                }
                                            }
                                        };
                                    }
                                    nite=nite.next;
                                }
                                while(nite!=L);
                            }
                        };
                        ret;
                    });
                };
                {
                    var F=a;
                    var L=p;
                    if(F!=null){
                        var nite=F;
                        do{
                            var q=nite;
                            {
                                q.positive=positive;
                            }
                            nite=nite.next;
                        }
                        while(nite!=L);
                    }
                };
                pre=p;
                p=p.next;
            }
            while(false);
        }
        if(ints==null){
            ints=new ZNPList_ZPP_CutInt();
        }
        if(paths==null){
            paths=new ZNPList_ZPP_CutVert();
        }
        var start=null;
        {
            var obj=ZPP_GeomVert.get(verts.posx,verts.posy);
            if(start==null)start=obj.prev=obj.next=obj;
            else{
                obj.next=start;
                obj.prev=start.prev;
                start.prev.next=obj;
                start.prev=obj;
            }
            obj;
        };
        var origin=start;
        var firstpath=ZPP_CutVert.path(start);
        paths.add(firstpath);
        var i=verts;
        do{
            var j=i.next;
            var pj=ZPP_GeomVert.get(j.posx,j.posy);
            if(i.positive==j.positive){
                {
                    var obj=pj;
                    if(start==null)start=obj.prev=obj.next=obj;
                    else{
                        obj.next=start;
                        obj.prev=start.prev;
                        start.prev.next=obj;
                        start.prev=obj;
                    }
                    obj;
                };
            }
            else{
                var ux:Float=0.0;
                var uy:Float=0.0;
                {
                    ux=j.posx-i.posx;
                    uy=j.posy-i.posy;
                };
                var denom=(dy*ux-dx*uy);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        denom!=0;
                    };
                    if(!res)throw "assert("+"denom!=0"+") :: "+("0 denom in int. should have been handlded by clash resolution");
                    #end
                };
                denom=1/denom;
                var pax:Float=0.0;
                var pay:Float=0.0;
                {
                    pax=px-i.posx;
                    pay=py-i.posy;
                };
                var s=(uy*pax-ux*pay)*denom;
                if(s<min||s>max){
                    ints.add(ZPP_CutInt.get(s,true));
                    {
                        var obj=pj;
                        if(start==null)start=obj.prev=obj.next=obj;
                        else{
                            obj.next=start;
                            obj.prev=start.prev;
                            start.prev.next=obj;
                            start.prev=obj;
                        }
                        obj;
                    };
                }
                else{
                    if(i.value==0){
                        var endof=start.prev;
                        start=null;
                        {
                            var obj=ZPP_GeomVert.get(endof.x,endof.y);
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        {
                            var obj=pj;
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        var prepath=paths.begin().elem();
                        paths.add(ZPP_CutVert.path(start));
                        var postpath=paths.begin().elem();
                        ints.add(ZPP_CutInt.get(s,endof,start,prepath,postpath,true));
                    }
                    else if(j.value==0){
                        {
                            var obj=pj;
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        var endof=start.prev;
                        start=null;
                        {
                            var obj=ZPP_GeomVert.get(j.posx,j.posy);
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        var prepath=paths.begin().elem();
                        paths.add(ZPP_CutVert.path(start));
                        var postpath=paths.begin().elem();
                        ints.add(ZPP_CutInt.get(s,endof,start,prepath,postpath,true));
                    }
                    else{
                        var t=(dy*pax-dx*pay)*denom;
                        var qx:Float=0.0;
                        var qy:Float=0.0;
                        {
                            qx=i.posx;
                            qy=i.posy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((qx!=qx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(qx)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"i.posx"+",in y: "+"i.posy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((qy!=qy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(qy)"+") :: "+("vec_set(in n: "+"q"+",in x: "+"i.posx"+",in y: "+"i.posy"+")");
                                #end
                            };
                        };
                        {
                            var t=(t);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"q"+",in b: "+"u"+",in s: "+"t"+")");
                                #end
                            };
                            qx+=ux*t;
                            qy+=uy*t;
                        };
                        {
                            var obj=ZPP_GeomVert.get(qx,qy);
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        var endof=start.prev;
                        start=null;
                        {
                            var obj=ZPP_GeomVert.get(qx,qy);
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        {
                            var obj=pj;
                            if(start==null)start=obj.prev=obj.next=obj;
                            else{
                                obj.next=start;
                                obj.prev=start.prev;
                                start.prev.next=obj;
                                start.prev=obj;
                            }
                            obj;
                        };
                        var prepath=paths.begin().elem();
                        paths.add(ZPP_CutVert.path(start));
                        var postpath=paths.begin().elem();
                        ints.add(ZPP_CutInt.get(s,endof,start,prepath,postpath,false));
                    }
                }
            }
            i=i.next;
        }
        while(i!=verts);
        var endof=start.prev;
        {
            endof.next.prev=origin.prev;
            origin.prev.next=endof.next;
            endof.next=origin;
            origin.prev=endof;
        };
        var lastpath=paths.begin().elem();
        {
            var xr=({
                if(firstpath==firstpath.parent)firstpath;
                else{
                    var obj=firstpath;
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
                if(lastpath==lastpath.parent)lastpath;
                else{
                    var obj=lastpath;
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
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                (ints.size()%2)==0;
            };
            if(!res)throw "assert("+"(ints.size()%2)==0"+") :: "+("odd number of intersections?????");
            #end
        };
        {
            var xxlist=ints;
            if(!xxlist.empty()&&xxlist.begin().next!=null){
                var head:ZNPNode_ZPP_CutInt=xxlist.begin();
                var tail:ZNPNode_ZPP_CutInt=null;
                var left:ZNPNode_ZPP_CutInt=null;
                var right:ZNPNode_ZPP_CutInt=null;
                var nxt:ZNPNode_ZPP_CutInt=null;
                var listSize=1;
                var numMerges:Int,leftSize:Int,rightSize:Int;
                do{
                    numMerges=0;
                    left=head;
                    tail=head=null;
                    while(left!=null){
                        numMerges++;
                        right=left;
                        leftSize=0;
                        rightSize=listSize;
                        while(right!=null&&leftSize<listSize){
                            leftSize++;
                            right=right.next;
                        }
                        while(leftSize>0||(rightSize>0&&right!=null)){
                            if(leftSize==0){
                                nxt=right;
                                right=right.next;
                                rightSize--;
                            }
                            else if(rightSize==0||right==null){
                                nxt=left;
                                left=left.next;
                                leftSize--;
                            }
                            else if(left.elem().time<right.elem().time){
                                nxt=left;
                                left=left.next;
                                leftSize--;
                            }
                            else{
                                nxt=right;
                                right=right.next;
                                rightSize--;
                            }
                            if(tail!=null)tail.next=nxt;
                            else head=nxt;
                            tail=nxt;
                        }
                        left=right;
                    }
                    tail.next=null;
                    listSize<<=1;
                }
                while(numMerges>1);
                xxlist.setbegin(head);
            }
        };
        while(!ints.empty()){
            var i=ints.pop_unsafe();
            var j=ints.pop_unsafe();
            if(!i.virtualint&&!j.virtualint){
                {
                    i.end.next.prev=j.start.prev;
                    j.start.prev.next=i.end.next;
                    i.end.next=j.start;
                    j.start.prev=i.end;
                };
                {
                    j.end.next.prev=i.start.prev;
                    i.start.prev.next=j.end.next;
                    j.end.next=i.start;
                    i.start.prev=j.end;
                };
                {
                    var xr=({
                        if(i.path0==i.path0.parent)i.path0;
                        else{
                            var obj=i.path0;
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
                        if(j.path1==j.path1.parent)j.path1;
                        else{
                            var obj=j.path1;
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
                {
                    var xr=({
                        if(i.path1==i.path1.parent)i.path1;
                        else{
                            var obj=i.path1;
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
                        if(j.path0==j.path0.parent)j.path0;
                        else{
                            var obj=j.path0;
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
            else if(i.virtualint&&!j.virtualint){
                j.end={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(j.end==null);
                        };
                        if(!res)throw "assert("+"!(j.end==null)"+") :: "+("can't pop from empty list derpiderp");
                        #end
                    };
                    if((j.end!=null&&j.end.prev==j.end)){
                        j.end.next=j.end.prev=null;
                        {
                            var o=j.end;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"j.end"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_GeomVert.zpp_pool;
                            ZPP_GeomVert.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                            ZPP_GeomVert.POOL_SUB++;
                            #end
                        };
                        null;
                    }
                    else{
                        var retnodes=j.end.prev;
                        j.end.prev.next=j.end.next;
                        j.end.next.prev=j.end.prev;
                        j.end.next=j.end.prev=null;
                        {
                            var o=j.end;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"j.end"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_GeomVert.zpp_pool;
                            ZPP_GeomVert.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                            ZPP_GeomVert.POOL_SUB++;
                            #end
                        };
                        j.end=null;
                        retnodes;
                    }
                };
                if(!j.vertex){
                    if(j.end!=j.path0.vert){
                        j.start.x=j.end.x;
                        j.start.y=j.end.y;
                        j.end={
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !(j.end==null);
                                };
                                if(!res)throw "assert("+"!(j.end==null)"+") :: "+("can't pop from empty list derpiderp");
                                #end
                            };
                            if((j.end!=null&&j.end.prev==j.end)){
                                j.end.next=j.end.prev=null;
                                {
                                    var o=j.end;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"j.end"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                null;
                            }
                            else{
                                var retnodes=j.end.prev;
                                j.end.prev.next=j.end.next;
                                j.end.next.prev=j.end.prev;
                                j.end.next=j.end.prev=null;
                                {
                                    var o=j.end;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"j.end"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                j.end=null;
                                retnodes;
                            }
                        };
                    }
                    else{
                        var n=j.start.next;
                        j.start.x=n.x;
                        j.start.y=n.y;
                        {
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !(n==null);
                                };
                                if(!res)throw "assert("+"!(n==null)"+") :: "+("can't pop from empty list herpaderp");
                                #end
                            };
                            if((n!=null&&n.prev==n)){
                                n.next=n.prev=null;
                                {
                                    var o=n;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"n"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                n=null;
                            }
                            else{
                                n.prev.next=n.next;
                                n.next.prev=n.prev;
                                n.next=n.prev=null;
                                {
                                    var o=n;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"n"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                            }
                        };
                    }
                }
                {
                    j.end.next.prev=j.start.prev;
                    j.start.prev.next=j.end.next;
                    j.end.next=j.start;
                    j.start.prev=j.end;
                };
                {
                    var xr=({
                        if(j.path0==j.path0.parent)j.path0;
                        else{
                            var obj=j.path0;
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
                        if(j.path1==j.path1.parent)j.path1;
                        else{
                            var obj=j.path1;
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
            else if(j.virtualint&&!i.virtualint){
                i.end={
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !(i.end==null);
                        };
                        if(!res)throw "assert("+"!(i.end==null)"+") :: "+("can't pop from empty list derpiderp");
                        #end
                    };
                    if((i.end!=null&&i.end.prev==i.end)){
                        i.end.next=i.end.prev=null;
                        {
                            var o=i.end;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"i.end"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_GeomVert.zpp_pool;
                            ZPP_GeomVert.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                            ZPP_GeomVert.POOL_SUB++;
                            #end
                        };
                        null;
                    }
                    else{
                        var retnodes=i.end.prev;
                        i.end.prev.next=i.end.next;
                        i.end.next.prev=i.end.prev;
                        i.end.next=i.end.prev=null;
                        {
                            var o=i.end;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    o!=null;
                                };
                                if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"i.end"+")");
                                #end
                            };
                            o.free();
                            o.next=ZPP_GeomVert.zpp_pool;
                            ZPP_GeomVert.zpp_pool=o;
                            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                            ZPP_GeomVert.POOL_SUB++;
                            #end
                        };
                        i.end=null;
                        retnodes;
                    }
                };
                if(!i.vertex){
                    if(i.end!=i.path0.vert){
                        i.start.x=i.end.x;
                        i.start.y=i.end.y;
                        i.end={
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !(i.end==null);
                                };
                                if(!res)throw "assert("+"!(i.end==null)"+") :: "+("can't pop from empty list derpiderp");
                                #end
                            };
                            if((i.end!=null&&i.end.prev==i.end)){
                                i.end.next=i.end.prev=null;
                                {
                                    var o=i.end;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"i.end"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                null;
                            }
                            else{
                                var retnodes=i.end.prev;
                                i.end.prev.next=i.end.next;
                                i.end.next.prev=i.end.prev;
                                i.end.next=i.end.prev=null;
                                {
                                    var o=i.end;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"i.end"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                i.end=null;
                                retnodes;
                            }
                        };
                    }
                    else{
                        var n=i.start.next;
                        i.start.x=n.x;
                        i.start.y=n.y;
                        {
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !(n==null);
                                };
                                if(!res)throw "assert("+"!(n==null)"+") :: "+("can't pop from empty list herpaderp");
                                #end
                            };
                            if((n!=null&&n.prev==n)){
                                n.next=n.prev=null;
                                {
                                    var o=n;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"n"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                                n=null;
                            }
                            else{
                                n.prev.next=n.next;
                                n.next.prev=n.prev;
                                n.next=n.prev=null;
                                {
                                    var o=n;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            o!=null;
                                        };
                                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_GeomVert"+", in obj: "+"n"+")");
                                        #end
                                    };
                                    o.free();
                                    o.next=ZPP_GeomVert.zpp_pool;
                                    ZPP_GeomVert.zpp_pool=o;
                                    #if NAPE_POOL_STATS ZPP_GeomVert.POOL_CNT++;
                                    ZPP_GeomVert.POOL_SUB++;
                                    #end
                                };
                            }
                        };
                    }
                }
                {
                    i.end.next.prev=i.start.prev;
                    i.start.prev.next=i.end.next;
                    i.end.next=i.start;
                    i.start.prev=i.end;
                };
                {
                    var xr=({
                        if(i.path0==i.path0.parent)i.path0;
                        else{
                            var obj=i.path0;
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
                        if(i.path1==i.path1.parent)i.path1;
                        else{
                            var obj=i.path1;
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
            {
                var o=i;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CutInt"+", in obj: "+"i"+")");
                    #end
                };
                o.free();
                o.next=ZPP_CutInt.zpp_pool;
                ZPP_CutInt.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_CutInt.POOL_CNT++;
                ZPP_CutInt.POOL_SUB++;
                #end
            };
            {
                var o=j;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        o!=null;
                    };
                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CutInt"+", in obj: "+"j"+")");
                    #end
                };
                o.free();
                o.next=ZPP_CutInt.zpp_pool;
                ZPP_CutInt.zpp_pool=o;
                #if NAPE_POOL_STATS ZPP_CutInt.POOL_CNT++;
                ZPP_CutInt.POOL_SUB++;
                #end
            };
        }
        var ret=(output==null)?new GeomPolyList():output;
        {
            var cx_ite=paths.begin();
            while(cx_ite!=null){
                var p=cx_ite.elem();
                {
                    var poly=({
                        if(p==p.parent)p;
                        else{
                            var obj=p;
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
                    if(poly.used){
                        cx_ite=cx_ite.next;
                        continue;
                    };
                    poly.used=true;
                    ({
                        var p=poly.vert;
                        var skip=true;
                        while(poly.vert!=null&&(skip||p!=poly.vert)){
                            skip=false;
                            if(p.x==p.next.x&&p.y==p.next.y){
                                if(p==poly.vert){
                                    poly.vert=p.next==p?null:p.next;
                                    skip=true;
                                }
                                p={
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !(p==null);
                                        };
                                        if(!res)throw "assert("+"!(p==null)"+") :: "+("can't pop from empty list herpaderp");
                                        #end
                                    };
                                    if((p!=null&&p.prev==p)){
                                        p.next=p.prev=null;
                                        {};
                                        p=null;
                                    }
                                    else{
                                        var retnodes=p.next;
                                        p.prev.next=p.next;
                                        p.next.prev=p.prev;
                                        p.next=p.prev=null;
                                        {};
                                        p=null;
                                        retnodes;
                                    }
                                };
                            }
                            else p=p.next;
                        }
                    });
                    if(poly.vert!=null){
                        var gp=GeomPoly.get();
                        gp.zpp_inner.vertices=poly.vert;
                        ret.add(gp);
                    }
                };
                cx_ite=cx_ite.next;
            }
        };
        {
            while(!paths.empty()){
                var p=paths.pop_unsafe();
                {
                    var o=p;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            o!=null;
                        };
                        if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CutVert"+", in obj: "+"p"+")");
                        #end
                    };
                    o.free();
                    o.next=ZPP_CutVert.zpp_pool;
                    ZPP_CutVert.zpp_pool=o;
                    #if NAPE_POOL_STATS ZPP_CutVert.POOL_CNT++;
                    ZPP_CutVert.POOL_SUB++;
                    #end
                };
            }
        };
        {
            while(!(verts==null))verts={
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !(verts==null);
                    };
                    if(!res)throw "assert("+"!(verts==null)"+") :: "+("can't pop from empty list herpaderp");
                    #end
                };
                if((verts!=null&&verts.prev==verts)){
                    verts.next=verts.prev=null;
                    {
                        var o=verts;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CutVert"+", in obj: "+"verts"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_CutVert.zpp_pool;
                        ZPP_CutVert.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_CutVert.POOL_CNT++;
                        ZPP_CutVert.POOL_SUB++;
                        #end
                    };
                    verts=null;
                }
                else{
                    var retnodes=verts.next;
                    verts.prev.next=verts.next;
                    verts.next.prev=verts.prev;
                    verts.next=verts.prev=null;
                    {
                        var o=verts;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                o!=null;
                            };
                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_CutVert"+", in obj: "+"verts"+")");
                            #end
                        };
                        o.free();
                        o.next=ZPP_CutVert.zpp_pool;
                        ZPP_CutVert.zpp_pool=o;
                        #if NAPE_POOL_STATS ZPP_CutVert.POOL_CNT++;
                        ZPP_CutVert.POOL_SUB++;
                        #end
                    };
                    verts=null;
                    retnodes;
                }
            }
        };
        return ret;
    }
}

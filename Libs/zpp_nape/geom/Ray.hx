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
class ZPP_Ray{
    #if(!NAPE_RELEASE_BUILD)
    public static var internal:Bool=false;
    #end
    public var origin:Vec2=null;
    public var direction:Vec2=null;
    public var maxdist:Float=0.0;
    public var userData:Dynamic<Dynamic>=null;
    public var originx:Float=0.0;
    public var originy:Float=0.0;
    public var dirx:Float=0.0;
    public var diry:Float=0.0;
    public var idirx:Float=0.0;
    public var idiry:Float=0.0;
    public var normalx:Float=0.0;
    public var normaly:Float=0.0;
    public var absnormalx:Float=0.0;
    public var absnormaly:Float=0.0;
    private function origin_invalidate(x:ZPP_Vec2):Void{
        {
            originx=x.x;
            originy=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((originx!=originx));
                };
                if(!res)throw "assert("+"!assert_isNaN(originx)"+") :: "+("vec_set(in n: "+"origin"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((originy!=originy));
                };
                if(!res)throw "assert("+"!assert_isNaN(originy)"+") :: "+("vec_set(in n: "+"origin"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
    }
    private function direction_invalidate(x:ZPP_Vec2):Void{
        {
            dirx=x.x;
            diry=x.y;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dirx!=dirx));
                };
                if(!res)throw "assert("+"!assert_isNaN(dirx)"+") :: "+("vec_set(in n: "+"dir"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((diry!=diry));
                };
                if(!res)throw "assert("+"!assert_isNaN(diry)"+") :: "+("vec_set(in n: "+"dir"+",in x: "+"x.x"+",in y: "+"x.y"+")");
                #end
            };
        };
        invalidate_dir();
    }
    public function new(){
        origin=Vec2.get();
        origin.zpp_inner._invalidate=origin_invalidate;
        direction=Vec2.get();
        direction.zpp_inner._invalidate=direction_invalidate;
        {
            originx=0;
            originy=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((originx!=originx));
                };
                if(!res)throw "assert("+"!assert_isNaN(originx)"+") :: "+("vec_set(in n: "+"origin"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((originy!=originy));
                };
                if(!res)throw "assert("+"!assert_isNaN(originy)"+") :: "+("vec_set(in n: "+"origin"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        {
            dirx=0;
            diry=0;
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((dirx!=dirx));
                };
                if(!res)throw "assert("+"!assert_isNaN(dirx)"+") :: "+("vec_set(in n: "+"dir"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
            {
                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                var res={
                    !((diry!=diry));
                };
                if(!res)throw "assert("+"!assert_isNaN(diry)"+") :: "+("vec_set(in n: "+"dir"+",in x: "+"0"+",in y: "+"0"+")");
                #end
            };
        };
        zip_dir=false;
    }
    public var zip_dir:Bool=false;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function invalidate_dir():Void{
        zip_dir=true;
    }
    public function validate_dir():Void{
        if(zip_dir){
            zip_dir=false;
            #if(!NAPE_RELEASE_BUILD)
            if((dirx*dirx+diry*diry)<Config.epsilon){
                throw "Error: Ray::direction is degenerate";
            }
            #end
            {
                var d=(dirx*dirx+diry*diry);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        d!=0.0;
                    };
                    if(!res)throw "assert("+"d!=0.0"+") :: "+("vec_normalise(in n: "+"dir"+")");
                    #end
                };
                var imag=1.0/Math.sqrt(d);
                {
                    var t=(imag);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"dir"+",in s: "+"imag"+")");
                        #end
                    };
                    dirx*=t;
                    diry*=t;
                };
            };
            {
                idirx=1/dirx;
                idiry=1/diry;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((idirx!=idirx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(idirx)"+") :: "+("vec_set(in n: "+"idir"+",in x: "+"1/dirx"+",in y: "+"1/diry"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((idiry!=idiry));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(idiry)"+") :: "+("vec_set(in n: "+"idir"+",in x: "+"1/dirx"+",in y: "+"1/diry"+")");
                    #end
                };
            };
            {
                normalx=-diry;
                normaly=dirx;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((normalx!=normalx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(normalx)"+") :: "+("vec_set(in n: "+"normal"+",in x: "+"-diry"+",in y: "+"dirx"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((normaly!=normaly));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(normaly)"+") :: "+("vec_set(in n: "+"normal"+",in x: "+"-diry"+",in y: "+"dirx"+")");
                    #end
                };
            };
            {
                absnormalx=({
                    var x=normalx;
                    x<0?-x:x;
                });
                absnormaly=({
                    var x=normaly;
                    x<0?-x:x;
                });
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((absnormalx!=absnormalx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(absnormalx)"+") :: "+("vec_set(in n: "+"absnormal"+",in x: "+"({var x=normalx;x<0?-x:x;})"+",in y: "+"({var x=normaly;x<0?-x:x;})"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((absnormaly!=absnormaly));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(absnormaly)"+") :: "+("vec_set(in n: "+"absnormal"+",in x: "+"({var x=normalx;x<0?-x:x;})"+",in y: "+"({var x=normaly;x<0?-x:x;})"+")");
                    #end
                };
            };
        }
    }
    public function rayAABB(){
        var x0=originx;
        var x1=x0;
        var y0=originy;
        var y1=y0;
        if(maxdist>=ZPP_Const.POSINF()){
            if(dirx>0){
                x1=ZPP_Const.POSINF();
            }
            else if(dirx<0){
                x1=ZPP_Const.NEGINF();
            }
            if(diry>0){
                y1=ZPP_Const.POSINF();
            }
            else if(diry<0){
                y1=ZPP_Const.NEGINF();
            }
        }
        else{
            x1+=maxdist*dirx;
            y1+=maxdist*diry;
        }
        if(x1<x0){
            var t=x0;
            x0=x1;
            x1=t;
        };
        if(y1<y0){
            var t=y0;
            y0=y1;
            y1=t;
        };
        var rayab=ZPP_AABB.get(x0,y0,x1,y1);
        return rayab;
    }
    public function aabbtest(a:ZPP_AABB):Bool{
        var dot1=normalx*(originx-0.5*(a.minx+a.maxx))+normaly*(originy-0.5*(a.miny+a.maxy));
        var dot2=absnormalx*0.5*(a.maxx-a.minx)+absnormaly*0.5*(a.maxy-a.miny);
        return({
            var x=dot1;
            x<0?-x:x;
        })<dot2;
    }
    public function aabbsect(a:ZPP_AABB):Float{
        var cx=originx>=a.minx&&originx<=a.maxx;
        var cy=originy>=a.miny&&originy<=a.maxy;
        if(cx&&cy)return 0.0;
        else{
            var ret=-1.0;
            do{
                if(dirx>=0&&originx>=a.maxx)break;
                if(dirx<=0&&originx<=a.minx)break;
                if(diry>=0&&originy>=a.maxy)break;
                if(diry<=0&&originy<=a.miny)break;
                if(dirx>0){
                    var t=(a.minx-originx)*idirx;
                    if(t>=0&&t<=maxdist){
                        var y=originy+t*diry;
                        if(y>=a.miny&&y<=a.maxy){
                            ret=t;
                            break;
                        }
                    }
                }
                else if(dirx<0){
                    var t=(a.maxx-originx)*idirx;
                    if(t>=0&&t<=maxdist){
                        var y=originy+t*diry;
                        if(y>=a.miny&&y<=a.maxy){
                            ret=t;
                            break;
                        }
                    }
                }
                if(diry>0){
                    var t=(a.miny-originy)*idiry;
                    if(t>=0&&t<=maxdist){
                        var x=originx+t*dirx;
                        if(x>=a.minx&&x<=a.maxx){
                            ret=t;
                            break;
                        }
                    }
                }
                else if(diry<0){
                    var t=(a.maxy-originy)*idiry;
                    if(t>=0&&t<=maxdist){
                        var x=originx+t*dirx;
                        if(x>=a.minx&&x<=a.maxx){
                            ret=t;
                            break;
                        }
                    }
                }
            }
            while(false);
            return ret;
        }
    }
    public function circlesect(c:ZPP_Circle,inner:Bool,mint:Float):Null<RayResult>{
        c.validate_worldCOM();
        var acx:Float=0.0;
        var acy:Float=0.0;
        {
            acx=originx-c.worldCOMx;
            acy=originy-c.worldCOMy;
        };
        var A=(dirx*dirx+diry*diry);
        var B=2*(acx*dirx+acy*diry);
        var C=(acx*acx+acy*acy)-c.radius*c.radius;
        var det=B*B-4*A*C;
        if(det==0){
            var t=-B/2*A;
            if((!inner||C>0)&&t>0&&t<mint&&t<=maxdist){
                var nx:Float=0.0;
                var ny:Float=0.0;
                {
                    nx=originx;
                    ny=originy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
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
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t"+")");
                        #end
                    };
                    nx+=dirx*t;
                    ny+=diry*t;
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                        #end
                    };
                    nx-=c.worldCOMx*t;
                    ny-=c.worldCOMy*t;
                };
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
                    var imag=1.0/Math.sqrt(d);
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
                if(C<=0){
                    nx=-nx;
                    ny=-ny;
                };
                return ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t,C<=0,c.outer);
            };
            else return null;
        }
        else{
            det=Math.sqrt(det);
            A=1/(2*A);
            var t0=(-B-det)*A;
            var t1=(-B+det)*A;
            if(t0>0){
                if(t0<mint&&t0<=maxdist){
                    var nx:Float=0.0;
                    var ny:Float=0.0;
                    {
                        nx=originx;
                        ny=originy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((nx!=nx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((ny!=ny));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                            #end
                        };
                    };
                    {
                        var t=(t0);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t0"+")");
                            #end
                        };
                        nx+=dirx*t;
                        ny+=diry*t;
                    };
                    {
                        var t=(1.0);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                            #end
                        };
                        nx-=c.worldCOMx*t;
                        ny-=c.worldCOMy*t;
                    };
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
                        var imag=1.0/Math.sqrt(d);
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
                    if(false){
                        nx=-nx;
                        ny=-ny;
                    };
                    return ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t0,false,c.outer);
                };
                else return null;
            }
            else if(t1>0&&inner){
                if(t1<mint&&t1<=maxdist){
                    var nx:Float=0.0;
                    var ny:Float=0.0;
                    {
                        nx=originx;
                        ny=originy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((nx!=nx));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((ny!=ny));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                            #end
                        };
                    };
                    {
                        var t=(t1);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t1"+")");
                            #end
                        };
                        nx+=dirx*t;
                        ny+=diry*t;
                    };
                    {
                        var t=(1.0);
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((t!=t));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                            #end
                        };
                        nx-=c.worldCOMx*t;
                        ny-=c.worldCOMy*t;
                    };
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
                        var imag=1.0/Math.sqrt(d);
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
                    if(true){
                        nx=-nx;
                        ny=-ny;
                    };
                    return ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t1,true,c.outer);
                };
                else return null;
            }
            else return null;
        }
    }
    public function circlesect2(c:ZPP_Circle,inner:Bool,list:RayResultList):Void{
        c.validate_worldCOM();
        var acx:Float=0.0;
        var acy:Float=0.0;
        {
            acx=originx-c.worldCOMx;
            acy=originy-c.worldCOMy;
        };
        var A=(dirx*dirx+diry*diry);
        var B=2*(acx*dirx+acy*diry);
        var C=(acx*acx+acy*acy)-c.radius*c.radius;
        var det=B*B-4*A*C;
        if(det==0){
            var t=-B/2*A;
            if((!inner||C>0)&&t>0&&t<=maxdist){
                var nx:Float=0.0;
                var ny:Float=0.0;
                {
                    nx=originx;
                    ny=originy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
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
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t"+")");
                        #end
                    };
                    nx+=dirx*t;
                    ny+=diry*t;
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                        #end
                    };
                    nx-=c.worldCOMx*t;
                    ny-=c.worldCOMy*t;
                };
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
                    var imag=1.0/Math.sqrt(d);
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
                if(C<=0){
                    nx=-nx;
                    ny=-ny;
                };
                var res=ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t,C<=0,c.outer);
                {
                    var pre=null;
                    {
                        var cx_ite=list.zpp_inner.inner.begin();
                        while(cx_ite!=null){
                            var j=cx_ite.elem();
                            {
                                if((res.distance<j.distance))break;
                                pre=cx_ite;
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    list.zpp_inner.inner.inlined_insert(pre,res);
                };
            };
        }
        else{
            det=Math.sqrt(det);
            A=1/(2*A);
            var t0=(-B-det)*A;
            var t1=(-B+det)*A;
            if(t0>0&&t0<=maxdist){
                var nx:Float=0.0;
                var ny:Float=0.0;
                {
                    nx=originx;
                    ny=originy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                };
                {
                    var t=(t0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t0"+")");
                        #end
                    };
                    nx+=dirx*t;
                    ny+=diry*t;
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                        #end
                    };
                    nx-=c.worldCOMx*t;
                    ny-=c.worldCOMy*t;
                };
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
                    var imag=1.0/Math.sqrt(d);
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
                if(false){
                    nx=-nx;
                    ny=-ny;
                };
                var res=ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t0,false,c.outer);
                {
                    var pre=null;
                    {
                        var cx_ite=list.zpp_inner.inner.begin();
                        while(cx_ite!=null){
                            var j=cx_ite.elem();
                            {
                                if((res.distance<j.distance))break;
                                pre=cx_ite;
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    list.zpp_inner.inner.inlined_insert(pre,res);
                };
            };
            if(t1>0&&t1<=maxdist&&inner){
                var nx:Float=0.0;
                var ny:Float=0.0;
                {
                    nx=originx;
                    ny=originy;
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((nx!=nx));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((ny!=ny));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"originx"+",in y: "+"originy"+")");
                        #end
                    };
                };
                {
                    var t=(t1);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"n"+",in b: "+"dir"+",in s: "+"t1"+")");
                        #end
                    };
                    nx+=dirx*t;
                    ny+=diry*t;
                };
                {
                    var t=(1.0);
                    {
                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                        var res={
                            !((t!=t));
                        };
                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"n"+",in b: "+"c.worldCOM"+",in s: "+"1.0"+")");
                        #end
                    };
                    nx-=c.worldCOMx*t;
                    ny-=c.worldCOMy*t;
                };
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
                    var imag=1.0/Math.sqrt(d);
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
                if(true){
                    nx=-nx;
                    ny=-ny;
                };
                var res=ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),t1,true,c.outer);
                {
                    var pre=null;
                    {
                        var cx_ite=list.zpp_inner.inner.begin();
                        while(cx_ite!=null){
                            var j=cx_ite.elem();
                            {
                                if((res.distance<j.distance))break;
                                pre=cx_ite;
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    list.zpp_inner.inner.inlined_insert(pre,res);
                };
            };
        }
    }
    public function polysect(p:ZPP_Polygon,inner:Bool,mint:Float):Null<RayResult>{
        var min=mint;
        var edge:ZPP_Edge=null;
        var ei=p.edges.begin();
        {
            var cx_cont=true;
            var cx_itei=p.gverts.begin();
            var u=cx_itei.elem();
            var cx_itej=cx_itei.next;
            while(cx_itej!=null){
                var v=cx_itej.elem();
                {
                    var e=ei.elem();
                    if(inner||(e.gnormx*dirx+e.gnormy*diry)<0){
                        var _vx:Float=0.0;
                        var _vy:Float=0.0;
                        {
                            _vx=v.x-u.x;
                            _vy=v.y-u.y;
                        };
                        var _sx:Float=0.0;
                        var _sy:Float=0.0;
                        {
                            _sx=u.x-originx;
                            _sy=u.y-originy;
                        };
                        var den=(_vy*dirx-_vx*diry);
                        if(den*den>Config.epsilon){
                            den=1/den;
                            var sxx=(_vy*_sx-_vx*_sy)*den;
                            if(sxx>0&&sxx<min&&sxx<=maxdist){
                                var txx=(diry*_sx-dirx*_sy)*den;
                                if(txx>-Config.epsilon&&txx<1+Config.epsilon){
                                    min=sxx;
                                    edge=ei.elem();
                                }
                            }
                        }
                    }
                    ei=ei.next;
                };
                {
                    cx_itei=cx_itej;
                    u=v;
                    cx_itej=cx_itej.next;
                };
            }
            if(cx_cont){
                do{
                    cx_itej=p.gverts.begin();
                    var v=cx_itej.elem();
                    {
                        var e=ei.elem();
                        if(inner||(e.gnormx*dirx+e.gnormy*diry)<0){
                            var _vx:Float=0.0;
                            var _vy:Float=0.0;
                            {
                                _vx=v.x-u.x;
                                _vy=v.y-u.y;
                            };
                            var _sx:Float=0.0;
                            var _sy:Float=0.0;
                            {
                                _sx=u.x-originx;
                                _sy=u.y-originy;
                            };
                            var den=(_vy*dirx-_vx*diry);
                            if(den*den>Config.epsilon){
                                den=1/den;
                                var sxx=(_vy*_sx-_vx*_sy)*den;
                                if(sxx>0&&sxx<min&&sxx<=maxdist){
                                    var txx=(diry*_sx-dirx*_sy)*den;
                                    if(txx>-Config.epsilon&&txx<1+Config.epsilon){
                                        min=sxx;
                                        edge=ei.elem();
                                    }
                                }
                            }
                        }
                        ei=ei.next;
                    };
                }
                while(false);
            }
        }
        if(edge!=null){
            var nx:Float=0.0;
            var ny:Float=0.0;
            {
                nx=edge.gnormx;
                ny=edge.gnormy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((nx!=nx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edge.gnormx"+",in y: "+"edge.gnormy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ny!=ny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edge.gnormx"+",in y: "+"edge.gnormy"+")");
                    #end
                };
            };
            var inner=(nx*dirx+ny*diry)>0;
            if(inner){
                nx=-nx;
                ny=-ny;
            };
            return ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),min,inner,p.outer);
        }
        else return null;
    }
    public function polysect2(p:ZPP_Polygon,inner:Bool,list:RayResultList):Void{
        var min=ZPP_Const.POSINF();
        var max=-1.0;
        var edge:ZPP_Edge=null;
        var edgemax:ZPP_Edge=null;
        var ei=p.edges.begin();
        {
            var cx_cont=true;
            var cx_itei=p.gverts.begin();
            var u=cx_itei.elem();
            var cx_itej=cx_itei.next;
            while(cx_itej!=null){
                var v=cx_itej.elem();
                {
                    var e=ei.elem();
                    if(inner||(e.gnormx*dirx+e.gnormy*diry)<0){
                        var _vx:Float=0.0;
                        var _vy:Float=0.0;
                        {
                            _vx=v.x-u.x;
                            _vy=v.y-u.y;
                        };
                        var _sx:Float=0.0;
                        var _sy:Float=0.0;
                        {
                            _sx=u.x-originx;
                            _sy=u.y-originy;
                        };
                        var den=(_vy*dirx-_vx*diry);
                        if(den*den>Config.epsilon){
                            den=1/den;
                            var sxx=(_vy*_sx-_vx*_sy)*den;
                            if(sxx>0&&sxx<=maxdist&&(sxx<min||sxx>max)){
                                var txx=(diry*_sx-dirx*_sy)*den;
                                if(txx>-Config.epsilon&&txx<1+Config.epsilon){
                                    if(sxx<min){
                                        min=sxx;
                                        edge=ei.elem();
                                    }
                                    if(sxx>max){
                                        max=sxx;
                                        edgemax=ei.elem();
                                    }
                                }
                            }
                        }
                    }
                    ei=ei.next;
                };
                {
                    cx_itei=cx_itej;
                    u=v;
                    cx_itej=cx_itej.next;
                };
            }
            if(cx_cont){
                do{
                    cx_itej=p.gverts.begin();
                    var v=cx_itej.elem();
                    {
                        var e=ei.elem();
                        if(inner||(e.gnormx*dirx+e.gnormy*diry)<0){
                            var _vx:Float=0.0;
                            var _vy:Float=0.0;
                            {
                                _vx=v.x-u.x;
                                _vy=v.y-u.y;
                            };
                            var _sx:Float=0.0;
                            var _sy:Float=0.0;
                            {
                                _sx=u.x-originx;
                                _sy=u.y-originy;
                            };
                            var den=(_vy*dirx-_vx*diry);
                            if(den*den>Config.epsilon){
                                den=1/den;
                                var sxx=(_vy*_sx-_vx*_sy)*den;
                                if(sxx>0&&sxx<=maxdist&&(sxx<min||sxx>max)){
                                    var txx=(diry*_sx-dirx*_sy)*den;
                                    if(txx>-Config.epsilon&&txx<1+Config.epsilon){
                                        if(sxx<min){
                                            min=sxx;
                                            edge=ei.elem();
                                        }
                                        if(sxx>max){
                                            max=sxx;
                                            edgemax=ei.elem();
                                        }
                                    }
                                }
                            }
                        }
                        ei=ei.next;
                    };
                }
                while(false);
            }
        }
        if(edge!=null){
            var nx:Float=0.0;
            var ny:Float=0.0;
            {
                nx=edge.gnormx;
                ny=edge.gnormy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((nx!=nx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edge.gnormx"+",in y: "+"edge.gnormy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ny!=ny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edge.gnormx"+",in y: "+"edge.gnormy"+")");
                    #end
                };
            };
            var inner=(nx*dirx+ny*diry)>0;
            if(inner){
                nx=-nx;
                ny=-ny;
            };
            var ret=ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),min,inner,p.outer);
            {
                var pre=null;
                {
                    var cx_ite=list.zpp_inner.inner.begin();
                    while(cx_ite!=null){
                        var j=cx_ite.elem();
                        {
                            if((ret.distance<j.distance))break;
                            pre=cx_ite;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                list.zpp_inner.inner.inlined_insert(pre,ret);
            };
        };
        if(edgemax!=null&&edge!=edgemax){
            var nx:Float=0.0;
            var ny:Float=0.0;
            {
                nx=edgemax.gnormx;
                ny=edgemax.gnormy;
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((nx!=nx));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(nx)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edgemax.gnormx"+",in y: "+"edgemax.gnormy"+")");
                    #end
                };
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((ny!=ny));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(ny)"+") :: "+("vec_set(in n: "+"n"+",in x: "+"edgemax.gnormx"+",in y: "+"edgemax.gnormy"+")");
                    #end
                };
            };
            var inner=(nx*dirx+ny*diry)>0;
            if(inner){
                nx=-nx;
                ny=-ny;
            };
            var ret=ZPP_ConvexRayResult.getRay(Vec2.get(nx,ny),max,inner,p.outer);
            {
                var pre=null;
                {
                    var cx_ite=list.zpp_inner.inner.begin();
                    while(cx_ite!=null){
                        var j=cx_ite.elem();
                        {
                            if((ret.distance<j.distance))break;
                            pre=cx_ite;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                list.zpp_inner.inner.inlined_insert(pre,ret);
            };
        };
    }
}

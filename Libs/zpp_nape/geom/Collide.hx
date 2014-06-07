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
class ZPP_Collide{
     public static function circleContains(c:ZPP_Circle,p:ZPP_Vec2){
        var dx:Float=0.0;
        var dy:Float=0.0;
        {
            dx=p.x-c.worldCOMx;
            dy=p.y-c.worldCOMy;
        };
        return(dx*dx+dy*dy)<c.radius*c.radius;
    }
     public static function polyContains(s:ZPP_Polygon,p:ZPP_Vec2){
        var retvar;
        {
            retvar=true;
            {
                var cx_ite=s.edges.begin();
                while(cx_ite!=null){
                    var a=cx_ite.elem();
                    {
                        if((a.gnormx*p.x+a.gnormy*p.y)<=a.gprojection){
                            {
                                cx_ite=cx_ite.next;
                                continue;
                            };
                        }
                        else{
                            retvar=false;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return retvar;
    }
     public static function shapeContains(s:ZPP_Shape,p:ZPP_Vec2){
        return if(s.isCircle())circleContains(s.circle,p);
        else polyContains(s.polygon,p);
    }
     public static function bodyContains(b:ZPP_Body,p:ZPP_Vec2){
        var retvar;
        {
            retvar=false;
            {
                var cx_ite=b.shapes.begin();
                while(cx_ite!=null){
                    var s=cx_ite.elem();
                    {
                        if(shapeContains(s,p)){
                            retvar=true;
                            break;
                        }
                    };
                    cx_ite=cx_ite.next;
                }
            };
        };
        return retvar;
    }
     public static function containTest(s1:ZPP_Shape,s2:ZPP_Shape){
        if(s1.aabb.contains(s2.aabb)){
            return if(s1.isCircle()){
                if(s2.isCircle()){
                    var minDist=s1.circle.radius+(-s2.circle.radius);
                    var px:Float=0.0;
                    var py:Float=0.0;
                    {
                        px=s2.circle.worldCOMx-s1.circle.worldCOMx;
                        py=s2.circle.worldCOMy-s1.circle.worldCOMy;
                    };
                    var distSqr=(px*px+py*py);
                    distSqr<=minDist*minDist;
                };
                else({
                    var retvar;
                    {
                        retvar=true;
                        {
                            var cx_ite=s2.polygon.gverts.begin();
                            while(cx_ite!=null){
                                var p=cx_ite.elem();
                                {
                                    if({
                                        var minDist=s1.circle.radius+0;
                                        var px:Float=0.0;
                                        var py:Float=0.0;
                                        {
                                            px=p.x-s1.circle.worldCOMx;
                                            py=p.y-s1.circle.worldCOMy;
                                        };
                                        var distSqr=(px*px+py*py);
                                        distSqr<=minDist*minDist;
                                    }){
                                        {
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                    }
                                    else{
                                        retvar=false;
                                        break;
                                    }
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    retvar;
                });
            }
            else{
                if(s2.isCircle())({
                    var retvar;
                    {
                        retvar=true;
                        {
                            var cx_ite=s1.polygon.edges.begin();
                            while(cx_ite!=null){
                                var a=cx_ite.elem();
                                {
                                    if((a.gnormx*s2.circle.worldCOMx+a.gnormy*s2.circle.worldCOMy)+s2.circle.radius<=a.gprojection){
                                        {
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                    }
                                    else{
                                        retvar=false;
                                        break;
                                    }
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    retvar;
                });
                else({
                    var retvar;
                    {
                        retvar=true;
                        {
                            var cx_ite=s1.polygon.edges.begin();
                            while(cx_ite!=null){
                                var a=cx_ite.elem();
                                {
                                    if({
                                        var max=-ZPP_Const.FMAX;
                                        {
                                            var cx_ite=s2.polygon.gverts.begin();
                                            while(cx_ite!=null){
                                                var v=cx_ite.elem();
                                                {
                                                    var k=(a.gnormx*v.x+a.gnormy*v.y);
                                                    if(k>max)max=k;
                                                };
                                                cx_ite=cx_ite.next;
                                            }
                                        };
                                        max<=a.gprojection;
                                    }){
                                        {
                                            cx_ite=cx_ite.next;
                                            continue;
                                        };
                                    }
                                    else{
                                        retvar=false;
                                        break;
                                    }
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
                    };
                    retvar;
                });
            };
        }
        else return false;
    }
    public static function contactCollide(s1:ZPP_Shape,s2:ZPP_Shape,arb:ZPP_ColArbiter,rev:Bool){
        if(s2.isPolygon()){
            if(s1.isPolygon()){
                var cont=true;
                var max=-ZPP_Const.FMAX;
                var maxmin=-ZPP_Const.FMAX;
                var maxi=-1;
                var axis1:ZPP_Edge=null;
                var axis2:ZPP_Edge=null;
                {
                    var cx_ite=s1.polygon.edges.begin();
                    while(cx_ite!=null){
                        var ax=cx_ite.elem();
                        {
                            var min=ZPP_Const.FMAX;
                            {
                                var cx_ite=s2.polygon.gverts.begin();
                                while(cx_ite!=null){
                                    var v=cx_ite.elem();
                                    {
                                        var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                        if(k<min)min=k;
                                        if(min-ax.gprojection<=max)break;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            min-=ax.gprojection;
                            if(min>=0){
                                cont=false;
                                break;
                            }
                            if(min>max){
                                max=min;
                                axis1=ax;
                                maxi=1;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(cont){
                    {
                        var cx_ite=s2.polygon.edges.begin();
                        while(cx_ite!=null){
                            var ax=cx_ite.elem();
                            {
                                var min=ZPP_Const.FMAX;
                                {
                                    var cx_ite=s1.polygon.gverts.begin();
                                    while(cx_ite!=null){
                                        var v=cx_ite.elem();
                                        {
                                            var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                            if(k<min)min=k;
                                            if(min-ax.gprojection<=max)break;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                min-=ax.gprojection;
                                if(min>=0){
                                    cont=false;
                                    break;
                                }
                                if(min>max){
                                    max=min;
                                    axis2=ax;
                                    maxi=2;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    if(!cont)return false;
                    else{
                        var q1,q2,ax,scale;
                        if(maxi==1){
                            q1=s1.polygon;
                            q2=s2.polygon;
                            ax=axis1;
                            scale=1.0;
                        }
                        else{
                            q1=s2.polygon;
                            q2=s1.polygon;
                            ax=axis2;
                            scale=-1.0;
                        }
                        var ay:ZPP_Edge=null;
                        var min=ZPP_Const.FMAX;
                        {
                            var cx_ite=q2.edges.begin();
                            while(cx_ite!=null){
                                var axis=cx_ite.elem();
                                {
                                    var k=(ax.gnormx*axis.gnormx+ax.gnormy*axis.gnormy);
                                    if(k<min){
                                        min=k;
                                        ay=axis;
                                    }
                                };
                                cx_ite=cx_ite.next;
                            }
                        };
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
                        var nx:Float=0.0;
                        var ny:Float=0.0;
                        {
                            var t=(scale);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"ax.gnorm"+",in s: "+"scale"+",out r: "+"n"+")");
                                #end
                            };
                            nx=ax.gnormx*t;
                            ny=ax.gnormy*t;
                        };
                        {
                            arb.lnormx=ax.lnormx;
                            arb.lnormy=ax.lnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((arb.lnormx!=arb.lnormx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(arb.lnormx)"+") :: "+("vec_set(in n: "+"arb.lnorm"+",in x: "+"ax.lnormx"+",in y: "+"ax.lnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((arb.lnormy!=arb.lnormy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(arb.lnormy)"+") :: "+("vec_set(in n: "+"arb.lnorm"+",in x: "+"ax.lnormx"+",in y: "+"ax.lnormy"+")");
                                #end
                            };
                        };
                        arb.lproj=ax.lprojection;
                        arb.radius=0;
                        arb.rev=rev!=(scale==-1);
                        arb.ptype=arb.rev?ZPP_ColArbiter.FACE2:ZPP_ColArbiter.FACE1;
                        var c0d=(c0x*ax.gnormx+c0y*ax.gnormy)-ax.gprojection;
                        var c1d=(c1x*ax.gnormx+c1y*ax.gnormy)-ax.gprojection;
                        if(c0d>0&&c1d>0){
                            return false;
                        }
                        else{
                            if(rev){
                                nx=-nx;
                                ny=-ny;
                            }
                            var con=arb.injectContact(c0x-(ax.gnormx*c0d*0.5),c0y-(ax.gnormy*c0d*0.5),nx,ny,c0d,arb.rev?1:0,c0d>0);
                            {
                                var t=(1.0);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"c0"+",in b: "+"q2.body.pos"+",in s: "+"1.0"+")");
                                    #end
                                };
                                c0x-=q2.body.posx*t;
                                c0y-=q2.body.posy*t;
                            };
                            {
                                con.inner.lr1x=c0x*q2.body.axisy+c0y*q2.body.axisx;
                                con.inner.lr1y=c0y*q2.body.axisy-c0x*q2.body.axisx;
                            };
                            con=arb.injectContact(c1x-(ax.gnormx*c1d*0.5),c1y-(ax.gnormy*c1d*0.5),nx,ny,c1d,arb.rev?0:1,c1d>0);
                            {
                                var t=(1.0);
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        !((t!=t));
                                    };
                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"c1"+",in b: "+"q2.body.pos"+",in s: "+"1.0"+")");
                                    #end
                                };
                                c1x-=q2.body.posx*t;
                                c1y-=q2.body.posy*t;
                            };
                            {
                                con.inner.lr1x=c1x*q2.body.axisy+c1y*q2.body.axisx;
                                con.inner.lr1y=c1y*q2.body.axisy-c1x*q2.body.axisx;
                            };
                            if(maxi==1){
                                arb.__ref_edge1=ax;
                                arb.__ref_edge2=ay;
                            }
                            else{
                                arb.__ref_edge2=ax;
                                arb.__ref_edge1=ay;
                            }
                            return true;
                        }
                    }
                }
                else return false;
            };
            else{
                var max=-ZPP_Const.FMAX;
                var minmax=-ZPP_Const.FMAX;
                var cont=true;
                var a0=null,vi=null;
                var vite=s2.polygon.gverts.begin();
                {
                    var cx_ite=s2.polygon.edges.begin();
                    while(cx_ite!=null){
                        var a=cx_ite.elem();
                        {
                            var dist=(a.gnormx*s1.circle.worldCOMx+a.gnormy*s1.circle.worldCOMy)-a.gprojection-s1.circle.radius;
                            if(dist>0){
                                cont=false;
                                break;
                            }
                            if(dist>max){
                                max=dist;
                                a0=a;
                                vi=vite;
                            }
                            vite=vite.next;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(cont){
                    var v0=vi.elem();
                    var v1=if(vi.next==null)s2.polygon.gverts.front()else vi.next.elem();
                    var dt=(s1.circle.worldCOMy*a0.gnormx-s1.circle.worldCOMx*a0.gnormy);
                    if(dt<=(v0.y*a0.gnormx-v0.x*a0.gnormy)){
                        var co={
                            var minDist=s1.circle.radius+0;
                            var px:Float=0.0;
                            var py:Float=0.0;
                            {
                                px=v0.x-s1.circle.worldCOMx;
                                py=v0.y-s1.circle.worldCOMy;
                            };
                            var distSqr=(px*px+py*py);
                            if(distSqr>minDist*minDist)null;
                            else if(distSqr<Config.epsilon*Config.epsilon)arb.injectContact(s1.circle.worldCOMx,s1.circle.worldCOMy,1,0,-minDist,0);
                            else{
                                var invDist=ZPP_Math.invsqrt(distSqr);
                                var dist=if(invDist<Config.epsilon)ZPP_Const.FMAX else 1.0/invDist;
                                var df=0.5+(s1.circle.radius-0.5*minDist)*invDist;
                                if(rev)arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,-px*invDist,-py*invDist,dist-minDist,0);
                                else arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,px*invDist,py*invDist,dist-minDist,0);
                            }
                        };
                        if(co!=null){
                            var con=co.inner;
                            arb.ptype=ZPP_ColArbiter.CIRCLE;
                            var vx:Float=0.0;
                            var vy:Float=0.0;
                            {
                                vx=v0.x-s2.polygon.body.posx;
                                vy=v0.y-s2.polygon.body.posy;
                            };
                            arb.__ref_edge1=a0;
                            arb.__ref_vertex=-1;
                            if(rev){
                                {
                                    con.lr1x=vx*s2.polygon.body.axisy+vy*s2.polygon.body.axisx;
                                    con.lr1y=vy*s2.polygon.body.axisy-vx*s2.polygon.body.axisx;
                                };
                                {
                                    con.lr2x=s1.circle.localCOMx;
                                    con.lr2y=s1.circle.localCOMy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr2x!=con.lr2x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr2x)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr2y!=con.lr2y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr2y)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                };
                            }
                            else{
                                {
                                    con.lr2x=vx*s2.polygon.body.axisy+vy*s2.polygon.body.axisx;
                                    con.lr2y=vy*s2.polygon.body.axisy-vx*s2.polygon.body.axisx;
                                };
                                {
                                    con.lr1x=s1.circle.localCOMx;
                                    con.lr1y=s1.circle.localCOMy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr1x!=con.lr1x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr1x)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr1y!=con.lr1y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr1y)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                };
                            }
                            arb.radius=s1.circle.radius;
                        }
                        return co!=null;
                    }
                    else if(dt>=(v1.y*a0.gnormx-v1.x*a0.gnormy)){
                        var co={
                            var minDist=s1.circle.radius+0;
                            var px:Float=0.0;
                            var py:Float=0.0;
                            {
                                px=v1.x-s1.circle.worldCOMx;
                                py=v1.y-s1.circle.worldCOMy;
                            };
                            var distSqr=(px*px+py*py);
                            if(distSqr>minDist*minDist)null;
                            else if(distSqr<Config.epsilon*Config.epsilon)arb.injectContact(s1.circle.worldCOMx,s1.circle.worldCOMy,1,0,-minDist,0);
                            else{
                                var invDist=ZPP_Math.invsqrt(distSqr);
                                var dist=if(invDist<Config.epsilon)ZPP_Const.FMAX else 1.0/invDist;
                                var df=0.5+(s1.circle.radius-0.5*minDist)*invDist;
                                if(rev)arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,-px*invDist,-py*invDist,dist-minDist,0);
                                else arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,px*invDist,py*invDist,dist-minDist,0);
                            }
                        };
                        if(co!=null){
                            var con=co.inner;
                            arb.ptype=ZPP_ColArbiter.CIRCLE;
                            var vx:Float=0.0;
                            var vy:Float=0.0;
                            {
                                vx=v1.x-s2.polygon.body.posx;
                                vy=v1.y-s2.polygon.body.posy;
                            };
                            arb.__ref_edge1=a0;
                            arb.__ref_vertex=1;
                            if(rev){
                                {
                                    con.lr1x=vx*s2.polygon.body.axisy+vy*s2.polygon.body.axisx;
                                    con.lr1y=vy*s2.polygon.body.axisy-vx*s2.polygon.body.axisx;
                                };
                                {
                                    con.lr2x=s1.circle.localCOMx;
                                    con.lr2y=s1.circle.localCOMy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr2x!=con.lr2x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr2x)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr2y!=con.lr2y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr2y)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                };
                            }
                            else{
                                {
                                    con.lr2x=vx*s2.polygon.body.axisy+vy*s2.polygon.body.axisx;
                                    con.lr2y=vy*s2.polygon.body.axisy-vx*s2.polygon.body.axisx;
                                };
                                {
                                    con.lr1x=s1.circle.localCOMx;
                                    con.lr1y=s1.circle.localCOMy;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr1x!=con.lr1x));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr1x)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((con.lr1y!=con.lr1y));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(con.lr1y)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                        #end
                                    };
                                };
                            }
                            arb.radius=s1.circle.radius;
                        }
                        return co!=null;
                    }
                    else{
                        var nx:Float=0.0;
                        var ny:Float=0.0;
                        {
                            var t=(s1.circle.radius+max*0.5);
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((t!=t));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_mul(in a: "+"a0.gnorm"+",in s: "+"s1.circle.radius+max*0.5"+",out r: "+"n"+")");
                                #end
                            };
                            nx=a0.gnormx*t;
                            ny=a0.gnormy*t;
                        };
                        var px:Float=0.0;
                        var py:Float=0.0;
                        {
                            px=s1.circle.worldCOMx-nx;
                            py=s1.circle.worldCOMy-ny;
                        };
                        var con=if(rev)arb.injectContact(px,py,a0.gnormx,a0.gnormy,max,0);
                        else arb.injectContact(px,py,-a0.gnormx,-a0.gnormy,max,0);
                        arb.ptype=if(rev)ZPP_ColArbiter.FACE1 else ZPP_ColArbiter.FACE2;
                        {
                            arb.lnormx=a0.lnormx;
                            arb.lnormy=a0.lnormy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((arb.lnormx!=arb.lnormx));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(arb.lnormx)"+") :: "+("vec_set(in n: "+"arb.lnorm"+",in x: "+"a0.lnormx"+",in y: "+"a0.lnormy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((arb.lnormy!=arb.lnormy));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(arb.lnormy)"+") :: "+("vec_set(in n: "+"arb.lnorm"+",in x: "+"a0.lnormx"+",in y: "+"a0.lnormy"+")");
                                #end
                            };
                        };
                        arb.rev=!rev;
                        arb.lproj=a0.lprojection;
                        arb.radius=s1.circle.radius;
                        {
                            con.inner.lr1x=s1.circle.localCOMx;
                            con.inner.lr1y=s1.circle.localCOMy;
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((con.inner.lr1x!=con.inner.lr1x));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(con.inner.lr1x)"+") :: "+("vec_set(in n: "+"con.inner.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                #end
                            };
                            {
                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                var res={
                                    !((con.inner.lr1y!=con.inner.lr1y));
                                };
                                if(!res)throw "assert("+"!assert_isNaN(con.inner.lr1y)"+") :: "+("vec_set(in n: "+"con.inner.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                                #end
                            };
                        };
                        arb.__ref_edge1=a0;
                        arb.__ref_vertex=0;
                        return true;
                    }
                }
                else return false;
            };
        }
        else{
            var co={
                var minDist=s1.circle.radius+s2.circle.radius;
                var px:Float=0.0;
                var py:Float=0.0;
                {
                    px=s2.circle.worldCOMx-s1.circle.worldCOMx;
                    py=s2.circle.worldCOMy-s1.circle.worldCOMy;
                };
                var distSqr=(px*px+py*py);
                if(distSqr>minDist*minDist)null;
                else if(distSqr<Config.epsilon*Config.epsilon)arb.injectContact(s1.circle.worldCOMx,s1.circle.worldCOMy,1,0,-minDist,0);
                else{
                    var invDist=ZPP_Math.invsqrt(distSqr);
                    var dist=if(invDist<Config.epsilon)ZPP_Const.FMAX else 1.0/invDist;
                    var df=0.5+(s1.circle.radius-0.5*minDist)*invDist;
                    if(rev)arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,-px*invDist,-py*invDist,dist-minDist,0);
                    else arb.injectContact(s1.circle.worldCOMx+px*df,s1.circle.worldCOMy+py*df,px*invDist,py*invDist,dist-minDist,0);
                }
            };
            if(co!=null){
                var con=co.inner;
                if(rev){
                    {
                        con.lr1x=s2.circle.localCOMx;
                        con.lr1y=s2.circle.localCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr1x!=con.lr1x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr1x)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s2.circle.localCOMx"+",in y: "+"s2.circle.localCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr1y!=con.lr1y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr1y)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s2.circle.localCOMx"+",in y: "+"s2.circle.localCOMy"+")");
                            #end
                        };
                    };
                    {
                        con.lr2x=s1.circle.localCOMx;
                        con.lr2y=s1.circle.localCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr2x!=con.lr2x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr2x)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr2y!=con.lr2y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr2y)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                            #end
                        };
                    };
                }
                else{
                    {
                        con.lr1x=s1.circle.localCOMx;
                        con.lr1y=s1.circle.localCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr1x!=con.lr1x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr1x)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr1y!=con.lr1y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr1y)"+") :: "+("vec_set(in n: "+"con.lr1"+",in x: "+"s1.circle.localCOMx"+",in y: "+"s1.circle.localCOMy"+")");
                            #end
                        };
                    };
                    {
                        con.lr2x=s2.circle.localCOMx;
                        con.lr2y=s2.circle.localCOMy;
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr2x!=con.lr2x));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr2x)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s2.circle.localCOMx"+",in y: "+"s2.circle.localCOMy"+")");
                            #end
                        };
                        {
                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                            var res={
                                !((con.lr2y!=con.lr2y));
                            };
                            if(!res)throw "assert("+"!assert_isNaN(con.lr2y)"+") :: "+("vec_set(in n: "+"con.lr2"+",in x: "+"s2.circle.localCOMx"+",in y: "+"s2.circle.localCOMy"+")");
                            #end
                        };
                    };
                }
                arb.radius=s1.circle.radius+s2.circle.radius;
                arb.ptype=ZPP_ColArbiter.CIRCLE;
                return true;
            }
            else return false;
        };
    }
     public static function testCollide_safe(s1:ZPP_Shape,s2:ZPP_Shape){
        if(s2.isCircle()){
            var t=s1;
            s1=s2;
            s2=t;
        };
        return testCollide(s1,s2);
    }
     public static function testCollide(s1:ZPP_Shape,s2:ZPP_Shape){
        if(s2.isPolygon()){
            return if(s1.isPolygon()){
                var cont=true;
                {
                    var cx_ite=s1.polygon.edges.begin();
                    while(cx_ite!=null){
                        var ax=cx_ite.elem();
                        {
                            var min=ZPP_Const.FMAX;
                            {
                                var cx_ite=s2.polygon.gverts.begin();
                                while(cx_ite!=null){
                                    var v=cx_ite.elem();
                                    {
                                        var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                        if(k<min)min=k;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            min-=ax.gprojection;
                            if(min>0){
                                cont=false;
                                break;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(cont){
                    {
                        var cx_ite=s2.polygon.edges.begin();
                        while(cx_ite!=null){
                            var ax=cx_ite.elem();
                            {
                                var min=ZPP_Const.FMAX;
                                {
                                    var cx_ite=s1.polygon.gverts.begin();
                                    while(cx_ite!=null){
                                        var v=cx_ite.elem();
                                        {
                                            var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                            if(k<min)min=k;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                min-=ax.gprojection;
                                if(min>0){
                                    cont=false;
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    cont;
                }
                else false;
            };
            else{
                var a0=null,vi=null;
                var cont=true;
                var max=-ZPP_Const.FMAX;
                var vite=s2.polygon.gverts.begin();
                {
                    var cx_ite=s2.polygon.edges.begin();
                    while(cx_ite!=null){
                        var a=cx_ite.elem();
                        {
                            var dist=(a.gnormx*s1.circle.worldCOMx+a.gnormy*s1.circle.worldCOMy)-a.gprojection-s1.circle.radius;
                            if(dist>0){
                                cont=false;
                                break;
                            }
                            if(dist>max){
                                max=dist;
                                a0=a;
                                vi=vite;
                            }
                            vite=vite.next;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(cont){
                    var v0=vi.elem();
                    var v1=if(vi.next==null)s2.polygon.gverts.front()else vi.next.elem();
                    var dt=(s1.circle.worldCOMy*a0.gnormx-s1.circle.worldCOMx*a0.gnormy);
                    if(dt<=(v0.y*a0.gnormx-v0.x*a0.gnormy)){
                        var minDist=s1.circle.radius+0;
                        var px:Float=0.0;
                        var py:Float=0.0;
                        {
                            px=v0.x-s1.circle.worldCOMx;
                            py=v0.y-s1.circle.worldCOMy;
                        };
                        var distSqr=(px*px+py*py);
                        distSqr<=minDist*minDist;
                    };
                    else if(dt>=(v1.y*a0.gnormx-v1.x*a0.gnormy)){
                        var minDist=s1.circle.radius+0;
                        var px:Float=0.0;
                        var py:Float=0.0;
                        {
                            px=v1.x-s1.circle.worldCOMx;
                            py=v1.y-s1.circle.worldCOMy;
                        };
                        var distSqr=(px*px+py*py);
                        distSqr<=minDist*minDist;
                    };
                    else true;
                }
                else false;
            };
        }
        else return{
            var minDist=s1.circle.radius+s2.circle.radius;
            var px:Float=0.0;
            var py:Float=0.0;
            {
                px=s2.circle.worldCOMx-s1.circle.worldCOMx;
                py=s2.circle.worldCOMy-s1.circle.worldCOMy;
            };
            var distSqr=(px*px+py*py);
            distSqr<=minDist*minDist;
        };
    }
     public static function flowCollide(s1:ZPP_Shape,s2:ZPP_Shape,arb:ZPP_FluidArbiter){
        if(s2.isPolygon()){
            return if(s1.isPolygon()){
                var out1=new Array<Bool>();
                var out2=new Array<Bool>();
                var cont=true;
                var total=true;
                {
                    var cx_ite=s1.polygon.edges.begin();
                    while(cx_ite!=null){
                        var ax=cx_ite.elem();
                        {
                            var min=ZPP_Const.FMAX;
                            var ind=0;
                            {
                                var cx_ite=s2.polygon.gverts.begin();
                                while(cx_ite!=null){
                                    var v=cx_ite.elem();
                                    {
                                        var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                        if(k<min)min=k;
                                        if(k>=ax.gprojection+Config.epsilon){
                                            out2[ind]=true;
                                            total=false;
                                        }
                                        ind++;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            min-=ax.gprojection;
                            if(min>0){
                                cont=false;
                                break;
                            }
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(total){
                    s2.polygon.validate_worldCOM();
                    arb.inject(s2.polygon.area,s2.polygon.worldCOMx,s2.polygon.worldCOMy);
                    true;
                }
                else if(cont){
                    total=true;
                    {
                        var cx_ite=s2.polygon.edges.begin();
                        while(cx_ite!=null){
                            var ax=cx_ite.elem();
                            {
                                var min=ZPP_Const.FMAX;
                                var ind=0;
                                {
                                    var cx_ite=s1.polygon.gverts.begin();
                                    while(cx_ite!=null){
                                        var v=cx_ite.elem();
                                        {
                                            var k=(ax.gnormx*v.x+ax.gnormy*v.y);
                                            if(k<min)min=k;
                                            if(k>=ax.gprojection+Config.epsilon){
                                                out1[ind]=true;
                                                total=false;
                                            }
                                            ind++;
                                        };
                                        cx_ite=cx_ite.next;
                                    }
                                };
                                min-=ax.gprojection;
                                if(min>0){
                                    cont=false;
                                    break;
                                }
                            };
                            cx_ite=cx_ite.next;
                        }
                    };
                    if(total){
                        s1.polygon.validate_worldCOM();
                        arb.inject(s1.polygon.area,s1.polygon.worldCOMx,s1.polygon.worldCOMy);
                        true;
                    }
                    else if(cont){
                        while(!flowpoly.empty()){
                            var p=flowpoly.pop_unsafe();
                            if(!p._inuse){
                                var o=p;
                                {
                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                    var res={
                                        o!=null;
                                    };
                                    if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"p"+")");
                                    #end
                                };
                                o.free();
                                o.next=ZPP_Vec2.zpp_pool;
                                ZPP_Vec2.zpp_pool=o;
                                #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
                                ZPP_Vec2.POOL_SUB++;
                                #end
                            };
                        }
                        var fst_vert=null;
                        var poly1=false;
                        var ite1=s1.polygon.gverts.begin();
                        var ind1=0;
                        var ite2=s2.polygon.gverts.begin();
                        var ind2=0;
                        for(i in 0...s2.polygon.edgeCnt)if(!out2[i]){
                            ind2=i;
                            break;
                        }
                        else ite2=ite2.next;
                        if(ite2==null){
                            ite2=s2.polygon.gverts.begin();
                            poly1=true;
                            for(i in 0...s1.polygon.edgeCnt)if(!out1[i]){
                                ind1=i;
                                break;
                            }
                            else ite1=ite1.next;
                            if(ite1==null)ite1=s1.polygon.gverts.begin();
                            else{
                                flowpoly.add(ite1.elem());
                                fst_vert=flowpoly.front();
                            }
                        }
                        else{
                            flowpoly.add(ite2.elem());
                            fst_vert=flowpoly.front();
                        }
                        var cnt=1;
                        if(flowpoly.empty()){
                            {
                                var cx_cont=true;
                                var cx_itei=s1.polygon.gverts.begin();
                                var u=cx_itei.elem();
                                var cx_itej=cx_itei.next;
                                while(cx_itej!=null){
                                    var v=cx_itej.elem();
                                    {
                                        var min=2.0;
                                        {
                                            var cx_cont=true;
                                            var cx_itei=s2.polygon.gverts.begin();
                                            var a=cx_itei.elem();
                                            var cx_itej=cx_itei.next;
                                            while(cx_itej!=null){
                                                var b=cx_itej.elem();
                                                {
                                                    var t=0.0;
                                                    if({
                                                        var _sx:Float=0.0;
                                                        var _sy:Float=0.0;
                                                        {
                                                            _sx=u.x-a.x;
                                                            _sy=u.y-a.y;
                                                        };
                                                        var _vx:Float=0.0;
                                                        var _vy:Float=0.0;
                                                        {
                                                            _vx=v.x-u.x;
                                                            _vy=v.y-u.y;
                                                        };
                                                        var _qx:Float=0.0;
                                                        var _qy:Float=0.0;
                                                        {
                                                            _qx=b.x-a.x;
                                                            _qy=b.y-a.y;
                                                        };
                                                        var den=(_vy*_qx-_vx*_qy);
                                                        if(den*den>Config.epsilon*Config.epsilon){
                                                            den=1/den;
                                                            var txx=(_qy*_sx-_qx*_sy)*den;
                                                            if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                                var sxx=(_vy*_sx-_vx*_sy)*den;
                                                                if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                    t=txx;
                                                                    true;
                                                                }
                                                                else false;
                                                            }
                                                            else false;
                                                        }
                                                        else false;
                                                    }){
                                                        if(t<min){
                                                            min=t;
                                                            ite2=cx_itei;
                                                        }
                                                    }
                                                };
                                                {
                                                    cx_itei=cx_itej;
                                                    a=b;
                                                    cx_itej=cx_itej.next;
                                                };
                                            }
                                            if(cx_cont){
                                                do{
                                                    cx_itej=s2.polygon.gverts.begin();
                                                    var b=cx_itej.elem();
                                                    {
                                                        var t=0.0;
                                                        if({
                                                            var _sx:Float=0.0;
                                                            var _sy:Float=0.0;
                                                            {
                                                                _sx=u.x-a.x;
                                                                _sy=u.y-a.y;
                                                            };
                                                            var _vx:Float=0.0;
                                                            var _vy:Float=0.0;
                                                            {
                                                                _vx=v.x-u.x;
                                                                _vy=v.y-u.y;
                                                            };
                                                            var _qx:Float=0.0;
                                                            var _qy:Float=0.0;
                                                            {
                                                                _qx=b.x-a.x;
                                                                _qy=b.y-a.y;
                                                            };
                                                            var den=(_vy*_qx-_vx*_qy);
                                                            if(den*den>Config.epsilon*Config.epsilon){
                                                                den=1/den;
                                                                var txx=(_qy*_sx-_qx*_sy)*den;
                                                                if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                                    var sxx=(_vy*_sx-_vx*_sy)*den;
                                                                    if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                        t=txx;
                                                                        true;
                                                                    }
                                                                    else false;
                                                                }
                                                                else false;
                                                            }
                                                            else false;
                                                        }){
                                                            if(t<min){
                                                                min=t;
                                                                ite2=cx_itei;
                                                            }
                                                        }
                                                    };
                                                }
                                                while(false);
                                            }
                                        };
                                        if(min!=2.0){
                                            var cx:Float=0.0;
                                            var cy:Float=0.0;
                                            {
                                                var T=(min);
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(((T!=T))||T<0.0||T>1.0);
                                                    };
                                                    if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"min"+",out c: "+"c"+")");
                                                    #end
                                                };
                                                cx=u.x+(v.x-u.x)*T;
                                                cy=u.y+(v.y-u.y)*T;
                                            };
                                            fst_vert=ZPP_Vec2.get(cx,cy);
                                            flowpoly.add(fst_vert);
                                            poly1=true;
                                            ite1=cx_itei;
                                            {
                                                cx_cont=false;
                                                break;
                                            };
                                        }
                                    };
                                    {
                                        cx_itei=cx_itej;
                                        u=v;
                                        cx_itej=cx_itej.next;
                                    };
                                }
                                if(cx_cont){
                                    do{
                                        cx_itej=s1.polygon.gverts.begin();
                                        var v=cx_itej.elem();
                                        {
                                            var min=2.0;
                                            {
                                                var cx_cont=true;
                                                var cx_itei=s2.polygon.gverts.begin();
                                                var a=cx_itei.elem();
                                                var cx_itej=cx_itei.next;
                                                while(cx_itej!=null){
                                                    var b=cx_itej.elem();
                                                    {
                                                        var t=0.0;
                                                        if({
                                                            var _sx:Float=0.0;
                                                            var _sy:Float=0.0;
                                                            {
                                                                _sx=u.x-a.x;
                                                                _sy=u.y-a.y;
                                                            };
                                                            var _vx:Float=0.0;
                                                            var _vy:Float=0.0;
                                                            {
                                                                _vx=v.x-u.x;
                                                                _vy=v.y-u.y;
                                                            };
                                                            var _qx:Float=0.0;
                                                            var _qy:Float=0.0;
                                                            {
                                                                _qx=b.x-a.x;
                                                                _qy=b.y-a.y;
                                                            };
                                                            var den=(_vy*_qx-_vx*_qy);
                                                            if(den*den>Config.epsilon*Config.epsilon){
                                                                den=1/den;
                                                                var txx=(_qy*_sx-_qx*_sy)*den;
                                                                if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                                    var sxx=(_vy*_sx-_vx*_sy)*den;
                                                                    if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                        t=txx;
                                                                        true;
                                                                    }
                                                                    else false;
                                                                }
                                                                else false;
                                                            }
                                                            else false;
                                                        }){
                                                            if(t<min){
                                                                min=t;
                                                                ite2=cx_itei;
                                                            }
                                                        }
                                                    };
                                                    {
                                                        cx_itei=cx_itej;
                                                        a=b;
                                                        cx_itej=cx_itej.next;
                                                    };
                                                }
                                                if(cx_cont){
                                                    do{
                                                        cx_itej=s2.polygon.gverts.begin();
                                                        var b=cx_itej.elem();
                                                        {
                                                            var t=0.0;
                                                            if({
                                                                var _sx:Float=0.0;
                                                                var _sy:Float=0.0;
                                                                {
                                                                    _sx=u.x-a.x;
                                                                    _sy=u.y-a.y;
                                                                };
                                                                var _vx:Float=0.0;
                                                                var _vy:Float=0.0;
                                                                {
                                                                    _vx=v.x-u.x;
                                                                    _vy=v.y-u.y;
                                                                };
                                                                var _qx:Float=0.0;
                                                                var _qy:Float=0.0;
                                                                {
                                                                    _qx=b.x-a.x;
                                                                    _qy=b.y-a.y;
                                                                };
                                                                var den=(_vy*_qx-_vx*_qy);
                                                                if(den*den>Config.epsilon*Config.epsilon){
                                                                    den=1/den;
                                                                    var txx=(_qy*_sx-_qx*_sy)*den;
                                                                    if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                                        var sxx=(_vy*_sx-_vx*_sy)*den;
                                                                        if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                            t=txx;
                                                                            true;
                                                                        }
                                                                        else false;
                                                                    }
                                                                    else false;
                                                                }
                                                                else false;
                                                            }){
                                                                if(t<min){
                                                                    min=t;
                                                                    ite2=cx_itei;
                                                                }
                                                            }
                                                        };
                                                    }
                                                    while(false);
                                                }
                                            };
                                            if(min!=2.0){
                                                var cx:Float=0.0;
                                                var cy:Float=0.0;
                                                {
                                                    var T=(min);
                                                    {
                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                        var res={
                                                            !(((T!=T))||T<0.0||T>1.0);
                                                        };
                                                        if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"min"+",out c: "+"c"+")");
                                                        #end
                                                    };
                                                    cx=u.x+(v.x-u.x)*T;
                                                    cy=u.y+(v.y-u.y)*T;
                                                };
                                                fst_vert=ZPP_Vec2.get(cx,cy);
                                                flowpoly.add(fst_vert);
                                                poly1=true;
                                                ite1=cx_itei;
                                                break;
                                            }
                                        };
                                    }
                                    while(false);
                                }
                            };
                            cnt=2;
                        }
                        while(true){
                            if(poly1){
                                ite1=ite1.next;
                                ind1++;
                                if(ite1==null){
                                    ite1=s1.polygon.gverts.begin();
                                    ind1=0;
                                }
                                if(!out1[ind1]){
                                    var ex=ite1.elem();
                                    if(fst_vert!=null&&ZPP_VecMath.vec_dsq(ex.x,ex.y,fst_vert.x,fst_vert.y)<Config.epsilon)break;
                                    flowpoly.add(ex);
                                    if(fst_vert==null)fst_vert=flowpoly.front();
                                    cnt=1;
                                }
                                else{
                                    var a=flowpoly.front();
                                    var b=ite1.elem();
                                    var u=ite2.elem();
                                    var itm=ite2.next;
                                    if(itm==null)itm=s2.polygon.gverts.begin();
                                    var max=-1.0;
                                    var itmo=null;
                                    var indo=0;
                                    var icnt=0;
                                    {
                                        var beg_ite=itm;
                                        var cx_ite=itm;
                                        do{
                                            var v=cx_ite.elem();
                                            {
                                                var t=0.0;
                                                if({
                                                    var _sx:Float=0.0;
                                                    var _sy:Float=0.0;
                                                    {
                                                        _sx=u.x-a.x;
                                                        _sy=u.y-a.y;
                                                    };
                                                    var _vx:Float=0.0;
                                                    var _vy:Float=0.0;
                                                    {
                                                        _vx=v.x-u.x;
                                                        _vy=v.y-u.y;
                                                    };
                                                    var _qx:Float=0.0;
                                                    var _qy:Float=0.0;
                                                    {
                                                        _qx=b.x-a.x;
                                                        _qy=b.y-a.y;
                                                    };
                                                    var den=(_vy*_qx-_vx*_qy);
                                                    if(den*den>Config.epsilon*Config.epsilon){
                                                        den=1/den;
                                                        var txx=(_qy*_sx-_qx*_sy)*den;
                                                        if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                            var sxx=(_vy*_sx-_vx*_sy)*den;
                                                            if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                t=txx;
                                                                true;
                                                            }
                                                            else false;
                                                        }
                                                        else false;
                                                    }
                                                    else false;
                                                }){
                                                    if(t>=max){
                                                        itmo=ite2;
                                                        indo=ind2;
                                                        if(++icnt==cnt){
                                                            max=t;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else max=t;
                                                    }
                                                }
                                                u=v;
                                                ite2=cx_ite;
                                                ind2++;
                                                if(ind2>=s2.polygon.edgeCnt)ind2=0;
                                            };
                                            cx_ite=cx_ite.next;
                                            if(cx_ite==null)cx_ite=s2.polygon.gverts.begin();
                                        }
                                        while(false);
                                        while(cx_ite!=beg_ite){
                                            var v=cx_ite.elem();
                                            {
                                                var t=0.0;
                                                if({
                                                    var _sx:Float=0.0;
                                                    var _sy:Float=0.0;
                                                    {
                                                        _sx=u.x-a.x;
                                                        _sy=u.y-a.y;
                                                    };
                                                    var _vx:Float=0.0;
                                                    var _vy:Float=0.0;
                                                    {
                                                        _vx=v.x-u.x;
                                                        _vy=v.y-u.y;
                                                    };
                                                    var _qx:Float=0.0;
                                                    var _qy:Float=0.0;
                                                    {
                                                        _qx=b.x-a.x;
                                                        _qy=b.y-a.y;
                                                    };
                                                    var den=(_vy*_qx-_vx*_qy);
                                                    if(den*den>Config.epsilon*Config.epsilon){
                                                        den=1/den;
                                                        var txx=(_qy*_sx-_qx*_sy)*den;
                                                        if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                            var sxx=(_vy*_sx-_vx*_sy)*den;
                                                            if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                t=txx;
                                                                true;
                                                            }
                                                            else false;
                                                        }
                                                        else false;
                                                    }
                                                    else false;
                                                }){
                                                    if(t>=max){
                                                        itmo=ite2;
                                                        indo=ind2;
                                                        if(++icnt==cnt){
                                                            max=t;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else max=t;
                                                    }
                                                }
                                                u=v;
                                                ite2=cx_ite;
                                                ind2++;
                                                if(ind2>=s2.polygon.edgeCnt)ind2=0;
                                            };
                                            cx_ite=cx_ite.next;
                                            if(cx_ite==null)cx_ite=s2.polygon.gverts.begin();
                                        }
                                    };
                                    if(itmo==null)break;
                                    var u=itmo.elem();
                                    var itm2=itmo.next;
                                    if(itm2==null)itm2=s2.polygon.gverts.begin();
                                    var v=itm2.elem();
                                    var cx:Float=0.0;
                                    var cy:Float=0.0;
                                    {
                                        var T=(max);
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !(((T!=T))||T<0.0||T>1.0);
                                            };
                                            if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"max"+",out c: "+"c"+")");
                                            #end
                                        };
                                        cx=u.x+(v.x-u.x)*T;
                                        cy=u.y+(v.y-u.y)*T;
                                    };
                                    if(fst_vert!=null&&ZPP_VecMath.vec_dsq(cx,cy,fst_vert.x,fst_vert.y)<Config.epsilon)break;
                                    flowpoly.add(ZPP_Vec2.get(cx,cy));
                                    if(fst_vert==null)fst_vert=flowpoly.front();
                                    ite2=itmo;
                                    ind2=indo;
                                    poly1=!poly1;
                                    cnt=2;
                                }
                            };
                            else{
                                ite2=ite2.next;
                                ind2++;
                                if(ite2==null){
                                    ite2=s2.polygon.gverts.begin();
                                    ind2=0;
                                }
                                if(!out2[ind2]){
                                    var ex=ite2.elem();
                                    if(fst_vert!=null&&ZPP_VecMath.vec_dsq(ex.x,ex.y,fst_vert.x,fst_vert.y)<Config.epsilon)break;
                                    flowpoly.add(ex);
                                    if(fst_vert==null)fst_vert=flowpoly.front();
                                    cnt=1;
                                }
                                else{
                                    var a=flowpoly.front();
                                    var b=ite2.elem();
                                    var u=ite1.elem();
                                    var itm=ite1.next;
                                    if(itm==null)itm=s1.polygon.gverts.begin();
                                    var max=-1.0;
                                    var itmo=null;
                                    var indo=0;
                                    var icnt=0;
                                    {
                                        var beg_ite=itm;
                                        var cx_ite=itm;
                                        do{
                                            var v=cx_ite.elem();
                                            {
                                                var t=0.0;
                                                if({
                                                    var _sx:Float=0.0;
                                                    var _sy:Float=0.0;
                                                    {
                                                        _sx=u.x-a.x;
                                                        _sy=u.y-a.y;
                                                    };
                                                    var _vx:Float=0.0;
                                                    var _vy:Float=0.0;
                                                    {
                                                        _vx=v.x-u.x;
                                                        _vy=v.y-u.y;
                                                    };
                                                    var _qx:Float=0.0;
                                                    var _qy:Float=0.0;
                                                    {
                                                        _qx=b.x-a.x;
                                                        _qy=b.y-a.y;
                                                    };
                                                    var den=(_vy*_qx-_vx*_qy);
                                                    if(den*den>Config.epsilon*Config.epsilon){
                                                        den=1/den;
                                                        var txx=(_qy*_sx-_qx*_sy)*den;
                                                        if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                            var sxx=(_vy*_sx-_vx*_sy)*den;
                                                            if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                t=txx;
                                                                true;
                                                            }
                                                            else false;
                                                        }
                                                        else false;
                                                    }
                                                    else false;
                                                }){
                                                    if(t>=max){
                                                        itmo=ite1;
                                                        indo=ind1;
                                                        if(++icnt==cnt){
                                                            max=t;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else max=t;
                                                    }
                                                }
                                                u=v;
                                                ite1=cx_ite;
                                                ind1++;
                                                if(ind1>=s1.polygon.edgeCnt)ind1=0;
                                            };
                                            cx_ite=cx_ite.next;
                                            if(cx_ite==null)cx_ite=s1.polygon.gverts.begin();
                                        }
                                        while(false);
                                        while(cx_ite!=beg_ite){
                                            var v=cx_ite.elem();
                                            {
                                                var t=0.0;
                                                if({
                                                    var _sx:Float=0.0;
                                                    var _sy:Float=0.0;
                                                    {
                                                        _sx=u.x-a.x;
                                                        _sy=u.y-a.y;
                                                    };
                                                    var _vx:Float=0.0;
                                                    var _vy:Float=0.0;
                                                    {
                                                        _vx=v.x-u.x;
                                                        _vy=v.y-u.y;
                                                    };
                                                    var _qx:Float=0.0;
                                                    var _qy:Float=0.0;
                                                    {
                                                        _qx=b.x-a.x;
                                                        _qy=b.y-a.y;
                                                    };
                                                    var den=(_vy*_qx-_vx*_qy);
                                                    if(den*den>Config.epsilon*Config.epsilon){
                                                        den=1/den;
                                                        var txx=(_qy*_sx-_qx*_sy)*den;
                                                        if(txx>Config.epsilon&&txx<1-Config.epsilon){
                                                            var sxx=(_vy*_sx-_vx*_sy)*den;
                                                            if(sxx>Config.epsilon&&sxx<1-Config.epsilon){
                                                                t=txx;
                                                                true;
                                                            }
                                                            else false;
                                                        }
                                                        else false;
                                                    }
                                                    else false;
                                                }){
                                                    if(t>=max){
                                                        itmo=ite1;
                                                        indo=ind1;
                                                        if(++icnt==cnt){
                                                            max=t;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else max=t;
                                                    }
                                                }
                                                u=v;
                                                ite1=cx_ite;
                                                ind1++;
                                                if(ind1>=s1.polygon.edgeCnt)ind1=0;
                                            };
                                            cx_ite=cx_ite.next;
                                            if(cx_ite==null)cx_ite=s1.polygon.gverts.begin();
                                        }
                                    };
                                    if(itmo==null)break;
                                    var u=itmo.elem();
                                    var itm2=itmo.next;
                                    if(itm2==null)itm2=s1.polygon.gverts.begin();
                                    var v=itm2.elem();
                                    var cx:Float=0.0;
                                    var cy:Float=0.0;
                                    {
                                        var T=(max);
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !(((T!=T))||T<0.0||T>1.0);
                                            };
                                            if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"max"+",out c: "+"c"+")");
                                            #end
                                        };
                                        cx=u.x+(v.x-u.x)*T;
                                        cy=u.y+(v.y-u.y)*T;
                                    };
                                    if(fst_vert!=null&&ZPP_VecMath.vec_dsq(cx,cy,fst_vert.x,fst_vert.y)<Config.epsilon)break;
                                    flowpoly.add(ZPP_Vec2.get(cx,cy));
                                    if(fst_vert==null)fst_vert=flowpoly.front();
                                    ite1=itmo;
                                    ind1=indo;
                                    poly1=!poly1;
                                    cnt=2;
                                }
                            };
                        }
                        if(flowpoly.begin()!=null&&flowpoly.begin().next!=null&&flowpoly.begin().next.next!=null){
                            var area=0.0;
                            var COMx:Float=0.0;
                            var COMy:Float=0.0;
                            {
                                {
                                    COMx=0;
                                    COMy=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((COMx!=COMx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(COMx)"+") :: "+("vec_set(in n: "+"COM"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((COMy!=COMy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(COMy)"+") :: "+("vec_set(in n: "+"COM"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                };
                                area=0.0;
                                {
                                    var cx_ite=flowpoly.begin();
                                    var u=cx_ite.elem();
                                    cx_ite=cx_ite.next;
                                    var v=cx_ite.elem();
                                    cx_ite=cx_ite.next;
                                    while(cx_ite!=null){
                                        var w=cx_ite.elem();
                                        {
                                            area+=v.x*(w.y-u.y);
                                            var cf=(w.y*v.x-w.x*v.y);
                                            COMx+=(v.x+w.x)*cf;
                                            COMy+=(v.y+w.y)*cf;
                                        };
                                        u=v;
                                        v=w;
                                        cx_ite=cx_ite.next;
                                    }
                                    cx_ite=flowpoly.begin();
                                    var w=cx_ite.elem();
                                    {
                                        area+=v.x*(w.y-u.y);
                                        var cf=(w.y*v.x-w.x*v.y);
                                        COMx+=(v.x+w.x)*cf;
                                        COMy+=(v.y+w.y)*cf;
                                    };
                                    u=v;
                                    v=w;
                                    cx_ite=cx_ite.next;
                                    var w=cx_ite.elem();
                                    {
                                        area+=v.x*(w.y-u.y);
                                        var cf=(w.y*v.x-w.x*v.y);
                                        COMx+=(v.x+w.x)*cf;
                                        COMy+=(v.y+w.y)*cf;
                                    };
                                };
                                area*=0.5;
                                var ia=1/(6*area);
                                {
                                    var t=(ia);
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((t!=t));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"COM"+",in s: "+"ia"+")");
                                        #end
                                    };
                                    COMx*=t;
                                    COMy*=t;
                                };
                            };
                            arb.inject(-area,COMx,COMy);
                            true;
                        }
                        else false;
                    }
                    else false;
                }
                else false;
            };
            else{
                var inte=new Array<Bool>();
                var total=true;
                var a0=null,vi=null;
                var max=-ZPP_Const.FMAX;
                var cont=true;
                var vite=s2.polygon.gverts.begin();
                var ind=0;
                {
                    var cx_ite=s2.polygon.edges.begin();
                    while(cx_ite!=null){
                        var a=cx_ite.elem();
                        {
                            var dist=(a.gnormx*s1.circle.worldCOMx+a.gnormy*s1.circle.worldCOMy);
                            if(dist>a.gprojection+s1.circle.radius){
                                cont=false;
                                break;
                            }
                            else if(dist+s1.circle.radius>a.gprojection+Config.epsilon){
                                total=false;
                                inte[ind]=true;
                            }
                            dist-=a.gprojection+s1.circle.radius;
                            if(dist>max){
                                max=dist;
                                a0=a;
                                vi=vite;
                            }
                            vite=vite.next;
                            ind++;
                        };
                        cx_ite=cx_ite.next;
                    }
                };
                if(cont){
                    if(total){
                        arb.inject(s1.circle.area,s1.circle.worldCOMx,s1.circle.worldCOMy);
                        true;
                    }
                    else{
                        var v0=vi.elem();
                        var v1=if(vi.next==null)s2.polygon.gverts.front()else vi.next.elem();
                        var dt=(s1.circle.worldCOMy*a0.gnormx-s1.circle.worldCOMx*a0.gnormy);
                        if(if(dt<=(v0.y*a0.gnormx-v0.x*a0.gnormy)){
                            var minDist=s1.circle.radius+0;
                            var px:Float=0.0;
                            var py:Float=0.0;
                            {
                                px=v0.x-s1.circle.worldCOMx;
                                py=v0.y-s1.circle.worldCOMy;
                            };
                            var distSqr=(px*px+py*py);
                            distSqr<=minDist*minDist;
                        }
                        else if(dt>=(v1.y*a0.gnormx-v1.x*a0.gnormy)){
                            var minDist=s1.circle.radius+0;
                            var px:Float=0.0;
                            var py:Float=0.0;
                            {
                                px=v1.x-s1.circle.worldCOMx;
                                py=v1.y-s1.circle.worldCOMy;
                            };
                            var distSqr=(px*px+py*py);
                            distSqr<=minDist*minDist;
                        }
                        else true){
                            var ins=new Array<Bool>();
                            var ind=0;
                            var total=true;
                            var vi=null;
                            var vind=0;
                            {
                                var cx_ite=s2.polygon.gverts.begin();
                                while(cx_ite!=null){
                                    var v=cx_ite.elem();
                                    {
                                        var dist=ZPP_VecMath.vec_dsq(v.x,v.y,s1.circle.worldCOMx,s1.circle.worldCOMy);
                                        if(!(ins[ind]=(dist<=s1.circle.radius*s1.circle.radius)))total=false;
                                        else{
                                            vind=ind;
                                            vi=cx_ite;
                                        }
                                        ind++;
                                    };
                                    cx_ite=cx_ite.next;
                                }
                            };
                            if(total){
                                s2.polygon.validate_worldCOM();
                                arb.inject(s2.polygon.area,s2.polygon.worldCOMx,s2.polygon.worldCOMy);
                                true;
                            }
                            else{
                                while(!flowpoly.empty()){
                                    var p=flowpoly.pop_unsafe();
                                    if(!p._inuse){
                                        var o=p;
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                o!=null;
                                            };
                                            if(!res)throw "assert("+"o!=null"+") :: "+("Free(in T: "+"ZPP_Vec2"+", in obj: "+"p"+")");
                                            #end
                                        };
                                        o.free();
                                        o.next=ZPP_Vec2.zpp_pool;
                                        ZPP_Vec2.zpp_pool=o;
                                        #if NAPE_POOL_STATS ZPP_Vec2.POOL_CNT++;
                                        ZPP_Vec2.POOL_SUB++;
                                        #end
                                    };
                                }
                                flowsegs.clear();
                                var fst_vert=null;
                                var state=1;
                                if(vi==null){
                                    vi=s2.polygon.gverts.begin();
                                    state=2;
                                }
                                else flowpoly.add(fst_vert=vi.elem());
                                while(state!=0){
                                    if(state==1){
                                        vi=vi.next;
                                        if(vi==null)vi=s2.polygon.gverts.begin();
                                        vind++;
                                        if(vind>=s2.polygon.edgeCnt)vind=0;
                                        if(ins[vind]){
                                            if(ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,vi.elem().x,vi.elem().y)<Config.epsilon)break;
                                            flowpoly.add(vi.elem());
                                        }
                                        else{
                                            var u=flowpoly.front();
                                            var v=vi.elem();
                                            var tval={
                                                var vx:Float=0.0;
                                                var vy:Float=0.0;
                                                {
                                                    vx=v.x-u.x;
                                                    vy=v.y-u.y;
                                                };
                                                var qx:Float=0.0;
                                                var qy:Float=0.0;
                                                {
                                                    qx=u.x-s1.circle.worldCOMx;
                                                    qy=u.y-s1.circle.worldCOMy;
                                                };
                                                var A=(vx*vx+vy*vy);
                                                var B=2*(qx*vx+qy*vy);
                                                var C=(qx*qx+qy*qy)-s1.circle.radius*s1.circle.radius;
                                                var D=Math.sqrt(B*B-4*A*C);
                                                A=1/(2*A);
                                                var t=(-B-D)*A;
                                                if(t<Config.epsilon)(-B+D)*A;
                                                else t;
                                            };
                                            var cx:Float=0.0;
                                            var cy:Float=0.0;
                                            {
                                                var T=(tval);
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !(((T!=T))||T<0.0||T>1.0);
                                                    };
                                                    if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"tval"+",out c: "+"c"+")");
                                                    #end
                                                };
                                                cx=u.x+(v.x-u.x)*T;
                                                cy=u.y+(v.y-u.y)*T;
                                            };
                                            if(ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,cx,cy)<Config.epsilon)break;
                                            flowpoly.add(ZPP_Vec2.get(cx,cy));
                                            state=2;
                                        }
                                    }
                                    else if(state==2){
                                        var vi2=vi.next;
                                        if(vi2==null)vi2=s2.polygon.gverts.begin();
                                        var u=vi.elem();
                                        state=0;
                                        {
                                            var beg_ite=vi2;
                                            var cx_ite=vi2;
                                            do{
                                                var v=cx_ite.elem();
                                                {
                                                    var vind2=vind+1;
                                                    if(vind2==s2.polygon.edgeCnt)vind2=0;
                                                    if(inte[vind]){
                                                        if(ins[vind2]){
                                                            var tval={
                                                                var vx:Float=0.0;
                                                                var vy:Float=0.0;
                                                                {
                                                                    vx=v.x-u.x;
                                                                    vy=v.y-u.y;
                                                                };
                                                                var qx:Float=0.0;
                                                                var qy:Float=0.0;
                                                                {
                                                                    qx=u.x-s1.circle.worldCOMx;
                                                                    qy=u.y-s1.circle.worldCOMy;
                                                                };
                                                                var A=(vx*vx+vy*vy);
                                                                var B=2*(qx*vx+qy*vy);
                                                                var C=(qx*qx+qy*qy)-s1.circle.radius*s1.circle.radius;
                                                                var D=Math.sqrt(B*B-4*A*C);
                                                                A=1/(2*A);
                                                                var t=(-B-D)*A;
                                                                if(t<Config.epsilon)(-B+D)*A;
                                                                else t;
                                                            };
                                                            var cx:Float=0.0;
                                                            var cy:Float=0.0;
                                                            {
                                                                var T=(tval);
                                                                {
                                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                    var res={
                                                                        !(((T!=T))||T<0.0||T>1.0);
                                                                    };
                                                                    if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"tval"+",out c: "+"c"+")");
                                                                    #end
                                                                };
                                                                cx=u.x+(v.x-u.x)*T;
                                                                cy=u.y+(v.y-u.y)*T;
                                                            };
                                                            if(ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,cx,cy)<Config.epsilon){
                                                                state=0;
                                                                {
                                                                    cx_ite=beg_ite;
                                                                    break;
                                                                };
                                                            }
                                                            var cp=ZPP_Vec2.get(cx,cy);
                                                            flowsegs.add(flowpoly.front());
                                                            flowsegs.add(cp);
                                                            flowpoly.add(cp);
                                                            state=1;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else{
                                                            var t0=0.0;
                                                            var t1=0.0;
                                                            var two={
                                                                var vx:Float=0.0;
                                                                var vy:Float=0.0;
                                                                {
                                                                    vx=v.x-u.x;
                                                                    vy=v.y-u.y;
                                                                };
                                                                var qx:Float=0.0;
                                                                var qy:Float=0.0;
                                                                {
                                                                    qx=u.x-s1.circle.worldCOMx;
                                                                    qy=u.y-s1.circle.worldCOMy;
                                                                };
                                                                var A=(vx*vx+vy*vy);
                                                                var B=2*(qx*vx+qy*vy);
                                                                var C=(qx*qx+qy*qy)-s1.circle.radius*s1.circle.radius;
                                                                var D=B*B-4*A*C;
                                                                if(D*D<Config.epsilon){
                                                                    if(D<0)t0=10.0;
                                                                    else t0=t1=-B/(2*A);
                                                                    false;
                                                                }
                                                                else{
                                                                    D=Math.sqrt(D);
                                                                    A=1/(2*A);
                                                                    t0=(-B-D)*A;
                                                                    t1=(-B+D)*A;
                                                                    true;
                                                                }
                                                            };
                                                            if(t0<1-Config.epsilon&&t1>Config.epsilon){
                                                                var cx:Float=0.0;
                                                                var cy:Float=0.0;
                                                                {
                                                                    var T=(t0);
                                                                    {
                                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                        var res={
                                                                            !(((T!=T))||T<0.0||T>1.0);
                                                                        };
                                                                        if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"t0"+",out c: "+"c"+")");
                                                                        #end
                                                                    };
                                                                    cx=u.x+(v.x-u.x)*T;
                                                                    cy=u.y+(v.y-u.y)*T;
                                                                };
                                                                if(fst_vert!=null&&ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,cx,cy)<Config.epsilon){
                                                                    state=0;
                                                                    {
                                                                        cx_ite=beg_ite;
                                                                        break;
                                                                    };
                                                                }
                                                                var cp=ZPP_Vec2.get(cx,cy);
                                                                if(!flowpoly.empty()){
                                                                    flowsegs.add(flowpoly.front());
                                                                    flowsegs.add(cp);
                                                                }
                                                                flowpoly.add(cp);
                                                                if(fst_vert==null)fst_vert=flowpoly.front();
                                                                if(two){
                                                                    var cx:Float=0.0;
                                                                    var cy:Float=0.0;
                                                                    {
                                                                        var T=(t1);
                                                                        {
                                                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                            var res={
                                                                                !(((T!=T))||T<0.0||T>1.0);
                                                                            };
                                                                            if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"t1"+",out c: "+"c"+")");
                                                                            #end
                                                                        };
                                                                        cx=u.x+(v.x-u.x)*T;
                                                                        cy=u.y+(v.y-u.y)*T;
                                                                    };
                                                                    flowpoly.add(ZPP_Vec2.get(cx,cy));
                                                                }
                                                            }
                                                        }
                                                    }
                                                    u=v;
                                                    vi=cx_ite;
                                                    vind=vind2;
                                                };
                                                cx_ite=cx_ite.next;
                                                if(cx_ite==null)cx_ite=s2.polygon.gverts.begin();
                                            }
                                            while(false);
                                            while(cx_ite!=beg_ite){
                                                var v=cx_ite.elem();
                                                {
                                                    var vind2=vind+1;
                                                    if(vind2==s2.polygon.edgeCnt)vind2=0;
                                                    if(inte[vind]){
                                                        if(ins[vind2]){
                                                            var tval={
                                                                var vx:Float=0.0;
                                                                var vy:Float=0.0;
                                                                {
                                                                    vx=v.x-u.x;
                                                                    vy=v.y-u.y;
                                                                };
                                                                var qx:Float=0.0;
                                                                var qy:Float=0.0;
                                                                {
                                                                    qx=u.x-s1.circle.worldCOMx;
                                                                    qy=u.y-s1.circle.worldCOMy;
                                                                };
                                                                var A=(vx*vx+vy*vy);
                                                                var B=2*(qx*vx+qy*vy);
                                                                var C=(qx*qx+qy*qy)-s1.circle.radius*s1.circle.radius;
                                                                var D=Math.sqrt(B*B-4*A*C);
                                                                A=1/(2*A);
                                                                var t=(-B-D)*A;
                                                                if(t<Config.epsilon)(-B+D)*A;
                                                                else t;
                                                            };
                                                            var cx:Float=0.0;
                                                            var cy:Float=0.0;
                                                            {
                                                                var T=(tval);
                                                                {
                                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                    var res={
                                                                        !(((T!=T))||T<0.0||T>1.0);
                                                                    };
                                                                    if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"tval"+",out c: "+"c"+")");
                                                                    #end
                                                                };
                                                                cx=u.x+(v.x-u.x)*T;
                                                                cy=u.y+(v.y-u.y)*T;
                                                            };
                                                            if(ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,cx,cy)<Config.epsilon){
                                                                state=0;
                                                                {
                                                                    cx_ite=beg_ite;
                                                                    break;
                                                                };
                                                            }
                                                            var cp=ZPP_Vec2.get(cx,cy);
                                                            flowsegs.add(flowpoly.front());
                                                            flowsegs.add(cp);
                                                            flowpoly.add(cp);
                                                            state=1;
                                                            {
                                                                cx_ite=beg_ite;
                                                                break;
                                                            };
                                                        }
                                                        else{
                                                            var t0=0.0;
                                                            var t1=0.0;
                                                            var two={
                                                                var vx:Float=0.0;
                                                                var vy:Float=0.0;
                                                                {
                                                                    vx=v.x-u.x;
                                                                    vy=v.y-u.y;
                                                                };
                                                                var qx:Float=0.0;
                                                                var qy:Float=0.0;
                                                                {
                                                                    qx=u.x-s1.circle.worldCOMx;
                                                                    qy=u.y-s1.circle.worldCOMy;
                                                                };
                                                                var A=(vx*vx+vy*vy);
                                                                var B=2*(qx*vx+qy*vy);
                                                                var C=(qx*qx+qy*qy)-s1.circle.radius*s1.circle.radius;
                                                                var D=B*B-4*A*C;
                                                                if(D*D<Config.epsilon){
                                                                    if(D<0)t0=10.0;
                                                                    else t0=t1=-B/(2*A);
                                                                    false;
                                                                }
                                                                else{
                                                                    D=Math.sqrt(D);
                                                                    A=1/(2*A);
                                                                    t0=(-B-D)*A;
                                                                    t1=(-B+D)*A;
                                                                    true;
                                                                }
                                                            };
                                                            if(t0<1-Config.epsilon&&t1>Config.epsilon){
                                                                var cx:Float=0.0;
                                                                var cy:Float=0.0;
                                                                {
                                                                    var T=(t0);
                                                                    {
                                                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                        var res={
                                                                            !(((T!=T))||T<0.0||T>1.0);
                                                                        };
                                                                        if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"t0"+",out c: "+"c"+")");
                                                                        #end
                                                                    };
                                                                    cx=u.x+(v.x-u.x)*T;
                                                                    cy=u.y+(v.y-u.y)*T;
                                                                };
                                                                if(fst_vert!=null&&ZPP_VecMath.vec_dsq(fst_vert.x,fst_vert.y,cx,cy)<Config.epsilon){
                                                                    state=0;
                                                                    {
                                                                        cx_ite=beg_ite;
                                                                        break;
                                                                    };
                                                                }
                                                                var cp=ZPP_Vec2.get(cx,cy);
                                                                if(!flowpoly.empty()){
                                                                    flowsegs.add(flowpoly.front());
                                                                    flowsegs.add(cp);
                                                                }
                                                                flowpoly.add(cp);
                                                                if(fst_vert==null)fst_vert=flowpoly.front();
                                                                if(two){
                                                                    var cx:Float=0.0;
                                                                    var cy:Float=0.0;
                                                                    {
                                                                        var T=(t1);
                                                                        {
                                                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                                            var res={
                                                                                !(((T!=T))||T<0.0||T>1.0);
                                                                            };
                                                                            if(!res)throw "assert("+"!(assert_isNaN(T)||T<0.0||T>1.0)"+") :: "+("vec_interp(in a: "+"u."+",in b: "+"v."+",in t: "+"t1"+",out c: "+"c"+")");
                                                                            #end
                                                                        };
                                                                        cx=u.x+(v.x-u.x)*T;
                                                                        cy=u.y+(v.y-u.y)*T;
                                                                    };
                                                                    flowpoly.add(ZPP_Vec2.get(cx,cy));
                                                                }
                                                            }
                                                        }
                                                    }
                                                    u=v;
                                                    vi=cx_ite;
                                                    vind=vind2;
                                                };
                                                cx_ite=cx_ite.next;
                                                if(cx_ite==null)cx_ite=s2.polygon.gverts.begin();
                                            }
                                        };
                                    }
                                }
                                if(flowpoly.begin()==null){
                                    false;
                                }
                                else if(flowpoly.begin().next==null){
                                    var all=true;
                                    {
                                        var cx_ite=s2.polygon.edges.begin();
                                        while(cx_ite!=null){
                                            var e=cx_ite.elem();
                                            {
                                                var dist=(e.gnormx*s1.circle.worldCOMx+e.gnormy*s1.circle.worldCOMy);
                                                if(dist>e.gprojection){
                                                    all=false;
                                                    break;
                                                }
                                            };
                                            cx_ite=cx_ite.next;
                                        }
                                    };
                                    if(all){
                                        arb.inject(s1.circle.area,s1.circle.worldCOMx,s1.circle.worldCOMy);
                                        true;
                                    }
                                    else false;
                                }
                                else{
                                    var COMx:Float=0;
                                    var COMy:Float=0;
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((COMx!=COMx));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(COMx)"+") :: "+("vec_new(in n: "+"COM"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                    {
                                        #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                        var res={
                                            !((COMy!=COMy));
                                        };
                                        if(!res)throw "assert("+"!assert_isNaN(COMy)"+") :: "+("vec_new(in n: "+"COM"+",in x: "+"0"+",in y: "+"0"+")");
                                        #end
                                    };
                                    var area=0.0;
                                    if(flowpoly.begin().next.next!=null){
                                        var parea=0.0;
                                        var pCOMx:Float=0.0;
                                        var pCOMy:Float=0.0;
                                        {
                                            {
                                                pCOMx=0;
                                                pCOMy=0;
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !((pCOMx!=pCOMx));
                                                    };
                                                    if(!res)throw "assert("+"!assert_isNaN(pCOMx)"+") :: "+("vec_set(in n: "+"pCOM"+",in x: "+"0"+",in y: "+"0"+")");
                                                    #end
                                                };
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !((pCOMy!=pCOMy));
                                                    };
                                                    if(!res)throw "assert("+"!assert_isNaN(pCOMy)"+") :: "+("vec_set(in n: "+"pCOM"+",in x: "+"0"+",in y: "+"0"+")");
                                                    #end
                                                };
                                            };
                                            parea=0.0;
                                            {
                                                var cx_ite=flowpoly.begin();
                                                var u=cx_ite.elem();
                                                cx_ite=cx_ite.next;
                                                var v=cx_ite.elem();
                                                cx_ite=cx_ite.next;
                                                while(cx_ite!=null){
                                                    var w=cx_ite.elem();
                                                    {
                                                        parea+=v.x*(w.y-u.y);
                                                        var cf=(w.y*v.x-w.x*v.y);
                                                        pCOMx+=(v.x+w.x)*cf;
                                                        pCOMy+=(v.y+w.y)*cf;
                                                    };
                                                    u=v;
                                                    v=w;
                                                    cx_ite=cx_ite.next;
                                                }
                                                cx_ite=flowpoly.begin();
                                                var w=cx_ite.elem();
                                                {
                                                    parea+=v.x*(w.y-u.y);
                                                    var cf=(w.y*v.x-w.x*v.y);
                                                    pCOMx+=(v.x+w.x)*cf;
                                                    pCOMy+=(v.y+w.y)*cf;
                                                };
                                                u=v;
                                                v=w;
                                                cx_ite=cx_ite.next;
                                                var w=cx_ite.elem();
                                                {
                                                    parea+=v.x*(w.y-u.y);
                                                    var cf=(w.y*v.x-w.x*v.y);
                                                    pCOMx+=(v.x+w.x)*cf;
                                                    pCOMy+=(v.y+w.y)*cf;
                                                };
                                            };
                                            parea*=0.5;
                                            var ia=1/(6*parea);
                                            {
                                                var t=(ia);
                                                {
                                                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                    var res={
                                                        !((t!=t));
                                                    };
                                                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"pCOM"+",in s: "+"ia"+")");
                                                    #end
                                                };
                                                pCOMx*=t;
                                                pCOMy*=t;
                                            };
                                        };
                                        {
                                            var t=(-parea);
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((t!=t));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"COM"+",in b: "+"pCOM"+",in s: "+"-parea"+")");
                                                #end
                                            };
                                            COMx+=pCOMx*t;
                                            COMy+=pCOMy*t;
                                        };
                                        area-=parea;
                                    }
                                    else{
                                        flowsegs.add(flowpoly.front());
                                        flowsegs.add(flowpoly.begin().next.elem());
                                    }
                                    while(!flowsegs.empty()){
                                        var u=flowsegs.pop_unsafe();
                                        var v=flowsegs.pop_unsafe();
                                        var dx:Float=0.0;
                                        var dy:Float=0.0;
                                        {
                                            dx=v.x-u.x;
                                            dy=v.y-u.y;
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
                                        var cx:Float=0.0;
                                        var cy:Float=0.0;
                                        {
                                            cx=u.x+v.x;
                                            cy=u.y+v.y;
                                        };
                                        {
                                            var t=(1.0/(2));
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((t!=t));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"c"+",in s: "+"1.0/(2)"+")");
                                                #end
                                            };
                                            cx*=t;
                                            cy*=t;
                                        };
                                        {
                                            var t=(1.0);
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((t!=t));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_subeq(in a: "+"c"+",in b: "+"s1.circle.worldCOM"+",in s: "+"1.0"+")");
                                                #end
                                            };
                                            cx-=s1.circle.worldCOMx*t;
                                            cy-=s1.circle.worldCOMy*t;
                                        };
                                        var xd=(nx*cx+ny*cy);
                                        var carea=0.0;
                                        var ccom=0.0;
                                        {
                                            var X=xd;
                                            var cos=X/s1.circle.radius;
                                            var sin=Math.sqrt(1-cos*cos);
                                            var theta=Math.acos(cos);
                                            carea=s1.circle.radius*(s1.circle.radius*theta-X*sin);
                                            ccom=(2/3)*s1.circle.radius*sin*sin*sin/(theta-cos*sin);
                                        };
                                        {
                                            cx=s1.circle.worldCOMx;
                                            cy=s1.circle.worldCOMy;
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((cx!=cx));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(cx)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"s1.circle.worldCOMx"+",in y: "+"s1.circle.worldCOMy"+")");
                                                #end
                                            };
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((cy!=cy));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(cy)"+") :: "+("vec_set(in n: "+"c"+",in x: "+"s1.circle.worldCOMx"+",in y: "+"s1.circle.worldCOMy"+")");
                                                #end
                                            };
                                        };
                                        {
                                            var t=(ccom);
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((t!=t));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"c"+",in b: "+"n"+",in s: "+"ccom"+")");
                                                #end
                                            };
                                            cx+=nx*t;
                                            cy+=ny*t;
                                        };
                                        {
                                            var t=(carea);
                                            {
                                                #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                                var res={
                                                    !((t!=t));
                                                };
                                                if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"COM"+",in b: "+"c"+",in s: "+"carea"+")");
                                                #end
                                            };
                                            COMx+=cx*t;
                                            COMy+=cy*t;
                                        };
                                        area+=carea;
                                    }
                                    {
                                        var t=(1.0/(area));
                                        {
                                            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                                            var res={
                                                !((t!=t));
                                            };
                                            if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_muleq(in a: "+"COM"+",in s: "+"1.0/(area)"+")");
                                            #end
                                        };
                                        COMx*=t;
                                        COMy*=t;
                                    };
                                    arb.inject(area,COMx,COMy);
                                    true;
                                }
                            }
                        }
                        else false;
                    }
                }
                else false;
            };
        }
        else return{
            var c1=s1.circle;
            var c2=s2.circle;
            var deltax:Float=0.0;
            var deltay:Float=0.0;
            {
                deltax=c2.worldCOMx-c1.worldCOMx;
                deltay=c2.worldCOMy-c1.worldCOMy;
            };
            var cr=c1.radius+c2.radius;
            var ds=(deltax*deltax+deltay*deltay);
            if(ds>cr*cr)false;
            else if(ds<Config.epsilon*Config.epsilon){
                if(c1.radius<c2.radius)arb.inject(c1.area,c1.worldCOMx,c1.worldCOMy);
                else arb.inject(c2.area,c2.worldCOMx,c2.worldCOMy);
                true;
            }
            else{
                var d=Math.sqrt(ds);
                var id=1/d;
                var x1=0.5*(d-(c2.radius*c2.radius-c1.radius*c1.radius)*id);
                if(x1<=-c1.radius)arb.inject(c1.area,c1.worldCOMx,c1.worldCOMy);
                else{
                    var x2=d-x1;
                    if(x2<=-c2.radius)arb.inject(c2.area,c2.worldCOMx,c2.worldCOMy);
                    else{
                        var area1=0.0;
                        var y1=0.0;
                        var area2=0.0;
                        var y2=0.0;
                        {
                            var X=x1;
                            var cos=X/c1.radius;
                            var sin=Math.sqrt(1-cos*cos);
                            var theta=Math.acos(cos);
                            area1=c1.radius*(c1.radius*theta-X*sin);
                            y1=(2/3)*c1.radius*sin*sin*sin/(theta-cos*sin);
                        };
                        {
                            var X=x2;
                            var cos=X/c2.radius;
                            var sin=Math.sqrt(1-cos*cos);
                            var theta=Math.acos(cos);
                            area2=c2.radius*(c2.radius*theta-X*sin);
                            y2=(2/3)*c2.radius*sin*sin*sin/(theta-cos*sin);
                        };
                        var tarea=area1+area2;
                        var ya=(y1*area1+(d-y2)*area2)/tarea*id;
                        arb.inject(tarea,c1.worldCOMx+deltax*ya,c1.worldCOMy+deltay*ya);
                    }
                }
                true;
            }
        };
    }
    public static var flowpoly:ZNPList_ZPP_Vec2=new ZNPList_ZPP_Vec2();
    public static var flowsegs=new ZNPList_ZPP_Vec2();
}

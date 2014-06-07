package zpp_nape.constraint;
import zpp_nape.Const;
import zpp_nape.constraint.PivotJoint;
import zpp_nape.ID;
import zpp_nape.constraint.Constraint;
import zpp_nape.constraint.WeldJoint;
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
class ZPP_UserConstraint extends ZPP_Constraint{
    public var outer_zn:UserConstraint=null;
    public function bindVec2_invalidate(_){
        outer_zn.__invalidate();
    }
    public var bodies:TArray<ZPP_UserBody>=null;
    public var dim:Int=0;
    public var jAcc:TArray<Float>=null;
    public var bias:TArray<Float>=null;
    public function addBody(b:ZPP_Body){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                b!=null;
            };
            if(!res)throw "assert("+"b!=null"+") :: "+("addBody :: null UserConstraint");
            #end
        };
        var match=null;
        for(x in bodies){
            if(x.body==b){
                match=x;
                break;
            }
        };
        if(match==null){
            bodies.push(new ZPP_UserBody(1,b));
            if(active&&space!=null){
                if(b!=null)b.constraints.add(this);
            };
        }
        else match.cnt++;
    }
    public function remBody(b:ZPP_Body){
        {
            #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
            var res={
                b!=null;
            };
            if(!res)throw "assert("+"b!=null"+") :: "+("remBody :: null Userconstraint");
            #end
        };
        var match=null;
        var bl=Std.int(bodies.length);
        var i=0;
        while(i<bl){
            var x=bodies[i];
            if(x.body==b){
                x.cnt--;
                if(x.cnt==0){
                    if(bl>0)bodies[i]=bodies[bl-1];
                    bodies.pop();
                    if(active&&space!=null){
                        if(b!=null)b.constraints.remove(this);
                    };
                }
                match=x;
                break;
            }
            i++;
        }
        return match!=null;
    }
    public function bodyImpulse(b:ZPP_Body){
        for(i in 0...dim)J[i]=jAcc[i];
        var ret=Vec3.get(0,0,0);
        if(stepped)outer_zn.__impulse(J,b.outer,ret);
        return ret;
    }
    public override function activeBodies(){
        for(b in bodies){
            if(b.body!=null)b.body.constraints.add(this);
        };
    }
    public override function inactiveBodies(){
        for(b in bodies){
            if(b.body!=null)b.body.constraints.remove(this);
        };
    }
    public var stepped:Bool=false;
    public override function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Constraint{
        var ret=outer_zn.__copy();
        copyto(ret);
        throw "not done yet";
        return ret;
    }
    public function new(dim:Int,velonly:Bool){
        super();
        bodies={
            #if flash10 new flash.Vector<ZPP_UserBody>()#else new Array<ZPP_UserBody>()#end;
        };
        this.dim=dim;
        this.velonly=velonly;
        jAcc={
            #if flash10 new flash.Vector<Float>(dim,true)#else new Array<Float>()#end;
        };
        bias={
            #if flash10 new flash.Vector<Float>(dim,true)#else new Array<Float>()#end;
        };
        L={
            #if flash10 new flash.Vector<Float>(dim*dim,true)#else new Array<Float>()#end;
        };
        J={
            #if flash10 new flash.Vector<Float>(dim,true)#else new Array<Float>()#end;
        };
        jOld={
            #if flash10 new flash.Vector<Float>(dim,true)#else new Array<Float>()#end;
        };
        y={
            #if flash10 new flash.Vector<Float>(dim,true)#else new Array<Float>()#end;
        };
        Keff={
            #if flash10 new flash.Vector<Float>((dim*(dim+1))>>>1,true)#else new Array<Float>()#end;
        };
        vec3=Vec3.get(0,0,0);
        for(i in 0...dim){
            jAcc[i]=bias[i]=J[i]=jOld[i]=y[i]=0.0;
            for(j in 0...dim)L[i*dim+j]=0.0;
        }
        stepped=false;
    }
    public override function validate(){
        for(b in bodies)if(b.body.space!=space)throw "Error: Constraints must have each body within the same sapce to which the constraint has been assigned";
        outer_zn.__validate();
    }
    public override function wake_connected(){
        for(b in bodies){
            if(b.body.isDynamic())b.body.wake();
        }
    }
    public override function forest(){
        for(b in bodies){
            if(b.body.isDynamic()){
                var xr=({
                    if(b.body.component==b.body.component.parent)b.body.component;
                    else{
                        var obj=b.body.component;
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
    }
    public override function pair_exists(id:Int,di:Int){
        var ret=false;
        var bl=Std.int(bodies.length);
        for(bi in 0...bl){
            var b=bodies[bi].body;
            for(ci in bi+1...bl){
                var c=bodies[ci].body;
                if((b.id==id&&c.id==di)||(b.id==di&&c.id==id)){
                    ret=true;
                    break;
                }
            }
            if(ret)break;
        }
        return ret;
    }
    public override function broken(){
        outer_zn.__broken();
    }
    public override function clearcache(){
        for(i in 0...dim)jAcc[i]=0.0;
        pre_dt=-1.0;
    }
    public function lsq(v:TArray<Float>){
        var sum=0.0;
        for(i in 0...dim)sum+=v[i]*v[i];
        return sum;
    }
    public function _clamp(v:TArray<Float>,max:Float){
        var x=lsq(v);
        if(x>max*max){
            var scale=max/Math.sqrt(x);
            for(i in 0...dim)v[i]*=scale;
        }
    }
    var L:TArray<Float>=null;
    public function solve(m:TArray<Float>){
        var ind=0;
        for(j in 0...dim){
            var sum=0.0;
            for(k in 0...j-1)sum+=L[j*dim+k]*L[j*dim+k];
            var rec=Math.sqrt(m[ind++]-sum);
            L[j*dim+j]=rec;
            if(rec!=0){
                rec=1.0/rec;
                for(i in j+1...dim){
                    var sum=0.0;
                    for(k in 0...j-1)sum+=L[i*dim+k]*L[j*dim+k];
                    L[i*dim+j]=rec*(m[ind++]-sum);
                }
            }
            else{
                for(i in j+1...dim)L[i*dim+j]=0.0;
                ind+=dim-j-1;
            }
        }
        return L;
    }
    var y:TArray<Float>=null;
    public function transform(L:TArray<Float>,x:TArray<Float>){
        for(i in 0...dim){
            var sum=x[i];
            var lii=L[i*dim+i];
            if(lii!=0){
                for(k in 0...i)sum-=L[i*dim+k]*y[k];
                y[i]=sum/lii;
            }
            else y[i]=0.0;
        }
        for(ix in 0...dim){
            var i=dim-1-ix;
            var lii=L[i*dim+i];
            if(lii!=0){
                var sum=y[i];
                for(k in i+1...dim)sum-=L[k*dim+i]*x[k];
                x[i]=sum/lii;
            }
            else x[i]=0.0;
        }
    }
    public var soft:Float=0.0;
    public var gamma:Float=0.0;
    public var velonly:Bool=false;
    public var jMax:Float=0.0;
    public var Keff:TArray<Float>=null;
    public override function preStep(dt:Float){
        #if NAPE_RELEASE_BUILD 
        outer_zn.__validate();
        #end
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        stepped=true;
        outer_zn.__prepare();
        outer_zn.__eff_mass(Keff);
        L=solve(Keff);
        if(!stiff&&!velonly){
            var biasCoef;
            soft={
                var omega=2*Math.PI*frequency;
                gamma=1/(dt*omega*(2*damping+omega*dt));
                var ig=1/(1+gamma);
                biasCoef=dt*omega*omega*gamma;
                gamma*=ig;
                ig;
            };
            outer_zn.__position(bias);
            if(breakUnderError&&lsq(bias)>maxError*maxError)return true;
            for(i in 0...dim)bias[i]*=-biasCoef;
            _clamp(bias,maxError);
        }
        else{
            for(i in 0...dim)bias[i]=0.0;
            gamma=0.0;
            soft=1.0;
        }
        for(i in 0...dim)jAcc[i]*=dtratio;
        jMax=maxForce*dt;
        return false;
    }
    var vec3:Vec3=null;
    public override function warmStart(){
        for(bs in bodies){
            var b=bs.body;
            outer_zn.__impulse(jAcc,b.outer,vec3);
            {
                var t=(b.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b.vel"+",in b: "+"vec3."+",in s: "+"b.imass"+")");
                    #end
                };
                b.velx+=vec3.x*t;
                b.vely+=vec3.y*t;
            };
            b.angvel+=vec3.z*b.iinertia;
        }
    }
    var J:TArray<Float>=null;
    var jOld:TArray<Float>=null;
    public override function applyImpulseVel(){
        outer_zn.__velocity(J);
        for(i in 0...dim)J[i]=bias[i]-J[i];
        transform(L,J);
        for(i in 0...dim){
            jOld[i]=jAcc[i];
            jAcc[i]+=(J[i]=(J[i]*soft-jAcc[i]*gamma));
        }
        outer_zn.__clamp(jAcc);
        if((breakUnderForce||!stiff)&&lsq(jAcc)>jMax*jMax){
            if(breakUnderForce)return true;
            else if(!stiff)_clamp(jAcc,jMax);
        }
        for(i in 0...dim)J[i]=jAcc[i]-jOld[i];
        for(bs in bodies){
            var b=bs.body;
            outer_zn.__impulse(J,b.outer,vec3);
            {
                var t=(b.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b.vel"+",in b: "+"vec3."+",in s: "+"b.imass"+")");
                    #end
                };
                b.velx+=vec3.x*t;
                b.vely+=vec3.y*t;
            };
            b.angvel+=vec3.z*b.iinertia;
        }
        return false;
    }
    public override function applyImpulsePos(){
        if(velonly)return false;
        outer_zn.__prepare();
        outer_zn.__position(J);
        var lj=lsq(J);
        if(breakUnderError&&lj>maxError*maxError)return true;
        else if(lj<Config.constraintLinearSlop*Config.constraintLinearSlop)return false;
        for(i in 0...dim)J[i]*=-1;
        outer_zn.__eff_mass(Keff);
        transform(solve(Keff),J);
        outer_zn.__clamp(J);
        for(bs in bodies){
            var b=bs.body;
            outer_zn.__impulse(J,b.outer,vec3);
            {
                var t=(b.imass);
                {
                    #if(NAPE_ASSERT&&!NAPE_RELEASE_BUILD)
                    var res={
                        !((t!=t));
                    };
                    if(!res)throw "assert("+"!assert_isNaN(t)"+") :: "+("vec_addeq(in a: "+"b.pos"+",in b: "+"vec3."+",in s: "+"b.imass"+")");
                    #end
                };
                b.posx+=vec3.x*t;
                b.posy+=vec3.y*t;
            };
            b.delta_rot(vec3.z*b.iinertia);
        }
        return false;
    }
    public override function draw(g:Debug){
        outer_zn.__draw(g);
    }
}
#if nape_swc@:keep #end
class ZPP_UserBody{
    public var cnt:Int=0;
    public var body:ZPP_Body=null;
    public function new(cnt:Int,body:ZPP_Body){
        this.cnt=cnt;
        this.body=body;
    }
}

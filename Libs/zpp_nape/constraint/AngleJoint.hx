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
class ZPP_AngleJoint extends ZPP_Constraint{
    public var outer_zn:AngleJoint=null;
    public var ratio:Float=0.0;
    public var jointMin:Float=0.0;
    public var jointMax:Float=0.0;
    public var slack:Bool=false;
    public var equal:Bool=false;
    public var scale:Float=0.0;
    #if NAPE_NO_INLINE#elseif(flash9&&flib)@:ns("flibdel")#end
    public#if NAPE_NO_INLINE#else inline #end
    function is_slack(){
        var slack;
        {
            var C=ratio*b2.rot-b1.rot;
            if(equal){
                C-=jointMax;
                slack=false;
                scale=1.0;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    scale=-1.0;
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    scale=1.0;
                    slack=false;
                }
                else{
                    scale=0.0;
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        return slack;
    }
    public function bodyImpulse(b:ZPP_Body){
        if(stepped){
            if(b==b1)return Vec3.get(0,0,-scale*jAcc);
            else return Vec3.get(0,0,ratio*scale*jAcc);
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
    }
    public override function inactiveBodies(){
        {
            if(b1!=null)b1.constraints.remove(this);
        };
        if(b2!=b1){
            if(b2!=null)b2.constraints.remove(this);
        };
    }
    public var b1:ZPP_Body=null;
    public var b2:ZPP_Body=null;
    public var kMass:Float=0.0;
    public var jAcc:Float=0.0;
    public var jMax:Float=0.0;
    public var gamma:Float=0.0;
    public var bias:Float=0.0;
    public var stepped:Bool=false;
    public override function copy(dict:Array<ZPP_CopyHelper>=null,todo:Array<ZPP_CopyHelper>=null):Constraint{
        var ret=new AngleJoint(null,null,jointMin,jointMax,ratio);
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
        return ret;
    }
    public function new(){
        super();
        ratio=1;
        jAcc=0;
        slack=false;
        jMax=ZPP_Const.POSINF();
        stepped=false;
    }
    public override function validate(){
        if(b1==null||b2==null)throw "Error: AngleJoint cannot be simulated null bodies";
        if(b1==b2)throw "Error: AngleJoint cannot be simulated with body1 == body2";
        if(b1.space!=space||b2.space!=space)throw "Error: Constraints must have each body within the same space to which the constraint has been assigned";
        if(jointMin>jointMax)throw "Error: AngleJoint must have jointMin <= jointMax";
        if(!b1.isDynamic()&&!b2.isDynamic())throw "Error: Constraints cannot have both bodies non-dynamic";
    }
    public override function wake_connected(){
        if(b1!=null&&b1.isDynamic())b1.wake();
        if(b2!=null&&b2.isDynamic())b2.wake();
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
    }
    public override function pair_exists(id:Int,di:Int){
        return(b1.id==id&&b2.id==di)||(b1.id==di&&b2.id==id);
    }
    public override function clearcache(){
        jAcc=0;
        pre_dt=-1.0;
        slack=false;
    }
    public override function preStep(dt:Float){
        if(pre_dt==-1.0)pre_dt=dt;
        var dtratio=dt/pre_dt;
        pre_dt=dt;
        stepped=true;
        equal=jointMin==jointMax;
        var C={
            var C=ratio*b2.rot-b1.rot;
            if(equal){
                C-=jointMax;
                slack=false;
                scale=1.0;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    scale=-1.0;
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    scale=1.0;
                    slack=false;
                }
                else{
                    scale=0.0;
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        if(!slack){
            kMass={
                b1.sinertia+ratio*ratio*b2.sinertia;
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
                b1.delta_rot(-scale*jAcc*b1.iinertia);
                b2.delta_rot(ratio*scale*jAcc*b2.iinertia);
            }
            else{
                b1.angvel-=scale*b1.iinertia*jAcc;
                b2.angvel+=ratio*scale*b2.iinertia*jAcc;
            }
        };
    }
    public override function applyImpulseVel(){
        if(slack)return false;
        var E={
            scale*(ratio*(b2.angvel+b2.kinangvel)-b1.angvel-b1.kinangvel);
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
                if(breakUnderForce&&(jAcc>jMax||jAcc<-jMax))return true;
                if(!stiff){
                    if(jAcc>jMax)jAcc=jMax;
                    else if(jAcc<-jMax)jAcc=-jMax;
                }
            };
            j=jAcc-jOld;
        };
        {
            if(false){
                b1.delta_rot(-scale*j*b1.iinertia);
                b2.delta_rot(ratio*scale*j*b2.iinertia);
            }
            else{
                b1.angvel-=scale*b1.iinertia*j;
                b2.angvel+=ratio*scale*b2.iinertia*j;
            }
        };
        return false;
    }
    public override function applyImpulsePos(){
        var E;
        var j;
        var slack;
        E={
            var C=ratio*b2.rot-b1.rot;
            if(equal){
                C-=jointMax;
                slack=false;
                scale=1.0;
            }
            else{
                if(C<jointMin){
                    C=jointMin-C;
                    scale=-1.0;
                    slack=false;
                }
                else if(C>jointMax){
                    C-=jointMax;
                    scale=1.0;
                    slack=false;
                }
                else{
                    scale=0.0;
                    C=0;
                    slack=true;
                }
            }
            C;
        };
        if(!slack){
            if(breakUnderError&&E*E>maxError*maxError)return true;
            E*=0.5;
            j=-E*kMass;
            if(equal||j<0){
                if(true){
                    b1.delta_rot(-scale*j*b1.iinertia);
                    b2.delta_rot(ratio*scale*j*b2.iinertia);
                }
                else{
                    b1.angvel-=scale*b1.iinertia*j;
                    b2.angvel+=ratio*scale*b2.iinertia*j;
                }
            };
        }
        return false;
    }
    public override function draw(g:Debug){
        #if(flash9||openfl||nme)var me=outer_zn;
        var minrad=10;
        var delrad=5/Math.PI/2;
        if(me.body1!=me.body1.space.world){
            var min=(me.ratio*me.body2.rotation-jointMin);
            var max=(me.ratio*me.body2.rotation-jointMax);
            if(min>max){
                var t=min;
                min=max;
                max=t;
            };
            if(me.body1.rotation>min){
                var dr=({
                    var x=me.body1.rotation;
                    var y=max;
                    x<y?x:y;
                });
                ZPP_AngleDraw.drawSpiral(g,me.body1.position,min,dr,(minrad+(min-min)*delrad),(minrad+(dr-min)*delrad),0xffff00);
            }
            else if(!stiff&&me.body1.rotation<min){
                ZPP_AngleDraw.drawSpiralSpring(g,me.body1.position,me.body1.rotation,min,(minrad+(me.body1.rotation-min)*delrad),(minrad+(min-min)*delrad),0xffff00);
            }
            if(me.body1.rotation<max){
                var dr=({
                    var x=me.body1.rotation;
                    var y=min;
                    x>y?x:y;
                });
                ZPP_AngleDraw.drawSpiral(g,me.body1.position,dr,max,(minrad+(dr-min)*delrad),(minrad+(max-min)*delrad),0xffff);
            }
            else if(!stiff&&me.body1.rotation>max){
                ZPP_AngleDraw.drawSpiralSpring(g,me.body1.position,me.body1.rotation,max,(minrad+(me.body1.rotation-min)*delrad),(minrad+(max-min)*delrad),0xffff);
            }
            ZPP_AngleDraw.indicator(g,me.body1.position,me.body1.rotation,(minrad+(me.body1.rotation-min)*delrad),0xff);
        };
        if(me.body2!=me.body2.space.world){
            var min=((jointMin+me.body1.rotation)/me.ratio);
            var max=((jointMax+me.body1.rotation)/me.ratio);
            if(min>max){
                var t=min;
                min=max;
                max=t;
            };
            if(me.body2.rotation>min){
                var dr=({
                    var x=me.body2.rotation;
                    var y=max;
                    x<y?x:y;
                });
                ZPP_AngleDraw.drawSpiral(g,me.body2.position,min,dr,(minrad+(min-min)*delrad),(minrad+(dr-min)*delrad),0xffff00);
            }
            else if(!stiff&&me.body2.rotation<min){
                ZPP_AngleDraw.drawSpiralSpring(g,me.body2.position,me.body2.rotation,min,(minrad+(me.body2.rotation-min)*delrad),(minrad+(min-min)*delrad),0xffff00);
            }
            if(me.body2.rotation<max){
                var dr=({
                    var x=me.body2.rotation;
                    var y=min;
                    x>y?x:y;
                });
                ZPP_AngleDraw.drawSpiral(g,me.body2.position,dr,max,(minrad+(dr-min)*delrad),(minrad+(max-min)*delrad),0xffff);
            }
            else if(!stiff&&me.body2.rotation>max){
                ZPP_AngleDraw.drawSpiralSpring(g,me.body2.position,me.body2.rotation,max,(minrad+(me.body2.rotation-min)*delrad),(minrad+(max-min)*delrad),0xffff);
            }
            ZPP_AngleDraw.indicator(g,me.body2.position,me.body2.rotation,(minrad+(me.body2.rotation-min)*delrad),0xff0000);
        };
        #end
    }
}
#if(flash9||openfl||nme)#if nape_swc@:keep #end
class ZPP_AngleDraw{
    public static function indicator(g:Debug,c:Vec2,ang:Float,rad:Float,col:Int){
        var dir=Vec2.get(Math.cos(ang),Math.sin(ang));
        g.drawFilledCircle(c.add(dir.mul(rad,true),true),2,col);
        dir.dispose();
    }
    static var maxarc=Math.PI/4;
    public static function drawSpiralSpring(g:Debug,c:Vec2,a0:Float,a1:Float,r0:Float,r1:Float,col:Int,coils:Int=4){
        if(a0>a1){
            {
                var t=a0;
                a0=a1;
                a1=t;
            };
            {
                var t=r0;
                r0=r1;
                r1=t;
            };
        }
        if(a0==a1)return;
        var dr=r1-r0;
        var da=a1-a0;
        var Delta=({
            var x=2*Math.PI*dr/da;
            x<0?-x:x;
        });
        var dcnt=({
            var x=Math.ceil(da/maxarc*3);
            var y=4*coils;
            x>y?x:y;
        });
        var drad=dr/dcnt;
        var dang=da/dcnt;
        var dtime=1/dcnt;
        var c0=Math.cos(a0);
        var s0=Math.sin(a0);
        var R0=({
            var p=r0+dr*0;
            p+0.75*Delta*Math.sin(2*coils*Math.PI*0);
        });
        var p0=Vec2.get(c.x+R0*c0,c.y+R0*s0);
        var DR=({
            dr+1.5*coils*Delta*Math.PI*Math.cos(2*coils*Math.PI*0);
        });
        var ux=DR*c0-R0*da*s0;
        var uy=DR*s0+R0*da*c0;
        var p1=Vec2.get();
        var ct=Vec2.get();
        for(i in 0...dcnt){
            var a1=a0+dang;
            var c1=Math.cos(a1);
            var s1=Math.sin(a1);
            var R1=({
                var p=r0+dr*(i+1)*dtime;
                p+0.75*Delta*Math.sin(2*coils*Math.PI*(i+1)*dtime);
            });
            p1.setxy(c.x+R1*c1,c.y+R1*s1);
            var DR=({
                dr+1.5*coils*Delta*Math.PI*Math.cos(2*coils*Math.PI*(i+1)*dtime);
            });
            var vx=DR*c1-R1*da*s1;
            var vy=DR*s1+R1*da*c1;
            var den=(ux*vy-uy*vx);
            if(den*den<Config.epsilon||(ux*vx+uy*vy)<=0||(ux*vx+uy*vy)>0.999)g.drawLine(p0,p1,col);
            else{
                var t=((p1.x-p0.x)*vy+(p0.y-p1.y)*vx)/den;
                if(t<=0)g.drawLine(p0,p1,col);
                else{
                    ct.x=p0.x+ux*t;
                    ct.y=p0.y+uy*t;
                    g.drawCurve(p0,ct,p1,col);
                }
            }
            a0=a1;
            c0=c1;
            s0=s1;
            ux=vx;
            uy=vy;
            p0.set(p1);
        }
        p0.dispose();
        p1.dispose();
        ct.dispose();
    }
    public static function drawSpiral(g:Debug,c:Vec2,a0:Float,a1:Float,r0:Float,r1:Float,col:Int){
        if(a0>a1){
            {
                var t=a0;
                a0=a1;
                a1=t;
            };
            {
                var t=r0;
                r0=r1;
                r1=t;
            };
        }
        if(a0==a1)return;
        var dr=r1-r0;
        var da=a1-a0;
        var dcnt=Math.ceil(da/maxarc);
        var drad=dr/dcnt;
        var dang=da/dcnt;
        var c0=Math.cos(a0);
        var s0=Math.sin(a0);
        var p0=Vec2.get(c.x+r0*c0,c.y+r0*s0);
        var ux=dr*c0-r0*da*s0;
        var uy=dr*s0+r0*da*c0;
        var p1=Vec2.get();
        var ct=Vec2.get();
        for(i in 0...dcnt){
            var r1=r0+drad;
            var a1=a0+dang;
            var c1=Math.cos(a1);
            var s1=Math.sin(a1);
            p1.setxy(c.x+r1*c1,c.y+r1*s1);
            var vx=dr*c1-r1*da*s1;
            var vy=dr*s1+r1*da*c1;
            var den=(ux*vy-uy*vx);
            if(den*den<Config.epsilon)g.drawLine(p0,p1,col);
            else{
                var t=((p1.x-p0.x)*vy+(p0.y-p1.y)*vx)/den;
                if(t<=0)g.drawLine(p0,p1,col);
                else{
                    ct.x=p0.x+ux*t;
                    ct.y=p0.y+uy*t;
                    g.drawCurve(p0,ct,p1,col);
                }
            }
            r0=r1;
            a0=a1;
            c0=c1;
            s0=s1;
            ux=vx;
            uy=vy;
            p0.set(p1);
        }
        p0.dispose();
        p1.dispose();
        ct.dispose();
    }
}
#end

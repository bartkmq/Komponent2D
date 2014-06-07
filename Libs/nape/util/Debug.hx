package nape.util;
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
import nape.util.BitmapDebug;
import nape.util.ShapeDebug;
/**
 * Debug class providing general utilities
 * <br/><br/>
 * Also serves as the base type for Debug draws.
 */
#if nape_swc@:keep #end
class Debug{
    /**
     * Query Nape version
     */
    public static function version(){
        return "Nape 2.0.16";
    }
    /**
     * Force clear all object pools, both internal and public.
     */
    public static function clearObjectPools(){
        
        {
            while(ConstraintIterator.zpp_pool!=null){
                var nxt=ConstraintIterator.zpp_pool.zpp_next;
                ConstraintIterator.zpp_pool.zpp_next=null;
                ConstraintIterator.zpp_pool=nxt;
            }
        }
        {
            while(InteractorIterator.zpp_pool!=null){
                var nxt=InteractorIterator.zpp_pool.zpp_next;
                InteractorIterator.zpp_pool.zpp_next=null;
                InteractorIterator.zpp_pool=nxt;
            }
        }
        {
            while(BodyIterator.zpp_pool!=null){
                var nxt=BodyIterator.zpp_pool.zpp_next;
                BodyIterator.zpp_pool.zpp_next=null;
                BodyIterator.zpp_pool=nxt;
            }
        }
        {
            while(CompoundIterator.zpp_pool!=null){
                var nxt=CompoundIterator.zpp_pool.zpp_next;
                CompoundIterator.zpp_pool.zpp_next=null;
                CompoundIterator.zpp_pool=nxt;
            }
        }
        {
            while(ListenerIterator.zpp_pool!=null){
                var nxt=ListenerIterator.zpp_pool.zpp_next;
                ListenerIterator.zpp_pool.zpp_next=null;
                ListenerIterator.zpp_pool=nxt;
            }
        }
        {
            while(CbTypeIterator.zpp_pool!=null){
                var nxt=CbTypeIterator.zpp_pool.zpp_next;
                CbTypeIterator.zpp_pool.zpp_next=null;
                CbTypeIterator.zpp_pool=nxt;
            }
        }
        {
            while(ConvexResultIterator.zpp_pool!=null){
                var nxt=ConvexResultIterator.zpp_pool.zpp_next;
                ConvexResultIterator.zpp_pool.zpp_next=null;
                ConvexResultIterator.zpp_pool=nxt;
            }
        }
        {
            while(GeomPolyIterator.zpp_pool!=null){
                var nxt=GeomPolyIterator.zpp_pool.zpp_next;
                GeomPolyIterator.zpp_pool.zpp_next=null;
                GeomPolyIterator.zpp_pool=nxt;
            }
        }
        {
            while(Vec2Iterator.zpp_pool!=null){
                var nxt=Vec2Iterator.zpp_pool.zpp_next;
                Vec2Iterator.zpp_pool.zpp_next=null;
                Vec2Iterator.zpp_pool=nxt;
            }
        }
        {
            while(RayResultIterator.zpp_pool!=null){
                var nxt=RayResultIterator.zpp_pool.zpp_next;
                RayResultIterator.zpp_pool.zpp_next=null;
                RayResultIterator.zpp_pool=nxt;
            }
        }
        {
            while(ShapeIterator.zpp_pool!=null){
                var nxt=ShapeIterator.zpp_pool.zpp_next;
                ShapeIterator.zpp_pool.zpp_next=null;
                ShapeIterator.zpp_pool=nxt;
            }
        }
        {
            while(EdgeIterator.zpp_pool!=null){
                var nxt=EdgeIterator.zpp_pool.zpp_next;
                EdgeIterator.zpp_pool.zpp_next=null;
                EdgeIterator.zpp_pool=nxt;
            }
        }
        {
            while(ContactIterator.zpp_pool!=null){
                var nxt=ContactIterator.zpp_pool.zpp_next;
                ContactIterator.zpp_pool.zpp_next=null;
                ContactIterator.zpp_pool=nxt;
            }
        }
        {
            while(ArbiterIterator.zpp_pool!=null){
                var nxt=ArbiterIterator.zpp_pool.zpp_next;
                ArbiterIterator.zpp_pool.zpp_next=null;
                ArbiterIterator.zpp_pool=nxt;
            }
        }
        {
            while(InteractionGroupIterator.zpp_pool!=null){
                var nxt=InteractionGroupIterator.zpp_pool.zpp_next;
                InteractionGroupIterator.zpp_pool.zpp_next=null;
                InteractionGroupIterator.zpp_pool=nxt;
            }
        }
        
        {
            while(ZNPNode_ZPP_CbType.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CbType.zpp_pool.next;
                ZNPNode_ZPP_CbType.zpp_pool.next=null;
                ZNPNode_ZPP_CbType.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbType.POOL_TOT=ZNPNode_ZPP_CbType.POOL_CNT=ZNPNode_ZPP_CbType.POOL_ADDNEW=ZNPNode_ZPP_CbType.POOL_ADD=ZNPNode_ZPP_CbType.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_CallbackSet.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CallbackSet.zpp_pool.next;
                ZNPNode_ZPP_CallbackSet.zpp_pool.next=null;
                ZNPNode_ZPP_CallbackSet.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CallbackSet.POOL_TOT=ZNPNode_ZPP_CallbackSet.POOL_CNT=ZNPNode_ZPP_CallbackSet.POOL_ADDNEW=ZNPNode_ZPP_CallbackSet.POOL_ADD=ZNPNode_ZPP_CallbackSet.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Material.zpp_pool!=null){
                var nxt=ZPP_Material.zpp_pool.next;
                ZPP_Material.zpp_pool.next=null;
                ZPP_Material.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Material.POOL_TOT=ZPP_Material.POOL_CNT=ZPP_Material.POOL_ADDNEW=ZPP_Material.POOL_ADD=ZPP_Material.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Shape.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Shape.zpp_pool.next;
                ZNPNode_ZPP_Shape.zpp_pool.next=null;
                ZNPNode_ZPP_Shape.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Shape.POOL_TOT=ZNPNode_ZPP_Shape.POOL_CNT=ZNPNode_ZPP_Shape.POOL_ADDNEW=ZNPNode_ZPP_Shape.POOL_ADD=ZNPNode_ZPP_Shape.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_FluidProperties.zpp_pool!=null){
                var nxt=ZPP_FluidProperties.zpp_pool.next;
                ZPP_FluidProperties.zpp_pool.next=null;
                ZPP_FluidProperties.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_FluidProperties.POOL_TOT=ZPP_FluidProperties.POOL_CNT=ZPP_FluidProperties.POOL_ADDNEW=ZPP_FluidProperties.POOL_ADD=ZPP_FluidProperties.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Body.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Body.zpp_pool.next;
                ZNPNode_ZPP_Body.zpp_pool.next=null;
                ZNPNode_ZPP_Body.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Body.POOL_TOT=ZNPNode_ZPP_Body.POOL_CNT=ZNPNode_ZPP_Body.POOL_ADDNEW=ZNPNode_ZPP_Body.POOL_ADD=ZNPNode_ZPP_Body.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Constraint.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Constraint.zpp_pool.next;
                ZNPNode_ZPP_Constraint.zpp_pool.next=null;
                ZNPNode_ZPP_Constraint.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Constraint.POOL_TOT=ZNPNode_ZPP_Constraint.POOL_CNT=ZNPNode_ZPP_Constraint.POOL_ADDNEW=ZNPNode_ZPP_Constraint.POOL_ADD=ZNPNode_ZPP_Constraint.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Compound.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Compound.zpp_pool.next;
                ZNPNode_ZPP_Compound.zpp_pool.next=null;
                ZNPNode_ZPP_Compound.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Compound.POOL_TOT=ZNPNode_ZPP_Compound.POOL_CNT=ZNPNode_ZPP_Compound.POOL_ADDNEW=ZNPNode_ZPP_Compound.POOL_ADD=ZNPNode_ZPP_Compound.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Arbiter.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Arbiter.zpp_pool.next;
                ZNPNode_ZPP_Arbiter.zpp_pool.next=null;
                ZNPNode_ZPP_Arbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Arbiter.POOL_TOT=ZNPNode_ZPP_Arbiter.POOL_CNT=ZNPNode_ZPP_Arbiter.POOL_ADDNEW=ZNPNode_ZPP_Arbiter.POOL_ADD=ZNPNode_ZPP_Arbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_Body.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_Body.zpp_pool.next;
                ZPP_Set_ZPP_Body.zpp_pool.next=null;
                ZPP_Set_ZPP_Body.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_Body.POOL_TOT=ZPP_Set_ZPP_Body.POOL_CNT=ZPP_Set_ZPP_Body.POOL_ADDNEW=ZPP_Set_ZPP_Body.POOL_ADD=ZPP_Set_ZPP_Body.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_CbSetPair.zpp_pool!=null){
                var nxt=ZPP_CbSetPair.zpp_pool.next;
                ZPP_CbSetPair.zpp_pool.next=null;
                ZPP_CbSetPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_CbSetPair.POOL_TOT=ZPP_CbSetPair.POOL_CNT=ZPP_CbSetPair.POOL_ADDNEW=ZPP_CbSetPair.POOL_ADD=ZPP_CbSetPair.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_InteractionListener.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_InteractionListener.zpp_pool.next;
                ZNPNode_ZPP_InteractionListener.zpp_pool.next=null;
                ZNPNode_ZPP_InteractionListener.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionListener.POOL_TOT=ZNPNode_ZPP_InteractionListener.POOL_CNT=ZNPNode_ZPP_InteractionListener.POOL_ADDNEW=ZNPNode_ZPP_InteractionListener.POOL_ADD=ZNPNode_ZPP_InteractionListener.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_CbSet.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CbSet.zpp_pool.next;
                ZNPNode_ZPP_CbSet.zpp_pool.next=null;
                ZNPNode_ZPP_CbSet.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSet.POOL_TOT=ZNPNode_ZPP_CbSet.POOL_CNT=ZNPNode_ZPP_CbSet.POOL_ADDNEW=ZNPNode_ZPP_CbSet.POOL_ADD=ZNPNode_ZPP_CbSet.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Interactor.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Interactor.zpp_pool.next;
                ZNPNode_ZPP_Interactor.zpp_pool.next=null;
                ZNPNode_ZPP_Interactor.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Interactor.POOL_TOT=ZNPNode_ZPP_Interactor.POOL_CNT=ZNPNode_ZPP_Interactor.POOL_ADDNEW=ZNPNode_ZPP_Interactor.POOL_ADD=ZNPNode_ZPP_Interactor.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_BodyListener.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_BodyListener.zpp_pool.next;
                ZNPNode_ZPP_BodyListener.zpp_pool.next=null;
                ZNPNode_ZPP_BodyListener.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_BodyListener.POOL_TOT=ZNPNode_ZPP_BodyListener.POOL_CNT=ZNPNode_ZPP_BodyListener.POOL_ADDNEW=ZNPNode_ZPP_BodyListener.POOL_ADD=ZNPNode_ZPP_BodyListener.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Callback.zpp_pool!=null){
                var nxt=ZPP_Callback.zpp_pool.next;
                ZPP_Callback.zpp_pool.next=null;
                ZPP_Callback.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Callback.POOL_TOT=ZPP_Callback.POOL_CNT=ZPP_Callback.POOL_ADDNEW=ZPP_Callback.POOL_ADD=ZPP_Callback.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_CbSet.zpp_pool!=null){
                var nxt=ZPP_CbSet.zpp_pool.next;
                ZPP_CbSet.zpp_pool.next=null;
                ZPP_CbSet.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_CbSet.POOL_TOT=ZPP_CbSet.POOL_CNT=ZPP_CbSet.POOL_ADDNEW=ZPP_CbSet.POOL_ADD=ZPP_CbSet.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_CbSetPair.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CbSetPair.zpp_pool.next;
                ZNPNode_ZPP_CbSetPair.zpp_pool.next=null;
                ZNPNode_ZPP_CbSetPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CbSetPair.POOL_TOT=ZNPNode_ZPP_CbSetPair.POOL_CNT=ZNPNode_ZPP_CbSetPair.POOL_ADDNEW=ZNPNode_ZPP_CbSetPair.POOL_ADD=ZNPNode_ZPP_CbSetPair.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_ConstraintListener.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_ConstraintListener.zpp_pool.next;
                ZNPNode_ZPP_ConstraintListener.zpp_pool.next=null;
                ZNPNode_ZPP_ConstraintListener.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_ConstraintListener.POOL_TOT=ZNPNode_ZPP_ConstraintListener.POOL_CNT=ZNPNode_ZPP_ConstraintListener.POOL_ADDNEW=ZNPNode_ZPP_ConstraintListener.POOL_ADD=ZNPNode_ZPP_ConstraintListener.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_GeomVert.zpp_pool!=null){
                var nxt=ZPP_GeomVert.zpp_pool.next;
                ZPP_GeomVert.zpp_pool.next=null;
                ZPP_GeomVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_GeomVert.POOL_TOT=ZPP_GeomVert.POOL_CNT=ZPP_GeomVert.POOL_ADDNEW=ZPP_GeomVert.POOL_ADD=ZPP_GeomVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_CbSetPair.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_CbSetPair.zpp_pool.next;
                ZPP_Set_ZPP_CbSetPair.zpp_pool.next=null;
                ZPP_Set_ZPP_CbSetPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_CbSetPair.POOL_TOT=ZPP_Set_ZPP_CbSetPair.POOL_CNT=ZPP_Set_ZPP_CbSetPair.POOL_ADDNEW=ZPP_Set_ZPP_CbSetPair.POOL_ADD=ZPP_Set_ZPP_CbSetPair.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_GeomVertexIterator.zpp_pool!=null){
                var nxt=ZPP_GeomVertexIterator.zpp_pool.next;
                ZPP_GeomVertexIterator.zpp_pool.next=null;
                ZPP_GeomVertexIterator.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_GeomVertexIterator.POOL_TOT=ZPP_GeomVertexIterator.POOL_CNT=ZPP_GeomVertexIterator.POOL_ADDNEW=ZPP_GeomVertexIterator.POOL_ADD=ZPP_GeomVertexIterator.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Mat23.zpp_pool!=null){
                var nxt=ZPP_Mat23.zpp_pool.next;
                ZPP_Mat23.zpp_pool.next=null;
                ZPP_Mat23.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Mat23.POOL_TOT=ZPP_Mat23.POOL_CNT=ZPP_Mat23.POOL_ADDNEW=ZPP_Mat23.POOL_ADD=ZPP_Mat23.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_CutVert.zpp_pool!=null){
                var nxt=ZPP_CutVert.zpp_pool.next;
                ZPP_CutVert.zpp_pool.next=null;
                ZPP_CutVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_CutVert.POOL_TOT=ZPP_CutVert.POOL_CNT=ZPP_CutVert.POOL_ADDNEW=ZPP_CutVert.POOL_ADD=ZPP_CutVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_CutInt.zpp_pool!=null){
                var nxt=ZPP_CutInt.zpp_pool.next;
                ZPP_CutInt.zpp_pool.next=null;
                ZPP_CutInt.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_CutInt.POOL_TOT=ZPP_CutInt.POOL_CNT=ZPP_CutInt.POOL_ADDNEW=ZPP_CutInt.POOL_ADD=ZPP_CutInt.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_CutInt.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CutInt.zpp_pool.next;
                ZNPNode_ZPP_CutInt.zpp_pool.next=null;
                ZNPNode_ZPP_CutInt.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutInt.POOL_TOT=ZNPNode_ZPP_CutInt.POOL_CNT=ZNPNode_ZPP_CutInt.POOL_ADDNEW=ZNPNode_ZPP_CutInt.POOL_ADD=ZNPNode_ZPP_CutInt.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_CutVert.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_CutVert.zpp_pool.next;
                ZNPNode_ZPP_CutVert.zpp_pool.next=null;
                ZNPNode_ZPP_CutVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_CutVert.POOL_TOT=ZNPNode_ZPP_CutVert.POOL_CNT=ZNPNode_ZPP_CutVert.POOL_ADDNEW=ZNPNode_ZPP_CutVert.POOL_ADD=ZNPNode_ZPP_CutVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Vec2.zpp_pool!=null){
                var nxt=ZPP_Vec2.zpp_pool.next;
                ZPP_Vec2.zpp_pool.next=null;
                ZPP_Vec2.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Vec2.POOL_TOT=ZPP_Vec2.POOL_CNT=ZPP_Vec2.POOL_ADDNEW=ZPP_Vec2.POOL_ADD=ZPP_Vec2.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_PartitionPair.zpp_pool!=null){
                var nxt=ZPP_PartitionPair.zpp_pool.next;
                ZPP_PartitionPair.zpp_pool.next=null;
                ZPP_PartitionPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_PartitionPair.POOL_TOT=ZPP_PartitionPair.POOL_CNT=ZPP_PartitionPair.POOL_ADDNEW=ZPP_PartitionPair.POOL_ADD=ZPP_PartitionPair.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_PartitionPair.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_PartitionPair.zpp_pool.next;
                ZPP_Set_ZPP_PartitionPair.zpp_pool.next=null;
                ZPP_Set_ZPP_PartitionPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionPair.POOL_TOT=ZPP_Set_ZPP_PartitionPair.POOL_CNT=ZPP_Set_ZPP_PartitionPair.POOL_ADDNEW=ZPP_Set_ZPP_PartitionPair.POOL_ADD=ZPP_Set_ZPP_PartitionPair.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_PartitionVertex.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_PartitionVertex.zpp_pool.next;
                ZNPNode_ZPP_PartitionVertex.zpp_pool.next=null;
                ZNPNode_ZPP_PartitionVertex.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionVertex.POOL_TOT=ZNPNode_ZPP_PartitionVertex.POOL_CNT=ZNPNode_ZPP_PartitionVertex.POOL_ADDNEW=ZNPNode_ZPP_PartitionVertex.POOL_ADD=ZNPNode_ZPP_PartitionVertex.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_PartitionVertex.zpp_pool!=null){
                var nxt=ZPP_PartitionVertex.zpp_pool.next;
                ZPP_PartitionVertex.zpp_pool.next=null;
                ZPP_PartitionVertex.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_PartitionVertex.POOL_TOT=ZPP_PartitionVertex.POOL_CNT=ZPP_PartitionVertex.POOL_ADDNEW=ZPP_PartitionVertex.POOL_ADD=ZPP_PartitionVertex.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_PartitionVertex.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_PartitionVertex.zpp_pool.next;
                ZPP_Set_ZPP_PartitionVertex.zpp_pool.next=null;
                ZPP_Set_ZPP_PartitionVertex.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_PartitionVertex.POOL_TOT=ZPP_Set_ZPP_PartitionVertex.POOL_CNT=ZPP_Set_ZPP_PartitionVertex.POOL_ADDNEW=ZPP_Set_ZPP_PartitionVertex.POOL_ADD=ZPP_Set_ZPP_PartitionVertex.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_PartitionedPoly.zpp_pool!=null){
                var nxt=ZPP_PartitionedPoly.zpp_pool.next;
                ZPP_PartitionedPoly.zpp_pool.next=null;
                ZPP_PartitionedPoly.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_PartitionedPoly.POOL_TOT=ZPP_PartitionedPoly.POOL_CNT=ZPP_PartitionedPoly.POOL_ADDNEW=ZPP_PartitionedPoly.POOL_ADD=ZPP_PartitionedPoly.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SimplifyV.zpp_pool!=null){
                var nxt=ZPP_SimplifyV.zpp_pool.next;
                ZPP_SimplifyV.zpp_pool.next=null;
                ZPP_SimplifyV.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SimplifyV.POOL_TOT=ZPP_SimplifyV.POOL_CNT=ZPP_SimplifyV.POOL_ADDNEW=ZPP_SimplifyV.POOL_ADD=ZPP_SimplifyV.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SimplifyP.zpp_pool!=null){
                var nxt=ZPP_SimplifyP.zpp_pool.next;
                ZPP_SimplifyP.zpp_pool.next=null;
                ZPP_SimplifyP.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SimplifyP.POOL_TOT=ZPP_SimplifyP.POOL_CNT=ZPP_SimplifyP.POOL_ADDNEW=ZPP_SimplifyP.POOL_ADD=ZPP_SimplifyP.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_PartitionedPoly.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_PartitionedPoly.zpp_pool.next;
                ZNPNode_ZPP_PartitionedPoly.zpp_pool.next=null;
                ZNPNode_ZPP_PartitionedPoly.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_PartitionedPoly.POOL_TOT=ZNPNode_ZPP_PartitionedPoly.POOL_CNT=ZNPNode_ZPP_PartitionedPoly.POOL_ADDNEW=ZNPNode_ZPP_PartitionedPoly.POOL_ADD=ZNPNode_ZPP_PartitionedPoly.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_SimplifyP.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_SimplifyP.zpp_pool.next;
                ZNPNode_ZPP_SimplifyP.zpp_pool.next=null;
                ZNPNode_ZPP_SimplifyP.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimplifyP.POOL_TOT=ZNPNode_ZPP_SimplifyP.POOL_CNT=ZNPNode_ZPP_SimplifyP.POOL_ADDNEW=ZNPNode_ZPP_SimplifyP.POOL_ADD=ZNPNode_ZPP_SimplifyP.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_AABB.zpp_pool!=null){
                var nxt=ZPP_AABB.zpp_pool.next;
                ZPP_AABB.zpp_pool.next=null;
                ZPP_AABB.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_AABB.POOL_TOT=ZPP_AABB.POOL_CNT=ZPP_AABB.POOL_ADDNEW=ZPP_AABB.POOL_ADD=ZPP_AABB.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_GeomVert.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_GeomVert.zpp_pool.next;
                ZNPNode_ZPP_GeomVert.zpp_pool.next=null;
                ZNPNode_ZPP_GeomVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomVert.POOL_TOT=ZNPNode_ZPP_GeomVert.POOL_CNT=ZNPNode_ZPP_GeomVert.POOL_ADDNEW=ZNPNode_ZPP_GeomVert.POOL_ADD=ZNPNode_ZPP_GeomVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_ToiEvent.zpp_pool!=null){
                var nxt=ZPP_ToiEvent.zpp_pool.next;
                ZPP_ToiEvent.zpp_pool.next=null;
                ZPP_ToiEvent.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_ToiEvent.POOL_TOT=ZPP_ToiEvent.POOL_CNT=ZPP_ToiEvent.POOL_ADDNEW=ZPP_ToiEvent.POOL_ADD=ZPP_ToiEvent.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_SimpleVert.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_SimpleVert.zpp_pool.next;
                ZPP_Set_ZPP_SimpleVert.zpp_pool.next=null;
                ZPP_Set_ZPP_SimpleVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleVert.POOL_TOT=ZPP_Set_ZPP_SimpleVert.POOL_CNT=ZPP_Set_ZPP_SimpleVert.POOL_ADDNEW=ZPP_Set_ZPP_SimpleVert.POOL_ADD=ZPP_Set_ZPP_SimpleVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SimpleVert.zpp_pool!=null){
                var nxt=ZPP_SimpleVert.zpp_pool.next;
                ZPP_SimpleVert.zpp_pool.next=null;
                ZPP_SimpleVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SimpleVert.POOL_TOT=ZPP_SimpleVert.POOL_CNT=ZPP_SimpleVert.POOL_ADDNEW=ZPP_SimpleVert.POOL_ADD=ZPP_SimpleVert.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SimpleSeg.zpp_pool!=null){
                var nxt=ZPP_SimpleSeg.zpp_pool.next;
                ZPP_SimpleSeg.zpp_pool.next=null;
                ZPP_SimpleSeg.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SimpleSeg.POOL_TOT=ZPP_SimpleSeg.POOL_CNT=ZPP_SimpleSeg.POOL_ADDNEW=ZPP_SimpleSeg.POOL_ADD=ZPP_SimpleSeg.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_SimpleSeg.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_SimpleSeg.zpp_pool.next;
                ZPP_Set_ZPP_SimpleSeg.zpp_pool.next=null;
                ZPP_Set_ZPP_SimpleSeg.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleSeg.POOL_TOT=ZPP_Set_ZPP_SimpleSeg.POOL_CNT=ZPP_Set_ZPP_SimpleSeg.POOL_ADDNEW=ZPP_Set_ZPP_SimpleSeg.POOL_ADD=ZPP_Set_ZPP_SimpleSeg.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_SimpleEvent.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_SimpleEvent.zpp_pool.next;
                ZPP_Set_ZPP_SimpleEvent.zpp_pool.next=null;
                ZPP_Set_ZPP_SimpleEvent.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_SimpleEvent.POOL_TOT=ZPP_Set_ZPP_SimpleEvent.POOL_CNT=ZPP_Set_ZPP_SimpleEvent.POOL_ADDNEW=ZPP_Set_ZPP_SimpleEvent.POOL_ADD=ZPP_Set_ZPP_SimpleEvent.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SimpleEvent.zpp_pool!=null){
                var nxt=ZPP_SimpleEvent.zpp_pool.next;
                ZPP_SimpleEvent.zpp_pool.next=null;
                ZPP_SimpleEvent.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SimpleEvent.POOL_TOT=ZPP_SimpleEvent.POOL_CNT=ZPP_SimpleEvent.POOL_ADDNEW=ZPP_SimpleEvent.POOL_ADD=ZPP_SimpleEvent.POOL_SUB=0;
            #end
        }
        {
            while(Hashable2_Boolfalse.zpp_pool!=null){
                var nxt=Hashable2_Boolfalse.zpp_pool.next;
                Hashable2_Boolfalse.zpp_pool.next=null;
                Hashable2_Boolfalse.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS Hashable2_Boolfalse.POOL_TOT=Hashable2_Boolfalse.POOL_CNT=Hashable2_Boolfalse.POOL_ADDNEW=Hashable2_Boolfalse.POOL_ADD=Hashable2_Boolfalse.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_MarchSpan.zpp_pool!=null){
                var nxt=ZPP_MarchSpan.zpp_pool.next;
                ZPP_MarchSpan.zpp_pool.next=null;
                ZPP_MarchSpan.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_MarchSpan.POOL_TOT=ZPP_MarchSpan.POOL_CNT=ZPP_MarchSpan.POOL_ADDNEW=ZPP_MarchSpan.POOL_ADD=ZPP_MarchSpan.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_MarchPair.zpp_pool!=null){
                var nxt=ZPP_MarchPair.zpp_pool.next;
                ZPP_MarchPair.zpp_pool.next=null;
                ZPP_MarchPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_MarchPair.POOL_TOT=ZPP_MarchPair.POOL_CNT=ZPP_MarchPair.POOL_ADDNEW=ZPP_MarchPair.POOL_ADD=ZPP_MarchPair.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_SimpleVert.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_SimpleVert.zpp_pool.next;
                ZNPNode_ZPP_SimpleVert.zpp_pool.next=null;
                ZNPNode_ZPP_SimpleVert.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleVert.POOL_TOT=ZNPNode_ZPP_SimpleVert.POOL_CNT=ZNPNode_ZPP_SimpleVert.POOL_ADDNEW=ZNPNode_ZPP_SimpleVert.POOL_ADD=ZNPNode_ZPP_SimpleVert.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_SimpleEvent.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_SimpleEvent.zpp_pool.next;
                ZNPNode_ZPP_SimpleEvent.zpp_pool.next=null;
                ZNPNode_ZPP_SimpleEvent.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_SimpleEvent.POOL_TOT=ZNPNode_ZPP_SimpleEvent.POOL_CNT=ZNPNode_ZPP_SimpleEvent.POOL_ADDNEW=ZNPNode_ZPP_SimpleEvent.POOL_ADD=ZNPNode_ZPP_SimpleEvent.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_AABBPair.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_AABBPair.zpp_pool.next;
                ZNPNode_ZPP_AABBPair.zpp_pool.next=null;
                ZNPNode_ZPP_AABBPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBPair.POOL_TOT=ZNPNode_ZPP_AABBPair.POOL_CNT=ZNPNode_ZPP_AABBPair.POOL_ADDNEW=ZNPNode_ZPP_AABBPair.POOL_ADD=ZNPNode_ZPP_AABBPair.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Edge.zpp_pool!=null){
                var nxt=ZPP_Edge.zpp_pool.next;
                ZPP_Edge.zpp_pool.next=null;
                ZPP_Edge.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Edge.POOL_TOT=ZPP_Edge.POOL_CNT=ZPP_Edge.POOL_ADDNEW=ZPP_Edge.POOL_ADD=ZPP_Edge.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Vec2.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Vec2.zpp_pool.next;
                ZNPNode_ZPP_Vec2.zpp_pool.next=null;
                ZNPNode_ZPP_Vec2.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Vec2.POOL_TOT=ZNPNode_ZPP_Vec2.POOL_CNT=ZNPNode_ZPP_Vec2.POOL_ADDNEW=ZNPNode_ZPP_Vec2.POOL_ADD=ZNPNode_ZPP_Vec2.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Edge.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Edge.zpp_pool.next;
                ZNPNode_ZPP_Edge.zpp_pool.next=null;
                ZNPNode_ZPP_Edge.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Edge.POOL_TOT=ZNPNode_ZPP_Edge.POOL_CNT=ZNPNode_ZPP_Edge.POOL_ADDNEW=ZNPNode_ZPP_Edge.POOL_ADD=ZNPNode_ZPP_Edge.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SweepData.zpp_pool!=null){
                var nxt=ZPP_SweepData.zpp_pool.next;
                ZPP_SweepData.zpp_pool.next=null;
                ZPP_SweepData.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SweepData.POOL_TOT=ZPP_SweepData.POOL_CNT=ZPP_SweepData.POOL_ADDNEW=ZPP_SweepData.POOL_ADD=ZPP_SweepData.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_AABBNode.zpp_pool!=null){
                var nxt=ZPP_AABBNode.zpp_pool.next;
                ZPP_AABBNode.zpp_pool.next=null;
                ZPP_AABBNode.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_AABBNode.POOL_TOT=ZPP_AABBNode.POOL_CNT=ZPP_AABBNode.POOL_ADDNEW=ZPP_AABBNode.POOL_ADD=ZPP_AABBNode.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_AABBPair.zpp_pool!=null){
                var nxt=ZPP_AABBPair.zpp_pool.next;
                ZPP_AABBPair.zpp_pool.next=null;
                ZPP_AABBPair.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_AABBPair.POOL_TOT=ZPP_AABBPair.POOL_CNT=ZPP_AABBPair.POOL_ADDNEW=ZPP_AABBPair.POOL_ADD=ZPP_AABBPair.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_AABBNode.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_AABBNode.zpp_pool.next;
                ZNPNode_ZPP_AABBNode.zpp_pool.next=null;
                ZNPNode_ZPP_AABBNode.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_AABBNode.POOL_TOT=ZNPNode_ZPP_AABBNode.POOL_CNT=ZNPNode_ZPP_AABBNode.POOL_ADDNEW=ZNPNode_ZPP_AABBNode.POOL_ADD=ZNPNode_ZPP_AABBNode.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Contact.zpp_pool!=null){
                var nxt=ZPP_Contact.zpp_pool.next;
                ZPP_Contact.zpp_pool.next=null;
                ZPP_Contact.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Contact.POOL_TOT=ZPP_Contact.POOL_CNT=ZPP_Contact.POOL_ADDNEW=ZPP_Contact.POOL_ADD=ZPP_Contact.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Component.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Component.zpp_pool.next;
                ZNPNode_ZPP_Component.zpp_pool.next=null;
                ZNPNode_ZPP_Component.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Component.POOL_TOT=ZNPNode_ZPP_Component.POOL_CNT=ZNPNode_ZPP_Component.POOL_ADDNEW=ZNPNode_ZPP_Component.POOL_ADD=ZNPNode_ZPP_Component.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Island.zpp_pool!=null){
                var nxt=ZPP_Island.zpp_pool.next;
                ZPP_Island.zpp_pool.next=null;
                ZPP_Island.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Island.POOL_TOT=ZPP_Island.POOL_CNT=ZPP_Island.POOL_ADDNEW=ZPP_Island.POOL_ADD=ZPP_Island.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Component.zpp_pool!=null){
                var nxt=ZPP_Component.zpp_pool.next;
                ZPP_Component.zpp_pool.next=null;
                ZPP_Component.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Component.POOL_TOT=ZPP_Component.POOL_CNT=ZPP_Component.POOL_ADDNEW=ZPP_Component.POOL_ADD=ZPP_Component.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_InteractionGroup.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_InteractionGroup.zpp_pool.next;
                ZNPNode_ZPP_InteractionGroup.zpp_pool.next=null;
                ZNPNode_ZPP_InteractionGroup.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_InteractionGroup.POOL_TOT=ZNPNode_ZPP_InteractionGroup.POOL_CNT=ZNPNode_ZPP_InteractionGroup.POOL_ADDNEW=ZNPNode_ZPP_InteractionGroup.POOL_ADD=ZNPNode_ZPP_InteractionGroup.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_CallbackSet.zpp_pool!=null){
                var nxt=ZPP_CallbackSet.zpp_pool.next;
                ZPP_CallbackSet.zpp_pool.next=null;
                ZPP_CallbackSet.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_CallbackSet.POOL_TOT=ZPP_CallbackSet.POOL_CNT=ZPP_CallbackSet.POOL_ADDNEW=ZPP_CallbackSet.POOL_ADD=ZPP_CallbackSet.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_Set_ZPP_CbSet.zpp_pool!=null){
                var nxt=ZPP_Set_ZPP_CbSet.zpp_pool.next;
                ZPP_Set_ZPP_CbSet.zpp_pool.next=null;
                ZPP_Set_ZPP_CbSet.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_Set_ZPP_CbSet.POOL_TOT=ZPP_Set_ZPP_CbSet.POOL_CNT=ZPP_Set_ZPP_CbSet.POOL_ADDNEW=ZPP_Set_ZPP_CbSet.POOL_ADD=ZPP_Set_ZPP_CbSet.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_InteractionFilter.zpp_pool!=null){
                var nxt=ZPP_InteractionFilter.zpp_pool.next;
                ZPP_InteractionFilter.zpp_pool.next=null;
                ZPP_InteractionFilter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_InteractionFilter.POOL_TOT=ZPP_InteractionFilter.POOL_CNT=ZPP_InteractionFilter.POOL_ADDNEW=ZPP_InteractionFilter.POOL_ADD=ZPP_InteractionFilter.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_ColArbiter.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_ColArbiter.zpp_pool.next;
                ZNPNode_ZPP_ColArbiter.zpp_pool.next=null;
                ZNPNode_ZPP_ColArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_ColArbiter.POOL_TOT=ZNPNode_ZPP_ColArbiter.POOL_CNT=ZNPNode_ZPP_ColArbiter.POOL_ADDNEW=ZNPNode_ZPP_ColArbiter.POOL_ADD=ZNPNode_ZPP_ColArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_FluidArbiter.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_FluidArbiter.zpp_pool.next;
                ZNPNode_ZPP_FluidArbiter.zpp_pool.next=null;
                ZNPNode_ZPP_FluidArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_FluidArbiter.POOL_TOT=ZNPNode_ZPP_FluidArbiter.POOL_CNT=ZNPNode_ZPP_FluidArbiter.POOL_ADDNEW=ZNPNode_ZPP_FluidArbiter.POOL_ADD=ZNPNode_ZPP_FluidArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_SensorArbiter.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_SensorArbiter.zpp_pool.next;
                ZNPNode_ZPP_SensorArbiter.zpp_pool.next=null;
                ZNPNode_ZPP_SensorArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_SensorArbiter.POOL_TOT=ZNPNode_ZPP_SensorArbiter.POOL_CNT=ZNPNode_ZPP_SensorArbiter.POOL_ADDNEW=ZNPNode_ZPP_SensorArbiter.POOL_ADD=ZNPNode_ZPP_SensorArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_SensorArbiter.zpp_pool!=null){
                var nxt=ZPP_SensorArbiter.zpp_pool.next;
                ZPP_SensorArbiter.zpp_pool.next=null;
                ZPP_SensorArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_SensorArbiter.POOL_TOT=ZPP_SensorArbiter.POOL_CNT=ZPP_SensorArbiter.POOL_ADDNEW=ZPP_SensorArbiter.POOL_ADD=ZPP_SensorArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_FluidArbiter.zpp_pool!=null){
                var nxt=ZPP_FluidArbiter.zpp_pool.next;
                ZPP_FluidArbiter.zpp_pool.next=null;
                ZPP_FluidArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_FluidArbiter.POOL_TOT=ZPP_FluidArbiter.POOL_CNT=ZPP_FluidArbiter.POOL_ADDNEW=ZPP_FluidArbiter.POOL_ADD=ZPP_FluidArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_Listener.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_Listener.zpp_pool.next;
                ZNPNode_ZPP_Listener.zpp_pool.next=null;
                ZNPNode_ZPP_Listener.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_Listener.POOL_TOT=ZNPNode_ZPP_Listener.POOL_CNT=ZNPNode_ZPP_Listener.POOL_ADDNEW=ZNPNode_ZPP_Listener.POOL_ADD=ZNPNode_ZPP_Listener.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_ToiEvent.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_ToiEvent.zpp_pool.next;
                ZNPNode_ZPP_ToiEvent.zpp_pool.next=null;
                ZNPNode_ZPP_ToiEvent.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_ToiEvent.POOL_TOT=ZNPNode_ZPP_ToiEvent.POOL_CNT=ZNPNode_ZPP_ToiEvent.POOL_ADDNEW=ZNPNode_ZPP_ToiEvent.POOL_ADD=ZNPNode_ZPP_ToiEvent.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_ColArbiter.zpp_pool!=null){
                var nxt=ZPP_ColArbiter.zpp_pool.next;
                ZPP_ColArbiter.zpp_pool.next=null;
                ZPP_ColArbiter.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZPP_ColArbiter.POOL_TOT=ZPP_ColArbiter.POOL_CNT=ZPP_ColArbiter.POOL_ADDNEW=ZPP_ColArbiter.POOL_ADD=ZPP_ColArbiter.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ConvexResult.zpp_pool!=null){
                var nxt=ZNPNode_ConvexResult.zpp_pool.next;
                ZNPNode_ConvexResult.zpp_pool.next=null;
                ZNPNode_ConvexResult.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ConvexResult.POOL_TOT=ZNPNode_ConvexResult.POOL_CNT=ZNPNode_ConvexResult.POOL_ADDNEW=ZNPNode_ConvexResult.POOL_ADD=ZNPNode_ConvexResult.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_ZPP_GeomPoly.zpp_pool!=null){
                var nxt=ZNPNode_ZPP_GeomPoly.zpp_pool.next;
                ZNPNode_ZPP_GeomPoly.zpp_pool.next=null;
                ZNPNode_ZPP_GeomPoly.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_ZPP_GeomPoly.POOL_TOT=ZNPNode_ZPP_GeomPoly.POOL_CNT=ZNPNode_ZPP_GeomPoly.POOL_ADDNEW=ZNPNode_ZPP_GeomPoly.POOL_ADD=ZNPNode_ZPP_GeomPoly.POOL_SUB=0;
            #end
        }
        {
            while(ZNPNode_RayResult.zpp_pool!=null){
                var nxt=ZNPNode_RayResult.zpp_pool.next;
                ZNPNode_RayResult.zpp_pool.next=null;
                ZNPNode_RayResult.zpp_pool=nxt;
            }
            #if NAPE_POOL_STATS ZNPNode_RayResult.POOL_TOT=ZNPNode_RayResult.POOL_CNT=ZNPNode_RayResult.POOL_ADDNEW=ZNPNode_RayResult.POOL_ADD=ZNPNode_RayResult.POOL_SUB=0;
            #end
        }
        
        {
            while(ZPP_PubPool.poolGeomPoly!=null){
                var nxt=ZPP_PubPool.poolGeomPoly.zpp_pool;
                ZPP_PubPool.poolGeomPoly.zpp_pool=null;
                ZPP_PubPool.poolGeomPoly=nxt;
            }
            #if NAPE_POOL_STATS GeomPoly.POOL_TOT=GeomPoly.POOL_CNT=GeomPoly.POOL_ADDNEW=GeomPoly.POOL_ADD=GeomPoly.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_PubPool.poolVec2!=null){
                var nxt=ZPP_PubPool.poolVec2.zpp_pool;
                ZPP_PubPool.poolVec2.zpp_pool=null;
                ZPP_PubPool.poolVec2=nxt;
            }
            #if NAPE_POOL_STATS Vec2.POOL_TOT=Vec2.POOL_CNT=Vec2.POOL_ADDNEW=Vec2.POOL_ADD=Vec2.POOL_SUB=0;
            #end
        }
        {
            while(ZPP_PubPool.poolVec3!=null){
                var nxt=ZPP_PubPool.poolVec3.zpp_pool;
                ZPP_PubPool.poolVec3.zpp_pool=null;
                ZPP_PubPool.poolVec3=nxt;
            }
            #if NAPE_POOL_STATS Vec3.POOL_TOT=Vec3.POOL_CNT=Vec3.POOL_ADDNEW=Vec3.POOL_ADD=Vec3.POOL_SUB=0;
            #end
        }
    }
    #if NAPE_TIMES static public var ltime=0;
    static public function TIMES(space:Space){
        var text="";
        var dt=space.timeStamp-ltime;
        text+="Validation:    "+Std.string(Debug.VALID/dt).substr(0,5)+"ms\n";
        text+="Broadphase:    "+Std.string((Debug.BROAD-Debug.NARROW)/dt).substr(0,5)+"ms :: total = "+Std.string(Debug.BROAD/dt).substr(0,5)+"ms\n";
        text+="Narrowphase:   "+Std.string(Debug.NARROW/dt).substr(0,5)+"ms\n";
        text+="Set-Forest:    "+Std.string(Debug.FOR/dt).substr(0,5)+"ms\n";
        text+="Prestep:       "+Std.string(Debug.PRE/dt).substr(0,5)+"ms\n";
        text+="Contact sort:  "+Std.string(Debug.SORT/dt).substr(0,5)+"ms\n";
        text+="Position It.:  "+Std.string(Debug.POS/dt).substr(0,5)+"ms\n";
        text+="Velocity It.:  "+Std.string(Debug.VEL/dt).substr(0,5)+"ms\n";
        text+="Debug Draw:    "+Std.string(Debug.DRAW/dt).substr(0,5)+"ms\n";
        text+="\n";
        text+="arb:        "+Debug.ACNT+"\n";
        text+="active arb: "+Debug.AACNT+"\n";
        text+="con:        "+Debug.CCNT+"\n";
        text+="active con: "+Debug.ACCNT+"\n";
        text+="\n";
        text+="hash collisions: "+Std.string(Debug.HASH/Debug.HASHT*100).substr(0,5)+"%\n";
        text+="\n";
        text+="Broadphase adjustments: "+Std.string(BROADCLASH/BROADTOTAL*100).substr(0,5)+"%\n";
        if(space.timeStamp-ltime>50){
            Debug.FOR=Debug.BROAD=Debug.PRE=Debug.POS=Debug.VEL=Debug.DRAW=Debug.VALID=Debug.SORT=Debug.NARROW=Debug.BROADCLASH=Debug.BROADTOTAL=0;
            ltime=space.timeStamp;
        }
        return text;
    }
    static public var ACNT=0;
    static public var AACNT=0;
    static public var CCNT=0;
    static public var ACCNT=0;
    static public var FOR=0;
    static public var PRE=0;
    static public var VEL=0;
    static public var POS=0;
    static public var BROAD=0;
    static public var NARROW=0;
    static public var DRAW=0;
    static public var VALID=0;
    static public var SORT=0;
    static public var HASH=0;
    static public var HASHT=0;
    static public var BROADCLASH=0;
    static public var BROADTOTAL=0;
    #end
    #if(flash9||openfl||nme)/**
     * Create a flash/openfl||nme Shape representing the given Body.
     *
     * @param body The body to create display Shape for.
     * @return A flash/openfl||nme.display.Shape representing Body.
     * @throws # If body is null.
     */
    #if nape_swc@:keep #end
    public static function createGraphic(body:Body):flash.display.Shape{
        #if(!NAPE_RELEASE_BUILD)
        if(body==null)throw "Error: Cannot create debug graphic for null Body";
        #end
        var ret=new flash.display.Shape();
        var graphics=ret.graphics;
        var idc:Int=(#if flash9 untyped __int__(0xffffff*Math.exp(-body.id/1500))#else Std.int(0xffffff*Math.exp(-body.id/1500))#end);
        var _r=(((idc&0xff0000)>>16))*0.7;
        var _g=(((idc&0xff00)>>8))*0.7;
        var _b=(idc&0xff)*0.7;
        var col=(((#if flash9 untyped __int__(_r)#else Std.int(_r)#end))<<16)|(((#if flash9 untyped __int__(_g)#else Std.int(_g)#end))<<8)|((#if flash9 untyped __int__(_b)#else Std.int(_b)#end));
        graphics.lineStyle(0.1,col,1);
        for(s in body.shapes){
            if(s.isCircle()){
                var c=s.castCircle;
                graphics.drawCircle(c.localCOM.x,c.localCOM.y,c.radius);
            }
            else{
                var p=s.castPolygon;
                graphics.moveTo(s.localCOM.x,s.localCOM.y);
                for(i in 0...p.worldVerts.length){
                    var px=p.localVerts.at(i);
                    graphics.lineTo(px.x,px.y);
                }
                var px=p.localVerts.at(0);
                graphics.lineTo(px.x,px.y);
            }
            if(s.isCircle()){
                var c=s.castCircle;
                graphics.moveTo(c.localCOM.x+c.radius*0.3,c.localCOM.y);
                graphics.lineTo(c.localCOM.x+c.radius,c.localCOM.y);
            }
        }
        return ret;
    }
    /**
     * @private
     */
    public var zpp_inner:ZPP_Debug=null;
    /**
     * If true, a representation of contact patches will be drawn.
     * <br/><br/>
     * Only active arbiters are drawn.
     * @default false
     */
    public var drawCollisionArbiters:Bool=false;
    /**
     * If true, a representation of centres of buoyancy and overlap will be drawn.
     * <br/><br/>
     * Only active arbiters are drawn.
     * @default false
     */
    public var drawFluidArbiters:Bool=false;
    /**
     * If true, a representation of sensor interactions will be drawn.
     * <br/><br/>
     * Only active arbiters are drawn.
     * @default false
     */
    public var drawSensorArbiters:Bool=false;
    /**
     * If true, then all bodies in the space (whether active or not) will be drawn.
     * @default true
     */
    public var drawBodies:Bool=false;
    /**
     * If true, then things like the body centre of mass, and bouncing box will be drawn.
     * <br/><br/>
     * This will only have an effect if drawBodies is true.
     * @default false
     */
    public var drawBodyDetail:Bool=false;
    /**
     * If true, then things like shape centre of mass and bounding box will be drawn.
     * <br/><br/>
     * This will only have an effect if drawBodies is true.
     * @default false
     */
    public var drawShapeDetail:Bool=false;
    /**
     * If true, then indicators of the shapes rotation will be drawn.
     * <br/><br/>
     * This will only have an effect if drawBodies is true.
     * @default true
     */
    public var drawShapeAngleIndicators:Bool=false;
    /**
     * If true, then representations of the active constraints will be drawn.
     * @default false
     */
    public var drawConstraints:Bool=false;
    /**
     * Background colour for debug draw display.
     * <br/><br/>
     * This value does not have much use for ShapeDebug, or for
     * a transparent BitmapDebug but will still be used in tinting
     * object colours to better fit an idealised background colour.
     */
    #if nape_swc@:isVar #end
    public var bgColour(get_bgColour,set_bgColour):Int;
    inline function get_bgColour():Int{
        return zpp_inner.bg_col;
    }
    inline function set_bgColour(bgColour:Int):Int{
        {
            #if flash10 if(zpp_inner.isbmp)zpp_inner.d_bmp.setbg(bgColour);
            else zpp_inner.d_shape.setbg(bgColour);
            #else zpp_inner.d_shape.setbg(bgColour);
            #end
        }
        return get_bgColour();
    }
    /**
     * User defined colour picking.
     * <br/><br/>
     * When not null, this method will be called to decide which colour
     * to use for an object with argument being the id of that object.
     * <br/><br/>
     * The return value should be an RGB value.
     *
     * @default null
     */
    public var colour:Null<Int->Int>=null;
    /**
     * @private
     */
    public function new(){
        #if(!NAPE_RELEASE_BUILD)
        if(!ZPP_Debug.internal)throw "Error: Cannot instantiate Debug derp! Use ShapeDebug, or BitmapDebug on flash10+";
        #end
        drawCollisionArbiters=false;
        drawFluidArbiters=false;
        drawSensorArbiters=false;
        drawBodies=true;
        drawShapeAngleIndicators=true;
        drawBodyDetail=false;
        drawShapeDetail=false;
        drawConstraints=false;
        cullingEnabled=false;
        colour=null;
    }
    /**
     * The flash/openfl||nme native display object representing debug draw.
     * <br/><br/>
     * When using debug drawer, you should add this to your display list.
     */
    #if nape_swc@:isVar #end
    public var display(get_display,never):flash.display.DisplayObject;
    inline function get_display():flash.display.DisplayObject{
        #if flash10 var ret:flash.display.DisplayObject;
        if(zpp_inner.isbmp)ret=zpp_inner.d_bmp.bitmap;
        else ret=zpp_inner.d_shape.shape;
        return ret;
        #else return zpp_inner.d_shape.shape;
        #end
    }
    /**
     * When true, objects outside the debug draw screen will not be drawn.
     * <br/><br/>
     * The debug draw screen is defined as the rectangle (0,0) -> (width,height).
     * To 'move' the debug draw screen in your world, you should modify the transform
     * property.
     * <br/><br/>
     * This culling has a cost, so is not worth enabling if everything is always on
     * screen anyways.
     *
     * @default false
     */
    public var cullingEnabled:Bool=false;
    /**
     * Transformation to apply to all debug draw operations.
     * <br/><br/>
     * This transform can be used to 'move' the debug draw screen through your
     * world as well as rotating and zooming in etc.
     * <br/><br/>
     * This transform effects 'all' debug draw operations and optimisation is in
     * place to not perform any transformation if matrix is the identity matrix.
     *
     * @default new Mat23()
     */
    #if nape_swc@:isVar #end
    public var transform(get_transform,set_transform):Mat23;
    inline function get_transform():Mat23{
        if(zpp_inner.xform==null)zpp_inner.setform();
        return zpp_inner.xform.outer;
    }
    inline function set_transform(transform:Mat23):Mat23{
        {
            #if(!NAPE_RELEASE_BUILD)
            if(transform==null)throw "Error: Cannot set Debug::transform to null";
            #end
            this.transform.set(transform);
        }
        return get_transform();
    }
    /**
     * Clear the debug view.
     */
    public function clear():Void{}
    /**
     * Flush any pending draw operations to debug view.
     * <br/><br/>
     * This operation is not needed for ShapeDebug at present.
     */
    public function flush():Void{}
    /**
     * Draw a Nape object to debug draw.
     * <br/><br/>
     * Possible argument types are: <code>Space, Compound, Body, Shape, Constraint</code>
     * <br/><br/>
     * To draw a Shape it must be part of a Body.
     * <br/><br/>
     * Debug draw settings like 'drawBodies' are overriden by a direct call to draw
     * with a Body or Shape. Equally even if drawConstraints is false, should
     * you call draw with a Constraint object directly it will be drawn regardless.
     *
     * @param object The object to draw.
     * @throws # If object is null or not of the expected Type.
     */
    public function draw(object:Dynamic):Void{}
    /**
     * Draw a line segment.
     * <br/><br/>
     * This line will be drawn with no thickness.
     *
     * @param start The start point of line segment.
     * @param end   The end point of line segment.
     * @param colour The colour of line in RGB.
     * @throws # If either start or end are null or disposed of.
     */
    public function drawLine(start:Vec2,end:Vec2,colour:Int):Void{}
    /**
     * Draw quadratic bezier curve.
     * <br/><br/>
     * This curve will be drawn with no thickness.
     *
     * @param start The start point of curve.
     * @param control The control point for curve.
     * @param end The end point of curve.
     * @param colour The colour of curve in RGB.
     * @throws # If any Vec2 argument is null or disposed of.
     */
    public function drawCurve(start:Vec2,control:Vec2,end:Vec2,colour:Int):Void{}
    /**
     * Draw circle.
     * <br/><br/>
     * This circle will be drawn with no thickness or fill.
     *
     * @param position The position of circle centre.
     * @param radius The radius of the circle.
     * @param colour The colour of circle in RGB.
     * @throws # If position is null or disposed of.
     * @throws # If radius is negative.
     * @throws # If transform is not equiorthogonal.
     */
    public function drawCircle(position:Vec2,radius:Float,colour:Int):Void{}
    /**
     * Draw AABB.
     * <br/><br/>
     * This AABB will be drawn with no thickness or fill.
     *
     * @param aabb The AABB to draw.
     * @param colour The colour to draw AABB with in RGB.
     * @throws # If AABB is null.
     */
    public function drawAABB(aabb:AABB,colour:Int):Void{}
    /**
     * Draw filled triangle.
     * <br/><br/>
     * This triangle will be drawn with no edges, only a solid fill.
     *
     * @param p0 The first point in triangle.
     * @param p1 The second point in triangle.
     * @param p2 The third point in triangle.
     * @param colour The colour to draw triangle with in RGB.
     * @throws # If any point argument is null or disposed of.
     */
    public function drawFilledTriangle(p0:Vec2,p1:Vec2,p2:Vec2,colour:Int):Void{}
    /**
     * Draw filled circle.
     * <br/><br/>
     * This circle will be drawn with no edges, only a solid fill.
     *
     * @param position The position of centre of circle.
     * @param radius The radius of circle.
     * @param colour The colour to draw circle with in RGB.
     * @throws # If position is null or disposed of.
     * @throws # If radius is negative.
     * @throws # If transform is not equiorthogonal.
     */
    public function drawFilledCircle(position:Vec2,radius:Float,colour:Int):Void{}
    /**
     * Draw polygon.
     * <br/><br/>
     * This polygon will be drawn with no thickness or fill.
     * <br/><br/>
     * The polygon argument is typed Dynamic and may be one of:
     * <code>Array&lt;Vec2&gt;, flash.Vector&lt;Vec2&gt;, Vec2List, GeomPoly</code>
     *
     * @param polygon The polygon to draw.
     * @param colour The colour to draw polygon with in RGB.
     * @throws # If polygon is null, or not of expected type.
     * @throws # If polygon contains disposed Vec2.
     */
    public function drawPolygon(polygon:Dynamic,colour:Int):Void{}
    /**
     * Draw filled polygon.
     * <br/><br/>
     * This polygon will be drawn no edges, only a solid fill.
     * <br/><br/>
     * The polygon argument is typed Dynamic and may be one of:
     * <code>Array&lt;Vec2&gt;, flash.Vector&lt;Vec2&gt;, Vec2List, GeomPoly</code>
     *
     * @param polygon The polygon to draw.
     * @param colour The colour to draw polygon with in RGB.
     * @throws # If polygon is null, or not of expected type.
     * @throws # If polygon contains disposed Vec2.
     */
    public function drawFilledPolygon(polygon:Dynamic,colour:Int):Void{}
    /**
     * Draw linear spring.
     * <br/><br/>
     * This spring will be drawn with no thickness.
     *
     * @param start The start point of spring.
     * @param end The end point of spring.
     * @param colour The colour of spring in RGB.
     * @param coils The number of coils in spring. (default 3)
     * @param radius The radius of spring. (default 3.0)
     * @throws # If start or end are either null or disposed of.
     * @throws # If number of coils is negative.
     */
    public function drawSpring(start:Vec2,end:Vec2,colour:Int,coils=3,radius=3.0):Void{}
    #end
}

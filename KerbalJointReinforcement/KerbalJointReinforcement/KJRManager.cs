/*
Kerbal Joint Reinforcement, v3.0.1
Copyright 2014, Michael Ferrara, aka Ferram4

    This file is part of Kerbal Joint Reinforcement.

    Kerbal Joint Reinforcement is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Kerbal Joint Reinforcement is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Kerbal Joint Reinforcement.  If not, see <http://www.gnu.org/licenses/>.
*/

using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using CompoundParts;

namespace KerbalJointReinforcement
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KJRManager : MonoBehaviour
    {
        List<Vessel> updatedVessels;
        Dictionary<Vessel, int> vesselOffRailsTick;
        Dictionary<Vessel, List<Joint>> vesselJointStrengthened;
        KJRMultiJointManager multiJointManager;

        FloatCurve physicsEasingCurve = new FloatCurve();
        int numTicksForEasing = 70;

        public void Awake()
        {
            KJRJointUtils.LoadConstants();
            updatedVessels = new List<Vessel>();
            vesselOffRailsTick = new Dictionary<Vessel, int>();
            vesselJointStrengthened = new Dictionary<Vessel, List<Joint>>();
            multiJointManager = new KJRMultiJointManager();
        }

        public void Start()
        {
            if (!CompatibilityChecker.IsAllCompatible())
                return;

            GameEvents.onVesselWasModified.Add(OnVesselWasModified);
            GameEvents.onVesselGoOffRails.Add(OnVesselOffRails);
            GameEvents.onVesselGoOnRails.Add(OnVesselOnRails);
            //GameEvents.onVesselLoaded.Add(RunVesselJointUpdateFunction);

            physicsEasingCurve.Add(numTicksForEasing, 1);
            physicsEasingCurve.Add(0, 0);
            
        }

        public void OnDestroy()
        {
            if (!CompatibilityChecker.IsAllCompatible())
                return;

            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
            GameEvents.onVesselGoOffRails.Remove(OnVesselOffRails);
            GameEvents.onVesselGoOnRails.Remove(OnVesselOnRails);
            //GameEvents.onVesselLoaded.Remove(RunVesselJointUpdateFunction);

            if (InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                InputLockManager.RemoveControlLock("KJRLoadLock");
            updatedVessels = null;
            vesselOffRailsTick = null;
            vesselJointStrengthened = null;

            multiJointManager.OnDestroy();
            multiJointManager = null;
        }

        private void OnVesselWasModified(Vessel v)
        {
            if ((object)v == null)
                return; 
            
            if (KJRJointUtils.debug)
            {
                StringBuilder debugString = new StringBuilder();
                debugString.AppendLine("KJR: Modified vessel " + v.id + " (" + v.GetName() + ")");
                debugString.AppendLine(System.Environment.StackTrace);
                debugString.AppendLine("Now contains: ");
                foreach (Part p in v.Parts)
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ")");
                Debug.Log(debugString);
            }

            updatedVessels.Remove(v);
            RunVesselJointUpdateFunction(v);
        }

        private void OnVesselOffRails(Vessel v)
        {
            if ((object)v == null)
                return; 
            
            RunVesselJointUpdateFunction(v);
            if(!vesselOffRailsTick.ContainsKey(v))
                vesselOffRailsTick.Add(v, numTicksForEasing);
        }

        private void OnVesselOnRails(Vessel v)
        {
            if ((object)v == null)
                return;

            if (updatedVessels.Contains(v))
            {
                if (vesselOffRailsTick.ContainsKey(v))
                {
                    foreach (Part p in v.Parts)
                    {
                        p.crashTolerance = p.crashTolerance * 1e-15f;
                        p.breakingForce = p.breakingForce * 1e-15f;
                        p.breakingTorque = p.breakingTorque * 1e-15f;
                    }

                    vesselOffRailsTick.Remove(v);
                }
                vesselJointStrengthened.Remove(v);
                updatedVessels.Remove(v);
            }
        }

        private void RunVesselJointUpdateFunction(Vessel v)
        {
            if (KJRJointUtils.debug)
            {
                Debug.Log("KJR: Processing vessel " + v.id + " (" + v.GetName() + "); root " +
                            v.rootPart.partInfo.name + " (" + v.rootPart.flightID + ")");
            }

            bool child_parts = false;
            bool success = false;

            foreach (Part p in v.Parts)
            {
                if (p.parent != null && p.physicalSignificance == Part.PhysicalSignificance.FULL)
                {
                    child_parts = true;

                    if (p.attachJoint != null)
                    {
                        success = true;

                        if (KJRJointUtils.reinforceAttachNodes)
                            UpdatePartJoint(p);
                    }
                }

                if (KJRJointUtils.reinforceDecouplersFurther)
                    if ((p.Modules.Contains("ModuleDecouple") || p.Modules.Contains("ModuleAnchoredDecoupler")) && !p.Modules.Contains("KJRDecouplerReinforcementModule"))
                    {
                        KJRJointUtils.AddDecouplerJointReinforcementModule(p);
                        continue;
                    }

                if (KJRJointUtils.reinforceLaunchClampsFurther)
                    if (p.Modules.Contains("LaunchClamp") && p.parent != null)
                    {
                        p.breakingForce = Mathf.Infinity;
                        p.breakingTorque = Mathf.Infinity;
                        p.mass = Mathf.Max(p.mass, (p.parent.mass + p.parent.GetResourceMass()) * 0.01f);          //We do this to make sure that there is a mass ratio of 100:1 between the clamp and what it's connected to.  This helps counteract some of the wobbliness simply, but also allows some give and springiness to absorb the initial physics kick
                        if (KJRJointUtils.debug)
                            Debug.Log("KJR: Launch Clamp Break Force / Torque increased");

                        if (!p.Modules.Contains("KJRLaunchClampReinforcementModule"))
                            KJRJointUtils.AddLaunchClampReinforcementModule(p);
                    }
            }

            if (success || !child_parts)
                updatedVessels.Add(v);
        }

/*        public void LateUpdate()
        {
            if (!FlightDriver.Pause && FlightGlobals.fetch && FlightGlobals.Vessels != null)
            {
                foreach (Vessel v in FlightGlobals.Vessels)
                {
                    if (!v.loaded)
                        continue;

//                    int tick = 0;
//                    float scalingFactor = 1;
                    //This scales up the inertia tensor over a few frames rather than trying to initialize with massive inertia tensors
//                    if (vesselOffRailsTick.TryGetValue(v, out tick))
//                    {
//                        scalingFactor = 1 - physicsEasingCurve.Evaluate(tick);
//                    }

                    //ScreenMessages.PostScreenMessage("Scaling Factor: " + scalingFactor, TimeWarp.deltaTime, ScreenMessageStyle.UPPER_LEFT);
                    
                    foreach (Part p in v.Parts)
                    {
                        //if (p.Modules.Contains("LaunchClamp"))
                        //    continue;
                        // This relies on KSP resetting the tensors to a constant in Update
                        if (p.started && p.State != PartStates.DEAD && p.rb)
                        {
                            float mass = p.rb.mass;// *scalingFactor;

                            if (mass > 1f)
                                p.rb.inertiaTensor *= mass;
                        }
                    }
                }
            }
        }*/

        public void FixedUpdate()
        {
            if (FlightGlobals.ready && FlightGlobals.Vessels != null)
            {
                List<Vessel> removeVessels = new List<Vessel>();
                List<Vessel> tmpList = new List<Vessel>(vesselOffRailsTick.Keys);
                foreach(Vessel v in tmpList)
                {
                    int tick = vesselOffRailsTick[v];
                    if (tick > 0)
                    {
                        float physicsScalingFactor = physicsEasingCurve.Evaluate(tick);
                        if (tick >= numTicksForEasing)
                        {
                            List<Joint> jointList = new List<Joint>();
                            foreach (Part p in v.Parts)
                            {
                                p.crashTolerance = p.crashTolerance * 1e15f;
                                p.breakingForce = p.breakingForce * 1e15f;
                                p.breakingTorque = p.breakingTorque * 1e15f;
                                
                                Joint[] partJoints = p.GetComponents<Joint>();
/*                                foreach (Joint j in partJoints)
                                {
//                                    j.breakForce *= 1000000000000000000;
//                                    j.breakTorque *= 1000000000000000000;
                                    jointList.Add(j);
                                    Debug.Log("Part: " + p.partInfo.title + " BreakForce = " + j.breakForce + " BreakTorque = " + j.breakTorque);
                                }*/
                                if (p.Modules.Contains("LaunchClamp"))
                                {
                                    foreach (Joint j in partJoints)
                                        if (j.connectedBody == null)
                                        {
                                            jointList.Remove(j);
                                            GameObject.Destroy(j);
                                            KJRJointUtils.ConnectLaunchClampToGround(p);
                                            break;
                                        }
                                }
                            }
                            //vesselJointStrengthened.Add(v, jointList);
                        }
                        bool easing = false;
                        if (v.situation == Vessel.Situations.PRELAUNCH || v.situation == Vessel.Situations.LANDED || v.situation == Vessel.Situations.SPLASHED)
                            easing = true;
                        else
                        {
                            foreach (Part p in v.Parts)
                                if (p.Modules.Contains("LaunchClamp"))
                                {
                                    easing = true;
                                    break;
                                }
                        }
                        if (easing)
                        {
                            Vector3d vesselPos = v.GetWorldPos3D();
                            Vector3d vesselVel = v.obt_velocity;
                            Vector3d vesselAcceleration = FlightGlobals.getGeeForceAtPosition(vesselPos) + FlightGlobals.getCentrifugalAcc(vesselPos, FlightGlobals.currentMainBody) + FlightGlobals.getCoriolisAcc(vesselVel, FlightGlobals.currentMainBody);
                            vesselAcceleration *= physicsScalingFactor;
                            foreach (Part p in v.Parts)
                                if (p.rb)
                                {
                                    p.rb.AddForce(-vesselAcceleration * p.rb.mass);
                                }

                        }
                        if (v == FlightGlobals.ActiveVessel)
                        {
                            if (InputLockManager.GetControlLock("KJRLoadLock") != ControlTypes.ALL_SHIP_CONTROLS)
                                InputLockManager.SetControlLock(ControlTypes.ALL_SHIP_CONTROLS, "KJRLoadLock");
                            ScreenMessages.PostScreenMessage("KJR stabilizing physics load...", TimeWarp.fixedDeltaTime, ScreenMessageStyle.UPPER_RIGHT);
                        }
                        else
                            if (InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                                InputLockManager.RemoveControlLock("KJRLoadLock");

                        tick--;
                        vesselOffRailsTick[v] = tick;
                    }
                    else if (tick == 0)
                    {
                        foreach (Part p in v.Parts)
                        {
                            p.crashTolerance = p.crashTolerance * 1e-15f;
                            p.breakingForce = p.breakingForce * 1e-15f;
                            p.breakingTorque = p.breakingTorque * 1e-15f;
                        }

                        removeVessels.Add(v);
                        if (InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                            InputLockManager.RemoveControlLock("KJRLoadLock");
                    }
                    else
                    {
                        foreach (Part p in v.Parts)
                        {
                            p.crashTolerance = p.crashTolerance * 1e-15f;
                            p.breakingForce = p.breakingForce * 1e-15f;
                            p.breakingTorque = p.breakingTorque * 1e-15f;
                        }

                        removeVessels.Add(v);
                        if (InputLockManager.GetControlLock("KJRLoadLock") == ControlTypes.ALL_SHIP_CONTROLS)
                            InputLockManager.RemoveControlLock("KJRLoadLock");
                    }
                }
                foreach (Vessel v in removeVessels)
                    vesselOffRailsTick.Remove(v);
            }
        }

        private void UpdatePartJoint(Part p)
        {
            if (!KJRJointUtils.JointAdjustmentValid(p) || p.rb == null || p.attachJoint == null)
                return;

            if (p.attachMethod == AttachNodeMethod.LOCKED_JOINT)
            {
                if (KJRJointUtils.debug)
                {
                    Debug.Log("KJR: Already processed part before: " + p.partInfo.name + " (" + p.flightID + ") -> " +
                              p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                }

                return;
            }
            List<ConfigurableJoint> jointList;
            if (p is StrutConnector)
            {
                StrutConnector s = p as StrutConnector;

                if (s.jointTarget == null || s.jointRoot == null)
                    return;

                jointList = KJRJointUtils.GetJointListFromAttachJoint(s.strutJoint);

                if (jointList != null)
                {
                    for (int i = 0; i < jointList.Count; i++)
                    {
                        ConfigurableJoint j = jointList[i];

                        if (j == null)
                            continue;

                        JointDrive strutDrive = j.angularXDrive;
                        strutDrive.positionSpring = KJRJointUtils.decouplerAndClampJointStrength;
                        strutDrive.maximumForce = KJRJointUtils.decouplerAndClampJointStrength;
                        j.xDrive = j.yDrive = j.zDrive = j.angularXDrive = j.angularYZDrive = strutDrive;

                        j.xMotion = j.yMotion = j.zMotion = ConfigurableJointMotion.Locked;
                        j.angularXMotion = j.angularYMotion = j.angularZMotion = ConfigurableJointMotion.Locked;

                        //float scalingFactor = (s.jointTarget.mass + s.jointTarget.GetResourceMass() + s.jointRoot.mass + s.jointRoot.GetResourceMass()) * 0.01f;

                        j.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                        j.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;
                    }

                    p.attachMethod = AttachNodeMethod.LOCKED_JOINT;
                }
            }
            if(p is CompoundPart)
            {
                if (p.Modules.Contains("CModuleStrut"))
                {
                    CModuleStrut s = (CModuleStrut)p.Modules["CModuleStrut"];

                    if (s.jointTarget == null || s.jointRoot == null)
                        return;

                    jointList = KJRJointUtils.GetJointListFromAttachJoint(s.strutJoint);

                    if (jointList != null)
                    {
                        for (int i = 0; i < jointList.Count; i++)
                        {
                            ConfigurableJoint j = jointList[i];

                            if (j == null)
                                continue;

                            JointDrive strutDrive = j.angularXDrive;
                            strutDrive.positionSpring = KJRJointUtils.decouplerAndClampJointStrength;
                            strutDrive.maximumForce = KJRJointUtils.decouplerAndClampJointStrength;
                            j.xDrive = j.yDrive = j.zDrive = j.angularXDrive = j.angularYZDrive = strutDrive;

                            j.xMotion = j.yMotion = j.zMotion = ConfigurableJointMotion.Locked;
                            j.angularXMotion = j.angularYMotion = j.angularZMotion = ConfigurableJointMotion.Locked;

                            //float scalingFactor = (s.jointTarget.mass + s.jointTarget.GetResourceMass() + s.jointRoot.mass + s.jointRoot.GetResourceMass()) * 0.01f;

                            j.breakForce = KJRJointUtils.decouplerAndClampJointStrength;
                            j.breakTorque = KJRJointUtils.decouplerAndClampJointStrength;
                        }

                        p.attachMethod = AttachNodeMethod.LOCKED_JOINT;
                    }
                }
            }

            jointList = KJRJointUtils.GetJointListFromAttachJoint(p.attachJoint);
            if (jointList == null)
                return;

            StringBuilder debugString = new StringBuilder();

            bool addAdditionalJointToParent = KJRJointUtils.multiPartAttachNodeReinforcement;
            addAdditionalJointToParent &= !(p.Modules.Contains("LaunchClamp") || (p.parent.Modules.Contains("ModuleDecouple") || p.parent.Modules.Contains("ModuleAnchoredDecoupler")));
            addAdditionalJointToParent &= !(p is StrutConnector || p.Modules.Contains("CModuleStrut"));

            float partMass = p.mass + p.GetResourceMass();
            for (int i = 0; i < jointList.Count; i++)
            {
                ConfigurableJoint j = jointList[i];
                if (j == null)
                    continue;

                String jointType = j.GetType().Name;
                Rigidbody connectedBody = j.connectedBody;

                Part connectedPart = connectedBody.GetComponent<Part>() ?? p.parent;
                float parentMass = connectedPart.mass + connectedPart.GetResourceMass();

                if (partMass < KJRJointUtils.massForAdjustment || parentMass < KJRJointUtils.massForAdjustment)
                {
                    if (KJRJointUtils.debug)
                    {
                        Debug.Log("KJR: Part mass too low, skipping: " + p.partInfo.name + " (" + p.flightID + ")");
                    }

                    continue;
                }                
                
                // Check attachment nodes for better orientation data
                AttachNode attach = p.findAttachNodeByPart(p.parent);
                AttachNode p_attach = p.parent.findAttachNodeByPart(p);
                AttachNode node = attach ?? p_attach;

                if (node == null)
                {
                    // Check if it's a pair of coupled docking ports
                    var dock1 = p.Modules["ModuleDockingNode"] as ModuleDockingNode;
                    var dock2 = p.parent.Modules["ModuleDockingNode"] as ModuleDockingNode;

                    //Debug.Log(dock1 + " " + (dock1 ? ""+dock1.dockedPartUId : "?") + " " + dock2 + " " + (dock2 ? ""+dock2.dockedPartUId : "?"));

                    if (dock1 && dock2 && (dock1.dockedPartUId == p.parent.flightID || dock2.dockedPartUId == p.flightID))
                    {
                        attach = p.findAttachNode(dock1.referenceAttachNode);
                        p_attach = p.parent.findAttachNode(dock2.referenceAttachNode);
                        node = attach ?? p_attach;
                    }
                }

                // If still no node and apparently surface attached, use the normal one if it's there
                if (node == null && p.attachMode == AttachModes.SRF_ATTACH)
                    node = attach = p.srfAttachNode;

                if (KJRJointUtils.debug)
                {
                    debugString.AppendLine("Original joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                    debugString.AppendLine("");
                    debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rigidbody.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                    debugString.AppendLine("");


                    debugString.AppendLine("Std. Joint Parameters");
                    debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                    debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                    if (attach != null)
                        debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                    if (p_attach != null)
                        debugString.AppendLine("Parent node: " + p_attach.id + " - " + p_attach.nodeType + " " + p_attach.size);
                    debugString.AppendLine("Anchor: " + p.attachJoint.Joint.anchor);
                    debugString.AppendLine("Axis: " + p.attachJoint.Joint.axis);
                    debugString.AppendLine("Sec Axis: " + p.attachJoint.Joint.secondaryAxis);
                    debugString.AppendLine("Break Force: " + p.attachJoint.Joint.breakForce);
                    debugString.AppendLine("Break Torque: " + p.attachJoint.Joint.breakTorque);
                    debugString.AppendLine("");

                    debugString.AppendLine("Joint Motion Locked: " + Convert.ToString(p.attachJoint.Joint.xMotion == ConfigurableJointMotion.Locked));

                    debugString.AppendLine("X Drive");
                    debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.xDrive.positionSpring);
                    debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.xDrive.positionDamper);
                    debugString.AppendLine("Max Force: " + p.attachJoint.Joint.xDrive.maximumForce);
                    debugString.AppendLine("Mode: " + p.attachJoint.Joint.xDrive.mode);
                    debugString.AppendLine("");

                    debugString.AppendLine("Y Drive");
                    debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.yDrive.positionSpring);
                    debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.yDrive.positionDamper);
                    debugString.AppendLine("Max Force: " + p.attachJoint.Joint.yDrive.maximumForce);
                    debugString.AppendLine("Mode: " + p.attachJoint.Joint.yDrive.mode);
                    debugString.AppendLine("");

                    debugString.AppendLine("Z Drive");
                    debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.zDrive.positionSpring);
                    debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.zDrive.positionDamper);
                    debugString.AppendLine("Max Force: " + p.attachJoint.Joint.zDrive.maximumForce);
                    debugString.AppendLine("Mode: " + p.attachJoint.Joint.zDrive.mode);
                    debugString.AppendLine("");

                    debugString.AppendLine("Angular X Drive");
                    debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.angularXDrive.positionSpring);
                    debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.angularXDrive.positionDamper);
                    debugString.AppendLine("Max Force: " + p.attachJoint.Joint.angularXDrive.maximumForce);
                    debugString.AppendLine("Mode: " + p.attachJoint.Joint.angularXDrive.mode);
                    debugString.AppendLine("");

                    debugString.AppendLine("Angular YZ Drive");
                    debugString.AppendLine("Position Spring: " + p.attachJoint.Joint.angularYZDrive.positionSpring);
                    debugString.AppendLine("Position Damper: " + p.attachJoint.Joint.angularYZDrive.positionDamper);
                    debugString.AppendLine("Max Force: " + p.attachJoint.Joint.angularYZDrive.maximumForce);
                    debugString.AppendLine("Mode: " + p.attachJoint.Joint.angularYZDrive.mode);
                    debugString.AppendLine("");


                    //Debug.Log(debugString.ToString());
                }


                float breakForce = p.breakingForce * KJRJointUtils.breakForceMultiplier;
                float breakTorque = p.breakingTorque * KJRJointUtils.breakTorqueMultiplier;
                Vector3 anchor = j.anchor;
                Vector3 connectedAnchor = j.connectedAnchor;
                Vector3 axis = j.axis;

                float radius = 0;
                float area = 0;
                float momentOfInertia = 0;

                if (node != null)
                {
                    // Part that owns the node. For surface attachment,
                    // this can only be parent if docking flips hierarchy.
                    Part main = (node == attach) ? p : p.parent;

                    // Orientation and position of the node in owner's local coords
                    Vector3 ndir = node.orientation.normalized;
                    Vector3 npos = node.position + node.offset;

                    // And in the current part's local coords
                    Vector3 dir = axis = p.transform.InverseTransformDirection(main.transform.TransformDirection(ndir));

                    if (node.nodeType == AttachNode.NodeType.Surface)
                    {
                        // Guessed main axis; for parts with stack nodes should be the axis of the stack
                        Vector3 up = KJRJointUtils.GuessUpVector(main).normalized;

                        // if guessed up direction is same as node direction, it's basically stack
                        // for instance, consider a radially-attached docking port
                        if (Mathf.Abs(Vector3.Dot(up, ndir)) > 0.9f)
                        {
                            radius = Mathf.Min(KJRJointUtils.CalculateRadius(main, ndir), KJRJointUtils.CalculateRadius(connectedPart, ndir));
                            if (radius <= 0.001)
                                radius = node.size * 1.25f;
                            area = Mathf.PI * radius * radius;                      //Area of cylinder
                            momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                        }
                        else
                        {
                            // x along surface, y along ndir normal to surface, z along surface & main axis (up)
                            var size1 = KJRJointUtils.CalculateExtents(main, ndir, up);

                            var size2 = KJRJointUtils.CalculateExtents(connectedPart, ndir, up);

                            // use average of the sides, since we don't know which one is used for attaching
                            float width1 = (size1.x + size1.z) / 2;
                            float width2 = (size2.x + size2.z) / 2;
                            if (size1.y * width1 > size2.y * width2)
                            {
                                area = size1.y * width1;
                                radius = Mathf.Max(size1.y, width1);
                            }
                            else
                            {
                                area = size2.y * width2;
                                radius = Mathf.Max(size2.y, width2);
                            }

                            momentOfInertia = area * area / 12;          //Moment of Inertia of a rectangle bending along the longer length
                        }
                    }
                    else
                    {
                        radius = Mathf.Min(KJRJointUtils.CalculateRadius(p, dir), KJRJointUtils.CalculateRadius(connectedPart, dir));
                        if (radius <= 0.001)
                            radius = node.size * 1.25f;
                        area = Mathf.PI * radius * radius;                      //Area of cylinder
                        momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                    }
                }
                //Assume part is attached along its "up" cross section; use a cylinder to approximate properties
                else if (p.attachMode == AttachModes.STACK)
                {
                    radius = Mathf.Min(KJRJointUtils.CalculateRadius(p, Vector3.up), KJRJointUtils.CalculateRadius(connectedPart, Vector3.up));
                    if (radius <= 0.001)
                        radius = node.size * 1.25f;
                    area = Mathf.PI * radius * radius;                      //Area of cylinder
                    momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                }
                else if (p.attachMode == AttachModes.SRF_ATTACH)
                {                    
                    // x,z sides, y along main axis
                    Vector3 up1 = KJRJointUtils.GuessUpVector(p);
                    var size1 = KJRJointUtils.CalculateExtents(p, up1);

                    Vector3 up2 = KJRJointUtils.GuessUpVector(connectedPart);
                    var size2 = KJRJointUtils.CalculateExtents(connectedPart, up2);

                    // use average of the sides, since we don't know which one is used for attaching
                    float width1 = (size1.x + size1.z) / 2;
                    float width2 = (size2.x + size2.z) / 2;
                    if (size1.y * width1 > size2.y * width2)
                    {
                        area = size1.y * width1;
                        radius = Mathf.Max(size1.y, width1);
                    }
                    else
                    {
                        area = size2.y * width2;
                        radius = Mathf.Max(size2.y, width2);
                    }
                    momentOfInertia = area * area / 12;          //Moment of Inertia of a rectangle bending along the longer length
                }

                if (KJRJointUtils.useVolumeNotArea)                //If using volume, raise al stiffness-affecting parameters to the 1.5 power
                {
                    area = Mathf.Pow(area, 1.5f);
                    momentOfInertia = Mathf.Pow(momentOfInertia, 1.5f);
                }


                breakForce = Mathf.Max(KJRJointUtils.breakStrengthPerArea * area, breakForce);
                breakTorque = Mathf.Max(KJRJointUtils.breakTorquePerMOI * momentOfInertia, breakTorque);
                

                JointDrive drive = j.angularXDrive;
                drive.positionSpring = Mathf.Max(momentOfInertia * KJRJointUtils.angularDriveSpring, drive.positionSpring);
                drive.positionDamper = Mathf.Max(momentOfInertia * KJRJointUtils.angularDriveDamper, drive.positionDamper);
                j.angularXDrive = j.angularYZDrive = j.slerpDrive = drive;

                SoftJointLimit lim = new SoftJointLimit();
                lim.damper = 0;
                lim.spring = 0;
                lim.limit = 0;
                lim.bounciness = 0;

                j.linearLimit = j.angularYLimit = j.angularZLimit = j.lowAngularXLimit = j.highAngularXLimit = lim;

                j.targetAngularVelocity = Vector3.zero;
                j.targetVelocity = Vector3.zero;
                j.targetRotation = Quaternion.identity;
                j.targetPosition = Vector3.zero;

                j.breakForce = breakForce;
                j.breakTorque = breakTorque;

                p.attachMethod = AttachNodeMethod.LOCKED_JOINT;

                if (addAdditionalJointToParent && p.parent.parent != null)
                {
                    addAdditionalJointToParent = false;
                    if (!KJRJointUtils.JointAdjustmentValid(p.parent) || p.parent.parent.rb == null)
                        continue;
                    ConfigurableJoint newJoint = p.gameObject.AddComponent<ConfigurableJoint>();

                    Part newConnectedPart = p.parent.parent;

                    newJoint.connectedBody = newConnectedPart.rb;
                    newJoint.axis = Vector3.right;
                    newJoint.secondaryAxis = Vector3.forward;
                    newJoint.anchor = Vector3.zero;
                    newJoint.connectedAnchor = p.transform.worldToLocalMatrix.MultiplyPoint(newConnectedPart.transform.position);

                    newJoint.angularXDrive = newJoint.angularYZDrive = newJoint.slerpDrive = drive;

                    newJoint.xDrive = j.xDrive;
                    newJoint.yDrive = j.yDrive;
                    newJoint.zDrive = j.zDrive;

                    newJoint.linearLimit = newJoint.angularYLimit = newJoint.angularZLimit = newJoint.lowAngularXLimit = newJoint.highAngularXLimit = lim;

                    newJoint.targetAngularVelocity = Vector3.zero;
                    newJoint.targetVelocity = Vector3.zero;
                    newJoint.targetRotation = Quaternion.identity;
                    newJoint.targetPosition = Vector3.zero;

                    newJoint.breakForce = breakForce;
                    newJoint.breakTorque = breakTorque;

                    //jointList.Add(newJoint);
                    multiJointManager.RegisterMultiJoint(p, newJoint);
                    multiJointManager.RegisterMultiJoint(p.parent, newJoint);
                }

                if (KJRJointUtils.debug)
                {
                    debugString.AppendLine("Updated joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
                    debugString.AppendLine("  " + p.partInfo.name + " (" + p.flightID + ") -> " + p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                    debugString.AppendLine("");
                    debugString.AppendLine(p.partInfo.title + " Inertia Tensor: " + p.rigidbody.inertiaTensor + " " + p.parent.partInfo.name + " Inertia Tensor: " + connectedBody.inertiaTensor);
                    debugString.AppendLine("");


                    /*debugString.AppendLine("Std. Joint Parameters");
                    debugString.AppendLine("Connected Body: " + p.attachJoint.Joint.connectedBody);
                    debugString.AppendLine("Attach mode: " + p.attachMode + " (was " + jointType + ")");
                    if (attach != null)
                        debugString.AppendLine("Attach node: " + attach.id + " - " + attach.nodeType + " " + attach.size);
                    if (p_attach != null)
                        debugString.AppendLine("Parent node: " + p_attach.id + " - " + p_attach.nodeType + " " + p_attach.size);
                    debugString.AppendLine("Anchor: " + p.attachJoint.Joint.anchor);
                    debugString.AppendLine("Axis: " + p.attachJoint.Joint.axis);
                    debugString.AppendLine("Sec Axis: " + p.attachJoint.Joint.secondaryAxis);
                    debugString.AppendLine("Break Force: " + p.attachJoint.Joint.breakForce);
                    debugString.AppendLine("Break Torque: " + p.attachJoint.Joint.breakTorque);
                    debugString.AppendLine("");

                    debugString.AppendLine("Joint Motion Locked: " + Convert.ToString(p.attachJoint.Joint.xMotion == ConfigurableJointMotion.Locked));

                    debugString.AppendLine("Angular Drive");
                    debugString.AppendLine("Position Spring: " + drive.positionSpring);
                    debugString.AppendLine("Position Damper: " + drive.positionDamper);
                    debugString.AppendLine("Max Force: " + drive.maximumForce);
                    debugString.AppendLine("Mode: " + drive.mode);
                    debugString.AppendLine("");

                    debugString.AppendLine("Cross Section Properties");
                    debugString.AppendLine("Radius: " + radius);
                    debugString.AppendLine("Area: " + area);
                    debugString.AppendLine("Moment of Inertia: " + momentOfInertia);*/

                }
            }
            if (KJRJointUtils.debug)
                Debug.Log(debugString.ToString());
        }
    }
}
/*
Kerbal Joint Reinforcement, v2.4.5
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
using KSP.IO;
using KSP;

namespace KerbalJointReinforcement
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KerbalJointReinforcement : MonoBehaviour
    {
        List<Vessel> updatedVessels = new List<Vessel>();
        Dictionary<Vessel, int> vesselOffRailsTick = new Dictionary<Vessel, int>();
        Dictionary<Vessel, List<Joint>> vesselJointStrengthened = new Dictionary<Vessel, List<Joint>>();
        FloatCurve physicsEasingCurve = new FloatCurve();
        int numTicksForEasing = 70;

        public void Awake()
        {
            JointUtils.LoadConstants();
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
        }

        private void OnVesselWasModified(Vessel v)
        {
            if ((object)v == null)
                return; 
            
            if (JointUtils.debug)
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
            if (JointUtils.debug)
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

                        if (JointUtils.reinforceAttachNodes)
                            UpdatePartJoint(p);
                    }
                }

                if (JointUtils.reinforceDecouplersFurther)
                    if ((p.Modules.Contains("ModuleDecouple") || p.Modules.Contains("ModuleAnchoredDecoupler")) && !p.Modules.Contains("DecouplerJointReinforcementModule"))
                    {
                        JointUtils.AddDecouplerJointReinforcementModule(p);
                        continue;
                    }

                if (JointUtils.reinforceLaunchClampsFurther)
                    if (p.Modules.Contains("LaunchClamp") && p.parent != null)
                    {
                        p.breakingForce = Mathf.Infinity;
                        p.breakingTorque = Mathf.Infinity;
                        p.mass = Mathf.Max(p.mass, (p.parent.mass + p.parent.GetResourceMass()) * 0.01f);          //We do this to make sure that there is a mass ratio of 100:1 between the clamp and what it's connected to.  This helps counteract some of the wobbliness simply, but also allows some give and springiness to absorb the initial physics kick
                        if (JointUtils.debug)
                            Debug.Log("KJR: Launch Clamp Break Force / Torque increased");

                        if(!p.Modules.Contains("LaunchClampReinforcementModule"))
                            JointUtils.AddLaunchClampReinforcementModule(p);
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
                                            JointUtils.ConnectLaunchClampToGround(p);
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
            if (!JointUtils.JointAdjustmentValid(p) || p.rb == null)
                return;

            if (p.attachJoint.Joint is ConfigurableJoint &&
                p.attachMethod == AttachNodeMethod.LOCKED_JOINT)
            {
                if (JointUtils.debug)
                {
                    Debug.Log("KJR: Already processed part before: " + p.partInfo.name + " (" + p.flightID + ") -> " +
                              p.parent.partInfo.name + " (" + p.parent.flightID + ")");
                }

                return;
            }

            if (p is StrutConnector)
            {
                StrutConnector s = p as StrutConnector;
                JointDrive strutDrive = s.strutJoint.Joint.angularXDrive;
                strutDrive.positionSpring = JointUtils.decouplerAndClampJointStrength;
                strutDrive.maximumForce = JointUtils.decouplerAndClampJointStrength;
                s.strutJoint.Joint.xDrive = s.strutJoint.Joint.yDrive = s.strutJoint.Joint.zDrive = s.strutJoint.Joint.angularXDrive = s.strutJoint.Joint.angularYZDrive = strutDrive;

                float scalingFactor = (s.jointTarget.mass + s.jointTarget.GetResourceMass() + s.jointRoot.mass + s.jointRoot.GetResourceMass()) * 0.01f;

                s.strutJoint.SetBreakingForces(s.strutJoint.Joint.breakForce * scalingFactor, s.strutJoint.Joint.breakTorque * scalingFactor);

                p.attachMethod = AttachNodeMethod.LOCKED_JOINT;

                return;
            }

            StringBuilder debugString = new StringBuilder();

            String jointType = p.attachJoint.Joint.GetType().Name;
            Rigidbody connectedBody = p.attachJoint.Joint.connectedBody;

            Part connectedPart = connectedBody.GetComponent<Part>() ?? p.parent;
            float partMass = p.mass + p.GetResourceMass();
            float parentMass = connectedPart.mass + connectedPart.GetResourceMass();

            if (partMass < JointUtils.massForAdjustment || parentMass < JointUtils.massForAdjustment)
            {
                if (JointUtils.debug)
                {
                    Debug.Log("KJR: Part mass too low, skipping: " + p.partInfo.name + " (" + p.flightID + ")");
                }

                return;
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

            if (JointUtils.debug)
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

            
            float breakForce = p.breakingForce * JointUtils.breakForceMultiplier;
            float breakTorque = p.breakingTorque * JointUtils.breakTorqueMultiplier;
            Vector3 anchor = p.attachJoint.Joint.anchor;
            Vector3 connectedAnchor = p.attachJoint.Joint.connectedAnchor;
            Vector3 axis = p.attachJoint.Joint.axis;

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
                    Vector3 up = JointUtils.GuessUpVector(main).normalized;

                    // if guessed up direction is same as node direction, it's basically stack
                    // for instance, consider a radially-attached docking port
                    if (Mathf.Abs(Vector3.Dot(up, ndir)) > 0.9f)
                    {
                        radius = Mathf.Min(JointUtils.CalculateRadius(main, ndir), JointUtils.CalculateRadius(connectedPart, ndir));
                        if (radius <= 0.001)
                            radius = node.size * 1.25f;
                        area = Mathf.PI * radius * radius;                      //Area of cylinder
                        momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                    }
                    else
                    {
                        // x along surface, y along ndir normal to surface, z along surface & main axis (up)
                        var size1 = JointUtils.CalculateExtents(main, ndir, up);

                        var size2 = JointUtils.CalculateExtents(connectedPart, ndir, up);

                        // use average of the sides, since we don't know which one is used for attaching
                        float width1 = (size1.x + size1.z) / 2;
                        float width2 = (size2.x + size2.z) / 2;
                        if (size1.y * width1 > size2.y * width2)
                        {
                            area = size1.y * width1;
                            radius = Mathf.Max(size1.y, width1);
                        }

                        momentOfInertia = area * area / 12;          //Moment of Inertia of a rectangle bending along the longer length
                    }
                }
                else
                {
                    radius = Mathf.Min(JointUtils.CalculateRadius(p, dir), JointUtils.CalculateRadius(connectedPart, dir));
                    if (radius <= 0.001)
                        radius = node.size * 1.25f;
                    area = Mathf.PI * radius * radius;                      //Area of cylinder
                    momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
                }
            }
            //Assume part is attached along its "up" cross section; use a cylinder to approximate properties
            else if (p.attachMode == AttachModes.STACK)
            {

                radius = Mathf.Min(JointUtils.CalculateRadius(p, Vector3.up), JointUtils.CalculateRadius(connectedPart, Vector3.up));
                if (radius <= 0.001)
                    radius = node.size * 1.25f;
                area = Mathf.PI * radius * radius;                      //Area of cylinder
                momentOfInertia = area * radius * radius / 4;           //Moment of Inertia of cylinder
            }
            else if (p.attachMode == AttachModes.SRF_ATTACH)
            {
                // x,z sides, y along main axis
                Vector3 up1 = JointUtils.GuessUpVector(p);
                var size1 = JointUtils.CalculateExtents(p, up1);

                Vector3 up2 = JointUtils.GuessUpVector(connectedPart);
                var size2 = JointUtils.CalculateExtents(connectedPart, up2);

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

            if (JointUtils.useVolumeNotArea)                //If using volume, raise al stiffness-affecting parameters to the 1.5 power
            {
                area = Mathf.Pow(area, 1.5f);
                momentOfInertia = Mathf.Pow(momentOfInertia, 1.5f);
            }

            if (!((JointUtils.breakStrengthPerArea <= 0 && JointUtils.breakTorquePerMOI <= 0 )|| area <= 0))
            {
                breakForce = Mathf.Max(JointUtils.breakStrengthPerArea * area, breakForce);
                breakTorque = Mathf.Max(JointUtils.breakTorquePerMOI * momentOfInertia, breakTorque);
            }

            JointDrive drive = p.attachJoint.Joint.angularXDrive;
            drive.positionSpring = Mathf.Max(area * JointUtils.angularDriveSpring, drive.positionSpring);
            drive.positionDamper = Mathf.Max(area * JointUtils.angularDriveDamper, drive.positionDamper);
            p.attachJoint.Joint.angularXDrive = p.attachJoint.Joint.angularYZDrive = drive;

            p.attachMethod = AttachNodeMethod.LOCKED_JOINT;

            p.attachJoint.SetBreakingForces(breakForce, breakTorque);

            if (JointUtils.debug)
            {
                debugString.AppendLine("Updated joint from " + p.partInfo.title + " to " + p.parent.partInfo.title);
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

                debugString.AppendLine("Angular Drive");
                debugString.AppendLine("Position Spring: " + drive.positionSpring);
                debugString.AppendLine("Position Damper: " + drive.positionDamper);
                debugString.AppendLine("Max Force: " + drive.maximumForce);
                debugString.AppendLine("Mode: " + drive.mode);
                debugString.AppendLine("");

                debugString.AppendLine("Cross Section Properties");
                debugString.AppendLine("Radius: " + radius);
                debugString.AppendLine("Area: " + area);
                debugString.AppendLine("Moment of Inertia: " + momentOfInertia);

                Debug.Log(debugString.ToString());
            }
        }
    }
    
    public static class JointUtils
    {

        public static bool reinforceAttachNodes = false;
        public static bool reinforceDecouplersFurther = false;
        public static bool reinforceLaunchClampsFurther = false;
        public static bool useVolumeNotArea = false;

        public static float angularDriveSpring = 0;
        public static float angularDriveDamper = 0;
        public static float angularMaxForceFactor = 0;

        public static float breakForceMultiplier = 1;
        public static float breakTorqueMultiplier = 1;

        public static float breakStrengthPerArea = 40;
        public static float breakTorquePerMOI = 40000;

        public static float decouplerAndClampJointStrength = float.MaxValue;

        public static float stiffeningExtensionMassRatioThreshold = 5;

        public static bool debug = false;

        public static List<string> exemptPartTypes = new List<string>();
        public static List<string> exemptModuleTypes = new List<string>();
        public static List<string> decouplerStiffeningExtensionType = new List<string>();

        public static float massForAdjustment = 0.001f;

        public static bool JointAdjustmentValid(Part p)
        {
            foreach (string s in exemptPartTypes)
                if (p.GetType().ToString() == s)
                    return false;

            foreach (string s in exemptModuleTypes)
                if (p.Modules.Contains(s))
                    return false;

            return true;
        }

        public static List<Part> DecouplerPartStiffeningList(Part p, bool childrenNotParent, bool onlyAddLastPart)
        {
            List<Part> tmpPartList = new List<Part>();
            bool extend = false;
            // non-physical parts are skipped over by attachJoints, so do the same
            if (p.physicalSignificance == Part.PhysicalSignificance.NONE)
                extend = true;
            foreach (string s in decouplerStiffeningExtensionType)
                if (p.GetType().ToString() == s)
                {
                    extend = true;
                    break;
                }
            if (!extend)
                foreach (string s in decouplerStiffeningExtensionType)
                    if (p.Modules.Contains(s))
                    {
                        extend = true;
                        break;
                    }

            List<Part> newAdditions = new List<Part>();
            if (extend)
            {
                if (childrenNotParent)
                {
                    if (p.children != null)
                    {
                        foreach (Part q in p.children)
                            if (q != null && q.parent == p)
                            {
                                newAdditions.AddRange(DecouplerPartStiffeningList(q, childrenNotParent, onlyAddLastPart));
                            }
                    }
                }
                else
                {
                    if (p.parent)
                    {
                        newAdditions.AddRange(DecouplerPartStiffeningList(p.parent, childrenNotParent, onlyAddLastPart));
                    }
                }
            }
            else
            {
                float thisPartMaxMass = MaximumPossiblePartMass(p);
                if (childrenNotParent)
                {
                    if (p.children != null)
                        foreach (Part q in p.children)
                        {
                            if (q != null && q.parent == p)
                            {

                                float massRatio = MaximumPossiblePartMass(q) / thisPartMaxMass;

                                if (massRatio > stiffeningExtensionMassRatioThreshold)
                                {
                                    newAdditions.Add(q);
                                    if (debug)
                                        Debug.Log("Part " + q.partInfo.title + " added to list due to mass ratio difference");
                                }
                            }
                        }
                }
                else
                {
                    if (p.parent)
                    {
                        float massRatio = MaximumPossiblePartMass(p.parent) / thisPartMaxMass;

                        if (massRatio > stiffeningExtensionMassRatioThreshold)
                        {
                            newAdditions.Add(p.parent);
                            if (debug)
                                Debug.Log("Part " + p.parent.partInfo.title + " added to list due to mass ratio difference");
                        }
                    }
                }
            }
            if (newAdditions.Count > 0)
                tmpPartList.AddRange(newAdditions);
            else if (onlyAddLastPart)
                extend = false;

            if(!(onlyAddLastPart && extend))
                tmpPartList.Add(p);

            return tmpPartList;
        }

        public static void ConnectLaunchClampToGround(Part clamp)
        {
            float breakForce = Mathf.Infinity;
            float breakTorque = Mathf.Infinity;
            FixedJoint newJoint;

            newJoint = clamp.gameObject.AddComponent<FixedJoint>();

            newJoint.connectedBody = null;
            newJoint.anchor = Vector3.zero;
            newJoint.axis = Vector3.up;
            //newJoint.secondaryAxis = Vector3.forward;
            newJoint.breakForce = breakForce;
            newJoint.breakTorque = breakTorque;

            //newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
            //newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;
        }
    
    
        private static float MaximumPossiblePartMass(Part p)
        {
            float maxMass = p.mass;
            foreach (PartResource r in p.Resources)
            {
                maxMass += (float)(r.info.density * r.maxAmount);
            }
            if (debug)
                Debug.Log("Maximum mass for part " + p.partInfo.title + " is " + maxMass);
            return maxMass;
        }

        public static void AddDecouplerJointReinforcementModule(Part p)
        {
            p.AddModule("DecouplerJointReinforcementModule");
            (p.Modules["DecouplerJointReinforcementModule"] as DecouplerJointReinforcementModule).OnPartUnpack();
            if (debug)
                Debug.Log("Added DecouplerJointReinforcementModule to part " + p.partInfo.title);
        }

        public static void AddLaunchClampReinforcementModule(Part p)
        {
            p.AddModule("LaunchClampReinforcementModule");
            (p.Modules["LaunchClampReinforcementModule"] as LaunchClampReinforcementModule).OnPartUnpack();
            if (debug)
                Debug.Log("Added LaunchClampReinforcementModule to part " + p.partInfo.title);
        }

        public static void LoadConstants()
        {
            PluginConfiguration config = PluginConfiguration.CreateForType<KerbalJointReinforcement>();
            config.load();

            reinforceAttachNodes = config.GetValue<bool>("reinforceAttachNodes");
            reinforceDecouplersFurther = config.GetValue<bool>("reinforceDecouplersFurther");
            reinforceLaunchClampsFurther = config.GetValue<bool>("reinforceLaunchClampsFurther");
            useVolumeNotArea = config.GetValue<bool>("useVolumeNotArea");

            angularDriveSpring = config.GetValue<float>("angularDriveSpring");
            angularDriveDamper = config.GetValue<float>("angularDriveDamper");
            angularMaxForceFactor = config.GetValue<float>("angularMaxForceFactor");
            if (angularMaxForceFactor < 0)
                angularMaxForceFactor = float.MaxValue;

            breakForceMultiplier = config.GetValue<float>("breakForceMultiplier", 1);
            breakTorqueMultiplier = config.GetValue<float>("breakTorqueMultiplier", 1);

            breakStrengthPerArea = config.GetValue<float>("breakStrengthPerArea", 40);
            breakTorquePerMOI = config.GetValue<float>("breakTorquePerMOI", 40000);

            decouplerAndClampJointStrength = config.GetValue<float>("decouplerAndClampJointStrength", float.MaxValue);
            if (decouplerAndClampJointStrength < 0)
                decouplerAndClampJointStrength = float.MaxValue;

            stiffeningExtensionMassRatioThreshold = config.GetValue<float>("stiffeningExtensionMassRatioThreshold", 5);

            massForAdjustment = config.GetValue<float>("massForAdjustment", 1);

            exemptPartTypes.Clear();
            exemptModuleTypes.Clear();
            decouplerStiffeningExtensionType.Clear();

            int i = 0;
            do
            {
                string tmpPart, tmpModule, tmpDecoupler;
                tmpPart = config.GetValue("exemptPartType" + i, "");
                tmpModule = config.GetValue("exemptModuleType" + i, "");
                tmpDecoupler = config.GetValue("decouplerStiffeningExtensionType" + i, "");

                if (tmpPart == "" && tmpModule == "" && tmpDecoupler == "")
                    break;

                if (tmpPart != "")
                    exemptPartTypes.Add(tmpPart);
                if (tmpModule != "")
                    exemptModuleTypes.Add(tmpModule);
                if (tmpDecoupler != "")
                    decouplerStiffeningExtensionType.Add(tmpDecoupler);

                i++;
            } while (true);

            debug = config.GetValue<bool>("debug", false);

            if (debug)
            {
                StringBuilder debugString = new StringBuilder();
                debugString.AppendLine("\n\rAngular Drive: \n\rSpring: " + angularDriveSpring + "\n\rDamp: " + angularDriveDamper + "\n\rMax Force Factor: " + angularMaxForceFactor);

                debugString.AppendLine("\n\rJoint Strength Multipliers: \n\rForce Multiplier: " + breakForceMultiplier + "\n\rTorque Multiplier: " + breakTorqueMultiplier);
                debugString.AppendLine("Joint Force Strength Per Unit Area: " + breakStrengthPerArea);
                debugString.AppendLine("Joint Torque Strength Per Unit MOI: " + breakTorquePerMOI);

                debugString.AppendLine("Strength For Additional Decoupler And Clamp Joints: " + decouplerAndClampJointStrength);

                debugString.AppendLine("\n\rDebug Output: " + debug);
                debugString.AppendLine("Reinforce Attach Nodes: " + reinforceAttachNodes);
                debugString.AppendLine("Reinforce Decouplers Further: " + reinforceDecouplersFurther);
                debugString.AppendLine("Reinforce Launch Clamps Further: " + reinforceLaunchClampsFurther);
                debugString.AppendLine("Use Volume For Calculations, Not Area: " + useVolumeNotArea);

                debugString.AppendLine("\n\rMass For Joint Adjustment: " + massForAdjustment);

                debugString.AppendLine("\n\rExempt Part Types");
                foreach (string s in exemptPartTypes)
                    debugString.AppendLine(s);

                debugString.AppendLine("\n\rExempt Module Types");
                foreach (string s in exemptModuleTypes)
                    debugString.AppendLine(s);

                debugString.AppendLine("\n\rDecoupler Stiffening Extension Types");
                foreach (string s in decouplerStiffeningExtensionType)
                    debugString.AppendLine(s);

                debugString.AppendLine("\n\rDecoupler Stiffening Extension Mass Ratio Threshold: " + stiffeningExtensionMassRatioThreshold);

                Debug.Log(debugString.ToString());
            }
        }

        public static Vector3 GuessUpVector(Part part)
        {
            // For intakes, use the intake vector
            if (part.Modules.Contains("ModuleResourceIntake"))
            {
                ModuleResourceIntake i = part.Modules["ModuleResourceIntake"] as ModuleResourceIntake;
                Transform intakeTrans = part.FindModelTransform(i.intakeTransformName);
                return part.transform.InverseTransformDirection(intakeTrans.forward);
            }
            // If surface attachable, and node normal is up, check stack nodes or use forward
            else if (part.srfAttachNode != null &&
                     part.attachRules.srfAttach &&
                     Mathf.Abs(part.srfAttachNode.orientation.normalized.y) > 0.9f)
            {
                // When the node normal is exactly Vector3.up, the editor orients forward along the craft axis
                Vector3 dir = Vector3.forward;
                bool first = true;

                foreach (AttachNode node in part.attachNodes)
                {
                    // Doesn't seem to ever happen, but anyway
                    if (node.nodeType == AttachNode.NodeType.Surface)
                        continue;

                    // If all node orientations agree, use that axis
                    if (first)
                    {
                        first = false;
                        dir = node.orientation.normalized;
                    }
                    // Conflicting node directions - bail out
                    else if (Mathf.Abs(Vector3.Dot(dir, node.orientation.normalized)) < 0.9f)
                        return Vector3.up;
                }

                if (debug)
                    MonoBehaviour.print(part.partInfo.title + ": Choosing axis " + dir + " for KJR surface attach" + (first ? "" : " from node") + ".");

                return dir;
            }
            else
            {
                return Vector3.up;
            }
        }

        public static Vector3 CalculateExtents(Part p, Vector3 up)
        {
            up = up.normalized;

            // Align y axis of the result to the 'up' vector in local coordinate space
            if (Mathf.Abs(up.y) < 0.9f)
                return CalculateExtents(p, Quaternion.FromToRotation(Vector3.up, up));

            return CalculateExtents(p, Quaternion.identity);
        }

        public static Vector3 CalculateExtents(Part p, Vector3 up, Vector3 forward)
        {
            // Adjust forward to be orthogonal to up; LookRotation might do the opposite
            Vector3.OrthoNormalize(ref up, ref forward);

            // Align y to up and z to forward in local coordinate space
            return CalculateExtents(p, Quaternion.LookRotation(forward, up));
        }

        public static Vector3 CalculateExtents(Part p, Quaternion alignment)
        {
            Vector3 maxBounds = new Vector3(-100, -100, -100);
            Vector3 minBounds = new Vector3(100, 100, 100);

            // alignment transforms from our desired rotation to the local coordinates, so inverse needed
            Matrix4x4 rotation = Matrix4x4.TRS(Vector3.zero, Quaternion.Inverse(alignment), Vector3.one);
            Matrix4x4 base_matrix = rotation * p.transform.worldToLocalMatrix;

            foreach (Transform t in p.FindModelComponents<Transform>())         //Get the max boundaries of the part
            {
                MeshFilter mf = t.GetComponent<MeshFilter>();
                if (mf == null)
                    continue;
                Mesh m = mf.mesh;

                if (m == null)
                    continue;

                Matrix4x4 matrix = base_matrix * t.transform.localToWorldMatrix;

                foreach (Vector3 vertex in m.vertices)
                {
                    Vector3 v = matrix.MultiplyPoint3x4(vertex);

                    maxBounds.x = Mathf.Max(maxBounds.x, v.x);
                    minBounds.x = Mathf.Min(minBounds.x, v.x);
                    maxBounds.y = Mathf.Max(maxBounds.y, v.y);
                    minBounds.y = Mathf.Min(minBounds.y, v.y);
                    maxBounds.z = Mathf.Max(maxBounds.z, v.z);
                    minBounds.z = Mathf.Min(minBounds.z, v.z);
                }
            }

            if (maxBounds == new Vector3(-100, -100, -100) && minBounds == new Vector3(100, 100, 100))
            {
                Debug.LogWarning("KerbalJointReinforcement: extents could not be properly built for part " + p.partInfo.title);
                maxBounds = minBounds = Vector3.zero;
            }
            else if (debug)
                Debug.Log("Extents: "+minBounds+" .. "+maxBounds+" = "+(maxBounds-minBounds));

            //attachNodeLoc = p.transform.worldToLocalMatrix.MultiplyVector(p.parent.transform.position - p.transform.position);
            return maxBounds - minBounds;
        }

        public static float CalculateRadius(Part p, Vector3 attachNodeLoc)
        {
            // y along attachNodeLoc; x,z orthogonal
            Vector3 maxExtents = CalculateExtents(p, attachNodeLoc);

            // Equivalent radius of an ellipse painted into the rectangle
            float radius = Mathf.Sqrt(maxExtents.x * maxExtents.z) / 2;

            return radius;
        }

        public static float CalculateSideArea(Part p, Vector3 attachNodeLoc)
        {
            Vector3 maxExtents = CalculateExtents(p, attachNodeLoc);
            float area;

            //maxExtents = Vector3.Exclude(maxExtents, Vector3.up);

            area = maxExtents.x * maxExtents.z;

            return area;
        }
    }

    //This class adds an extra joint between a launch clamp and the part it is connected to for stiffness
    public class LaunchClampReinforcementModule : PartModule
    {
        private List<ConfigurableJoint> joints;
        private List<Part> neighbours = new List<Part>();
        //private bool decoupled = false;

        public override void OnAwake()
        {
            base.OnAwake();
            joints = new List<ConfigurableJoint>();
        }

        public void OnPartUnpack()
        {
            if (part.parent == null)
                return;

            neighbours.Add(part.parent);

            StringBuilder debugString = null;
            if (JointUtils.debug)
            {
                debugString = new StringBuilder();
                debugString.AppendLine("The following joints added by " + part.partInfo.title + " to increase stiffness:");
            }

            if (part.parent.Rigidbody != null)
                StrutConnectParts(part, part.parent);

            if (JointUtils.debug)
            {
                debugString.AppendLine(part.parent.partInfo.title + " connected to part " + part.partInfo.title);
                Debug.Log(debugString.ToString());
            }

            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Add(OnVesselWasModified);
        }

        private void OnVesselWasModified(Vessel v)
        {
            foreach (Part p in neighbours)
            {
                if (p.vessel == part.vessel)
                    continue;

                if (JointUtils.debug)
                    Debug.Log("Decoupling part " + part.partInfo.title + "; destroying all extra joints");

                BreakAllInvalidJoints();
                return;
            }
        }

        public void OnPartPack()
        {
            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();
            neighbours.Clear();
        }

        public void OnDestroy()
        {
            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
        }

        private void BreakAllInvalidJoints()
        {
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();

            var vessels = new List<Vessel>();

            foreach (Part n in neighbours)
            {
                if (n.vessel == null || vessels.Contains(n.vessel))
                    continue;

                vessels.Add(n.vessel);

                foreach (Part p in n.vessel.Parts)
                {
                    if (p is StrutConnector || p.Modules.Contains("LaunchClamp"))
                        continue;


                    ConfigurableJoint[] possibleConnections = p.GetComponents<ConfigurableJoint>();

                    if (possibleConnections == null)
                        continue;

                    foreach (ConfigurableJoint j in possibleConnections)
                    {
                        if (j.connectedBody == null)
                        {
                            GameObject.Destroy(j);
                            continue;
                        }
                        Part cp = j.connectedBody.GetComponent<Part>();
                        if (cp != null && cp.vessel != p.vessel)
                            GameObject.Destroy(j);
                    }
                }
            }
            neighbours.Clear();
        }


        private void StrutConnectParts(Part partWithJoint, Part partConnectedByJoint)
        {
            Rigidbody rigidBody = partConnectedByJoint.rigidbody;
            float breakForce = JointUtils.decouplerAndClampJointStrength;
            float breakTorque = JointUtils.decouplerAndClampJointStrength;
            Vector3 anchor, axis;
            anchor = Vector3.zero;
            axis = Vector3.right;
            ConfigurableJoint newJoint;

            newJoint = partWithJoint.gameObject.AddComponent<ConfigurableJoint>();

            newJoint.connectedBody = rigidBody;
            newJoint.anchor = anchor;
            newJoint.axis = axis;
            newJoint.secondaryAxis = Vector3.forward;
            newJoint.breakForce = breakForce;
            newJoint.breakTorque = breakTorque;

            newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
            newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;

            joints.Add(newJoint);
        }
    }

    //This class adds extra joints between the parts connected to a decoupler to stiffen up the connection
    public class DecouplerJointReinforcementModule : PartModule
    {
        protected List<ConfigurableJoint> joints = new List<ConfigurableJoint>();
        protected List<Part> neighbours = new List<Part>();
        private PartModule decoupler = null;
        //private bool radiallyAttached = false;

        public override void OnAwake()
        {
            base.OnAwake();

            foreach (PartModule m in part.Modules)
                if (m is ModuleDecouple)
                {
                    decoupler = m;
                    break;
                }
                else if (m is ModuleAnchoredDecoupler)
                {
                    decoupler = m;
                    break;
                }
        }

        public override void OnStart(PartModule.StartState state)
        {
            base.OnStart(state);

            if (part.parent == null)
                return;

            //if (part.attachMode == AttachModes.SRF_ATTACH)
            //    radiallyAttached = true;
        }


        public void OnPartUnpack()
        {
            if (part.parent == null || part.children.Count == 0)
                return;
            if (decoupler is ModuleDecouple && (decoupler as ModuleDecouple).isDecoupled)
                return;
            if (decoupler is ModuleAnchoredDecoupler && (decoupler as ModuleAnchoredDecoupler).isDecoupled)
                return;

            AddExtraJoints();
        }

        private void OnVesselWasModified(Vessel v)
        {
            foreach (Part p in neighbours)
            {
                if (p.vessel == part.vessel)
                    continue;

                if (JointUtils.debug)
                    Debug.Log("Decoupling part " + part.partInfo.title + "; destroying all extra joints");

                BreakAllInvalidJoints();
                break;
            }
        }

        private void AddExtraJoints()
        {
            List<Part> childParts = new List<Part>();
            List<Part> parentParts = new List<Part>();

            parentParts = JointUtils.DecouplerPartStiffeningList(part.parent, false, true);
            foreach (Part p in part.children)
                childParts.AddRange(JointUtils.DecouplerPartStiffeningList(p, true, true));

            neighbours.Clear();
            neighbours.AddRange(parentParts);
            neighbours.AddRange(childParts);
            neighbours.Remove(part);

            parentParts.Add(part);

            StringBuilder debugString = null;

            if (JointUtils.debug)
            {
                debugString = new StringBuilder();
                debugString.AppendLine(parentParts.Count + " parts above decoupler to be connected to " + childParts.Count + " below decoupler.");
                debugString.AppendLine("The following joints added by " + part.partInfo.title + " to increase stiffness:");
            }

            foreach (Part p in parentParts)
            {
                if (p == null || p.rigidbody == null || p.Modules.Contains("ProceduralFairingDecoupler") || !JointUtils.JointAdjustmentValid(p))
                    continue;
                foreach (Part q in childParts)
                {
                    if (q == null || q.rigidbody == null || q.Modules.Contains("ProceduralFairingDecoupler") || p == q || !JointUtils.JointAdjustmentValid(q))
                        continue;

                    StrutConnectParts(p, q);

                    if (JointUtils.debug)
                        debugString.AppendLine(p.partInfo.title + " connected to part " + q.partInfo.title);
                }
            }

            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Add(OnVesselWasModified);

            if (JointUtils.debug)
                Debug.Log(debugString.ToString());
        }

        public void OnPartPack()
        {
            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();
            neighbours.Clear();
        }

        public void OnDestroy()
        {
            if (joints.Count > 0)
                GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
        }

        private void BreakAllInvalidJoints()
        {
            GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

            foreach (ConfigurableJoint j in joints)
                GameObject.Destroy(j);

            joints.Clear();

            var vessels = new List<Vessel>();

            foreach (Part n in neighbours)
            {
                if (n.vessel == null || vessels.Contains(n.vessel))
                    continue;

                vessels.Add(n.vessel);

                foreach (Part p in n.vessel.Parts)
                {
                    if (p is StrutConnector || p.Modules.Contains("LaunchClamp"))
                        continue;

                    ConfigurableJoint[] possibleConnections = p.GetComponents<ConfigurableJoint>();

                    if (possibleConnections == null)
                        continue;

                    foreach (ConfigurableJoint j in possibleConnections)
                    {
                        if (j.connectedBody == null)
                        {
                            GameObject.Destroy(j);
                            continue;
                        }
                        Part cp = j.connectedBody.GetComponent<Part>();
                        if (cp != null && cp.vessel != p.vessel)
                            GameObject.Destroy(j);
                    }
                }
            }
            neighbours.Clear();

            if (part.parent == null || part.children.Count == 0)
                return;
            if (decoupler is ModuleDecouple && (decoupler as ModuleDecouple).isDecoupled)
                return;
            if (decoupler is ModuleAnchoredDecoupler && (decoupler as ModuleAnchoredDecoupler).isDecoupled)
                return;

            AddExtraJoints();
        }

        private void StrutConnectParts(Part partWithJoint, Part partConnectedByJoint)
        {
            Rigidbody rigidBody = partConnectedByJoint.rigidbody;
            float breakForce = JointUtils.decouplerAndClampJointStrength;
            float breakTorque = JointUtils.decouplerAndClampJointStrength;
            Vector3 anchor, axis;

            anchor = Vector3.zero;
            axis = Vector3.right;

            ConfigurableJoint newJoint;

            newJoint = partWithJoint.gameObject.AddComponent<ConfigurableJoint>();

            newJoint.connectedBody = rigidBody;
            newJoint.anchor = anchor;
            newJoint.axis = axis;
            newJoint.secondaryAxis = Vector3.forward;
            newJoint.breakForce = breakForce;
            newJoint.breakTorque = breakTorque;

            newJoint.xMotion = newJoint.yMotion = newJoint.zMotion = ConfigurableJointMotion.Locked;
            newJoint.angularXMotion = newJoint.angularYMotion = newJoint.angularZMotion = ConfigurableJointMotion.Locked;


            joints.Add(newJoint);

        }
    }
}
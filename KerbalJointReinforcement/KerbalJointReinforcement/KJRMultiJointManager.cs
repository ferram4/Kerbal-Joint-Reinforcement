/*
Kerbal Joint Reinforcement, v3.3.3
Copyright 2015, Michael Ferrara, aka Ferram4

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
using System.Linq;
using UnityEngine;

namespace KerbalJointReinforcement
{
    //All this class exists to do is to act as a box attached to 
    //For a sequence of three parts (A, B, C), connected in series, this will exist on B and hold the strengthening joint from A to C
    //If the joint from A to B or B to C is broken, this will destroy the joint A to C and then destroy itself
    class KJRMultiJointManager
    {
        public static KJRMultiJointManager fetch;

        Dictionary<Part, List<ConfigurableJoint>> multiJointDict;
        List<Part> linkPart1List;
        List<Part> linkPart2List;
        HashSet<Part> linkedSet;

        public KJRMultiJointManager()
        {
            multiJointDict = new Dictionary<Part, List<ConfigurableJoint>>();
            linkPart1List = new List<Part>();
            linkPart2List = new List<Part>();
            linkedSet = new HashSet<Part>();
            fetch = this;
            GameEvents.onVesselCreate.Add(VesselCreate);
            GameEvents.onPartUndock.Add(OnJointBreak);
            GameEvents.onPartDie.Add(OnJointBreak);
        }

        public void OnDestroy()
        {
            Debug.Log("KJRMultiJointManager cleanup");
            GameEvents.onVesselCreate.Remove(VesselCreate);
            GameEvents.onPartUndock.Remove(OnJointBreak);
            GameEvents.onPartDie.Remove(OnJointBreak);
        }
        
        //This entire scheme relies on a simple fact: when a vessel is created, the part that was decoupled is the root part
        //Therefore, we only need to use vessel.RootPart and the parts with no children to ensure that all multijoints are broken
        private void VesselCreate(Vessel v)
        {
            //Debug.Log(v.name + " joint break");
            OnJointBreak(v.rootPart);

            for(int i = 0; i < v.Parts.Count; ++i)
            {
                Part p = v.Parts[i];
                OnJointBreak(p);
            }
        }

        public bool TrySetValidLinkedSet(Part linkPart1, Part linkPart2)
        {
            linkPart1List.Clear();
            linkPart2List.Clear();
            linkedSet.Clear();

            if (!KJRJointUtils.JointAdjustmentValid(linkPart1) || !KJRJointUtils.JointAdjustmentValid(linkPart2))
                return false;

            //Add the parts we're connecting to their respective lists
            linkPart1List.Add(linkPart1);
            linkPart2List.Add(linkPart2);

            //iterate through each part's parents, making them the new linkPart and then adding them to their respective lists.  LinkPart.parent will not be null until it reaches the root part
            while ((object)linkPart1.parent != null)
            {
                linkPart1 = linkPart1.parent;
                linkPart1List.Add(linkPart1);
            }
            while ((object)linkPart2.parent != null)
            {
                linkPart2 = linkPart2.parent;
                linkPart2List.Add(linkPart2);
            }

            int index1, index2;
            index1 = linkPart1List.Count - 1;
            index2 = linkPart2List.Count - 1;
            bool reachedMergeNode = false;

            //Start iterating through the lists, starting from the last indices.  Due to the way the lists were created, index 0 = connected part, while index = Count - 1 = root part
            while (true)
            {
                Part p = null, q = null;

                //if the indices are valid, get those particular parts
                if (index1 >= 0)
                    p = linkPart1List[index1];
                if (index2 >= 0)
                    q = linkPart2List[index2];

                //decrement indices for next iteration
                index1--;
                index2--;

                //if p and q are the same, this is the merge we care about; set variable noting this now
                if (p == q)
                    reachedMergeNode = true;

                //if we haven't reached the merge yet, go back to the beginning of the loop
                if (!reachedMergeNode)
                    continue;

                //if we have reached the merge, all the remaining parts are parts of the line connecting these parts and should be set to be disconnected
                if ((object)p != null)
                {
                    if (KJRJointUtils.JointAdjustmentValid(p))
                        linkedSet.Add(p);
                    else
                        return false;
                }
                if ((object)q != null)
                {
                    if (KJRJointUtils.JointAdjustmentValid(q))
                        linkedSet.Add(q);
                    else
                        return false;
                }

                //if both p and q are null, that means we've reached the end of both lines and so all the parts have been added
                if ((object)p == null && (object)q == null)
                    break;
            }

            return true;
        }

        public void RegisterMultiJointBetweenParts(Part linkPart1, Part linkPart2, ConfigurableJoint multiJoint)
        {
            foreach (Part p in linkedSet)
                RegisterMultiJoint(p, multiJoint);
        }

        public void RegisterMultiJoint(Part testPart, ConfigurableJoint multiJoint)
        {
            List<ConfigurableJoint> configJointList;
            if (multiJointDict.TryGetValue(testPart, out configJointList))
            {
                for (int i = configJointList.Count - 1; i >= 0; --i)
                    if (configJointList[i] == null)
                        configJointList.RemoveAt(i);

                configJointList.Add(multiJoint);
            }
            else
            {
                configJointList = new List<ConfigurableJoint>();
                configJointList.Add(multiJoint);
                multiJointDict.Add(testPart, configJointList);
            }
        }

        public bool CheckMultiJointBetweenParts(Part testPart1, Part testPart2)
        {
            if (testPart1 == null || testPart2 == null || testPart1 == testPart2)
                return false;

            List<ConfigurableJoint> testMultiJoints;

            if (!multiJointDict.TryGetValue(testPart1, out testMultiJoints))
                return false;

            Rigidbody testRb = testPart2.rb;

            for (int i = 0; i < testMultiJoints.Count; i++)
            {
                ConfigurableJoint joint = testMultiJoints[i];
                if (joint == null)
                {
                    testMultiJoints.RemoveAt(i);
                    --i;
                    continue;
                }
                if (joint.connectedBody == testRb)
                    return true;
            }

            return false;
        }

        public void OnJointBreak(PartJoint partJoint)
        {
            OnJointBreak(partJoint.Parent);
        }

        public void OnJointBreak(Part part)
        {
            if (part == null)
                return;
            List<ConfigurableJoint> configJointList;
            if (multiJointDict.TryGetValue(part, out configJointList))
            {
                for(int i = 0; i < configJointList.Count; i++)
                {
                    ConfigurableJoint joint = configJointList[i];
                    if (joint != null)
                    {
                        GameObject.Destroy(joint);
                    }
                }

                multiJointDict.Remove(part);
            }
        }
    }
}
